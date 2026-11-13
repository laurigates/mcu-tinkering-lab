/**
 * @file twin_main.c
 * @brief robocar-twin: the robocar's planner and executor on a virtual clock
 *        (ADR-030, ADR-036, PRD-017).
 *
 * Every decision below is a call into firmware code compiled for the host:
 * planner_cycle_core.c for the planner cycle, plan_activity / plan_budget /
 * scene_change / ambient_audio / speech_budget / dialogue_style for the gates,
 * gemini_parse.c for the model's reply, goal_state.c for the goal, and
 * reactive_controller.c with motor_controller.c, ultrasonic_core.c, head_aim.c,
 * idle_gaze.c and cliff_core.c for the executor (ADR-036 §1). This file owns
 * only what planner_task.c, gemini_backend.c, ambient_listener.c and
 * voice_turn.c own on the robot: sequencing, I/O and time. Where it mirrors one
 * of those, the comment names the firmware lines it follows. What sits under
 * the executor (the rangefinder bus, the PCA9685, the floor sensors) is in
 * twin_exec.c.
 *
 * Protocol (one JSON object per line):
 *   stdin:  config, world events, {"t":"run"}, then one model_response per
 *           model_request.
 *   stdout: trace records (cycle, goal, force_stop, utterance, suppressed,
 *           announce, backoff, note_offered, note, note_not_offered, wheels,
 *           head), model_request, and a final done.
 *
 * Firmware modules log with printf on the host (goal_state.c's host branch), so
 * stdout is moved to stderr at start and the trace goes to a private dup of the
 * original stdout: nothing a module prints can corrupt the protocol.
 */

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ambient_audio.h"
#include "cJSON.h"
#include "dialogue_style.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "gemini_parse.h"
#include "goal_state.h"
#include "note_gate.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "planner_cycle_core.h"
#include "reactive_controller.h"
#include "scene_change.h"
#include "servo_controller.h"
#include "speech_budget.h"
#include "speech_queue.h" /* SPEECH_TEXT_MAX */
#include "twin_exec.h"
#include "twin_sha256.h"
#include "twin_signals.h"

#ifndef ESP_ERR_NOT_ALLOWED
#define ESP_ERR_NOT_ALLOWED 0x10C /* ESP-IDF's value; the host esp_err.h shim lacks it */
#endif

/* ========================================================================== */
/* Virtual clock                                                              */
/* ========================================================================== */

/** Milliseconds since boot, 64-bit like esp_timer's microsecond count, so it
 *  never wraps in a run. Firmware modules take their own uint32 casts of it, as
 *  on the robot, which is what puts the day-49 millisecond wrap in reach. */
static uint64_t s_now_ms;

/** The esp_timer shim every firmware module reads: the virtual clock. */
int64_t esp_timer_get_time(void)
{
    return (int64_t)s_now_ms * 1000;
}

/* ========================================================================== */
/* Output                                                                     */
/* ========================================================================== */

static FILE *s_out;

static void die(int code, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fputs("robocar-twin: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(code);
}

/**
 * Append @p s as a JSON string, escaped the way Python's
 * json.dumps(ensure_ascii=False) escapes it: the canonical descriptor is hashed
 * on both sides, so the two serialisations must agree byte for byte.
 */
static size_t json_escape(char *out, size_t cap, size_t pos, const char *s)
{
#define PUT(c)                    \
    do {                          \
        if (pos + 1 < cap) {      \
            out[pos] = (char)(c); \
        }                         \
        pos++;                    \
    } while (0)
    PUT('"');
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        const unsigned char c = *p;
        switch (c) {
            case '"':
                PUT('\\');
                PUT('"');
                break;
            case '\\':
                PUT('\\');
                PUT('\\');
                break;
            case '\n':
                PUT('\\');
                PUT('n');
                break;
            case '\r':
                PUT('\\');
                PUT('r');
                break;
            case '\t':
                PUT('\\');
                PUT('t');
                break;
            case '\b':
                PUT('\\');
                PUT('b');
                break;
            case '\f':
                PUT('\\');
                PUT('f');
                break;
            default:
                if (c < 0x20) {
                    char u[8];
                    snprintf(u, sizeof(u), "\\u%04x", c);
                    for (const char *q = u; *q; q++) {
                        PUT(*q);
                    }
                } else {
                    PUT(c);
                }
        }
    }
    PUT('"');
#undef PUT
    if (cap > 0) {
        out[pos < cap ? pos : cap - 1] = '\0';
    }
    return pos;
}

/** A JSON string literal for @p s in a static buffer, valid until the next call. */
static const char *jstr(const char *s)
{
    static char buf[4 * SPEECH_TEXT_MAX + 64];
    const size_t n = json_escape(buf, sizeof(buf), 0, s ? s : "");
    if (n >= sizeof(buf)) {
        die(4, "string too long to emit (%zu bytes)", n);
    }
    return buf;
}

static void emit(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(s_out, fmt, ap);
    va_end(ap);
    fputc('\n', s_out);
    fflush(s_out);
}

/* ========================================================================== */
/* World events                                                               */
/* ========================================================================== */

typedef enum {
    EV_FRAME,
    EV_RANGE,
    EV_RANGE_FAIL,
    EV_SOUND,
    EV_VOICE_TURN,
    EV_CONSOLE,
    EV_LINK,
    EV_FLOOR,
} ev_kind_t;

typedef struct {
    ev_kind_t kind;
    uint64_t at_ms;
    uint32_t seq; /* file order, for a stable sort */
    char scene[96];
    char pose;
    int cm;
    int peak_db;
    uint64_t dur_ms;
    uint64_t record_ms; /* voice_turn: how long record_clip() holds the mic */
    char reply[SPEECH_TEXT_MAX];
    bool wifi;
    bool key;
    ultrasonic_id_t sensor; /* range, range_fail: which rangefinder */
    bool floor_front;       /* floor: floor present under each sensor */
    bool floor_rear;
} world_event_t;

static world_event_t *s_events;
static size_t s_event_count;
static size_t s_event_next;

typedef struct {
    uint32_t seed;
    uint64_t duration_ms;
    uint32_t period_ms;
    long max_requests; /* -1: firmware default */
    long max_tokens;
    bool head;  /* servos present: the executor aims the head (ADR-036 §1) */
    bool cliff; /* cliff reflex armed */
} twin_config_t;

static twin_config_t s_cfg = {
    .seed = 1,
    .duration_ms = 0,
    .period_ms = 15000,
    .max_requests = -1,
    .max_tokens = -1,
    .head = false,
    .cliff = false,
};

/* ---- World state, as the robot's sensors and links would report it ---- */

static bool s_have_frame;
static scene_fingerprint_t s_frame_fp; /* .pose: the scripted frame.pose */
static char s_frame_scene[96];
/** The dwell pose of the frame the planner last captured, as the letter the
 *  request descriptor carries: the scripted frame.pose, or with `head` set the
 *  pose sampled from the servos at the planner tick. */
static char s_capture_pose = 'C';

static bool s_wifi;
static bool s_key;

/* Range, range validity, robot_moving and reflex trips are not kept here: the
 * planner reads them from reactive_controller_get_telemetry(), as on the robot. */

#define MAX_BURSTS 64
typedef struct {
    uint64_t start_ms;
    uint64_t end_ms;
    int peak_db;
    uint32_t frames;
} burst_t;
static burst_t s_bursts[MAX_BURSTS];
static size_t s_burst_count;

/* ---- Voice turn (scripted in phase 1; voice_turn_core is phase 2) ---- */

static bool s_voice_busy;
static uint32_t s_voice_turns; /* voice_turn.c s_turns_started */
static uint64_t s_voice_end_ms;
static uint64_t s_voice_rec_start_ms; /* mic_pdm_lock() .. mic_pdm_unlock() */
static uint64_t s_voice_rec_end_ms;
static char s_voice_reply[SPEECH_TEXT_MAX];
/** The depth-1 queue's one slot: a request accepted while a turn runs. */
static bool s_voice_pending;
static world_event_t s_voice_next;

/* ---- Ambient listener ---- */

/** ambient_listener.c reads AMBIENT_FRAME_SAMPLES per mic_pdm_read() at
 *  AMBIENT_SAMPLE_RATE_HZ: 1024 / 16000 Hz = 64 ms per frame, back to back
 *  (no delay in the loop; the DMA sets the pace). */
#define AUDIO_FRAME_MS ((AMBIENT_FRAME_SAMPLES * 1000u) / AMBIENT_SAMPLE_RATE_HZ)

static uint64_t s_next_audio_ms = AUDIO_FRAME_MS; /* end of the first frame */
static ambient_fingerprint_t s_room_fp;
static uint32_t s_last_playback_end_ms; /* ambient_listener.c s_last_playback_end_ms */
static uint32_t s_last_cue_ms;          /* ambient_listener.c s_last_cue_ms */

/* ========================================================================== */
/* Input                                                                      */
/* ========================================================================== */

static char *read_line(void)
{
    static char *line;
    static size_t cap;
    for (;;) {
        const ssize_t n = getline(&line, &cap, stdin);
        if (n < 0) {
            return NULL;
        }
        size_t len = (size_t)n;
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len > 0) {
            return line;
        }
    }
}

static const cJSON *field(const cJSON *obj, const char *name, bool required, const char *t)
{
    const cJSON *f = cJSON_GetObjectItemCaseSensitive(obj, name);
    if (!f && required) {
        die(2, "'%s' record is missing '%s'", t, name);
    }
    return f;
}

static long int_field(const cJSON *obj, const char *name, bool required, long dflt, const char *t)
{
    const cJSON *f = field(obj, name, required, t);
    if (!f) {
        return dflt;
    }
    if (!cJSON_IsNumber(f)) {
        die(2, "'%s'.%s must be a number", t, name);
    }
    return (long)f->valuedouble;
}

static bool bool_field(const cJSON *obj, const char *name, const char *t)
{
    const cJSON *f = field(obj, name, true, t);
    if (!cJSON_IsBool(f)) {
        die(2, "'%s'.%s must be true or false", t, name);
    }
    return cJSON_IsTrue(f);
}

/** An optional true/false; @p dflt when absent. */
static bool opt_bool_field(const cJSON *obj, const char *name, bool dflt, const char *t)
{
    return field(obj, name, false, t) ? bool_field(obj, name, t) : dflt;
}

static void str_field(const cJSON *obj, const char *name, char *out, size_t cap, const char *t)
{
    const cJSON *f = field(obj, name, true, t);
    if (!cJSON_IsString(f)) {
        die(2, "'%s'.%s must be a string", t, name);
    }
    if (strlen(f->valuestring) >= cap) {
        die(2, "'%s'.%s is longer than %zu bytes", t, name, cap - 1);
    }
    strcpy(out, f->valuestring);
}

/** The optional `sensor` of a range or range_fail event: front when absent. */
static ultrasonic_id_t sensor_field(const cJSON *obj, const char *t)
{
    if (!field(obj, "sensor", false, t)) {
        return ULTRASONIC_FRONT;
    }
    char name[8];
    str_field(obj, "sensor", name, sizeof(name), t);
    if (strcmp(name, "front") == 0) {
        return ULTRASONIC_FRONT;
    }
    if (strcmp(name, "rear") == 0) {
        return ULTRASONIC_REAR;
    }
    die(2, "'%s'.sensor must be front or rear (got '%s')", t, name);
    return ULTRASONIC_FRONT; /* not reached */
}

static void add_event(const world_event_t *ev)
{
    static size_t cap;
    if (s_event_count == cap) {
        cap = cap ? cap * 2 : 64;
        s_events = realloc(s_events, cap * sizeof(*s_events));
        if (!s_events) {
            die(5, "out of memory");
        }
    }
    s_events[s_event_count++] = *ev;
}

static int cmp_event(const void *a, const void *b)
{
    const world_event_t *x = a, *y = b;
    if (x->at_ms != y->at_ms) {
        return x->at_ms < y->at_ms ? -1 : 1;
    }
    return x->seq < y->seq ? -1 : (x->seq > y->seq);
}

/** Read config and world events up to {"t":"run"}. */
static void read_setup(void)
{
    bool have_config = false;
    char *line;
    while ((line = read_line()) != NULL) {
        cJSON *obj = cJSON_Parse(line);
        if (!obj || !cJSON_IsObject(obj)) {
            die(2, "stdin line is not a JSON object: %.200s", line);
        }
        char t[32];
        str_field(obj, "t", t, sizeof(t), "?");
        if (strcmp(t, "run") == 0) {
            cJSON_Delete(obj);
            if (!have_config) {
                die(2, "'run' before 'config'");
            }
            qsort(s_events, s_event_count, sizeof(*s_events), cmp_event);
            return;
        }
        if (strcmp(t, "config") == 0) {
            s_cfg.seed = (uint32_t)int_field(obj, "seed", true, 0, t);
            const long duration = int_field(obj, "duration_ms", true, 0, t);
            if (duration < 0) {
                die(2, "duration_ms must be >= 0");
            }
            s_cfg.duration_ms = (uint64_t)duration;
            s_cfg.period_ms = (uint32_t)int_field(obj, "planner_period_ms", false, 15000, t);
            s_cfg.max_requests = int_field(obj, "plan_budget_max_requests", false, -1, t);
            s_cfg.max_tokens = int_field(obj, "plan_budget_max_tokens", false, -1, t);
            if (s_cfg.period_ms == 0) {
                die(2, "planner_period_ms must be > 0");
            }
            s_cfg.head = opt_bool_field(obj, "head", false, t);
            s_cfg.cliff = opt_bool_field(obj, "cliff", false, t);
            have_config = true;
            cJSON_Delete(obj);
            continue;
        }

        world_event_t ev = {0};
        ev.seq = (uint32_t)s_event_count;
        const long at_ms = int_field(obj, "at_ms", true, 0, t);
        if (at_ms < 0) {
            die(2, "'%s'.at_ms must be >= 0", t);
        }
        ev.at_ms = (uint64_t)at_ms;
        if (strcmp(t, "frame") == 0) {
            ev.kind = EV_FRAME;
            str_field(obj, "scene", ev.scene, sizeof(ev.scene), t);
            char pose[4] = "C";
            if (field(obj, "pose", false, t)) {
                str_field(obj, "pose", pose, sizeof(pose), t);
            }
            if (strlen(pose) != 1 || !strchr("CLRF", pose[0])) {
                die(2, "frame.pose must be one of C, L, R, F (got '%s')", pose);
            }
            ev.pose = pose[0];
        } else if (strcmp(t, "range") == 0) {
            ev.kind = EV_RANGE;
            ev.cm = (int)int_field(obj, "cm", true, 0, t);
            /* The bounds ultrasonic_core_decode() accepts (ultrasonic_core.h). */
            if (ev.cm < (int)ULTRASONIC_MIN_RANGE_CM || ev.cm > (int)ULTRASONIC_MAX_RANGE_CM) {
                die(2, "range.cm must be in %u..%u (got %d)", ULTRASONIC_MIN_RANGE_CM,
                    ULTRASONIC_MAX_RANGE_CM, ev.cm);
            }
            ev.sensor = sensor_field(obj, t);
        } else if (strcmp(t, "range_fail") == 0) {
            ev.kind = EV_RANGE_FAIL;
            ev.sensor = sensor_field(obj, t);
        } else if (strcmp(t, "floor") == 0) {
            ev.kind = EV_FLOOR;
            ev.floor_front = bool_field(obj, "front", t);
            ev.floor_rear = bool_field(obj, "rear", t);
        } else if (strcmp(t, "sound") == 0) {
            ev.kind = EV_SOUND;
            ev.peak_db = (int)int_field(obj, "peak_db", true, 0, t);
            const long dur = int_field(obj, "dur_ms", true, 0, t);
            ev.dur_ms = (uint64_t)(dur < 0 ? 0 : dur);
        } else if (strcmp(t, "voice_turn") == 0) {
            ev.kind = EV_VOICE_TURN;
            const long dur = int_field(obj, "dur_ms", true, 0, t);
            ev.dur_ms = (uint64_t)(dur < 0 ? 0 : dur);
            const long rec = int_field(obj, "record_ms", false, dur, t);
            if (rec < 0 || rec > dur) {
                die(2, "voice_turn.record_ms must be in 0..dur_ms");
            }
            ev.record_ms = (uint64_t)rec;
            str_field(obj, "reply", ev.reply, sizeof(ev.reply), t);
        } else if (strcmp(t, "console") == 0) {
            ev.kind = EV_CONSOLE;
        } else if (strcmp(t, "link") == 0) {
            ev.kind = EV_LINK;
            ev.wifi = bool_field(obj, "wifi", t);
            ev.key = bool_field(obj, "key", t);
        } else if (strcmp(t, "reflex") == 0) {
            die(2, "'reflex' events were removed (ADR-036): the reflex is the executor's "
                   "reaction to a range reading, so script the obstacle with 'range'");
        } else {
            die(2, "unknown record type '%s'", t);
        }
        add_event(&ev);
        cJSON_Delete(obj);
    }
    die(2, "stdin closed before 'run'");
}

/* ========================================================================== */
/* World simulation                                                           */
/* ========================================================================== */

static uint8_t pose_from_letter(char c)
{
    switch (c) {
        case 'L':
            return SCENE_POSE_LEFT;
        case 'R':
            return SCENE_POSE_RIGHT;
        case 'F':
            return SCENE_POSE_FLOOR;
        default:
            return SCENE_POSE_CENTRE;
    }
}

static void voice_turn_start_now(const world_event_t *ev);

/**
 * End of a scripted voice turn. Mirrors what voice_turn.c does to the state the
 * planner reads, in its order:
 *   - speech_queue_post(s_reply)            voice_turn.c:516 (here: the utterance record)
 *   - dialogue_style_note_spoken(s_reply)   voice_turn.c:542
 *   - speech_budget_note(now)               voice_turn.c:543 (moves speech_budget_notes())
 *   - ambient_audio_mark_spoken()           voice_turn.c:544
 *   - s_busy = false                        voice_turn.c:560, after run_turn() returns
 * scene_change_mark_spoken() is deliberately not called (voice_turn.c:535).
 */
static void voice_turn_end(void)
{
    emit("{\"t\":\"utterance\",\"at_ms\":%llu,\"source\":\"voice_turn\",\"text\":%s}",
         (unsigned long long)s_now_ms, jstr(s_voice_reply));
    dialogue_style_note_spoken(s_voice_reply);
    speech_budget_note((uint32_t)(esp_timer_get_time() / 1000));
    ambient_audio_mark_spoken();
    s_voice_busy = false;
    /* The reply is played back; the listener quarantines playback plus
     * AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT (ambient_listener.c:86-95). Phase 1
     * folds playback into the turn's duration, so playback ends here. */
    s_last_playback_end_ms = (uint32_t)s_now_ms;

    /* voice_turn_task() loops straight back to xQueueReceive(): a request
     * accepted during this turn starts now (voice_turn.c:555-560). */
    if (s_voice_pending) {
        s_voice_pending = false;
        voice_turn_start_now(&s_voice_next);
    }
}

/**
 * voice_turn_task() takes a request off the queue, increments s_turns_started
 * and sets s_busy before run_turn() (voice_turn.c:555-558). record_clip() holds
 * the microphone only for the recording (voice_turn.c:262-277); the request and
 * the reply follow with the mic free. The scripted turn's durations count from
 * this moment.
 */
static void voice_turn_start_now(const world_event_t *ev)
{
    s_voice_turns = s_voice_turns + 1u;
    s_voice_busy = true;
    s_voice_rec_start_ms = s_now_ms;
    s_voice_rec_end_ms = s_now_ms + ev->record_ms;
    s_voice_end_ms = s_now_ms + ev->dur_ms;
    snprintf(s_voice_reply, sizeof(s_voice_reply), "%s", ev->reply);
}

/**
 * A voice turn request. The task has already dequeued the running turn
 * (voice_turn.c:555), so the depth-1 queue has room for one more:
 * xQueueSend(s_queue, req, 0) at voice_turn.c:620 accepts it and the task runs
 * it after the current one. Only a request while that slot is also full is
 * refused. enqueue()'s other refusals (playback active, no WiFi, voice budget;
 * voice_turn.c:604-619) are not modelled in phase 1.
 */
static void voice_turn_begin(const world_event_t *ev)
{
    if (!s_voice_busy) {
        voice_turn_start_now(ev);
        return;
    }
    if (s_voice_pending) {
        fprintf(stderr,
                "robocar-twin: voice_turn at %llu ms refused: a turn is running and one is "
                "queued\n",
                (unsigned long long)ev->at_ms);
        return;
    }
    s_voice_pending = true;
    s_voice_next = *ev;
}

static void apply_event(const world_event_t *ev)
{
    switch (ev->kind) {
        case EV_FRAME: {
            uint8_t luma[TWIN_THUMB_W * TWIN_THUMB_H];
            twin_scene_luma(ev->scene, luma);
            scene_fingerprint_from_luma(luma, TWIN_THUMB_W, TWIN_THUMB_H, &s_frame_fp);
            /* With `head` set the pose comes from the servos at the planner
             * tick (planner_tick() step 1b) and this one is not used. */
            s_frame_fp.pose = pose_from_letter(ev->pose);
            snprintf(s_frame_scene, sizeof(s_frame_scene), "%s", ev->scene);
            s_have_frame = true;
            break;
        }
        case EV_RANGE:
            twin_exec_set_range(ev->sensor, ev->cm);
            break;
        case EV_RANGE_FAIL:
            twin_exec_set_range(ev->sensor, TWIN_RANGE_FAILED);
            break;
        case EV_FLOOR:
            twin_exec_set_floor(true, ev->floor_front, ev->floor_rear);
            break;
        case EV_SOUND:
            if (s_burst_count == MAX_BURSTS) {
                die(2, "more than %d sound events", MAX_BURSTS);
            }
            s_bursts[s_burst_count++] = (burst_t){
                .start_ms = ev->at_ms,
                .end_ms = ev->at_ms + ev->dur_ms,
                .peak_db = ev->peak_db,
                .frames = 0,
            };
            break;
        case EV_VOICE_TURN:
            voice_turn_begin(ev);
            break;
        case EV_CONSOLE:
            /* main.c:1882 — any completed console line wakes the planner. */
            plan_activity_wake();
            break;
        case EV_LINK:
            s_wifi = ev->wifi;
            s_key = ev->key;
            break;
    }
}

/**
 * One microphone frame, as ambient_listener_task() handles it
 * (ambient_listener.c:103-164), called at the frame's END: the listener stamps
 * it with now_ms() after mic_pdm_read() returns (ambient_listener.c:117), so the
 * frame holds the 64 ms before its stamp. A sound that starts exactly at a tick
 * is therefore first noted one frame later, never by that same tick.
 *
 * Skipped if any of it falls inside a voice turn's recording, while
 * record_clip() holds the mic lock (voice_turn.c:262-277); outside the
 * recording the listener keeps noting frames through the turn's request.
 * Then screened by ambient_capture_allowed() and ambient_gate_accepts(),
 * fingerprinted and noted. Playback and the start cue are not modelled in
 * phase 1 beyond the voice turn's own reply (see voice_turn_end()).
 */
static void audio_frame(void)
{
    const uint64_t frame_start = s_now_ms - AUDIO_FRAME_MS;
    if (s_voice_busy && frame_start < s_voice_rec_end_ms && s_now_ms > s_voice_rec_start_ms) {
        return; /* mic_pdm_lock() held by record_clip(): the listener misses it */
    }
    const uint32_t t = (uint32_t)s_now_ms;
    const bool allowed = ambient_capture_allowed(false, t, s_last_playback_end_ms,
                                                 AMBIENT_PLAYBACK_HANGOVER_MS_DEFAULT);
    const bool to_gate =
        ambient_gate_accepts(allowed, false, t, &s_last_cue_ms, AMBIENT_CUE_HANGOVER_MS_DEFAULT);
    if (!to_gate) {
        return;
    }

    burst_t *burst = NULL;
    for (size_t i = 0; i < s_burst_count; i++) {
        if (frame_start < s_bursts[i].end_ms && s_now_ms > s_bursts[i].start_ms) {
            burst = &s_bursts[i];
        }
    }
    if (burst == NULL) {
        /* The room tone is the same PCM every frame, so its fingerprint is
         * computed once (main) rather than 15 times a second. */
        ambient_audio_note(&s_room_fp, t);
        return;
    }
    int16_t pcm[AMBIENT_FRAME_SAMPLES];
    twin_pcm_burst(s_cfg.seed, burst->frames++, burst->peak_db, pcm, AMBIENT_FRAME_SAMPLES);
    ambient_fingerprint_t fp;
    ambient_fingerprint_from_pcm(pcm, AMBIENT_FRAME_SAMPLES, &fp);
    ambient_audio_note(&fp, t);
}

/** When the next executor tick falls due. The first is at t = 0. */
static uint64_t s_next_exec_ms;

/**
 * One pass of reactive_task()'s loop body (reactive_controller.c:925-1104) at
 * the current virtual time, then the `wheels` and `head` records for what it
 * changed. xTaskDelayUntil() paces the task at REACTIVE_LOOP_PERIOD_MS on the
 * robot; here advance_to() does.
 *
 * The executor's host path keeps its own clock, ticks * 33 ms
 * (reactive_controller.c s_ticks), while goal_state, motor_controller.c and
 * everything else read esp_timer_get_time(). The two agree only while every
 * tick is issued exactly on its 33 ms boundary, so that is checked.
 */
static void executor_tick(void)
{
    const uint64_t expected_ms = twin_exec_ticks() * REACTIVE_LOOP_PERIOD_MS;
    if (expected_ms != s_now_ms) {
        die(6, "executor tick %llu issued at %llu ms, not %llu ms",
            (unsigned long long)twin_exec_ticks(), (unsigned long long)s_now_ms,
            (unsigned long long)expected_ms);
    }

    twin_exec_report_t r;
    twin_exec_tick(s_voice_busy, &r);
    if (r.wheels_changed) {
        emit("{\"t\":\"wheels\",\"at_ms\":%llu,\"left\":{\"drive\":\"%s\",\"duty\":%u},"
             "\"right\":{\"drive\":\"%s\",\"duty\":%u}}",
             (unsigned long long)s_now_ms, r.left.drive, (unsigned)r.left.duty, r.right.drive,
             (unsigned)r.right.duty);
    }
    if (r.head_changed) {
        emit("{\"t\":\"head\",\"at_ms\":%llu,\"pan_deg\":%d,\"tilt_deg\":%d,\"mode\":\"%s\","
             "\"settled\":%s}",
             (unsigned long long)s_now_ms, r.pan_deg, r.tilt_deg, r.mode,
             r.settled ? "true" : "false");
    }
}

/**
 * Advance the virtual clock to @p t_ms, applying everything due on the way in
 * time order: world events, the end of a voice turn, microphone frames,
 * executor ticks. At the same instant world events go first, so an event at a
 * tick's time is seen by that tick, and the executor goes last, so its tick
 * sees everything else that happened at that instant. A sound is the exception
 * in effect, not in order: it is heard through frames, and the first frame
 * that holds it ends after it starts.
 *
 * backend_plan() calls this for a reply's latency window, so executor ticks
 * inside the window run against the previous goal, as on the robot.
 */
static void advance_to(uint64_t t_ms)
{
    for (;;) {
        uint64_t next = UINT64_MAX;
        int which = -1;
        if (s_event_next < s_event_count && s_events[s_event_next].at_ms <= t_ms) {
            next = s_events[s_event_next].at_ms;
            which = 0;
        }
        if (s_voice_busy && s_voice_end_ms <= t_ms && s_voice_end_ms < next) {
            next = s_voice_end_ms;
            which = 1;
        }
        if (s_next_audio_ms <= t_ms && s_next_audio_ms < next) {
            next = s_next_audio_ms;
            which = 2;
        }
        if (s_next_exec_ms <= t_ms && s_next_exec_ms < next) {
            next = s_next_exec_ms;
            which = 3;
        }
        if (which < 0) {
            break;
        }
        if (next > s_now_ms) {
            s_now_ms = next;
        }
        if (which == 0) {
            apply_event(&s_events[s_event_next++]);
        } else if (which == 1) {
            voice_turn_end();
        } else if (which == 2) {
            audio_frame();
            s_next_audio_ms += AUDIO_FRAME_MS;
        } else {
            executor_tick();
            s_next_exec_ms += REACTIVE_LOOP_PERIOD_MS;
        }
    }
    if (t_ms > s_now_ms) {
        s_now_ms = t_ms;
    }
}

/* ========================================================================== */
/* Model port                                                                 */
/* ========================================================================== */

static uint32_t s_request_id;

typedef struct {
    int status;
    uint32_t latency_ms;
    char *body; /* malloc'd, may be "" */
} model_reply_t;

/** Block on stdin for the model_response answering request @p id. */
static model_reply_t await_response(uint32_t id)
{
    char *line = read_line();
    if (!line) {
        die(3, "stdin closed while waiting for model_response %u", id);
    }
    cJSON *obj = cJSON_Parse(line);
    if (!obj || !cJSON_IsObject(obj)) {
        die(2, "model_response line is not a JSON object: %.200s", line);
    }
    char t[32];
    str_field(obj, "t", t, sizeof(t), "?");
    if (strcmp(t, "model_response") != 0) {
        die(2, "expected model_response for request %u, got '%s'", id, t);
    }
    const long got_id = int_field(obj, "id", true, 0, t);
    if (got_id != (long)id) {
        die(2, "model_response id %ld does not answer request %u", got_id, id);
    }
    model_reply_t r = {0};
    r.status = (int)int_field(obj, "status", true, 0, t);
    const long lat = int_field(obj, "latency_ms", true, 0, t);
    r.latency_ms = (uint32_t)(lat < 0 ? 0 : lat);
    const cJSON *body = field(obj, "body", false, t);
    if (body && !cJSON_IsString(body) && !cJSON_IsNull(body)) {
        die(2, "model_response.body must be a string");
    }
    r.body = strdup((body && cJSON_IsString(body)) ? body->valuestring : "");
    cJSON_Delete(obj);
    if (!r.body) {
        die(5, "out of memory");
    }
    return r;
}

/**
 * The request descriptor: a phase-1 proxy for the request body gemini_backend.c
 * would send (gemini_request.c is not extracted yet, ADR-030 §2). Keys in sorted
 * order with no whitespace, so the bytes are the canonical form the harness
 * hashes. The cycle index is deliberately absent: the same evidence must map to
 * the same cassette whenever it occurs.
 */
static void build_descriptor(char *out, size_t cap, const char *gate, bool speak_allowed,
                             note_trigger_t note_offer)
{
    size_t pos = 0;
    pos += (size_t)snprintf(out + pos, cap - pos, "{\"gate\":");
    pos = json_escape(out, cap, pos, gate);
    pos += (size_t)snprintf(out + pos, cap - pos, ",\"kind\":\"plan\",\"note\":");
    pos = json_escape(out, cap, pos, note_trigger_name(note_offer));
    pos += (size_t)snprintf(out + pos, cap - pos, ",\"pose\":");
    const char pose[2] = {s_capture_pose, '\0'};
    pos = json_escape(out, cap, pos, pose);
    pos += (size_t)snprintf(out + pos, cap - pos, ",\"scene\":");
    pos = json_escape(out, cap, pos, s_frame_scene);
    pos += (size_t)snprintf(out + pos, cap - pos, ",\"speak_allowed\":%s}",
                            speak_allowed ? "true" : "false");
    if (pos >= cap) {
        die(4, "descriptor too long");
    }
}

/**
 * gemini_backend_plan() on the twin. The guard order is gemini_backend.c:610-632
 * (fuse, then WiFi, then key); the result handling is gemini_backend.c:668-704,
 * with gemini_http.c:95 turning any status other than 200 into an error:
 * a non-200 status is a failed request, counted against the request ceiling
 * and charged no tokens (plan_budget_note(-1, false)); a 200 is parsed, and the
 * tokens from gemini_parse_last_usage() are charged whether or not the parse
 * produced a goal (-1 when usage is absent, which plan_budget charges at its
 * assumed figure).
 *
 * World events due inside the request's latency are applied before the reply
 * is handled, so a voice turn that starts mid-request is visible to the
 * planner's after-request snapshot exactly as on the robot.
 */
static esp_err_t backend_plan(uint64_t request_ms, goal_t *goal, char *speech, size_t cap,
                              note_trigger_t note_offer, gemini_note_t *note)
{
    speech[0] = '\0';
    note->kind = NOTE_KIND_NONE; /* gemini_backend_plan() clears it on every path */
    note->text[0] = '\0';
    if (!plan_budget_allows()) {
        goal->kind = GOAL_KIND_STOP;
        return ESP_ERR_NOT_ALLOWED;
    }
    if (!s_wifi || !s_key) {
        goal->kind = GOAL_KIND_STOP;
        return ESP_ERR_INVALID_STATE;
    }

    /* build_request_json() decides whether to offer `speak` from the gates as
     * they stand before the request (gemini_backend.c:414-418); the verdict
     * string encodes the same budget && (scene || audio). */
    const char *gate = planner_cycle_gate_verdict((uint32_t)request_ms);
    const bool speak_allowed = (gate[0] == 'V' || gate[0] == 'A');
    char descriptor[512];
    build_descriptor(descriptor, sizeof(descriptor), gate, speak_allowed, note_offer);
    char key[65];
    twin_sha256_hex(descriptor, strlen(descriptor), key);

    const uint32_t id = ++s_request_id;
    emit("{\"t\":\"model_request\",\"id\":%u,\"at_ms\":%llu,\"kind\":\"plan\",\"descriptor\":%s,"
         "\"key\":\"%s\"}",
         id, (unsigned long long)request_ms, descriptor, key);
    model_reply_t reply = await_response(id);

    advance_to(request_ms + reply.latency_ms);

    esp_err_t err;
    if (reply.status != 200) {
        plan_budget_note(-1, false);
        goal->kind = GOAL_KIND_STOP;
        err = ESP_FAIL;
    } else {
        err = gemini_parse_response(reply.body, goal, speech, cap);
        gemini_usage_t usage;
        gemini_parse_last_usage(&usage);
        plan_budget_note(usage.present ? usage.total : -1, true);
        /* build_tools(may_speak) declares `speak` only when it was offered
         * (gemini_backend.c:535), so the model cannot call it otherwise. A
         * scripted or recorded reply that does is one the robot could never
         * receive: flag it and drop the line, so no scenario passes on speech
         * the firmware did not offer. */
        if (speech[0] != '\0' && !speak_allowed) {
            emit("{\"t\":\"speak_not_offered\",\"at_ms\":%llu,\"text\":%s}",
                 (unsigned long long)s_now_ms, jstr(speech));
            speech[0] = '\0';
        }
        /* gemini_backend_plan() hands back a note only when it offered one
         * (gemini_backend.c, after the parse); flag and drop the rest. */
        gemini_parse_last_note(note);
        if (note->kind != NOTE_KIND_NONE && note_offer == NOTE_TRIGGER_NONE) {
            emit("{\"t\":\"note_not_offered\",\"at_ms\":%llu,\"text\":%s}",
                 (unsigned long long)s_now_ms, jstr(note->text));
            note->kind = NOTE_KIND_NONE;
            note->text[0] = '\0';
        }
    }
    free(reply.body);
    return err;
}

/* ========================================================================== */
/* Planner cycle                                                              */
/* ========================================================================== */

static planner_cycle_state_t s_cycle;

static const char *goal_kind_name(goal_kind_t kind)
{
    switch (kind) {
        case GOAL_KIND_NONE:
            return "none";
        case GOAL_KIND_STOP:
            return "stop";
        case GOAL_KIND_DRIVE:
            return "drive";
        case GOAL_KIND_TRACK:
            return "track";
        case GOAL_KIND_ROTATE:
            return "rotate";
        default:
            return "unknown";
    }
}

static void force_stop(const char *reason)
{
    goal_state_force_stop();
    emit("{\"t\":\"force_stop\",\"at_ms\":%llu,\"reason\":\"%s\"}", (unsigned long long)s_now_ms,
         reason);
}

static void announce(const char *what)
{
    emit("{\"t\":\"announce\",\"at_ms\":%llu,\"what\":\"%s\"}", (unsigned long long)s_now_ms, what);
}

/** The planner log line's fields (planner_task.c "Goal:" / "Idle:"). */
static void emit_cycle(const char *why_cycle, uint64_t tick_ms, const char *gate,
                       const planner_cycle_decision_t *d, bool requested)
{
    emit("{\"t\":\"cycle\",\"at_ms\":%llu,\"tick_ms\":%llu,\"why\":%s,\"cause\":\"%s\","
         "\"requested\":%s,\"step\":%u,\"period_ms\":%u,\"dormant\":%s,\"still\":%u,"
         "\"scene\":%u,\"pose\":\"%c\",\"gate\":\"%s\",\"warranted\":%s,\"affordable\":%s,"
         "\"spent_req\":%u,\"spent_tok\":%llu}",
         (unsigned long long)s_now_ms, (unsigned long long)tick_ms, jstr(plan_activity_verdict()),
         why_cycle, requested ? "true" : "false", (unsigned)plan_activity_step() + 1u,
         (unsigned)plan_activity_period_ms(), plan_activity_dormant() ? "true" : "false",
         plan_activity_scene_score(), scene_change_score(), scene_pose_letter(scene_change_pose()),
         gate, d->warranted ? "true" : "false", d->affordable ? "true" : "false",
         (unsigned)plan_budget_requests(), (unsigned long long)plan_budget_tokens());
}

/** plan_evidence_t as planner_task.c gather_evidence() builds it. */
static plan_evidence_t gather_evidence(const scene_fingerprint_t *fp, uint32_t now_ms)
{
    plan_evidence_t ev = {0};
    ev.frame = *fp;
    ev.audio_event = ambient_audio_event(now_ms);
    /* planner_task.c:165-179, line for line. Nothing in the twin takes a
     * manual lease, so manual_active stays false. */
    reactive_telemetry_t tele = {0};
    if (reactive_controller_get_telemetry(&tele) == ESP_OK) {
        ev.distance_cm = tele.distance_cm;
        ev.range_valid = !tele.sensor_failed;
        ev.robot_moving = tele.manual_active;
    }
    goal_t current = {0};
    bool fresh = false;
    if (!ev.robot_moving && goal_state_read(&current, &fresh) == ESP_OK) {
        ev.robot_moving = goal_moves_wheels(&current, fresh);
    }
    return ev;
}

/**
 * One pass of planner_task()'s loop body (planner_task.c, steps 1-6), at the
 * virtual time @p tick_ms. Returns the time of the next tick.
 */
static uint64_t planner_tick(uint64_t tick_ms)
{
    const uint64_t period = s_cfg.period_ms;

    /* ---- 1. Capture frame ---- (no frame yet == camera_capture() failed) */
    if (!s_have_frame) {
        force_stop("camera");
        return tick_ms + period;
    }

    /* Where the head was pointing when the frame was taken, sampled as
     * planner_task.c:269-279 samples it right after camera_capture() returns.
     * Without servos (`head` unset) both angles are zero and the head reads as
     * settled. */
    servo_position_t head_at_capture = {0};
    bool head_settled = true;
    if (servo_is_initialized()) {
        (void)servo_get_position(&head_at_capture);
        reactive_telemetry_t tele = {0};
        head_settled = (reactive_controller_get_telemetry(&tele) == ESP_OK) && tele.head_settled;
    }

    /* ---- 1b. Fingerprint with the pose at capture, note it ----
     * planner_task.c:298-300. With `head` set the pose is the dwell pose the
     * servos are at, or none while the head is unsettled or between poses.
     * Without it the frame keeps its scripted frame.pose, which is how a
     * scenario exercises the per-pose references with no servos fitted. */
    scene_fingerprint_t fp = s_frame_fp;
    if (s_cfg.head) {
        fp.pose = head_settled ? (uint8_t)scene_pose_from_head(head_at_capture.pan_angle,
                                                               head_at_capture.tilt_angle)
                               : (uint8_t)SCENE_POSE_NONE;
    }
    s_capture_pose = scene_pose_letter(fp.pose);
    scene_change_note(&fp);

    /* ---- 1c. Decide whether this cycle is worth a request ---- */
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const plan_evidence_t ev = gather_evidence(&fp, now_ms);
    const planner_cycle_decision_t decision = planner_cycle_decide(&s_cycle, &ev, now_ms);

    if (decision.action != PLANNER_CYCLE_REQUEST) {
        const char *cause = "idle";
        switch (decision.action) {
            case PLANNER_CYCLE_SKIP_BUDGET_ANNOUNCE:
                announce("budget");
                force_stop("budget");
                cause = "budget";
                break;
            case PLANNER_CYCLE_SKIP_BUDGET_SILENT:
                cause = "budget";
                break;
            case PLANNER_CYCLE_SKIP_DORMANT_ANNOUNCE:
                announce("dormant");
                force_stop("dormant");
                cause = "dormant";
                break;
            case PLANNER_CYCLE_SKIP_DORMANT_QUIET:
                cause = "dormant";
                break;
            case PLANNER_CYCLE_SKIP_IDLE:
            case PLANNER_CYCLE_REQUEST:
            default:
                break;
        }
        /* No gate is sampled on a skipped cycle in the firmware; the verdict here
         * is a read of the same gates, reported for the trace only. */
        emit_cycle(cause, tick_ms, planner_cycle_gate_verdict(now_ms), &decision, false);
        return tick_ms + period;
    }

    if (decision.announce_waking) {
        announce("waking");
    }

    /* ---- 1d. Offer the note tool? (planner_task.c step 1d) ---- */
    reactive_telemetry_t note_tele = {0};
    (void)reactive_controller_get_telemetry(&note_tele);
    const planner_note_inputs_t note_in = {
        .reflex_trips = note_tele.reflex_trips,
        .blind = !scene_change_has_measurement(),
    };
    const note_trigger_t note_offer = planner_cycle_note_offer(&s_cycle, &note_in, now_ms);
    if (note_offer != NOTE_TRIGGER_NONE) {
        emit("{\"t\":\"note_offered\",\"at_ms\":%llu,\"trigger\":\"%s\",\"offers\":%u}",
             (unsigned long long)s_now_ms, note_trigger_name(note_offer),
             (unsigned)note_gate_offers());
    }

    /* ---- 2. Call the planner model ---- */
    goal_t goal = {0};
    char speech[SPEECH_TEXT_MAX] = {0};
    gemini_note_t note;
    planner_speech_snapshot_t at_request = {0};
    at_request.speech_notes = speech_budget_notes();
    at_request.voice_turns = s_voice_turns;
    const esp_err_t ret = backend_plan(now_ms, &goal, speech, sizeof(speech), note_offer, &note);

    const char *const gate_verdict =
        planner_cycle_after_request(now_ms, (uint32_t)(esp_timer_get_time() / 1000));

    /* ---- 2b. Screen and post the utterance ---- */
    planner_speech_snapshot_t after_request = {0};
    after_request.voice_busy = s_voice_busy;
    after_request.voice_turns = s_voice_turns;
    after_request.speech_notes = speech_budget_notes();
    const planner_speech_verdict_t speech_verdict =
        planner_cycle_screen_speech(speech, &at_request, &after_request);
    if (speech_verdict == PLANNER_SPEECH_SUPPRESS_REPEAT) {
        emit("{\"t\":\"suppressed\",\"at_ms\":%llu,\"reason\":\"repeat\",\"text\":%s}",
             (unsigned long long)s_now_ms, jstr(speech));
    } else if (speech_verdict == PLANNER_SPEECH_SUPPRESS_PERSON_SPOKE) {
        emit("{\"t\":\"suppressed\",\"at_ms\":%llu,\"reason\":\"person_spoke\",\"text\":%s}",
             (unsigned long long)s_now_ms, jstr(speech));
    } else if (speech_verdict == PLANNER_SPEECH_POST) {
        /* speech_queue_post() always accepts in phase 1: TTS playback time is
         * not modelled, so the queue is never found still speaking. */
        planner_cycle_note_spoken(speech, (uint32_t)(esp_timer_get_time() / 1000));
        emit("{\"t\":\"utterance\",\"at_ms\":%llu,\"source\":\"planner\",\"text\":%s}",
             (unsigned long long)s_now_ms, jstr(speech));
    }

    /* ---- 3 / 4. Write goal or force stop ---- */
    const planner_result_action_t result = planner_cycle_result(&s_cycle, ret);

    /* ---- 4b. Notes (planner_task.c step 4b); event_report is not compiled
     * in, so the note is a trace record. */
    planner_cycle_note_outcome(&s_cycle, result, goal.kind, speech_verdict, note_offer);
    if (note_offer != NOTE_TRIGGER_NONE && note.kind != NOTE_KIND_NONE) {
        note_gate_noted();
        emit("{\"t\":\"note\",\"at_ms\":%llu,\"trigger\":\"%s\",\"kind\":\"%s\",\"text\":%s}",
             (unsigned long long)s_now_ms, note_trigger_name(note_offer), note_kind_name(note.kind),
             jstr(note.text));
    }
    if (result == PLANNER_RESULT_WRITE_GOAL) {
        /* planner_task.c:447-448: the goal is read in the frame's coordinates,
         * so it carries the head angles sampled at capture. */
        goal.head_pan_deg = head_at_capture.pan_angle;
        goal.head_tilt_deg = head_at_capture.tilt_angle;
        if (goal_state_write(&goal, 0) != ESP_OK) {
            fprintf(stderr, "robocar-twin: goal_state_write failed at %llu ms\n",
                    (unsigned long long)s_now_ms);
        }
        emit("{\"t\":\"goal\",\"at_ms\":%llu,\"kind\":\"%s\"}", (unsigned long long)s_now_ms,
             goal_kind_name(goal.kind));
    } else if (result == PLANNER_RESULT_STOP_IDLE) {
        force_stop("no_link");
    } else {
        force_stop("backend_error");
        emit("{\"t\":\"backoff\",\"at_ms\":%llu,\"periods\":%u}", (unsigned long long)s_now_ms,
             (unsigned)s_cycle.backoff_periods);
    }
    emit_cycle("request", tick_ms, gate_verdict, &decision, true);

    /* ---- 6. Sleep until the next period ----
     * xTaskDelayUntil(last_wake, P * wait) delays only while the deadline is
     * still ahead; past it, it returns pdFALSE and the loop resyncs with
     * vTaskDelay(P) (planner_task.c:401-406). */
    const uint64_t deadline = tick_ms + period * planner_cycle_wait_periods(&s_cycle);
    const uint64_t now_after = s_now_ms;
    return (deadline > now_after) ? deadline : now_after + period;
}

/* ========================================================================== */
/* main                                                                       */
/* ========================================================================== */

int main(void)
{
    /* Keep the protocol stream private: firmware modules printf on the host. */
    const int proto_fd = dup(STDOUT_FILENO);
    if (proto_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        die(5, "cannot set up stdout: %s", strerror(errno));
    }
    s_out = fdopen(proto_fd, "w");
    if (!s_out) {
        die(5, "cannot open protocol stream: %s", strerror(errno));
    }

    read_setup();

    /* Firmware init order: main.c init_hierarchical_ai() (goal_state, then the
     * speech gates, then plan_budget and plan_activity). */
    s_now_ms = 0;
    goal_state_set_clock_override(esp_timer_get_time);
    if (goal_state_init() != ESP_OK) {
        die(5, "goal_state_init failed");
    }
    dialogue_style_seed(s_cfg.seed); /* voice_persona.c seeds it from esp_random() */
    speech_budget_init();
    scene_change_init();
    ambient_audio_init();
    plan_budget_init();
    if (s_cfg.max_requests >= 0 || s_cfg.max_tokens >= 0) {
        uint32_t max_req = 0, max_tok = 0;
        plan_budget_get(&max_req, &max_tok);
        plan_budget_configure(s_cfg.max_requests >= 0 ? (uint32_t)s_cfg.max_requests : max_req,
                              s_cfg.max_tokens >= 0 ? (uint32_t)s_cfg.max_tokens : max_tok);
    }
    plan_activity_init(s_cfg.period_ms);
    note_gate_init();

    /* The executor: motors and servos from init_hardware(), then
     * reactive_controller_init() (main.c:2396), which planner_task_init()
     * follows. Its first tick is at t = 0. */
    const char *const exec_failed = twin_exec_init(s_cfg.seed, s_cfg.head, s_cfg.cliff);
    if (exec_failed != NULL) {
        die(5, "%s failed", exec_failed);
    }

    int16_t room[AMBIENT_FRAME_SAMPLES];
    twin_pcm_room(s_cfg.seed, room, AMBIENT_FRAME_SAMPLES);
    ambient_fingerprint_from_pcm(room, AMBIENT_FRAME_SAMPLES, &s_room_fp);

    uint64_t next_tick = 0;
    while (next_tick < s_cfg.duration_ms) {
        advance_to(next_tick);
        next_tick = planner_tick(next_tick);
    }
    advance_to(s_cfg.duration_ms);
    emit("{\"t\":\"done\",\"at_ms\":%llu}", (unsigned long long)s_now_ms);
    fclose(s_out);
    return 0;
}
