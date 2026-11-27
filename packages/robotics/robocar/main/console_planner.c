/**
 * @file console_planner.c
 * @brief Console commands for the planner: `plan`, `idle`, `quest` and `report`.
 *
 * Moved from main.c unchanged; execute_console_line() reaches each handler
 * through the console command table (console_cmd_core.h).
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"

#include "config.h"
#include "console_handlers.h"
#include "event_report.h"
#include "mqtt_logger.h"
#include "note_gate.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "planner_task.h"
#include "quest_log_core.h"
#include "quest_store.h"
#include "reactive_controller.h"
#include "scene_change.h"
#include "wall_clock.h"

/**
 * `plan` — the request gate and the spend ceiling.
 *
 * Every threshold here is a console knob for the same reason `cam gainceiling`
 * and `voice scene` are: whether the robot is sensibly frugal or annoyingly
 * asleep is a judgement that needs somebody standing in the room, and a reflash
 * per trial is far too slow a loop. Like those, they deliberately do NOT persist
 * to NVS — a boot should come up at the documented default, not at whatever last
 * night's experiment left behind.
 */
void handle_plan_cmd(const char *buf)
{
    char op[16] = {0};
    long value = 0;
    const int n = sscanf(buf, "plan %15s %ld", op, &value);

    if (n >= 1) {
        if (strcmp(op, "on") == 0 || strcmp(op, "off") == 0) {
            const bool on = (op[0] == 'o' && op[1] == 'n');
            plan_activity_set_enabled(on);
            printf("plan: dormancy %s%s\n", on ? "ENABLED" : "DISABLED",
                   on ? "" : " — a request every planner period, budget still enforced");
            return;
        }
        if (strcmp(op, "wake") == 0) {
            plan_activity_wake();
            printf("plan: woken — next planner cycle will make a request\n");
            return;
        }
        if (strcmp(op, "sleep") == 0) {
            plan_activity_sleep();
            printf("plan: dormant — still watching, no requests until something changes\n");
            return;
        }
        if (strcmp(op, "resume") == 0) {
            plan_budget_resume();
            note_gate_resume();
            plan_activity_wake();
            printf("plan: budget and note counters cleared, planner released\n");
            return;
        }
        if (n == 2 && strcmp(op, "scene") == 0) {
            uint8_t range = 0;
            plan_activity_get(NULL, &range);
            plan_activity_configure((uint8_t)value, range);
            printf("plan: wake scene threshold = %ld%s\n", value,
                   (value == 0) ? " (view no longer wakes it)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "range") == 0) {
            uint8_t scene = 0;
            plan_activity_get(&scene, NULL);
            plan_activity_configure(scene, (uint8_t)value);
            printf("plan: wake range threshold = %ld cm%s\n", value,
                   (value == 0) ? " (rangefinder no longer wakes it)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "requests") == 0) {
            uint32_t max_tokens = 0;
            plan_budget_get(NULL, &max_tokens);
            plan_budget_configure((uint32_t)value, max_tokens);
            printf("plan: request ceiling = %ld%s\n", value, (value == 0) ? " (no limit)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "tokens") == 0) {
            uint32_t max_requests = 0;
            plan_budget_get(&max_requests, NULL);
            plan_budget_configure(max_requests, (uint32_t)value);
            printf("plan: token ceiling = %ld%s\n", value, (value == 0) ? " (no limit)" : "");
            return;
        }
        printf("plan: usage: plan | plan on|off|wake|sleep|resume | "
               "plan scene <n> | plan range <cm> | plan requests <n> | plan tokens <n>\n");
        return;
    }

    /* ---- Status ---- */
    uint8_t scene_threshold = 0;
    uint8_t range_threshold = 0;
    plan_activity_get(&scene_threshold, &range_threshold);

    uint32_t max_requests = 0;
    uint32_t max_tokens = 0;
    plan_budget_get(&max_requests, &max_tokens);

    const plan_budget_state_t budget = plan_budget_state();

    printf("plan: gate=%s state=%s\n", plan_activity_enabled() ? "on" : "OFF",
           plan_activity_dormant() ? "DORMANT" : "awake");
    printf("  cadence: step %u/%u -> %" PRIu32 " ms   (base %u ms)\n",
           (unsigned)plan_activity_step() + 1u, (unsigned)PLAN_LADDER_STEPS,
           plan_activity_period_ms(), (unsigned)PLANNER_LOOP_PERIOD_MS);
    printf("  wake on: view %u/%u at pose %c   range %u/%u cm   sound (see `mic`)   why=%s\n",
           plan_activity_scene_score(), (unsigned)scene_threshold,
           scene_pose_letter(plan_activity_pose()), plan_activity_range_score(),
           (unsigned)range_threshold, plan_activity_verdict());

    /* The assumed-cost count is printed unconditionally rather than only when
     * non-zero: it is the one field that says whether the token figure beside it
     * is measured or guessed, and a reader who has to notice an ABSENT line to
     * learn that will not notice it. */
    printf("  spent:   %" PRIu32 " requests", plan_budget_requests());
    if (max_requests != 0u) {
        printf(" / %" PRIu32, max_requests);
    } else {
        printf(" (no ceiling)");
    }
    printf(",  %llu tokens", (unsigned long long)plan_budget_tokens());
    if (max_tokens != 0u) {
        printf(" / %" PRIu32 "\n", max_tokens);
    } else {
        printf(" (no ceiling)\n");
    }
    printf("  of which %" PRIu32 " request(s) charged the assumed %u tokens because the "
           "response carried no usageMetadata\n",
           plan_budget_assumed(), (unsigned)PLAN_BUDGET_ASSUMED_TOKENS);

    if (budget != PLAN_BUDGET_OK) {
        printf("  *** BUDGET EXHAUSTED (%s) — no requests until `plan resume` ***\n",
               (budget == PLAN_BUDGET_TRIP_REQUESTS) ? "request ceiling" : "token ceiling");
    }

    /* The note gate (ADR-033): offers are what is rationed, so they lead. */
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    printf("  notes:   %" PRIu32 "/%u offered, %" PRIu32 " written, next %s", note_gate_offers(),
           (unsigned)NOTE_GATE_MAX_PER_BOOT, note_gate_written(),
           note_gate_spent() ? "after `plan resume`" : "");
    if (!note_gate_spent()) {
        printf("in %" PRIu32 " s", note_gate_wait_ms(now_ms) / 1000u);
    }
    printf(",  pending:");
    const uint32_t pending = note_gate_pending();
    if (pending == 0u) {
        printf(" none");
    }
    for (int t = NOTE_TRIGGER_NONE + 1; t < NOTE_TRIGGER_COUNT; t++) {
        if (pending & (1u << (unsigned)t)) {
            printf(" %s", note_trigger_name((note_trigger_t)t));
        }
    }
    printf("\n");
}

/**
 * @brief `idle` — the idle behaviour the executor runs on its own (ADR-029).
 *
 * `idle` alone prints the mode, what the head is doing and whether it has
 * settled (whether a frame captured now gets a dwell pose). `idle gaze on|off`
 * switches the gaze; off returns the head to centre whenever nothing directs
 * it, as before ADR-029. Not persisted: every boot starts with the gaze on.
 * `idle` alone only reads, which is why it is on the MQTT read-only list.
 */
void handle_idle_cmd(const char *buf)
{
    char what[8] = {0};
    char state[8] = {0};
    const int n = sscanf(buf, "idle %7s %7s", what, state);

    if (n == 2 && strcmp(what, "gaze") == 0 &&
        (strcmp(state, "on") == 0 || strcmp(state, "off") == 0)) {
        const bool on = (strcmp(state, "on") == 0);
        reactive_controller_set_gaze(on);
        printf("idle: gaze %s\n", on ? "ON — the head looks around when nothing directs it"
                                     : "OFF — the head re-centres when nothing directs it");
        return;
    }
    if (n > 0) {
        printf("idle: usage: idle | idle gaze on|off\n");
        return;
    }

    reactive_telemetry_t tele = {0};
    const bool have_tele = (reactive_controller_get_telemetry(&tele) == ESP_OK);
    printf("idle: gaze=%s\n", reactive_controller_gaze_enabled() ? "on" : "off");
    if (have_tele) {
        printf("  head: %s, %s\n", reactive_head_mode_name(tele.head_mode),
               tele.head_settled ? "settled (frames get a dwell pose)"
                                 : "moving (frames are off-pose)");
    }
    printf("  usage: idle gaze on|off\n");
}

/* ---- `quest`: Teuvo's quest log (ADR-039 §1) ---------------------------- */

static const char *skip_spaces(const char *p)
{
    while (*p == ' ') {
        p++;
    }
    return p;
}

/* Copy the next space-delimited word of *p into @p out and advance *p past it. */
static void next_word(const char **p, char *out, size_t out_len)
{
    const char *s = skip_spaces(*p);
    size_t n = 0;
    while (s[n] != '\0' && s[n] != ' ') {
        n++;
    }
    const size_t keep = n < out_len - 1 ? n : out_len - 1;
    memcpy(out, s, keep);
    out[keep] = '\0';
    *p = skip_spaces(s + n);
}

static bool parse_quest_id(const char *word, uint16_t *id)
{
    char *end = NULL;
    const unsigned long v = strtoul(word[0] == '#' ? word + 1 : word, &end, 10);
    if (end == NULL || *end != '\0' || v == 0 || v > UINT16_MAX) {
        return false;
    }
    *id = (uint16_t)v;
    return true;
}

static void print_quest_line(const quest_log_t *log, const quest_t *q)
{
    printf("  #%u %-7s %-7s %s (%u note%s)\n", (unsigned)q->id, quest_kind_name(q->kind),
           quest_state_name(q->state), q->title, (unsigned)quest_log_note_count(log, q->id),
           quest_log_note_count(log, q->id) == 1 ? "" : "s");
}

static void print_note_line(const quest_note_t *n)
{
    char when[24];
    wall_clock_format(n->at, when, sizeof(when));
    printf("    #%u %s %c  %s\n", (unsigned)n->quest_id, when, n->pose, n->text);
}

static void print_quest_summary(const quest_log_t *log)
{
    char now[24];
    const quest_time_t t = wall_clock_now();
    wall_clock_format(t, now, sizeof(now));
    printf("quest: %u quest(s), %u open, %u note(s), %u bytes on flash; clock %s%s\n",
           (unsigned)log->quest_count, (unsigned)quest_log_open_count(log),
           (unsigned)log->note_count, (unsigned)quest_store_file_bytes(), now,
           t.synced ? "" : " (not synced)");
    size_t closed = 0;
    for (size_t i = 0; i < log->quest_count; i++) {
        const quest_t *q = &log->quests[i];
        if (q->state == QUEST_STATE_ACTIVE || q->state == QUEST_STATE_OPEN) {
            print_quest_line(log, q);
        } else {
            closed++;
        }
    }
    if (closed > 0) {
        printf("  closed: %u (quest show <id>)\n", (unsigned)closed);
    }
    const size_t first = log->note_count > 5 ? log->note_count - 5 : 0;
    if (log->note_count > 0) {
        printf("  last notes:\n");
        for (size_t i = first; i < log->note_count; i++) {
            print_note_line(&log->notes[i]);
        }
    }
}

static void print_quest_detail(const quest_log_t *log, const quest_t *q)
{
    char opened[24];
    wall_clock_format(q->opened, opened, sizeof(opened));
    printf("quest #%u: %s\n  kind %s, %s, opened %s\n", (unsigned)q->id, q->title,
           quest_kind_name(q->kind), quest_state_name(q->state), opened);
    if (q->why[0] != '\0') {
        printf("  why: %s\n", q->why);
    }
    if (q->state == QUEST_STATE_DONE || q->state == QUEST_STATE_DROPPED) {
        char closed[24];
        wall_clock_format(q->closed, closed, sizeof(closed));
        printf("  closed %s: %s\n", closed, q->outcome[0] != '\0' ? q->outcome : "(no outcome)");
    }
    for (size_t i = 0; i < log->note_count; i++) {
        if (log->notes[i].quest_id == q->id) {
            print_note_line(&log->notes[i]);
        }
    }
}

/**
 * @brief `quest` console command: read and edit Teuvo's quest log by hand.
 *
 * `quest` lists the open quests and the last notes; the rest change the log
 * and are written to flash by quest_store's task. Opening and noting by hand
 * is how the planner's quest clause is tried on the bench before the curator
 * exists (ADR-039, implementation order).
 */
void handle_quest_cmd(const char *buf)
{
    static const char *const usage =
        "quest: usage: quest | quest show <id> | quest open <explore|find|revisit|ask> <title> |"
        " quest note <id> <text> | quest active <id> | quest done|drop <id> [outcome] |"
        " quest clear yes";
    const char *p = skip_spaces(buf + 5);
    char sub[12];
    next_word(&p, sub, sizeof(sub));

    quest_log_t *log = quest_store_lock();
    if (log == NULL) {
        printf("quest: store %s\n", quest_store_state());
        return;
    }

    bool changed = false;
    quest_err_t err = QUEST_OK;
    char word[16];
    uint16_t id = 0;
    const quest_time_t now = wall_clock_now();

    if (sub[0] == '\0') {
        print_quest_summary(log);
        printf("  %s\n", usage + strlen("quest: "));
    } else if (strcmp(sub, "show") == 0) {
        next_word(&p, word, sizeof(word));
        const quest_t *q = parse_quest_id(word, &id) ? quest_log_find(log, id) : NULL;
        if (q == NULL) {
            printf("quest: no quest %s\n", word);
        } else {
            print_quest_detail(log, q);
        }
    } else if (strcmp(sub, "open") == 0) {
        quest_kind_t kind;
        next_word(&p, word, sizeof(word));
        if (!quest_kind_from_name(word, &kind) || *p == '\0') {
            printf("%s\n", usage);
        } else {
            err = quest_log_open(log, kind, p, "opened by hand at the console", now, &id);
            if (err == QUEST_OK) {
                changed = true;
                printf("quest: opened #%u (%s)\n", (unsigned)id, quest_kind_name(kind));
            }
        }
    } else if (strcmp(sub, "note") == 0) {
        next_word(&p, word, sizeof(word));
        if (!parse_quest_id(word, &id) || *p == '\0') {
            printf("%s\n", usage);
        } else {
            err = quest_log_note(log, id, '-', p, now);
            if (err == QUEST_OK) {
                changed = true;
                printf("quest: noted on #%u\n", (unsigned)id);
            }
        }
    } else if (strcmp(sub, "active") == 0) {
        next_word(&p, word, sizeof(word));
        if (!parse_quest_id(word, &id)) {
            printf("%s\n", usage);
        } else {
            err = quest_log_set_active(log, id);
            if (err == QUEST_OK) {
                changed = true;
                printf("quest: #%u is active\n", (unsigned)id);
            }
        }
    } else if (strcmp(sub, "done") == 0 || strcmp(sub, "drop") == 0) {
        const bool done = (strcmp(sub, "done") == 0);
        next_word(&p, word, sizeof(word));
        if (!parse_quest_id(word, &id)) {
            printf("%s\n", usage);
        } else {
            err = quest_log_close(log, id, done ? QUEST_STATE_DONE : QUEST_STATE_DROPPED, p, now);
            if (err == QUEST_OK) {
                changed = true;
                printf("quest: #%u %s\n", (unsigned)id, done ? "done" : "dropped");
            }
        }
    } else if (strcmp(sub, "clear") == 0) {
        next_word(&p, word, sizeof(word));
        if (strcmp(word, "yes") != 0) {
            printf("quest: `quest clear yes` deletes every quest and note\n");
        } else {
            quest_log_clear(log);
            changed = true;
            printf("quest: log cleared\n");
        }
    } else {
        printf("%s\n", usage);
    }

    if (err != QUEST_OK) {
        printf("quest: %s\n", quest_err_name(err));
    }
    quest_store_unlock(changed);
}

/**
 * @brief `report` — the GlitchTip event path (ADR-033).
 *
 * `report` prints the event counters and whether the last boot's breadcrumb
 * ring survived its reset. `report test` queues a synthetic event carrying
 * this boot's breadcrumbs, to check the path end to end (robot, broker,
 * bridge, GlitchTip). Not on the MQTT read-only list: `report test` sends.
 */
void handle_report_cmd(const char *buf)
{
    char sub[8] = {0};
    const int n = sscanf(buf, "report %7s", sub);

#if EVENT_REPORT_CRASH_COMMAND
    if (n == 1 && strcmp(sub, "crash") == 0) {
        printf("report: crashing now (abort) to test the core dump path\n");
        fflush(stdout);
        abort();
    }
#endif
    if (n == 1 && strcmp(sub, "test") == 0) {
        const esp_err_t err = event_report_send_test();
        printf("report: test event %s\n", err == ESP_OK ? "queued" : esp_err_to_name(err));
        return;
    }
    if (n == 1) {
        printf("report: usage: report | report test\n");
        return;
    }

    event_report_stats_t st = {0};
    event_report_get_stats(&st);
    printf("report: topic %s, mqtt %s\n", EVENT_REPORT_TOPIC,
           mqtt_logger_is_connected() ? "connected" : "down");
    printf("  events: queued %u, sent %u, dropped %u, pending %u\n", (unsigned)st.queued,
           (unsigned)st.sent, (unsigned)st.dropped, (unsigned)st.pending);
    if (st.boot_crumbs_present) {
        printf("  last boot's breadcrumbs: %u line(s) survived the reset\n",
               (unsigned)st.boot_crumbs);
    } else {
        printf("  last boot's breadcrumbs: none (power-on, or the reset cleared RTC memory)\n");
    }
    printf("  core dump: %s\n", st.core_dump != NULL ? st.core_dump : "none");
    printf("  usage: report test\n");
}
