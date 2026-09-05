/**
 * @file planner_task.c
 * @brief Gemini planner task implementation (PLANNER_LOOP_PERIOD_MS, 15 s default).
 *
 * Captures a JPEG frame, calls Gemini Robotics-ER 2 via gemini_backend_plan(),
 * and writes the resulting goal into goal_state.  On any failure the planner
 * writes a STOP goal so the reactive executor immediately falls back to safe
 * hold rather than running stale.
 */

#include "planner_task.h"

#include <inttypes.h>

#include "ambient_audio.h"
#include "camera.h"
#include "dialogue_style.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame_dump.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gemini_backend.h"
#include "goal_state.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "reactive_controller.h"
#include "scene_change.h"
#include "speech_budget.h"
#include "speech_queue.h"

static const char *TAG = "planner";

/* =========================================================================
 * Task constants
 * ========================================================================= */

/* 10 KB rather than 8: gemini_backend.c's build_request_json() frame holds the
 * system prompt (GEMINI_SYSTEM_PROMPT_MAX) plus the variation directive and the
 * recalled-lines list, which together grew past 4 KB when the robot gained a
 * memory of what it had said. Grow this with those buffers, not after a stack
 * overflow reports it. */
#define PLANNER_TASK_STACK_SIZE 10240U
#define PLANNER_TASK_PRIORITY 3U
#define PLANNER_TASK_CORE 1

/* =========================================================================
 * Static task state
 * ========================================================================= */

static TaskHandle_t s_planner_task_handle;

/**
 * @brief Which gate decided this cycle, as a short literal for the log.
 *
 * "-" the budget held (rationed, regardless of evidence), "V" the view changed,
 * "A" the room sounded different, "VA" both, "." neither — nothing to say.
 *
 * This exists so the OR in gemini_backend.c can be judged from a capture instead
 * of defended from an armchair. Count the "A"-only cycles in a 20-minute
 * `just robocar-unified::monitor | tee` run: if they track the HVAC, the fix is
 * `voice loud` / `voice sound`; if they coincide with sentences worth hearing, the
 * OR is vindicated; if "A" never appears at all, the audio gate is dead weight and
 * either the thresholds or the mic gain are wrong.
 *
 * Returns a STRING LITERAL, and callers must pass it as a %s ARGUMENT. A ternary
 * in the format position expands into a syntax error pointing inside
 * esp_log_color.h, far from the cause — see .claude/rules/esp-log-format-literal.md.
 */
static const char *planner_gate_verdict(void)
{
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (!speech_budget_allows(now)) {
        return "-";
    }
    const bool v = scene_change_novel();
    const bool a = ambient_audio_novel(now);
    if (v && a) {
        return "VA";
    }
    if (v) {
        return "V";
    }
    if (a) {
        return "A";
    }
    return ".";
}

/* =========================================================================
 * Goal kind name — for human-readable logging
 * ========================================================================= */

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

/* =========================================================================
 * Threshold readbacks — the logged score is useless without the number it is
 * being compared against, and plan_activity/plan_budget expose theirs through
 * out-parameter getters rather than scalars.
 * ========================================================================= */

static uint8_t plan_scene_threshold(void)
{
    uint8_t scene = 0;
    plan_activity_get(&scene, NULL);
    return scene;
}

static uint8_t plan_range_threshold(void)
{
    uint8_t range = 0;
    plan_activity_get(NULL, &range);
    return range;
}

static uint32_t budget_max_requests(void)
{
    uint32_t max_requests = 0;
    plan_budget_get(&max_requests, NULL);
    return max_requests;
}

/* =========================================================================
 * Dormancy evidence
 * ========================================================================= */

/**
 * @brief Collect this cycle's on-device evidence for plan_activity.
 *
 * Every field is free — a fingerprint the frame decode already produced, a
 * telemetry snapshot the executor already maintains, a latch the microphone
 * task already fills. That is the whole reason dormancy can be decided before
 * the request rather than inside it.
 */
static plan_evidence_t gather_evidence(const scene_fingerprint_t *fp, uint32_t now_ms)
{
    plan_evidence_t ev = {0};
    ev.frame = *fp;

    /* ambient_audio_EVENT, not ambient_audio_novel(). novel() additionally
     * reports a room never spoken about as new — correct for speech, fatal
     * here: the robot only speaks when awake, so a dormant robot would see that
     * branch stay true forever and never go quiet at all. See ambient_audio.h. */
    ev.audio_event = ambient_audio_event(now_ms);

    reactive_telemetry_t tele = {0};
    if (reactive_controller_get_telemetry(&tele) == ESP_OK) {
        ev.distance_cm = tele.distance_cm;
        /* A failed rangefinder reads as max range through the filter, so an
         * unguarded reading would show one large step and then sit perfectly
         * still — a sensor that appears to agree that nothing is happening. */
        ev.range_valid = !tele.sensor_failed;
        ev.robot_moving = tele.manual_active;
    }

    goal_t current = {0};
    bool fresh = false;
    if (!ev.robot_moving && goal_state_read(&current, &fresh) == ESP_OK) {
        ev.robot_moving = fresh && current.kind != GOAL_KIND_STOP && current.kind != GOAL_KIND_NONE;
    }

    return ev;
}

/* =========================================================================
 * Planner task body
 * ========================================================================= */

static void planner_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Planner task started on core %d, period %u ms", xPortGetCoreID(),
             PLANNER_LOOP_PERIOD_MS);

    TickType_t last_wake_time = xTaskGetTickCount();

    /* Extra periods to wait after a failed plan. Gemini answers a quota
     * overrun with HTTP 429 and a retryDelay that *grows* while the client
     * keeps asking (observed climbing 19 s -> 31 s under a 1 Hz loop), so
     * retrying at full rate makes the outage longer. Capped so recovery stays
     * bounded; reset on the first success. */
    uint32_t backoff_periods = 0;

    /* One-shot latches so entering dormancy and tripping the fuse are announced
     * once each rather than every 15 s for the rest of the night — and so the
     * matching "waking" line has something to be the counterpart of. */
    bool dormant_announced = false;
    bool budget_announced = false;

    while (1) {
        /* ---- 1. Capture frame ---- */
        camera_fb_t *fb = camera_capture();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed — forcing stop");
            goal_state_force_stop();
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
            continue;
        }

        /* ---- 1b. Measure and (if armed) dump the frame ----
         * Placed here, before gemini_backend_plan() and while the planner still
         * holds the buffer, so what is measured and dumped is byte-identical to
         * what the model is about to be shown — and so a frame still lands even
         * if the HTTP call later hangs or 429s.
         *
         * The fingerprint is recorded before the request is built, because
         * gemini_backend reads scene_change_novel() while deciding whether to
         * offer the `speak` tool at all. */
        scene_fingerprint_t fp;
        frame_stats_log(fb->buf, fb->len, &fp);
        scene_change_note(&fp);
        frame_dump_maybe(fb->buf, fb->len);

        /* ---- 1c. Decide whether this cycle is worth a request at all ----
         * The evidence loop above runs every tick regardless; only the network
         * call below is gated. Capturing and fingerprinting a frame costs CPU
         * and nothing else, which is what keeps wake latency at one tick even
         * when the planner has been dormant for hours. */
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const plan_evidence_t ev = gather_evidence(&fp, now_ms);
        const bool warranted = plan_activity_should_call(&ev, now_ms);
        const bool affordable = plan_budget_allows();

        if (!warranted || !affordable) {
            /* No request. The goal is deliberately left to expire on its own
             * TTL rather than being force-stopped every skipped cycle: the
             * executor already treats a stale goal as a hold, and re-stopping a
             * stopped robot 240 times an hour would bury the one transition
             * that matters in the log. */
            if (!affordable) {
                /* The fuse outranks everything else in this branch: while it is
                 * blown there is no "next request in N ms" to report, and saying
                 * there is would describe a cadence that will never run. Reported
                 * once, then silent — an exhausted budget does not become news
                 * again every 15 s. */
                if (!budget_announced) {
                    budget_announced = true;
                    ESP_LOGE(TAG,
                             "PLANNER BUDGET EXHAUSTED (%s) — %" PRIu32 " requests, %llu tokens. "
                             "No further requests until `plan resume`.",
                             (plan_budget_state() == PLAN_BUDGET_TRIP_REQUESTS) ? "request ceiling"
                                                                                : "token ceiling",
                             plan_budget_requests(), (unsigned long long)plan_budget_tokens());
                    goal_state_force_stop();
                }
            } else if (plan_activity_dormant() && !dormant_announced) {
                dormant_announced = true;
                ESP_LOGI(TAG,
                         "Dormant — nothing has changed for the whole ladder. Still watching "
                         "every %u ms; a moving view, a sound, a range change or any console "
                         "command wakes it.",
                         PLANNER_LOOP_PERIOD_MS);
                goal_state_force_stop();
            } else if (!plan_activity_dormant()) {
                /* Climbing the ladder: the interesting phase, and the one whose
                 * thresholds get tuned from a capture. Logged at INFO with every
                 * score beside its threshold, the same way the speech gates are.
                 * Dormant cycles are NOT logged at INFO — an overnight run would
                 * otherwise emit thousands of identical lines. */
                ESP_LOGI(TAG,
                         "Idle: no request | still: %u/%u | range: %u/%u cm | loud: %u/%u dB | "
                         "rung: %u/%u every %" PRIu32 " ms | why: %s",
                         plan_activity_scene_score(), (unsigned)plan_scene_threshold(),
                         plan_activity_range_score(), (unsigned)plan_range_threshold(),
                         ambient_audio_loud_score(), (unsigned)ambient_audio_loud_threshold(),
                         (unsigned)plan_activity_step() + 1u, (unsigned)PLAN_LADDER_STEPS,
                         plan_activity_period_ms(), plan_activity_verdict());
            } else {
                ESP_LOGD(TAG, "Dormant: still %u/%u | range %u/%u cm", plan_activity_scene_score(),
                         (unsigned)plan_scene_threshold(), plan_activity_range_score(),
                         (unsigned)plan_range_threshold());
            }

            camera_return_fb(fb);
            if (xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS)) ==
                pdFALSE) {
                vTaskDelay(pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
                last_wake_time = xTaskGetTickCount();
            }
            continue;
        }

        if (dormant_announced || budget_announced) {
            ESP_LOGI(TAG, "Waking — %s", plan_activity_verdict());
            dormant_announced = false;
            budget_announced = false;
        }

        /* ---- 2. Call Gemini planner ---- */
        goal_t goal = {0};
        uint32_t latency_ms = 0;
        char speech[SPEECH_TEXT_MAX] = {0};
        esp_err_t ret =
            gemini_backend_plan(fb->buf, fb->len, &goal, &latency_ms, speech, sizeof(speech));

        /* Noted whatever the outcome: the request went out, so it moves the
         * reference to the view just planned on and advances the ladder. Noting
         * only successes would let a run of failures replan the same unchanged
         * scene at full rate forever. */
        plan_activity_note_call(now_ms);

        /* Which gate opened (or held) THIS cycle, sampled here and not at the log
         * line below, because mark_spoken() re-references both detectors in
         * between. Reading them afterwards would report the state the next cycle
         * starts from and make every successful utterance look like it came from a
         * closed gate. */
        const char *const gate_verdict = planner_gate_verdict();

        /* ---- 2b. Hand any utterance to the TTS task ----
         * Posted before the goal is written and regardless of whether the
         * motion goal parsed, so the robot can speak while holding position.
         * Non-blocking: a full queue drops the line rather than stalling the
         * planner loop.
         *
         * The prompt already carries what was recently said and asks for
         * something new; this re-checks the answer, because a request is not a
         * guarantee. Only generated lines are screened — console auditions and
         * status reports go straight to the queue. See dialogue_style.h. */
        if (speech[0] != '\0' && dialogue_style_is_repetitive(speech)) {
            ESP_LOGI(TAG, "Suppressed near-repeat: \"%s\"", speech);
            speech[0] = '\0';
        }
        if (speech[0] != '\0') {
            const esp_err_t sp_ret = speech_queue_post(speech);
            if (sp_ret == ESP_OK) {
                /* Feeds the next prompt's recall list and avoid-list, and spends
                 * the speaking budget. Only on a successful post: a line dropped
                 * by a full queue is never heard, so it is neither a repetition
                 * anyone can notice nor an utterance worth rationing. */
                dialogue_style_note_spoken(speech);
                speech_budget_note((uint32_t)(esp_timer_get_time() / 1000));
                /* This view is now the one the robot has remarked on, so it
                 * becomes the reference the next frames are measured against.
                 * The soundscape is re-referenced for the same reason and at the
                 * same moment: both detectors measure "since the robot last
                 * spoke", so they must be marked together or the surviving one
                 * keeps licensing remarks about evidence already used. */
                scene_change_mark_spoken();
                ambient_audio_mark_spoken();
                ESP_LOGI(TAG, "Speech: \"%s\"", speech);
            } else if (sp_ret == ESP_ERR_NO_MEM) {
                ESP_LOGD(TAG, "still speaking — dropped: \"%s\"", speech);
            }
        }

        /* ---- 3 / 4. Write goal or force stop ---- */
        if (ret == ESP_OK) {
            esp_err_t write_ret = goal_state_write(&goal, 0 /* use default TTL */);
            if (write_ret != ESP_OK) {
                ESP_LOGW(TAG, "goal_state_write failed: %d", write_ret);
            }
            /* Every gate's live score sits beside its own threshold, so all of
             * them can be chosen from real numbers in a monitor log rather than
             * guessed — see scene_change.h, ambient_audio.h and plan_activity.h.
             *
             * scene= distance from the frame the robot last spoke about.
             * loud=  peak excursion above the adaptive noise floor since then.
             * sound= how far the room's spectral shape has moved since then.
             * floor= the adaptive floor itself, which is what explains loud=:
             *        a high floor with no excursion is a noisy but unchanging
             *        room, and is the difference between a working gate and a
             *        deaf one.
             * gate=  which branch decided whether to SPEAK (planner_gate_verdict).
             * still= distance from the frame the robot last PLANNED on, and
             *        why=/step= the request gate's own verdict and ladder rung.
             *        Distinct from scene= on purpose: the two references move at
             *        different moments, so a single field could not serve both.
             * spent= requests and tokens charged against the fuse this boot. */
            ESP_LOGI(TAG,
                     "Goal: %s | latency: %" PRIu32 " ms | scene: %u/%u | loud: %u/%u dB | "
                     "sound: %u/%u dB | floor: %d dB | gate: %s | still: %u/%u | why: %s | "
                     "step: %u/%u | spent: %" PRIu32 "/%" PRIu32 " req, %llu tok",
                     goal_kind_name(goal.kind), latency_ms, scene_change_score(),
                     (unsigned)scene_change_threshold(), ambient_audio_loud_score(),
                     (unsigned)ambient_audio_loud_threshold(), ambient_audio_shape_score(),
                     (unsigned)ambient_audio_shape_threshold(), (int)ambient_audio_floor_db(),
                     gate_verdict, plan_activity_scene_score(), (unsigned)plan_scene_threshold(),
                     plan_activity_verdict(), (unsigned)plan_activity_step() + 1u,
                     (unsigned)PLAN_LADDER_STEPS, plan_budget_requests(), budget_max_requests(),
                     (unsigned long long)plan_budget_tokens());
            backoff_periods = 0;
        } else {
            backoff_periods = (backoff_periods == 0) ? 1
                              : (backoff_periods < PLANNER_MAX_BACKOFF_PERIODS)
                                  ? backoff_periods * 2
                                  : PLANNER_MAX_BACKOFF_PERIODS;
            ESP_LOGW(TAG,
                     "gemini_backend_plan failed (%s) — forcing stop, backing off %" PRIu32
                     " extra period(s)",
                     esp_err_to_name(ret), backoff_periods);
            goal_state_force_stop();
        }

        /* ---- 5. Return framebuffer ---- */
        camera_return_fb(fb);

        /* ---- 6. Sleep until next period ----
         * A request slower than the period (a 21 s DNS timeout beats a 15 s
         * period) leaves the wake deadline in the past, and xTaskDelayUntil
         * then returns immediately — so the loop would spin at full tilt
         * exactly when the backend is already struggling. Resync on overrun
         * so the pacing holds. */
        if (xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS *
                                                           (1 + backoff_periods))) == pdFALSE) {
            vTaskDelay(pdMS_TO_TICKS(PLANNER_LOOP_PERIOD_MS));
            last_wake_time = xTaskGetTickCount();
        }
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t planner_task_init(void)
{
    if (s_planner_task_handle != NULL) {
        /* Idempotent — already running */
        return ESP_OK;
    }

    esp_err_t ret = gemini_backend_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gemini_backend_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    BaseType_t created =
        xTaskCreatePinnedToCore(planner_task, "planner", PLANNER_TASK_STACK_SIZE, NULL,
                                PLANNER_TASK_PRIORITY, &s_planner_task_handle, PLANNER_TASK_CORE);

    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create planner task");
        gemini_backend_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Planner task created (stack %u, priority %u, core %d)", PLANNER_TASK_STACK_SIZE,
             PLANNER_TASK_PRIORITY, PLANNER_TASK_CORE);
    return ESP_OK;
}

esp_err_t planner_task_stop(void)
{
    if (s_planner_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Force stop first so the executor falls back to safe hold immediately */
    goal_state_force_stop();

    vTaskDelete(s_planner_task_handle);
    s_planner_task_handle = NULL;

    gemini_backend_deinit();

    ESP_LOGI(TAG, "Planner task stopped");
    return ESP_OK;
}
