/**
 * @file console_voice.c
 * @brief Console commands for the voice path: `voice`, `mic` and `listen`.
 *
 * Moved from main.c unchanged; execute_console_line() reaches each handler
 * through the console command table (console_cmd_core.h).
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ambient_audio.h"
#include "ambient_listener.h"
#include "audio_clip.h"
#include "audio_player.h"
#include "config.h"
#include "console_handlers.h"
#include "dialogue_style.h"
#include "live_session.h"
#include "mic_dump.h"
#include "mic_pdm.h"
#include "pin_config.h"
#include "scene_change.h"
#include "speech_budget.h"
#include "speech_queue.h"
#include "speech_trigger.h"
#include "voice_budget.h"
#include "voice_fx_core.h"
#include "voice_persona.h"
#include "voice_turn.h"
#include "wifi_manager.h"

static const char *TAG = "robocar";

// ========================================
/* `voice`                      — show current persona/voice/budget and options
 * `voice <slug>`               — switch persona (e.g. `voice fi-1950`)
 * `voice say <text>`           — speak a line now, to audition the persona
 * `voice name <VoiceName>`     — override the prebuilt voice (`voice name -` clears)
 * `voice vary`                 — draw and print a variation directive (see below)
 * `voice said`                 — print what the robot remembers saying
 * `voice quiet <seconds>`      — minimum gap between utterances (0 = none)
 * `voice budget <n> <seconds>` — at most n utterances per window (n=0 mutes)
 * `voice repeat <pct>`         — similarity at which a line counts as a repeat
 * `voice scene <n>`            — how much the view must change (0 = gate off)
 * `voice loud <db>`            — excursion above the noise floor (0 = sub-gate off)
 * `voice sound <db>`           — how much the room's spectrum must move (0 = off)
 * `voice endpoint <ms> <db>`   — quiet time / speech margin that ends a VAD turn
 * `voice trigger <%> <db> <ms>` — voice-band share / floor margin / duration that starts one
 * `voice ration <n> <seconds>` — at most n hands-free voice-turn requests per window
 * `voice turns [<n>]`          — voice-turn spend; with n, the per-boot ceiling (0 = off)
 * `voice resume`               — reopen a tripped voice-turn ceiling and zero the counters
 * `voice live [on|off|stop|idle <s>|cap <s>]` — hands-free turns as Gemini Live sessions (ADR-032)
 * `mic` / `mic dump <n>`       — microphone state; dump PCM frames to the console
 * `listen [seconds]`           — one push-to-talk voice turn (record, ask, answer)
 * `trace`                      — camera + endpoint activity counters since boot
 * `trace led on|off`           — LED activity indicators (counters keep running)
 * `trace reset`                — zero the counters and clear any held fault colour
 *
 * Switching and auditioning are runtime rather than compile-time because only a
 * listener can judge a voice; a reflash per candidate is far too slow a loop.
 *
 * `voice vary` exists for the same reason one step further in: the variation
 * pools (dialogue_style.h) only pay off if their entries read naturally in the
 * persona's language, and the alternative way to see what the model is being
 * told is to wait out a 15 s planner period per sample.
 *
 * `quiet` / `budget` / `repeat` / `scene` are the same argument again: how
 * talkative and how repetitive the robot is allowed to be, and how much the view
 * must move to be worth mentioning, are judgements that need a person in the
 * room, and every trial otherwise costs a rebuild plus a flash. `scene` in
 * particular has no defensible compile-time default — what counts as a changed
 * view depends on the room and the lens, so read the `scene:` field the planner
 * logs each cycle and set the threshold from real numbers. None of the four
 * persist: a boot comes up at the documented default rather than at whatever an
 * experiment left behind. */
/**
 * @brief One line on Gemini Live sessions (ADR-032): the switch, the close
 *        knobs, and what sessions have done since boot.
 */
static void print_live_status(const char *indent)
{
    uint32_t idle_ms = 0;
    uint32_t max_ms = 0;
    uint32_t sessions = 0;
    uint32_t failures = 0;
    uint32_t turns = 0;
    uint32_t waits = 0;
    live_session_get_config(&idle_ms, &max_ms);
    live_session_stats(&sessions, &failures, &turns, &waits);
    printf("%slive:   %s%s | idle %u s, cap %u s | sessions %u (failed %u), turns %u, waits %u\n",
           indent, live_session_enabled() ? "on" : "off",
           live_session_is_open() ? " (session open)" : "", (unsigned)(idle_ms / 1000u),
           (unsigned)(max_ms / 1000u), (unsigned)sessions, (unsigned)failures, (unsigned)turns,
           (unsigned)waits);
}

/**
 * @brief Two lines on what voice turns have spent (voice_budget.h, issue #623).
 *
 * Shared by `voice`, `voice turns` and a refused `listen`, so all three say the
 * same thing. The ignored count sits beside the hands-free count on purpose:
 * the two close together is the sign that `voice trigger` is too permissive.
 * "not sent" (#949) is hands-free wakes with too little speech to upload: false
 * wakes too, but outside the request count, so read it beside both.
 */
static void print_voice_turn_budget(const char *indent, uint32_t now_ms)
{
    const uint32_t ceiling = voice_budget_ceiling();
    const uint32_t requests = voice_budget_requests();
    const uint32_t vad = voice_budget_vad_requests();
    /* Clamped: the two counters are read unlocked, and a charge landing between
     * the reads (or a resume) can leave vad > requests for one line. */
    const uint32_t listen = (requests > vad) ? (requests - vad) : 0u;
    printf("%sturns:  %u", indent, (unsigned)requests);
    if (ceiling) {
        printf("/%u", (unsigned)ceiling);
    }
    printf(" requests (hands-free %u, ignored %u, not sent %u; listen %u, ignored %u)%s\n",
           (unsigned)vad, (unsigned)voice_budget_ignored_vad(), (unsigned)voice_budget_dropped(),
           (unsigned)listen, (unsigned)voice_budget_ignored_listen(),
           ceiling ? "" : " — NO CEILING");

    uint8_t ration_max = 0;
    uint32_t ration_window_ms = 0;
    voice_budget_get_ration(&ration_max, &ration_window_ms);
    if (ration_max == 0u || ration_window_ms == 0u) {
        printf("%s        ration: off (cooldown only), refused %u\n", indent,
               (unsigned)voice_budget_refused());
    } else {
        const uint32_t wait_ms = voice_budget_ration_wait_ms(now_ms);
        printf("%s        ration: %u/%u per %us, refused %u", indent,
               (unsigned)voice_budget_ration_used(now_ms), (unsigned)ration_max,
               (unsigned)(ration_window_ms / 1000U), (unsigned)voice_budget_refused());
        if (wait_ms) {
            printf(", next in %us", (unsigned)((wait_ms + 999U) / 1000U));
        }
        printf("\n");
    }
    if (voice_budget_check(now_ms, false) == VOICE_BUDGET_TRIP_CEILING) {
        printf("%s        STOPPED: spend ceiling reached — `voice resume` to continue\n", indent);
    }
}

void handle_voice_cmd(const char *buf)
{
    char op[24] = {0};
    const int n = sscanf(buf, "voice %23s", op);

    if (n <= 0) {
        char voice[VOICE_PERSONA_VOICE_MAX];
        voice_persona_effective_voice(voice, sizeof(voice));
        const voice_persona_t *cur = voice_persona_get();
        printf("voice: persona=%s lang=%s voice=%s\n", cur->slug, cur->language_code, voice);
        for (size_t i = 0;; ++i) {
            const voice_persona_t *p = voice_persona_at(i);
            if (!p) {
                break;
            }
            printf("  %c %-12s %s\n", (p == cur) ? '*' : ' ', p->slug, p->label);
        }

        uint32_t gap_ms = 0;
        uint32_t window_ms = 0;
        uint8_t max_per = 0;
        speech_budget_get(&gap_ms, &max_per, &window_ms);
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const uint32_t wait_ms = speech_budget_wait_ms(now_ms);
        printf("  budget: quiet=%us max=%u/%us used=%u repeat=%u%%\n", (unsigned)(gap_ms / 1000U),
               (unsigned)max_per, (unsigned)(window_ms / 1000U),
               (unsigned)speech_budget_used(now_ms), (unsigned)dialogue_style_repeat_pct());
        /* "BLIND": no frame has ever decoded, so novel() fails closed and the 0
         * is "never seen anything", not a still room (issue #703; the mic line's
         * DEAF is the same word for the other sense). "no pose": the head is
         * between dwell poses, so the 0 is "cannot measure", not "unchanged"
         * (ADR-029 §2). */
        printf("  scene:  %u/%u at pose %c (%s)\n", scene_change_score(),
               (unsigned)scene_change_threshold(), scene_pose_letter(scene_change_pose()),
               !scene_change_has_measurement()          ? "BLIND — no frame decoded"
               : scene_change_threshold() == 0          ? "gate off"
               : scene_change_pose() == SCENE_POSE_NONE ? "no pose"
               : scene_change_novel()                   ? "new"
                                                        : "same");
        printf("  loud:   %u/%u dB (floor %d dB)\n", ambient_audio_loud_score(now_ms),
               (unsigned)ambient_audio_loud_threshold(), (int)ambient_audio_floor_db());
        printf("  sound:  %u/%u dB (%s)\n", ambient_audio_shape_score(now_ms),
               (unsigned)ambient_audio_shape_threshold(),
               !ambient_audio_has_measurement() ? "DEAF — nothing ever heard"
               : ambient_audio_novel(now_ms)    ? "new"
                                                : "same");
        uint32_t ep_silence = 0;
        uint32_t ep_min = 0;
        uint32_t ep_max = 0;
        uint8_t ep_margin = 0;
        voice_turn_get_endpoint(&ep_silence, &ep_margin, &ep_min, &ep_max);
        printf("  vad:    %s | endpoint: quiet %u ms at %u dB over floor, min %u ms, max %u ms\n",
               voice_turn_get_vad() ? "on" : "off", (unsigned)ep_silence, (unsigned)ep_margin,
               (unsigned)ep_min, (unsigned)ep_max);
        printf("          trigger: voice band >= %u%%, +%u dB over floor, for %u ms\n",
               (unsigned)speech_trigger_share_pct(), (unsigned)speech_trigger_margin_db(),
               (unsigned)speech_trigger_sustain_ms());
        print_voice_turn_budget("  ", now_ms);
        print_live_status("  ");
        /* Every gate is reported, and which one is holding, because a silent robot
         * is otherwise indistinguishable from a broken one — and on a static,
         * quiet scene silence is the correct behaviour. The evidence line names
         * BOTH senses now: after the OR, "waiting on the view" would be a lie
         * whenever the room could still open the gate by sounding different. */
        if (wait_ms == UINT32_MAX) {
            printf("  speech: muted\n");
        } else if (wait_ms) {
            printf("  speech: waiting on budget (%us)\n", (unsigned)(wait_ms / 1000U));
        } else if (!scene_change_novel() && !ambient_audio_novel(now_ms)) {
            /* A blind camera has no view to change; only the room can open it. */
            printf("  speech: %s\n", scene_change_has_measurement()
                                         ? "waiting on the view or the room to change"
                                         : "waiting on the room to change (camera BLIND)");
        } else {
            printf("  speech: allowed now\n");
        }

        printf("  volume: %u%% of full scale (boot default %u%%)\n",
               (unsigned)audio_player_volume_pct(), (unsigned)AUDIO_VOLUME_PCT);
        printf("  usage: voice <slug> | say <text> | name <VoiceName|-> | vary | said\n");
        printf("         voice quiet <s> | budget <n> <s> | repeat <pct> | scene <n>\n");
        printf("         voice loud <db> | sound <db> | vad on|off   (see also: mic)\n");
        printf(
            "         voice endpoint <quiet_ms> <db>   when a hands-free turn stops recording\n");
        printf("         voice trigger <band%%> <db> <ms>  what starts one (see `mic`)\n");
        printf("         voice ration <n> <s> | turns <n> | resume   what voice turns may spend\n");
        printf("         voice live on|off|stop | idle <s> | cap <s>   Gemini Live sessions\n");
        printf("         voice volume <pct>   amplitude, not loudness: halving = -6 dB\n");
        printf("         voice fx [on|off|body <10..200>|metal <0..85>|drive <10..400>]\n");
        printf(
            "                              retro-robot body resonance; body is TENTHS of a ms\n");
        return;
    }

    if (strcmp(op, "volume") == 0) {
        unsigned pct = 0;
        if (sscanf(buf, "voice volume %u", &pct) != 1 || pct > AUDIO_VOLUME_PCT_MAX) {
            printf("voice: usage: voice volume <0..%u>  (now %u%%)\n",
                   (unsigned)AUDIO_VOLUME_PCT_MAX, (unsigned)audio_player_volume_pct());
            return;
        }
        audio_player_set_volume_pct((uint8_t)pct);
        /* Reported as a ratio against the boot default rather than in dB: the
         * dB would need log10() and a %f, and a float print is silently wrong
         * if CONFIG_NEWLIB_NANO_FORMAT is ever turned on. The conversion worth
         * remembering is fixed anyway and lives in the usage line — halving the
         * percent is -6 dB, and an ear reads about -10 dB as half as loud. */
        printf("voice: volume=%u%% of full scale (boot default %u%%, so %u/100 of it)\n",
               (unsigned)audio_player_volume_pct(), (unsigned)AUDIO_VOLUME_PCT,
               (unsigned)((audio_player_volume_pct() * 100U) / AUDIO_VOLUME_PCT));
        return;
    }

    /* The retro-robot body resonance. Every parameter is an INTEGER on the
     * console even though the effect is float internally: printing a float
     * needs %f, and %f is silently wrong under CONFIG_NEWLIB_NANO_FORMAT — the
     * same reason `voice volume` reports a ratio rather than dB. Body is in
     * tenths of a millisecond, the other two in percent.
     *
     * Not persisted, like every other knob here: a boot comes up at the
     * documented default rather than at last night's experiment. */
    if (strcmp(op, "fx") == 0) {
        voice_fx_t *fx = audio_player_fx();
        char sub[16] = {0};
        unsigned value = 0;
        const int got = sscanf(buf, "voice fx %15s %u", sub, &value);

        if (got >= 1 && strcmp(sub, "on") == 0) {
            voice_fx_set_enabled(fx, true);
        } else if (got >= 1 && strcmp(sub, "off") == 0) {
            voice_fx_set_enabled(fx, false);
        } else if (got == 2 && strcmp(sub, "body") == 0) {
            if (!voice_fx_set_body_ms(fx, (float)value / 10.0f)) {
                printf("voice: fx body out of range (10..200 tenths of a ms)\n");
                return;
            }
        } else if (got == 2 && strcmp(sub, "metal") == 0) {
            if (!voice_fx_set_feedback(fx, (float)value / 100.0f)) {
                printf("voice: fx metal out of range (0..85 %%)\n");
                return;
            }
        } else if (got == 2 && strcmp(sub, "drive") == 0) {
            if (!voice_fx_set_drive(fx, (float)value / 100.0f)) {
                printf("voice: fx drive out of range (10..400 %%)\n");
                return;
            }
        } else if (got >= 1) {
            printf(
                "voice: usage: voice fx [on|off|body <10..200>|metal <0..85>|drive <10..400>]\n");
            return;
        }

        /* Report unconditionally, including after a bare `voice fx`, so the
         * command doubles as the read-back. The resonance frequency is what
         * `body` actually means to an ear, so it is printed beside it. */
        const unsigned body_tenths = (unsigned)(fx->body_ms * 10.0f + 0.5f);
        const unsigned resonance_hz = body_tenths ? (unsigned)(10000u / body_tenths) : 0u;
        printf("voice: fx=%s body=%u.%u ms (~%u Hz, %u samples) metal=%u%% drive=%u%%\n",
               fx->enabled ? "on" : "off", body_tenths / 10u, body_tenths % 10u, resonance_hz,
               (unsigned)fx->delay_samples, (unsigned)(fx->feedback * 100.0f + 0.5f),
               (unsigned)(fx->drive * 100.0f + 0.5f));
        return;
    }

    if (strcmp(op, "scene") == 0) {
        unsigned threshold = 0;
        if (sscanf(buf, "voice scene %u", &threshold) != 1 || threshold > UINT8_MAX) {
            printf("voice: usage: voice scene <0..255>  (0 = gate off)\n");
            return;
        }
        scene_change_set_threshold((uint8_t)threshold);
        printf("voice: scene=%u (now %u)\n", (unsigned)scene_change_threshold(),
               scene_change_score());
        return;
    }

    /* NOTE the inverted zero convention against `voice scene` above, and it is not
     * a slip: scene is ANDed into the evidence term, so its 0 means "always
     * novel"; loud and sound are ORed, so their 0 must mean "never contributes".
     * Both readings say "this gate is not participating" — the polarity just
     * follows the operator. Setting BOTH loud and sound to 0 restores exactly the
     * pre-microphone behaviour, which ambient_audio's host tests pin. */
    if (strcmp(op, "loud") == 0) {
        unsigned db = 0;
        if (sscanf(buf, "voice loud %u", &db) != 1 || db > UINT8_MAX) {
            printf("voice: usage: voice loud <0..255> dB  (0 = sub-gate off)\n");
            return;
        }
        ambient_audio_set_loud_threshold((uint8_t)db);
        printf("voice: loud=%u dB (now %u dB, floor %d dB)\n",
               (unsigned)ambient_audio_loud_threshold(),
               ambient_audio_loud_score((uint32_t)(esp_timer_get_time() / 1000)),
               (int)ambient_audio_floor_db());
        return;
    }

    if (strcmp(op, "sound") == 0) {
        unsigned db = 0;
        if (sscanf(buf, "voice sound %u", &db) != 1 || db > UINT8_MAX) {
            printf("voice: usage: voice sound <0..255> dB  (0 = sub-gate off)\n");
            return;
        }
        ambient_audio_set_shape_threshold((uint8_t)db);
        printf("voice: sound=%u dB (now %u dB)\n", (unsigned)ambient_audio_shape_threshold(),
               ambient_audio_shape_score((uint32_t)(esp_timer_get_time() / 1000)));
        return;
    }

    if (strcmp(op, "said") == 0) {
        char lines[DIALOGUE_RECALL_SLOTS * (DIALOGUE_RECALL_MAX + 4)];
        printf("voice: said %s\n",
               dialogue_style_recent_lines(lines, sizeof(lines)) ? lines : "(nothing yet)");
        return;
    }

    if (strcmp(op, "quiet") == 0) {
        unsigned secs = 0;
        if (sscanf(buf, "voice quiet %u", &secs) != 1) {
            printf("voice: usage: voice quiet <seconds>\n");
            return;
        }
        uint32_t window_ms = 0;
        uint8_t max_per = 0;
        speech_budget_get(NULL, &max_per, &window_ms);
        speech_budget_configure(secs * 1000U, max_per, window_ms);
        printf("voice: quiet=%us\n", secs);
        return;
    }

    if (strcmp(op, "budget") == 0) {
        unsigned max_per = 0;
        unsigned secs = 0;
        if (sscanf(buf, "voice budget %u %u", &max_per, &secs) != 2 || max_per > UINT8_MAX) {
            printf("voice: usage: voice budget <n> <window seconds>  (n=0 mutes)\n");
            return;
        }
        uint32_t gap_ms = 0;
        speech_budget_get(&gap_ms, NULL, NULL);
        speech_budget_configure(gap_ms, (uint8_t)max_per, secs * 1000U);
        /* Read back rather than echo the argument: the cap is clamped to the
         * history ring, and a silently ignored value is worse than none. */
        uint8_t applied = 0;
        speech_budget_get(NULL, &applied, NULL);
        printf("voice: budget=%u/%us\n", (unsigned)applied, secs);
        return;
    }

    if (strcmp(op, "repeat") == 0) {
        unsigned pct = 0;
        if (sscanf(buf, "voice repeat %u", &pct) != 1 || pct < 1U || pct > 100U) {
            printf("voice: usage: voice repeat <1..100>  (percent word overlap)\n");
            return;
        }
        dialogue_style_set_repeat_pct((uint8_t)pct);
        printf("voice: repeat=%u%%\n", (unsigned)dialogue_style_repeat_pct());
        return;
    }

    if (strcmp(op, "vary") == 0) {
        const voice_persona_t *cur = voice_persona_get();
        char directive[DIALOGUE_STYLE_MAX];
        char recent[DIALOGUE_RECENT_SLOTS * (DIALOGUE_OPENING_MAX + 4)];

        /* The avoid-list is printed on its own line as well as inside the
         * directive: it is the only part carried over from what was really
         * spoken, so it is worth seeing even when the persona has no pools. */
        printf("voice: avoiding %s\n",
               dialogue_style_recent_openings(recent, sizeof(recent)) ? recent : "(nothing yet)");
        if (dialogue_style_directive(&cur->openers, &cur->shapes, cur->avoid_lead, directive,
                                     sizeof(directive)) > 0) {
            printf("voice: vary \"%s\"\n", directive);
        } else {
            printf("voice: persona '%s' has no variation pools\n", cur->slug);
        }
        return;
    }

    if (strcmp(op, "say") == 0) {
        const char *text = strstr(buf, "say");
        text = text ? text + 3 : NULL;
        while (text && *text == ' ') {
            ++text;
        }
        if (!text || *text == '\0') {
            printf("voice: usage: voice say <text>\n");
            return;
        }
        printf("voice: speaking \"%s\"\n", text);
        if (speech_queue_post(text) != ESP_OK) {
            printf("voice: speech queue busy\n");
        }
        return;
    }

    if (strcmp(op, "name") == 0) {
        char name[VOICE_PERSONA_VOICE_MAX] = {0};
        if (sscanf(buf, "voice name %23s", name) != 1) {
            printf("voice: usage: voice name <VoiceName|->\n");
            return;
        }
        const bool clear = (strcmp(name, "-") == 0);
        if (voice_persona_set_voice(clear ? NULL : name, true) == ESP_OK) {
            printf("voice: voice=%s\n", clear ? "(persona default)" : name);
        } else {
            printf("voice: could not set voice\n");
        }
        return;
    }

    if (strcmp(op, "live") == 0) {
        /* Gemini Live sessions in place of hands-free turns (ADR-032 phase 1).
         * Off at boot and not persisted, like every voice switch. */
        char sub[8] = {0};
        unsigned secs = 0;
        const int got = sscanf(buf, "voice live %7s %u", sub, &secs);
        if (got >= 1 && strcmp(sub, "on") == 0) {
            live_session_set_enabled(true);
        } else if (got >= 1 && strcmp(sub, "off") == 0) {
            live_session_set_enabled(false);
            live_session_request_stop();
        } else if (got >= 1 && strcmp(sub, "stop") == 0) {
            live_session_request_stop();
        } else if (got == 2 && strcmp(sub, "idle") == 0 && secs >= 2 && secs <= 120) {
            live_session_configure(secs * 1000u, 0);
        } else if (got == 2 && strcmp(sub, "cap") == 0 && secs >= 30 && secs <= 540) {
            /* 540 s keeps a session inside the ~10 min connection limit. */
            live_session_configure(0, secs * 1000u);
        } else if (got >= 1) {
            printf("voice: usage: voice live on|off|stop | idle <2..120 s> | cap <30..540 s>\n");
            return;
        }
        print_live_status("voice: ");
        if (got >= 1) {
            ESP_LOGI(TAG, "voice: live %s", sub);
        }
        return;
    }

    if (strcmp(op, "vad") == 0) {
        char state[8] = {0};
        if (sscanf(buf, "voice vad %7s", state) != 1 ||
            (strcmp(state, "on") != 0 && strcmp(state, "off") != 0)) {
            printf("voice: usage: voice vad on|off  (now %s)\n",
                   voice_turn_get_vad() ? "on" : "off");
            return;
        }
        const bool enable = (strcmp(state, "on") == 0);
        voice_turn_set_vad(enable);
        printf("voice: vad=%s\n", enable ? "on" : "off");
        ESP_LOGI(TAG, "voice: vad=%s", enable ? "on" : "off");
        return;
    }

    if (strcmp(op, "endpoint") == 0) {
        /* Not persisted, like every other voice knob. Whether 700 ms cuts people
         * off mid-thought or leaves the robot waiting is a judgement that needs
         * someone talking to it; the per-turn `listen: vad clip=` line is what
         * to tune against. */
        unsigned quiet_ms = 0;
        unsigned margin_db = 0;
        if (sscanf(buf, "voice endpoint %u %u", &quiet_ms, &margin_db) != 2 || quiet_ms < 100 ||
            quiet_ms > 5000 || margin_db > 40) {
            uint32_t cur_quiet = 0;
            uint8_t cur_margin = 0;
            voice_turn_get_endpoint(&cur_quiet, &cur_margin, NULL, NULL);
            printf("voice: usage: voice endpoint <100..5000 ms> <0..40 dB>  (now %u ms, %u dB;"
                   " 0 dB records to the ceiling)\n",
                   (unsigned)cur_quiet, (unsigned)cur_margin);
            return;
        }
        voice_turn_set_endpoint((uint32_t)quiet_ms, (uint8_t)margin_db);
        printf("voice: endpoint quiet=%u ms margin=%u dB\n", quiet_ms, margin_db);
        return;
    }

    if (strcmp(op, "trigger") == 0) {
        /* Not persisted. Tune by speaking and reading `mic`: its speech line
         * shows the highest voice-band share and longest run since the last
         * `mic`, so a band threshold set too high shows up as a near miss. The
         * peak band counts only frames that cleared the floor margin, so a
         * margin set too high reads as band 0%. */
        unsigned share = 0;
        unsigned margin_db = 0;
        unsigned sustain_ms = 0;
        if (sscanf(buf, "voice trigger %u %u %u", &share, &margin_db, &sustain_ms) != 3 ||
            share > 100 || margin_db > 40 || sustain_ms > 2000) {
            printf("voice: usage: voice trigger <0..100 %%> <0..40 dB> <0..2000 ms>  (now %u%%, "
                   "%u dB, %u ms; 0 drops a check)\n",
                   (unsigned)speech_trigger_share_pct(), (unsigned)speech_trigger_margin_db(),
                   (unsigned)speech_trigger_sustain_ms());
            return;
        }
        speech_trigger_set_share_pct((uint8_t)share);
        speech_trigger_set_margin_db((uint8_t)margin_db);
        speech_trigger_set_sustain_ms((uint32_t)sustain_ms);
        printf("voice: trigger band=%u%% margin=%u dB sustain=%u ms\n", share, margin_db,
               sustain_ms);
        return;
    }

    /* What voice turns may spend (voice_budget.h, issue #623). Not persisted,
     * like every other knob here — a boot comes up at the documented defaults —
     * and `resume` is the only thing short of a reboot that reopens a tripped
     * ceiling, because a fuse that reopens itself is no use on an unattended
     * board. */
    if (strcmp(op, "ration") == 0) {
        unsigned max_per = 0;
        unsigned secs = 0;
        if (sscanf(buf, "voice ration %u %u", &max_per, &secs) != 2 || max_per > UINT8_MAX ||
            secs > 86400U) {
            printf("voice: usage: voice ration <n> <window seconds>  (hands-free requests per "
                   "window; n=0 or 0 s removes the ration)\n");
            return;
        }
        voice_budget_configure_ration((uint8_t)max_per, secs * 1000U);
        /* Read back rather than echo: the cap is clamped to the history ring. */
        uint8_t applied = 0;
        voice_budget_get_ration(&applied, NULL);
        printf("voice: ration=%u/%us\n", (unsigned)applied, secs);
        return;
    }

    if (strcmp(op, "turns") == 0) {
        unsigned max_requests = 0;
        if (sscanf(buf, "voice turns %u", &max_requests) == 1) {
            voice_budget_configure_ceiling((uint32_t)max_requests);
            printf("voice: turns ceiling=%u per boot%s\n", max_requests,
                   max_requests ? "" : " (disabled)");
            ESP_LOGI(TAG, "voice: turns ceiling=%u", max_requests);
        }
        print_voice_turn_budget("voice: ", (uint32_t)(esp_timer_get_time() / 1000));
        return;
    }

    if (strcmp(op, "resume") == 0) {
        voice_budget_resume();
        printf("voice: voice-turn budget cleared — counters zeroed, ceiling reopened\n");
        ESP_LOGI(TAG, "voice: turns resumed");
        return;
    }

    if (voice_persona_set(op, true) == ESP_OK) {
        voice_turn_reset_history();
        printf("voice: persona=%s\n", op);
    } else {
        printf("voice: unknown persona '%s'\n", op);
    }
}

/**
 * @brief `mic` / `mic dump <n>` — the microphone's counterpart to `cam`.
 *
 * Bare `mic` is how a dead microphone is told apart from a quiet room, which is
 * the single most useful thing this command does: both produce a silent robot,
 * and only one of them is a fault. A live floor that moves when you clap means
 * the mic works; a floor frozen at one value with frames still accumulating means
 * it does not.
 *
 * `mic dump` is the `snap` of the audio path, and exists for the same reason
 * frame_dump.c does: every claim about the camera was inference until somebody
 * looked at a frame, and the first one dumped disproved the sensor named in the
 * header. Nobody has listened to this microphone either.
 */

void handle_mic_cmd(const char *buf)
{
    int frames = 0;
    if (sscanf(buf, "mic dump %d", &frames) == 1 || strncmp(buf, "mic dump", 8) == 0) {
        if (frames <= 0) {
            frames = (strncmp(buf, "mic dump", 8) == 0 && frames == 0) ? 1 : 0;
        }
        mic_dump_arm(frames);
        if (frames == 0) {
            printf("mic: dumping disarmed\n");
        } else {
            printf("mic: will dump the next %d frame(s) — decode with "
                   "tools/decode-mic-dump.py\n",
                   frames);
        }
        return;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint32_t last = ambient_listener_last_accept_ms();

    printf("mic: driver=%s listener=%s rate=%u Hz\n", mic_pdm_is_ready() ? "ready" : "NOT READY",
           ambient_listener_is_running() ? "running" : "STOPPED", (unsigned)MIC_SAMPLE_RATE_HZ);
    printf("  level:  %d dB      floor: %d dB   (dB above 1 LSB; full scale ~90)\n",
           (int)ambient_listener_level_db(), (int)ambient_audio_floor_db());
    if (!ambient_audio_has_measurement()) {
        printf("  gate:   DEAF — no measurable frame has ever reached it, so it reports "
               "no novelty\n");
    }
    printf("  loud:   %u/%u dB    sound: %u/%u dB\n", ambient_audio_loud_score(now_ms),
           (unsigned)ambient_audio_loud_threshold(), ambient_audio_shape_score(now_ms),
           (unsigned)ambient_audio_shape_threshold());
    /* The hands-free trigger's evidence. "best" is since the previous `mic`, and
     * reading it clears it: speak once, then type `mic`, and the line says how
     * close the voice came to each threshold. */
    uint32_t best_run = 0;
    uint8_t peak_share = 0;
    speech_trigger_best(&best_run, &peak_share, true);
    printf("  speech: now band %u%% run %u ms | best since last mic: band %u%% run %u ms | "
           "need %u%% +%u dB %u ms (vad %s)\n",
           (unsigned)speech_trigger_last_share_pct(), (unsigned)speech_trigger_run_ms(),
           (unsigned)peak_share, (unsigned)best_run, (unsigned)speech_trigger_share_pct(),
           (unsigned)speech_trigger_margin_db(), (unsigned)speech_trigger_sustain_ms(),
           voice_turn_get_vad() ? "on" : "off");
    printf("  frames: %u accepted, %u muted by playback/cue, %u muted by head servos\n",
           (unsigned)ambient_listener_frames_accepted(), (unsigned)ambient_listener_frames_muted(),
           (unsigned)ambient_listener_frames_head_muted());
    if (last == 0U) {
        printf("  last:   never — no frame has reached the gate\n");
    } else {
        printf("  last:   %u ms ago\n", (unsigned)(now_ms - last));
    }
    printf("  usage: mic | mic dump <n>   (thresholds: voice loud <db> | voice sound <db> | "
           "voice trigger)\n");
}

/**
 * @brief `listen [seconds]` — one push-to-talk voice turn.
 *
 * Records, asks Gemini, and speaks the reply through the existing TTS path. The
 * window is a knob rather than a constant because "is four seconds long enough
 * to finish a sentence" is a judgement only somebody talking to the robot can
 * make, and a reflash per trial is far too slow a loop.
 */
void handle_listen_cmd(const char *buf)
{
    if (strcmp(buf, "listen clear") == 0) {
        voice_turn_reset_history();
        printf("listen: conversational history cleared\n");
        return;
    }

    unsigned secs = 0;
    uint32_t window_ms = AUDIO_CLIP_WINDOW_MS_DEFAULT;
    if (sscanf(buf, "listen %u", &secs) == 1) {
        if (secs < AUDIO_CLIP_WINDOW_S_MIN || secs > AUDIO_CLIP_WINDOW_S_MAX) {
            printf("listen: usage: listen [%d..%d seconds] | listen clear\n",
                   AUDIO_CLIP_WINDOW_S_MIN, AUDIO_CLIP_WINDOW_S_MAX);
            return;
        }
        window_ms = (uint32_t)secs * 1000U;
    }

    const esp_err_t ret = voice_turn_request(window_ms);
    switch (ret) {
        case ESP_OK:
            printf("listen: recording %u ms after the beep — speak now\n", (unsigned)window_ms);
            break;
        case ESP_ERR_NO_MEM:
            printf("listen: busy — a turn is already in flight\n");
            break;
        case ESP_ERR_NOT_ALLOWED:
            /* `listen` is exempt from the hands-free ration, so this is always
             * the ceiling. */
            printf("listen: voice-turn spend ceiling reached — `voice resume` to continue\n");
            print_voice_turn_budget("  ", (uint32_t)(esp_timer_get_time() / 1000));
            break;
        case ESP_ERR_INVALID_STATE:
            /* Three quite different causes, and telling them apart matters:
             * a robot mid-sentence is working correctly, a missing microphone is
             * a fault, and an unstarted task means the AI phase did not come up. */
            if (!wifi_is_connected()) {
                printf("listen: no WiFi link — voice turns wait for the network\n");
            } else if (!mic_pdm_is_ready()) {
                printf("listen: no microphone — voice turns unavailable\n");
            } else if (audio_player_is_active()) {
                printf("listen: the robot is speaking — try again in a moment\n");
            } else {
                printf("listen: voice turns are not running\n");
            }
            break;
        default:
            printf("listen: rejected (%s)\n", esp_err_to_name(ret));
            break;
    }
}
