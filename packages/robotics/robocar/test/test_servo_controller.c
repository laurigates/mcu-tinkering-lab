/**
 * @file test_servo_controller.c
 *
 * Pins servo_controller against the bus it drives.
 *
 * Init centres both servos via servo_set_angle(), and servo_set_angle()
 * refuses with ESP_ERR_INVALID_STATE until the module is initialised. The
 * flag used to be raised only AFTER centring, so init could never succeed:
 * it returned ESP_ERR_INVALID_STATE without a single bus write, and the
 * ESP_ERROR_CHECK around init_hardware() rebooted the board. A bare board
 * never reached the call (the hardware phase is skipped when I2C init
 * fails), which is why it survived until the PCA9685 was first fitted.
 *
 * The later cases pin the 2026-09 bench incident, where both servos were held
 * squealing against their end stops: a frequency change replayed counts from
 * the old period, the angle maths stretched tilt's ±45 deg across the servo's
 * full travel, and pan's +90 deg asked for 2500 us, past the SG90's 2400 us.
 *
 * servo_controller.c is compiled unmodified; the i2c_bus it writes through is
 * the recording stub at the bottom of this file.
 */

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "i2c_bus.h"
#include "pin_config.h"
#include "servo_controller.h"

/* servo_move_smooth() and servo_exercise() sleep between steps. */
void vTaskDelay(TickType_t ticks)
{
    (void)ticks;
}

/* servo_last_move_ms() reads esp_timer; the tests drive it. */
static int64_t s_now_us;

int64_t esp_timer_get_time(void)
{
    return s_now_us;
}

/* ---- recording i2c_bus stub --------------------------------------------- */

/* One ordered log of everything the module does to the chip, so a test can
 * assert not just what was written but in which order relative to a frequency
 * change. */
typedef enum { EV_WRITE, EV_FREQ } event_kind_t;

static struct {
    event_kind_t kind;
    uint8_t channel;
    uint16_t value; /* count for EV_WRITE, Hz for EV_FREQ */
} s_events[32];
static int s_event_count;
static esp_err_t s_next_result = ESP_OK;

/* The chip-wide prescaler the pulse maths reads. Settable so the tests can ask
 * what a given angle becomes at a frequency other than the shipped 200 Hz. */
static uint16_t s_pwm_hz = 200u;

static void record(event_kind_t kind, uint8_t channel, uint16_t value)
{
    if (s_event_count < (int)(sizeof(s_events) / sizeof(s_events[0]))) {
        s_events[s_event_count].kind = kind;
        s_events[s_event_count].channel = channel;
        s_events[s_event_count].value = value;
    }
    s_event_count++;
}

uint16_t i2c_bus_pca9685_frequency(void)
{
    return s_pwm_hz;
}

esp_err_t i2c_bus_pca9685_set_frequency(uint16_t hz)
{
    record(EV_FREQ, 0, hz);
    if (s_next_result == ESP_OK) {
        s_pwm_hz = hz;
    }
    return s_next_result;
}

/* ---- interleaving gate (issue #1058) ------------------------------------
 *
 * The executor and the console task both write the servos. To pin how a write
 * and a release interleave, the stub can park the next write to one channel,
 * either before it reaches the bus (the writer has passed its checks and been
 * preempted) or after (the transaction landed, the state update has not). The
 * test then runs the other writer on a second thread and opens the gate once
 * that writer has either finished or blocked on the module's lock. The lock is
 * a real pthread mutex (the semaphore shim below), so a writer that should wait
 * does wait, and the order the stub records is the order the bus would see. */

static pthread_mutex_t s_gate_m = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_gate_cv = PTHREAD_COND_INITIALIZER;
static int s_gate_channel = -1; /* armed when >= 0; disarms on first match */
static bool s_gate_after;       /* park after recording the write */
static bool s_gate_parked;
static bool s_gate_open;
static bool s_lock_contended; /* a take found the module's lock held */

static void park_locked(void)
{
    s_gate_parked = true;
    pthread_cond_broadcast(&s_gate_cv);
    while (!s_gate_open) {
        pthread_cond_wait(&s_gate_cv, &s_gate_m);
    }
}

esp_err_t i2c_bus_pca9685_set(uint8_t channel, uint16_t count)
{
    pthread_mutex_lock(&s_gate_m);
    const bool park = (s_gate_channel == (int)channel);
    if (park) {
        s_gate_channel = -1;
    }
    if (park && !s_gate_after) {
        park_locked();
    }
    record(EV_WRITE, channel, count);
    if (park && s_gate_after) {
        park_locked();
    }
    const esp_err_t ret = s_next_result;
    pthread_mutex_unlock(&s_gate_m);
    return ret;
}

/* Semaphore shim backed by a pthread mutex, so contention is real. A take that
 * finds the mutex held says so before it blocks, which is how the test knows
 * the second writer is waiting rather than finished. The timeout is not
 * modelled: no case here holds the lock for long. */
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    pthread_mutex_t *m = calloc(1, sizeof(*m));
    if (m != NULL) {
        pthread_mutex_init(m, NULL);
    }
    return m;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait)
{
    (void)wait;
    pthread_mutex_t *m = sem;
    if (pthread_mutex_trylock(m) == 0) {
        return pdTRUE;
    }
    pthread_mutex_lock(&s_gate_m);
    s_lock_contended = true;
    pthread_cond_broadcast(&s_gate_cv);
    pthread_mutex_unlock(&s_gate_m);
    pthread_mutex_lock(m);
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    pthread_mutex_unlock((pthread_mutex_t *)sem);
    return pdTRUE;
}

static void reset_bus(void)
{
    s_event_count = 0;
    s_next_result = ESP_OK;
}

/* The expected count for a pulse, computed independently of the module. */
static uint16_t count_for_pulse(uint32_t pulse_us, uint16_t hz)
{
    const uint32_t count = (pulse_us * 4096u) / (1000000u / hz);
    return (uint16_t)(count > 4095u ? 4095u : count);
}

/* ---- harness ------------------------------------------------------------ */

static int s_failures;

#define CHECK(cond, ...)                                           \
    do {                                                           \
        if (!(cond)) {                                             \
            s_failures++;                                          \
            fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                          \
            fputc('\n', stderr);                                   \
        }                                                          \
    } while (0)

#define CHECK_WRITE(i, ch, count)                                                             \
    CHECK(s_events[i].kind == EV_WRITE && s_events[i].channel == (ch) &&                      \
              s_events[i].value == (count),                                                   \
          "event %d: expected write ch%u=%u, got kind=%d ch%u value=%u", (i), (unsigned)(ch), \
          (unsigned)(count), (int)s_events[i].kind, (unsigned)s_events[i].channel,            \
          (unsigned)s_events[i].value)

/* The module keeps its state in a file-static struct with no reset hook, so
 * the tests run in one process in a fixed order, each leaving the state the
 * next one documents. */

static void test_failed_centring_leaves_module_uninitialised(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_FAIL, "init returned %d, expected the bus error", err);
    CHECK(s_event_count == 1, "expected the first centring write, got %d", s_event_count);
    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_CENTER) == ESP_ERR_INVALID_STATE,
          "a failed init must not leave the servos reported as ready");
}

static void test_init_centres_both_servos(void)
{
    reset_bus();

    esp_err_t err = servo_controller_init();

    CHECK(err == ESP_OK, "init returned %d", err);
    CHECK(s_event_count == 2, "expected two centring writes, got %d", s_event_count);
    CHECK(s_events[0].channel == SERVO_PAN_CHANNEL, "first write to channel %u",
          s_events[0].channel);
    CHECK(s_events[1].channel == SERVO_TILT_CHANNEL, "second write to channel %u",
          s_events[1].channel);
    CHECK(s_events[0].value > 0 && s_events[1].value > 0, "centre pulse must be non-zero");

    int16_t pan = -1, tilt = -1;
    CHECK(servo_get_angle(SERVO_PAN, &pan) == ESP_OK && pan == SERVO_PAN_CENTER,
          "pan reads %d after init", pan);
    CHECK(servo_get_angle(SERVO_TILT, &tilt) == ESP_OK && tilt == SERVO_TILT_CENTER,
          "tilt reads %d after init", tilt);
}

static void test_set_angle_writes_after_init(void)
{
    reset_bus();

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_CENTER + 10) == ESP_OK, "set_angle after init");
    CHECK(s_event_count == 1 && s_events[0].channel == SERVO_PAN_CHANNEL,
          "set_angle must write the pan channel once");
}

/* The centre pulse is 1500 us (SERVO_CENTER_PULSE_US), and a PCA9685 count is
 * that width as a fraction of the period over 4096 steps. So a centred servo's
 * count is entirely determined by the frequency:
 *
 *   200 Hz ->  5000 us period -> 1500 * 4096 /  5000 = 1228
 *    50 Hz -> 20000 us period -> 1500 * 4096 / 20000 =  307
 *
 * The maths used to divide by a hardcoded 5000, so both came out 1228 — and at
 * 50 Hz that count is a 6 ms pulse, several times longer than any servo range
 * accepts. Nothing caught it because the frequency could not be changed. */
static void test_the_count_tracks_the_pwm_frequency(void)
{
    s_pwm_hz = 200u;
    CHECK(servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER) == 1228u,
          "centre at 200 Hz should be count 1228, got %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER));

    s_pwm_hz = 50u;
    CHECK(servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER) == 307u,
          "centre at 50 Hz should be count 307, got %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, SERVO_PAN_CENTER));

    /* Off-centre angles scale with it too, rather than staying put. */
    s_pwm_hz = 200u;
    const uint16_t at_200 = servo_angle_to_count(SERVO_PAN, 45);
    s_pwm_hz = 50u;
    const uint16_t at_50 = servo_angle_to_count(SERVO_PAN, 45);
    CHECK(at_50 < at_200, "a lower frequency must need a smaller count: %u vs %u", (unsigned)at_50,
          (unsigned)at_200);

    s_pwm_hz = 200u;
}

/* A pulse cannot outlast its period. At the driver's 1526 Hz ceiling even a
 * modest pulse is longer than the 655 us frame, and an unclamped count would
 * wrap in the 12-bit register and emerge as a SHORT pulse — a servo slamming to
 * the opposite endpoint rather than refusing. */
static void test_an_impossible_pulse_is_clamped(void)
{
    s_pwm_hz = 1526u;
    const uint16_t count = servo_angle_to_count(SERVO_TILT, 30);
    CHECK(count == 4095u, "an over-long pulse must clamp to 4095, got %u", (unsigned)count);
    s_pwm_hz = 200u;
}

/* An angle is a real servo angle: 1000 us per 90 deg about 1500 us, clamped to
 * the SG90's 500-2400 us. The old maths stretched each servo's configured range
 * across 500-2500 us, so tilt's "+45" was really +90 and pan's +90 was 2500 us. */
static void test_an_angle_is_a_real_servo_angle_within_the_sg90_range(void)
{
    s_pwm_hz = 200u;
    CHECK(servo_angle_to_count(SERVO_TILT, 45) == count_for_pulse(2000u, 200u),
          "tilt +45 must be a 2000 us pulse, got count %u",
          (unsigned)servo_angle_to_count(SERVO_TILT, 45));
    /* Tilt, because it is not mirrored: pan's sign is pinned separately. */
    CHECK(servo_angle_to_count(SERVO_TILT, -90) == count_for_pulse(500u, 200u),
          "-90 must be the 500 us minimum, got count %u",
          (unsigned)servo_angle_to_count(SERVO_TILT, -90));
    CHECK(servo_angle_to_count(SERVO_TILT, 90) == count_for_pulse(2400u, 200u),
          "+90 must clamp to the 2400 us maximum, got count %u",
          (unsigned)servo_angle_to_count(SERVO_TILT, 90));
    CHECK(servo_angle_to_count(SERVO_TILT, -120) == count_for_pulse(500u, 200u),
          "an angle below the range must clamp to 500 us");

    int16_t hw_lo = 0, hw_hi = 0;
    servo_get_hw_range(&hw_lo, &hw_hi);
    CHECK(hw_lo == -90 && hw_hi == 81, "hardware range should be -90..+81, got %d..%d", hw_lo,
          hw_hi);
}

/* The pan servo is mounted mirrored on this robot: a longer pulse turns the
 * head LEFT (bench, 2026-11-05: `servo pan 20` pointed left after the horns
 * were re-seated straight at 1500 us). Angles stay + = right everywhere above
 * this module, so pan's pulse is mirrored about the centre and tilt's is not. */
static void test_pan_is_mirrored_about_the_centre_and_tilt_is_not(void)
{
    s_pwm_hz = 100u;
    CHECK(SERVO_PAN_INVERTED == 1, "this robot's pan servo is mounted mirrored");
    CHECK(servo_angle_to_count(SERVO_PAN, 0) == count_for_pulse(1500u, 100u),
          "pan 0 must stay the 1500 us centre");
    CHECK(servo_angle_to_count(SERVO_PAN, 20) == count_for_pulse(1278u, 100u),
          "pan +20 (right) must be a SHORTER pulse, got count %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, 20));
    CHECK(servo_angle_to_count(SERVO_PAN, -20) == count_for_pulse(1722u, 100u),
          "pan -20 (left) must be a LONGER pulse, got count %u",
          (unsigned)servo_angle_to_count(SERVO_PAN, -20));
    CHECK(servo_angle_to_count(SERVO_TILT, 20) == count_for_pulse(1722u, 100u),
          "tilt +20 (up) stays a longer pulse, got count %u",
          (unsigned)servo_angle_to_count(SERVO_TILT, 20));
    s_pwm_hz = 200u;
}

static void test_the_boot_limits_gate_every_move(void)
{
    reset_bus();

    int16_t lo = 0, hi = 0;
    CHECK(servo_get_limits(SERVO_PAN, &lo, &hi) == ESP_OK && lo == SERVO_PAN_LIMIT_MIN_DEFAULT &&
              hi == SERVO_PAN_LIMIT_MAX_DEFAULT,
          "pan boot limits %d..%d", lo, hi);

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_LIMIT_MAX_DEFAULT + 1) == ESP_ERR_INVALID_ARG,
          "a pan angle past the limit must be refused");
    CHECK(servo_set_angle(SERVO_TILT, SERVO_TILT_LIMIT_MIN_DEFAULT - 1) == ESP_ERR_INVALID_ARG,
          "a tilt angle past the limit must be refused");
    CHECK(s_event_count == 0, "a refused move must not touch the bus, got %d events",
          s_event_count);

    CHECK(servo_set_angle(SERVO_PAN, SERVO_PAN_LIMIT_MAX_DEFAULT) == ESP_OK,
          "the limit itself is allowed");
}

static void test_set_limits_rejects_what_the_hardware_or_centring_cannot_do(void)
{
    CHECK(servo_set_limits(SERVO_PAN, -91, 60) == ESP_ERR_INVALID_ARG, "below -90 accepted");
    CHECK(servo_set_limits(SERVO_PAN, -60, 82) == ESP_ERR_INVALID_ARG,
          "above the 2400 us angle accepted");
    CHECK(servo_set_limits(SERVO_PAN, 5, 60) == ESP_ERR_INVALID_ARG,
          "a range excluding centre accepted — init could never centre");
    CHECK(servo_set_limits(SERVO_PAN, 0, 0) == ESP_ERR_INVALID_ARG, "an empty range accepted");

    reset_bus();
    CHECK(servo_set_limits(SERVO_PAN, -90, 81) == ESP_OK, "the full hardware range is allowed");
    CHECK(s_event_count == 0, "widening must not move the servo");
    CHECK(servo_set_angle(SERVO_PAN, 81) == ESP_OK, "a widened limit must be usable");
}

static void test_narrowing_the_limits_pulls_the_servo_inside(void)
{
    reset_bus();

    CHECK(servo_set_limits(SERVO_PAN, -60, 60) == ESP_OK, "narrow pan to -60..60");
    CHECK(s_event_count == 1, "expected one corrective write, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, servo_angle_to_count(SERVO_PAN, 60));

    int16_t pan = 0;
    CHECK(servo_get_angle(SERVO_PAN, &pan) == ESP_OK && pan == 60,
          "pan must now read the new limit, got %d", pan);
}

/* The bench incident: centred at 50 Hz (count 307), then `servo freq 200`
 * without a re-send left count 307 on a 5 ms period — a ~0.37 ms pulse that
 * held both servos against their end stops until USB was pulled. */
static void test_a_frequency_change_keeps_every_pulse_width(void)
{
    /* State from the previous test: pan=60, tilt=0, both enabled, 200 Hz. */
    reset_bus();

    CHECK(servo_set_pwm_frequency(50u) == ESP_OK, "change to 50 Hz");
    CHECK(s_event_count == 5, "expected release x2, freq, re-send x2; got %d events",
          s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, 0u);
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, 0u);
    CHECK(s_events[2].kind == EV_FREQ && s_events[2].value == 50u,
          "the prescaler must change only after both outputs are released");
    /* Pan is mounted mirrored (SERVO_PAN_INVERTED): +60 is the shorter pulse. */
    CHECK_WRITE(3, SERVO_PAN_CHANNEL, count_for_pulse(1500u - 60u * 1000u / 90u, 50u));
    CHECK_WRITE(4, SERVO_TILT_CHANNEL, count_for_pulse(1500u, 50u));

    reset_bus();
    CHECK(servo_set_pwm_frequency(200u) == ESP_OK, "back to 200 Hz");
    CHECK_WRITE(4, SERVO_TILT_CHANNEL, 1228u);
    CHECK(s_event_count == 5 && s_events[4].value != 307u,
          "the 50 Hz centre count must never survive onto a 200 Hz period");
}

static void test_a_failed_release_leaves_the_frequency_alone(void)
{
    reset_bus();
    s_next_result = ESP_FAIL;

    CHECK(servo_set_pwm_frequency(50u) == ESP_FAIL, "the release error must be returned");
    CHECK(s_event_count == 1 && s_events[0].kind == EV_WRITE,
          "must stop after the failed release, got %d events", s_event_count);
    CHECK(s_pwm_hz == 200u, "frequency changed to %u despite the failed release",
          (unsigned)s_pwm_hz);

    reset_bus();
}

static void test_off_releases_the_outputs_and_on_restores_them(void)
{
    reset_bus();

    CHECK(servo_disable_all() == ESP_OK, "servo off");
    CHECK(s_event_count == 2, "off must write both channels, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, 0u);
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, 0u);
    CHECK(!servo_is_enabled(SERVO_PAN) && !servo_is_enabled(SERVO_TILT), "both must read off");

    reset_bus();
    CHECK(servo_set_angle(SERVO_TILT, 10) == ESP_ERR_INVALID_STATE, "a move while off");
    CHECK(s_event_count == 0, "a move while off must not touch the bus");

    /* A frequency change while off must not wake the outputs up. */
    CHECK(servo_set_pwm_frequency(50u) == ESP_OK, "freq while off");
    CHECK(s_event_count == 1 && s_events[0].kind == EV_FREQ,
          "only the prescaler may change while off, got %d events", s_event_count);

    reset_bus();
    CHECK(servo_enable_all() == ESP_OK, "servo on");
    CHECK(s_event_count == 2, "on must re-send both channels, got %d", s_event_count);
    CHECK_WRITE(0, SERVO_PAN_CHANNEL, servo_angle_to_count(SERVO_PAN, 60));
    CHECK_WRITE(1, SERVO_TILT_CHANNEL, count_for_pulse(1500u, 50u));

    CHECK(servo_set_pwm_frequency(200u) == ESP_OK, "restore 200 Hz");
}

static void test_the_exercise_never_leaves_the_limits(void)
{
    CHECK(servo_set_limits(SERVO_TILT, -20, 25) == ESP_OK, "tilt -20..25");
    reset_bus();

    CHECK(servo_exercise() == ESP_OK, "exercise");
    CHECK(s_event_count == 10, "expected ten steps, got %d", s_event_count);

    /* Pan is mirrored, so its -60 is the larger count; order the bounds. */
    const uint16_t pan_a = servo_angle_to_count(SERVO_PAN, -60);
    const uint16_t pan_b = servo_angle_to_count(SERVO_PAN, 60);
    const uint16_t pan_lo = (pan_a < pan_b) ? pan_a : pan_b;
    const uint16_t pan_hi = (pan_a < pan_b) ? pan_b : pan_a;
    const uint16_t tilt_lo = servo_angle_to_count(SERVO_TILT, -20);
    const uint16_t tilt_hi = servo_angle_to_count(SERVO_TILT, 25);
    bool reached_tilt_max = false;

    for (int i = 0; i < s_event_count && i < 32; i++) {
        const uint16_t v = s_events[i].value;
        if (s_events[i].channel == SERVO_PAN_CHANNEL) {
            CHECK(v >= pan_lo && v <= pan_hi, "pan step %d count %u outside %u..%u", i, (unsigned)v,
                  (unsigned)pan_lo, (unsigned)pan_hi);
        } else {
            CHECK(v >= tilt_lo && v <= tilt_hi, "tilt step %d count %u outside %u..%u", i,
                  (unsigned)v, (unsigned)tilt_lo, (unsigned)tilt_hi);
            reached_tilt_max = reached_tilt_max || v == tilt_hi;
        }
    }
    CHECK(reached_tilt_max, "the exercise must travel to the live tilt limit, not a constant");
}

/* The ambient listener mutes the microphone for a while after this timestamp
 * (ambient_head_quiet()), so what counts as motion decides whether the robot is
 * deaf for a moment or for good: the executor re-asserts the head pose every
 * refresh interval, and that must NOT move it. Runs last, from whatever state
 * the earlier cases left. */
static void test_only_a_changed_angle_counts_as_head_motion(void)
{
    reset_bus();
    CHECK(servo_enable(SERVO_PAN) == ESP_OK, "re-enable pan failed");
    CHECK(servo_enable(SERVO_TILT) == ESP_OK, "re-enable tilt failed");
    CHECK(servo_set_angle(SERVO_PAN, 10) == ESP_OK, "pan 10 failed");

    s_now_us = 5000 * 1000;
    CHECK(servo_set_angle(SERVO_PAN, 20) == ESP_OK, "pan 20 failed");
    CHECK(servo_last_move_ms() == 5000u, "a changed angle must stamp the clock, got %u",
          (unsigned)servo_last_move_ms());

    s_now_us = 6000 * 1000;
    CHECK(servo_set_angle(SERVO_PAN, 20) == ESP_OK, "re-assert failed");
    CHECK(servo_last_move_ms() == 5000u, "an identical re-write is not motion, got %u",
          (unsigned)servo_last_move_ms());

    s_now_us = 7000 * 1000;
    CHECK(servo_set_angle(SERVO_TILT, 5) == ESP_OK, "tilt 5 failed");
    CHECK(servo_last_move_ms() == 7000u, "tilt motion counts too, got %u",
          (unsigned)servo_last_move_ms());

    s_now_us = 8000 * 1000;
    s_next_result = ESP_FAIL;
    (void)servo_set_angle(SERVO_PAN, -20);
    s_next_result = ESP_OK;
    CHECK(servo_last_move_ms() == 7000u, "a failed write moved nothing, got %u",
          (unsigned)servo_last_move_ms());

    s_now_us = 9000 * 1000;
    CHECK(servo_disable(SERVO_PAN) == ESP_OK, "disable failed");
    CHECK(servo_last_move_ms() == 7000u, "releasing a servo is silent, got %u",
          (unsigned)servo_last_move_ms());
    s_now_us = 9500 * 1000;
    CHECK(servo_enable(SERVO_PAN) == ESP_OK, "enable failed");
    CHECK(servo_last_move_ms() == 9500u, "re-energising can twitch the horn, got %u",
          (unsigned)servo_last_move_ms());
}

/* ---- write vs release interleavings (issue #1058) ------------------------ */

static void arm_gate(uint8_t channel, bool after)
{
    pthread_mutex_lock(&s_gate_m);
    s_gate_channel = channel;
    s_gate_after = after;
    s_gate_parked = false;
    s_gate_open = false;
    s_lock_contended = false;
    pthread_mutex_unlock(&s_gate_m);
}

/* Waits on s_gate_cv until *flag_a or *flag_b is set, for at most 2 s, so a
 * deadlock fails the case instead of hanging the suite. */
static bool wait_for(const bool *flag_a, const bool *flag_b)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += 2;

    pthread_mutex_lock(&s_gate_m);
    int rc = 0;
    while (!*flag_a && !(flag_b != NULL && *flag_b) && rc != ETIMEDOUT) {
        rc = pthread_cond_timedwait(&s_gate_cv, &s_gate_m, &deadline);
    }
    const bool ok = *flag_a || (flag_b != NULL && *flag_b);
    pthread_mutex_unlock(&s_gate_m);
    return ok;
}

static void open_gate(void)
{
    pthread_mutex_lock(&s_gate_m);
    s_gate_open = true;
    pthread_cond_broadcast(&s_gate_cv);
    pthread_mutex_unlock(&s_gate_m);
}

/* One writer: the executor's servo_set_angle() or the console's
 * servo_disable_all(). `done` is set under s_gate_m and signalled. */
typedef struct {
    bool is_release;
    int16_t pan_deg;
    esp_err_t result;
    bool done;
} writer_t;

static void *writer_main(void *arg)
{
    writer_t *w = arg;
    const esp_err_t ret =
        w->is_release ? servo_disable_all() : servo_set_angle(SERVO_PAN, w->pan_deg);
    pthread_mutex_lock(&s_gate_m);
    w->result = ret;
    w->done = true;
    pthread_cond_broadcast(&s_gate_cv);
    pthread_mutex_unlock(&s_gate_m);
    return NULL;
}

/* The last value the stub recorded on a channel, or -1 if none. */
static int last_write(uint8_t channel)
{
    int value = -1;
    for (int i = 0; i < s_event_count && i < 32; i++) {
        if (s_events[i].kind == EV_WRITE && s_events[i].channel == channel) {
            value = s_events[i].value;
        }
    }
    return value;
}

/* The executor has checked that pan is enabled and is about to write when the
 * console's `servo off` runs. The release must land after that write: the
 * outputs end at 0 with both servos reported off. Unserialised, the write
 * landed after the release and the head stayed driven while `servo` said off. */
static void test_off_during_an_executor_write_stays_off(void)
{
    CHECK(servo_enable_all() == ESP_OK, "servo on failed");
    reset_bus();

    writer_t exec = {.is_release = false, .pan_deg = 25};
    writer_t console = {.is_release = true};
    pthread_t te, tc;

    arm_gate(SERVO_PAN_CHANNEL, false);
    pthread_create(&te, NULL, writer_main, &exec);
    CHECK(wait_for(&s_gate_parked, NULL), "the executor's write never reached the bus");

    pthread_create(&tc, NULL, writer_main, &console);
    CHECK(wait_for(&console.done, &s_lock_contended),
          "the release neither finished nor waited for the in-flight write");
    open_gate();
    pthread_join(te, NULL);
    pthread_join(tc, NULL);

    CHECK(console.result == ESP_OK, "servo off returned %d", console.result);
    CHECK(last_write(SERVO_PAN_CHANNEL) == 0,
          "pan must end released, but the last pan write was %d (the executor's %u)",
          last_write(SERVO_PAN_CHANNEL), (unsigned)servo_angle_to_count(SERVO_PAN, 25));
    CHECK(last_write(SERVO_TILT_CHANNEL) == 0, "tilt must end released, got %d",
          last_write(SERVO_TILT_CHANNEL));
    CHECK(!servo_is_enabled(SERVO_PAN) && !servo_is_enabled(SERVO_TILT), "both must read off");
}

/* The console's release has reached the bus but not yet marked pan off when
 * the executor's tick writes. The executor must see the servo off and write
 * nothing. Unserialised, it still read pan as enabled, re-drove it after the
 * release, and the flag then said off: a driven servo the executor no longer
 * refreshes or releases. */
static void test_an_executor_write_during_off_is_refused(void)
{
    CHECK(servo_enable_all() == ESP_OK, "servo on failed");
    reset_bus();

    writer_t exec = {.is_release = false, .pan_deg = -25};
    writer_t console = {.is_release = true};
    pthread_t te, tc;

    arm_gate(SERVO_PAN_CHANNEL, true);
    pthread_create(&tc, NULL, writer_main, &console);
    CHECK(wait_for(&s_gate_parked, NULL), "the release never reached the bus");

    pthread_create(&te, NULL, writer_main, &exec);
    CHECK(wait_for(&exec.done, &s_lock_contended),
          "the executor's write neither finished nor waited for the release");
    open_gate();
    pthread_join(tc, NULL);
    pthread_join(te, NULL);

    CHECK(console.result == ESP_OK, "servo off returned %d", console.result);
    CHECK(exec.result == ESP_ERR_INVALID_STATE, "a write after the release must be refused, got %d",
          exec.result);
    CHECK(last_write(SERVO_PAN_CHANNEL) == 0,
          "pan must end released, but the last pan write was %d (the executor's %u)",
          last_write(SERVO_PAN_CHANNEL), (unsigned)servo_angle_to_count(SERVO_PAN, -25));
    CHECK(!servo_is_enabled(SERVO_PAN) && !servo_is_enabled(SERVO_TILT), "both must read off");
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"failed_centring_leaves_module_uninitialised",
         test_failed_centring_leaves_module_uninitialised},
        {"init_centres_both_servos", test_init_centres_both_servos},
        {"set_angle_writes_after_init", test_set_angle_writes_after_init},
        {"count_tracks_the_pwm_frequency", test_the_count_tracks_the_pwm_frequency},
        {"an_impossible_pulse_is_clamped", test_an_impossible_pulse_is_clamped},
        {"an_angle_is_a_real_servo_angle_within_the_sg90_range",
         test_an_angle_is_a_real_servo_angle_within_the_sg90_range},
        {"pan_is_mirrored_about_the_centre_and_tilt_is_not",
         test_pan_is_mirrored_about_the_centre_and_tilt_is_not},
        {"the_boot_limits_gate_every_move", test_the_boot_limits_gate_every_move},
        {"set_limits_rejects_what_the_hardware_or_centring_cannot_do",
         test_set_limits_rejects_what_the_hardware_or_centring_cannot_do},
        {"narrowing_the_limits_pulls_the_servo_inside",
         test_narrowing_the_limits_pulls_the_servo_inside},
        {"a_frequency_change_keeps_every_pulse_width",
         test_a_frequency_change_keeps_every_pulse_width},
        {"a_failed_release_leaves_the_frequency_alone",
         test_a_failed_release_leaves_the_frequency_alone},
        {"off_releases_the_outputs_and_on_restores_them",
         test_off_releases_the_outputs_and_on_restores_them},
        {"the_exercise_never_leaves_the_limits", test_the_exercise_never_leaves_the_limits},
        {"only_a_changed_angle_counts_as_head_motion",
         test_only_a_changed_angle_counts_as_head_motion},
        {"off_during_an_executor_write_stays_off", test_off_during_an_executor_write_stays_off},
        {"an_executor_write_during_off_is_refused", test_an_executor_write_during_off_is_refused},
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
