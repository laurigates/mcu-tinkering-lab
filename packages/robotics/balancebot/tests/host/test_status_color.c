// Host unit tests for status_color.c: what the RGB LED shows for each state,
// and the auto-arm guidance (no Pico SDK).
#include <assert.h>
#include <stdio.h>

#include "status_color.h"

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("FAIL: %s\n", what);
        fflush(stdout);
    }
    assert(cond);
}

static status_inputs_t idle_at(float pitch)
{
    return (status_inputs_t){
        .state = 0,
        .starting = false,
        .imu_ok = true,
        .pitch_deg = pitch,
        .arm_holdoff_ticks = 0,
        .gesture_ticks = 0,
        .gesture_needed = 1500,
        .arm_window_deg = 1.0f,
        .ms = 0,
    };
}

static int is_green(rgb_t c)
{
    return c.g > 0 && c.r == 0 && c.b == 0;
}

static void test_guidance_by_angle(void)
{
    status_inputs_t in = idle_at(12.0f);
    rgb_t far = status_color(&in);
    check(far.b > 0 && far.r == 0 && far.g == 0, "far: blue");
    in.pitch_deg = -3.0f;
    rgb_t near = status_color(&in);
    check(near.r > 0 && near.g > 0 && near.b == 0, "within 5 deg: yellow");
    in.pitch_deg = 0.5f;
    check(is_green(status_color(&in)), "inside the window: green");
    in.pitch_deg = -0.99f;
    check(is_green(status_color(&in)), "window is symmetric");
    in.pitch_deg = 1.0f;
    check(!is_green(status_color(&in)), "window edge excluded, as the gesture's < test");
}

static void test_window_brightens_with_the_count(void)
{
    status_inputs_t in = idle_at(0.2f);
    in.gesture_ticks = 0;
    const uint8_t start = status_color(&in).g;
    in.gesture_ticks = 750;
    const uint8_t half = status_color(&in).g;
    in.gesture_ticks = 1500;
    const uint8_t full = status_color(&in).g;
    in.gesture_ticks = 99999;
    const uint8_t over = status_color(&in).g;
    check(start > 0, "visible on entering the window");
    check(start < half && half < full, "brightens with the count");
    check(full == STATUS_RGB_MAX, "full scale when it arms");
    check(over == full, "capped past the count");
}

static void test_state_priorities(void)
{
    status_inputs_t in = idle_at(0.0f);
    in.starting = true;
    rgb_t c = status_color(&in);
    check(c.g > 0 && c.b > 0 && c.r == 0, "starting: cyan, whatever the angle");

    in = idle_at(0.0f);
    in.state = 1;
    check(is_green(status_color(&in)) && status_color(&in).g == STATUS_RGB_MAX, "run: green");

    in = idle_at(0.0f);
    in.state = 2;
    in.ms = 0;
    c = status_color(&in);
    check(c.r > 0 && c.g == 0 && c.b == 0, "fault: red on");
    in.ms = 100;
    c = status_color(&in);
    check(c.r == 0 && c.g == 0 && c.b == 0, "fault: off half the time");

    in = idle_at(0.0f);
    in.imu_ok = false;
    c = status_color(&in);
    check(c.r > 0 && c.g == 0 && c.b == 0, "imu down: red");

    in = idle_at(0.0f);
    in.arm_holdoff_ticks = 10;
    c = status_color(&in);
    check(c.r > 0 && c.b > 0 && c.g == 0, "hold-off: purple, not green in the window");
}

static void test_never_above_ceiling(void)
{
    status_inputs_t in = idle_at(0.0f);
    for (uint32_t ms = 0; ms < 4000; ms += 7) {
        in.starting = true;
        in.ms = ms;
        rgb_t c = status_color(&in);
        check(c.r <= STATUS_RGB_MAX && c.g <= STATUS_RGB_MAX && c.b <= STATUS_RGB_MAX,
              "pulse stays under the ceiling");
    }
}

int main(void)
{
    test_guidance_by_angle();
    test_window_brightens_with_the_count();
    test_state_priorities();
    test_never_above_ceiling();
    printf("test_status_color: all tests passed\n");
    return 0;
}
