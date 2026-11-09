// Host unit tests for cli_gate.c (no Pico SDK): what the CLI's `arm` and
// `load` decide from the snapshot before sending anything to core 1.
#include <assert.h>
#include <stdio.h>

#include "cli_gate.h"
#include "ipc.h"

// assert() aborts without flushing a redirected stdout, so print and flush
// the failing case's name first (.claude/rules/testing.md).
static void check(int cond, const char *what)
{
    if (!cond) {
        printf("FAIL: %s\n", what);
        fflush(stdout);
    }
    assert(cond);
}

static const imu_gravity_check_t UPRIGHT = {.still = true, .upside_down = false, .up_g = 1.0f};
static const imu_gravity_check_t UPSIDE_DOWN = {.still = true, .upside_down = true, .up_g = -1.0f};

// 'mount down' + 'save' + 'arm' pasted: the save's sector erase stalls core 1
// past the CLI's wait, so the snapshot predates the mount. Nothing in it is
// judged, not even a gravity check that would refuse.
static void test_arm_stale_snapshot_not_judged(void)
{
    check(cli_gate_arm(false, BAL_STATE_IDLE, &UPRIGHT) == CLI_GATE_BUSY,
          "stale, upright: busy, not sent");
    check(cli_gate_arm(false, BAL_STATE_IDLE, &UPSIDE_DOWN) == CLI_GATE_BUSY,
          "stale, upside down: busy");
    check(cli_gate_arm(false, BAL_STATE_FAULT, &UPRIGHT) == CLI_GATE_BUSY, "stale, fault: busy");
}

// Core 1 arms only from IDLE; in FAULT or RUN it ignores CMD_ARM.
static void test_arm_needs_idle(void)
{
    check(cli_gate_arm(true, BAL_STATE_FAULT, &UPRIGHT) == CLI_GATE_NOT_IDLE, "fault: refused");
    check(cli_gate_arm(true, BAL_STATE_RUN, &UPRIGHT) == CLI_GATE_NOT_IDLE, "run: refused");
    // The state is reported before the gravity check: in FAULT the robot may
    // be lying down, and 'check mount' would send the user the wrong way.
    check(cli_gate_arm(true, BAL_STATE_FAULT, &UPSIDE_DOWN) == CLI_GATE_NOT_IDLE,
          "fault, upside down: state named first");
}

static void test_arm_idle(void)
{
    check(cli_gate_arm(true, BAL_STATE_IDLE, &UPRIGHT) == CLI_GATE_SEND, "idle, upright: sent");
    check(cli_gate_arm(true, BAL_STATE_IDLE, &UPSIDE_DOWN) == CLI_GATE_UPSIDE_DOWN,
          "idle, upside down: refused");
    const imu_gravity_check_t moving = {.still = false, .upside_down = true, .up_g = -1.0f};
    check(cli_gate_arm(true, BAL_STATE_IDLE, &moving) == CLI_GATE_SEND,
          "idle, moving: not refused by the gravity check");
}

// Core 1 ignores CMD_SET_IMU_MOUNT in RUN, so a load there would apply
// everything but the mount.
static void test_load(void)
{
    check(cli_gate_load(true, BAL_STATE_RUN) == CLI_GATE_ARMED, "run: refused");
    check(cli_gate_load(true, BAL_STATE_IDLE) == CLI_GATE_SEND, "idle: sent");
    check(cli_gate_load(true, BAL_STATE_FAULT) == CLI_GATE_SEND, "fault: sent (mount accepted)");
    check(cli_gate_load(false, BAL_STATE_IDLE) == CLI_GATE_BUSY, "stale: busy");
}

int main(void)
{
    test_arm_stale_snapshot_not_judged();
    test_arm_needs_idle();
    test_arm_idle();
    test_load();
    printf("test_cli_gate: all tests passed\n");
    return 0;
}
