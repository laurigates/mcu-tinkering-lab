// Host unit tests for cli_gate.c (no Pico SDK): what the CLI's `arm`, `mount`
// and `load` decide from the snapshot before sending anything to core 1, and
// what they report once core 1 has drained the command.
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
    check(cli_gate_arm(false, BAL_STATE_IDLE, true, &UPRIGHT) == CLI_GATE_BUSY,
          "stale, upright: busy, not sent");
    check(cli_gate_arm(false, BAL_STATE_IDLE, true, &UPSIDE_DOWN) == CLI_GATE_BUSY,
          "stale, upside down: busy");
    check(cli_gate_arm(false, BAL_STATE_FAULT, true, &UPRIGHT) == CLI_GATE_BUSY,
          "stale, fault: busy");
    check(cli_gate_arm(false, BAL_STATE_FAULT, false, &UPRIGHT) == CLI_GATE_BUSY,
          "stale, fault, no IMU: busy");
}

// Core 1 arms only from IDLE; in FAULT or RUN it ignores CMD_ARM.
static void test_arm_needs_idle(void)
{
    check(cli_gate_arm(true, BAL_STATE_FAULT, true, &UPRIGHT) == CLI_GATE_NOT_IDLE,
          "fault: refused");
    check(cli_gate_arm(true, BAL_STATE_RUN, true, &UPRIGHT) == CLI_GATE_NOT_IDLE, "run: refused");
    // The state is reported before the gravity check: in FAULT the robot may
    // be lying down, and 'check mount' would send the user the wrong way.
    check(cli_gate_arm(true, BAL_STATE_FAULT, true, &UPSIDE_DOWN) == CLI_GATE_NOT_IDLE,
          "fault, upside down: state named first");
}

// Core 1 also enters FAULT when the IMU read fails, and clears FAULT by a hold
// only while the IMU reads. With the IMU silent, the refusal must not offer the
// hold: only 'disarm' clears it.
static void test_arm_fault_without_imu(void)
{
    check(cli_gate_arm(true, BAL_STATE_FAULT, false, &UPRIGHT) == CLI_GATE_FAULT_NO_IMU,
          "fault, IMU silent: only disarm clears it");
    check(cli_gate_arm(true, BAL_STATE_RUN, false, &UPRIGHT) == CLI_GATE_NOT_IDLE,
          "run, IMU silent: already armed");
}

static void test_arm_idle(void)
{
    check(cli_gate_arm(true, BAL_STATE_IDLE, true, &UPRIGHT) == CLI_GATE_SEND,
          "idle, upright: sent");
    check(cli_gate_arm(true, BAL_STATE_IDLE, true, &UPSIDE_DOWN) == CLI_GATE_UPSIDE_DOWN,
          "idle, upside down: refused");
    const imu_gravity_check_t moving = {.still = false, .upside_down = true, .up_g = -1.0f};
    check(cli_gate_arm(true, BAL_STATE_IDLE, true, &moving) == CLI_GATE_SEND,
          "idle, moving: not refused by the gravity check");
}

// Core 1 ignores CMD_SET_IMU_MOUNT in RUN. `mount` judges a snapshot taken
// after the commands before it drained: a pasted 'disarm' + 'mount down' is
// judged on the IDLE that disarm leaves, not a stale RUN.
static void test_mount(void)
{
    check(cli_gate_mount(true, BAL_STATE_RUN) == CLI_GATE_ARMED, "run: refused");
    check(cli_gate_mount(true, BAL_STATE_IDLE) == CLI_GATE_SEND, "idle: sent");
    check(cli_gate_mount(true, BAL_STATE_FAULT) == CLI_GATE_SEND, "fault: sent (mount accepted)");
    check(cli_gate_mount(false, BAL_STATE_IDLE) == CLI_GATE_BUSY, "stale idle: busy");
    check(cli_gate_mount(false, BAL_STATE_RUN) == CLI_GATE_BUSY,
          "stale run: busy, not judged on the stale RUN");
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

// After CMD_ARM drained. Core 1 arms in the command drain and runs the RUN
// branch in the same tick, so a robot lying past the tilt cutoff, or an IMU
// that does not read, shows FAULT in the first snapshot after the arm.
static void test_arm_outcome(void)
{
    check(cli_arm_outcome(true, 3, 3, BAL_STATE_RUN, true) == CLI_ARM_OK, "run: OK");
    check(cli_arm_outcome(false, 3, 3, BAL_STATE_IDLE, true) == CLI_ARM_NO_ANSWER,
          "not drained: no answer, nothing judged");
    check(cli_arm_outcome(false, 3, 4, BAL_STATE_IDLE, true) == CLI_ARM_NO_ANSWER,
          "not drained, refusal moved: still no answer");
    check(cli_arm_outcome(true, 3, 4, BAL_STATE_IDLE, true) == CLI_ARM_REFUSED,
          "refusals moved: core 1 refused");
    check(cli_arm_outcome(true, 3, 3, BAL_STATE_FAULT, true) == CLI_ARM_FAULTED_TILT,
          "armed past the tilt cutoff: faulted, not OK");
    check(cli_arm_outcome(true, 3, 3, BAL_STATE_FAULT, false) == CLI_ARM_FAULTED_NO_IMU,
          "armed with the IMU silent: faulted on the IMU");
    check(cli_arm_outcome(true, 3, 3, BAL_STATE_IDLE, true) == CLI_ARM_NOT_ARMED,
          "still idle, no refusal: not armed, not OK");
    // The refusal counter wraps; a change is a change.
    check(cli_arm_outcome(true, 0xFFFFFFFFu, 0u, BAL_STATE_IDLE, true) == CLI_ARM_REFUSED,
          "refusal counter wrapped: refused");
}

// After CMD_SET_IMU_MOUNT drained (`mount`, or the mount in `load`'s burst).
// If the hold-upright gesture armed core 1 first, core 1 dropped the mount and
// the CLI's mirror must take the mount core 1 applies.
static void test_mount_outcome(void)
{
    check(cli_mount_outcome(true, IMU_MOUNT_DOWN, IMU_MOUNT_DOWN) == CLI_MOUNT_APPLIED, "applied");
    check(cli_mount_outcome(true, IMU_MOUNT_UP, IMU_MOUNT_DOWN) == CLI_MOUNT_DROPPED,
          "core 1 kept another mount: dropped");
    check(cli_mount_outcome(false, IMU_MOUNT_UP, IMU_MOUNT_DOWN) == CLI_MOUNT_NO_ANSWER,
          "not drained: no answer");
    check(cli_mount_outcome(false, IMU_MOUNT_DOWN, IMU_MOUNT_DOWN) == CLI_MOUNT_NO_ANSWER,
          "not drained, same mount: still no answer");
}

int main(void)
{
    test_arm_stale_snapshot_not_judged();
    test_arm_needs_idle();
    test_arm_fault_without_imu();
    test_arm_idle();
    test_mount();
    test_load();
    test_arm_outcome();
    test_mount_outcome();
    printf("test_cli_gate: all tests passed\n");
    return 0;
}
