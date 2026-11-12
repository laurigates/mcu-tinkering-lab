#include "cli_gate.h"

#include "ipc.h"

cli_gate_t cli_gate_arm(bool synced, uint8_t state, bool imu_ok, const imu_gravity_check_t *gravity)
{
    if (!synced) {
        return CLI_GATE_BUSY;
    }
    if (state == BAL_STATE_FAULT && !imu_ok) {
        return CLI_GATE_FAULT_NO_IMU;
    }
    if (state != BAL_STATE_IDLE) {
        return CLI_GATE_NOT_IDLE;
    }
    if (imu_gravity_refuses_arm(gravity)) {
        return CLI_GATE_UPSIDE_DOWN;
    }
    return CLI_GATE_SEND;
}

cli_gate_t cli_gate_mount(bool synced, uint8_t state)
{
    if (!synced) {
        return CLI_GATE_BUSY;
    }
    if (state == BAL_STATE_RUN) {
        return CLI_GATE_ARMED;
    }
    return CLI_GATE_SEND;
}

cli_gate_t cli_gate_load(bool synced, uint8_t state)
{
    return cli_gate_mount(synced, state);
}

cli_arm_outcome_t cli_arm_outcome(bool synced, uint32_t refusals_before, uint32_t refusals_now,
                                  uint8_t state, bool imu_ok)
{
    if (!synced) {
        return CLI_ARM_NO_ANSWER;
    }
    if (refusals_now != refusals_before) {
        return CLI_ARM_REFUSED;
    }
    switch (state) {
        case BAL_STATE_RUN:
            return CLI_ARM_OK;
        case BAL_STATE_FAULT:
            return imu_ok ? CLI_ARM_FAULTED_TILT : CLI_ARM_FAULTED_NO_IMU;
        default:
            return CLI_ARM_NOT_ARMED;
    }
}

cli_mount_outcome_t cli_mount_outcome(bool synced, uint32_t applied, uint32_t requested)
{
    if (!synced) {
        return CLI_MOUNT_NO_ANSWER;
    }
    return applied == requested ? CLI_MOUNT_APPLIED : CLI_MOUNT_DROPPED;
}
