#include "cli_gate.h"

#include "ipc.h"

cli_gate_t cli_gate_arm(bool synced, uint8_t state, const imu_gravity_check_t *gravity)
{
    if (!synced) {
        return CLI_GATE_BUSY;
    }
    if (state != BAL_STATE_IDLE) {
        return CLI_GATE_NOT_IDLE;
    }
    if (imu_gravity_refuses_arm(gravity)) {
        return CLI_GATE_UPSIDE_DOWN;
    }
    return CLI_GATE_SEND;
}

cli_gate_t cli_gate_load(bool synced, uint8_t state)
{
    if (!synced) {
        return CLI_GATE_BUSY;
    }
    if (state == BAL_STATE_RUN) {
        return CLI_GATE_ARMED;
    }
    return CLI_GATE_SEND;
}
