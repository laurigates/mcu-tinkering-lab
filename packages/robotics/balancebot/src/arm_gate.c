#include "arm_gate.h"

void arm_gate_observe(arm_gate_t *g, uint32_t mount, bool ok, const float accel_g[3],
                      const float gyro_dps[3])
{
    g->sample_ok = ok;
    if (!ok) {
        g->check = (imu_gravity_check_t){0};
        return;
    }
    for (int i = 0; i < 3; i++) {
        g->accel_g[i] = accel_g[i];
        g->gyro_dps[i] = gyro_dps[i];
    }
    g->check = imu_gravity_check(mount, accel_g, gyro_dps);
}

bool arm_gate_allows(arm_gate_t *g, uint32_t mount)
{
    if (!g->sample_ok) {
        return true;
    }
    const imu_gravity_check_t now = imu_gravity_check(mount, g->accel_g, g->gyro_dps);
    if (!imu_gravity_refuses_arm(&now)) {
        return true;
    }
    g->refusals++;
    g->refused_up_g = now.up_g;
    return false;
}

bool arm_gate_mount_accepted(uint32_t current, uint32_t requested, bool running)
{
    return !running && (requested == IMU_MOUNT_UP || requested == IMU_MOUNT_DOWN) &&
           requested != current;
}
