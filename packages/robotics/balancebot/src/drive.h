// Driving while balancing: a commanded forward speed and a differential
// (turn) speed, ramped, timed out, and mixed onto the two wheels. Pure C with
// no SDK includes so the host tests compile it (tests/host/test_drive.c);
// float only, safe on core 1.
//
// The balance law sees only the mean wheel speed, so the drive enters it as a
// moving reference: the PID's velocity loop holds the wheels at v_ref instead
// of 0, and the LQR tracks a position reference advancing at v_ref. The lean
// that accelerates the robot comes out of the balance law itself. The turn is
// added to one wheel and subtracted from the other after the balance law, so
// the mean, and with it the pitch dynamics, is unchanged.
//
// Units are steps/s throughout; core 0 converts from m/s. Signs: speed > 0 is
// forward (the sign of wheel_rate_sps); turn > 0 speeds the left wheel up and
// slows the right one, a clockwise turn seen from above.
#ifndef BALANCEBOT_DRIVE_H
#define BALANCEBOT_DRIVE_H

typedef struct {
    float speed_target_sps;  // commanded, clamped
    float turn_target_sps;
    float speed_ref_sps;  // ramped towards the targets each tick
    float turn_ref_sps;
    float remaining_s;  // until the targets drop to 0 on their own
} drive_t;

typedef struct {
    float left_sps;
    float right_sps;
} drive_wheels_t;

// Stop at once: targets, references and the timer to 0. On every state
// change, so a drive never outlives the RUN it was sent in.
void drive_reset(drive_t *d);

// New targets for `seconds`. Speed is clamped to +/- speed_max, turn to
// +/- turn_max, and seconds to [0, DRIVE_MAX_S]; a non-finite value is
// treated as 0. The references keep their current value and ramp from it.
void drive_command(drive_t *d, float speed_sps, float turn_sps, float seconds, float speed_max,
                   float turn_max);

// One control tick: count down the timer (at 0 the targets become 0), then
// move each reference towards its target by at most accel * dt.
void drive_step(drive_t *d, float dt, float speed_accel_sps2, float turn_accel_sps2);

// The LQR's position reference, advanced by speed_ref * dt and held within
// +/- max_lead_steps of the wheel position, so a robot that cannot keep up
// (wheels at the rate clamp, a slope) is not pulled harder and harder by a
// reference running away from it.
float drive_advance_ref(float x_ref_steps, float x_steps, float speed_ref_sps, float dt,
                        float max_lead_steps);

// Per-wheel rates from the balance law's mean rate and the turn. Balance has
// priority: the turn is cut so that neither wheel exceeds +/- max_rate, and
// the mean is never changed by the turn.
drive_wheels_t drive_mix(float mean_sps, float turn_sps, float max_rate);

#endif  // BALANCEBOT_DRIVE_H
