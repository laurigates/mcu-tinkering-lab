// LQR for the wheeled inverted pendulum. Pure C (no SDK dependencies) so it
// can be host-unit-tested; see tests/host/test_lqr.c and lqr_reference.py.
//
// Model (input = wheel acceleration, matching ADR-018's acceleration-output
// framing, so the integrate-to-step-rate path is shared with the PID):
//
//     state  s = [theta, theta_dot, x, x_dot]   (rad, rad/s, m, m/s)
//     input  u = x_ddot                          (m/s^2, wheel-contact accel)
//     theta_ddot = (g * theta - u) / L_eff
//     x_ddot     = u
//
// theta is measured from vertical, positive when the body leans towards +x
// (the direction a positive u, and a positive wheel step rate, drives). The
// control law is u = -K s.
//
// Two halves with different cost profiles:
//   - lqr_solve()  (double, ~20 doubling iterations, 4x4 inverses): core 0
//     only, at boot and when a model/weight parameter changes. Never on
//     core 1, never in the 500 Hz loop. Allocates nothing; bounded loop.
//   - lqr_accel() and the unit helpers (float, a handful of multiplies):
//     safe for the 500 Hz loop on core 1.
//
// Sign convention against the PID (pinned by test_lqr.c):
// the angle PID computes accel = kp * (sp - pitch) - kd * gyro, so a small
// positive pitch (IMU_PITCH_SIGN already applied) commands a NEGATIVE wheel
// acceleration. LQR drives into the lean too (u has the sign of theta), so
// theta must be the NEGATED pitch: lqr_theta_from_pitch() and
// lqr_theta_dot_from_gyro() do that mapping. Whatever IMU_PITCH_SIGN
// bring-up settles on for the PID therefore carries over to LQR unchanged.
// x is wheel travel in the positive step direction (steps / steps-per-metre).
#ifndef BALANCEBOT_LQR_H
#define BALANCEBOT_LQR_H

#define LQR_N 4
#define LQR_G_MPS2 9.80665

// Doubling iterations before lqr_solve() gives up. SDA converges
// quadratically; the reference cases converge in well under half of this.
#define LQR_MAX_ITER 60

// Placeholder defaults. Measure L_eff with the swing test (hang the robot
// from its axle, time small swings: L_eff = g * (T / 2pi)^2). Weights follow
// Bryson's rule: q_i = 1 / (acceptable excursion)^2, r = 1 / (accel budget)^2,
// here 0.05 rad, 1 rad/s, 0.2 m, 0.5 m/s, 5 m/s^2.
#define LQR_DEFAULT_L_EFF_M 0.12f
#define LQR_DEFAULT_WHEEL_DIAM_M 0.09f
#define LQR_DEFAULT_STEPS_PER_REV 1600.0f  // 200 full steps x 1/8 microstep
#define LQR_DEFAULT_Q_THETA 400.0f
#define LQR_DEFAULT_Q_DTHETA 1.0f
#define LQR_DEFAULT_Q_X 25.0f
#define LQR_DEFAULT_Q_DX 4.0f
#define LQR_DEFAULT_R 0.04f

typedef enum {
    LQR_OK = 0,
    LQR_ERR_INVALID = -1,      // L_eff <= 0, g <= 0, dt <= 0, q < 0, r <= 0, non-finite input
    LQR_ERR_SINGULAR = -2,     // a 4x4 inverse hit a zero pivot
    LQR_ERR_NO_CONVERGE = -3,  // doubling did not converge within LQR_MAX_ITER
    LQR_ERR_NON_FINITE = -4,   // P, K, rho or residual not finite
    LQR_ERR_UNSTABLE = -5,     // closed-loop spectral radius >= 1 (e.g. q_x = 0)
} lqr_status_t;

typedef struct {
    double l_eff_m;  // equivalent pendulum length (m), > 0
    double g;        // gravity (m/s^2), > 0; normally LQR_G_MPS2
} lqr_model_t;

typedef struct {
    double q[LQR_N];  // diagonal state weights, >= 0 (theta, theta_dot, x, x_dot)
    double r;         // input weight, > 0
} lqr_weights_t;

typedef struct {
    float K[LQR_N];   // u = -(K . s), u in m/s^2
    double rho;       // closed-loop spectral radius of A - B K (< 1 on success)
    double residual;  // ||DARE(P)||_F / max(1, ||P||_F)
    int iterations;   // doubling iterations used
} lqr_result_t;

// Exact zero-order-hold discretisation of the model at step dt (closed form
// via cosh/sinh of w = sqrt(g / L_eff)). Returns LQR_OK or LQR_ERR_INVALID
// (A and B untouched on failure).
int lqr_discretise(const lqr_model_t *model, double dt, double A[LQR_N][LQR_N], double B[LQR_N]);

// Discretise, solve the DARE by the structure-preserving doubling algorithm,
// and form K. On any failure returns a negative lqr_status_t and leaves *out
// untouched, so a previously valid K survives a bad parameter change.
int lqr_solve(const lqr_model_t *model, const lqr_weights_t *weights, double dt, lqr_result_t *out);

// Control law u = -(K0 theta + K1 theta_dot + K2 x_err + K3 x_dot), m/s^2.
// x_err = x - x_ref. Pure float; safe on core 1.
float lqr_accel(const float K[LQR_N], float theta, float theta_dot, float x_err, float x_dot);

// pitch (deg, IMU_PITCH_SIGN applied) -> theta (rad). See sign convention above.
float lqr_theta_from_pitch(float pitch_deg);

// gyro pitch rate (deg/s, IMU_PITCH_SIGN applied) -> theta_dot (rad/s).
float lqr_theta_dot_from_gyro(float gyro_dps);

// Steps per metre of wheel travel: steps_per_rev / (pi * wheel_diam_m).
// Multiply m/s^2 by this for steps/s^2; divide steps by it for metres.
float lqr_steps_per_metre(float steps_per_rev, float wheel_diam_m);

#endif  // BALANCEBOT_LQR_H
