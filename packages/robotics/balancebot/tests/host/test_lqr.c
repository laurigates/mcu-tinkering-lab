// Host unit tests for lqr.c (no Pico SDK). Links pid.c for the sign test.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "lqr.h"
#include "pid.h"

#define DT (1.0 / 500.0)

static const lqr_model_t DEFAULT_MODEL = {LQR_DEFAULT_L_EFF_M, LQR_G_MPS2};
static const lqr_weights_t DEFAULT_WEIGHTS = {
    {LQR_DEFAULT_Q_THETA, LQR_DEFAULT_Q_DTHETA, LQR_DEFAULT_Q_X, LQR_DEFAULT_Q_DX},
    LQR_DEFAULT_R,
};

// Reference gains and closed-loop spectral radii from scipy
// (expm + solve_discrete_are), produced by:
//   uv run --with scipy --with numpy python3 tests/host/lqr_reference.py
// The parameter sets here must match CASES in that script.
static const struct {
    const char *name;
    double l_eff;
    double q[4];
    double r;
    double K[4];
    double rho;
} CASES[] = {
    {"default",
     0.12,
     {400.0, 1.0, 25.0, 4.0},
     0.04,
     {-1.250794707459e+02, -1.027758295924e+01, -2.346975882868e+01, -2.529596690166e+01},
     9.975763707590e-01},
    {"short_l",
     0.05,
     {400.0, 1.0, 25.0, 4.0},
     0.04,
     {-1.133503125419e+02, -6.721580634008e+00, -2.211177851598e+01, -2.334874764368e+01},
     9.975836499452e-01},
    {"heavy_r",
     0.12,
     {400.0, 1.0, 25.0, 4.0},
     4.0,
     {-2.885999617750e+01, -3.043159263191e+00, -2.444663951961e+00, -3.233527364421e+00},
     9.979986233490e-01},
};

// Series expm of the 5x5 augmented matrix [[Ac, Bc], [0, 0]] * dt. With
// ||Ac dt|| ~ 0.02 the Taylor series reaches double precision in < 20 terms.
static void series_zoh(double l, double g, double dt, double A[4][4], double B[4])
{
    double M[5][5] = {{0}};
    M[0][1] = 1.0;
    M[1][0] = g / l;
    M[2][3] = 1.0;
    M[1][4] = -1.0 / l;
    M[3][4] = 1.0;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            M[i][j] *= dt;
        }
    }
    double E[5][5] = {{0}}, T[5][5] = {{0}};
    for (int i = 0; i < 5; i++) {
        E[i][i] = 1.0;
        T[i][i] = 1.0;
    }
    for (int n = 1; n < 30; n++) {
        double N[5][5];
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                double s = 0.0;
                for (int k = 0; k < 5; k++) {
                    s += T[i][k] * M[k][j];
                }
                N[i][j] = s / n;
            }
        }
        memcpy(T, N, sizeof(T));
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                E[i][j] += T[i][j];
            }
        }
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A[i][j] = E[i][j];
        }
        B[i] = E[i][4];
    }
}

static void test_discretise_matches_series_expm(void)
{
    const double ls[] = {0.03, 0.12, 0.5};
    for (unsigned c = 0; c < sizeof(ls) / sizeof(ls[0]); c++) {
        lqr_model_t m = {ls[c], LQR_G_MPS2};
        double A[4][4], B[4], As[4][4], Bs[4];
        assert(lqr_discretise(&m, DT, A, B) == LQR_OK);
        series_zoh(ls[c], LQR_G_MPS2, DT, As, Bs);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                assert(fabs(A[i][j] - As[i][j]) < 1e-12);
            }
            assert(fabs(B[i] - Bs[i]) < 1e-12);
        }
    }
}

static void test_solve_matches_scipy_reference(void)
{
    for (unsigned c = 0; c < sizeof(CASES) / sizeof(CASES[0]); c++) {
        lqr_model_t m = {CASES[c].l_eff, LQR_G_MPS2};
        lqr_weights_t w;
        memcpy(w.q, CASES[c].q, sizeof(w.q));
        w.r = CASES[c].r;
        lqr_result_t res;
        int st = lqr_solve(&m, &w, DT, &res);
        printf("  %-8s K = [%.6g %.6g %.6g %.6g] rho = %.10f residual = %.3g iter = %d\n",
               CASES[c].name, (double)res.K[0], (double)res.K[1], (double)res.K[2],
               (double)res.K[3], res.rho, res.residual, res.iterations);
        assert(st == LQR_OK);
        for (int i = 0; i < 4; i++) {
            // K is stored as float: compare to float precision.
            assert(fabs((double)res.K[i] - CASES[c].K[i]) <= 1e-5 * fabs(CASES[c].K[i]));
        }
        assert(res.residual < 1e-9);
        assert(res.rho < 1.0);
        assert(fabs(res.rho - CASES[c].rho) < 1e-6);
        assert(res.iterations > 0 && res.iterations <= LQR_MAX_ITER);
    }
}

// Small positive pitch, zero rates and zero x: LQR's wheel acceleration has
// the same sign as the angle PID's (which is how the PID is fed in
// balance_control.c: setpoint 0 + lean offset, measurement = pitch with
// IMU_PITCH_SIGN applied, rate = gyro).
static void test_sign_matches_pid(void)
{
    lqr_result_t res;
    assert(lqr_solve(&DEFAULT_MODEL, &DEFAULT_WEIGHTS, DT, &res) == LQR_OK);
    const float spm = lqr_steps_per_metre(LQR_DEFAULT_STEPS_PER_REV, LQR_DEFAULT_WHEEL_DIAM_M);
    const float pitches[] = {0.5f, 2.0f, -0.5f, -2.0f};
    for (unsigned c = 0; c < sizeof(pitches) / sizeof(pitches[0]); c++) {
        pid_t pid;
        pid_init(&pid, DEFAULT_ANGLE_KP, DEFAULT_ANGLE_KI, DEFAULT_ANGLE_KD, ANGLE_INTEGRAL_MAX,
                 ANGLE_PID_OUT_MAX);
        float pid_out = pid_update(&pid, 0.0f, pitches[c], 0.0f, (float)DT);
        float lqr_out = spm * lqr_accel(res.K, lqr_theta_from_pitch(pitches[c]),
                                        lqr_theta_dot_from_gyro(0.0f), 0.0f, 0.0f);
        assert(pid_out != 0.0f && lqr_out != 0.0f);
        assert((pid_out > 0.0f) == (lqr_out > 0.0f));
    }
    // Same for the rate term: a positive pitch rate alone (kd path).
    {
        pid_t pid;
        pid_init(&pid, 0.0f, 0.0f, DEFAULT_ANGLE_KD, ANGLE_INTEGRAL_MAX, ANGLE_PID_OUT_MAX);
        float pid_out = pid_update(&pid, 0.0f, 0.0f, 10.0f, (float)DT);
        float lqr_out = lqr_accel(res.K, 0.0f, lqr_theta_dot_from_gyro(10.0f), 0.0f, 0.0f);
        assert((pid_out > 0.0f) == (lqr_out > 0.0f));
    }
}

static void test_unit_helpers(void)
{
    // 1600 steps per pi*0.09 m.
    float spm = lqr_steps_per_metre(1600.0f, 0.09f);
    assert(fabsf(spm - 5658.842f) < 0.01f);
    assert(fabsf(lqr_theta_from_pitch(180.0f) + 3.1415927f) < 1e-6f);
    float K[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    assert(lqr_accel(K, 1.0f, 1.0f, 1.0f, 1.0f) == -10.0f);
}

static int k_untouched(const lqr_result_t *r)
{
    return r->K[0] == 7.0f && r->K[1] == 7.0f && r->K[2] == 7.0f && r->K[3] == 7.0f &&
           r->rho == 7.0 && r->residual == 7.0 && r->iterations == 7;
}

static void expect_fail(const lqr_model_t *m, const lqr_weights_t *w, double dt)
{
    lqr_result_t res = {{7.0f, 7.0f, 7.0f, 7.0f}, 7.0, 7.0, 7};
    int st = lqr_solve(m, w, dt, &res);
    assert(st < 0);
    assert(k_untouched(&res));
}

static void test_invalid_inputs_fail_closed(void)
{
    lqr_model_t m = DEFAULT_MODEL;
    lqr_weights_t w = DEFAULT_WEIGHTS;

    m.l_eff_m = 0.0;
    expect_fail(&m, &w, DT);
    m.l_eff_m = -0.1;
    expect_fail(&m, &w, DT);
    m.l_eff_m = NAN;
    expect_fail(&m, &w, DT);
    m.l_eff_m = INFINITY;
    expect_fail(&m, &w, DT);
    m = DEFAULT_MODEL;
    m.g = 0.0;
    expect_fail(&m, &w, DT);
    m = DEFAULT_MODEL;

    w.r = 0.0;
    expect_fail(&m, &w, DT);
    w.r = -1.0;
    expect_fail(&m, &w, DT);
    w.r = NAN;
    expect_fail(&m, &w, DT);
    w = DEFAULT_WEIGHTS;
    w.q[1] = -1.0;
    expect_fail(&m, &w, DT);
    w.q[2] = NAN;
    expect_fail(&m, &w, DT);
    w = DEFAULT_WEIGHTS;

    expect_fail(&m, &w, 0.0);
    expect_fail(&m, &w, NAN);
    expect_fail(NULL, &w, DT);
    expect_fail(&m, NULL, DT);
    assert(lqr_solve(&m, &w, DT, NULL) == LQR_ERR_INVALID);

    double A[4][4], B[4];
    m.l_eff_m = 0.0;
    assert(lqr_discretise(&m, DT, A, B) == LQR_ERR_INVALID);
}

// q_x = 0 leaves the wheel position an uncontrolled integrator (eigenvalue
// 1): not a stabilising K, so the solver must refuse rather than hand one out.
static void test_position_weight_zero_refused(void)
{
    lqr_weights_t w = DEFAULT_WEIGHTS;
    w.q[2] = 0.0;
    lqr_result_t res = {{7.0f, 7.0f, 7.0f, 7.0f}, 7.0, 7.0, 7};
    int st = lqr_solve(&DEFAULT_MODEL, &w, DT, &res);
    printf("  q_x = 0 -> status %d\n", st);
    assert(st == LQR_ERR_UNSTABLE || st == LQR_ERR_NO_CONVERGE);
    assert(k_untouched(&res));
}

// Weights spanning twelve orders of magnitude: a badly conditioned DARE.
// The solver must either return a stabilising K or refuse, within the
// LQR_MAX_ITER bound; it must never hand out a bad K.
static void test_extreme_weights_bounded(void)
{
    lqr_weights_t w = {{1e6, 1e-6, 1e6, 1e-6}, 1e-6};
    lqr_result_t res = {{7.0f, 7.0f, 7.0f, 7.0f}, 7.0, 7.0, 7};
    int st = lqr_solve(&DEFAULT_MODEL, &w, DT, &res);
    printf("  extreme weights -> status %d rho %.10f iter %d\n", st, res.rho, res.iterations);
    if (st == LQR_OK) {
        assert(res.rho < 1.0 && res.iterations <= LQR_MAX_ITER && res.residual < 1e-6);
    } else {
        assert(k_untouched(&res));
    }
}

int main(void)
{
    test_discretise_matches_series_expm();
    test_solve_matches_scipy_reference();
    test_sign_matches_pid();
    test_unit_helpers();
    test_invalid_inputs_fail_closed();
    test_position_weight_zero_refused();
    test_extreme_weights_bounded();
    printf("test_lqr: all tests passed\n");
    return 0;
}
