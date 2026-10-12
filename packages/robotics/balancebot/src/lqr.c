#include "lqr.h"

#include <math.h>
#include <string.h>

// Convergence: relative Frobenius change of P between doubling iterations.
#define LQR_TOL 1e-13
// Repeated squarings for the spectral-radius estimate: rho ~ ||M^N||^(1/N)
// with N = 2^LQR_RHO_SQUARINGS. An upper bound that tightens as N grows.
#define LQR_RHO_SQUARINGS 30

typedef double mat_t[LQR_N][LQR_N];

static void mat_mul(const mat_t a, const mat_t b, mat_t out)
{
    mat_t t;
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            double s = 0.0;
            for (int k = 0; k < LQR_N; k++) {
                s += a[i][k] * b[k][j];
            }
            t[i][j] = s;
        }
    }
    memcpy(out, t, sizeof(t));
}

static void mat_transpose(const mat_t a, mat_t out)
{
    mat_t t;
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            t[i][j] = a[j][i];
        }
    }
    memcpy(out, t, sizeof(t));
}

static double mat_frob(const mat_t a)
{
    double s = 0.0;
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            s += a[i][j] * a[i][j];
        }
    }
    return sqrt(s);
}

static int mat_finite(const mat_t a)
{
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            if (!isfinite(a[i][j])) {
                return 0;
            }
        }
    }
    return 1;
}

// Gauss-Jordan with partial pivoting. Returns 0 on a zero/non-finite pivot.
static int mat_inv(const mat_t a, mat_t out)
{
    double m[LQR_N][2 * LQR_N];
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            m[i][j] = a[i][j];
            m[i][LQR_N + j] = (i == j) ? 1.0 : 0.0;
        }
    }
    for (int c = 0; c < LQR_N; c++) {
        int p = c;
        for (int r = c + 1; r < LQR_N; r++) {
            if (fabs(m[r][c]) > fabs(m[p][c])) {
                p = r;
            }
        }
        if (!(fabs(m[p][c]) > 0.0) || !isfinite(m[p][c])) {
            return 0;
        }
        if (p != c) {
            for (int j = 0; j < 2 * LQR_N; j++) {
                double t = m[c][j];
                m[c][j] = m[p][j];
                m[p][j] = t;
            }
        }
        double inv_piv = 1.0 / m[c][c];
        for (int j = 0; j < 2 * LQR_N; j++) {
            m[c][j] *= inv_piv;
        }
        for (int r = 0; r < LQR_N; r++) {
            if (r == c) {
                continue;
            }
            double f = m[r][c];
            if (f != 0.0) {
                for (int j = 0; j < 2 * LQR_N; j++) {
                    m[r][j] -= f * m[c][j];
                }
            }
        }
    }
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            out[i][j] = m[i][LQR_N + j];
        }
    }
    return 1;
}

// Spectral radius upper bound via Gelfand's formula with normalised repeated
// squaring: robust for complex pairs and defective matrices, where a power
// iteration is not.
static double spectral_radius(const mat_t a)
{
    mat_t m;
    double s = mat_frob(a);
    if (!(s > 0.0)) {
        return 0.0;
    }
    double log_norm = log(s);  // log ||a^(2^k)|| scale carried separately from m
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            m[i][j] = a[i][j] / s;
        }
    }
    double n = 1.0;
    for (int k = 0; k < LQR_RHO_SQUARINGS; k++) {
        mat_mul(m, m, m);
        s = mat_frob(m);
        if (!(s > 0.0)) {
            return 0.0;  // nilpotent to working precision
        }
        for (int i = 0; i < LQR_N; i++) {
            for (int j = 0; j < LQR_N; j++) {
                m[i][j] /= s;
            }
        }
        log_norm = 2.0 * log_norm + log(s);
        n *= 2.0;
    }
    return exp(log_norm / n);
}

static int model_valid(const lqr_model_t *model, double dt)
{
    return model != NULL && isfinite(model->l_eff_m) && model->l_eff_m > 0.0 &&
           isfinite(model->g) && model->g > 0.0 && isfinite(dt) && dt > 0.0;
}

int lqr_discretise(const lqr_model_t *model, double dt, double A[LQR_N][LQR_N], double B[LQR_N])
{
    if (!model_valid(model, dt) || A == NULL || B == NULL) {
        return LQR_ERR_INVALID;
    }
    const double w = sqrt(model->g / model->l_eff_m);
    const double ch = cosh(w * dt);
    const double sh = sinh(w * dt);
    if (!isfinite(ch) || !isfinite(sh)) {
        return LQR_ERR_INVALID;
    }

    // Continuous A = [[0,1,0,0],[w^2,0,0,0],[0,0,0,1],[0,0,0,0]],
    // B = [0, -1/L, 0, 1]. The theta block is a hyperbolic rotation, the x
    // block a double integrator; they couple only through u.
    memset(A, 0, sizeof(double) * LQR_N * LQR_N);
    A[0][0] = ch;
    A[0][1] = sh / w;
    A[1][0] = w * sh;
    A[1][1] = ch;
    A[2][2] = 1.0;
    A[2][3] = dt;
    A[3][3] = 1.0;

    // B_d = integral_0^dt e^{A s} ds B. Theta part: -(1/L) [(ch-1)/w^2, sh/w],
    // and (1/L)/w^2 = 1/g.
    B[0] = -(ch - 1.0) / model->g;
    B[1] = -sh / (w * model->l_eff_m);
    B[2] = 0.5 * dt * dt;
    B[3] = dt;
    return LQR_OK;
}

static int weights_valid(const lqr_weights_t *w)
{
    if (w == NULL || !isfinite(w->r) || !(w->r > 0.0)) {
        return 0;
    }
    for (int i = 0; i < LQR_N; i++) {
        if (!isfinite(w->q[i]) || w->q[i] < 0.0) {
            return 0;
        }
    }
    return 1;
}

int lqr_solve(const lqr_model_t *model, const lqr_weights_t *weights, double dt, lqr_result_t *out)
{
    if (out == NULL || !weights_valid(weights)) {
        return LQR_ERR_INVALID;
    }
    mat_t A;
    double B[LQR_N];
    int st = lqr_discretise(model, dt, A, B);
    if (st != LQR_OK) {
        return st;
    }

    // Structure-preserving doubling (Chu, Fan, Lin 2005):
    //   W_k     = I + G_k H_k
    //   A_{k+1} = A_k W_k^-1 A_k
    //   G_{k+1} = G_k + A_k W_k^-1 G_k A_k^T
    //   H_{k+1} = H_k + A_k^T H_k W_k^-1 A_k
    // with A_0 = A, G_0 = B R^-1 B^T, H_0 = Q; H_k -> P quadratically.
    mat_t Ak, G, H;
    memcpy(Ak, A, sizeof(A));
    memset(H, 0, sizeof(H));
    for (int i = 0; i < LQR_N; i++) {
        H[i][i] = weights->q[i];
        for (int j = 0; j < LQR_N; j++) {
            G[i][j] = B[i] * B[j] / weights->r;
        }
    }

    int converged = 0;
    int iter = 0;
    while (iter < LQR_MAX_ITER && !converged) {
        iter++;
        mat_t W, Winv, AkT, t1, t2;
        mat_mul(G, H, W);
        for (int i = 0; i < LQR_N; i++) {
            W[i][i] += 1.0;
        }
        if (!mat_inv(W, Winv)) {
            return LQR_ERR_SINGULAR;
        }
        mat_transpose(Ak, AkT);

        mat_t Wi_A;  // W^-1 A_k, shared by the A and H updates
        mat_mul(Winv, Ak, Wi_A);

        mat_t Hn;  // H + A^T H W^-1 A
        mat_mul(H, Wi_A, t1);
        mat_mul(AkT, t1, t2);
        for (int i = 0; i < LQR_N; i++) {
            for (int j = 0; j < LQR_N; j++) {
                Hn[i][j] = H[i][j] + t2[i][j];
            }
        }

        mat_t Gn;  // G + A W^-1 G A^T
        mat_mul(G, AkT, t1);
        mat_mul(Winv, t1, t2);
        mat_mul(Ak, t2, t1);
        for (int i = 0; i < LQR_N; i++) {
            for (int j = 0; j < LQR_N; j++) {
                Gn[i][j] = G[i][j] + t1[i][j];
            }
        }

        mat_mul(Ak, Wi_A, Ak);

        mat_t d;
        for (int i = 0; i < LQR_N; i++) {
            for (int j = 0; j < LQR_N; j++) {
                d[i][j] = Hn[i][j] - H[i][j];
            }
        }
        if (!mat_finite(Hn) || !mat_finite(Gn) || !mat_finite(Ak)) {
            return LQR_ERR_NON_FINITE;
        }
        converged = mat_frob(d) <= LQR_TOL * (1.0 + mat_frob(Hn));
        // Symmetrise to keep rounding from accumulating an antisymmetric part.
        for (int i = 0; i < LQR_N; i++) {
            for (int j = 0; j < LQR_N; j++) {
                H[i][j] = 0.5 * (Hn[i][j] + Hn[j][i]);
                G[i][j] = 0.5 * (Gn[i][j] + Gn[j][i]);
            }
        }
    }
    if (!converged) {
        return LQR_ERR_NO_CONVERGE;
    }

    // K = (r + B^T P B)^-1 B^T P A
    const mat_t *P = (const mat_t *)&H;
    double PB[LQR_N];
    double btpb = 0.0;
    for (int i = 0; i < LQR_N; i++) {
        double s = 0.0;
        for (int j = 0; j < LQR_N; j++) {
            s += (*P)[i][j] * B[j];
        }
        PB[i] = s;
        btpb += B[i] * s;
    }
    const double denom = weights->r + btpb;
    if (!(denom > 0.0) || !isfinite(denom)) {
        return LQR_ERR_NON_FINITE;
    }
    double K[LQR_N];
    for (int j = 0; j < LQR_N; j++) {
        double s = 0.0;  // (B^T P A)_j = sum_i (P B)_i A_ij
        for (int i = 0; i < LQR_N; i++) {
            s += PB[i] * A[i][j];
        }
        K[j] = s / denom;
    }

    // Residual: A^T P A - P + Q - (A^T P B) K, relative to ||P||.
    mat_t AT, PA, R;
    mat_transpose(A, AT);
    mat_mul(*P, A, PA);
    mat_mul(AT, PA, R);
    double atpb[LQR_N];
    for (int i = 0; i < LQR_N; i++) {
        double s = 0.0;
        for (int k = 0; k < LQR_N; k++) {
            s += AT[i][k] * PB[k];
        }
        atpb[i] = s;
    }
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            R[i][j] += -(*P)[i][j] - atpb[i] * K[j];
        }
        R[i][i] += weights->q[i];
    }
    const double pn = mat_frob(*P);
    const double residual = mat_frob(R) / (pn > 1.0 ? pn : 1.0);

    mat_t Acl;
    for (int i = 0; i < LQR_N; i++) {
        for (int j = 0; j < LQR_N; j++) {
            Acl[i][j] = A[i][j] - B[i] * K[j];
        }
    }
    const double rho = spectral_radius(Acl);

    float Kf[LQR_N];
    for (int i = 0; i < LQR_N; i++) {
        Kf[i] = (float)K[i];
        if (!isfinite(K[i]) || !isfinite(Kf[i])) {
            return LQR_ERR_NON_FINITE;
        }
    }
    if (!isfinite(residual) || !isfinite(rho)) {
        return LQR_ERR_NON_FINITE;
    }
    if (!(rho < 1.0)) {
        return LQR_ERR_UNSTABLE;
    }

    memcpy(out->K, Kf, sizeof(Kf));
    out->rho = rho;
    out->residual = residual;
    out->iterations = iter;
    return LQR_OK;
}

float lqr_accel(const float K[LQR_N], float theta, float theta_dot, float x_err, float x_dot)
{
    return -(K[0] * theta + K[1] * theta_dot + K[2] * x_err + K[3] * x_dot);
}

#define LQR_DEG2RAD_F 0.017453292519943295f

float lqr_theta_from_pitch(float pitch_deg)
{
    return -pitch_deg * LQR_DEG2RAD_F;
}

float lqr_theta_dot_from_gyro(float gyro_dps)
{
    return -gyro_dps * LQR_DEG2RAD_F;
}

float lqr_steps_per_metre(float steps_per_rev, float wheel_diam_m)
{
    return steps_per_rev / (3.14159265358979f * wheel_diam_m);
}
