"""Reference LQR gains for tests/host/test_lqr.c.

Independent of src/lqr.c: discretises with scipy.linalg.expm (not the
closed-form cosh/sinh ZOH) and solves the DARE with
scipy.linalg.solve_discrete_are (not the doubling algorithm). The printed
constants are pasted into test_lqr.c; re-run after changing a parameter set:

    uv run --with scipy --with numpy python3 tests/host/lqr_reference.py

Model (see src/lqr.h): state [theta, theta_dot, x, x_dot] in SI, input
u = x_ddot (wheel acceleration, m/s^2), theta_ddot = (g*theta - u) / L_eff.
"""

import numpy as np
from scipy.linalg import expm, solve_discrete_are

G = 9.80665
DT = 1.0 / 500.0

# name, L_eff (m), q[4], r -- must match the CASES table in test_lqr.c
CASES = [
    ("default", 0.12, (400.0, 1.0, 25.0, 4.0), 0.04),
    ("short_l", 0.05, (400.0, 1.0, 25.0, 4.0), 0.04),
    ("heavy_r", 0.12, (400.0, 1.0, 25.0, 4.0), 4.0),
]


def discretise(l_eff: float, dt: float):
    a = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [G / l_eff, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0],
        ]
    )
    b = np.array([[0.0], [-1.0 / l_eff], [0.0], [1.0]])
    m = np.zeros((5, 5))
    m[:4, :4] = a
    m[:4, 4:] = b
    e = expm(m * dt)
    return e[:4, :4], e[:4, 4:]


def main() -> None:
    for name, l_eff, q, r in CASES:
        ad, bd = discretise(l_eff, DT)
        qm = np.diag(q)
        rm = np.array([[r]])
        p = solve_discrete_are(ad, bd, qm, rm)
        k = np.linalg.solve(rm + bd.T @ p @ bd, bd.T @ p @ ad)
        rho = max(abs(np.linalg.eigvals(ad - bd @ k)))
        ks = ", ".join(f"{v:.12e}" for v in k[0])
        print(f"/* {name} */ {{{ks}}}, /* rho = {rho:.12e} */")


if __name__ == "__main__":
    main()
