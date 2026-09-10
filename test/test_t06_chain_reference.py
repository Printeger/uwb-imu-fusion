#!/usr/bin/env python3
"""Independent SciPy epigraph-QP reference for the T06 chain solver."""

import argparse
import json
import subprocess
import unittest

import numpy as np
from scipy.optimize import minimize


CASES = [
    ([-0.2, 0.0, -0.1], [1.0, 3.0, 2.0], 0.1, 0.2),
    ([0.5, 0.5, 0.5, 0.5], [2.0] * 4, 0.1, 0.2),
    ([-0.2, 0.0, 0.2, 0.6, 1.0], [0.25, 1.0, 4.0, 16.0, 3.0], 0.0, 0.0),
    ([0.0, 0.8, 0.2, 1.1, 0.4], [0.5, 3.0, 1.0, 9.0, 2.0], 0.05, 0.1),
    ([1.0], [2.0], 0.6, 9.0),
]


def reference(e, w, lambda_l1, lambda_tv):
    e = np.asarray(e, dtype=float)
    w = np.asarray(w, dtype=float)
    n = e.size
    m = max(0, n - 1)

    def objective(x):
        u = x[:n]
        t = x[n:]
        return 0.5 * np.sum(w * (u - e) ** 2) + lambda_l1 * np.sum(u) + lambda_tv * np.sum(t)

    constraints = []
    for i in range(m):
        constraints.append({"type": "ineq", "fun": lambda x, i=i: x[n + i] - (x[i + 1] - x[i])})
        constraints.append({"type": "ineq", "fun": lambda x, i=i: x[n + i] + (x[i + 1] - x[i])})
    x0 = np.r_[np.maximum(e, 0.0), np.abs(np.diff(np.maximum(e, 0.0)))]
    solved = minimize(
        objective,
        x0,
        method="SLSQP",
        bounds=[(0.0, None)] * (n + m),
        constraints=constraints,
        options={"ftol": 1e-13, "maxiter": 5000},
    )
    if not solved.success:
        raise AssertionError(solved.message)
    u = solved.x[:n]
    t = solved.x[n:]
    if np.min(u) < -1e-10 or (m and np.max(np.abs(np.diff(u)) - t) > 1e-8):
        raise AssertionError("reference feasibility check failed")
    return u, objective(solved.x)


class ChainReferenceTest(unittest.TestCase):
    def run_cpp(self, e, w, l1, ltv, rho):
        command = [
            ARGS.solver,
            "--e", ",".join(map(str, e)),
            "--weights", ",".join(map(str, w)),
            "--lambda-l1", str(l1),
            "--lambda-tv", str(ltv),
            "--rho-scale", str(rho),
        ]
        completed = subprocess.run(command, check=False, text=True, capture_output=True)
        self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout)
        return json.loads(completed.stdout)

    def test_epigraph_reference_feasibility_objective_and_kkt(self):
        for e, w, l1, ltv in CASES:
            reference_u, reference_objective = reference(e, w, l1, ltv)
            rho_solutions = []
            for rho in (0.1, 1.0, 10.0):
                result = self.run_cpp(e, w, l1, ltv, rho)
                self.assertEqual(result["status"], "CONVERGED")
                u = np.asarray(result["u"])
                p = np.asarray(result["p"])
                self.assertGreaterEqual(np.min(u), 0.0)
                self.assertLessEqual(result["primal_residual_m"], result["primal_threshold_m"])
                self.assertLessEqual(result["dual_residual_objective_per_m"], result["dual_threshold_objective_per_m"])
                self.assertTrue(np.allclose(u, reference_u, atol=6e-6, rtol=2e-6))
                self.assertAlmostEqual(result["objective"], reference_objective, delta=2e-8)

                du = np.diff(u)
                if p.size:
                    self.assertLessEqual(np.max(np.abs(p)), ltv + 2.1e-7)
                    for d, dual in zip(du, p):
                        if d > 1e-10:
                            self.assertAlmostEqual(dual, ltv, delta=2.1e-7)
                        elif d < -1e-10:
                            self.assertAlmostEqual(dual, -ltv, delta=2.1e-7)
                dtp = np.zeros(len(u))
                if p.size:
                    dtp[:-1] -= p
                    dtp[1:] += p
                g = np.asarray(w) * (u - np.asarray(e)) + l1 + dtp
                for value, gradient in zip(u, g):
                    if value > 1e-10:
                        self.assertLessEqual(abs(gradient), 2.1e-7)
                    else:
                        self.assertGreaterEqual(gradient, -2.1e-7)
                rho_solutions.append(u)
            for candidate in rho_solutions[1:]:
                self.assertTrue(np.allclose(candidate, rho_solutions[0], atol=6e-6, rtol=2e-6))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--solver", required=True)
    ARGS, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__] + remaining)
