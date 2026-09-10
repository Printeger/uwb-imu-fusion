#!/usr/bin/env python3

import copy
import json
import math
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.paper.reference_recoverability import (  # noqa: E402
    DEFAULT_TOLERANCES,
    Tolerances,
    build_fixture_bundle,
    compare_fixture_bundles,
    compute_recoverability,
    main,
)


class RecoverabilityGoldenTest(unittest.TestCase):
    def assertMatrixClose(self, actual, expected):
        np.testing.assert_allclose(
            np.asarray(actual, dtype=float),
            np.asarray(expected, dtype=float),
            atol=DEFAULT_TOLERANCES.comparison_atol,
            rtol=DEFAULT_TOLERANCES.comparison_rtol,
        )

    def test_known_navigation_has_R_equal_N_and_eta_one(self):
        result = compute_recoverability(
            [[], [], []], [[2.0, 0.0], [0.0, 3.0], [1.0, 1.0]]
        )
        self.assertEqual(result["status"], "OK")
        self.assertEqual(result["rank"]["F_scaled"], 0)
        self.assertMatrixClose(result["R"], result["N"])
        self.assertAlmostEqual(result["eta"], 1.0)

    def test_scalar_confounding_has_invertible_A_but_zero_R(self):
        F = np.array([[1.0]])
        self.assertGreater(np.linalg.det(F.T @ F), 0.0)
        result = compute_recoverability(F, [[1.0]])
        self.assertMatrixClose(result["R"], [[0.0]])
        self.assertEqual(result["status"], "RANK_DEFICIENT")
        self.assertIsNone(result["s_m"])
        self.assertTrue(result["s_is_infinite"])

    def test_irrelevant_exact_zero_nuisance_column_does_not_destroy_information(self):
        result = compute_recoverability(
            [[1.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            [[0.0], [1.0], [1.0]],
        )
        self.assertEqual(result["column_equilibration"]["exact_zero_column_indices"], [1])
        self.assertEqual(result["rank"]["F_scaled"], 1)
        self.assertMatrixClose(result["N"], [[2.0]])
        self.assertMatrixClose(result["R"], [[2.0]])
        self.assertEqual(result["status"], "OK")

    def test_nonzero_nuisance_column_scaling_does_not_change_projection(self):
        F = np.array([[1.0, 2.0], [2.0, -1.0], [0.5, 3.0], [1.0, 0.0]])
        G = np.array([[0.0], [1.0], [1.0], [2.0]])
        baseline = compute_recoverability(F, G)
        rescaled = compute_recoverability(F @ np.diag([1.0e8, 1.0e-6]), G)
        self.assertMatrixClose(rescaled["N"], baseline["N"])
        self.assertMatrixClose(rescaled["R"], baseline["R"])
        self.assertAlmostEqual(rescaled["eta"], baseline["eta"], places=12)

    def test_extreme_nonzero_nuisance_columns_preserve_column_space(self):
        for magnitude in (1.0e-200, 1.0e200):
            with self.subTest(magnitude=magnitude):
                result = compute_recoverability([[magnitude]], [[1.0]])
                self.assertEqual(
                    result["column_equilibration"]["exact_zero_column_indices"], []
                )
                self.assertEqual(result["rank"]["F_scaled"], 1)
                self.assertMatrixClose(result["F_scaled"], [[1.0]])
                self.assertMatrixClose(result["R"], [[0.0]])
                self.assertEqual(result["status"], "RANK_DEFICIENT")
                self.assertTrue(result["s_is_infinite"])
                json.dumps(result, allow_nan=False)

    def test_unrepresentable_nuisance_dynamic_range_is_numerical_failure(self):
        result = compute_recoverability(
            [[1.0e200], [1.0e-200]],
            [[0.0], [1.0]],
        )
        self.assertEqual(result["status"], "NUMERICAL_FAILURE")
        self.assertEqual(
            result["status_detail"], "COLUMN_NORMALIZATION_UNDERFLOW:F_COLUMN_0"
        )
        json.dumps(result, allow_nan=False)

    def test_near_degenerate_multi_amplitude_exposes_rank_threshold_sensitivity(self):
        F = [[1.0, 1.0], [0.0, 1.0e-9], [0.0, 0.0]]
        G = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
        default = compute_recoverability(F, G)
        relaxed_rank = compute_recoverability(
            F, G, replace(DEFAULT_TOLERANCES, rank_rel=1.0e-8)
        )
        self.assertEqual(default["rank"]["F_scaled"], 2)
        self.assertEqual(default["rank"]["R"], 1)
        self.assertEqual(default["status"], "RANK_DEFICIENT")
        self.assertTrue(default["s_is_infinite"])
        self.assertEqual(relaxed_rank["rank"]["F_scaled"], 1)
        self.assertEqual(relaxed_rank["status"], "OK")
        self.assertGreater(
            relaxed_rank["spectrum"]["R_eigenvalues_asc_m2_inv"][0], 0.9
        )

    def test_high_eta_can_coexist_with_one_hundred_metre_s(self):
        result = compute_recoverability([[]], [[0.01]])
        self.assertMatrixClose(result["N"], [[1.0e-4]])
        self.assertMatrixClose(result["R"], [[1.0e-4]])
        self.assertAlmostEqual(result["eta"], 1.0)
        self.assertAlmostEqual(result["s_m"], 100.0)

    def test_large_scale_complete_confounding_cannot_create_finite_s(self):
        result = compute_recoverability(
            [[1.0], [1.0], [1.0]],
            [[1.0e10], [1.0e10], [1.0e10]],
        )
        residual_eigenvalue = result["spectrum"]["R_eigenvalues_asc_m2_inv"][0]
        projection_floor = result["thresholds_used"][
            "R_projection_roundoff_floor_m2_inv"
        ]
        self.assertGreater(residual_eigenvalue, 0.0)
        self.assertLessEqual(residual_eigenvalue, projection_floor)
        self.assertEqual(result["status"], "RANK_DEFICIENT")
        self.assertIsNone(result["s_m"])
        self.assertTrue(result["s_is_infinite"])

    def test_saved_orthogonal_roundoff_case_uses_N_R_scale(self):
        F = [
            [-0.7196126036616686],
            [-0.08898196796664617],
            [0.05933026952132415],
            [-0.20389520345987927],
            [-0.5791562235819454],
            [-0.30614480861013815],
        ]
        G = [
            [-30.040694659004963, -15.029897764651837],
            [-61.49306987426392, 18.166846263469424],
            [-12.81533686015166, -94.79174669264178],
            [8.01244984863999, 18.15789853162673],
            [5.02967351020658, 5.13308718007666],
            [71.15074978387733, -10.125945185438164],
        ]
        result = compute_recoverability(F, G)
        tolerance = result["thresholds_used"]["N_minus_R_psd_m2_inv"]
        minimum = result["spectrum"]["N_minus_R_eigenvalues_asc_m2_inv"][0]
        self.assertEqual(result["status"], "OK")
        self.assertGreaterEqual(minimum, -tolerance)
        self.assertGreaterEqual(-1.826067290073587e-12, -tolerance)
        self.assertEqual(
            result["audits"]["roundoff_model"]["scale_for_N_minus_R"],
            "spectral scales of N and R, not the spectrum of N-R",
        )
        unit_scale = compute_recoverability([[]], [[1.0]])
        self.assertLess(
            unit_scale["thresholds_used"]["N_minus_R_psd_m2_inv"],
            DEFAULT_TOLERANCES.psd_abs_m2_inv,
        )

        N = np.asarray(result["N"])
        R = np.asarray(result["R"])
        corrupted = R + np.eye(R.shape[0]) * (10.0 * tolerance)
        corrupted_minimum = np.linalg.eigvalsh(N - corrupted)[0]
        self.assertLess(corrupted_minimum, -tolerance)

    def test_free_static_beta_has_translation_ambiguity_with_dynamic_c(self):
        result = compute_recoverability(
            [[1.0], [1.0], [1.0]], [[1.0], [1.0], [1.0]]
        )
        beta, c, shift = 0.2, 0.7, 0.1
        self.assertAlmostEqual(beta + c, (beta - shift) + (c + shift))
        self.assertEqual(result["status"], "RANK_DEFICIENT")
        self.assertTrue(result["s_is_infinite"])

    def test_full_rank_projector_matches_schur(self):
        F = np.array(
            [
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
                [1.0, -1.0],
                [0.5, 0.25],
            ]
        )
        G = np.array(
            [
                [1.0, 0.0],
                [0.0, 1.0],
                [0.0, 0.0],
                [1.0, 1.0],
                [1.0, -0.5],
            ]
        )
        result = compute_recoverability(F, G)
        A = F.T @ F
        B = F.T @ G
        schur = G.T @ G - B.T @ np.linalg.solve(A, B)
        self.assertEqual(result["rank"]["F_scaled"], F.shape[1])
        self.assertMatrixClose(result["R"], schur)

    def test_future_information_increases_R_under_fixed_model(self):
        before = compute_recoverability([[1.0], [1.0]], [[0.0], [1.0]])
        after = compute_recoverability(
            [[1.0], [1.0], [math.sqrt(3.0)]], [[0.0], [1.0], [0.0]]
        )
        self.assertMatrixClose(before["N"], after["N"])
        self.assertMatrixClose(before["R"], [[0.5]])
        self.assertMatrixClose(after["R"], [[0.8]])
        self.assertGreater(after["eta"], before["eta"])
        self.assertLess(after["s_m"], before["s_m"])

    def test_dependent_amplitude_columns_invalidate_nominal_information(self):
        result = compute_recoverability([[], []], [[1.0, 1.0], [0.0, 0.0]])
        self.assertEqual(result["status"], "INVALID_NOMINAL_INFORMATION")
        self.assertIsNone(result["eta"])

    def test_nonfinite_input_is_rejected_not_regularized(self):
        with self.assertRaisesRegex(ValueError, "non-finite"):
            compute_recoverability([[float("nan")]], [[1.0]])

    def test_nonfinite_intermediate_is_numerical_failure_and_strict_json(self):
        result = compute_recoverability([[]], [[1.0e200]])
        self.assertEqual(result["status"], "NUMERICAL_FAILURE")
        self.assertEqual(result["status_detail"], "NONFINITE_INTERMEDIATE:N")
        json.dumps(result, allow_nan=False)

    def test_fixture_bundle_is_strict_json_and_marks_tolerances_as_numerical(self):
        bundle = build_fixture_bundle()
        serialized = json.dumps(bundle, allow_nan=False)
        parsed = json.loads(serialized)
        self.assertEqual(len(parsed["cases"]), 13)
        self.assertIn("not tau_eta", parsed["tolerance_scope"])
        near = next(
            case
            for case in parsed["cases"]
            if case["id"] == "near_degenerate_multi_amplitude"
        )
        self.assertEqual(
            near["sensitivity_status"], "TOLERANCE_SENSITIVE_NOT_GATE_ELIGIBLE"
        )
        statuses = [item["status"] for item in near["rank_tolerance_sensitivity"]]
        self.assertIn("RANK_DEFICIENT", statuses)
        self.assertIn("OK", statuses)

    def test_committed_fixture_matches_generator(self):
        fixture_path = (
            REPOSITORY_ROOT
            / "test"
            / "fixtures"
            / "recoverability"
            / "golden_cases.json"
        )
        committed = json.loads(fixture_path.read_text(encoding="utf-8"))
        differences = compare_fixture_bundles(committed, build_fixture_bundle())
        self.assertEqual(differences, [])

    def test_fixture_comparison_is_tolerant_only_for_float_fields(self):
        reference = build_fixture_bundle()
        numerical = copy.deepcopy(reference)
        numerical["cases"][0]["expected"]["N"][0][0] += (
            0.5 * DEFAULT_TOLERANCES.comparison_atol
        )
        self.assertEqual(compare_fixture_bundles(reference, numerical), [])

        mutations = []
        for path in ("status", "shape", "id", "structure"):
            candidate = copy.deepcopy(reference)
            if path == "status":
                candidate["cases"][0]["expected"]["status"] = "RANK_DEFICIENT"
            elif path == "shape":
                candidate["cases"][0]["expected"]["shape"]["rows"] += 1
            elif path == "id":
                candidate["cases"][0]["id"] = "different_case"
            else:
                del candidate["cases"][0]["expected"]["units"]
            mutations.append((path, candidate))
        for name, candidate in mutations:
            with self.subTest(name=name):
                self.assertTrue(compare_fixture_bundles(reference, candidate))

    def test_same_environment_generation_is_byte_identical(self):
        first = json.dumps(
            build_fixture_bundle(), sort_keys=True, allow_nan=False
        )
        second = json.dumps(
            build_fixture_bundle(), sort_keys=True, allow_nan=False
        )
        self.assertEqual(first, second)

    def test_cli_invalid_input_has_explicit_status(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            input_path = root / "bad.json"
            output_path = root / "result.json"
            input_path.write_text('{"F": [[1.0]], "G": [[1.0], [2.0]]}\n')
            exit_code = main(
                ["--input", str(input_path), "--output", str(output_path)]
            )
            result = json.loads(output_path.read_text())
        self.assertEqual(exit_code, 2)
        self.assertEqual(result["status"], "INVALID_INPUT")

    def test_tolerances_reject_unknown_or_negative_values(self):
        with self.assertRaisesRegex(ValueError, "unknown tolerance"):
            Tolerances.from_dict({"gate_threshold": 0.5})
        with self.assertRaisesRegex(ValueError, "nonnegative"):
            Tolerances.from_dict({"rank_abs": -1.0})
        with self.assertRaisesRegex(ValueError, "at least 1"):
            Tolerances.from_dict({"roundoff_safety_factor": 0.0})


if __name__ == "__main__":
    unittest.main()
