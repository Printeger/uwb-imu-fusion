#!/usr/bin/env python3
"""Small-matrix NumPy golden reference for recoverability information.

Inputs ``F`` and ``G`` are already whitened Jacobian blocks.  This reference is
deliberately independent of GTSAM and is not a locator or estimator.  It uses an
SVD basis for the numerically retained column space of equilibrated ``F`` and
never adds damping, jitter, priors, or discarded L1/TV curvature.
"""

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


SCHEMA_VERSION = "recoverability-golden-v2"


@dataclass(frozen=True)
class Tolerances:
    """Numerical-test tolerances, separate from any formal gate threshold."""

    rank_abs: float = 1.0e-12
    rank_rel: float = 1.0e-10
    symmetry_abs: float = 1.0e-12
    symmetry_rel: float = 1.0e-10
    projection_abs: float = 1.0e-12
    projection_rel: float = 1.0e-10
    psd_abs_m2_inv: float = 1.0e-12
    psd_rel: float = 1.0e-10
    pd_abs_m2_inv: float = 1.0e-12
    pd_rel: float = 1.0e-10
    eta_abs: float = 1.0e-12
    comparison_atol: float = 1.0e-10
    comparison_rtol: float = 1.0e-7
    roundoff_safety_factor: float = 8.0

    @classmethod
    def from_dict(cls, values: Optional[Dict[str, Any]]) -> "Tolerances":
        if values is None:
            return cls()
        known = set(asdict(cls()).keys())
        unknown = set(values.keys()) - known
        if unknown:
            raise ValueError("unknown tolerance fields: {}".format(sorted(unknown)))
        converted = {key: float(value) for key, value in values.items()}
        result = cls(**converted)
        for key, value in asdict(result).items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("tolerance {} must be finite and nonnegative".format(key))
        if result.roundoff_safety_factor < 1.0:
            raise ValueError("roundoff_safety_factor must be at least 1")
        return result


DEFAULT_TOLERANCES = Tolerances()


def _matrix(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.ndim != 2:
        raise ValueError("{} must be a two-dimensional matrix".format(name))
    if not np.all(np.isfinite(matrix)):
        raise ValueError("{} contains a non-finite value".format(name))
    return matrix


def _scaled_tolerance(absolute: float, relative: float, scale: float) -> float:
    return max(absolute, relative * abs(scale))


def _json_float(value: Optional[float]) -> Optional[float]:
    if value is None or not math.isfinite(float(value)):
        return None
    return float(value)


def _array_list(value: np.ndarray) -> List[Any]:
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)):
        raise RuntimeError("attempted to serialize a non-finite numerical result")
    return array.tolist()


class _NumericalIssue(RuntimeError):
    """An explicitly classified failure of a floating-point intermediate."""


def _require_finite(value: Any, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(array)):
        raise _NumericalIssue("NONFINITE_INTERMEDIATE:{}".format(name))
    return array


def _finite_matmul(left: np.ndarray, right: np.ndarray, name: str) -> np.ndarray:
    with np.errstate(over="ignore", invalid="ignore", under="ignore"):
        product = left @ right
    return _require_finite(product, name)


def _finite_subtract(left: np.ndarray, right: np.ndarray, name: str) -> np.ndarray:
    with np.errstate(over="ignore", invalid="ignore", under="ignore"):
        difference = left - right
    return _require_finite(difference, name)


def _finite_symmetric_part(matrix: np.ndarray, name: str) -> np.ndarray:
    with np.errstate(over="ignore", invalid="ignore", under="ignore"):
        symmetric = 0.5 * matrix + 0.5 * matrix.T
    return _require_finite(symmetric, name)


def _stable_frobenius_norm(value: np.ndarray, name: str) -> float:
    """Return a scale-safe Frobenius norm or classify an unrepresentable result."""

    array = _require_finite(value, name)
    if array.size == 0:
        return 0.0
    maximum = float(np.max(np.abs(array)))
    if maximum == 0.0:
        return 0.0
    scaled = _require_finite(array / maximum, name + "_NORM_SCALING")
    scaled_norm = float(np.sqrt(np.sum(scaled * scaled)))
    if not math.isfinite(scaled_norm) or scaled_norm <= 0.0:
        raise _NumericalIssue("UNRELIABLE_NORM:{}".format(name))
    if maximum > np.finfo(float).max / scaled_norm:
        raise _NumericalIssue("UNREPRESENTABLE_NORM:{}".format(name))
    result = maximum * scaled_norm
    if not math.isfinite(result):
        raise _NumericalIssue("UNRELIABLE_NORM:{}".format(name))
    return result


def _equilibrate_nuisance_columns(
    F: np.ndarray,
) -> Tuple[np.ndarray, List[Optional[float]], List[Optional[float]], List[int]]:
    """Normalize columns without first forming a vulnerable unscaled norm."""

    rows, columns = F.shape
    scaled_F = np.zeros((rows, columns), dtype=float)
    norms: List[Optional[float]] = []
    inverse_norms: List[Optional[float]] = []
    zero_columns: List[int] = []
    maximum_float = np.finfo(float).max

    for index in range(columns):
        column = F[:, index]
        if not np.any(column != 0.0):
            zero_columns.append(index)
            norms.append(0.0)
            inverse_norms.append(1.0)
            continue

        maximum = float(np.max(np.abs(column)))
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise _NumericalIssue("UNRELIABLE_COLUMN_SCALE:F_COLUMN_{}".format(index))
        first_stage = _require_finite(
            column / maximum, "F_COLUMN_{}_MAX_SCALING".format(index)
        )
        if np.any((column != 0.0) & (first_stage == 0.0)):
            raise _NumericalIssue(
                "COLUMN_NORMALIZATION_UNDERFLOW:F_COLUMN_{}".format(index)
            )
        squared_norm = float(np.sum(first_stage * first_stage))
        second_stage_norm = math.sqrt(squared_norm)
        if not math.isfinite(second_stage_norm) or second_stage_norm < 1.0:
            raise _NumericalIssue("UNRELIABLE_COLUMN_NORM:F_COLUMN_{}".format(index))
        scaled_F[:, index] = _require_finite(
            first_stage / second_stage_norm,
            "F_COLUMN_{}_NORMALIZATION".format(index),
        )
        if np.any((first_stage != 0.0) & (scaled_F[:, index] == 0.0)):
            raise _NumericalIssue(
                "COLUMN_NORMALIZATION_UNDERFLOW:F_COLUMN_{}".format(index)
            )

        if maximum <= maximum_float / second_stage_norm:
            norms.append(maximum * second_stage_norm)
        else:
            norms.append(None)

        with np.errstate(over="ignore", invalid="ignore", under="ignore"):
            if maximum >= 1.0:
                inverse = (1.0 / maximum) / second_stage_norm
            else:
                inverse = (1.0 / second_stage_norm) / maximum
        inverse_norms.append(
            inverse if math.isfinite(inverse) and inverse > 0.0 else None
        )

    _require_finite(scaled_F, "F_SCALED")
    return scaled_F, norms, inverse_norms, zero_columns


def _gamma(operation_count: int) -> float:
    """Higham-style gamma_k bound using IEEE-754 binary64 unit roundoff."""

    unit_roundoff = np.finfo(float).eps / 2.0
    product = max(1, int(operation_count)) * unit_roundoff
    if product >= 1.0:
        raise _NumericalIssue("ROUNDING_ERROR_MODEL_NOT_APPLICABLE")
    return product / (1.0 - product)


def _roundoff_bounds(
    row_count: int,
    rank_F: int,
    amplitude_cols: int,
    n_scale: float,
    r_scale: float,
    tolerances: Tolerances,
) -> Dict[str, float]:
    """Conservative small-matrix error bounds, separate from gate thresholds."""

    gram_gamma = _gamma(row_count)
    projector_operations = row_count + 2 * rank_F + amplitude_cols
    projector_gamma = _gamma(projector_operations)
    eig_gamma = _gamma(amplitude_cols)
    safety = tolerances.roundoff_safety_factor
    n_minus_r = safety * (
        (gram_gamma + eig_gamma) * (n_scale + r_scale)
        + 2.0 * projector_gamma * n_scale
    )
    g_operator_norm = math.sqrt(max(0.0, n_scale))
    projection_residual = safety * projector_gamma * g_operator_norm
    projection_information = projection_residual * projection_residual
    values = {
        "unit_roundoff": np.finfo(float).eps / 2.0,
        "gram_gamma": gram_gamma,
        "projector_gamma": projector_gamma,
        "eigen_gamma": eig_gamma,
        "N_minus_R_psd_m2_inv": n_minus_r,
        "projection_residual_m_inv": projection_residual,
        "projection_information_floor_m2_inv": projection_information,
    }
    _require_finite(list(values.values()), "ROUNDOFF_BOUNDS")
    return values


def _failure_result(
    detail: str,
    row_count: int,
    nuisance_cols: int,
    amplitude_cols: int,
    tolerances: Tolerances,
    column_equilibration: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": "NUMERICAL_FAILURE",
        "status_detail": detail,
        "shape": {
            "rows": row_count,
            "nuisance_columns": nuisance_cols,
            "amplitude_columns": amplitude_cols,
        },
        "tolerances": asdict(tolerances),
    }
    if column_equilibration is not None:
        result["column_equilibration"] = column_equilibration
    return result


def compute_recoverability(
    F: Any,
    G: Any,
    tolerances: Tolerances = DEFAULT_TOLERANCES,
) -> Dict[str, Any]:
    """Compute the rank-aware small-matrix reference result.

    The nuisance columns are deterministically 2-norm equilibrated.  Exact zero
    columns remain zero.  If ``U_r`` contains the retained left singular vectors,
    ``E = G - U_r (U_r.T G)`` and ``R = sym(E.T E)``.

    Status meanings:

    * ``OK``: all audits pass and R is positive definite at the stated tolerance.
    * ``RANK_DEFICIENT``: R has a direction at or below the PD tolerance; s is
      represented as JSON null plus ``s_is_infinite=true``.
    * ``INVALID_NOMINAL_INFORMATION``: N is not positive definite, so eta is
      undefined for the contracted amplitude parameterization.
    * ``NUMERICAL_FAILURE``: a decomposition fails or a symmetry, orthogonality,
      PSD, R<=N, or eta-range audit exceeds its declared tolerance.

    Invalid shapes, non-finite inputs, or invalid tolerance values raise
    ``ValueError``; the CLI reports these as ``INVALID_INPUT``.
    """

    F_array = _matrix(F, "F")
    G_array = _matrix(G, "G")
    if F_array.shape[0] != G_array.shape[0]:
        raise ValueError("F and G must have the same row count")
    if G_array.shape[0] == 0:
        raise ValueError("F and G must contain at least one row")
    if G_array.shape[1] == 0:
        raise ValueError("G must contain at least one amplitude column")

    row_count, nuisance_cols = F_array.shape
    amplitude_cols = G_array.shape[1]
    try:
        F_scaled, column_norms, column_scales, zero_columns = (
            _equilibrate_nuisance_columns(F_array)
        )
        column_equilibration = {
            "rule": (
                "nonzero F columns divided by max-absolute value then by the "
                "scaled 2-norm; exact zero is decided from original elements"
            ),
            "mathematical_scale": "d_j=1/||F[:,j]||_2",
            "application": "two_stage_scale_safe_division",
            "original_column_norms": column_norms,
            "column_scales": column_scales,
            "null_numeric_meaning": "finite binary64 representation unavailable",
            "exact_zero_column_indices": zero_columns,
        }
        U, singular_values, _ = np.linalg.svd(F_scaled, full_matrices=False)
        _require_finite(U, "F_SVD_LEFT_VECTORS")
        _require_finite(singular_values, "F_SVD_SINGULAR_VALUES")
    except (_NumericalIssue, np.linalg.LinAlgError) as error:
        detail = str(error)
        if isinstance(error, np.linalg.LinAlgError):
            detail = "SVD_FAILED:{}".format(error)
        return _failure_result(
            detail, row_count, nuisance_cols, amplitude_cols, tolerances
        )

    sigma_max = float(singular_values[0]) if singular_values.size else 0.0
    rank_threshold = _scaled_tolerance(
        tolerances.rank_abs, tolerances.rank_rel, sigma_max
    )
    rank_F = int(np.count_nonzero(singular_values > rank_threshold))
    try:
        retained_basis = U[:, :rank_F]
        projected_coordinates = _finite_matmul(
            retained_basis.T, G_array, "PROJECTED_COORDINATES"
        )
        projected_G = _finite_matmul(
            retained_basis, projected_coordinates, "PROJECTED_G"
        )
        E = _finite_subtract(G_array, projected_G, "E")
        N = _finite_matmul(G_array.T, G_array, "N")
        R_raw = _finite_matmul(E.T, E, "R_RAW")

        symmetry_difference = _finite_subtract(
            R_raw, R_raw.T, "R_SYMMETRY_DIFFERENCE"
        )
        symmetry_error = (
            float(np.max(np.abs(symmetry_difference)))
            if symmetry_difference.size
            else 0.0
        )
        symmetry_scale = float(np.max(np.abs(R_raw))) if R_raw.size else 0.0
        symmetry_tolerance = _scaled_tolerance(
            tolerances.symmetry_abs, tolerances.symmetry_rel, symmetry_scale
        )
        R = _finite_symmetric_part(R_raw, "R_SYMMETRIZED")

        retained_normal = _finite_matmul(
            retained_basis.T, E, "RETAINED_PROJECTOR_NORMAL_RESIDUAL"
        )
        retained_orthogonality = _stable_frobenius_norm(
            retained_normal, "RETAINED_PROJECTOR_NORMAL_RESIDUAL"
        )
        projection_scale = _stable_frobenius_norm(G_array, "G")
        projection_tolerance = _scaled_tolerance(
            tolerances.projection_abs, tolerances.projection_rel, projection_scale
        )
        full_scaled_normal = _finite_matmul(
            F_scaled.T, E, "FULL_SCALED_F_NORMAL_RESIDUAL"
        )
        full_scaled_normal_residual = _stable_frobenius_norm(
            full_scaled_normal, "FULL_SCALED_F_NORMAL_RESIDUAL"
        )

        eig_N = np.linalg.eigvalsh(_finite_symmetric_part(N, "N_SYMMETRIZED"))
        eig_R = np.linalg.eigvalsh(R)
        N_minus_R = _finite_subtract(N, R, "N_MINUS_R")
        eig_N_minus_R = np.linalg.eigvalsh(
            _finite_symmetric_part(N_minus_R, "N_MINUS_R_SYMMETRIZED")
        )
        _require_finite(eig_N, "N_EIGENVALUES")
        _require_finite(eig_R, "R_EIGENVALUES")
        _require_finite(eig_N_minus_R, "N_MINUS_R_EIGENVALUES")
    except (_NumericalIssue, np.linalg.LinAlgError) as error:
        detail = str(error)
        if isinstance(error, np.linalg.LinAlgError):
            detail = "EIGEN_DECOMPOSITION_FAILED:{}".format(error)
        return _failure_result(
            detail,
            row_count,
            nuisance_cols,
            amplitude_cols,
            tolerances,
            column_equilibration,
        )

    n_scale = float(np.max(np.abs(eig_N))) if eig_N.size else 0.0
    r_scale = float(np.max(np.abs(eig_R))) if eig_R.size else 0.0
    n_pd_tolerance = _scaled_tolerance(
        tolerances.pd_abs_m2_inv, tolerances.pd_rel, n_scale
    )
    try:
        roundoff = _roundoff_bounds(
            row_count, rank_F, amplitude_cols, n_scale, r_scale, tolerances
        )
    except _NumericalIssue as error:
        return _failure_result(
            str(error),
            row_count,
            nuisance_cols,
            amplitude_cols,
            tolerances,
            column_equilibration,
        )
    r_pd_tolerance = max(
        _scaled_tolerance(tolerances.pd_abs_m2_inv, tolerances.pd_rel, r_scale),
        roundoff["projection_information_floor_m2_inv"],
    )
    r_psd_tolerance = _scaled_tolerance(
        tolerances.psd_abs_m2_inv, tolerances.psd_rel, r_scale
    )
    n_minus_r_psd_tolerance = roundoff["N_minus_R_psd_m2_inv"]
    rank_N = int(np.count_nonzero(eig_N > n_pd_tolerance))
    rank_R = int(np.count_nonzero(eig_R > r_pd_tolerance))

    status = "OK"
    details: List[str] = []
    eta_raw: Optional[float] = None
    eta: Optional[float] = None

    if symmetry_error > symmetry_tolerance:
        status = "NUMERICAL_FAILURE"
        details.append("R_SYMMETRY_AUDIT_FAILED")
    if retained_orthogonality > projection_tolerance:
        status = "NUMERICAL_FAILURE"
        details.append("RETAINED_PROJECTOR_ORTHOGONALITY_FAILED")
    if eig_R[0] < -r_psd_tolerance:
        status = "NUMERICAL_FAILURE"
        details.append("R_PSD_AUDIT_FAILED")
    if eig_N_minus_R[0] < -n_minus_r_psd_tolerance:
        status = "NUMERICAL_FAILURE"
        details.append("R_LE_N_AUDIT_FAILED")

    if eig_N[0] <= n_pd_tolerance:
        if status != "NUMERICAL_FAILURE":
            status = "INVALID_NOMINAL_INFORMATION"
        details.append("N_NOT_POSITIVE_DEFINITE")
        generalized_spectrum: List[float] = []
    else:
        try:
            eigenvalues_N, eigenvectors_N = np.linalg.eigh(N)
            N_inverse_half = _finite_matmul(
                _finite_matmul(
                    eigenvectors_N,
                    np.diag(1.0 / np.sqrt(eigenvalues_N)),
                    "N_INVERSE_HALF_LEFT",
                ),
                eigenvectors_N.T,
                "N_INVERSE_HALF",
            )
            normalized_R = _finite_matmul(
                _finite_matmul(N_inverse_half, R, "NORMALIZED_R_LEFT"),
                N_inverse_half,
                "NORMALIZED_R",
            )
            normalized_R = _finite_symmetric_part(
                normalized_R, "NORMALIZED_R_SYMMETRIZED"
            )
            generalized = np.linalg.eigvalsh(normalized_R)
            _require_finite(generalized, "GENERALIZED_EIGENVALUES")
            generalized_spectrum = [float(value) for value in generalized]
            eta_raw = float(generalized[0])
            if eta_raw < -tolerances.eta_abs or eta_raw > 1.0 + tolerances.eta_abs:
                status = "NUMERICAL_FAILURE"
                details.append("ETA_RANGE_AUDIT_FAILED")
            else:
                eta = min(1.0, max(0.0, eta_raw))
        except (_NumericalIssue, np.linalg.LinAlgError) as error:
            status = "NUMERICAL_FAILURE"
            details.append("GENERALIZED_EIGENPROBLEM_FAILED: {}".format(error))
            generalized_spectrum = []

    r_is_pd = bool(eig_R[0] > r_pd_tolerance)
    if status == "OK" and not r_is_pd:
        status = "RANK_DEFICIENT"
        details.append("R_NOT_POSITIVE_DEFINITE_AT_DECLARED_TOLERANCE")
    s_m = 1.0 / math.sqrt(float(eig_R[0])) if r_is_pd else None

    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "status_detail": ";".join(details) if details else "ALL_NUMERICAL_AUDITS_PASSED",
        "shape": {
            "rows": row_count,
            "nuisance_columns": nuisance_cols,
            "amplitude_columns": amplitude_cols,
        },
        "units": {
            "F": "whitened residual per nuisance local coordinate",
            "G": "m^-1",
            "N": "m^-2",
            "R": "m^-2",
            "eta": "1",
            "s": "m",
        },
        "column_equilibration": column_equilibration,
        "rank": {
            "F_scaled": rank_F,
            "N": rank_N,
            "R": rank_R,
        },
        "spectrum": {
            "F_scaled_singular_values_desc": _array_list(singular_values),
            "N_eigenvalues_asc_m2_inv": _array_list(eig_N),
            "R_eigenvalues_asc_m2_inv": _array_list(eig_R),
            "N_minus_R_eigenvalues_asc_m2_inv": _array_list(eig_N_minus_R),
            "generalized_R_N_eigenvalues_asc": generalized_spectrum,
        },
        "thresholds_used": {
            "F_rank": rank_threshold,
            "N_positive_definite_m2_inv": n_pd_tolerance,
            "R_positive_definite_m2_inv": r_pd_tolerance,
            "R_psd_m2_inv": r_psd_tolerance,
            "N_minus_R_psd_m2_inv": n_minus_r_psd_tolerance,
            "R_symmetry": symmetry_tolerance,
            "retained_projection_orthogonality": projection_tolerance,
            "projection_residual_roundoff_m_inv": roundoff[
                "projection_residual_m_inv"
            ],
            "R_projection_roundoff_floor_m2_inv": roundoff[
                "projection_information_floor_m2_inv"
            ],
        },
        "audits": {
            "R_symmetry_error": symmetry_error,
            "retained_projection_orthogonality": retained_orthogonality,
            "full_scaled_F_normal_residual_diagnostic": full_scaled_normal_residual,
            "N_minus_R_min_eigenvalue_m2_inv": float(eig_N_minus_R[0]),
            "roundoff_model": {
                "model": "binary64 gamma_k forward-error bound",
                "scale_for_N_minus_R": (
                    "spectral scales of N and R, not the spectrum of N-R"
                ),
                "safety_factor": tolerances.roundoff_safety_factor,
                **roundoff,
            },
        },
        "F_scaled": _array_list(F_scaled),
        "E": _array_list(E),
        "N": _array_list(N),
        "R": _array_list(R),
        "eta_raw": _json_float(eta_raw),
        "eta": _json_float(eta),
        "s_m": _json_float(s_m),
        "s_is_infinite": not r_is_pd,
        "tolerances": asdict(tolerances),
    }


def _fixture_inputs() -> List[Dict[str, Any]]:
    epsilon = 1.0e-9
    return [
        {
            "id": "known_navigation",
            "description": "No nuisance columns: R=N and eta=1.",
            "F": [[], [], []],
            "G": [[2.0, 0.0], [0.0, 3.0], [1.0, 1.0]],
            "analytic": {"status": "OK", "eta": 1.0},
        },
        {
            "id": "scalar_confounding",
            "description": "z=x+c: A=1 is invertible but G lies in col(F).",
            "F": [[1.0]],
            "G": [[1.0]],
            "analytic": {
                "A_invertible": True,
                "R": [[0.0]],
                "status": "RANK_DEFICIENT",
                "s_is_infinite": True,
            },
        },
        {
            "id": "extreme_small_nuisance_scale",
            "description": "A nonzero 1e-200 nuisance column must not underflow to an exact-zero column.",
            "F": [[1.0e-200]],
            "G": [[1.0]],
            "analytic": {
                "rank_F_scaled": 1,
                "R": [[0.0]],
                "status": "RANK_DEFICIENT",
                "s_is_infinite": True,
            },
        },
        {
            "id": "extreme_large_nuisance_scale",
            "description": "A nonzero 1e200 nuisance column must normalize without overflow.",
            "F": [[1.0e200]],
            "G": [[1.0]],
            "analytic": {
                "rank_F_scaled": 1,
                "R": [[0.0]],
                "status": "RANK_DEFICIENT",
                "s_is_infinite": True,
            },
        },
        {
            "id": "scaled_complete_confounding",
            "description": "G=1e10 F is exactly confounded; projection roundoff cannot create finite information.",
            "F": [[1.0], [1.0], [1.0]],
            "G": [[1.0e10], [1.0e10], [1.0e10]],
            "analytic": {
                "mathematical_R": [[0.0]],
                "status": "RANK_DEFICIENT",
                "s_is_infinite": True,
            },
        },
        {
            "id": "orthogonal_roundoff_psd",
            "description": "Saved orthogonal case whose computed N-R has a tiny negative eigenvalue within the binary64 bound.",
            "F": [
                [-0.7196126036616686],
                [-0.08898196796664617],
                [0.05933026952132415],
                [-0.20389520345987927],
                [-0.5791562235819454],
                [-0.30614480861013815],
            ],
            "G": [
                [-30.040694659004963, -15.029897764651837],
                [-61.49306987426392, 18.166846263469424],
                [-12.81533686015166, -94.79174669264178],
                [8.01244984863999, 18.15789853162673],
                [5.02967351020658, 5.13308718007666],
                [71.15074978387733, -10.125945185438164],
            ],
            "analytic": {
                "N": [[10000.0, 0.0], [0.0, 10000.0]],
                "R": [[10000.0, 0.0], [0.0, 10000.0]],
                "status": "OK",
                "roundoff_may_make_N_minus_R_indefinite": True,
            },
        },
        {
            "id": "irrelevant_nuisance_nullspace",
            "description": "An exact zero nuisance column does not destroy independent amplitude information.",
            "F": [[1.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            "G": [[0.0], [1.0], [1.0]],
            "analytic": {
                "rank_F_scaled": 1,
                "exact_zero_column_indices": [1],
                "N": [[2.0]],
                "R": [[2.0]],
                "eta": 1.0,
                "status": "OK",
            },
        },
        {
            "id": "near_degenerate_multi_amplitude",
            "description": "A near-null nuisance direction can absorb one of two amplitudes when retained.",
            "F": [[1.0, 1.0], [0.0, epsilon], [0.0, 0.0]],
            "G": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            "analytic": {
                "default_status": "RANK_DEFICIENT",
                "default_rank_F_scaled": 2,
                "tolerance_sensitive": True,
                "gate_eligible_from_sweep": False,
            },
        },
        {
            "id": "weak_absolute_information",
            "description": "N=R=1e-4 m^-2 gives eta=1 but s=100 m.",
            "F": [[]],
            "G": [[0.01]],
            "analytic": {
                "N": [[1.0e-4]],
                "R": [[1.0e-4]],
                "eta": 1.0,
                "s_m": 100.0,
                "status": "OK",
            },
        },
        {
            "id": "static_beta_translation_ambiguity",
            "description": "A free static beta and dynamic c share the same measurement direction.",
            "F": [[1.0], [1.0], [1.0]],
            "G": [[1.0], [1.0], [1.0]],
            "analytic": {
                "invariance_example_m": {
                    "beta": 0.2,
                    "c": 0.7,
                    "shift_d": 0.1,
                    "beta_shifted": 0.1,
                    "c_shifted": 0.8,
                },
                "status": "RANK_DEFICIENT",
                "s_is_infinite": True,
            },
        },
        {
            "id": "full_rank_projector_schur",
            "description": "Well-conditioned full-column-rank F for projector/Schur equivalence.",
            "F": [
                [1.0, 0.0],
                [0.0, 1.0],
                [1.0, 1.0],
                [1.0, -1.0],
                [0.5, 0.25],
            ],
            "G": [
                [1.0, 0.0],
                [0.0, 1.0],
                [0.0, 0.0],
                [1.0, 1.0],
                [1.0, -0.5],
            ],
            "analytic": {"status": "OK", "projector_equals_schur": True},
        },
        {
            "id": "future_information_base",
            "description": "Historical scalar model with navigation precision a=1 and candidate precision w=1.",
            "F": [[1.0], [1.0]],
            "G": [[0.0], [1.0]],
            "analytic": {"R": [[0.5]], "eta": 0.5, "s_m": math.sqrt(2.0)},
        },
        {
            "id": "future_information_augmented",
            "description": "Same historical model plus future navigation precision f=3; G is zero on the new row.",
            "F": [[1.0], [1.0], [math.sqrt(3.0)]],
            "G": [[0.0], [1.0], [0.0]],
            "analytic": {
                "R": [[0.8]],
                "eta": 0.8,
                "s_m": math.sqrt(1.25),
                "same_historical_amplitude_model": True,
            },
        },
    ]


def build_fixture_bundle(
    tolerances: Tolerances = DEFAULT_TOLERANCES,
) -> Dict[str, Any]:
    cases: List[Dict[str, Any]] = []
    for fixture in _fixture_inputs():
        result = compute_recoverability(fixture["F"], fixture["G"], tolerances)
        case = {
            "id": fixture["id"],
            "description": fixture["description"],
            "F": fixture["F"],
            "G": fixture["G"],
            "N": result["N"],
            "R": result["R"],
            "expected": result,
            "analytic_expectation": fixture["analytic"],
        }
        if fixture["id"] == "full_rank_projector_schur":
            F_array = _matrix(fixture["F"], "F")
            G_array = _matrix(fixture["G"], "G")
            A = F_array.T @ F_array
            B = F_array.T @ G_array
            schur = G_array.T @ G_array - B.T @ np.linalg.solve(A, B)
            case["schur_R"] = _array_list(0.5 * (schur + schur.T))
        if fixture["id"] == "near_degenerate_multi_amplitude":
            sweep = []
            for relative_tolerance in (1.0e-12, 1.0e-10, 1.0e-8):
                selected = replace(tolerances, rank_rel=relative_tolerance)
                sweep_result = compute_recoverability(
                    fixture["F"], fixture["G"], selected
                )
                sweep.append(
                    {
                        "rank_rel": relative_tolerance,
                        "rank_F_scaled": sweep_result["rank"]["F_scaled"],
                        "rank_R": sweep_result["rank"]["R"],
                        "R_eigenvalues_asc_m2_inv": sweep_result["spectrum"][
                            "R_eigenvalues_asc_m2_inv"
                        ],
                        "status": sweep_result["status"],
                        "eta": sweep_result["eta"],
                        "s_m": sweep_result["s_m"],
                        "s_is_infinite": sweep_result["s_is_infinite"],
                    }
                )
            case["rank_tolerance_sensitivity"] = sweep
            case["sensitivity_status"] = "TOLERANCE_SENSITIVE_NOT_GATE_ELIGIBLE"
        cases.append(case)

    return {
        "schema_version": SCHEMA_VERSION,
        "generator": "tools/paper/reference_recoverability.py --generate-fixtures",
        "input_contract": "F and G are already whitened; G remains in physical metre amplitude coordinates.",
        "forbidden_information": [
            "LM damping",
            "diagonal jitter",
            "artificial prior",
            "discarded L1 curvature",
            "discarded TV curvature",
        ],
        "tolerance_scope": (
            "These are numerical golden-test tolerances only. They are not tau_eta, "
            "tau_s, tau_gamma, validation-locked gates, or experiment parameters."
        ),
        "comparison_policy": {
            "floating_point_fields": "comparison_atol/comparison_rtol",
            "strict_fields": "case identity, status, dimensions, types, keys, list lengths, and ordering",
            "byte_identity": "same-environment reproducibility check only",
        },
        "roundoff_policy": {
            "N_minus_R": (
                "binary64 gamma_k bound scaled by N and R spectra; no fixed "
                "absolute floor and no scaling by the N-R spectrum"
            ),
            "R_rank": (
                "declared PD tolerance plus squared projector forward-error "
                "bound; unresolved projection residuals cannot yield finite s"
            ),
        },
        "tolerances": asdict(tolerances),
        "status_definitions": {
            "OK": "audits pass and R is positive definite at the declared numerical tolerance",
            "RANK_DEFICIENT": "R has a direction at/below the PD tolerance; s is infinite",
            "INVALID_NOMINAL_INFORMATION": "N is not positive definite, so eta is undefined",
            "NUMERICAL_FAILURE": "decomposition or declared numerical audit failed",
            "INVALID_INPUT": "CLI input shape, finiteness, or tolerance validation failed",
        },
        "relations": {
            "future_information": {
                "base_case": "future_information_base",
                "augmented_case": "future_information_augmented",
                "expected": "N unchanged, R and eta increase, s decreases",
            }
        },
        "cases": cases,
    }


def _assert_close(actual: Any, expected: Any, tolerances: Tolerances) -> None:
    np.testing.assert_allclose(
        np.asarray(actual, dtype=float),
        np.asarray(expected, dtype=float),
        atol=tolerances.comparison_atol,
        rtol=tolerances.comparison_rtol,
    )


def compare_fixture_bundles(
    reference: Any,
    candidate: Any,
    tolerances: Optional[Tolerances] = None,
) -> List[str]:
    """Compare fixture structure strictly and floating-point leaves tolerantly."""

    if tolerances is None:
        if isinstance(reference, dict) and isinstance(
            reference.get("tolerances"), dict
        ):
            tolerances = Tolerances.from_dict(reference["tolerances"])
        else:
            tolerances = DEFAULT_TOLERANCES
    differences: List[str] = []

    def compare(expected: Any, actual: Any, path: str) -> None:
        if isinstance(expected, dict):
            if not isinstance(actual, dict):
                differences.append(
                    "{}: expected object, got {}".format(path, type(actual).__name__)
                )
                return
            expected_keys = set(expected)
            actual_keys = set(actual)
            if expected_keys != actual_keys:
                differences.append(
                    "{}: key mismatch missing={} extra={}".format(
                        path,
                        sorted(expected_keys - actual_keys),
                        sorted(actual_keys - expected_keys),
                    )
                )
                return
            for key in expected:
                compare(expected[key], actual[key], "{}.{}".format(path, key))
            return

        if isinstance(expected, list):
            if not isinstance(actual, list):
                differences.append(
                    "{}: expected list, got {}".format(path, type(actual).__name__)
                )
                return
            if len(expected) != len(actual):
                differences.append(
                    "{}: list length {} != {}".format(path, len(expected), len(actual))
                )
                return
            for index, (expected_item, actual_item) in enumerate(
                zip(expected, actual)
            ):
                compare(expected_item, actual_item, "{}[{}]".format(path, index))
            return

        if isinstance(expected, float):
            if not isinstance(actual, float) or not math.isfinite(actual):
                differences.append(
                    "{}: expected finite float, got {!r}".format(path, actual)
                )
                return
            if not math.isclose(
                actual,
                expected,
                rel_tol=tolerances.comparison_rtol,
                abs_tol=tolerances.comparison_atol,
            ):
                differences.append(
                    "{}: {} != {} at atol={} rtol={}".format(
                        path,
                        actual,
                        expected,
                        tolerances.comparison_atol,
                        tolerances.comparison_rtol,
                    )
                )
            return

        if type(expected) is not type(actual) or expected != actual:
            differences.append(
                "{}: strict value/type mismatch {!r} != {!r}".format(
                    path, actual, expected
                )
            )

    compare(reference, candidate, "$")
    return differences


def run_self_test(tolerances: Tolerances = DEFAULT_TOLERANCES) -> None:
    fixtures = {fixture["id"]: fixture for fixture in _fixture_inputs()}
    results = {
        name: compute_recoverability(item["F"], item["G"], tolerances)
        for name, item in fixtures.items()
    }

    known = results["known_navigation"]
    _assert_close(known["R"], known["N"], tolerances)
    _assert_close(known["eta"], 1.0, tolerances)

    scalar = results["scalar_confounding"]
    _assert_close(scalar["R"], [[0.0]], tolerances)
    assert scalar["status"] == "RANK_DEFICIENT" and scalar["s_is_infinite"]

    for name in ("extreme_small_nuisance_scale", "extreme_large_nuisance_scale"):
        extreme = results[name]
        assert extreme["rank"]["F_scaled"] == 1
        assert extreme["column_equilibration"]["exact_zero_column_indices"] == []
        _assert_close(extreme["F_scaled"], [[1.0]], tolerances)
        assert extreme["status"] == "RANK_DEFICIENT"

    scaled_confounding = results["scaled_complete_confounding"]
    assert scaled_confounding["status"] == "RANK_DEFICIENT"
    assert scaled_confounding["s_is_infinite"]
    assert scaled_confounding["s_m"] is None
    assert (
        scaled_confounding["spectrum"]["R_eigenvalues_asc_m2_inv"][0]
        <= scaled_confounding["thresholds_used"][
            "R_projection_roundoff_floor_m2_inv"
        ]
    )

    orthogonal = results["orthogonal_roundoff_psd"]
    assert orthogonal["status"] == "OK"
    assert (
        orthogonal["spectrum"]["N_minus_R_eigenvalues_asc_m2_inv"][0]
        >= -orthogonal["thresholds_used"]["N_minus_R_psd_m2_inv"]
    )

    irrelevant = results["irrelevant_nuisance_nullspace"]
    _assert_close(irrelevant["R"], irrelevant["N"], tolerances)
    assert irrelevant["status"] == "OK"
    assert irrelevant["column_equilibration"]["exact_zero_column_indices"] == [1]

    near = results["near_degenerate_multi_amplitude"]
    assert near["status"] == "RANK_DEFICIENT"
    assert near["rank"]["F_scaled"] == 2 and near["rank"]["R"] == 1
    loose_rank = compute_recoverability(
        fixtures["near_degenerate_multi_amplitude"]["F"],
        fixtures["near_degenerate_multi_amplitude"]["G"],
        replace(tolerances, rank_rel=1.0e-8),
    )
    assert loose_rank["rank"]["F_scaled"] == 1 and loose_rank["status"] == "OK"

    weak = results["weak_absolute_information"]
    _assert_close(weak["N"], [[1.0e-4]], tolerances)
    _assert_close(weak["R"], [[1.0e-4]], tolerances)
    _assert_close(weak["eta"], 1.0, tolerances)
    _assert_close(weak["s_m"], 100.0, tolerances)

    beta = results["static_beta_translation_ambiguity"]
    assert beta["status"] == "RANK_DEFICIENT" and beta["s_is_infinite"]
    _assert_close(0.2 + 0.7, (0.2 - 0.1) + (0.7 + 0.1), tolerances)

    full = fixtures["full_rank_projector_schur"]
    F_full = _matrix(full["F"], "F")
    G_full = _matrix(full["G"], "G")
    A = F_full.T @ F_full
    B = F_full.T @ G_full
    schur = G_full.T @ G_full - B.T @ np.linalg.solve(A, B)
    _assert_close(results["full_rank_projector_schur"]["R"], schur, tolerances)

    before = results["future_information_base"]
    after = results["future_information_augmented"]
    _assert_close(before["N"], after["N"], tolerances)
    _assert_close(before["R"], [[0.5]], tolerances)
    _assert_close(after["R"], [[0.8]], tolerances)
    assert after["eta"] > before["eta"]
    assert after["s_m"] < before["s_m"]

    print("PASS: 12 contracted/review recoverability scenarios (13 matrices)")
    print("PASS: extreme nuisance scaling, roundoff PSD, and complete-confounding regressions")
    print("PASS: near-degenerate rank-threshold sensitivity recorded, not accepted as a gate")
    print("PASS: no damping, jitter, artificial prior, L1/TV curvature, or finite pseudoinverse variance")


def _write_json(payload: Dict[str, Any], output: Optional[Path]) -> None:
    serialized = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if output is None:
        sys.stdout.write(serialized)
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialized, encoding="utf-8")


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--input", type=Path, help="JSON object with F, G, and optional tolerances")
    action.add_argument("--self-test", action="store_true", help="run the analytic golden checks")
    action.add_argument(
        "--generate-fixtures",
        type=Path,
        metavar="PATH",
        help="write the machine-readable C++ comparison fixture bundle",
    )
    action.add_argument(
        "--compare-fixtures",
        nargs=2,
        type=Path,
        metavar=("REFERENCE", "CANDIDATE"),
        help="strictly compare structure/status and tolerantly compare floats",
    )
    parser.add_argument("--output", type=Path, help="result path for --input; stdout if omitted")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        if args.self_test:
            if args.output is not None:
                raise ValueError("--output is only valid with --input")
            run_self_test()
            return 0
        if args.generate_fixtures is not None:
            if args.output is not None:
                raise ValueError("--output is only valid with --input")
            _write_json(build_fixture_bundle(), args.generate_fixtures)
            return 0
        if args.compare_fixtures is not None:
            if args.output is not None:
                raise ValueError("--output is only valid with --input")
            reference_path, candidate_path = args.compare_fixtures
            with reference_path.open("r", encoding="utf-8") as stream:
                reference = json.load(stream)
            with candidate_path.open("r", encoding="utf-8") as stream:
                candidate = json.load(stream)
            declared_tolerances = Tolerances.from_dict(reference.get("tolerances"))
            differences = compare_fixture_bundles(
                reference, candidate, declared_tolerances
            )
            if differences:
                sys.stderr.write(
                    "FAIL: fixture mismatch ({} differences)\n".format(
                        len(differences)
                    )
                )
                for difference in differences[:20]:
                    sys.stderr.write(difference + "\n")
                return 1
            print(
                "PASS: fixture structure/status exact; floats within atol={} rtol={}".format(
                    declared_tolerances.comparison_atol,
                    declared_tolerances.comparison_rtol,
                )
            )
            return 0

        with args.input.open("r", encoding="utf-8") as stream:
            payload = json.load(stream)
        tolerances = Tolerances.from_dict(payload.get("tolerances"))
        result = compute_recoverability(payload["F"], payload["G"], tolerances)
        _write_json(result, args.output)
        return 0 if result["status"] != "NUMERICAL_FAILURE" else 1
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        failure = {
            "schema_version": SCHEMA_VERSION,
            "status": "INVALID_INPUT",
            "status_detail": str(error),
        }
        _write_json(failure, args.output)
        return 2


if __name__ == "__main__":
    sys.exit(main())
