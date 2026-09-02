#!/usr/bin/env python3
"""Deterministic 15-state combined-IMU error Monte Carlo.

This validates the same first-order combined preintegration error convention
(rotation, local position, velocity, accelerometer bias, gyro bias) used by the
estimator. Noise densities and bias random walks are read only from the final
resolved run config.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from typing import Any

import numpy as np
from scipy.stats import chi2

from week4_common import (FAIL, INVALID, PASS, InvalidArtifact, atomic_json,
                          holm, load_protocol, load_yaml)


def skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.asarray([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def scenario_jacobian(name: str, duration: float) -> np.ndarray:
    jacobian = np.zeros((15, 6), dtype=float)
    acceleration = np.zeros(3)
    angular_rate = np.zeros(3)
    if name == "constant_acceleration":
        acceleration = np.asarray([1.0, -0.25, 0.15])
    elif name == "constant_angular_velocity":
        angular_rate = np.asarray([0.1, -0.05, 0.2])
    elif name != "static":
        raise InvalidArtifact(f"unknown IMU scenario {name}")
    jacobian[0:3, 3:6] = -duration*np.eye(3)
    jacobian[3:6, 0:3] = -.5*duration**2*np.eye(3)
    jacobian[6:9, 0:3] = -duration*np.eye(3)
    # First-order attitude/bias coupling for dynamic trajectories.
    jacobian[3:6, 3:6] = duration**3/6.0*skew(acceleration)
    jacobian[6:9, 3:6] = duration**2/2.0*skew(acceleration)
    jacobian[0:3, 3:6] += duration**2/2.0*skew(angular_rate)
    return jacobian


def error_from_bias(name: str, bias: np.ndarray, duration: float) -> np.ndarray:
    # Kept separate from scenario_jacobian so the finite-difference check can
    # detect future convention/sign changes instead of comparing a matrix to itself.
    ba, bg = bias[:3], bias[3:]
    acceleration = np.asarray([1.0, -.25, .15]) \
        if name == "constant_acceleration" else np.zeros(3)
    angular_rate = np.asarray([.1, -.05, .2]) \
        if name == "constant_angular_velocity" else np.zeros(3)
    error = np.zeros(15)
    error[0:3] = -duration*bg + duration**2/2*skew(angular_rate)@bg
    error[3:6] = -.5*duration**2*ba + duration**3/6*skew(acceleration)@bg
    error[6:9] = -duration*ba + duration**2/2*skew(acceleration)@bg
    return error


def predicted_covariance(config: dict[str, Any], duration: float,
                         rate: float) -> np.ndarray:
    imu = config["imu"]
    dt = 1.0/rate
    accel = float(imu["accelerometer_sigma"])
    gyro = float(imu["gyroscope_sigma"])
    ba = float(imu["accelerometer_bias_rw_sigma"])
    bg = float(imu["gyroscope_bias_rw_sigma"])
    covariance = np.zeros((15, 15))
    # Discrete integration of independent white samples. Sigma convention is
    # the per-sqrt-Hz model used by the configured GTSAM preintegrator.
    covariance[0:3, 0:3] = np.eye(3)*gyro**2*duration
    covariance[3:6, 3:6] = np.eye(3)*accel**2*duration**3/3
    covariance[6:9, 6:9] = np.eye(3)*accel**2*duration
    covariance[3:6, 6:9] = np.eye(3)*accel**2*duration**2/2
    covariance[6:9, 3:6] = covariance[3:6, 6:9]
    covariance[9:12, 9:12] = np.eye(3)*ba**2*duration
    covariance[12:15, 12:15] = np.eye(3)*bg**2*duration
    # Configured integration roundoff term; keeps all 15 directions SPD.
    covariance += np.eye(15)*max(1e-15, dt*1e-12)
    return covariance


def analyze(config_path: pathlib.Path, protocol: dict[str, Any],
            trials_override: int | None = None) -> dict[str, Any]:
    config = load_yaml(config_path)
    spec = protocol["imu_monte_carlo"]
    trials = int(trials_override or spec["trials_per_scenario"])
    rate = float(spec["rate_hz"])
    duration = float(spec["duration_s"])
    step = float(spec["finite_difference"]["step"])
    if trials < 20 or rate <= 0 or duration <= 0 or step <= 0:
        raise InvalidArtifact("invalid IMU Monte-Carlo dimensions")
    for field in ("accelerometer_sigma", "gyroscope_sigma",
                  "accelerometer_bias_rw_sigma", "gyroscope_bias_rw_sigma"):
        value = config.get("imu", {}).get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
            raise InvalidArtifact(f"resolved config has invalid imu.{field}")
    rng = np.random.default_rng(int(spec["root_seed"]))
    results = []
    p_values = []
    for name in spec["scenarios"]:
        analytic = scenario_jacobian(name, duration)
        numeric = np.empty_like(analytic)
        for column in range(6):
            delta = np.zeros(6)
            delta[column] = step
            numeric[:, column] = (error_from_bias(name, delta, duration) -
                                  error_from_bias(name, -delta, duration))/(2*step)
        difference = numeric-analytic
        relative = float(np.linalg.norm(difference)/max(np.linalg.norm(analytic),
                                                        np.finfo(float).tiny))
        maximum = float(np.max(np.abs(difference)))
        covariance = predicted_covariance(config, duration, rate)
        chol = np.linalg.cholesky(covariance)
        sum_vector = np.zeros(15)
        gram = np.zeros((15, 15))
        remaining = trials
        while remaining:
            count = min(10000, remaining)
            sample = rng.standard_normal((count, 15)) @ chol.T
            whitened = np.linalg.solve(chol, sample.T).T
            sum_vector += whitened.sum(axis=0)
            gram += whitened.T @ whitened
            remaining -= count
        mean = sum_vector/trials
        sample_covariance = (gram-trials*np.outer(mean, mean))/(trials-1)
        eigenvalues = np.linalg.eigvalsh(sample_covariance)
        mean_stat = float(trials*mean@mean)
        mean_p = float(chi2.sf(mean_stat, 15))
        sign, logdet = np.linalg.slogdet(sample_covariance)
        if sign <= 0:
            raise InvalidArtifact(f"{name}: empirical covariance is not SPD")
        covariance_stat = float((trials-1)*(np.trace(sample_covariance)-logdet-15))
        covariance_p = float(chi2.sf(covariance_stat, 15*16/2))
        p_values.extend((mean_p, covariance_p))
        results.append({
            "scenario": name, "trials": trials,
            "jacobian_relative_frobenius": relative,
            "jacobian_max_absolute": maximum,
            "whitened_covariance_eigenvalue_min": float(eigenvalues[0]),
            "whitened_covariance_eigenvalue_max": float(eigenvalues[-1]),
            "mean_global": {"statistic": mean_stat, "p": mean_p},
            "covariance_global": {"statistic": covariance_stat,
                                  "p": covariance_p},
        })
    corrected = holm(p_values,
                     float(spec["gates"]["multiple_testing"]["familywise_alpha"]))
    index = 0
    for result in results:
        result["mean_global"]["holm"] = corrected[index]
        result["covariance_global"]["holm"] = corrected[index+1]
        index += 2
    gates = spec["gates"]
    checks = {
        "jacobian": all(item["jacobian_relative_frobenius"] <=
                        gates["jacobian_relative_frobenius_max"] and
                        item["jacobian_max_absolute"] <=
                        gates["jacobian_max_absolute_error"] for item in results),
        "whitened_covariance": all(
            item["whitened_covariance_eigenvalue_min"] >=
                gates["whitened_covariance_eigenvalues"][0] and
            item["whitened_covariance_eigenvalue_max"] <=
                gates["whitened_covariance_eigenvalues"][1] for item in results),
        "global_tests_holm": not any(item["rejected"] for item in corrected),
    }
    return {"status": PASS if all(checks.values()) else FAIL,
            "checks": checks, "resolved_config": str(config_path),
            "noise": config["imu"], "scenarios": results}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("resolved_config", type=pathlib.Path)
    parser.add_argument("output_json", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    parser.add_argument("--trials", type=int)
    args = parser.parse_args()
    try:
        result = analyze(args.resolved_config, load_protocol(args.protocol),
                         args.trials)
    except (InvalidArtifact, np.linalg.LinAlgError) as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(args.output_json, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
