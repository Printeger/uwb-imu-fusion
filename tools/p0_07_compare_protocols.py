#!/usr/bin/env python3
"""Process-C field comparator; contains no oracle or production computation."""

import argparse
from pathlib import Path
import numpy as np


def matrix(fields, at):
    rows, cols = int(fields[at]), int(fields[at + 1])
    end = at + 2 + rows * cols
    return np.asarray(fields[at + 2:end], dtype=float).reshape(rows, cols), end


def close(name, expected, actual):
    if expected.shape != actual.shape:
        raise AssertionError(f"{name}: shape {expected.shape} != {actual.shape}")
    scale = max(1.0, float(np.linalg.norm(expected)))
    tolerance = np.finfo(float).eps * scale * max(32, expected.size) * 1.0e7
    error = float(np.linalg.norm(expected - actual))
    if not np.isfinite(error) or error > tolerance:
        raise AssertionError(f"{name}: error {error} > {tolerance}")


def load_oracle(path):
    result = {"matrices": {}, "actions": {}, "proofs": {}, "outcomes": {}}
    for line in Path(path).read_text().splitlines():
        f = line.split("\t")
        if f[0] == "P007_ORACLE_V1": result["header"] = f[1:]
        elif f[0] == "INPUT": result["input"] = tuple(map(int, f[1:]))
        elif f[0] in {"SERVED_H", "SERVED_Z", "BASE_INFORMATION", "BASE_RHS", "BASE_STATE"}:
            result["matrices"][f[0]], _ = matrix(f, 1)
        elif f[0] == "ACTION": result["actions"][int(f[1])] = (int(f[2]), int(f[3]), float(f[4]))
        elif f[0] == "ORACLE_PROOF":
            gram, at = matrix(f, 5); response, at = matrix(f, at); slopes, at = matrix(f, at)
            component, at = matrix(f, at)
            result["proofs"][(int(f[1]), int(f[2]))] = (
                int(f[3]), int(f[4]), gram, response, slopes, component,
                float(f[at]), float(f[at + 1]), float(f[at + 2]),
                float(f[at + 3]), f[at + 4])
        elif f[0] == "ACTION_OUTCOME":
            result["outcomes"][int(f[1])] = (
                int(f[2]), int(f[3]), int(f[4]), int(f[5]), int(f[6]), f[7])
        elif f[0] == "RISK": result["risk"] = tuple(map(float, f[1:]))
        elif f[0] == "DECISION": result["decision"] = (int(f[1]), f[2], int(f[3]), int(f[4]))
        elif f[0] == "OBSERVED":
            count = int(f[1]); result["observed"] = (
                tuple(map(int, f[2:2 + count])), int(f[2 + count]),
                int(f[3 + count]), f[4 + count].strip('"'))
    return result


def load_actual(path):
    result = {"matrices": {}, "proofs": {}, "outcomes": {}}
    for line in Path(path).read_text().splitlines():
        f = line.split("\t")
        if f[0] == "P007_DECISION_V1": result["decision_header"] = f[1:]
        elif f[0] == "INPUT": result["input"] = tuple(map(int, f[1:]))
        elif f[0] == "P007_ACTUAL_V1": result["header"] = f[1:]
        elif f[0] == "RISK": result["risk"] = tuple(map(float, f[1:]))
        elif f[0] == "OBSERVED":
            count = int(f[1]); result["observed"] = (
                tuple(map(int, f[2:2 + count])), int(f[2 + count]),
                int(f[3 + count]), f[4 + count].strip('"'))
        elif f[0] in {"SERVED_H", "SERVED_Z", "BASE_INFORMATION", "BASE_RHS", "BASE_STATE"}:
            result["matrices"][f[0]], _ = matrix(f, 1)
        elif f[0] == "PROOF":
            gram, at = matrix(f, 10); response, at = matrix(f, at); slopes, at = matrix(f, at)
            null_class = int(f[at]); at += 1
            component, at = matrix(f, at)
            key = (int(f[1]), int(f[2]))
            result["proofs"][key] = (
                int(f[8]), null_class, gram, response, slopes, component,
                float(f[at]), float(f[at + 1]), float(f[at + 2]),
                float(f[at + 3]), f[9])
            result["outcomes"].setdefault(int(f[1]), (
                int(f[at + 4]), int(f[at + 5]), int(f[at + 6]),
                int(f[at + 7]), int(f[at + 8]), f[at + 9].strip('"')))
    return result


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--oracle", required=True); ap.add_argument("--actual", required=True)
    ap.add_argument("--skip-observed", action="store_true",
                    help="compare authoritative fields only (DERIVED leaf runs)")
    args = ap.parse_args(); oracle = load_oracle(args.oracle); actual = load_actual(args.actual)
    # Square-root rows have an orthogonal gauge. Compare their complete least-
    # squares objective (H' H, H' z, z' z), then compare canonical products.
    oh, oz = oracle["matrices"]["SERVED_H"], oracle["matrices"]["SERVED_Z"]
    ah, az = actual["matrices"]["SERVED_H"], actual["matrices"]["SERVED_Z"]
    if oracle["input"] != actual["input"]:
        raise AssertionError(f"frozen replay input mismatch {oracle['input']} != {actual['input']}")
    close("served HtH", oh.T @ oh, ah.T @ ah)
    close("served Htz", oh.T @ oz, ah.T @ az)
    close("served ztz", oz.T @ oz, az.T @ az)
    for name in ("BASE_INFORMATION", "BASE_RHS", "BASE_STATE"):
        close(name, oracle["matrices"][name], actual["matrices"][name])
    if len(oracle["proofs"]) != 7 * 504 or set(oracle["proofs"]) != set(actual["proofs"]):
        raise AssertionError("action x hypothesis census mismatch")
    available = 0
    for key, expected in oracle["proofs"].items():
        observed = actual["proofs"][key]
        if expected[0] != observed[0]: raise AssertionError(f"{key}: availability mismatch")
        if not expected[0]:
            if not observed[10]: raise AssertionError(f"{key}: unavailable without reason")
            continue
        available += 1
        if expected[1] != observed[1]: raise AssertionError(f"{key}: nullspace class mismatch")
        close(f"{key} gram", expected[2], observed[2]); close(f"{key} response", expected[3], observed[3])
        close(f"{key} slopes", expected[4], observed[4])
        expected_finite = np.all(np.isfinite(expected[5]))
        observed_finite = np.all(np.isfinite(observed[5]))
        if expected_finite != observed_finite:
            raise AssertionError(f"{key}: PL finite/unavailable mismatch ({expected[10]})")
        if expected_finite:
            close(f"{key} PL contribution", expected[5], observed[5])
        for index, label in ((6, "prior"), (7, "p_md"), (8, "allocation"),
                             (9, "hypothesis tail")):
            close(f"{key} {label}", np.asarray([expected[index]]),
                  np.asarray([observed[index]]))
    if set(oracle["outcomes"]) != set(actual["outcomes"]):
        raise AssertionError("action outcome census mismatch")
    for action, expected in oracle["outcomes"].items():
        observed = actual["outcomes"][action]
        if expected[:5] != observed[:5]:
            raise AssertionError(f"action {action}: validity/post/disposition/eligibility/selection mismatch {expected[:5]} != {observed[:5]}")
        if not expected[3] and not observed[5]:
            raise AssertionError(f"action {action}: refusal reason missing")
    if "risk" not in actual:
        raise AssertionError("actual risk protocol missing")
    close("risk ledger", np.asarray(oracle["risk"]), np.asarray(actual["risk"]))
    decision = actual["decision_header"]
    observed_decision = (int(decision[0]), decision[8].strip('"'), int(decision[3]), int(decision[4]))
    if oracle["decision"] != observed_decision:
        raise AssertionError(f"winner/terminal/formal/protected mismatch {oracle['decision']} != {observed_decision}")
    if not args.skip_observed and oracle["observed"] != actual["observed"]:
        raise AssertionError(f"same-run observed metadata mismatch {oracle['observed']} != {actual['observed']}")
    print(f"P007_PROCESS_ABC_PASS records={len(oracle['proofs'])} available={available}")


if __name__ == "__main__": main()
