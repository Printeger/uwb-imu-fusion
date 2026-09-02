#!/usr/bin/env python3
"""Gate three preregistered fixed-lag performance repetitions."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import re
from collections import defaultdict

from week4_common import FAIL, INVALID, PASS, InvalidArtifact, atomic_json, load_protocol, percentile

PL_STAGES = {"conditional_statistic", "hypothesis_incidence_slope",
             "protection_level_risk_availability"}
ESTIMATOR_STAGES = {"imu_preintegration", "no_uwb_isam_update", "state_query",
                    "current_joint_marginal", "snapshot_extraction", "uwb_commit",
                    "uwb_reject"}


def read_csv(path):
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError as error:
        raise InvalidArtifact(f"missing {path}: {error}") from error


def analyze(root: pathlib.Path, protocol) -> dict:
    spec = protocol["performance"]
    records = []
    for repetition in range(1, int(spec["repetitions"])+1):
        directory = root / f"run_{repetition}"
        timing = read_csv(directory / "timing.csv")
        events = read_csv(directory / "events.csv")
        try:
            summary = json.loads((directory / "summary.json").read_text())
            manifest = json.loads((directory / "run_manifest.json").read_text())
        except (OSError, json.JSONDecodeError) as error:
            raise InvalidArtifact(f"run {repetition}: malformed JSON: {error}") from error
        by_stage = defaultdict(list)
        core_epochs = set()
        for row in timing:
            try:
                epoch = int(row["epoch"])
                value = float(row["wall_ms"])
            except (KeyError, ValueError) as error:
                raise InvalidArtifact(f"run {repetition}: invalid timing row: {error}") from error
            if not math.isfinite(value) or value < 0:
                raise InvalidArtifact(f"run {repetition}: invalid timing value")
            if row["stage"] == "core_total": core_epochs.add(epoch)
            if epoch > int(spec["warmup_epochs"]): by_stage[row["stage"]].append(value)
        expected = int(spec["epochs_per_repetition"])
        if len(core_epochs) != expected or summary.get("processed") != expected:
            raise InvalidArtifact(f"run {repetition}: processed/core epoch count mismatch")
        core = by_stage["core_total"]
        if len(core) != expected-int(spec["warmup_epochs"]):
            raise InvalidArtifact(f"run {repetition}: retained timing count mismatch")
        pl_mean = sum(sum(by_stage[stage]) for stage in PL_STAGES) / len(core)
        estimator_mean = sum(sum(by_stage[stage]) for stage in ESTIMATOR_STAGES) / len(core)
        retained = []
        marginalizations = 0
        numerical = 0
        for event in events:
            if event["event"] == "FIXED_LAG_MARGINALIZE":
                marginalizations += 1
                match = re.search(r"retained_epochs=([0-9]+)", event["detail"])
                if match: retained.append(int(match.group(1)))
            if event["event"] == "NUMERICAL_REINITIALIZE": numerical += 1
        checks = {
            "mean": sum(core)/len(core) < spec["gates"]["core_total_mean_ms_max"],
            "p99": percentile(core, .99) < spec["gates"]["core_total_p99_ms_max"],
            "ratio": pl_mean/estimator_mean <
                spec["gates"]["pl_to_estimator_mean_ratio_max"],
            "errors": summary.get("errors") == 0,
            "numerical_reinitializations": numerical == 0,
            "fixed_lag": manifest.get("fixed_lag_epochs") == spec["fixed_lag_epochs"],
            "retained": bool(retained) and max(retained) <=
                spec["gates"]["retained_epochs_max"],
            "marginalization": marginalizations > 0,
        }
        records.append({"repetition": repetition,
                        "core_total_mean_ms": sum(core)/len(core),
                        "core_total_p99_ms": percentile(core, .99),
                        "pl_estimator_mean_ratio": pl_mean/estimator_mean,
                        "marginalizations": marginalizations,
                        "maximum_retained_epochs": max(retained) if retained else None,
                        "checks": checks,
                        "status": PASS if all(checks.values()) else FAIL})
    return {"status": PASS if all(item["status"] == PASS for item in records) else FAIL,
            "runs": records, "platform_scope": "current WSL2 only"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=pathlib.Path)
    parser.add_argument("output_json", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    args = parser.parse_args()
    try:
        result = analyze(args.root, load_protocol(args.protocol))
    except InvalidArtifact as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(args.output_json, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
