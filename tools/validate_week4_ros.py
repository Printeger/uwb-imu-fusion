#!/usr/bin/env python3
"""Validate retained outputs from all deterministic Week-4 ROS scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import pathlib

from week4_common import FAIL, INVALID, PASS, InvalidArtifact, atomic_json, load_protocol


def rows(path: pathlib.Path):
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError as error:
        raise InvalidArtifact(f"missing ROS artifact {path}: {error}") from error


def validate(root: pathlib.Path, protocol) -> dict:
    scenario_results = {}
    for scenario in protocol["ros_topic_tests"]["scenarios"]:
        directory = root / scenario
        events = rows(directory / "events.csv")
        integrity = rows(directory / "integrity.csv")
        fault_truth = rows(directory / "fault_truth.csv")
        try:
            inventory = json.loads((directory / "publisher_inventory.json").read_text())
            summary = json.loads((directory / "summary.json").read_text())
        except (OSError, json.JSONDecodeError) as error:
            raise InvalidArtifact(f"{scenario}: malformed inventory/summary: {error}") from error
        event_names = [row["event"] for row in events]
        checks = {
            "zero_processing_errors": summary.get("errors") == 0,
            "no_crash": bool(inventory.get("completed")),
            "processed_output": bool(integrity),
        }
        if scenario == "delayed_stale_uwb":
            checks["stale_rejected"] = "STALE_UWB_REJECT" in event_names
        elif scenario == "uwb_drop_resume":
            commit_indices = [index for index, name in enumerate(event_names)
                              if name == "UWB_COMMIT"]
            checks["drop_resume_processed"] = len(commit_indices) >= 2
        elif scenario == "imu_gap":
            checks["imu_gap_explicit"] = "IMU_GAP_REINITIALIZE" in event_names and any(
                row.get("availability", "").lower() == "unavailable" for row in integrity)
        elif scenario == "anchor_set_8_7_8":
            sizes = [int(row["group_size"]) for row in integrity]
            dofs = [int(row["conditional_dof"]) for row in integrity]
            checks["anchor_and_dof"] = 7 in sizes and 8 in sizes and 7 in dofs and 8 in dofs
        elif scenario.startswith("fault_"):
            planned = {int(item["timestamp_ns"]): item for item in inventory["epochs"]}
            matching = 0
            for row in fault_truth:
                item = planned.get(int(row["timestamp_ns"]))
                if item and abs(float(row["injected_bias_m"])-
                                float(item["injected_bias_m"])) < 1e-12 and \
                        (row["active"].lower() in ("1", "true")) == item["fault_active"]:
                    matching += 1
            checks["fault_truth_matches"] = matching == len(planned)
            checks["detector_response"] = any(
                row.get("conditional_passed", "1").lower() in ("0", "false")
                for row in integrity)
        scenario_results[scenario] = {"status": PASS if all(checks.values()) else FAIL,
                                      "checks": checks, "integrity_rows": len(integrity)}
    return {"status": PASS if all(item["status"] == PASS
                                  for item in scenario_results.values()) else FAIL,
            "scenarios": scenario_results}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=pathlib.Path)
    parser.add_argument("output_json", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    args = parser.parse_args()
    try:
        result = validate(args.root, load_protocol(args.protocol))
    except InvalidArtifact as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(args.output_json, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
