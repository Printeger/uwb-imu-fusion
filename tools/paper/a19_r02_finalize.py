#!/usr/bin/env python3
"""Audit the stopped A19-R02 run without reading evaluation truth."""

import csv
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Dict, List


def load(path: Path) -> dict:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise RuntimeError(f"expected JSON object: {path}")
    return value


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ordered_blocks(root: Path) -> List[Dict]:
    paths = sorted(root.glob("outer*/block_status.json"),
                   key=lambda path: int(path.parent.name[5:]))
    return [load(path) for path in paths]


def sums(rows: List[Dict]) -> dict:
    result = {
        name: sum(row[name] for row in rows)
        for name in ("calls", "trials", "accepted", "rejected", "unresolved")
    }
    result["certificate_seconds"] = sum(row["certificate_s"] for row in rows)
    return result


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: a19_r02_finalize.py EVIDENCE_DIR")
    evidence = Path(sys.argv[1]).resolve()
    attempt = evidence / "attempts/attempt1"
    output = attempt / "pilot/output"
    preflight = load(evidence / "STARTUP_PREFLIGHT.json")
    first_preflight = load(evidence / "commands/preflight/STARTUP_PREFLIGHT.json")
    ticket = load(attempt / "TICKET.json")
    completion = load(attempt / "COMPLETION.json")
    command = load(attempt / "estimator_run/command.json")
    pipeline = load(output / "pipeline_status.json")
    stage1 = load(output / "stage1/status.json")
    stage2 = load(output / "stage2/status.json")
    partition_identity = load(output / "stage1/partition_identity.json")
    stage1_blocks = ordered_blocks(output / "stage1")
    stage2_blocks = ordered_blocks(output / "stage2")
    stage1_sum = sums(stage1_blocks)
    stage2_sum = sums(stage2_blocks)
    resources = (attempt / "estimator_run/resources.txt").read_text()
    rss_match = re.search(r"Maximum resident set size \(kbytes\): (\d+)",
                          resources)
    if not rss_match:
        raise RuntimeError("maximum RSS is missing")
    maximum_rss_kib = int(rss_match.group(1))

    with (output / "stage1/partition.csv").open(newline="", encoding="utf-8") as stream:
        segments = list(csv.DictReader(stream))
    candidate_count = sum(int(row["obs_count"]) for row in segments)
    trace = (attempt / "estimator_run/file_access.trace").read_text()
    opened = []
    for line in trace.splitlines():
        match = re.search(r'open(?:at)?\([^\"]*\"([^\"]+)\"', line)
        if match:
            opened.append(match.group(1))
    forbidden = [path for path in opened if
                 "/truth/" in path.lower() or
                 "ground_truth" in path.lower() or
                 Path(path).name.lower() in {"truth.csv", "gt.csv"} or
                 "/oracle/" in path.lower()]

    checks = {
        "startup_preflight_second_passed": preflight.get("passed") is True,
        "startup_preflight_first_failure_preserved":
            first_preflight.get("passed") is False and
            first_preflight.get("checks", {}).get("dynamic_library_resolution") is False,
        "single_ticket_consumed": ticket.get("attempt") == "attempt1" and
            len(list((evidence / "attempts").glob("*/TICKET.json"))) == 1,
        "no_attempt2": not (evidence / "attempts/attempt2").exists(),
        "bounded_exit_accounted": completion.get("actual_exit") == 1 and
            command.get("exit_code") == 1 and command.get("timeout_s") == 900 and
            float(command.get("external_wall_s", 901)) < 900,
        "raw_initialization_only": ticket.get("initialization") ==
            "ORIGINAL_RAW_NO_CHECKPOINT",
        "truth_gt_forbidden_and_unopened": ticket.get("truth_gt") == "FORBIDDEN" and
            not completion.get("forbidden_input_open_lines") and not forbidden,
        "stage1_converged": stage1.get("status") == "CONVERGED" and
            stage1.get("outers") == 90 and len(stage1_blocks) == 90,
        "stage1_counters_exact": all(stage1_sum[name] == stage1[name]
            for name in ("calls", "trials", "accepted", "rejected", "unresolved")),
        "stage1_partition_checkpointed": partition_identity.get("segments") == 2 and
            partition_identity.get("candidate_observations") == 32 and
            len(segments) == 2 and candidate_count == 32,
        "stage2_first_failure_exact": stage2.get("status") == "FAILED" and
            stage2.get("reason") == "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED" and
            len(stage2_blocks) == 15 and stage2_blocks[-1].get("outer") == 15 and
            stage2_blocks[-1].get("converged") is False,
        "stage2_counters_exact": all(stage2_sum[name] == stage2[name]
            for name in ("calls", "trials", "accepted", "rejected", "unresolved")),
        "scoring_not_run": pipeline.get("scoring") == "NOT_RUN" and
            not (output / "scores.csv").exists(),
        "stage2_final_not_exported": not (output / "stage2/final_values.csv").exists() and
            not (output / "segments.csv").exists(),
        "downstream_comparison_not_run": not (evidence / "finals").exists(),
        "evaluation_truth_not_read": not (evidence / "evaluation").exists(),
    }
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError("R02 audit failed: " + ", ".join(failed))

    result = {
        "schema": "A19_R02_STOPPED_RESULT_V1",
        "role": "development",
        "consumable": False,
        "status": "STOPPED_AT_FIRST_ALGORITHMIC_FAILURE",
        "startup": {
            "first_preflight": "FAILED_IDENTITY_SYMLINK_CHECK_ESTIMATOR_NOT_RUN",
            "second_preflight": "PASSED",
        },
        "automatic": {
            "actual_exit": completion["actual_exit"],
            "external_wall_seconds": completion["external_wall_s"],
            "maximum_rss_kib": maximum_rss_kib,
            "stage1": stage1,
            "partition": partition_identity,
            "segments": [{
                "segment_id": row["segment_id"],
                "ordinal": int(row["ordinal"]),
                "tag_id": int(row["tag_id"]),
                "anchor_id": int(row["anchor_id"]),
                "obs_count": int(row["obs_count"]),
                "start_s": float(row["start"]),
                "end_s": float(row["end"]),
                "short_support": row["short_support"] == "1",
                "merge_mean_m": float(row["merge_mean_m"]),
            } for row in segments],
            "stage2": {
                **stage2,
                "entered_outer": 15,
                "completed_outer_observer_rows": 14,
                "certificate_seconds": stage2_sum["certificate_seconds"],
                "terminal_block": stage2_blocks[-1],
            },
            "scoring": "NOT_RUN",
        },
        "downstream": {
            "decisions": "NOT_FROZEN",
            "five_treatment_comparison": "NOT_RUN_PREREQUISITE_STAGE2_FAILED",
            "evaluation_only_truth": "NOT_READ",
            "trajectory_delta_vs_suppress_all": "UNAVAILABLE",
        },
        "stop_rule": "ALGORITHM_STARTED_FIRST_FAILURE_NO_RETRY",
        "attempt2": "NOT_ISSUED",
        "checks": checks,
        "evidence_sha256": {
            "ticket": digest(attempt / "TICKET.json"),
            "command": digest(attempt / "estimator_run/command.json"),
            "pipeline_status": digest(output / "pipeline_status.json"),
            "stage1_status": digest(output / "stage1/status.json"),
            "partition": digest(output / "stage1/partition.csv"),
            "partition_identity": digest(output / "stage1/partition_identity.json"),
            "stage2_status": digest(output / "stage2/status.json"),
            "stage2_trace": digest(output / "stage2/outers.csv"),
            "stage2_terminal_block": digest(output / "stage2/outer15/block_status.json"),
        },
    }
    atomic_json(evidence / "AUTOMATIC_RESULT.json", result)

    comparison = evidence / "FINAL_COMPARISON.csv"
    temporary = comparison.with_name(comparison.name + ".tmp")
    with temporary.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["treatment", "decision_status", "final_status", "reason",
                         "trajectory_full", "trajectory_3_6_s", "fallback"])
        for treatment in ("suppress_all", "structured_debias", "fit_only",
                          "s_fit", "full_gate"):
            writer.writerow([treatment, "NOT_FROZEN", "NOT_RUN",
                             "PREREQUISITE_STAGE2_FAILED", "UNAVAILABLE",
                             "UNAVAILABLE", "NOT_RUN"])
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, comparison)
    print(json.dumps({"passed": True, "checks": len(checks),
                      "result": str(evidence / "AUTOMATIC_RESULT.json")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
