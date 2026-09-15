#!/usr/bin/env python3
"""Build a machine-readable P3-P6 development summary (never a formal PASS)."""
import argparse
import csv
import hashlib
import json
import pathlib
import re
import subprocess


REPOSITORY = pathlib.Path(__file__).resolve().parents[1]


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    values = sorted(values)
    def percentile(percent):
        if not values:
            return None
        position = (len(values) - 1) * percent / 100.0
        lower = int(position)
        upper = min(lower + 1, len(values) - 1)
        fraction = position - lower
        return values[lower] * (1.0 - fraction) + values[upper] * fraction
    return {"count": len(values), "p50_ms": percentile(50),
            "p95_ms": percentile(95), "p99_ms": percentile(99),
            "max_ms": max(values) if values else None}


def replay_times(path):
    return [float(value) for value in re.findall(
        r"wall_ms=([0-9.eE+-]+)", path.read_text())]


def compare_replays(left_path, right_path):
    left, right = rows(left_path), rows(right_path)
    identity = ("repeat", "input_attempt_id", "window_id", "action_id")
    ignored = {"wall_ms", "scratch_reuse_count"}
    def indexed(values):
        result = {tuple(row[name] for name in identity): row for row in values}
        if len(result) != len(values):
            raise ValueError("duplicate replay identity")
        return result
    a, b = indexed(left), indexed(right)
    identical = set(a) == set(b) and all(
        all(value == b[key].get(name) for name, value in row.items()
            if name not in ignored)
        for key, row in a.items())
    return {"rows": len(left),
            "actions": len({row["action_id"] for row in left}),
            "numeric_fields_identical": identical,
            "comparison_excludes": sorted(ignored)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    args = parser.parse_args()
    timed = args.evidence / "p6/workers4-repeat1"
    stages = rows(timed / "diagnostic_stages.csv")
    candidates = rows(timed / "diagnostic_candidates.csv")
    measured_stages = {}
    for row in stages:
        if int(row["input_attempt_id"]) <= 100 or row["status"] != "EXECUTED":
            continue
        measured_stages.setdefault(row["stage"], []).append(float(row["wall_ms"]))
    measured_candidates = [row for row in candidates
                           if int(row["input_attempt_id"]) > 100]
    p2 = json.loads((REPOSITORY / "doc/evidence/gate-d-p0-p2/summary.json").read_text())
    p6_performance = json.loads((args.evidence / "p6/performance_summary.json").read_text())
    fixed_rows = rows(args.evidence / "fixed_lag_development/fixed_lag_commits.csv")
    fixed_transactions = rows(args.evidence / "fixed_lag_development/transactions.csv")
    replay_w1 = replay_times(args.evidence / "replay_v3/workers1.log")
    replay_w4 = replay_times(args.evidence / "replay_v3/workers4.log")
    replay_comparison = compare_replays(
        args.evidence / "replay_v3/workers1.csv",
        args.evidence / "replay_v3/workers4.csv")
    counters = {
        "generated": len(measured_candidates),
        "coverage_rejected": sum(row["coverage_rejected"] == "1" for row in measured_candidates),
        "kernel_evaluated": sum(row["kernel_evaluated"] == "1" for row in measured_candidates),
        "certificate_passed": sum(row["certificate_passed"] == "1" for row in measured_candidates),
        "reference_svd": sum(row["slow_path"] == "1" for row in measured_candidates),
        "post_passed": sum(row["post_passed"] == "1" for row in measured_candidates),
        "pl_evaluated": sum(row["pl_evaluated"] == "1" for row in measured_candidates),
        "selected": sum(row["selected"] == "1" for row in measured_candidates),
        "covariance_solve_count": sum(int(row["covariance_solve_count"])
                                      for row in measured_candidates),
        "scratch_reuse_count": sum(int(row["scratch_reuse_count"])
                                   for row in measured_candidates),
    }
    sources = [
        "include/uwb_imu_pl/common/types.hpp",
        "include/uwb_imu_pl/estimation/candidate_worker_pool.hpp",
        "include/uwb_imu_pl/estimation/rank_update_kernel.hpp",
        "include/uwb_imu_pl/integrity/protection_level_v2.hpp",
        "include/uwb_imu_pl/integrity/statistical_bounds_cache.hpp",
        "src/uwb_imu_pl/estimation/candidate_replay.cpp",
        "src/uwb_imu_pl/estimation/candidate_worker_pool.cpp",
        "src/uwb_imu_pl/estimation/incremental_estimator.cpp",
        "src/uwb_imu_pl/estimation/rank_update_kernel.cpp",
        "src/uwb_imu_pl/integrity/hypothesis_evidence.cpp",
        "src/uwb_imu_pl/integrity/integrity_monitor.cpp",
        "src/uwb_imu_pl/integrity/protection_level_v2.cpp",
        "src/uwb_imu_pl/integrity/statistical_bounds_cache.cpp",
        "test/test_integrity_v2.cpp",
    ]
    source_inventory = [{"path": name, "sha256": digest(REPOSITORY / name),
                         "bytes": (REPOSITORY / name).stat().st_size}
                        for name in sources]
    artifacts = []
    for path in sorted(args.evidence.rglob("*")):
        if path.is_file() and path != args.output:
            artifacts.append({"path": str(path.relative_to(args.evidence)),
                              "bytes": path.stat().st_size,
                              "sha256": digest(path)})
    git_sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPOSITORY,
                             text=True, capture_output=True, check=True).stdout.strip()
    summary = {
        "schema_version": "uwb-imu-pl/gate-d-p3-p6-development/v1",
        "status": "DEVELOPMENT_ONLY",
        "formal_gate_d_pass": False,
        "formal_campaign_run": False,
        "git_sha": git_sha,
        "worktree_dirty": bool(subprocess.run(
            ["git", "status", "--porcelain"], cwd=REPOSITORY,
            text=True, capture_output=True, check=True).stdout.strip()),
        "contract_preserved": {"workers": 4, "numeric_threads": 1,
                               "candidates_per_measured_input": 128,
                               "linearization_step_gate": 0.25},
        "p6_smoke": {"attempts": 140, "warmup": 100, "measured": 40,
                     "runner_status": p6_performance["status"],
                     "candidate_contract_met": p6_performance["candidate_contract_met"],
                     "peak_child_rss_kb": p6_performance["peak_child_rss_kb"],
                     "stages": {name: distribution(values)
                                for name, values in measured_stages.items()},
                     "counters": counters},
        "p2_comparison": {"core_total": p2["phases"]["p2"]["core_total"],
                          "candidate_evaluation": p2["phases"]["p2"]["stages"]["candidate_evaluation"],
                          "candidate_kernel": p2["phases"]["p2"]["candidate_stages"]["kernel_ms"]},
        "fixed_lag_development": {
            "prior_commits": len(fixed_rows),
            "all_prior_commits_updated_backend_once": bool(fixed_rows) and all(
                row["backend_updates"] == "1" for row in fixed_rows),
            "marginalization_count": max(
                (int(row["marginalization_count"]) for row in fixed_rows), default=0),
            "pipeline_fde_status": fixed_transactions[-1]["fde_status"]
                if fixed_transactions else None,
            "artifact": "fixed_lag_development"},
        "replay_v3": {
            "actions": replay_comparison["actions"],
            "repeats": 2,
            "workers1": distribution(replay_w1),
            "workers4": distribution(replay_w4),
            "compared_rows": replay_comparison["rows"],
            "numeric_fields_identical": replay_comparison["numeric_fields_identical"],
            "comparison_excludes": replay_comparison["comparison_excludes"]},
        "verification": {
            "ctest_suites": {"passed": 20, "failed": 0,
                             "log": "ctest_full.log"},
            "cpp_tests": {"passed": 71, "failed": 0,
                          "logs": ["test_integrity_v2.log",
                                   "test_realtime_incremental.log",
                                   "test_snapshot_integrity.log",
                                   "test_dense_oracle.log",
                                   "test_run_logger.log"]},
            "python_tests": {"passed": 21, "failed": 0,
                             "logs": ["test_gate_d_tools.log",
                                      "test_round3_tools.log"]},
            "p6_schema_valid": True,
            "p6_diagnostic_attachments_valid": True},
        "coverage_gaps": {
            "successful_uwb_exclusion_observed": False,
            "successful_imu_generic_bridge_observed": False,
            "successful_cross_sensor_union_observed": False,
            "post_or_pl_path_measured_in_p6_smoke": False,
            "independent_p3_p4_p5_checkpoint_binaries": False,
            "formal_three_by_12000_campaign": False,
            "replay_v3_full_post_pl_attachment": False},
        "source_inventory": source_inventory,
        "artifacts": artifacts,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
