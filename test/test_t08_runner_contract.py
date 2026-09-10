#!/usr/bin/env python3
"""End-to-end T08 artifact, identity, and strict serialization contract."""

import argparse
import csv
import json
import pathlib
import subprocess
import tempfile
import time


GATE_LABEL = "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION"

CSV_COLUMNS = {
    "decisions.csv": [
        "inference_id", "group_id", "start_time", "end_time",
        "segment_ordinals", "decision", "reason_code", "eligible",
        "eta_pass", "s_pass", "gamma_pass", "max_gamma",
        "max_gamma_available", "decision_linearization_id",
    ],
    "scores_decision.csv": [
        "inference_id", "phase", "group_id", "segment_ordinals", "status",
        "eligible", "evaluated", "valid_score_exported", "eta", "s_m",
        "s_is_infinite", "max_gamma", "numerical_status",
        "linearization_id",
    ],
    "scores_final.csv": [
        "inference_id", "phase", "group_id", "segment_ordinals", "status",
        "eligible", "evaluated", "valid_score_exported", "eta", "s_m",
        "s_is_infinite", "max_gamma", "numerical_status",
        "linearization_id",
    ],
    "scores_recovery_attempt.csv": [
        "inference_id", "phase", "group_id", "segment_ordinals", "status",
        "eligible", "evaluated", "valid_score_exported", "eta", "s_m",
        "s_is_infinite", "max_gamma", "numerical_status",
        "linearization_id",
    ],
    "final_factor_audit.csv": [
        "inference_id", "obs_id", "classification", "final_factor_count",
        "expected_count", "ok",
    ],
    "final_masks.csv": [
        "inference_id", "obs_id", "candidate", "noncandidate_reference",
        "decision_use", "final_use", "fallback_use", "segment_id",
        "group_id", "reason_code",
    ],
}


def strict_json(path: pathlib.Path):
    def reject_constant(value: str):
        raise ValueError(f"non-standard JSON constant {value} in {path}")
    return json.loads(path.read_text(encoding="utf-8"),
                      parse_constant=reject_constant)


def read_csv(path: pathlib.Path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if reader.fieldnames is None:
            raise AssertionError(f"missing CSV header: {path}")
        for row in rows:
            if None in row or len(row) != len(reader.fieldnames):
                raise AssertionError(f"malformed CSV row: {path}: {row}")
        return reader.fieldnames, rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="uifgo-t08-runner-test-") as temp:
        root = pathlib.Path(temp)
        external_started = time.perf_counter()
        completed = subprocess.run(
            [str(args.runner.resolve()), "--config", str(args.config.resolve()),
             "--output-root", str(root), "--run-id", "normal"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False,
        )
        external_elapsed = time.perf_counter() - external_started
        if completed.returncode != 0:
            raise AssertionError(
                f"T08 runner failed ({completed.returncode}):\n"
                f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
            )
        run = root / "normal"
        summary = strict_json(run / "final_inference_summary.json")
        fallback = strict_json(run / "fallback_attempt.json")
        covariance = strict_json(run / "covariance_status.json")
        status = strict_json(run / "run_status.json")
        capability = strict_json(run / "capability_status.json")
        identity = strict_json(run / "final_content_identity.json")
        stage2_status = strict_json(run / "stage2_refit_status.json")
        strict_json(run / "input_manifest.json")
        inference_id = summary["inference_id"]
        if not inference_id.startswith("t08inference-sha256:"):
            raise AssertionError(f"bad inference identity: {inference_id}")
        if summary["status"] != "OK" or not summary["valid_estimate"]:
            raise AssertionError(f"normal T08 inference did not pass: {summary}")
        if summary["gate_parameter_provenance"] != GATE_LABEL:
            raise AssertionError("development-only gate provenance missing")
        if summary["fallback_attempt_count"] > 1:
            raise AssertionError("fallback attempt count exceeded one")
        if summary["factor_audit_status"] != "OK":
            raise AssertionError(f"factor audit failed: {summary}")
        if summary["corrected_pseudo_range_count"] != 0:
            raise AssertionError("corrected pseudo-range entered final graph")
        for document in (fallback, covariance, status):
            if document["inference_id"] != inference_id:
                raise AssertionError("JSON inference identity mismatch")
        if identity["inference_id"] != inference_id:
            raise AssertionError("content identity document mismatch")
        for key, prefix in {
            "graph_linearization_sha256": "t08graphlin-sha256:",
            "values_sha256": "t08values-sha256:",
            "context_sha256": "t08context-sha256:",
        }.items():
            if not identity[key].startswith(prefix):
                raise AssertionError(f"bad content fingerprint {key}: {identity}")
        if status["run_id"] != "normal":
            raise AssertionError("run_id execution namespace missing")
        if status["elapsed_seconds_semantics"] != (
                "RUNNER_WALL_FROM_MAIN_ENTRY_THROUGH_FINAL_STATUS_PREPARATION"):
            raise AssertionError(f"unclear runner timing semantics: {status}")
        runner_elapsed = status["elapsed_seconds"]
        stage4_elapsed = status["stage4_engine_seconds"]
        if not (runner_elapsed >= stage4_elapsed >= 0.0):
            raise AssertionError(f"invalid runner/stage4 timing: {status}")
        if abs(external_elapsed - runner_elapsed) > 2.0:
            raise AssertionError(
                f"runner wall {runner_elapsed} disagrees with external "
                f"wall {external_elapsed}")
        if status["stage2_seconds"] is None:
            raise AssertionError("Stage 2 timing was not retained")
        if (stage2_status["phase"] != "STAGE2_DEBIASED_REFIT" or
                stage2_status["solver_status"] != "CONVERGED"):
            raise AssertionError(f"Stage 2 diagnostics incomplete: {stage2_status}")
        if capability["output_semantics"] != "SINGLE_T08_INFERENCE_RESULT":
            raise AssertionError(f"wrong output semantics: {capability}")
        if covariance["reference_group_R_c_is_covariance"] is not False:
            raise AssertionError("full covariance confused with reference R_c")
        if covariance["status"] not in {"AVAILABLE", "UNAVAILABLE"}:
            raise AssertionError(f"invalid covariance state: {covariance}")

        csv_rows = {}
        for name, expected_columns in CSV_COLUMNS.items():
            columns, rows = read_csv(run / name)
            if columns != expected_columns:
                raise AssertionError(f"wrong columns in {name}: {columns}")
            csv_rows[name] = rows
        for path in run.glob("*.csv"):
            read_csv(path)
        for path in run.glob("*.json"):
            strict_json(path)
        for name in (
            "decisions.csv", "scores_decision.csv",
            "scores_recovery_attempt.csv", "scores_final.csv",
            "scores_decision_segments.csv",
            "scores_recovery_attempt_segments.csv",
            "scores_final_segments.csv",
            "final_factor_audit.csv", "final_masks.csv", "covariance.csv",
            "imu_bias.csv", "segment_bias.csv", "residuals.csv",
            "final_factor_metadata.csv",
        ):
            _, rows = read_csv(run / name)
            if any(row["inference_id"] != inference_id for row in rows):
                raise AssertionError(f"inference identity mismatch in {name}")
        for name in (
            "stage2_refit_iterations.csv", "recovery_refit_iterations.csv",
            "fallback_refit_iterations.csv", "final_refit_iterations.csv",
        ):
            columns, rows = read_csv(run / name)
            if not rows:
                raise AssertionError(f"missing explicit diagnostic row in {name}")
        if csv_rows["scores_recovery_attempt.csv"] != \
                csv_rows["scores_final.csv"]:
            # Phase labels intentionally differ; compare the actual payload.
            recovery = [dict(row, phase="FINAL")
                        for row in csv_rows["scores_recovery_attempt.csv"]]
            if recovery != csv_rows["scores_final.csv"]:
                raise AssertionError("normal final scores are not recovery scores")

        decisions = {row["group_id"]: row for row in csv_rows["decisions.csv"]}
        if not decisions:
            raise AssertionError("fixture produced no group decision")
        for row in csv_rows["final_masks.csv"]:
            if row["candidate"] != "1":
                continue
            decision = decisions[row["group_id"]]["decision"]
            expected = "1" if decision == "USE" else "0"
            if row["decision_use"] != expected or row["final_use"] != expected:
                raise AssertionError(f"non-atomic candidate group mask: {row}")
        for row in csv_rows["final_factor_audit.csv"]:
            if row["ok"] != "1" or row["final_factor_count"] != row["expected_count"]:
                raise AssertionError(f"factor multiplicity audit failed: {row}")
        decision_linearizations = {
            row["linearization_id"] for row in csv_rows["scores_decision.csv"]
        }
        final_linearizations = {
            row["linearization_id"] for row in csv_rows["scores_final.csv"]
        }
        if decision_linearizations & final_linearizations:
            raise AssertionError("decision and final linearization IDs overlap")
        metadata = (run / "final_factor_metadata.csv").read_text("utf-8")
        if "corrected" in metadata.lower() or "pseudo" in metadata.lower():
            raise AssertionError("corrected pseudo-range metadata present")

        repeated = subprocess.run(
            [str(args.runner.resolve()), "--config", str(args.config.resolve()),
             "--output-root", str(root), "--run-id", "repeat"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False,
        )
        if repeated.returncode != 0:
            raise AssertionError(
                f"T08 repeated runner failed ({repeated.returncode}):\n"
                f"stdout:\n{repeated.stdout}\nstderr:\n{repeated.stderr}")
        repeat_identity = strict_json(
            root / "repeat" / "final_content_identity.json")
        if repeat_identity != identity:
            raise AssertionError(
                "same input/config produced a different content identity")

        # A deterministic one-iteration Stage-2 failure must use the same
        # runner-wall definition, while all later stages remain not run.
        failure_config = root / "stage2_failure.yaml"
        config_text = args.config.resolve().read_text(encoding="utf-8")
        marker = "  max_refit_iterations: 20\n"
        bag_marker = "  path: ../../data/sim_circle_2026-06-15-16-04-35.bag\n"
        support_marker = "  oracle_support: sim_circle_t04_oracle_support.yaml\n"
        if (config_text.count(marker) != 1 or
                config_text.count(bag_marker) != 1 or
                config_text.count(support_marker) != 1):
            raise AssertionError("cannot create deterministic Stage-2 failure config")
        bag_path = (args.config.resolve().parent /
                    "../../data/sim_circle_2026-06-15-16-04-35.bag").resolve()
        support_path = (args.config.resolve().parent /
                        "sim_circle_t04_oracle_support.yaml").resolve()
        absolute_config = config_text.replace(
            bag_marker, f"  path: {bag_path}\n")
        failure_config.write_text(
            absolute_config.replace(marker, "  max_refit_iterations: 1\n")
            .replace(support_marker, f"  oracle_support: {support_path}\n"),
            encoding="utf-8")
        failure_started = time.perf_counter()
        failed = subprocess.run(
            [str(args.runner.resolve()), "--config", str(failure_config),
             "--output-root", str(root), "--run-id", "stage2_failure"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False,
        )
        failure_external_elapsed = time.perf_counter() - failure_started
        if failed.returncode == 0:
            raise AssertionError("one-iteration Stage-2 negative fixture passed")
        failure_status = strict_json(
            root / "stage2_failure" / "run_status.json")
        if failure_status["elapsed_seconds_semantics"] != \
                status["elapsed_seconds_semantics"]:
            raise AssertionError("failure path changed runner timing semantics")
        if failure_status["stage4_engine_seconds"] is not None:
            raise AssertionError("failure path mislabeled an unrun Stage 4")
        if abs(failure_external_elapsed -
               failure_status["elapsed_seconds"]) > 2.0:
            raise AssertionError("failure runner wall disagrees with external wall")

        invalid_support = root / "invalid_support.yaml"
        invalid_support.write_text(
            support_path.read_text(encoding="utf-8") +
            "review_unknown_field: true\n", encoding="utf-8")
        invalid_config = root / "invalid_support_config.yaml"
        invalid_config.write_text(
            absolute_config.replace(
                support_marker, f"  oracle_support: {invalid_support}\n"),
            encoding="utf-8")
        invalid_started = time.perf_counter()
        invalid = subprocess.run(
            [str(args.runner.resolve()), "--config", str(invalid_config),
             "--output-root", str(root), "--run-id", "invalid_support"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False,
        )
        invalid_external_elapsed = time.perf_counter() - invalid_started
        if invalid.returncode == 0:
            raise AssertionError("invalid oracle manifest unexpectedly passed")
        invalid_run = root / "invalid_support"
        invalid_status = strict_json(invalid_run / "run_status.json")
        if invalid_status["failure_stage"] != "ORACLE_SUPPORT_LOAD":
            raise AssertionError(f"wrong invalid-manifest stage: {invalid_status}")
        if invalid_status["solver_status"] != "INVALID_ORACLE_MANIFEST":
            raise AssertionError(f"wrong invalid-manifest status: {invalid_status}")
        if invalid_status["valid_estimate_exported"] is not False:
            raise AssertionError("invalid manifest exported a valid estimate")
        if invalid_status["elapsed_seconds_semantics"] != \
                status["elapsed_seconds_semantics"]:
            raise AssertionError("exception path changed runner timing semantics")
        if abs(invalid_external_elapsed -
               invalid_status["elapsed_seconds"]) > 2.0:
            raise AssertionError(
                "exception runner wall disagrees with external wall")
        for name in (
                "stage1_seconds", "stage2_seconds",
                "stage3_decision_score_seconds", "stage4_engine_seconds"):
            if invalid_status[name] is not None:
                raise AssertionError(
                    f"unrun stage {name} has a fabricated duration: "
                    f"{invalid_status}")
        for name in (
                "trajectory.tum", "imu_bias.csv", "static_bias.csv",
                "segment_bias.csv", "residuals.csv", "covariance.csv",
                "final_inference_summary.json"):
            if (invalid_run / name).exists():
                raise AssertionError(
                    f"invalid manifest exported estimate artifact {name}")

        # The legacy all-range checked-LM exception exit shares the same
        # wall-clock schema even though none of the T08 stages applies.
        t02_config = args.config.resolve().parent / "sfuise_walk1_t02_smoke.yaml"
        t02_text = t02_config.read_text(encoding="utf-8")
        data_marker = "  data_dir: ../../data/SFUISE\n"
        lm_marker = "  lm_max_iter: 200\n"
        if t02_text.count(data_marker) != 1 or t02_text.count(lm_marker) != 1:
            raise AssertionError("cannot create deterministic LmFailure config")
        data_path = (t02_config.parent / "../../data/SFUISE").resolve()
        lm_failure_config = root / "lm_failure_config.yaml"
        lm_failure_config.write_text(
            t02_text.replace(data_marker, f"  data_dir: {data_path}\n")
            .replace(lm_marker, "  lm_max_iter: 1\n"), encoding="utf-8")
        lm_started = time.perf_counter()
        lm_failed = subprocess.run(
            [str(args.runner.resolve()), "--config", str(lm_failure_config),
             "--output-root", str(root), "--run-id", "lm_failure"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            check=False,
        )
        lm_external_elapsed = time.perf_counter() - lm_started
        if lm_failed.returncode == 0:
            raise AssertionError("one-iteration checked-LM fixture passed")
        lm_run = root / "lm_failure"
        lm_status = strict_json(lm_run / "run_status.json")
        if not lm_status["reason"].startswith("LM_"):
            raise AssertionError(f"checked-LM reason not preserved: {lm_status}")
        if lm_status["elapsed_seconds_semantics"] != \
                status["elapsed_seconds_semantics"]:
            raise AssertionError("LmFailure path changed runner timing semantics")
        if abs(lm_external_elapsed - lm_status["elapsed_seconds"]) > 2.0:
            raise AssertionError("LmFailure wall disagrees with external wall")
        for name in (
                "stage1_seconds", "stage2_seconds",
                "stage3_decision_score_seconds", "stage4_engine_seconds"):
            if lm_status[name] is not None:
                raise AssertionError(
                    f"non-applicable LmFailure stage {name} has duration")
        if (lm_run / "trajectory.tum").exists():
            raise AssertionError("LmFailure exported a trajectory")

    print("T08 runner strict artifact, timing, and content identity checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
