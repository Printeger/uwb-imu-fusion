#!/usr/bin/env python3
"""Fail-closed P1-06 revised-scope synthetic acceptance summarizer."""

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

import yaml

from validate_simulation_calibrations import validate as validate_calibrations
from run_fde_profile_validation import (
    GATE24_GOLDEN_PREFIX, execution_identity,
    golden_anchor_from_protocol, golden_reference_manifest,
    terminal_artifacts_match)
from compare_p1_04_outputs import compare_directories
from analyze_fde_profile_runs import analyze_run


csv.field_size_limit(sys.maxsize)
REQUIRED_TIMING_STAGES = (
    "core_total", "analysis_completion", "arrival_to_publish", "outer_epoch")
STRICT_EQUIVALENCE_FILES = (
    "bridge.csv", "candidates.csv", "diagnostic_attempts.csv",
    "diagnostic_candidates.csv", "diagnostic_coverage.csv",
    "diagnostic_history_summary.csv", "diagnostic_snapshot_identity.csv",
    "diagnostic_square_root.csv", "diagnostic_state_steps.csv",
    "hypotheses.csv", "integrity.csv", "states.csv", "transactions.csv")


def percentile(values, probability):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * probability
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def distribution(values):
    return {"count": len(values), "p50_ms": percentile(values, .50),
            "p95_ms": percentile(values, .95),
            "p99_ms": percentile(values, .99),
            "max_ms": max(values) if values else None,
            "raw_ms": list(values)}


def csv_rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def rss_kib(run):
    stderr = run / "stderr.log"
    if not stderr.exists():
        return None
    match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)",
                      stderr.read_text(encoding="utf-8", errors="replace"))
    return int(match.group(1)) if match else None


def truthy(value):
    return value in (True, 1, "1", "true", "True")


def integer(row, key):
    try:
        return int(row.get(key, ""))
    except (AttributeError, TypeError, ValueError):
        return None


def finite_float(row, key):
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (AttributeError, TypeError, ValueError):
        return None


def nonnegative_extended_float(row, key):
    try:
        value = float(row.get(key, ""))
        return value if not math.isnan(value) and value >= 0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def indexed(rows, key, errors, label):
    result = {}
    for offset, row in enumerate(rows, 2):
        value = integer(row, key)
        if value is None or value in result:
            errors.append(f"{label}: invalid/duplicate {key} at CSV line {offset}")
        else:
            result[value] = row
    return result


def acceptance_path(attempt, candidates=None, integrity=None):
    """Classify at the action-kernel entrance, before post/PL outcomes."""
    generated = integer(attempt, "generated_actions")
    kernel = integer(attempt, "kernel_evaluated_actions")
    if (generated == 0 and kernel == 0 and not (candidates or []) and
            (integrity or {}).get("fde_status") == "FDE_DISABLED"):
        return "fde_disabled"
    if generated is None or kernel is None or generated < 1 or kernel < 1:
        return "unclassified"
    candidates = candidates or []
    window = integer(attempt, "window_id")
    sole = candidates[0] if len(candidates) == 1 else {}
    sole_keep_all = (
        generated == 1 and kernel == 1 and len(candidates) == 1 and
        integer(sole, "window_id") == window and
        integer(sole, "action_id") is not None and
        integer(sole, "cardinality") == 0 and
        sole.get("action_type") == "ACTION_SEARCH_OCCURRENCE_V1" and
        not sole.get("removed_group_ids") and not sole.get("added_group_ids") and
        "KEEP_ALL" in sole.get("physical_source_ids", "") and
        "operation_identity=ACTION_OPERATION_V1|0;0;0;0;0;0;0;0:" in
            sole.get("model_error_record", ""))
    return "normal" if sole_keep_all else "alarm_recovery"


def proof_partition(authoritative, generated_actions=None,
                    hypothesis_count=None, kernel_evaluated_actions=None):
    """Read emitted leaf census; never synthesize leaves from action counts."""
    del generated_actions, hypothesis_count, kernel_evaluated_actions
    if not isinstance(authoritative, dict):
        return {"valid": False, "expected": None, "exact_evaluated": None,
                "proof_covered": None, "uncovered": None,
                "reason": "authoritative emitted leaf census absent"}
    exact = integer(authoritative, "coverage_exact_leaves")
    proof = integer(authoritative, "coverage_enveloped_leaves")
    uncovered = integer(authoritative, "coverage_uncovered_leaves")
    proof_count = integer(authoritative, "coverage_proof_count")
    envelope_count = integer(authoritative, "coverage_envelope_count")
    values = (exact, proof, uncovered, proof_count, envelope_count)
    if any(value is None or value < 0 for value in values):
        return {"valid": False, "expected": None, "exact_evaluated": exact,
                "proof_covered": proof, "uncovered": uncovered,
                "reason": "invalid authoritative emitted leaf census"}
    expected = exact + proof + uncovered
    no_unapproved_proof = proof == 0 and proof_count == 0 and envelope_count == 0
    valid = (authoritative.get("coverage_status") == "COMPLETE" and
             expected > 0 and exact == expected and uncovered == 0 and
             no_unapproved_proof)
    return {"valid": valid, "expected": expected,
            "exact_evaluated": exact, "proof_covered": proof,
            "uncovered": uncovered,
            "reason": "" if valid else "leaf census incomplete or unapproved proof"}


def expected_run_identity(run_id, profile, scenario, epochs, seed, protocol,
                          calibration, frozen_execution=None):
    expected = {
        "schema": "uwb-imu-pl/fde-validation-run/v1", "run_id": run_id,
        "gate": "profiles", "scale": "full", "profile": profile,
        "scenario": scenario, "epochs": epochs, "seed": seed, "repeat": 1,
        "protocol_id": protocol["protocol_id"],
        "protocol_hash": calibration["validation_protocol_sha256"],
        "config_hash": calibration["runtime_config_sha256"],
        "calibration_artifact_class": "simulation/synthetic",
        "simulation_calibration_id": calibration["calibration_id"],
        "complete_model_config_digest":
            calibration["complete_model_config_digest"],
        "hardware_formal_qualification": "CLOSED/NOT_CLAIMED",
        "fault_overrides": {},
    }
    if frozen_execution:
        expected.update({key: frozen_execution[key] for key in (
            "git_sha", "source_tree_sha256", "source_file_count",
            "benchmark_source_sha256", "benchmark_path", "benchmark_sha256",
            "loaded_dso_sha256")})
    return expected


def verified_campaign_identity(root):
    path = root / "campaign_identity.json"
    frozen = json.loads(path.read_text(encoding="utf-8"))
    required = {"schema", "git_sha", "source_tree_sha256",
                "source_file_count", "benchmark_source_sha256",
                "benchmark_path", "benchmark_sha256", "loaded_dso_sha256"}
    if set(frozen) != required:
        raise ValueError("campaign identity field set mismatch")
    current = execution_identity(Path(frozen["benchmark_path"]))
    if frozen != current:
        raise ValueError("frozen source/binary/DSO identity is not current")
    return frozen


def identity_failures(identity, expected):
    failures = []
    for key, value in expected.items():
        if identity.get(key) != value:
            failures.append(f"{key}: expected {value!r}, got {identity.get(key)!r}")
    if identity.get("exit_code") != 0:
        failures.append("exit_code: not zero")
    wall = identity.get("runner_wall_s")
    if not isinstance(wall, (int, float)) or not math.isfinite(wall) or wall < 0:
        failures.append("runner_wall_s: missing/non-finite")
    return failures


def publication_failures(diag, integrity, candidates, deadline_missed,
                         complete, transaction=None, terminal=None,
                         arrival_ms=None):
    failures = []
    required_diag = (
        "transaction_id", "window_id", "publication_snapshot_id",
        "publication_state_solution_id", "publication_history_summary_id",
        "publication_manifest_digest", "publication_health_state",
        "publication_risk_proof_id")
    identity_accepted = diag.get("publication_identity_check") == "ACCEPTED"
    missing_identity = bool(diag.get("publication_missing_identity_fields"))
    if identity_accepted and any(integer(diag, key) in (None, 0)
                                 for key in required_diag):
        failures.append("accepted publication lacks authoritative identity")
    if not identity_accepted and not missing_identity:
        failures.append("refused publication lacks explicit missing identity")
    if (not truthy(diag.get("transaction_opened")) or
            not truthy(diag.get("publication_gate_executed")) or
            not truthy(diag.get("publication_transition_accepted")) or
            diag.get("status") not in ("EXECUTED", "COMMITTED", "DISCARDED")):
        failures.append("transaction/publication terminal state inconsistent")
    if (integer(diag, "transaction_id") != integer(integrity, "transaction_id") or
            integer(diag, "window_id") != integer(integrity, "window_id")):
        failures.append("attempt/receipt identity mismatch")
    committed = truthy(integrity.get("batch_committed"))
    before, after = integer(diag, "backend_epoch_before"), integer(diag, "backend_epoch_after")
    if (before is None or after is None or
            (committed and after != before + 1) or
            (not committed and after != before)):
        failures.append("commit receipt/backend epoch mismatch")
    protected = truthy(diag.get("publication_protected")) or truthy(
        integrity.get("publication_protected"))
    if protected or truthy(integrity.get("formal_eligible")):
        failures.append("simulation attempted protected/formal output")
    if (deadline_missed or not complete) and protected:
        failures.append("late/partial result became protected")
    protocol = integer(integrity, "final_packet_protocol_version")
    digest = integer(integrity, "final_packet_digest")
    if protocol != 2 or digest in (None, 0):
        failures.append("final packet protocol/digest absent")
    if truthy(integrity.get("final_packet_authoritative")):
        failures.append("simulation logger mirror incorrectly authoritative")
    selected = [row for row in candidates if truthy(row.get("selected"))]
    selected_count = integer(diag, "selected_actions")
    selected_id = integer(integrity, "selected_action_id")
    refusal = bool(diag.get("publication_refusal"))
    if selected_count not in (0, 1) or len(selected) != selected_count:
        failures.append("winner census mismatch")
    elif selected:
        winner = selected[0]
        if (integer(winner, "action_id") != selected_id or
                winner.get("action_type") != integrity.get("selected_action_type") or
                not truthy(winner.get("valid")) or
                not truthy(winner.get("post_detector_passed")) or
                not truthy(winner.get("covers_plausible_set")) or
                not truthy(winner.get("model_error_validated"))):
            failures.append("winner identity/eligibility mismatch")
    elif selected_id not in (None, 0):
        failures.append("unselected terminal output names a winner")
    if (truthy(diag.get("publication_unprotected")) and
            not refusal and not truthy(diag.get("publication_protected"))):
        failures.append("unprotected/refusal disposition inconsistent")
    selection_proof = integer(diag, "selection_risk_proof_id")
    publication_proof = integer(diag, "publication_risk_proof_id")
    closes = truthy(diag.get("risk_ledger_closes"))
    if ((selected_count == 1 or closes or protected or
         truthy(integrity.get("formal_eligible"))) and
            (selection_proof in (None, 0) or
             publication_proof != selection_proof)):
        failures.append("selection/publication risk proof mismatch")
    if selected_count == 0 and publication_proof != selection_proof:
        failures.append("unselected risk proof identity mismatch")
    risk_total = finite_float(diag, "risk_total")
    risk_bound = finite_float(diag, "risk_upper_bound")
    validated = truthy(diag.get("risk_ledger_all_validated"))
    charged = finite_float(diag, "risk_ledger_charged_total")
    declared = finite_float(diag, "risk_ledger_declared_total")
    margin = finite_float(diag, "risk_margin")
    if closes:
        if (risk_total is None or risk_bound is None or
                risk_total > risk_bound + 1e-18 or
                not diag.get("risk_ledger_terms") or
                not validated or any(value is None or value < 0 for value in
                                     (charged, declared, margin)) or
                not math.isclose(charged, declared, rel_tol=1e-12,
                                 abs_tol=1e-18) or
                not math.isclose(margin, risk_bound - charged, rel_tol=1e-12,
                                 abs_tol=1e-18) or charged > risk_bound + 1e-18):
            failures.append("closed final risk ledger is inconsistent")
    else:
        open_total = nonnegative_extended_float(diag, "risk_total")
        open_bound = nonnegative_extended_float(diag, "risk_upper_bound")
        if (open_total is None or open_bound is None or
                open_total > open_bound):
            failures.append("open risk ledger/bound invalid")
        explicit_debt = (integer(diag, "risk_ledger_unvalidated_terms") not in
                         (None, 0) or
                         integer(diag, "risk_ledger_not_implemented_terms") not in
                         (None, 0) or
                         (not identity_accepted and missing_identity and
                          "risk_proof_id" in
                          diag.get("publication_missing_identity_fields", "")))
        if validated or not explicit_debt:
            failures.append("open final risk ledger lacks explicit debt")
        if protected or truthy(integrity.get("formal_eligible")) or not refusal:
            failures.append("open final risk ledger was not fail-closed")
    certificate = integer(diag, "publication_certificate_id")
    if not protected and certificate not in (None, 0):
        failures.append("unprotected output carries a protection certificate")
    if transaction is not None:
        for key in ("transaction_id", "window_id", "selected_action_id"):
            if integer(transaction, key) != integer(integrity, key):
                failures.append(f"transaction {key} mismatch")
        if (integer(transaction, "backend_updates") !=
                integer(integrity, "backend_updates") or
                transaction.get("fde_status") != integrity.get("fde_status")):
            failures.append("transaction disposition mismatch")
    else:
        failures.append("terminal transaction record absent")
    if terminal is not None:
        terminal_pairs = (
            ("input_attempt_id", diag, "input_attempt_id"),
            ("transaction_id", integrity, "transaction_id"),
            ("window_id", integrity, "window_id"),
            ("selected_action_id", integrity, "selected_action_id"),
            ("backend_epoch_before", diag, "backend_epoch_before"),
            ("backend_epoch_after", diag, "backend_epoch_after"),
            ("batch_committed", integrity, "batch_committed"),
            ("risk_ledger_closes", diag, "risk_ledger_closes"),
            ("risk_ledger_all_validated", diag,
             "risk_ledger_all_validated"),
            ("selection_risk_proof_id", diag, "selection_risk_proof_id"),
            ("publication_risk_proof_id", diag,
             "publication_risk_proof_id"),
            ("publication_certificate_id", diag,
             "publication_certificate_id"),
            ("publication_protected", integrity,
             "publication_protected"),
            ("formal_eligible", integrity, "formal_eligible"),
            ("deadline_missed", integrity, "deadline_missed"),
            ("final_packet_digest", integrity, "final_packet_digest"))
        for terminal_key, source, source_key in terminal_pairs:
            if integer(terminal, terminal_key) != integer(source, source_key):
                failures.append(f"terminal packet {terminal_key} mismatch")
        call = integer(terminal, "publish_call_steady_ns")
        returned = integer(terminal, "publish_return_steady_ns")
        elapsed = integer(terminal, "arrival_to_publish_ns")
        if (call is None or returned is None or elapsed is None or
                call <= 0 or returned < call or elapsed < 0 or
                arrival_ms is None or
                not math.isclose(arrival_ms, elapsed / 1e6,
                                 rel_tol=1e-12, abs_tol=1e-9)):
            failures.append("publication boundary/timing mismatch")
    else:
        failures.append("terminal packet binding absent")
    return failures


def aggregate_profiles(root, analyzed, protocol, calibrations, frozen_execution):
    expected = []
    epochs = int(protocol["epochs"]["behavior"])
    seed = int(protocol["input"]["seeds"][0])
    for profile in protocol["profiles"]:
        for scenario in protocol["scenarios"]["profiles"].get(profile, []):
            run_id = f"{profile}__{scenario}__e{epochs}__s{seed}__r1"
            expected.append((run_id, profile, scenario))
    analyzed_runs = analyzed.get("runs", [])
    by_id = {item.get("run_id"): item for item in analyzed_runs}
    calibration_by_profile = {item["profile"]: item
                              for item in calibrations["profiles"]}
    path_metrics = {name: {"core": [], "analysis": [], "outer": [],
                           "attempts": 0, "complete": 0,
                           "deadline_misses": 0, "contract_failures": []}
                    for name in ("normal", "alarm_recovery", "fde_disabled",
                                 "unclassified")}
    core, analysis, arrival, attempt_details = [], [], [], []
    planned_attempts = epochs * len(expected)
    observed_attempts = complete_attempts = deadline_misses = 0
    process_exit_counts, metadata_failures = Counter(), []
    rss_values, per_run, source_identities = [], [], set()
    if len(by_id) != len(analyzed_runs):
        metadata_failures.append(
            {"run_id": "validation_summary", "failures": ["duplicate run_id"]})
    for run_id, profile, scenario in expected:
        run, item = root / run_id, by_id.get(run_id)
        if item is None or not (run / "validation_run.json").exists():
            process_exit_counts["MISSING_RUN"] += 1
            metadata_failures.append({"run_id": run_id,
                                      "failures": ["run/metadata missing"]})
            per_run.append({"run_id": run_id, "status": "MISSING"})
            continue
        try:
            identity = json.loads((run / "validation_run.json").read_text())
        except (OSError, ValueError) as error:
            identity = {}
            metadata_failures.append({"run_id": run_id,
                                      "failures": [f"invalid metadata: {error}"]})
        expected_identity = expected_run_identity(
            run_id, profile, scenario, epochs, seed, protocol,
            calibration_by_profile[profile], frozen_execution)
        failures = identity_failures(identity, expected_identity)
        if not terminal_artifacts_match(run, identity.get("terminal_artifacts")):
            failures.append("terminal raw artifact manifest mismatch")
        if failures:
            metadata_failures.append({"run_id": run_id, "failures": failures})
        source_identities.add((identity.get("git_sha"),
                               identity.get("dirty_diff_hash")))
        exit_code = identity.get("exit_code", -1)
        process_exit_counts[str(exit_code)] += 1
        csv_failures = []
        timing_rows = csv_rows(run / "timing.csv")
        timings = {}
        for offset, row in enumerate(timing_rows, 2):
            attempt, stage = integer(row, "epoch"), row.get("stage")
            value = finite_float(row, "wall_ms")
            if (attempt is None or not stage or value is None or value < 0 or
                    stage in timings.setdefault(attempt, {})):
                csv_failures.append(f"timing.csv:{offset}: invalid/duplicate stage")
            else:
                timings[attempt][stage] = row
        attempts = indexed(csv_rows(run / "diagnostic_attempts.csv"),
                           "input_attempt_id", csv_failures,
                           "diagnostic_attempts.csv")
        transactions = indexed(csv_rows(run / "transactions.csv"),
                               "transaction_id", csv_failures,
                               "transactions.csv")
        terminal_packets = indexed(csv_rows(run / "terminal_packets.csv"),
                                   "input_attempt_id", csv_failures,
                                   "terminal_packets.csv")
        diagnostic_stages = {}
        for offset, row in enumerate(csv_rows(run / "diagnostic_stages.csv"), 2):
            attempt, stage = integer(row, "input_attempt_id"), row.get("stage")
            value = finite_float(row, "wall_ms")
            if (attempt is None or not stage or value is None or value < 0 or
                    stage in diagnostic_stages.setdefault(attempt, {})):
                csv_failures.append(
                    f"diagnostic_stages.csv:{offset}: invalid/duplicate stage")
            else:
                diagnostic_stages[attempt][stage] = row
        candidates_by_window = {}
        for row in csv_rows(run / "candidates.csv"):
            candidates_by_window.setdefault(integer(row, "window_id"), []).append(row)
        integrity = csv_rows(run / "integrity.csv")
        count = int(item.get("counts", {}).get("attempted", 0))
        if (len(attempts) != count or len(integrity) != count or
                len(transactions) != count or len(terminal_packets) != count):
            csv_failures.append("attempt/integrity terminal row count mismatch")
        observed_attempts += count
        run_deadline = int(item.get("counts", {}).get("deadline_missed", 0))
        deadline_misses += run_deadline
        complete = 0
        for attempt_id in range(1, count + 1):
            stage_rows = timings.get(attempt_id, {})
            diag = attempts.get(attempt_id, {})
            integrity_row = integrity[attempt_id - 1] if attempt_id <= len(integrity) else {}
            candidates = candidates_by_window.get(integer(diag, "window_id"), [])
            path = acceptance_path(diag, candidates, integrity_row)
            bucket = path_metrics[path]
            bucket["attempts"] += 1
            deadline = truthy(integrity_row.get("deadline_missed"))
            bucket["deadline_misses"] += int(deadline)
            required = [stage_rows.get(stage) for stage in REQUIRED_TIMING_STAGES]
            is_complete = (bool(diag) and all(required) and
                           all(row.get("success") == "1" for row in required) and
                           diag.get("status") in
                           ("EXECUTED", "COMMITTED", "DISCARDED"))
            detail = {"run_id": run_id, "attempt": attempt_id, "path": path,
                      "complete_work": is_complete, "deadline_missed": deadline,
                      "timeout": truthy(diag.get("watchdog_wall_timeout"))}
            if all(required):
                detail.update({"core_ms": finite_float(required[0], "wall_ms"),
                               "analysis_completion_ms": finite_float(required[1], "wall_ms"),
                               "arrival_to_publish_ms": finite_float(required[2], "wall_ms")})
            if is_complete and all(detail.get(key) is not None for key in
                                   ("core_ms", "analysis_completion_ms",
                                    "arrival_to_publish_ms")):
                complete += 1
                values = (detail["core_ms"], detail["analysis_completion_ms"],
                          detail["arrival_to_publish_ms"])
                core.append(values[0]); analysis.append(values[1]); arrival.append(values[2])
                bucket["core"].append(values[0]); bucket["analysis"].append(values[1])
                bucket["outer"].append(values[2]); bucket["complete"] += 1
            elif is_complete:
                is_complete = detail["complete_work"] = False
            contract = []
            partition = proof_partition(diag)
            disabled_partition = (
                path == "fde_disabled" and profile == "off" and
                all(integer(diag, key) == 0 for key in (
                    "coverage_exact_leaves", "coverage_enveloped_leaves",
                    "coverage_uncovered_leaves", "coverage_proof_count",
                    "coverage_envelope_count", "hypothesis_count",
                    "generated_actions", "kernel_evaluated_actions")))
            if not partition["valid"] and not disabled_partition:
                contract.append({"leaf_partition": partition})
            contract.extend(publication_failures(
                diag, integrity_row, candidates, deadline, is_complete,
                transactions.get(integer(integrity_row, "transaction_id")),
                terminal_packets.get(attempt_id),
                detail.get("arrival_to_publish_ms")))
            if not is_complete:
                contract.append("missing/non-finite/failed terminal timing")
            if contract:
                bucket["contract_failures"].append(
                    {"run_id": run_id, "attempt": attempt_id,
                     "failures": contract})
            detail["contract_failures"] = contract
            attempt_details.append(detail)
        complete_attempts += complete
        rss = rss_kib(run)
        if rss is not None:
            rss_values.append(rss)
        else:
            csv_failures.append("RSS evidence absent")
        if csv_failures:
            metadata_failures.append({"run_id": run_id,
                                      "failures": csv_failures})
        counts = item.get("counts", {})
        accuracy = item.get("accuracy", {})
        per_run.append({
            "run_id": run_id, "profile": profile, "scenario": scenario,
            "exit_code": exit_code, "planned_attempts": epochs,
            "observed_attempts": count, "complete_attempts": complete,
            "deadline_misses": run_deadline,
            "committed": int(counts.get("committed", 0)),
            "valid_states": int(counts.get("state_valid", 0)),
            "finite_pl": int(counts.get("finite_pl", 0)),
            "within_alert_limits": int(counts.get("within_alert_limits", 0)),
            "position_rmse_m": accuracy.get("truth_at_state_position_rmse_m"),
            "position_p95_m": accuracy.get("truth_at_state_position_p95_m"),
            "attitude_rmse_rad": accuracy.get("attitude_geodesic_rmse_rad"),
            "velocity_rmse_mps": accuracy.get("velocity_rmse_mps"),
            "rss_peak_kib": rss,
            "fde_status_counts": item.get("action_and_status_counts", {}).get("fde_status", {}),
            "selected_action_counts": item.get("action_and_status_counts", {}).get("selected_action", {})})
    if len(source_identities) != 1:
        metadata_failures.append(
            {"run_id": "campaign", "failures":
             ["runs do not share one current source identity"]})
    missing_attempts = planned_attempts - observed_attempts
    failed_runs = sum(value for key, value in process_exit_counts.items() if key != "0")
    aggregate = {
        "expected_runs": len(expected), "observed_runs": len(by_id),
        "process_exit_counts": dict(sorted(process_exit_counts.items())),
        "failed_runs": failed_runs, "planned_attempts": planned_attempts,
        "observed_attempts": observed_attempts, "missing_attempts": missing_attempts,
        "complete_work_attempts": complete_attempts,
        "complete_work_rate_over_planned": complete_attempts / planned_attempts,
        "deadline_misses_observed": deadline_misses,
        "deadline_miss_rate_over_planned": deadline_misses / planned_attempts,
        "core_compute": distribution(core),
        "analysis_completion": distribution(analysis),
        "arrival_to_publish": distribution(arrival),
        "rss_peak_kib": max(rss_values) if rss_values else None,
        "metadata_failures": metadata_failures,
        "attempt_details": attempt_details, "acceptance_paths": {}}
    for name, bucket in path_metrics.items():
        count = bucket["attempts"]
        aggregate["acceptance_paths"][name] = {
            "attempts": count, "complete_attempts": bucket["complete"],
            "complete_work_rate": bucket["complete"] / count if count else None,
            "deadline_misses": bucket["deadline_misses"],
            "deadline_miss_rate": bucket["deadline_misses"] / count if count else None,
            "core_compute": distribution(bucket["core"]),
            "analysis_completion": distribution(bucket["analysis"]),
            "arrival_to_publish": distribution(bucket["outer"]),
            "contract_failures": bucket["contract_failures"]}
    return aggregate, per_run


def fgo_gates(analyzed, gates, protocol, calibrations, frozen_execution,
              campaign_root=None):
    failures = []
    runs = analyzed.get("runs", [])
    scenarios = protocol["scenarios"]["fgo"]
    if len(runs) != 4 or {item.get("scenario") for item in runs} != set(scenarios):
        failures.append("requires exactly four named FGO processes")
    calibration = next(item for item in calibrations["profiles"]
                       if item["profile"] == "off")
    by_scenario = {}
    for item in runs:
        scenario = item.get("scenario")
        by_scenario[scenario] = item
        epochs = int(protocol["epochs"]["fgo"])
        seed = int(protocol["input"]["seeds"][0])
        run_id = f"off__{scenario}__e{epochs}__s{seed}__r1"
        expected_id = expected_run_identity(
            run_id, "off", scenario, epochs, seed, protocol, calibration,
            frozen_execution)
        expected_id.update({"gate": "fgo", "scale": "full"})
        failures.extend(f"{scenario}: {value}" for value in
                        identity_failures(item, expected_id))
        try:
            if campaign_root is None:
                raise ValueError("FGO campaign root is required")
            artifact = bound_campaign_child(
                Path(campaign_root), item.get("artifact_directory", ""), run_id)
        except (OSError, ValueError) as error:
            failures.append(f"{scenario}: invalid artifact path: {error}")
            continue
        if not terminal_artifacts_match(
                artifact, item.get("terminal_artifacts")):
            failures.append(f"{scenario}: terminal raw artifact manifest mismatch")
        try:
            raw = analyze_run(artifact)
        except (OSError, ValueError, KeyError, TypeError) as error:
            raw = {}
            failures.append(f"{scenario}: raw FGO analysis failed: {error}")
        raw_counts = raw.get("counts", {})
        raw_accuracy = raw.get("accuracy", {})
        state_rows = csv_rows(artifact / "states.csv")
        truth_rows = csv_rows(artifact / "ground_truth.csv")
        integrity_rows = csv_rows(artifact / "integrity.csv")
        attempt_rows = csv_rows(artifact / "diagnostic_attempts.csv")
        terminal_rows = csv_rows(artifact / "terminal_packets.csv")
        expected_attempts = list(range(1, epochs + 1))
        attempt_ids = [integer(row, "input_attempt_id") for row in attempt_rows]
        terminal_ids = [integer(row, "input_attempt_id") for row in terminal_rows]
        state_times = [integer(row, "timestamp_ns") for row in state_rows]
        truth_times = [integer(row, "timestamp_ns") for row in truth_rows]
        integrity_times = [integer(row, "state_timestamp_ns")
                           for row in integrity_rows]
        attempted_times = [integer(row, "attempted_timestamp_ns")
                           for row in integrity_rows]
        diagnostic_input_times = [integer(row, "input_timestamp_ns")
                                  for row in attempt_rows]
        diagnostic_output_times = [integer(row, "output_timestamp_ns")
                                   for row in attempt_rows]
        if (len(state_rows) != epochs or len(truth_rows) != epochs or
                len(integrity_rows) != epochs or len(attempt_rows) != epochs or
                len(terminal_rows) != epochs or attempt_ids != expected_attempts or
                terminal_ids != expected_attempts or
                any(value is None for value in state_times + truth_times +
                    integrity_times + attempted_times +
                    diagnostic_input_times + diagnostic_output_times) or
                state_times != sorted(state_times) or
                truth_times != sorted(truth_times) or
                attempted_times != sorted(attempted_times) or
                len(set(state_times)) != epochs or
                len(set(truth_times)) != epochs or
                len(set(attempted_times)) != epochs or
                state_times != truth_times or state_times != integrity_times or
                state_times != attempted_times or
                state_times != diagnostic_input_times or
                state_times != diagnostic_output_times):
            failures.append(
                f"{scenario}: raw FGO terminal identity/200-row contract mismatch")
        finite_fields(
            state_rows,
            ("px", "py", "pz", "qw", "qx", "qy", "qz", "vx", "vy", "vz",
             "bax", "bay", "baz", "bgx", "bgy", "bgz"),
            f"{scenario}:states.csv", failures)
        finite_fields(truth_rows,
                      ("px", "py", "pz", "qw", "qx", "qy", "qz"),
                      f"{scenario}:ground_truth.csv", failures)
        valid_quaternions(state_rows, f"{scenario}:states.csv", failures)
        valid_quaternions(truth_rows, f"{scenario}:ground_truth.csv", failures)
        if int(raw_counts.get("attempted", -1)) != epochs:
            failures.append(f"{scenario}: FGO run is not {epochs}/{epochs}")
        for key, value in raw_counts.items():
            if item.get("counts", {}).get(key) != value:
                failures.append(f"{scenario}: summary counts.{key} raw mismatch")
        for key, value in raw_accuracy.items():
            reported = item.get("accuracy", {}).get(key)
            if not same_analyzed_value(reported, value):
                failures.append(f"{scenario}: summary accuracy.{key} raw mismatch")
        valid_counts = raw_accuracy.get("valid_counts", {})
        required_valid_counts = {
            "position": epochs, "attitude": epochs, "velocity": epochs,
            "accel_bias": epochs, "gyro_bias": epochs,
            "service_position": epochs,
        }
        if valid_counts != required_valid_counts:
            failures.append(
                f"{scenario}: accuracy valid_count is not the explicit "
                f"{epochs}/{epochs} protocol denominator")
        # Quality gates below consume only this independently recomputed view.
        if raw:
            by_scenario[scenario] = dict(item, counts=raw_counts,
                                         accuracy=raw_accuracy)
    checks = {}
    try:
        noiseless = by_scenario["noiseless"]["accuracy"]
        nominal = by_scenario["nominal"]["accuracy"]
        checks = {
            "noiseless_position_rmse": noiseless["truth_at_state_position_rmse_m"] <= float(gates["noiseless_position_rmse_m_max"]),
            "noiseless_attitude_rmse": noiseless["attitude_geodesic_rmse_rad"] <= float(gates["noiseless_attitude_rmse_rad_max"]),
            "nominal_position_rmse": nominal["truth_at_state_position_rmse_m"] <= float(gates["nominal_position_rmse_m_max"]),
            "nominal_position_p95": nominal["truth_at_state_position_p95_m"] <= float(gates["nominal_position_p95_m_max"]),
            "nominal_attitude_rmse": nominal["attitude_geodesic_rmse_rad"] <= float(gates["nominal_attitude_rmse_rad_max"]),
            "nominal_velocity_rmse": nominal["velocity_rmse_mps"] <= float(gates["nominal_velocity_rmse_mps_max"]),
        }
    except (KeyError, TypeError):
        failures.append("FGO accuracy evidence incomplete")
    return {"status": "PASS" if checks and all(checks.values()) and not failures else "FAIL",
            "checks": checks, "failures": failures,
            "runs": {key: by_scenario[key] for key in sorted(by_scenario)}}


def profile_quality_gates(per_run, protocol):
    gates, checks, not_applicable = protocol["gates"], {}, {}
    for run in per_run:
        if run.get("status") == "MISSING":
            continue
        count = run["observed_attempts"]
        key = run["run_id"]
        checks[f"{key}:valid_new_state_fraction"] = (
            count > 0 and run["valid_states"] / count >=
            float(gates["valid_new_state_fraction_min"]))
        if run["scenario"] == "nominal" and run["profile"] != "off":
            checks[f"{key}:finite_pl_fraction"] = (
                count > 0 and run["finite_pl"] / count >=
                float(gates["active_nominal_finite_pl_fraction_min"]))
            checks[f"{key}:within_alert_fraction"] = (
                count > 0 and run["within_alert_limits"] / count >=
                float(gates["active_nominal_within_alert_fraction_min"]))
        if run["scenario"].endswith("_recovery"):
            checks[f"{key}:recovered_finite_epochs"] = (
                run["finite_pl"] >= int(gates["recovered_finite_epochs_min"]))
            checks[f"{key}:recovered_within_alert_fraction"] = (
                run["finite_pl"] > 0 and
                run["within_alert_limits"] / run["finite_pl"] >=
                float(gates["recovered_within_alert_fraction_min"]))
    not_applicable["horizontal_alert_limit_m"] = (
        "threshold is exercised by within_alert_limits; not a separate outcome gate")
    not_applicable["vertical_alert_limit_m"] = (
        "threshold is exercised by within_alert_limits; not a separate outcome gate")
    return {"status": "PASS" if checks and all(checks.values()) else "FAIL",
            "checks": checks, "not_applicable": not_applicable}


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_data_row_count(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return sum(1 for _ in csv.DictReader(stream))


def same_number(left, right, tolerance=1e-12):
    return (isinstance(left, (int, float)) and
            isinstance(right, (int, float)) and
            math.isfinite(float(left)) and math.isfinite(float(right)) and
            math.isclose(float(left), float(right), rel_tol=tolerance,
                         abs_tol=tolerance))


def same_analyzed_value(reported, recomputed):
    if recomputed is None:
        return reported is None
    if isinstance(recomputed, list):
        return (isinstance(reported, list) and
                len(reported) == len(recomputed) and
                all(same_analyzed_value(a, b)
                    for a, b in zip(reported, recomputed)))
    if isinstance(recomputed, (int, float)) and not isinstance(recomputed, bool):
        return same_number(reported, recomputed)
    return reported == recomputed


def bound_campaign_child(root, recorded, child_name):
    """Return an exact in-root child, rejecting lexical and symlink escapes."""
    root = root.resolve()
    lexical = Path(str(recorded))
    if ".." in lexical.parts:
        raise ValueError("path contains '..'")
    expected = root / child_name
    actual = lexical if lexical.is_absolute() else root / lexical
    if actual.resolve() != expected.resolve():
        raise ValueError("path is not the expected campaign child")
    try:
        expected.resolve().relative_to(root)
    except ValueError as error:
        raise ValueError("path escapes campaign root") from error
    cursor = root
    for part in Path(child_name).parts:
        cursor = cursor / part
        if cursor.is_symlink():
            raise ValueError("campaign child uses a symlink")
    if not expected.is_dir():
        raise ValueError("campaign child is not a directory")
    return expected.resolve()


def finite_fields(rows, fields, label, failures):
    for offset, row in enumerate(rows, 2):
        for field in fields:
            value = finite_float(row, field)
            if value is None:
                failures.append(f"{label}:{offset}:{field} is non-finite/missing")


def valid_quaternions(rows, label, failures):
    fields = ("qw", "qx", "qy", "qz")
    for offset, row in enumerate(rows, 2):
        values = [finite_float(row, field) for field in fields]
        if any(value is None for value in values):
            continue
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or norm <= 1e-12:
            failures.append(f"{label}:{offset}: quaternion is not normalizable")


def validate_gate24_run(root, run, index, frozen_execution=None):
    failures = []
    expected_ids = list(range(1, 46))
    outcomes = run.get("raw_attempt_outcomes", [])
    if len(outcomes) != 45:
        failures.append("raw_attempt_outcomes denominator !=45")
    ids = [item.get("attempt") for item in outcomes]
    if ids != expected_ids or len(set(ids)) != 45:
        failures.append("attempt IDs are not the unique expected 1..45 set")
    try:
        run_dir = bound_campaign_child(
            root, run.get("run_directory", ""), f"run{index}")
    except (OSError, ValueError) as error:
        failures.append(f"run_directory is not exact in-root run{index}: {error}")
        return failures, {"core": [], "analysis": [], "arrival": [],
                          "deadline": 0, "rss": None}
    if frozen_execution is not None:
        try:
            marker = json.loads(
                (run_dir / "validation_run.json").read_text(encoding="utf-8"))
            for key, value in frozen_execution.items():
                if marker.get(key) != value:
                    failures.append(f"run identity {key} mismatch")
            required = {"gate": "performance", "profile": "joint_order2",
                        "scenario": "nominal", "epochs": 45,
                        "seed": 20260928, "repeat": index,
                        "fault_overrides": {}, "exit_code": 0}
            for key, value in required.items():
                if marker.get(key) != value:
                    failures.append(f"run workload {key} mismatch")
            if not terminal_artifacts_match(
                    run_dir, marker.get("terminal_artifacts")):
                failures.append("run terminal raw artifact manifest mismatch")
        except (OSError, ValueError, TypeError) as error:
            failures.append(f"run identity unreadable: {error}")
    timing_failures = []
    timing_rows = csv_rows(run_dir / "timing.csv")
    timings = {}
    for offset, row in enumerate(timing_rows, 2):
        attempt, stage = integer(row, "epoch"), row.get("stage")
        value = finite_float(row, "wall_ms")
        if (attempt not in expected_ids or not stage or value is None or value < 0 or
                stage in timings.setdefault(attempt, {})):
            timing_failures.append(f"timing.csv:{offset} invalid/duplicate")
        else:
            timings[attempt][stage] = (value, row.get("success") == "1")
    attempts = indexed(csv_rows(run_dir / "diagnostic_attempts.csv"),
                       "input_attempt_id", timing_failures,
                       "diagnostic_attempts.csv")
    integrity = csv_rows(run_dir / "integrity.csv")
    if set(attempts) != set(expected_ids) or len(integrity) != 45:
        timing_failures.append("terminal raw attempt denominator/ID mismatch")
    core, analysis, arrival, deadline = [], [], [], 0
    for attempt_id in expected_ids:
        stages = timings.get(attempt_id, {})
        required = [stages.get(name) for name in
                    ("core_total", "analysis_completion", "arrival_to_publish")]
        outcome = outcomes[attempt_id - 1] if len(outcomes) >= attempt_id else {}
        diag = attempts.get(attempt_id, {})
        integ = integrity[attempt_id - 1] if len(integrity) >= attempt_id else {}
        raw_complete = (diag.get("status") == "EXECUTED" and all(required) and
                        all(item[1] for item in required))
        raw_deadline = truthy(integ.get("deadline_missed"))
        raw_timeout = truthy(diag.get("watchdog_wall_timeout"))
        if (outcome.get("complete_work") is not raw_complete or
                outcome.get("status") != diag.get("status") or
                outcome.get("deadline_missed") is not raw_deadline or
                outcome.get("timeout") is not raw_timeout or
                outcome.get("sensor_stale") is not
                    truthy(diag.get("watchdog_sensor_stale")) or
                outcome.get("watchdog_reason") != diag.get("watchdog_reason") or
                outcome.get("terminal_reason") != diag.get("reason") or
                outcome.get("transaction_opened") is not
                    truthy(diag.get("transaction_opened"))):
            timing_failures.append(f"attempt {attempt_id} outcome mismatch")
        if not raw_complete:
            timing_failures.append(f"attempt {attempt_id} incomplete")
            continue
        values = [item[0] for item in required]
        core.append(values[0]); analysis.append(values[1]); arrival.append(values[2])
        deadline += int(raw_deadline)
    failures.extend(timing_failures)
    for key, values in (("raw_core_ms", core),
                        ("raw_analysis_completion_ms", analysis),
                        ("raw_arrival_to_publish_ms", arrival)):
        reported = run.get(key, [])
        if (len(reported) != 45 or any(not same_number(a, b) for a, b in
                                       zip(reported, values))):
            failures.append(f"{key} does not match raw timing.csv")
    for key, values in (("core_compute", core),
                        ("analysis_completion", analysis),
                        ("arrival_to_publish", arrival)):
        reported, computed = run.get(key, {}), distribution(values)
        for metric in ("count", "p50_ms", "p95_ms", "p99_ms", "max_ms"):
            if metric == "count":
                if reported.get(metric) != computed.get(metric):
                    failures.append(f"{key}.{metric} mismatch")
            elif not same_number(reported.get(metric), computed.get(metric)):
                failures.append(f"{key}.{metric} mismatch")
    if (run.get("attempted") != 45 or run.get("complete_work_attempts") != 45 or
            run.get("excluded") != 0 or run.get("complete_work_rate") != 1.0 or
            run.get("deadline_misses") != deadline or
            not same_number(run.get("deadline_miss_rate"), deadline / 45)):
        failures.append("run counters are not raw-derived strict 45/45")
    time_path = (run_dir / "stderr.log" if (run_dir / "stderr.log").exists()
                 else run_dir.parent / f"{run_dir.name}.time")
    try:
        match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)",
                          time_path.read_text(encoding="utf-8"))
        raw_rss = int(match.group(1)) if match else None
    except OSError:
        raw_rss = None
    if raw_rss is None or run.get("rss_peak_kib") != raw_rss:
        failures.append("RSS does not match raw /usr/bin/time record")
    return failures, {"core": core, "analysis": analysis,
                      "arrival": arrival, "deadline": deadline,
                      "rss": raw_rss}


def validate_gate24(root, frozen_execution=None, golden_copy_root=None,
                    protocol_path=None):
    failures = []
    root = root.resolve()
    try:
        payload = json.loads((root / "all-attempt-performance.json").read_text())
        groups = payload["groups"]
        if len(groups) != 1:
            failures.append("gate must contain exactly one 3-run group")
            group = {}
        else:
            group = groups[0]
        aggregate, runs = group.get("aggregate", {}), group.get("runs", [])
        for key, expected in {"attempted": 135, "complete_work_attempts": 135,
                              "excluded": 0}.items():
            if aggregate.get(key) != expected:
                failures.append(f"aggregate {key} != {expected}")
        if aggregate.get("complete_work_rate") != 1.0:
            failures.append("aggregate complete-work rate != 1")
        rss = aggregate.get("rss_peak_kib")
        if not isinstance(rss, (int, float)) or rss >= 1024 * 1024:
            failures.append("aggregate RSS missing or >=1GiB")
        if len(runs) != 3:
            failures.append("gate does not contain three runs")
        raw_runs = []
        for index, run in enumerate(runs, 1):
            local, raw = validate_gate24_run(root, run, index, frozen_execution)
            raw_runs.append(raw)
            failures.extend(f"run{index}: {value}" for value in local)
        all_core = [value for run in raw_runs for value in run["core"]]
        all_analysis = [value for run in raw_runs for value in run["analysis"]]
        all_arrival = [value for run in raw_runs for value in run["arrival"]]
        expected_aggregate = {
            "attempted": 135, "complete_work_attempts": 135, "excluded": 0,
            "complete_work_rate": 1.0,
            "deadline_misses": sum(run["deadline"] for run in raw_runs),
            "deadline_miss_rate": sum(run["deadline"] for run in raw_runs) / 135,
            "rss_peak_kib": max((run["rss"] for run in raw_runs
                                 if run["rss"] is not None), default=None),
            "core_compute": distribution(all_core),
            "analysis_completion": distribution(all_analysis),
            "arrival_to_publish": distribution(all_arrival),
        }
        for key in ("deadline_misses", "deadline_miss_rate", "rss_peak_kib"):
            expected_value = expected_aggregate[key]
            if ((isinstance(expected_value, float) and
                 not same_number(aggregate.get(key), expected_value)) or
                    (not isinstance(expected_value, float) and
                     aggregate.get(key) != expected_value)):
                failures.append(f"aggregate {key} raw mismatch")
        for key in ("core_compute", "analysis_completion", "arrival_to_publish"):
            for metric in ("count", "p50_ms", "p95_ms", "p99_ms", "max_ms"):
                got, expected_value = aggregate.get(key, {}).get(metric), \
                    expected_aggregate[key].get(metric)
                if ((metric == "count" and got != expected_value) or
                        (metric != "count" and not same_number(got, expected_value))):
                    failures.append(f"aggregate {key}.{metric} raw mismatch")
        identity = json.loads((root / "gate24_run_identity.json").read_text())
        protocol_file = (Path(protocol_path).resolve() if protocol_path else
                         Path(__file__).resolve().parents[1] /
                         "config/fde_profiles_validation.yaml")
        anchor = golden_anchor_from_protocol(yaml.safe_load(
            protocol_file.read_text(encoding="utf-8")))
        golden_reference = anchor["reference"]
        expected_identity_anchor = {
            "golden_reference": golden_reference,
            "golden_annotated_tag_object": anchor["annotated_tag_object"],
            "golden_peeled_commit": anchor["peeled_commit"],
            "golden_tree": anchor["tree"],
            "golden_manifest_sha256": anchor["manifest_sha256"],
            "golden_anchor_schema": anchor["schema"],
        }
        for key, value in expected_identity_anchor.items():
            if identity.get(key) != value:
                failures.append(f"gate identity {key} differs from pinned anchor")
        recorded_golden = json.loads(
            (root / "golden_reference_manifest.json").read_text())
        expected_golden = golden_reference_manifest(
            golden_reference,
            protocol={"golden_reference_anchor": anchor})
        if recorded_golden != expected_golden:
            failures.append("golden tag commit/blob manifest mismatch")
        golden_copy = (Path(golden_copy_root).resolve()
                       if golden_copy_root is not None else
                       Path(__file__).resolve().parents[1] /
                       GATE24_GOLDEN_PREFIX)
        for run_name, manifest_files in expected_golden["runs"].items():
            for name, metadata in manifest_files.items():
                local_path = golden_copy / run_name / name
                if (local_path.is_symlink() or not local_path.is_file() or
                        file_sha256(local_path) != metadata["sha256"] or
                        local_path.stat().st_size != metadata["bytes"]):
                    failures.append(
                        f"golden working copy differs from tag blob: "
                        f"{run_name}/{name}")
        if frozen_execution:
            campaign_identity = json.loads(
                (root / "campaign_identity.json").read_text())
            if campaign_identity != frozen_execution:
                failures.append("gate campaign identity mismatch")
            for key, value in frozen_execution.items():
                if identity.get(key) != value:
                    failures.append(f"gate identity {key} mismatch")
    except (OSError, ValueError, KeyError, TypeError) as error:
        aggregate = {}
        failures.append(f"gate evidence unreadable/incomplete: {error}")
    equivalence = []
    for index in (1, 2, 3):
        path = root / f"equivalence-run{index}.json"
        try:
            item = json.loads(path.read_text())
            local = []
            if item.get("status") != "PASS":
                local.append("top-level status != PASS")
            before = Path(str(item.get("before", ""))).resolve()
            after = Path(str(item.get("after", ""))).resolve()
            expected_before = (Path(__file__).resolve().parents[1] /
                "docs/evidence/p1-05-deterministic-concurrency/archived-raw/"
                "candidate-3x45" / f"run{index}").resolve()
            if before != expected_before or not before.is_dir():
                local.append("comparison input is not exact P1-05 runN")
            if (index <= len(runs) and after !=
                    Path(str(runs[index - 1].get("run_directory", ""))).resolve()):
                local.append("comparison output is not gate runN")
            files = item.get("files", {})
            if set(files) != set(STRICT_EQUIVALENCE_FILES):
                local.append("strict file set mismatch")
            try:
                recomputed = compare_directories(before, after, gate24_audit=True)
            except (OSError, ValueError, KeyError, TypeError) as error:
                recomputed = {}
                local.append(f"canonical comparator failed: {error}")
            if recomputed:
                for key in ("schema", "status", "allowed_semantic_delta",
                            "additive_diagnostic", "performance_ignores",
                            "audit_exact_fields"):
                    if item.get(key) != recomputed.get(key):
                        local.append(f"comparator evidence {key} mismatch")
                if files != recomputed.get("files"):
                    local.append("comparator evidence files mismatch")
                if recomputed.get("status") != "PASS":
                    local.append("canonical raw comparator result != PASS")
            equivalence.append({"run": index,
                                "status": "PASS" if not local else "FAIL",
                                "failures": local})
            failures.extend(f"equivalence-run{index}: {value}" for value in local)
        except (OSError, ValueError, KeyError, TypeError) as error:
            equivalence.append({"run": index, "status": "FAIL",
                                "failures": [str(error)]})
            failures.append(f"equivalence-run{index} unreadable: {error}")
    return {"status": "PASS" if not failures else "FAIL",
            "failures": failures, "equivalence": equivalence,
            "aggregate": aggregate}


def revised_nonrealtime_pass(profiles, contract_pass, resource_pass,
                             fgo, quality, gate24):
    profile_execution_pass = (
        profiles["failed_runs"] == 0 and profiles["missing_attempts"] == 0 and
        profiles["complete_work_rate_over_planned"] == 1.0 and
        not profiles["metadata_failures"])
    return (profile_execution_pass and contract_pass and resource_pass and
            fgo["status"] == "PASS" and quality["status"] == "PASS" and
            gate24["status"] == "PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profiles", required=True, type=Path)
    parser.add_argument("--fgo", required=True, type=Path)
    parser.add_argument("--gate24", required=True, type=Path)
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--calibrations", type=Path,
                        default=Path(__file__).resolve().parents[1] /
                        "config/p1_06_simulation_calibrations.json")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    protocol = yaml.safe_load(args.protocol.read_text(encoding="utf-8"))
    calibrations = validate_calibrations(args.calibrations.resolve())
    profiles_execution = verified_campaign_identity(args.profiles)
    fgo_execution = verified_campaign_identity(args.fgo)
    if profiles_execution != fgo_execution:
        raise SystemExit("profile and FGO campaigns have different identities")
    analyzed_profiles = json.loads(
        (args.profiles / "validation_summary.json").read_text())
    analyzed_fgo = json.loads((args.fgo / "validation_summary.json").read_text())
    profiles, per_run = aggregate_profiles(
        args.profiles, analyzed_profiles, protocol, calibrations,
        profiles_execution)
    paths = profiles["acceptance_paths"]
    normal, alarm, disabled, unclassified = (
        paths["normal"], paths["alarm_recovery"], paths["fde_disabled"],
        paths["unclassified"])
    profile_execution_pass = (
        profiles["failed_runs"] == 0 and profiles["missing_attempts"] == 0 and
        profiles["complete_work_rate_over_planned"] == 1.0 and
        not profiles["metadata_failures"])
    contract_pass = (unclassified["attempts"] == 0 and
                     not normal["contract_failures"] and
                     not alarm["contract_failures"] and
                     not disabled["contract_failures"] and
                     alarm["attempts"] > 0 and disabled["attempts"] > 0)
    resource_pass = (profiles["rss_peak_kib"] is not None and
                     profiles["rss_peak_kib"] < 1024 * 1024)
    normal_gate = protocol["acceptance_paths"]["normal"]
    normal_realtime_pass = (
        normal["attempts"] > 0 and normal["complete_work_rate"] == 1.0 and
        normal["core_compute"]["p99_ms"] is not None and
        normal["core_compute"]["p99_ms"] <= float(normal_gate["core_p99_ms_max"]) and
        normal["arrival_to_publish"]["p99_ms"] is not None and
        normal["arrival_to_publish"]["p99_ms"] <=
            float(normal_gate["arrival_to_publish_p99_ms_max"]) and
        normal["deadline_miss_rate"] <=
            float(normal_gate["deadline_miss_fraction_max"]))
    fgo = fgo_gates(analyzed_fgo, protocol["gates"], protocol, calibrations,
                    profiles_execution, args.fgo)
    quality = profile_quality_gates(per_run, protocol)
    gate24 = validate_gate24(args.gate24, profiles_execution,
                             protocol_path=args.protocol)
    revised_pass = revised_nonrealtime_pass(
        profiles, contract_pass, resource_pass, fgo, quality, gate24)
    payload = {
        "schema": "uwb-imu-pl/p1-06-revised-nonrealtime-acceptance/v3",
        "simulation_acceptance": {
            "status": "PASS" if revised_pass else "FAIL",
            "scope": "REVISED_NONREALTIME_GOAL_ONLY",
            "original_section_2_full_acceptance": "NOT_CLAIMED",
            "artifact_class": "simulation/synthetic",
            "hardware_formal_qualification": "CLOSED/NOT_CLAIMED",
            "production_formal_eligible": False,
            "production_protected_output": False},
        "fgo_quality": fgo, "profile_quality": quality,
        "all_profile_scenarios": profiles,
        "all_profile_execution_status": "PASS" if profile_execution_pass else "FAIL",
        "normal_realtime_status": ("PASS / DEFERRED_TO_REALTIME_GOAL" if
                                   normal_realtime_pass else
                                   "FAILED / DEFERRED_TO_REALTIME_GOAL"),
        "realtime_gate_blocks_revised_goal": False,
        "all_attempt_contract_status": "PASS" if contract_pass else "FAIL",
        "all_attempt_resource_status": "PASS" if resource_pass else "FAIL",
        "gate_2_4": gate24,
        "pl_qualification_boundary": {
            "status": "NOT_APPLICABLE_TO_SIMULATION_PASS",
            "reason": "production PL/protected-output qualification remains hardware-gated",
            "finite_pl_observations": sum(item.get("finite_pl", 0) for item in per_run)},
        "per_run": per_run}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True,
                                      allow_nan=False) + "\n")
    print(json.dumps({"status": payload["simulation_acceptance"]["status"],
                      "failed_runs": profiles["failed_runs"],
                      "missing_attempts": profiles["missing_attempts"],
                      "metadata_failures": len(profiles["metadata_failures"]),
                      "complete_work_rate": profiles["complete_work_rate_over_planned"],
                      "core_p99_ms": profiles["core_compute"]["p99_ms"],
                      "rss_peak_kib": profiles["rss_peak_kib"]}, sort_keys=True))
    return 0 if revised_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
