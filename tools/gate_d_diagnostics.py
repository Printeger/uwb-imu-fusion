"""Independent Gate-D diagnostic attachment schema and strict attempt joins."""
from collections import Counter, defaultdict
import csv
import math

SCHEMA = "uwb-imu-pl/gate-d-diagnostics/v1"
IDENTITY = "schema_version input_attempt_id input_timestamp_ns transaction_id window_id graph_version ordering_version noise_model_version linpoint_version output_timestamp_ns".split()
STAGES = set("prepare integrity_window window_boundary_provenance window_factor_linearization_whitening window_dense_assembly window_svd window_normal_equations window_llt_state_solves window_fingerprint all_in_detector current_sensitivity historical_sensitivity model_generation hypothesis_evidence health_actions hypothesis_audit base_factorization candidate_evaluation fde_decision candidate_audit finalize_commit_audit core_total".split())


def read_rows(directory, name):
    with (directory / name).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0:
        raise ValueError(f"invalid nonnegative duration: {value!r}")
    return parsed


def validate_attachments(directory, epochs=None, warmup=100, expected_candidates=None, require_legacy_timing=True):
    attempts = read_rows(directory, "diagnostic_attempts.csv")
    stages = read_rows(directory, "diagnostic_stages.csv")
    candidates = read_rows(directory, "diagnostic_candidates.csv")
    expected = list(range(1, (epochs if epochs is not None else len(attempts)) + 1))
    ids = [int(row["input_attempt_id"]) for row in attempts]
    if ids != expected:
        raise ValueError("missing, duplicate or unordered input attempt IDs")
    indexed = dict(zip(ids, attempts))
    if any(row["schema_version"] not in (SCHEMA, "uwb-imu-pl/gate-d-diagnostics/v2", "uwb-imu-pl/gate-d-diagnostics/v3", "uwb-imu-pl/gate-d-diagnostics/v4", "uwb-imu-pl/gate-d-diagnostics/v5", "uwb-imu-pl/gate-d-diagnostics/v6", "uwb-imu-pl/gate-d-diagnostics/v7", "uwb-imu-pl/gate-d-diagnostics/v8", "uwb-imu-pl/gate-d-diagnostics/v9", "uwb-imu-pl/gate-d-diagnostics/v10", "uwb-imu-pl/gate-d-diagnostics/v11", "uwb-imu-pl/gate-d-diagnostics/v12", "uwb-imu-pl/gate-d-diagnostics/v13") for row in attempts):
        raise ValueError("unsupported diagnostic schema")
    by_stage, by_candidate = defaultdict(dict), defaultdict(dict)
    for rows, target, field in ((stages, by_stage, "stage"), (candidates, by_candidate, "action_id")):
        for row in rows:
            attempt = int(row["input_attempt_id"])
            parent = indexed.get(attempt)
            if parent is None or any(row[key] != parent[key] for key in IDENTITY):
                raise ValueError(f"{field}: attempt/transaction/window/version mismatch")
            key = row[field]
            if key in target[attempt]:
                raise ValueError(f"duplicate {field} for attempt {attempt}")
            target[attempt][key] = row
    transactions = read_rows(directory, "transactions.csv")
    if len(transactions) != len(attempts):
        raise ValueError("transaction rows do not match attempts")
    old_candidates = read_rows(directory, "candidates.csv")
    if len(old_candidates) != len(candidates):
        raise ValueError("candidate attachment missing or mismatched")
    old_by_attempt = defaultdict(dict)
    # Both files are emitted in the same candidate order by writeIntegrity.
    # Attempt IDs are primary: transaction/window IDs can restart on reinit.
    for diagnostic, row in zip(candidates, old_candidates):
        if (row["action_id"] != diagnostic["action_id"] or
                row["window_id"] != diagnostic["window_id"] or
                row["timestamp_ns"] != diagnostic["output_timestamp_ns"]):
            raise ValueError("legacy candidate row does not match diagnostic attempt")
        attempt = int(diagnostic["input_attempt_id"])
        if row["action_id"] in old_by_attempt[attempt]:
            raise ValueError("duplicate legacy candidate in attempt")
        old_by_attempt[attempt][row["action_id"]] = row
    legacy_core = ([r for r in read_rows(directory, "timing.csv") if r["stage"] == "core_total"]
                   if require_legacy_timing else [None] * len(attempts))
    # Legacy CSV meanings stay unchanged. The attachment provides the input key;
    # the core records must match its exact order, state time, epoch and duration.
    if len(legacy_core) != len(attempts):
        raise ValueError("missing/duplicate legacy core_total")
    if require_legacy_timing and any(row["schema_version"].endswith(("/v2", "/v3", "/v4")) for row in attempts):
        legacy_rows = read_rows(directory, "timing.csv")
        links = read_rows(directory, "diagnostic_timing_links.csv")
        if [int(r["timing_row"]) for r in links] != list(range(1, len(legacy_rows) + 1)):
            raise ValueError("missing/duplicate timing row links")
        linked_stages = set()
        for link, legacy_row in zip(links, legacy_rows):
            attempt = int(link["input_attempt_id"])
            parent = indexed.get(attempt)
            if parent is None or any(link[k] != parent[k] for k in ("transaction_id", "window_id")):
                raise ValueError("timing link attempt/transaction/window mismatch")
            key = (attempt, link["stage"])
            if key in linked_stages or link["stage"] != legacy_row["stage"]:
                raise ValueError("duplicate or mismatched linked stage")
            linked_stages.add(key)
            finite(legacy_row["wall_ms"])
            if legacy_row["success"] not in ("0", "1"):
                raise ValueError("invalid legacy success flag")
            # Legacy success retains its model-gate meaning. Execution versus
            # exception is authoritative in the independent diagnostic status.
            diagnostic = by_stage[attempt].get(link["stage"])
            if diagnostic is not None and (diagnostic["status"] != "EXECUTED" or
                    not math.isclose(finite(diagnostic["wall_ms"]), finite(legacy_row["wall_ms"]), rel_tol=1e-5, abs_tol=1e-5)):
                raise ValueError("linked timing stage/duration mismatch")
        for attempt, items in by_stage.items():
            for stage, row in items.items():
                if row["status"] == "EXECUTED" and (attempt, stage) not in linked_stages:
                    raise ValueError("executed diagnostic stage lacks legacy timing link")
    for attempt, row, tx, legacy in zip(ids, attempts, transactions, legacy_core):
        if row["status"] != "EXECUTED":
            raise ValueError(f"attempt {attempt}: {row['status']}")
        finite(row["pending_duration_s"])
        for key, old in (("transaction_id", "transaction_id"), ("window_id", "window_id"),
                         ("output_timestamp_ns", "timestamp_ns"), ("graph_version", "base_graph_version"),
                         ("linpoint_version", "linearization_version")):
            if row[key] != tx[old]:
                raise ValueError("transaction attachment mismatch")
        if not STAGES.issubset(by_stage[attempt]) or set(by_stage[attempt]) - STAGES - {"commit", "discard", "state_audit", "shared_cache"}:
            raise ValueError(f"attempt {attempt}: incomplete stage set")
        for stage in by_stage[attempt].values():
            if stage["status"] == "SKIPPED":
                if stage["wall_ms"] or not stage["reason"]:
                    raise ValueError("skipped stage must have reason and no fabricated duration")
            elif stage["status"] == "EXECUTED":
                finite(stage["wall_ms"])
            else:
                raise ValueError("stage exception or invalid status")
        core = by_stage[attempt]["core_total"]
        if core["status"] != "EXECUTED":
            raise ValueError("core_total was not executed")
        if require_legacy_timing and (legacy["timestamp_ns"] != row["output_timestamp_ns"] or
                legacy["epoch"] != row["backend_epoch_after"] or
                legacy["cold_warm"] != ("cold" if attempt <= warmup else "warm") or
                legacy["success"] != "1" or
                not math.isclose(finite(legacy["wall_ms"]), finite(core["wall_ms"]), rel_tol=1e-5, abs_tol=1e-5)):
            raise ValueError("legacy timing does not match attempt diagnostics")
        current = by_candidate[attempt]
        old = old_by_attempt.get(attempt, {})
        if set(current) != set(old):
            raise ValueError("candidate attachment missing or mismatched")
        for key, candidate in current.items():
            eligibility_skip = (row["schema_version"].endswith(("/v8", "/v9", "/v10", "/v11", "/v12", "/v13")) and
                                candidate["kernel_evaluated"] == "0" and
                                candidate["skip_reason"].startswith("SKIPPED_INELIGIBLE:"))
            if candidate["kernel_evaluated"] != "1" and not eligibility_skip:
                raise ValueError("candidate kernel was not executed")
            if eligibility_skip:
                if any(candidate[field] for field in
                       ("kernel_ms", "post_ms", "bridge_ms", "fault_map_ms", "pl_ms")):
                    raise ValueError("ineligible candidate has fabricated duration")
                if candidate["numerical_valid"] != "0" or candidate["pl_evaluated"] != "0":
                    raise ValueError("ineligible candidate has fabricated numerical result")
            else:
                for field in ("kernel_ms", "post_ms"):
                    finite(candidate[field])
            for field in ("bridge_ms", "fault_map_ms", "pl_ms"):
                if candidate["pl_evaluated"] == "1":
                    finite(candidate[field])
                elif candidate[field] or not candidate["skip_reason"]:
                    raise ValueError("uncomputed candidate stage lacks skip reason")
            if row["schema_version"].endswith(
                    ("/v3", "/v4", "/v5", "/v6", "/v7", "/v8", "/v9", "/v10", "/v11", "/v12", "/v13")):
                frozen = set(filter(None, row["frozen_group_ids"].split(";")))
                removed = set(filter(None, old[key]["removed_group_ids"].split(";")))
                if not removed.issubset(frozen):
                    raise ValueError("candidate removal is outside actual frozen window")
            finite(old[key]["evaluation_wall_ms"])
            if old[key]["timestamp_ns"] != row["output_timestamp_ns"]:
                raise ValueError("candidate output time mismatch")
        if expected_candidates is not None and attempt > warmup and len(current) != expected_candidates:
            raise ValueError(f"attempt {attempt}: expected {expected_candidates} kernels, got {len(current)}")
    if sum(len(items) for items in by_candidate.values()) != len(old_candidates):
        raise ValueError("orphan legacy candidates")
    return attempts, by_stage, by_candidate
