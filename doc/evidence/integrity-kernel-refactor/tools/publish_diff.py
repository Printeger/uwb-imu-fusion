#!/usr/bin/env python3
"""W2 publication diff: compare PUBLISHED quantities between two run dirs.

Usage: publish_diff.py <baseline_dir> <candidate_dir>

Only published quantities are compared (integrity.csv / transactions.csv /
candidates.csv / hypotheses.csv on their COMMON column names, plus the
diagnostic coverage/decision counters); the new v16 diagnostic columns are
reported separately.  A non-empty diff on a published column is a finding, so
"published action/state/PL changes" can be listed line by line.

Evidence for W2 (2026-09-22): base = the b7feb9c scenario batch,
candidate = the wip(W2) scenario batch (see runbook.md section 21).
"""
import csv
import sys
from pathlib import Path

PUBLISHED_INTEGRITY = [
    "pl_x", "pl_y", "pl_z", "hpl_m", "vpl_m", "availability", "label",
    "formal_eligible", "risk_budget_valid", "allocated_hmi_risk",
    "hmi_risk_requirement", "batch_committed", "transaction_id", "window_id",
    "base_graph_version", "linearization_version", "selected_action_id",
    "selected_action_type", "fde_status", "bridge_pl_x", "bridge_pl_y",
    "bridge_pl_z", "history_provenance_valid", "backend_updates", "stale_state",
    "controlled_reinitialization_required", "historical_groups_removed",
    "historical_groups_added", "recovery_epoch_begin", "recovery_epoch_end",
    "reinitialization_request_id", "reinitialization_phase",
    "reinitialization_reason", "reason",
]
PUBLISHED_TRANSACTIONS = [
    "transaction_id", "window_id", "base_graph_version", "linearization_version",
    "selected_action_id", "fde_status", "backend_updates", "stale_state",
    "reinitialization_required",
]
PUBLISHED_CANDIDATES = [
    "action_id", "action_type", "cardinality", "valid", "post_detector_passed",
    "covers_plausible_set", "statistic", "threshold", "rank", "dof",
    "condition_number", "information_logdet", "hpl_m", "vpl_m", "selected",
    "reason",
]
PUBLISHED_HYPOTHESES = [
    "hypothesis_id", "fault_unit_ids", "sensor", "fault_kind", "mode_ids",
    "onset_epoch", "onset_time_ns", "parameter_dimension", "fault_rank",
    "sigma_min", "sigma_max", "condition_number", "boundary_direction_gram",
    "noncentrality_boundary", "prior_bound", "p_md_allocation", "hmi_allocation",
    "monitorable", "plausible", "conditioned_statistic", "log_evidence",
    "z_rank", "z_classification", "coverage_label", "coverage_envelope_id",
    "reason",
]
# Diagnostic coverage / decision columns that must not shrink.
COVERAGE_ATTEMPT = [
    "coverage_status", "coverage_exact_leaves", "coverage_enveloped_leaves",
    "coverage_uncovered_leaves", "coverage_envelope_count",
    "coverage_accepted_envelope_count", "coverage_proof_count",
    "kernel_evaluated_actions", "post_passed_actions", "pl_evaluated_actions",
    "selected_actions", "generated_actions", "hypothesis_count",
    "risk_ledger_closes", "risk_ledger_charged_total", "risk_ledger_declared_total",
    "primary_failure", "all_failures", "not_evaluated_checks",
]


def read(path, limit=None):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return rows[:limit] if limit else rows


def compare(name, base_rows, new_rows, columns, findings, limit=5):
    if len(base_rows) != len(new_rows):
        findings.append(f"{name}: row count {len(base_rows)} -> {len(new_rows)}")
        return
    differing = {}
    for index, (base, new) in enumerate(zip(base_rows, new_rows), 2):
        for column in columns:
            if column not in base or column not in new:
                continue
            if base[column] != new[column]:
                key = (column, base[column], new[column])
                differing[key] = differing.get(key, (index,)) + (index,)
    for (column, old, new), lines in sorted(differing.items()):
        shown = ",".join(str(x) for x in lines[1:limit + 1])
        findings.append(
            f"{name}:{column}: {old!r} -> {new!r} (rows {lines[0]},"
            f"+{len(lines) - 1} more{':' if len(lines) > 1 else ''}{shown})")


def main():
    base_dir, new_dir = Path(sys.argv[1]), Path(sys.argv[2])
    # Optional row cap: used for the H_mature_union prefix comparison, where the
    # full 226-epoch run is the expensive side.
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else None
    findings = []
    for name, columns in (("integrity.csv", PUBLISHED_INTEGRITY),
                          ("transactions.csv", PUBLISHED_TRANSACTIONS),
                          ("candidates.csv", PUBLISHED_CANDIDATES),
                          ("hypotheses.csv", PUBLISHED_HYPOTHESES)):
        base_path, new_path = base_dir / name, new_dir / name
        base_rows = read(base_path, limit) if base_path.is_file() else []
        new_rows = read(new_path, limit) if new_path.is_file() else []
        compare(name, base_rows, new_rows, columns, findings)
    base_attempts = read(base_dir / "diagnostic_attempts.csv", limit)
    new_attempts = read(new_dir / "diagnostic_attempts.csv", limit)
    compare("diagnostic_attempts.csv", base_attempts, new_attempts,
            COVERAGE_ATTEMPT, findings)
    new_columns = [c for c in new_attempts[0] if c.startswith(
        ("publication_", "watchdog_", "selection_", "transaction_opened"))]
    print(f"new v16 columns ({len(new_columns)}): {','.join(new_columns)}")
    refused = sum(1 for r in new_attempts if r.get("status") == "WATCHDOG_REFUSED")
    protected = sum(1 for r in new_attempts
                    if r.get("publication_protected") == "1")
    checks = {}
    for row in new_attempts:
        name = row.get("publication_identity_check", "?")
        checks[name] = checks.get(name, 0) + 1
    print(f"refused={refused} protected={protected} identity_checks={checks}")
    if findings:
        print("FINDINGS:")
        for line in findings:
            print("  " + line)
        sys.exit(1)
    print("OK: no published quantity changed")


if __name__ == "__main__":
    main()
