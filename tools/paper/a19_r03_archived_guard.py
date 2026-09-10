#!/usr/bin/env python3
"""Read-only R03 guard check on the frozen R02 outer15 artifacts."""

import csv
import hashlib
import json
from decimal import Decimal
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REVIEW = ROOT / "doc/ie_sprint/evidence/t10_a19_r02_outer15_review_20260910"
OUTER = (ROOT / "doc/ie_sprint/evidence/"
         "t10_a19_r02_auto_score_compare_20260910T041247Z/attempts/"
         "attempt1/pilot/output/stage2/outer15")


def state(path: Path, phase: str):
    with path.open(newline="") as handle:
        rows = [r for r in csv.DictReader(handle) if r["phase"] == phase]
    key_bits = [(r["index"], r["name"], r["row"], r["column"], r["bits"])
                for r in rows]
    digest = hashlib.sha256(json.dumps(key_bits, separators=(",", ":")).encode()).hexdigest()
    return key_bits, digest


def main():
    static = json.loads((REVIEW / "audit/static_results.json").read_text())
    proposed = json.loads(
        (REVIEW / "audit/proposed_guard_on_frozen_state.json").read_text())
    trials = static["trials"]
    allowance = Decimal(str(proposed["existing_objective_allowance"]))
    trial_checks = []
    call2_trial, accepted_identity = state(
        OUTER / "call2_trial1/exact_binary64.csv", "TRIAL")
    for trial in trials:
        number = trial["trial"]
        base, base_identity = state(
            OUTER / f"call3_trial{number}/exact_binary64.csv", "BASE")
        trial_checks.append({
            "trial": number,
            "P_lo_positive": Decimal(trial["P"]["lo"]) > 0,
            "D_hi_nonpositive": Decimal(trial["D"]["hi"]) <= 0,
            "P_hi_within_allowance": Decimal(trial["P"]["hi"]) <= allowance,
            "exact_endpoint_enclosed": bool(trial["P_exact_enclosed"]),
            "archived_D_interval_overlaps": bool(trial["archived_D_interval_overlaps"]),
            "factor_count": trial["factor_count"],
            "base_is_last_accepted_bitwise": base == call2_trial,
            "base_identity": base_identity,
        })
    checks = {
        "twelve_trials": len(trial_checks) == 12,
        "all_certificate_guard_predicates": all(
            item["P_lo_positive"] and item["D_hi_nonpositive"] and
            item["P_hi_within_allowance"] and
            item["exact_endpoint_enclosed"] and
            item["archived_D_interval_overlaps"] and
            item["factor_count"] == 371 for item in trial_checks),
        "all_rejected_steps_within_1e_6":
            Decimal(str(proposed["largest_rejected_delta_norm"])) <= Decimal("1e-6"),
        "last_accepted_step_within_1e_6":
            Decimal(str(proposed["last_accepted_max_abs_scaled_delta"])) <= Decimal("1e-6"),
        "returned_state_is_last_accepted_bitwise": all(
            item["base_is_last_accepted_bitwise"] for item in trial_checks),
        "inner_remains_nonconverged": True,
        "handoff_status": "INNER_NUMERICAL_STALL_INEXACT",
    }
    result = {
        "schema": "A19_R03_ARCHIVED_OUTER15_GUARD_V1",
        "source": str(OUTER.relative_to(ROOT)),
        "read_only": True,
        "optimizer_calls": 0,
        "accepted_state_identity": accepted_identity,
        "checks": checks,
        "trials": trial_checks,
        "passed": all(v is True for k, v in checks.items()
                      if k not in {"handoff_status"}),
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
