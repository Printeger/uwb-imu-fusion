#!/usr/bin/env python3
"""Validate the bounded convergence run; preserve all previous evidence."""
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "results/fde_convergence_20261010"
REPORT = ROOT / "docs/benchmark/fde_operability_20261010.json"


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def source(path):
    return {"path": str(path.relative_to(ROOT)),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def numbers(row):
    result = {}
    for key, value in row.items():
        try:
            number = float(value)
            result[key] = number if math.isfinite(number) else None
        except ValueError:
            result[key] = value
    return result


report = json.loads(REPORT.read_text())
result = {
    "status": "IMPLEMENTABLE_WORK_COMPLETE; scoped failures retained",
    "production_contract_changed": False,
    "formal_eligible": False, "publication_protected": False,
    "config": source(ROOT / "config/fde_imu_order1.yaml"),
    "flows": {},
}
for name in ("uwb", "imu"):
    path = RUN / (name + ".csv")
    flow = rows(path)
    assert len(flow) == 7
    assert all(r["commit"] == r["backend_updates"] == "1" for r in flow)
    assert all(r["formal_eligible"] == r["publication_protected"] == "0" for r in flow)
    fault = flow[2]
    if name == "uwb":
        assert fault["alarm"] == fault["exclusion"] == fault["uwb_exclusion_correct"] == "1"
        assert fault["retained_current_ranges"] == "7"
        assert all(r["selected"] == r["conditional_available"] == "1" for r in flow)
        assert fault["strict_risk_closes"] == "0"
        assert all(float(r["position_error"]) <= math.hypot(float(r["hpl"]), float(r["vpl"])) + 1e-12 for r in flow)
    else:
        assert all(r["alarm"] == r["exclusion"] == "0" for r in flow)
        assert all(r["selected"] == r["conditional_available"] == "0" for r in flow[2:])
        assert "NOT_RUN" in (RUN / "imu.log").read_text()
    errors = [float(r["position_error"]) for r in flow]
    result["flows"][name] = {
        "status": "FUNCTIONAL / CONDITIONAL" if name == "uwb" else "BLOCKED_RECOVERY / FUNCTIONAL_REFUSAL",
        "source": source(path), "log": source(RUN / (name + ".log")),
        "fault_row": numbers(fault),
        "epoch_outputs": [{k: numbers(r)[k] for k in ("epoch", "alarm", "selected", "exclusion", "conditional_available", "position_error", "core_ms")} for r in flow],
        "position_rmse_m": math.sqrt(sum(e*e for e in errors) / len(errors)),
        "position_max_m": max(errors),
        "selected_commits": sum(r["selected"] == "1" for r in flow),
        "subsequent_selected_commits": sum(r["selected"] == "1" for r in flow[3:]),
        "unavailable_epochs": [int(r["epoch"]) for r in flow if r["conditional_available"] == "0"],
        "median_core_ms": statistics.median(float(r["core_ms"]) for r in flow),
        "max_core_ms": max(float(r["core_ms"]) for r in flow),
        "protected_latency_ms": None,
        "protected_latency_status": "RIGHT_CENSORED",
    }

directed = rows(RUN / "directions.csv")
assert len(directed) == 24
assert len({(r["case"], r["axis"], r["duration_s"]) for r in directed}) == 24
assert max(float(r["finite_signal_relative_error"]) for r in directed) < 1e-3
assert max(float(r["orthogonality"]) for r in directed) < 1e-10
old = rows(ROOT / "results/fde_research_20261010/power.csv")
for r in directed:
    assert int(r["raw_blocks"]) == (2 if float(r["duration_s"]) < .1 else 10)
    if r["case"] == "stationary" and r["axis"] in ("0", "5"):
        reference = next(o for o in old if o["case"] == "stationary" and o["epoch"] == "12" and o["axis"] == r["axis"] and o["duration_s"] == r["duration_s"])
        assert math.isclose(float(r["lambda"]), float(reference["lambda_at_amplitude"]), rel_tol=1e-10, abs_tol=1e-20)
result["imu_directions"] = {
    "status": "CONDITIONAL_FROZEN_MODEL; actual reintegration, no isolation claim",
    "snapshots": 2, "source": source(RUN / "directions.csv"),
    "rows": [numbers(r) for r in directed],
    "linear_amplitude_caveat": "Required amplitude is a diagnostic extrapolation, not a new tested injection; step cap is ALL-IN only, not an exclusion impossibility proof.",
    "noise_caveat": "nc-chi-square requires independent whitened Gaussian errors; shared raw noise and hardware probabilities remain unqualified.",
}
result["reused_performance"] = {
    "report_section": "research_continuation.performance",
    "scope": "original frozen paired evidence; unchanged raw-support algorithm",
}
result["new_35_attempt_pairing"] = "NOT_NEEDED: no change to measured raw support construction or production numerical algorithms; original frozen paired evidence retained"
assert "checks=40" in (RUN / "contract_checks.log").read_text()
assert "[  PASSED  ] 9 tests." in (RUN / "core_safety.log").read_text()
result["verification"] = {
    "directed_rows": 24, "contract_checks": 40, "core_safety_gtests": 9,
    "core_log": source(RUN / "core_safety.log"),
    "direction_wall_time": (RUN / "directions.time").read_text().strip(),
}
report["functional_convergence"] = result
REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
print("VERIFIED UWB correct exclusion + four followups; IMU five refused epochs; 24 directions, 40 checks, 9 safety tests; original contracts retained")
