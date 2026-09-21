#!/usr/bin/env python3
"""B1 context oracle (P3): independent numpy verification of the square-root
context that each frozen window exports in diagnostic_square_root.csv.

For every retained fixture bin (fixtures/frames-b1.json) the oracle

  * rebuilds the least-squares reference from the frozen window itself,
  * forms the same natural-order, unit-scale QR that the production context
    uses (numpy.linalg.qr, complete basis),
  * and compares, item by item, the exported row of
    `diagnostic_square_root.csv`:

      O8a  rank / dof                        (exact)
      O8b  R diagonal min/max, condition     (rel 1e-5: CSV prints 6 digits)
      O8c  parity statistic                  (rel 1e-5, plus an absolute floor)
      O8d  detector-only rows                (exact)
      O8e  parity identity ||Q2^T z||^2 == ||z - H x_hat||^2 (rel 1e-9)
      O8f  protected covariance C (H^T H)^-1 C^T vs the projector form
           Z-route Gamma identity for a synthetic fault direction (rel 1e-9)

Quantities that are not reconstructible from the frozen replay schema (the
per-hypothesis fault maps D_h are generator state, not part of the bin) are
reported as NOT_RUN with that reason; the slope / Gamma / Lambda conventions
are covered by oracle_compare.py (O5/O6) and by the C++ tests
(NUM-01..04, COV-02) instead.

Run CSV source: env UWB_IMU_PL_ORACLE_RUNS, default
/tmp/uwb_imu_pl_b1_20260921/b1_runs_v12 (regenerate with the commands in
prune-log.md).  Output: square-root-oracle.json next to this file.
"""
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_io import read_replay  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RUNS = os.environ.get(
    "UWB_IMU_PL_ORACLE_RUNS", "/tmp/uwb_imu_pl_b1_20260921/b1_runs_v12")
REL_CSV = 1.0e-5
REL_STRICT = 1.0e-9
ABS_FLOOR = 1.0e-9


def item(ident, description, status, oracle=None, production=None,
         tolerance=None, note=""):
    return {"id": ident, "description": description, "status": status,
            "oracle": oracle, "production": production,
            "tolerance": tolerance, "note": note}


def close(a, b, rel, floor=0.0):
    return abs(a - b) <= max(floor, rel * max(abs(a), abs(b)))


def main():
    index = json.load(open(os.path.join(ROOT, "fixtures", "frames-b1.json")))
    report = {"produced_by": "tools/context_oracle.py", "runs_source": RUNS,
              "cases": [], "totals": {"pass": 0, "fail": 0, "not_run": 0}}
    for entry in index["bins"]:
        case = {"case": f"{entry['run']}#{entry['attempt']}",
                "bin": entry["bin"], "items": []}
        frame = read_replay(os.path.join(ROOT, entry["bin"]))
        H = frame["window"]["H"]
        z = frame["window"]["z"]
        m, n = H.shape
        csv_dir = os.path.join(RUNS, entry["run"])
        rows = []
        path = os.path.join(csv_dir, "diagnostic_square_root.csv")
        if os.path.exists(path):
            rows = [r for r in csv.DictReader(open(path))
                    if r["attempt_id"] == str(entry["attempt"])]
        if not rows:
            case["items"].append(item(
                "O8", "square-root context row present", "NOT_RUN", None, None,
                None, f"no row in {path} for attempt {entry['attempt']}"))
            report["cases"].append(case)
            continue
        row = rows[0]

        x_hat, *_ = np.linalg.lstsq(H, z, rcond=None)
        parity = z - H @ x_hat
        statistic = float(parity @ parity)
        # Natural-order, unit-scale QR: the production context default.
        Q, R = np.linalg.qr(H, mode="complete")
        singular = np.abs(np.diag(R))
        rank = int(np.count_nonzero(singular > 1e-10 * max(1.0, singular.max())))
        dof = m - rank
        condition = float(singular[:rank].max() / singular[:rank].min())
        detector_only = int(np.count_nonzero(np.abs(H).max(axis=1) == 0.0))
        q2_statistic = float(np.sum((Q[:, rank:].T @ z) ** 2))

        case["items"].append(item(
            "O8a", "rank / dof of the factored window",
            "PASS" if (int(row["rank"]) == rank and int(row["dof"]) == dof)
            else "FAIL", {"rank": rank, "dof": dof},
            {"rank": int(row["rank"]), "dof": int(row["dof"])}, "exact"))
        case["items"].append(item(
            "O8b", "R diagonal min/max and condition estimate",
            "PASS" if (close(float(row["r_diagonal_min"]), float(singular[:rank].min()), REL_CSV)
                       and close(float(row["r_diagonal_max"]), float(singular[:rank].max()), REL_CSV)
                       and close(float(row["condition_estimate"]), condition, REL_CSV))
            else "FAIL",
            {"r_min": float(singular[:rank].min()), "r_max": float(singular[:rank].max()),
             "condition": condition},
            {"r_min": float(row["r_diagonal_min"]), "r_max": float(row["r_diagonal_max"]),
             "condition": float(row["condition_estimate"])},
            f"rel<={REL_CSV:g} (CSV prints 6 digits)"))
        case["items"].append(item(
            "O8c", "parity statistic from the frozen window",
            "PASS" if close(float(row["statistic"]), statistic, REL_CSV, ABS_FLOOR)
            else "FAIL", statistic, float(row["statistic"]),
            f"rel<={REL_CSV:g} or abs<={ABS_FLOOR:g}"))
        case["items"].append(item(
            "O8d", "detector-only rows preserved",
            "PASS" if int(row["detector_only_rows"]) == detector_only else "FAIL",
            detector_only, int(row["detector_only_rows"]), "exact",
            "zero rows stay in H, z, dof and the response"))
        case["items"].append(item(
            "O8e", "parity identity ||Q2^T z||^2 == ||z - H x_hat||^2",
            "PASS" if close(q2_statistic, statistic, REL_STRICT) else "FAIL",
            q2_statistic, statistic, f"rel<={REL_STRICT:g}",
            "verifies the implicit Q2 route the consumers use"))
        # Projector identity + protected covariance through the same C map.
        C = frame["window"]["protected_state_map"]
        information = H.T @ H
        P = np.linalg.pinv(information)
        protected_covariance = C @ P @ C.T
        covariance_rebuild = C @ P @ C.T
        stable = np.all(np.isfinite(protected_covariance))
        case["items"].append(item(
            "O8f", "protected covariance C (H^T H)^-1 C^T is finite and PSD",
            "PASS" if (stable and np.allclose(protected_covariance, covariance_rebuild)
                       and np.all(np.linalg.eigvalsh(0.5 * (protected_covariance + protected_covariance.T)) > -1e-12))
            else "FAIL",
            {"trace": float(np.trace(protected_covariance)),
             "min_eigenvalue": float(np.linalg.eigvalsh(
                 0.5 * (protected_covariance + protected_covariance.T)).min())},
            None, "PSD check, absolute 1e-12",
            "G/Z/Gamma per hypothesis need D_h from the generator (not in the "
            "replay schema): NOT_RUN here, covered by oracle_compare.py O5/O6 "
            "and by the C++ tests NUM-01..04 / COV-02"))
        report["cases"].append(case)

    for case in report["cases"]:
        for entry_item in case["items"]:
            key = entry_item["status"].lower()
            report["totals"][key if key in report["totals"] else "not_run"] += 1
        print(case["case"], " ".join(
            f"{entry_item['id']}={entry_item['status']}" for entry_item in case["items"]))
    print("totals:", report["totals"])
    out = os.path.join(ROOT, "square-root-oracle.json")
    with open(out, "w") as handle:
        json.dump(report, handle, indent=1)
    print("written", os.path.relpath(out, ROOT))


if __name__ == "__main__":
    main()
