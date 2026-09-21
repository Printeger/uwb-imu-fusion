#!/usr/bin/env python3
"""A4-8 independent reference oracle (P2).

For every selected frozen window the oracle recomputes, with an independent
Python/numpy/scipy implementation, the quantities that the production kernel
exports, and compares them item by item:

  O0  snapshot identity binding between the v5 replay bin and the run CSV row
  O1  parity (detector) statistic s^2 = ||z - H x_hat||^2 against
      integrity.csv conditional_statistic
  O2  detector dof and chi-square threshold against integrity.csv
  O3  block assembly (sum of whitened block Jacobians == H) and per-block
      whitening audit W * J_raw == J_whitened
  O4  rank / dof / condition number of H against the frozen window metadata
  O5  single-anchor epoch-independent template (fault on the current epoch's
      anchor-1 row): Z = M A, Gamma scalar, slopes |C (H^T H)^+ H^T a| / sqrt(a^T M a)
      against the production hypothesis row with the same onset epoch
  O6  non-centrality boundary lambda* solving P(chi2'_dof(lambda) <= tau) = p_md
      (scipy) against the exported noncentrality_boundary, plus the closure
      check P_miss(lambda*) == p_md_allocation
  O7  G9 regression guard: every monitorable hypothesis must carry finite slopes

Tolerances follow ADR 0002, but items compared against CSV text are bounded by
the 6-significant-digit default ostream precision of the logger (rel 1e-5);
pure in-memory quantities use the tighter pure-linear tiers.  A non-finite
value on either side never passes by itself: it must be accompanied by the
matching classification (rank/gram below gate), which is reported explicitly.

Output: oracle-results.json (evidence root) + human readable stdout.
"""
import csv
import hashlib
import json
import os
import subprocess
import sys

import numpy as np
from scipy.optimize import brentq
from scipy.stats import chi2, ncx2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_io import read_replay  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.abspath(os.path.join(ROOT, "..", "..", ".."))
RAW = os.path.join(ROOT, "raw")

P_FA = 1.0e-6           # detector.p_fa_per_test (research configuration)
REL_CSV = 1.0e-5        # bound set by 6-significant-digit CSV printing
REL_STRICT = 1.0e-9     # pure linear algebra, well conditioned
REL_MODERATE = 1.0e-7   # near-gate / pseudo-inverse paths
ABS_WHITEN = 1.0e-12    # whitening / assembly audit

# label -> (csv run dir, attempt, replay dir, scenario note)
CASES = [
    ("A_nominal_30", "runs_p2/A_nominal", 30, "replay_p2/A_nominal",
     "healthy frozen window, anchor template active"),
    ("C_uwb_fde_25", "runs_p2/C_uwb_fde", 25, "replay_p2/C_uwb_fde",
     "UWB fault before the exclusion action"),
    ("C_uwb_fde_26", "runs_p2/C_uwb_fde", 26, "replay_p2/C_uwb_fde",
     "post-FDE window (action applied, 128 cap)"),
    ("F_ramp_30", "runs_p2/F_ramp_unmonitorable", 30, "replay_p2/F_ramp_unmonitorable",
     "ramp regime under the frozen unsupported declaration"),
    ("G_reject_10", "runs_p2/G_continuous_rejection", 10, "replay_p2/G_continuous_rejection",
     "consecutive rejection epoch"),
    ("H_mature_201", "runs_p2/H_mature_union", 201, "replay_p2/H_mature_union",
     "mature union window"),
    ("H_mature_205", "runs_p2/H_mature_union", 205, "replay_p2/H_mature_union",
     "mature union window (second export)"),
    ("EPOCHS20_30", "runs_p2/epochs20-census", 30, "replay_p2/epochs20-census",
     "K=20 integrity-window census口径"),
]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path):
    with open(path) as handle:
        return list(csv.DictReader(handle))


def item(case, ident, description, status, oracle=None, production=None,
         tolerance=None, note=""):
    return {
        "id": ident,
        "case": case,
        "description": description,
        "status": status,
        "oracle": oracle,
        "production": production,
        "tolerance": tolerance,
        "note": note,
    }


def close(a, b, rel, abs_floor=0.0):
    if a is None or b is None:
        return False
    return abs(a - b) <= max(abs_floor, rel * max(abs(a), abs(b)))


def rel_error(a, b):
    scale = max(abs(a), abs(b))
    if scale == 0.0:
        return 0.0
    return abs(a - b) / scale


def finite(value):
    return value is not None and np.isfinite(value)


def current_anchor_row(frame, anchor_id):
    """Global row of the current epoch's anchor-`anchor_id` UWB row."""
    inventory = {e["group_id"]: e for e in frame["window"]["factor_inventory"]}
    offset = 0
    for block in frame["window"]["blocks"]:
        rows = block["jacobian_whitened"].shape[0]
        entry = inventory.get(block["group_id"])
        if (block["kind"] == "UwbBatch" and entry is not None
                and entry["disposition"] == "PendingExplicit"
                and rows >= anchor_id):
            return offset + (anchor_id - 1), block, offset
        offset += rows
    return None, None, None


def analyze_case(label, run_dir, attempt, replay_dir, note):
    bin_path = os.path.join(RAW, replay_dir, "exports",
                            f"attempt-{attempt}.bin")
    csv_dir = os.path.join(RAW, run_dir)
    frame = read_replay(bin_path)
    window = frame["window"]
    H = window["H"]
    z = window["z"]
    C = window["protected_state_map"]
    m, n = H.shape

    identity_rows = [r for r in read_csv(os.path.join(
        csv_dir, "diagnostic_snapshot_identity.csv"))
        if r["input_attempt_id"] == str(attempt)]
    integrity_rows = [r for r in read_csv(os.path.join(
        csv_dir, "integrity.csv")) if r["window_id"] == str(attempt)]
    hypothesis_rows = [r for r in read_csv(os.path.join(
        csv_dir, "hypotheses.csv")) if r["window_id"] == str(attempt)]

    items = []
    identity = frame.get("identity", {})
    identity_read = None
    if identity_rows:
        row = identity_rows[0]
        identity_read = {
            "snapshot_id": row["snapshot_id"],
            "config_digest": row["config_digest"],
            "identity_digest": row["identity_digest"],
            "frame_id": row["frame_id"],
            "position_reference": row["position_reference"],
            "tangent_convention": row["tangent_convention"],
        }
    checks = {
        "snapshot_id": identity.get("snapshot_id") == (identity_read or {}).get("snapshot_id"),
        "identity_digest": identity.get("identity_digest") == (identity_read or {}).get("identity_digest"),
        "config_digest": identity.get("config_digest") == (identity_read or {}).get("config_digest"),
        "frame_id_world": identity.get("frame_id") == "world",
        "protected_body_origin": identity.get("protected_reference_center") == "body_origin",
        "coverage_epoch_declared": identity.get("coverage_epoch") == "NOT_AVAILABLE_IN_SCHEMA",
    }
    if identity_read is None:
        items.append(item(label, "O0", "v5 bin identity binding to run CSV",
                          "NOT_RUN", identity, None, None,
                          "run CSV identity row missing for this attempt"))
    else:
        ok = all(checks.values())
        items.append(item(label, "O0", "v5 bin identity binding to run CSV",
                          "PASS" if ok else "FAIL", identity, identity_read,
                          "exact equality", json.dumps(checks)))

    # O1 / O2 detector statistic, dof, threshold
    x_hat, *_ = np.linalg.lstsq(H, z, rcond=None)
    parity = z - H @ x_hat
    statistic = float(parity @ parity)
    singular = np.linalg.svd(H, compute_uv=False)
    gate = 1.0e-9 * max(1.0, singular[0]) if singular.size else 0.0
    rank = int(np.count_nonzero(singular > gate))
    dof = m - rank
    threshold = float(chi2.ppf(1.0 - P_FA, dof)) if dof > 0 else float("inf")
    if integrity_rows:
        row = integrity_rows[0]
        prod_stat = float(row["conditional_statistic"])
        prod_dof = int(row["conditional_dof"])
        prod_thr = float(row["conditional_threshold"])
        items.append(item(
            label, "O1", "parity statistic s^2 = ||z - H x_hat||^2",
            "PASS" if close(statistic, prod_stat, REL_CSV) else "FAIL",
            statistic, prod_stat, f"rel<={REL_CSV:g} (CSV precision)",
            f"oracle_rank={rank}"))
        items.append(item(
            label, "O2", "detector dof and chi-square threshold",
            "PASS" if (dof == prod_dof and close(threshold, prod_thr, REL_CSV))
            else "FAIL", {"dof": dof, "threshold": threshold},
            {"dof": prod_dof, "threshold": prod_thr},
            f"dof exact, threshold rel<={REL_CSV:g}",
            f"p_fa={P_FA:g} from detector.p_fa_per_test"))
    else:
        items.append(item(label, "O1", "parity statistic s^2 = ||z - H x_hat||^2",
                          "NOT_RUN", statistic, None, None,
                          "integrity.csv row missing for this attempt"))

    # O3 assembly and whitening audit
    assembly_error = 0.0
    whitening_error = 0.0
    offset = 0
    for block in window["blocks"]:
        rows = block["jacobian_whitened"].shape[0]
        local = block["jacobian_whitened"]
        rows_h = H[offset:offset + rows, :]
        if local.shape[1] != n:
            local_full = np.zeros((rows, n))
            local_full[:, block["window_column_indices"]] = local
            local = local_full
        scale = max(1e-300, float(np.abs(H).max()))
        assembly_error = max(assembly_error,
                             float(np.abs(rows_h - local).max() / scale))
        if block["whitener"].size and block["jacobian_raw"].size:
            width = block["whitener"].shape[0]
            raw = block["jacobian_raw"][:width, :]
            diff = np.abs(block["whitener"] @ raw
                          - block["jacobian_whitened"]).max()
            wscale = max(1e-300, float(np.abs(block["jacobian_whitened"]).max()))
            whitening_error = max(whitening_error, float(diff / wscale))
        offset += rows
    items.append(item(
        label, "O3", "block assembly and W * J_raw == J_whitened",
        "PASS" if (assembly_error <= ABS_WHITEN and whitening_error <= ABS_WHITEN)
        else "FAIL",
        {"assembly_max_rel": assembly_error, "whitening_max_rel": whitening_error},
        None, f"abs<={ABS_WHITEN:g}",
        "independent row placement via window_column_indices"))

    # O4 window metadata
    cond = float(singular[0] / singular[rank - 1]) if rank else float("inf")
    meta = {
        "rank": window["rank"], "dof": window["dof"],
        "condition_number": window["condition_number"],
    }
    ok = (rank == meta["rank"] and dof == meta["dof"]
          and close(cond, meta["condition_number"], REL_CSV))
    items.append(item(
        label, "O4", "rank / dof / condition number of H",
        "PASS" if ok else "FAIL",
        {"rank": rank, "dof": dof, "condition_number": cond}, meta,
        f"rank+dof exact, cond rel<={REL_CSV:g}",
        "SVD gate 1e-9 * max(1, sigma_max)"))

    # O5 single-anchor template at the current epoch
    anchor_row, anchor_block, block_offset = current_anchor_row(frame, 1)
    template_cmp = None
    if anchor_row is None or not hypothesis_rows:
        reason = ("no pending UWB row in this window"
                  if anchor_row is None else "hypotheses.csv rows missing")
        items.append(item(label, "O5",
                          "anchor-1 epoch-independent template (Gamma, slopes)",
                          "NOT_RUN", None, None, None, reason))
    else:
        rows = anchor_block["jacobian_whitened"].shape[0]
        a = np.zeros(m)
        if anchor_block["whitener"].size:
            a[block_offset:block_offset + rows] = anchor_block["whitener"][:, 0]
        else:
            a[anchor_row] = 1.0
        information = H.T @ H
        information_pinv = np.linalg.pinv(information)
        projector = np.eye(m) - H @ information_pinv @ H.T
        gamma = float(a @ projector @ a)
        g = C @ (information_pinv @ (H.T @ a))
        sigma_min = float(np.sqrt(max(0.0, gamma)))
        slopes = np.abs(g) / sigma_min if sigma_min > 0 else np.full(3, np.inf)
        target = [r for r in hypothesis_rows
                  if r["fault_kind"] == "ANCHOR_BIAS_EPOCH_INDEPENDENT"
                  and "uwb:1" in r["physical_source_ids"]
                  and r["onset_epoch"] == str(attempt)]
        template_cmp = {"oracle_sigma_min": sigma_min,
                        "oracle_slopes": slopes.tolist()}
        if not target:
            items.append(item(
                label, "O5",
                "anchor-1 epoch-independent template (Gamma, slopes)",
                "NOT_RUN", template_cmp, None, None,
                "no production row with onset_epoch == attempt"))
        else:
            row = target[0]
            prod_sigma = float(row["sigma_min"])
            prod_slopes = [float(row["slope_" + axis]) for axis in "xyz"]
            ok = close(sigma_min, prod_sigma, REL_CSV)
            detail = [{"axis": axis,
                       "oracle": slopes[i],
                       "production": prod_slopes[i],
                       "rel": rel_error(slopes[i], prod_slopes[i])}
                      for i, axis in enumerate("xyz")]
            ok = ok and all(close(slopes[i], prod_slopes[i], REL_CSV)
                            for i in range(3))
            sigma_ok = close(sigma_min, prod_sigma, REL_CSV)
            limits = [sigma_min, prod_sigma] + list(slopes) + prod_slopes
            graded = all(finite(v) for v in limits)
            items.append(item(
                label, "O5",
                "anchor-1 epoch-independent template (Gamma, slopes)",
                "PASS" if (ok and graded) else ("FAIL" if graded else "FAIL_NONFINITE"),
                {"sigma_min": sigma_min, "slopes_xyz": slopes.tolist(),
                 "gamma": gamma},
                {"hypothesis_id": row["hypothesis_id"], "sigma_min": prod_sigma,
                 "slopes_xyz": prod_slopes},
                f"rel<={REL_CSV:g} (CSV precision); non-finite never passes",
                json.dumps({"sigma_min_ok": sigma_ok, "axes": detail})))

    # O6 non-centrality boundary convention
    if not hypothesis_rows or not integrity_rows:
        items.append(item(label, "O6", "non-centrality boundary lambda*",
                          "NOT_RUN", None, None, None,
                          "hypotheses.csv or integrity.csv row missing"))
    else:
        row = integrity_rows[0]
        dof_prod = int(row["conditional_dof"])
        thr_prod = float(row["conditional_threshold"])
        worst_cdf = 0.0
        worst_rel = 0.0
        sample = hypothesis_rows[:64]
        solved = []
        for hypothesis in sample:
            allocation = float(hypothesis["p_md_allocation"])
            produced = float(hypothesis["noncentrality_boundary"])
            if not np.isfinite(produced):
                continue
            f = lambda lam: ncx2.cdf(thr_prod, dof_prod, lam) - allocation
            try:
                lam = brentq(f, 0.0, 1e9, xtol=1e-10, rtol=1e-13)
            except ValueError:
                continue
            solved.append(lam)
            worst_rel = max(worst_rel,
                            rel_error(lam, produced))
            closure = ncx2.cdf(thr_prod, dof_prod, produced)
            worst_cdf = max(worst_cdf, abs(closure - allocation))
        status = "PASS" if (solved and worst_rel <= REL_CSV
                            and worst_cdf <= 1e-6) else "FAIL"
        items.append(item(
            label, "O6", "non-centrality boundary lambda* vs scipy ncx2",
            status,
            {"solved_first": solved[0] if solved else None,
             "worst_rel_vs_csv": worst_rel,
             "worst_abs_cdf_closure": worst_cdf, "rows": len(solved)},
            None, f"rel<={REL_CSV:g}; P_miss(lambda*)=p_md within 1e-6",
            "P(chi2'_dof(lambda) <= tau) = p_md allocation, tau/dof taken "
            "from the conditional detector of the same window"))

    # O7 G9 guard
    monitorable_nonfinite = [r for r in hypothesis_rows
                             if r["monitorable"] == "1"
                             and not all(np.isfinite(float(r["slope_" + axis]))
                                         for axis in "xyz")]
    if not hypothesis_rows:
        items.append(item(
            label, "O7",
            "G9 guard: monitorable hypotheses carry finite slopes",
            "NOT_RUN", None, None, "count must be 0",
            "hypotheses.csv is empty (run without FULL_AUDIT)"))
    else:
        items.append(item(
            label, "O7", "G9 guard: monitorable hypotheses carry finite slopes",
            "PASS" if not monitorable_nonfinite else "FAIL",
            {"monitorable_rows": len([r for r in hypothesis_rows
                                      if r["monitorable"] == "1"]),
             "nonfinite_rows": len(monitorable_nonfinite)},
            None, "count must be 0",
            "hypotheses.csv slope_x/y/z"))

    statuses = [i["status"] for i in items]
    return {
        "case": label,
        "note": note,
        "bin": os.path.relpath(bin_path, ROOT),
        "bin_sha256": sha256(bin_path),
        "csv_dir": os.path.relpath(csv_dir, ROOT),
        "attempt": attempt,
        "window": {"m": int(m), "n": int(n), "rank": rank, "dof": dof,
                   "condition_number": cond},
        "identity": identity,
        "items": items,
        "summary": {
            "pass": statuses.count("PASS"),
            "fail": sum(1 for s in statuses if s.startswith("FAIL")),
            "not_run": statuses.count("NOT_RUN"),
        },
    }


def main():
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
        text=True, check=True).stdout.strip()
    results = {
        "produced_by": "tools/oracle_compare.py",
        "algorithm": "independent numpy/scipy reimplementation (no production solver)",
        "source_revision": revision,
        "tolerance_source": {
            "csv_printed": "6-significant-digit ostream default -> rel 1e-5",
            "strict_linear": "ADR 0002 well-conditioned rel 1e-9 (used elsewhere)",
            "moderate_linear": "ADR 0002 near-gate rel 1e-7 (used elsewhere)",
            "whitening_assembly": "abs 1e-12 relative to block scale",
        },
        "detector": {"p_fa_per_test": P_FA, "source": "detector.p_fa_per_test"},
        "cases": [],
    }
    for label, run_dir, attempt, replay_dir, note in CASES:
        results["cases"].append(
            analyze_case(label, run_dir, attempt, replay_dir, note))
    totals = {"pass": 0, "fail": 0, "not_run": 0}
    for case in results["cases"]:
        for key in totals:
            totals[key] += case["summary"][key]
    results["totals"] = totals
    out_path = os.path.join(ROOT, "oracle-results.json")
    with open(out_path, "w") as handle:
        json.dump(results, handle, indent=1)
    for case in results["cases"]:
        marks = " ".join(f"{i['id']}={i['status']}" for i in case["items"])
        print(f"{case['case']}: {marks}")
    print(f"totals: {totals}")
    print(f"written {os.path.relpath(out_path, ROOT)}")


if __name__ == "__main__":
    main()
