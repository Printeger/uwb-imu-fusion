"""Offline census + fixture + ladder analysis for the integrity kernel (A2).

Reads the frozen-window replay bins exported by apps/r0_r1_development
(UWB_IMU_PL_REPLAY_EXPORT_DIR) and the diagnostic v10 CSVs, and produces:

  * census.json           - real block/rank/nnz/fill-in census of frozen windows
  * fixtures/frames.json  - the five retained frame classes with inputs,
                            first-cause/all-checks shadow diagnostics and the
                            one-dim / template / free-profile ladder

Independence: this script never calls production C++ code.  Every algebraic
quantity is recomputed here with numpy/scipy from the serialized H/z/C/blocks.

No claim is made that the ladder *proves* a root cause; it only locates which
algebraic step stops first under frozen H/C/noise/risk parameters.
"""

import csv
import hashlib
import json
import os
import sys
from collections import Counter

import numpy as np
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from replay_io import read_replay  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "raw")
RANK_TOL = 1e-10
MAX_COND = 1e10
MIN_SIGMA = 1e-8
MAX_GRAM_COND = 1e10
P_FA = 1e-6
P_MD = 1e-3
NOMINAL_TAIL = 1e-5
N_ANCHORS = 8

FIXTURES = [
    {"class": "normal", "run": "A_nominal", "attempt": 30,
     "scenario": "A_nominal", "note": "KEEP_ALL, all-in detector pass, finite PL"},
    {"class": "alarm", "run": "C_uwb_fde", "attempt": 25,
     "scenario": "C_uwb_fde", "note": "anchor-1 range bias +2.25 m, detector alarm, union exclusion committed"},
    {"class": "inf_pl", "run": "C_uwb_fde", "attempt": 26,
     "scenario": "C_uwb_fde", "note": "first epoch after the alarm: no eligible action, best-effort commit, PL never evaluated"},
    {"class": "missing", "run": "G_continuous_rejection", "attempt": 10,
     "scenario": "G_continuous_rejection", "note": "all-anchor bias; only candidate rejected by the 0.25 step gate; epoch discarded (no UWB commit)"},
    {"class": "marginalization", "run": "H_mature_union", "attempt": 201,
     "scenario": "H_mature_union", "note": "first fixed-lag marginalization epoch; KEEP_ALL committed"},
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


# --------------------------------------------------------------------------
# frozen-window census
# --------------------------------------------------------------------------

def block_table(frame):
    window = frame["window"]
    inventory = {entry["group_id"]: entry for entry in window["factor_inventory"]}
    table = []
    offset = 0
    for block in window["blocks"]:
        rows = block["jacobian_whitened"].shape[0]
        entry = inventory.get(block["group_id"])
        columns = len(block["window_column_indices"])
        touched = sorted({index // 15 for index in block["window_column_indices"]})
        table.append({
            "group_id": block["group_id"],
            "kind": block["kind"],
            "sensor": block["sensor"],
            "role": block["role"],
            "epoch": entry["epoch"] if entry else None,
            "disposition": entry["disposition"] if entry else "BoundaryPrior",
            "row_begin": offset,
            "rows": rows,
            "columns": columns,
            "nnz_jacobian": int(np.count_nonzero(block["jacobian_whitened"])),
            "whitening_model_id": block["whitening_model_id"],
            "state_blocks_touched": len(touched),
            "span_first": touched[0] if touched else None,
            "span_last": touched[-1] if touched else None,
            "whitened_eq_whitener_raw_rel": _whitening_error(block),
        })
        offset += rows
    return table


def _whitening_error(block):
    if block["whitener"].size == 0 or block["jacobian_raw"].size == 0:
        return None
    diff = np.abs(block["whitener"] @ block["jacobian_raw"]
                  - block["jacobian_whitened"]).max()
    scale = max(1e-300, np.abs(block["jacobian_whitened"]).max())
    return float(diff / scale)


def symbolic_fill(base_information):
    """Number of nonzeros introduced by an elimination (Cholesky/QR) on the
    given symmetric sparsity pattern, eliminated in index order."""
    n = base_information.shape[0]
    pattern = [set(np.nonzero(base_information[i])[0].tolist()) for i in range(n)]
    filled = 0
    for k in range(n):
        neighbours = [j for j in pattern[k] if j > k]
        for i in neighbours:
            for j in neighbours:
                if j > i and j not in pattern[i]:
                    pattern[i].add(j)
                    pattern[j].add(i)
                    filled += 1
    final = sum(len(row) for row in pattern) // 2
    return filled, final


def frame_census(frame):
    window = frame["window"]
    H = window["H"]
    z = window["z"]
    information = H.T @ H
    singular = np.linalg.svd(H, compute_uv=False)
    largest = singular[0] if singular.size else 0.0
    gate = RANK_TOL * max(1.0, largest)
    rank = int(np.count_nonzero(singular > gate))
    dof = H.shape[0] - rank
    condition = float(singular[0] / singular[rank - 1]) if rank else float("inf")
    x_hat, *_ = np.linalg.lstsq(H, z, rcond=None)
    parity = z - H @ x_hat
    threshold = float(stats.chi2.ppf(1.0 - P_FA, dof)) if dof > 0 else float("inf")
    stored_information = window["base_information"]
    table = block_table(frame)
    filled, final_nnz = symbolic_fill(information)
    return {
        "m": int(H.shape[0]),
        "n": int(H.shape[1]),
        "rank_H": rank,
        "nu": dof,
        "cond_H": condition,
        "nnz_H": int(np.count_nonzero(H)),
        "nnz_H_density": float(np.count_nonzero(H) / H.size),
        "nnz_information": int(np.count_nonzero(information)),
        "nnz_information_density": float(np.count_nonzero(information) / information.size),
        "symbolic_fill_in": int(filled),
        "symbolic_final_nnz": int(final_nnz),
        "recorded_rank": window["rank"],
        "recorded_dof": window["dof"],
        "recorded_cond": float(window["condition_number"]),
        "parity_statistic": float(parity @ parity),
        "chi2_threshold": threshold,
        "detector_pass": bool(parity @ parity <= threshold),
        "base_information_rebuild_rel_err":
            float(np.abs(stored_information - information).max()
                  / max(1e-300, np.abs(information).max())),
        "protected_state_map_nonzero_cols":
            np.nonzero(window["protected_state_map"].any(axis=0))[0].tolist(),
        "capabilities": window["capabilities"],
        "blocks": table,
        "block_kind_counts": dict(Counter(
            f"{b['kind']}/{b['disposition']}" for b in table)),
    }


# --------------------------------------------------------------------------
# ladder: one-dim instant / existing template / source free profile
# --------------------------------------------------------------------------

def anchor_row_indices(frame, anchor_id):
    """Row indices of a UWB anchor (1-based id, config order) in every
    UwbBatch block of the frozen window.  Verified empirically at the injected
    frame: the dominant parity residual lands on row `anchor_id - 1`."""
    window = frame["window"]
    inventory = {entry["group_id"]: entry for entry in window["factor_inventory"]}
    rows = []
    offset = 0
    for block in window["blocks"]:
        count = block["jacobian_whitened"].shape[0]
        if block["kind"] == "UwbBatch":
            entry = inventory.get(block["group_id"])
            rows.append({
                "group_id": block["group_id"],
                "epoch": entry["epoch"] if entry else None,
                "disposition": entry["disposition"] if entry else None,
                "row": offset + (anchor_id - 1),
            })
        offset += count
    return rows


def row_lookup(frame):
    """global row -> (block_row_offset, local_row, whitener, block_rows).  A
    unit additive fault on raw row l of a block maps into the whitened space as
    the l-th column of that block's whitener (production: whitened = W * raw)."""
    lookup = {}
    offset = 0
    for block in frame["window"]["blocks"]:
        rows = block["jacobian_whitened"].shape[0]
        for local in range(rows):
            lookup[offset + local] = (offset, local, block["whitener"], rows)
        offset += rows
    return lookup


def ladder_variant(frame, columns):
    """Pure algebra for one fault map defined on raw measurement rows.

    columns: list of parameters; each parameter is a list of (global_row, value)
    entries in raw residual units.  Returns parity-space quantities.
    """
    H = frame["window"]["H"]
    z = frame["window"]["z"]
    C = frame["window"]["protected_state_map"]
    lookup = row_lookup(frame)
    m, n = H.shape
    A = np.zeros((m, len(columns)))
    for index, column in enumerate(columns):
        for row, value in column:
            offset, local, whitener, rows = lookup[row]
            if whitener.size:
                A[offset:offset + rows, index] += value * whitener[:, local]
            else:
                A[row, index] += value
    rows_touched = sorted({row for column in columns for row, _ in column})
    information = H.T @ H
    solved = np.linalg.pinv(information) @ (H.T @ A)
    Z = A - H @ solved
    gamma = Z.T @ Z
    x_hat, *_ = np.linalg.lstsq(H, z, rcond=None)
    score = A.T @ (z - H @ x_hat)
    singular = np.linalg.svd(gamma, compute_uv=False)
    largest = singular[0] if singular.size else 0.0
    gate = RANK_TOL * max(1.0, largest)
    rank = int(np.count_nonzero(singular > gate))
    sigma_min = float(np.sqrt(singular[rank - 1])) if rank else 0.0
    cond = float(largest / singular[rank - 1]) if rank == len(columns) and rank else float("inf")
    g = C @ (np.linalg.pinv(information) @ (H.T @ A))
    slopes = []
    for axis in range(3):
        v = g[axis]
        try:
            value = float(np.sqrt(max(0.0, v @ np.linalg.solve(gamma, v))))
        except np.linalg.LinAlgError:
            value = float("inf")
        slopes.append(value)
    monitorable_evidence = bool(
        rank == len(columns) and sigma_min >= MIN_SIGMA and cond <= MAX_GRAM_COND)
    monitorable_pl = bool(rank == len(columns))
    return {
        "q": len(columns),
        "rows_touched": len(rows_touched),
        "rank_A": int(np.linalg.matrix_rank(A, tol=RANK_TOL)),
        "rank_Z": rank,
        "sigma_min_Z": sigma_min,
        "condition_Z": cond,
        "slopes_xyz": slopes,
        "monitorable_evidence_gate": monitorable_evidence,
        "monitorable_pl_gate": monitorable_pl,
        "detection_energy": float(score @ np.linalg.solve(gamma, score)) if rank == len(columns) else None,
    }


def build_ladder(frame, anchor_id=1):
    rows = anchor_row_indices(frame, anchor_id)
    current = [r for r in rows if r["disposition"] == "PendingExplicit"]
    history = [r for r in rows if r["disposition"] == "ExplicitMeasurement"]
    if not current:
        return {"reason": "no pending UWB block in this frame (rejected/missing stream)"}
    current_row = current[0]["row"]
    persistent_support = [r["row"] for r in history] + [current_row]
    variants = {
        # 1) one-dimensional instantaneous fault on the current epoch's row
        "one_dim_instant": [[(current_row, 1.0)]],
        # 2) production template family ("existing template"):
        #    epoch-independent (current epoch only) and persistent (window-wide),
        #    plus their two-parameter union exactly as production tests them
        "template_epoch_independent": [[(current_row, 1.0)]],
        "template_persistent": [[(row, 1.0) for row in persistent_support]],
        "template_family_union": [
            [(row, 1.0) for row in persistent_support],
            [(current_row, 1.0)],
        ],
        # 3) source-level free profile: one independent parameter per epoch
        "source_free_profile": [
            [(row, 1.0)] for row in persistent_support
        ],
    }
    result = {
        "anchor_id": anchor_id,
        "row_support": {
            "current_rows": [r["row"] for r in current],
            "history_rows": [r["row"] for r in history],
            "blocks": rows,
        },
        "variants": {},
    }
    for name, columns in variants.items():
        result["variants"][name] = ladder_variant(frame, columns)
    return result


# --------------------------------------------------------------------------
# shadow diagnostics from diagnostic v10 tables
# --------------------------------------------------------------------------

CHECK_ORDER = [
    ("integrity_window", "window construction (model_valid, rank, condition)"),
    ("window_boundary_provenance", "boundary provenance / slot identity"),
    ("window_factor_linearization_whitening", "factor linearization + whitening"),
    ("window_dense_assembly", "dense assembly"),
    ("window_svd", "canonical Jacobian-space SVD"),
    ("window_normal_equations", "normal equations (H^T H)"),
    ("window_llt_state_solves", "LLT base solves and parity"),
    ("window_fingerprint", "content fingerprint"),
    ("all_in_detector", "all-in joint window detector"),
    ("current_sensitivity", "analytic IMU sensitivity input/computation"),
    ("model_generation", "fault mode/hypothesis/action generation"),
    ("historical_sensitivity", "historical IMU sensitivity recomputation"),
    ("hypothesis_evidence", "per-hypothesis Gram/rank/slope evidence"),
    ("health_actions", "health state transits + candidate eligibility"),
    ("hypothesis_audit", "hypothesis audit export"),
    ("base_factorization", "base candidate kernel factorization"),
    ("shared_cache", "per-block shared solve cache"),
    ("candidate_evaluation", "candidate kernel + post detector + PL"),
    ("fde_decision", "FDE decision / candidate selection"),
    ("candidate_audit", "candidate + coverage audit"),
    ("commit", "atomic backend commit"),
    ("discard", "fail-closed discard"),
    ("state_audit", "ledger/health audit snapshot"),
    ("finalize_commit_audit", "finalize + publication"),
]


def shadow_diagnostics(run, attempt):
    attempts = {r["input_attempt_id"]: r for r in
                read_csv(os.path.join(RAW, "runs", run, "diagnostic_attempts.csv"))}
    stages = [r for r in read_csv(
        os.path.join(RAW, "runs", run, "diagnostic_stages.csv"))
        if r["input_attempt_id"] == str(attempt)]
    candidates = [r for r in read_csv(
        os.path.join(RAW, "runs", run, "diagnostic_candidates.csv"))
        if r["input_attempt_id"] == str(attempt)]
    coverage = [r for r in read_csv(
        os.path.join(RAW, "runs", run, "diagnostic_coverage.csv"))
        if r["input_attempt_id"] == str(attempt)]
    integrity = [r for r in read_csv(
        os.path.join(RAW, "runs", run, "integrity.csv"))][attempt - 1]
    attempt_row = attempts[str(attempt)]

    stage_by_name = {s["stage"]: s for s in stages}
    checks = []
    for name, label in CHECK_ORDER:
        stage = stage_by_name.get(name)
        if stage is None:
            checks.append({"name": name, "label": label, "status": "NOT_EVALUATED",
                           "detail": "stage absent from diagnostic_stages.csv"})
            continue
        status = stage["status"]
        mapped = "PASS" if status == "EXECUTED" else (
            "SKIP_WITH_REASON" if status == "SKIPPED" else "FAIL")
        checks.append({"name": name, "label": label, "status": mapped,
                       "detail": stage["reason"][:200]})
    failed = [c for c in checks if c["status"] == "FAIL"]
    # First limitation search: stage-level failure, then candidate-level
    # numerical rejection, then coverage/eligibility outcome, then publication.
    limitation = None
    if failed:
        limitation = {"layer": "stage", "name": failed[0]["name"],
                      "detail": failed[0]["detail"]}
    if limitation is None:
        for c in candidates:
            if c["numerical_valid"] == "0" and (c["skip_reason"] or c["fallback_reason"]):
                limitation = {
                    "layer": "candidate",
                    "name": f"action {c['action_id']}",
                    "detail": (c["skip_reason"] or c["fallback_reason"])[:200],
                }
                break
    if limitation is None:
        for c in coverage:
            if c["outcome"] not in ("SELECTED", "COVERED"):
                limitation = {
                    "layer": "coverage",
                    "name": f"action {c['action_id']}: {c['outcome']}",
                    "detail": c["reason"][:200],
                }
                break
    if limitation is None:
        limitation = {"layer": "publication", "name": attempt_row["status"],
                      "detail": attempt_row["reason"][:200] or
                      "no identified limitation; nominal path"}
    candidate_checks = [{
        "action_id": c["action_id"],
        "kernel_evaluated": c["kernel_evaluated"] == "1",
        "numerical_valid": c["numerical_valid"] == "1",
        "post_passed": c["post_passed"] == "1",
        "pl_evaluated": c["pl_evaluated"] == "1",
        "selected": c["selected"] == "1",
        "numerical_path": c["numerical_path"],
        "skip_reason": c["skip_reason"][:120],
        "fallback_reason": c["fallback_reason"][:120],
    } for c in candidates]
    coverage_checks = [{
        "action_id": c["action_id"],
        "outcome": c["outcome"],
        "reason": c["reason"][:160],
        "uncovered_hypothesis_ids": c["uncovered_hypothesis_ids"][:160],
    } for c in coverage]
    # checks that the online path provably did not evaluate (early return)
    not_evaluated = []
    if attempt_row["pl_evaluated_actions"] == "0":
        not_evaluated.append("post-FDE protection level for every hypothesis/action "
                             "(no kernel+PL pass reached)")
    if all(c["kernel_evaluated"] == "0" for c in candidate_checks):
        not_evaluated.append("candidate numerical certificates (no kernel evaluation)")
    if attempt_row["post_passed_actions"] == "0":
        not_evaluated.append("post-FDE detector for every action")
    return {
        "attempt_status": attempt_row["status"],
        "attempt_reason": attempt_row["reason"],
        "checks": checks,
        "first_limitation": limitation,
        "candidate_checks": candidate_checks,
        "coverage_checks": coverage_checks,
        "not_evaluated": not_evaluated,
        "publication": {
            "batch_committed": integrity["batch_committed"] == "1",
            "availability": integrity["availability"],
            "fde_status": integrity["fde_status"],
            "pl_xyz": [integrity["pl_x"], integrity["pl_y"], integrity["pl_z"]],
            "reason": integrity["reason"][:200],
        },
        "counts": {
            "hypotheses": int(attempt_row["hypothesis_count"]),
            "generated_actions": int(attempt_row["generated_actions"]),
            "kernel_evaluated": int(attempt_row["kernel_evaluated_actions"]),
            "post_passed": int(attempt_row["post_passed_actions"]),
            "pl_evaluated": int(attempt_row["pl_evaluated_actions"]),
            "selected": int(attempt_row["selected_actions"]),
            "single_uwb": int(attempt_row["single_uwb_hypotheses"]),
            "single_accel": int(attempt_row["single_accel_hypotheses"]),
            "single_gyro": int(attempt_row["single_gyro_hypotheses"]),
            "double_uwb_accel": int(attempt_row["double_uwb_accel_hypotheses"]),
            "double_uwb_gyro": int(attempt_row["double_uwb_gyro_hypotheses"]),
        },
    }


def production_hypothesis_crosscheck(run, attempt, anchor_id):
    """Compare the ladder's single-anchor template numbers with the production
    hypothesis evidence tables of the same frame."""
    rows = [r for r in read_csv(os.path.join(RAW, "runs", run, "hypotheses.csv"))
            if r["window_id"] == str(attempt)]
    matches = []
    for row in rows:
        if row["sensor"] != "UWB":
            continue
        if row["fault_kind"] != "ANCHOR_BIAS_EPOCH_INDEPENDENT":
            continue
        if f"uwb:{anchor_id}" not in row["physical_source_ids"]:
            continue
        matches.append({
            "hypothesis_id": row["hypothesis_id"],
            "fault_rank": row["fault_rank"],
            "sigma_min": row["sigma_min"],
            "sigma_max": row["sigma_max"],
            "condition_number": row["condition_number"],
            "slopes_xyz": [row["slope_x"], row["slope_y"], row["slope_z"]],
            "monitorable": row["monitorable"] == "1",
        })
    return matches


# --------------------------------------------------------------------------

def main():
    census = {
        "source_revision": open(os.path.join(ROOT, "worktree-before.txt")).read()
            .split("--- git rev-parse HEAD")[1].split("\n")[1].strip(),
        "frames": {},
        "hypothesis_construction": {
            "formula_single": (
                "single = N_anchor * (epochs+1) * (2 + ramp) + 6 * occurrences"
            ),
            "formula_double": (
                "double_uwb_accel = single_uwb_structural * (3 * occurrences); "
                "double_uwb_gyro = single_uwb_structural * (3 * occurrences)"
            ),
            "note": (
                "occurrences = committed epochs whose factor groups are explicitly "
                "inside the frozen window (rejected epochs drop out); structural UWB "
                "modes = 2 per (anchor,onset) [+1 ramp] with onsets spanning "
                "[detector_first_epoch, proposed_epoch]"
            ),
            "observed": {},
        },
    }
    fixtures_out = {"frames": []}
    for spec in FIXTURES:
        bin_path = os.path.join(RAW, "replay", spec["run"], "exports",
                                f"attempt-{spec['attempt']}.bin")
        frame = read_replay(bin_path)
        census["frames"][f"{spec['run']}#{spec['attempt']}"] = frame_census(frame)
        entry = {
            "class": spec["class"],
            "run": spec["run"],
            "attempt": spec["attempt"],
            "scenario": spec["scenario"],
            "note": spec["note"],
            "bin": os.path.relpath(bin_path, ROOT),
            "bin_sha256": sha256(bin_path),
            "window": {
                "id": frame["window"]["id"],
                "detector_first_epoch": frame["window"]["detector_first_epoch"],
                "recovery_first_epoch": frame["window"]["recovery_first_epoch"],
                "model_valid": frame["window"]["model_valid"],
                "reason": frame["window"]["reason"],
            },
        }
        entry["ladder"] = build_ladder(frame, anchor_id=1)
        entry["shadow"] = shadow_diagnostics(spec["run"], spec["attempt"])
        entry["production_hypothesis_crosscheck"] = production_hypothesis_crosscheck(
            spec["run"], spec["attempt"], anchor_id=1)
        fixtures_out["frames"].append(entry)

    # hypothesis census observed numbers (from the runs, not hand-entered)
    for run, attempt, label in [
        ("A_nominal", 30, "A_nominal_epoch30"),
        ("F_ramp_unmonitorable", 30, "F_ramp_epoch30"),
        ("G_continuous_rejection", 45, "G_epoch45_rejected_stream"),
        ("H_mature_union", 201, "H_epoch201_mature"),
        ("raw/census/sd", 6, "SD_census_epoch6_double_faults"),
    ]:
        path = os.path.join(RAW, "runs", run, "diagnostic_attempts.csv")
        if run.startswith("raw/"):
            path = os.path.join(ROOT, run, "diagnostic_attempts.csv")
        row = [r for r in read_csv(path)
               if r["input_attempt_id"] == str(attempt)][0]
        census["hypothesis_construction"]["observed"][label] = {
            "single_uwb": int(row["single_uwb_hypotheses"]),
            "single_accel": int(row["single_accel_hypotheses"]),
            "single_gyro": int(row["single_gyro_hypotheses"]),
            "double_uwb_accel": int(row["double_uwb_accel_hypotheses"]),
            "double_uwb_gyro": int(row["double_uwb_gyro_hypotheses"]),
            "total": int(row["hypothesis_count"]),
            "effective_cardinality": int(row["effective_fault_cardinality"]),
        }

    with open(os.path.join(ROOT, "census.json"), "w") as handle:
        json.dump(census, handle, indent=1)
    os.makedirs(os.path.join(ROOT, "fixtures"), exist_ok=True)
    with open(os.path.join(ROOT, "fixtures", "frames.json"), "w") as handle:
        json.dump(fixtures_out, handle, indent=1)
    print("census.json and fixtures/frames.json written")


if __name__ == "__main__":
    main()
