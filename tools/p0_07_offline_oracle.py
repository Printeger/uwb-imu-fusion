#!/usr/bin/env python3
"""Process-A numerical oracle for the P0-07 frozen raw-factor protocol.

This program intentionally has no uwb_imu_pl dependency.  It reconstructs the
served and post-action linear systems from raw H/z/C, then independently
projects every declaratively enumerated UWB fault basis.
"""

import argparse
import functools
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import brentq
from scipy.stats import chi2, ncx2, norm


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


DERIVED_INPUT_PATHS = {
    "replay.epochs",
    "replay.epoch_period_ns", "replay.anchors[7]",
    "replay.imu_samples_per_epoch",
    "detector_and_action_recipe.detector.p_fa_per_test",
    "detector_and_action_recipe.risk.total_decimal",
    "detector_and_action_recipe.risk.nominal_axis_tail",
    "detector_and_action_recipe.risk.p_nm",
    "detector_and_action_recipe.risk.p_bridge_escape",
    "detector_and_action_recipe.risk.p_history_contamination",
    "detector_and_action_recipe.risk.p_model_escape",
    "detector_and_action_recipe.risk.horizontal_alert_limit_m",
    "detector_and_action_recipe.risk.vertical_alert_limit_m",
}


def flatten_leaves(value, prefix=""):
    leaves = {}
    if isinstance(value, dict):
        if not value:
            leaves[prefix] = value
        for key, child in value.items():
            path = f"{prefix}.{key}" if prefix else key
            leaves.update(flatten_leaves(child, path))
    elif isinstance(value, list):
        if not value:
            leaves[prefix] = value
        for index, child in enumerate(value):
            leaves.update(flatten_leaves(child, f"{prefix}[{index}]"))
    else:
        leaves[prefix] = value
    return leaves


def fixed_contract_source(path):
    if path == "schema":
        return "docs/evidence/p0-07-corrected-exhaustive/oracle-manifest-v1.json:2:schema/versioned oracle protocol"
    if path == "authority":
        return "docs/evidence/p0-07-corrected-exhaustive/oracle-manifest-v1.json:3:authority/declarative authority boundary"
    if path.startswith("factor_construction.history_raw.slot_rules"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:3212:IncrementalUwbImuEstimator::auditRawRowOwnershipV1/factor-ledger slot identity"
    if path.startswith("factor_construction.history_raw.group_rules"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:1386:IncrementalUwbImuEstimator::buildIntegrityWindow/factor group identity"
    if path.startswith("detector_and_action_recipe.action_generation.replacement_group"):
        return "src/uwb_imu_pl/integrity/hypothesis_generator.cpp:1846:HypothesisGenerator::actionsForPlausibleSetV1/replacement identity"
    if path.startswith("fault_contract"):
        return "src/uwb_imu_pl/integrity/hypothesis_generator.cpp:1198:HypothesisGenerator::generate/physical fault-family contract"
    if path.startswith("tolerance_policy"):
        return "tools/p0_07_compare_protocols.py:15:close/quantity-aware comparison policy"
    if path.startswith("missing_provenance_fixture"):
        return "src/uwb_imu_pl/integrity/hypothesis_generator.cpp:1846:HypothesisGenerator::actionsForPlausibleSetV1/action provenance fail-closed contract"
    if path.startswith("detector_and_action_recipe.detector"):
        return "src/uwb_imu_pl/integrity/joint_window_detector.cpp:146:JointWindowDetector::evaluate/dual-channel detector contract"
    if path.startswith("detector_and_action_recipe.action_generation"):
        return "src/uwb_imu_pl/integrity/hypothesis_generator.cpp:1846:HypothesisGenerator::actionsForPlausibleSetV1/action census contract"
    if path.startswith("detector_and_action_recipe.risk"):
        return "src/uwb_imu_pl/integrity/risk_budget_audit.cpp:128:buildRiskLedger/risk contract"
    if path.startswith("detector_and_action_recipe.selection"):
        return "src/uwb_imu_pl/integrity/fde_manager.cpp:133:FdeManager::decide/stable winner ordering at line 287"
    if path.startswith("detector_and_action_recipe.resolved_config_binding"):
        return "src/uwb_imu_pl/config/integrity_config.cpp:285:IntegrityConfigLoader::load/strict resolved config"
    if path.startswith("factor_construction.state_columns"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:1386:IncrementalUwbImuEstimator::buildIntegrityWindow/state layout"
    if path.startswith("factor_construction.served_window"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:1386:IncrementalUwbImuEstimator::buildIntegrityWindow/served row layout"
    if path.startswith("factor_construction.whitening"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:1386:IncrementalUwbImuEstimator::buildIntegrityWindow/upper information root"
    if path.startswith("factor_construction.history_raw"):
        return "src/uwb_imu_pl/estimation/incremental_estimator.cpp:3212:IncrementalUwbImuEstimator::auditRawRowOwnershipV1/raw ownership"
    if path.startswith("replay"):
        return "tools/p0_07_production_probe.cpp:144:main/frozen replay protocol"
    raise KeyError("FIXED path has no audited production source: " + path)


def validate_fixed_manifest_contract(recipe, canonical):
    """A-owned path->typed invariant table; never reads the case TSV."""
    actual = flatten_leaves(recipe)
    expected = flatten_leaves(canonical)
    fixed_actual = {path for path in actual
                    if not path.startswith("observed_non_authoritative.")}
    fixed_expected = {path for path in expected
                      if not path.startswith("observed_non_authoritative.")}
    if fixed_actual != fixed_expected:
        missing = sorted(fixed_expected - fixed_actual)
        extra = sorted(fixed_actual - fixed_expected)
        raise ValueError(f"FIXED_TYPED_SCHEMA_LEAF_SET:missing={missing}:extra={extra}")
    for path in sorted(fixed_expected):
        if path in DERIVED_INPUT_PATHS or path.startswith("observed_non_authoritative."):
            continue
        lhs, rhs = actual[path], expected[path]
        if type(lhs) is not type(rhs):
            raise ValueError(f"FIXED_TYPED_TYPE:{path}:{type(rhs).__name__}")
        if lhs != rhs:
            raise ValueError(
                f"FIXED_TYPED_INVARIANT:{path}:{fixed_contract_source(path)}")


def write_fixed_contract_map(canonical, output):
    leaves = flatten_leaves(canonical)
    rows = ["path\tjson_type\tproduction_source_line_symbol_contract"]
    for path in sorted(leaves):
        if path in DERIVED_INPUT_PATHS or path.startswith("observed_non_authoritative."):
            continue
        value = leaves[path]
        json_type = ("boolean" if isinstance(value, bool) else
                     "integer" if isinstance(value, int) else
                     "number" if isinstance(value, float) else
                     "string" if isinstance(value, str) else
                     "array" if isinstance(value, list) else "object")
        rows.append(f"{path}\t{json_type}\t{fixed_contract_source(path)}")
    Path(output).write_text("\n".join(rows) + "\n")


def observed_protocol_line(recipe):
    observed = recipe["observed_non_authoritative"]
    plausible = observed["plausible_mode_ordinals"]
    fields = ["OBSERVED", str(len(plausible))]
    fields.extend(str(int(value)) for value in plausible)
    fields.extend((str(int(observed["generated_action_count"])),
                   str(int(observed["winner"])), observed["note"]))
    return "\t".join(fields)


def take_matrix(fields, at):
    rows, cols = int(fields[at]), int(fields[at + 1])
    end = at + 2 + rows * cols
    return np.asarray(fields[at + 2:end], dtype=float).reshape(rows, cols), end


def take_block(fields, at=1):
    group, kind, sensor, role = map(int, fields[at:at + 4])
    at += 4
    raw_h, at = take_matrix(fields, at)
    raw_z, at = take_matrix(fields, at)
    cov, at = take_matrix(fields, at)
    _claimed_h, at = take_matrix(fields, at)
    _claimed_z, at = take_matrix(fields, at)
    count = int(fields[at])
    columns = [int(x) for x in fields[at + 1:at + 1 + count]]
    return {"group": group, "kind": kind, "sensor": sensor, "role": role,
            "raw_h": raw_h, "raw_z": raw_z[:, 0], "cov": cov,
            "columns": columns}


def whiten(block, state_columns):
    # Declared convention: upper information square root R, R.T R = C^-1.
    root = np.linalg.cholesky(np.linalg.inv(block["cov"])).T
    raw_h = block["raw_h"]
    if raw_h.shape[1] != state_columns:
        expanded = np.zeros((raw_h.shape[0], state_columns))
        expanded[:, block["columns"]] = raw_h
        raw_h = expanded
    return root @ raw_h, root @ block["raw_z"]


def parse_capture(path):
    metadata, blocks, actions, added = {}, [], {}, {}
    matrices, columns = {}, []
    for line in Path(path).read_text().splitlines():
        f = line.split("\t")
        if len(f) == 2 and f[0].endswith("_sha256"):
            metadata[f[0]] = f[1]
        elif f[0] == "BLOCK":
            blocks.append(take_block(f))
        elif f[0] == "ACTION_INPUT":
            n = int(f[2]); action = int(f[1])
            actions[action] = [int(x) for x in f[3:3 + n]]
            added[action] = []
        elif f[0] == "ADDED_BLOCK":
            added[int(f[1])].append(take_block(f, 2))
        elif f[0] in {"PROTECTED_MAP", "HISTORY_RESPONSE",
                      "HISTORY_DETECTOR_RESPONSE", "HISTORY_D_PERP"}:
            matrices[f[0]], end = take_matrix(f, 1)
            if end != len(f):
                raise ValueError("trailing matrix payload: " + f[0])
        elif f[0] == "HISTORY_COLUMN":
            columns.append(tuple(map(int, f[1:4])))
    return metadata, blocks, actions, added, matrices, columns


def solve(h, z, tolerance):
    u, s, vt = np.linalg.svd(h, full_matrices=False)
    cutoff = tolerance * max(h.shape) * (s[0] if s.size else 1.0)
    keep = s > cutoff
    pinv = (vt[keep].T / s[keep]) @ u[:, keep].T
    state = pinv @ z
    residual = z - h @ state
    return {"rank": int(np.count_nonzero(keep)), "dof": h.shape[0] - int(np.count_nonzero(keep)),
            "state": state, "stat": float(residual @ residual), "pinv": pinv,
            "projector": np.eye(h.shape[0]) - h @ pinv,
            "information": h.T @ h, "rhs": h.T @ z}


def enumerate_modes(raw, recipe, blocks, history, history_columns):
    first = recipe["fault_contract"]["onset_epochs"]["first"]
    last = recipe["fault_contract"]["onset_epochs"]["last"]
    anchors = recipe["replay"]["anchors"]
    period = recipe["replay"]["epoch_period_ns"] * 1e-9
    rows_by_epoch = {}
    for row in raw["uwb_rows"]:
        rows_by_epoch.setdefault(int(row["epoch"]), []).append(int(row["anchor_id"]))
    block_rows = {b["group"]: b["raw_h"].shape[0] for b in blocks}
    modes = []
    for anchor in anchors:
        for onset in range(first, last + 1):
            for family in (1, 2, 3):
                dim = 2 if family == 3 else 1
                per_group, raw_maps = {}, {}
                # History response is already the frozen raw-to-served linear
                # transformation; coefficients are independently constructed.
                hist = np.zeros((history.shape[0], dim))
                for col, (kind, source, epoch) in enumerate(history_columns):
                    if source != anchor:
                        continue
                    effective = first if onset == recipe["replay"]["detector_first_epoch"] and family != 1 else onset
                    if (family == 1 and epoch != onset) or (family != 1 and epoch < effective):
                        continue
                    weight = np.zeros(dim)
                    if kind == 0:
                        weight[0] = 1.0
                        if family == 3:
                            weight[1] = (epoch - onset - 1) * period
                    elif kind == 1 and family == 3:
                        weight[1] = 1.0
                    hist += np.outer(history[:, col], weight)
                if np.any(hist):
                    per_group[23000] = hist
                for epoch, anchor_rows in rows_by_epoch.items():
                    group = epoch * 1000 + 2
                    if group not in block_rows:
                        continue
                    effective = first if onset == recipe["replay"]["detector_first_epoch"] and family != 1 else onset
                    if (family == 1 and epoch != onset) or (family != 1 and epoch < effective):
                        continue
                    raw_map = np.zeros((block_rows[group], dim))
                    for row, row_anchor in enumerate(anchor_rows):
                        if row_anchor == anchor:
                            raw_map[row, 0] = 1.0
                            if family == 3:
                                raw_map[row, 1] = (epoch - onset) * period
                    block = next(b for b in blocks if b["group"] == group)
                    root = np.linalg.cholesky(np.linalg.inv(block["cov"])).T
                    if np.any(raw_map):
                        per_group[group] = root @ raw_map
                        raw_maps[group] = raw_map
                full_fault = (np.vstack(list(per_group.values()))
                              if per_group else np.zeros((0, dim)))
                active = [column for column in range(dim)
                          if np.any(full_fault[:, column] != 0.0)]
                modes.append({"id": len(modes) + 1, "anchor": anchor,
                              "onset": onset, "family": family,
                              "dimension": dim, "maps": per_group,
                              "raw_maps": raw_maps, "active": active})
    return modes


def classify(gram, response, tolerance):
    eig, vec = np.linalg.eigh((gram + gram.T) * 0.5)
    cutoff = tolerance * max(1.0, float(np.max(np.abs(eig)))) * max(gram.shape)
    positive = eig > cutoff
    null_response = response @ vec[:, ~positive] if np.any(~positive) else np.zeros((3, 0))
    dangerous = null_response.size and np.max(np.abs(null_response)) > cutoff
    if dangerous:
        return 3, np.full(3, np.inf)
    inverse = (vec[:, positive] / eig[positive]) @ vec[:, positive].T if np.any(positive) else np.zeros_like(gram)
    slopes = np.sqrt(np.maximum(0.0, np.diag(response @ inverse @ response.T)))
    return (2 if np.any(~positive) else 1), slopes


def certified_psd(matrix, tolerance):
    """Independent small-matrix PSD/rank-transition decision."""
    symmetric = (matrix + matrix.T) * 0.5
    eig, vec = np.linalg.eigh(symmetric)
    scale = max(1.0, float(np.max(np.abs(eig))) if eig.size else 0.0)
    cutoff = tolerance * scale * max(1, matrix.shape[0])
    if np.any(eig < -cutoff):
        return None, "INDEFINITE"
    # A positive eigenvalue inside the rank-decision band is neither safely
    # zero nor safely positive.  The production numerical contract calls this
    # unavailable instead of silently choosing a rank.
    if np.any((eig > 0.0) & (eig <= cutoff)):
        return None, "RANK_TRANSITION_INDETERMINATE"
    keep = eig > cutoff
    return (symmetric, eig, vec, keep), "OK"


@functools.lru_cache(maxsize=None)
def noncentrality_boundary(dof, threshold, p_md):
    # Every hypothesis in one detector channel shares this scalar statistical
    # boundary.  Cache only that pure chi-square inversion; all 7 x 504
    # hypothesis matrices, responses, nullspaces, PLs and risks are still
    # rebuilt independently below.
    def residual(value):
        return ncx2.cdf(threshold, dof, value) - p_md
    upper = 1.0
    while residual(upper) > 0.0:
        upper *= 2.0
        if upper > 1.0e12:
            raise ValueError("NONCENTRALITY_BOUNDARY_UNAVAILABLE")
    return brentq(residual, 0.0, upper, xtol=1e-13, rtol=1e-14)


def dual_channel_component(total_gram, history_gram, response, sigma,
                           current_dof, history_dof, p_fa, p_md,
                           axis_tail, tolerance):
    channels = []
    for name, gram, dof in (
            ("CURRENT", total_gram - history_gram, current_dof),
            ("HISTORY", history_gram, history_dof)):
        certificate, reason = certified_psd(gram, tolerance)
        if certificate is None:
            return None, name + "_" + reason
        symmetric, eig, vec, keep = certificate
        if not np.any(keep) or dof <= 0:
            continue
        threshold = chi2.ppf(1.0 - p_fa, dof)
        lam = noncentrality_boundary(dof, threshold, p_md)
        channels.append((symmetric, lam))
    if not channels:
        return None, "NO_USABLE_CHANNEL"
    weight = 1.0 / len(channels)
    w = sum((weight / lam) * gram for gram, lam in channels)
    certificate, reason = certified_psd(w, tolerance)
    if certificate is None:
        return None, "W_" + reason
    symmetric, eig, vec, keep = certificate
    null = response @ vec[:, ~keep] if np.any(~keep) else np.zeros((3, 0))
    cutoff = tolerance * max(1.0, float(np.max(np.abs(eig)))) * max(1, w.shape[0])
    if null.size and np.max(np.abs(null)) > cutoff:
        return None, "DANGEROUS_W_NULLSPACE"
    inverse = ((vec[:, keep] / eig[keep]) @ vec[:, keep].T
               if np.any(keep) else np.zeros_like(w))
    bound = np.sqrt(np.maximum(0.0, np.diag(response @ inverse @ response.T)))
    multiplier = norm.ppf(1.0 - axis_tail / 2.0)
    component = bound + multiplier * sigma
    if not np.all(np.isfinite(component)):
        return None, "NONFINITE_COMPONENT"
    return component, "FINITE"


def replacement_fault_map(block, mode, base_blocks):
    """Map each replacement row back to its exact source UWB raw row."""
    raw_map = np.zeros((block["raw_h"].shape[0], mode["dimension"]))
    for target_row, values in enumerate(block["raw_h"]):
        match = None
        for source in base_blocks:
            if source["group"] not in mode["raw_maps"]:
                continue
            for source_row, source_values in enumerate(source["raw_h"]):
                if np.array_equal(values, source_values):
                    match = mode["raw_maps"][source["group"]][source_row]
                    break
            if match is not None:
                break
        if match is not None:
            raw_map[target_row] = match
    root = np.linalg.cholesky(np.linalg.inv(block["cov"])).T
    return root @ raw_map


def write_matrix(out, label, matrix):
    flat = "\t".join(format(float(x), ".17g") for x in np.asarray(matrix).ravel())
    out.write(f"{label}\t{matrix.shape[0]}\t{matrix.shape[1]}")
    if flat:
        out.write("\t" + flat)
    out.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", required=True); ap.add_argument("--raw", required=True)
    ap.add_argument("--truth", required=True); ap.add_argument("--config", required=True)
    ap.add_argument("--manifest", required=True); ap.add_argument("--output", required=True)
    ap.add_argument("--canonical-manifest", required=True)
    ap.add_argument("--base-oracle")
    ap.add_argument("--fixed-contract-map")
    args = ap.parse_args()
    recipe = json.loads(Path(args.manifest).read_text())
    canonical_recipe = json.loads(Path(args.canonical_manifest).read_text())
    validate_fixed_manifest_contract(recipe, canonical_recipe)
    if args.fixed_contract_map:
        write_fixed_contract_map(canonical_recipe, args.fixed_contract_map)
    if args.base_oracle:
        # OBSERVED fields are non-authoritative declarations.  Preserve the
        # already hash-bound numerical oracle byte-for-byte and change only
        # its expected same-run observation line.  The ordinary path below
        # still verifies every frozen input hash before doing any arithmetic.
        lines = Path(args.base_oracle).read_text().splitlines()
        replaced = False
        for index, line in enumerate(lines):
            if line.startswith("OBSERVED\t"):
                lines[index] = observed_protocol_line(recipe)
                replaced = True
        if not replaced:
            raise ValueError("base oracle has no OBSERVED protocol")
        Path(args.output).write_text("\n".join(lines) + "\n")
        return
    metadata, blocks, actions, added, matrices, history_columns = parse_capture(args.capture)
    for key, path in (("raw_sha256", args.raw), ("truth_sha256", args.truth),
                      ("config_sha256", args.config), ("manifest_sha256", args.manifest)):
        if metadata.get(key) != sha256(path):
            raise SystemExit("frozen input hash mismatch: " + key)
    raw = json.loads(Path(args.raw).read_text())
    config = yaml.safe_load(Path(args.config).read_text())
    tolerance = float(config["integrity_window"]["rank_tolerance"])
    truth = json.loads(Path(args.truth).read_text())
    fault_anchor = int(truth["fault"]["anchor_id"])
    # The candidate column count is an observed property of the independently
    # captured raw blocks.  This keeps genuine alarm/window input mutations
    # from borrowing the base recipe's 11-epoch dimension.
    state_columns = max(
        max(block["raw_h"].shape[1],
            (max(block["columns"]) + 1) if block["columns"] else 0)
        for block in blocks)
    whitened = {b["group"]: whiten(b, state_columns) for b in blocks}
    served_h = np.vstack([whitened[b["group"]][0] for b in blocks])
    served_z = np.concatenate([whitened[b["group"]][1] for b in blocks])
    base = solve(served_h, served_z, tolerance)
    history = np.vstack((matrices["HISTORY_RESPONSE"], matrices["HISTORY_DETECTOR_RESPONSE"]))
    modes = enumerate_modes(raw, recipe, blocks, history, history_columns)
    results = []
    action_outcomes = []
    risk = config["risk"]
    prior = float(config["fault_models"]["uwb"]["prior_probability_bound"])
    p_md = float(config["fault_models"]["uwb"]["p_md"])
    remaining = (float(risk["p_hmi_total"]) -
                 3.0 * float(risk["nominal_axis_tail"]) -
                 float(risk["p_nm"]) - float(risk["p_bridge_escape"]) -
                 float(risk["p_history_contamination"]) -
                 float(risk["p_model_escape"]))
    allocation = remaining / len(modes)
    while allocation * len(modes) > remaining:
        allocation = np.nextafter(allocation, 0.0)
    hypothesis_tail = min(0.5, allocation / prior)
    axis_tail = hypothesis_tail / 3.0
    history_state_rows = matrices["HISTORY_RESPONSE"].shape[0]
    history_detector_rows = matrices["HISTORY_DETECTOR_RESPONSE"].shape[0]
    for action in sorted(actions):
        kept = [b for b in blocks if b["group"] not in actions[action]] + added[action]
        parts = [(b, whiten(b, state_columns)) for b in kept]
        h = np.vstack([p[1][0] for p in parts]); z = np.concatenate([p[1][1] for p in parts])
        solution = solve(h, z, tolerance)
        post_passed = solution["stat"] <= chi2.ppf(
            1.0 - float(config["detector"]["p_fa_per_test"]),
            solution["dof"])
        protected_gain = matrices["PROTECTED_MAP"] @ solution["pinv"]
        finite_components = []
        unavailable_codes = []
        step_valid = float(np.linalg.norm(solution["state"])) <= float(
            config["integrity_window"]["max_linearization_step_norm"])
        for mode in modes:
            removed_epochs = {group // 1000 for group in actions[action]
                              if group % 1000 == 2}
            covered = mode["anchor"] == fault_anchor and mode["onset"] in removed_epochs
            if not post_passed or covered:
                results.append((action, mode["id"], False, np.empty((0, 0)),
                                np.empty((0, 0)), np.full((3, 1), np.inf), -1,
                                np.full((3, 1), np.inf), 0.0,
                                "NOT_IN_POST_ACTION_HYPOTHESIS_SET"))
                continue
            maps = []
            for block, (bh, bz) in parts:
                if block["group"] in mode["maps"]:
                    maps.append(mode["maps"][block["group"]])
                elif any(block is replacement for replacement in added[action]):
                    maps.append(replacement_fault_map(block, mode, blocks))
                else:
                    maps.append(np.zeros((bh.shape[0], mode["dimension"])))
            fault = np.vstack(maps)
            # Production's effective basis contract removes only structurally
            # exact-zero physical columns (not small singular directions).
            fault = fault[:, mode["active"]]
            if fault.shape[1] == 0:
                results.append((action, mode["id"], False,
                                np.empty((0, 0)), np.empty((0, 0)),
                                np.full((3, 1), np.inf), -1,
                                np.full((3, 1), np.inf), 0.0,
                                "EMPTY_EFFECTIVE_BASIS"))
                continue
            # A removed physical group has no fault in its replacement block.
            gram = fault.T @ solution["projector"] @ fault
            response = protected_gain @ fault
            null_class, slopes = classify(gram, response, tolerance)
            # The history summary block is first in this frozen protocol.  Its
            # state-supported rows precede the detector-only F_b rows.
            begin = history_state_rows
            end = begin + history_detector_rows
            history_gram = fault[begin:end].T @ fault[begin:end]
            sigma = np.sqrt(np.maximum(
                0.0, np.diag(protected_gain @ protected_gain.T)))
            component, component_code = dual_channel_component(
                gram, history_gram, response, sigma,
                solution["dof"] - history_detector_rows,
                history_detector_rows,
                float(config["detector"]["p_fa_per_test"]), p_md,
                axis_tail, tolerance)
            if component is None:
                component = np.full(3, np.inf)
                unavailable_codes.append(component_code)
                served_tail = 0.0
            else:
                finite_components.append(component)
                served_tail = hypothesis_tail
            results.append((action, mode["id"], True, gram, response,
                            slopes.reshape(3, 1), null_class,
                            component.reshape(3, 1), served_tail,
                            component_code))
        candidate_valid = step_valid and not unavailable_codes
        eligible = candidate_valid and post_passed
        refusal = ("STEP_GATE" if not step_valid else
                   (unavailable_codes[0] if unavailable_codes else
                    ("POST_DETECTOR" if not post_passed else "")))
        action_outcomes.append((action, candidate_valid, post_passed,
                                eligible, False, refusal))
        results.append(("ACTION", action, solution))
    with Path(args.output).open("w") as out:
        anchors = sorted({int(row["anchor_id"]) for row in raw["uwb_rows"]})
        out.write(f"INPUT\t{int(raw['epochs'])}\t{int(truth['fault']['epoch'])}\t"
                  f"{int(raw['imu']['samples_per_epoch'])}\t"
                  f"{int(raw['imu']['period_ns'])}\t{len(anchors)}")
        for anchor in anchors:
            out.write(f"\t{anchor}")
        out.write("\n")
        out.write(f"P007_ORACLE_V1\t{base['rank']}\t{base['dof']}\t{base['stat']:.17g}\t{len(modes)}\t{len(actions)}\n")
        write_matrix(out, "SERVED_H", served_h); write_matrix(out, "SERVED_Z", served_z.reshape(-1, 1))
        write_matrix(out, "BASE_INFORMATION", base["information"]); write_matrix(out, "BASE_RHS", base["rhs"].reshape(-1, 1))
        write_matrix(out, "BASE_STATE", base["state"].reshape(-1, 1))
        for item in results:
            if item[0] == "ACTION":
                s = item[2]; out.write(f"ACTION\t{item[1]}\t{s['rank']}\t{s['dof']}\t{s['stat']:.17g}\n")
                continue
            (action, mode, available, gram, response, slopes, null_class,
             component, served_tail, component_code) = item
            out.write(f"ORACLE_PROOF\t{action}\t{mode}\t{int(available)}\t{null_class}\t")
            for matrix in (gram, response, slopes, component):
                out.write(f"{matrix.shape[0]}\t{matrix.shape[1]}")
                for value in matrix.ravel(): out.write("\t" + format(float(value), ".17g"))
                out.write("\t")
            out.write(f"{prior:.17g}\t{p_md:.17g}\t{allocation:.17g}\t"
                      f"{served_tail:.17g}\t{component_code}\n")
        for action, valid, post, eligible, selected, refusal in action_outcomes:
            out.write(f"ACTION_OUTCOME\t{action}\t{int(valid)}\t{int(post)}\t"
                      f"2\t{int(eligible)}\t{int(selected)}\t{refusal}\n")
        charged_hypotheses = allocation * len(modes)
        charged_total = (3.0 * float(risk["nominal_axis_tail"]) +
                         float(risk["p_nm"]) +
                         float(risk["p_bridge_escape"]) +
                         float(risk["p_history_contamination"]) +
                         float(risk["p_model_escape"]) + charged_hypotheses)
        out.write(f"RISK\t{3.0 * float(risk['nominal_axis_tail']):.17g}\t"
                  f"{float(risk['p_nm']):.17g}\t{charged_hypotheses:.17g}\t"
                  f"{charged_total:.17g}\t{float(risk['p_hmi_total']):.17g}\n")
        winner = next((a for a, _, _, eligible, _, _ in action_outcomes
                       if eligible), 0)
        terminal = "RECOVERED" if winner else "AMBIGUOUS_UNAVAILABLE"
        out.write(f"DECISION\t{winner}\t{terminal}\t0\t0\n")
        out.write(observed_protocol_line(recipe) + "\n")


if __name__ == "__main__":
    main()
