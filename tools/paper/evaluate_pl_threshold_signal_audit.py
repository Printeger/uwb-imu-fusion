#!/usr/bin/env python3
"""Post-seal evaluator for the PL threshold and per-anchor signal audit."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.stats import chi2


SOURCE_PREFLIGHT_SEAL_SHA256 = (
    "f709379166fb3b67556ad196e8d0b57d0c17d9dbe2d79d4fa01a080abd8e1693"
)
LOCKED_MANIFEST_SHA256 = (
    "1660f7a1e8d1bd32bfabda394adf9e69675fb849917b028b66fff7ec0e87e820"
)
EXPECTED_MAX_AFFECTED_T = 4.860523050243609
P_FA_SWEEP = (1e-5, 1e-4, 1e-3, 1e-2, 0.05, 0.10)
IDENTITY_FIELDS = ("group_id", "anchor_id", "obs_id")


class PrecheckError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def load_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_seal(root: Path, seal: Path):
    checked = []
    for raw in seal.read_text(encoding="utf-8").splitlines():
        expected, name = raw.split(maxsplit=1)
        path = Path(name)
        if not path.is_absolute():
            path = root / path
        if sha256(path) != expected:
            raise PrecheckError(f"SEALED_HASH_MISMATCH:{path}")
        checked.append(str(path))
    return checked


def trace_forbidden_reads(path: Path):
    """Return successful detector-process opens of truth/GT/oracle data."""
    forbidden = ("/truth/", "/ground_truth", "/oracle", "/mocap", "/data/")
    hits = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        lower = line.lower()
        if " = -1 " in lower:
            continue
        if any(token in lower for token in forbidden):
            hits.append(line)
    return hits


def close(expected: float, actual: float) -> bool:
    scale = max(1.0, abs(expected), abs(actual))
    return abs(expected - actual) <= 1e-12 + 1e-10 * scale


def epoch_rows(path: Path):
    return [row for row in load_csv(path) if int(row["keyframe_id"]) > 4]


def compare_replay(source_rows, audit_rows, label: str):
    if len(source_rows) != len(audit_rows):
        raise PrecheckError(f"{label.upper()}_GROUP_COUNT_MISMATCH")
    discrete = (
        "keyframe_id", "sensor_time", "group_size", "obs_ids", "anchors",
        "committed_before_count", "committed_before_hash", "omnibus_status",
        "dof", "alarm", "outcome", "committed_count",
        "prior_degradation_event",
    )
    for old, new in zip(source_rows, audit_rows):
        for field in discrete:
            if old[field] != new[field]:
                raise PrecheckError(
                    f"{label.upper()}_REPLAY_DISCRETE_MISMATCH:"
                    f"{new['keyframe_id']}:{field}")
        for field in ("statistic", "threshold"):
            if not close(float(old[field]), float(new[field])):
                raise PrecheckError(
                    f"{label.upper()}_REPLAY_NUMERIC_MISMATCH:"
                    f"{new['keyframe_id']}:{field}")


def keyed(rows):
    result = {}
    for row in rows:
        key = tuple(row[field] for field in IDENTITY_FIELDS)
        if key in result:
            raise PrecheckError(f"DUPLICATE_ROW_IDENTITY:{key}")
        result[key] = row
    return result


def combine_anchor_rows(per_anchor_path: Path, conditional_path: Path):
    marginal = keyed(load_csv(per_anchor_path))
    conditional = keyed(load_csv(conditional_path))
    if marginal.keys() != conditional.keys():
        raise PrecheckError("MARGINAL_CONDITIONAL_IDENTITY_MISMATCH")
    output = {}
    for key in marginal:
        a = marginal[key]
        c = conditional[key]
        if a["diagnostic_valid"] != "1" or c["diagnostic_valid"] != "1":
            reason = a["invalid_reason"] or c["invalid_reason"] or "UNKNOWN"
            raise PrecheckError(f"ROW_DIAGNOSTIC_INVALID:{key}:{reason}")
        output[key] = {
            **a,
            "conditional_nu_m": c["conditional_nu_m"],
            "conditional_variance_m2": c["conditional_variance_m2"],
            "conditional_z": c["conditional_z"],
            "additive_group_contribution": c["additive_group_contribution"],
            "conditional_quadratic_increment":
                c["conditional_quadratic_increment"],
            "group_statistic": c["group_statistic"],
            "dof": c["dof"],
        }
    return output


def numeric(row, field):
    try:
        value = float(row[field])
    except (KeyError, ValueError) as error:
        raise PrecheckError(f"INVALID_NUMERIC_FIELD:{field}") from error
    if not math.isfinite(value):
        raise PrecheckError(f"NONFINITE_NUMERIC_FIELD:{field}")
    return value


def describe(values):
    array = np.asarray(values, dtype=float)
    if array.size == 0 or not np.isfinite(array).all():
        raise PrecheckError("EMPTY_OR_NONFINITE_PERSISTENT_SEQUENCE")
    return {
        "N": int(array.size),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "std_sample": float(np.std(array, ddof=1)) if array.size > 1 else 0.0,
        "P10": float(np.quantile(array, 0.10)),
        "P90": float(np.quantile(array, 0.90)),
        "nlos_direction_positive_fraction": float(np.mean(array > 0.0)),
        "max_absolute": float(np.max(np.abs(array))),
        "Z_sum": float(np.sum(array) / math.sqrt(array.size)),
    }


def mean(values):
    return float(np.mean(np.asarray(values, dtype=float)))


def write_csv(path: Path, fieldnames, rows):
    if path.exists():
        raise PrecheckError(f"REFUSING_TO_OVERWRITE:{path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def evaluate(args):
    root = args.audit_root.resolve()
    source = args.source_preflight_root.resolve()
    if sha256(source / "sealed_detector_hashes.sha256") != \
            SOURCE_PREFLIGHT_SEAL_SHA256:
        raise PrecheckError("SOURCE_PREFLIGHT_SEAL_IDENTITY_MISMATCH")
    if sha256(args.locked_manifest) != LOCKED_MANIFEST_SHA256:
        raise PrecheckError("LOCKED_MANIFEST_IDENTITY_MISMATCH")
    sealed_files = verify_seal(root, root / "sealed_statistics_hashes.sha256")

    trace_hits = {}
    for label in ("clean", "injected"):
        source_status = load_json(source / label / "pl_conditional_status.json")
        audit_status = load_json(root / label / "pl_conditional_status.json")
        if (audit_status.get("gt_read") is not False or
                audit_status.get("oracle_read") is not False):
            raise PrecheckError(f"{label.upper()}_TRUTH_BLIND_STATUS_INVALID")
        hits = trace_forbidden_reads(root / f"{label}_file_access.trace")
        trace_hits[label] = hits
        if hits:
            raise PrecheckError(f"{label.upper()}_FORBIDDEN_FILE_OPEN")
        for field in ("detector_identity_hash", "input_plan_hash",
                      "detected_group_count", "alarm_group_count",
                      "prior_degradation_event_count"):
            if source_status[field] != audit_status[field]:
                raise PrecheckError(
                    f"{label.upper()}_STATUS_IDENTITY_MISMATCH:{field}")
        compare_replay(
            epoch_rows(source / label / "pl_conditional_epochs.csv"),
            epoch_rows(root / label / "pl_conditional_epochs.csv"), label)

    clean_epochs = epoch_rows(root / "clean/pl_conditional_epochs.csv")
    injected_epochs = epoch_rows(root / "injected/pl_conditional_epochs.csv")
    if len(clean_epochs) != 224 or sum(int(r["alarm"]) for r in clean_epochs):
        raise PrecheckError("CLEAN_224_ZERO_ALARM_REPRODUCTION_FAILED")

    # All production-style nu/S/z quantities above are verified and sealed
    # before this first truth read.
    truth = load_json(args.truth)
    spec = truth["specification"]
    planned_ids = {str(value) for value in truth["planned_affected_obs_ids"]}
    if (truth.get("planned_status") != "ACTUAL_PLAN" or
            len(planned_ids) != 30 or spec["amplitude_m"] != 0.5 or
            spec["anchor_id"] != 20276 or spec["tag_id"] != 27956 or
            spec["start_s"] != 1664959678.3077347 or
            spec["end_s"] != 1664959686.3077347 or
            spec["interval"] != "closed"):
        raise PrecheckError("LOCKED_TRUTH_SPECIFICATION_MISMATCH")

    affected_epochs = []
    covered = set()
    for row in injected_epochs:
        ids = set(filter(None, row["obs_ids"].split(";")))
        overlap = ids & planned_ids
        if overlap:
            affected_epochs.append(row)
            covered.update(overlap)
    if len(affected_epochs) != 30 or covered != planned_ids:
        raise PrecheckError("AFFECTED_GROUP_OR_OBS_COVERAGE_MISMATCH")
    if sum(int(row["alarm"]) for row in affected_epochs):
        raise PrecheckError("AFFECTED_ZERO_ALARM_REPRODUCTION_FAILED")
    max_epoch = max(affected_epochs, key=lambda row: float(row["statistic"]))
    max_t = float(max_epoch["statistic"])
    if not close(EXPECTED_MAX_AFFECTED_T, max_t):
        raise PrecheckError("MAX_AFFECTED_STATISTIC_REPRODUCTION_FAILED")

    clean_groups = [(float(row["statistic"]), int(row["dof"]))
                    for row in clean_epochs]
    affected_groups = [(float(row["statistic"]), int(row["dof"]))
                       for row in affected_epochs]
    dofs = sorted({dof for _, dof in clean_groups + affected_groups})
    sweep_rows = []
    for p_fa in P_FA_SWEEP:
        thresholds = {dof: float(chi2.isf(p_fa, dof)) for dof in dofs}
        clean_alarm = sum(t > thresholds[dof] for t, dof in clean_groups)
        affected_alarm = sum(t > thresholds[dof]
                             for t, dof in affected_groups)
        row = {
            "p_fa": format(p_fa, ".17g"),
            **{f"threshold_dof_{dof}": format(thresholds[dof], ".17g")
               for dof in dofs},
            "clean_group_count": len(clean_groups),
            "clean_alarm_count": clean_alarm,
            "clean_empirical_alarm_fraction":
                format(clean_alarm / len(clean_groups), ".17g"),
            "affected_group_count": len(affected_groups),
            "affected_alarm_count": affected_alarm,
            "affected_recall":
                format(affected_alarm / len(affected_groups), ".17g"),
        }
        sweep_rows.append(row)
    sweep_fields = (["p_fa"] + [f"threshold_dof_{dof}" for dof in dofs] +
                    ["clean_group_count", "clean_alarm_count",
                     "clean_empirical_alarm_fraction", "affected_group_count",
                     "affected_alarm_count", "affected_recall"])
    write_csv(root / "pl_group_threshold_sweep.csv", sweep_fields, sweep_rows)

    clean = combine_anchor_rows(
        root / "pl_per_anchor_innovations_clean.csv",
        root / "pl_conditional_innovations_clean.csv")
    injected = combine_anchor_rows(
        root / "pl_per_anchor_innovations_injected.csv",
        root / "pl_conditional_innovations_injected.csv")
    if clean.keys() != injected.keys():
        raise PrecheckError("CLEAN_INJECTED_ROW_IDENTITY_MISMATCH")

    affected_group_ids = {f"pl-group-{row['keyframe_id']}"
                          for row in affected_epochs}
    paired_rows = []
    selected = []
    for key in sorted(clean, key=lambda item: (int(item[0].split("-")[-1]),
                                                int(item[1]))):
        c = clean[key]
        i = injected[key]
        if key[0] not in affected_group_ids:
            continue
        row = {
            "timestamp": i["timestamp"],
            "group_id": key[0],
            "keyframe_id": i["keyframe_id"],
            "tag_id": i["tag_id"],
            "anchor_id": key[1],
            "obs_id": key[2],
        }
        for field in ("nu_m", "marginal_z", "conditional_z"):
            clean_value = numeric(c, field)
            injected_value = numeric(i, field)
            row[f"{field}_clean"] = format(clean_value, ".17g")
            row[f"{field}_injected"] = format(injected_value, ".17g")
            row[f"delta_{field}"] = format(injected_value - clean_value, ".17g")
        paired_rows.append(row)
        selected.append((c, i, row))
    if len([row for row in paired_rows if row["obs_id"] in planned_ids]) != 30:
        raise PrecheckError("TARGET_PAIRED_OBSERVATION_COUNT_MISMATCH")
    write_csv(
        root / "pl_clean_injected_paired.csv",
        ["timestamp", "group_id", "keyframe_id", "tag_id", "anchor_id",
         "obs_id", "nu_m_clean", "nu_m_injected", "delta_nu_m",
         "marginal_z_clean", "marginal_z_injected", "delta_marginal_z",
         "conditional_z_clean", "conditional_z_injected",
         "delta_conditional_z"], paired_rows)

    target = [(c, i, row) for c, i, row in selected
              if row["obs_id"] in planned_ids and
              int(row["anchor_id"]) == spec["anchor_id"]]
    if len(target) != 30:
        raise PrecheckError("TARGET_ANCHOR_PAIRING_MISMATCH")
    target.sort(key=lambda item: int(item[2]["keyframe_id"]))
    target_clean_z = [numeric(c, "conditional_z") for c, _, _ in target]
    target_injected_z = [numeric(i, "conditional_z") for _, i, _ in target]
    target_delta_z = [i - c for c, i in zip(target_clean_z,
                                             target_injected_z)]

    healthy_by_anchor = {}
    for anchor in sorted({int(row["anchor_id"]) for row in paired_rows
                          if int(row["anchor_id"]) != spec["anchor_id"]}):
        rows = [(c, i) for c, i, row in selected
                if int(row["anchor_id"]) == anchor]
        clean_z = [numeric(c, "conditional_z") for c, _ in rows]
        injected_z = [numeric(i, "conditional_z") for _, i in rows]
        delta_z = [i - c for c, i in zip(clean_z, injected_z)]
        healthy_by_anchor[str(anchor)] = {
            "clean": describe(clean_z),
            "injected": describe(injected_z),
            "paired_delta": describe(delta_z),
        }

    cumulative_rows = []
    cumulative_clean = cumulative_injected = cumulative_delta = 0.0
    for index, ((_, _, row), clean_z, injected_z, delta_z) in enumerate(
            zip(target, target_clean_z, target_injected_z, target_delta_z), 1):
        cumulative_clean += clean_z
        cumulative_injected += injected_z
        cumulative_delta += delta_z
        cumulative_rows.append({
            "epoch": index,
            "timestamp": row["timestamp"],
            "group_id": row["group_id"],
            "keyframe_id": row["keyframe_id"],
            "obs_id": row["obs_id"],
            "conditional_z_clean": format(clean_z, ".17g"),
            "conditional_z_injected": format(injected_z, ".17g"),
            "delta_conditional_z": format(delta_z, ".17g"),
            "cumulative_signed_z_clean": format(cumulative_clean, ".17g"),
            "cumulative_signed_z_injected":
                format(cumulative_injected, ".17g"),
            "cumulative_signed_delta_z": format(cumulative_delta, ".17g"),
            "normalized_cumulative_z_clean":
                format(cumulative_clean / math.sqrt(index), ".17g"),
            "normalized_cumulative_z_injected":
                format(cumulative_injected / math.sqrt(index), ".17g"),
            "normalized_cumulative_delta_z":
                format(cumulative_delta / math.sqrt(index), ".17g"),
        })
    write_csv(root / "pl_persistent_cumulative.csv",
              list(cumulative_rows[0]), cumulative_rows)

    target_injected_rows = [i for _, i, _ in target]
    target_additive = sum(numeric(row, "additive_group_contribution")
                          for row in target_injected_rows)
    healthy_rows = [i for _, i, row in selected
                    if int(row["anchor_id"]) != spec["anchor_id"]]
    healthy_additive = sum(numeric(row, "additive_group_contribution")
                           for row in healthy_rows)
    total_group_t = sum(float(row["statistic"]) for row in affected_epochs)
    if not close(total_group_t, target_additive + healthy_additive):
        raise PrecheckError("GROUP_QUADRATIC_CONTRIBUTION_SUM_MISMATCH")
    decomposition = {
        "target_nu_m": describe(
            [numeric(row, "nu_m") for row in target_injected_rows]),
        "target_abs_nu_m": describe(
            [abs(numeric(row, "nu_m")) for row in target_injected_rows]),
        "target_measurement_variance_m2_mean": mean(
            [numeric(row, "R_mm") for row in target_injected_rows]),
        "target_prior_projected_variance_m2_mean": mean(
            [numeric(row, "prior_projected_variance_m")
             for row in target_injected_rows]),
        "target_total_variance_m2_mean": mean(
            [numeric(row, "S_mm") for row in target_injected_rows]),
        "target_measurement_fraction_of_S_mean": mean(
            [numeric(row, "R_mm") / numeric(row, "S_mm")
             for row in target_injected_rows]),
        "target_prior_fraction_of_S_mean": mean(
            [numeric(row, "prior_projected_variance_m") /
             numeric(row, "S_mm") for row in target_injected_rows]),
        "target_conditional_to_marginal_variance_ratio_mean": mean(
            [numeric(row, "conditional_variance_m2") /
             numeric(row, "S_mm") for row in target_injected_rows]),
        "affected_group_T_sum": total_group_t,
        "target_additive_group_contribution_sum": target_additive,
        "healthy_additive_group_contribution_sum": healthy_additive,
        "target_additive_fraction_of_T_sum": target_additive / total_group_t,
        "target_conditional_increment_sum": sum(
            numeric(row, "conditional_quadratic_increment")
            for row in target_injected_rows),
        "healthy_conditional_increment_sum": sum(
            numeric(row, "conditional_quadratic_increment")
            for row in healthy_rows),
    }

    max_group_id = f"pl-group-{max_epoch['keyframe_id']}"
    max_group_rows = [i for _, i, row in selected
                      if row["group_id"] == max_group_id]
    max_group_target = [row for row in max_group_rows
                        if int(row["anchor_id"]) == spec["anchor_id"]]
    if len(max_group_target) != 1:
        raise PrecheckError("MAX_GROUP_TARGET_ROW_MISMATCH")
    max_target = max_group_target[0]
    max_group_healthy = [row for row in max_group_rows
                         if int(row["anchor_id"]) != spec["anchor_id"]]
    decomposition["max_T_group"] = {
        "group_id": max_group_id,
        "keyframe_id": int(max_epoch["keyframe_id"]),
        "dof": int(max_epoch["dof"]),
        "T": max_t,
        "threshold_pfa_1e5": float(max_epoch["threshold"]),
        "target_nu_m": numeric(max_target, "nu_m"),
        "target_R_mm": numeric(max_target, "R_mm"),
        "target_prior_projected_variance_m":
            numeric(max_target, "prior_projected_variance_m"),
        "target_S_mm": numeric(max_target, "S_mm"),
        "target_marginal_z": numeric(max_target, "marginal_z"),
        "target_conditional_nu_m": numeric(max_target, "conditional_nu_m"),
        "target_conditional_variance_m2":
            numeric(max_target, "conditional_variance_m2"),
        "target_conditional_z": numeric(max_target, "conditional_z"),
        "target_additive_group_contribution":
            numeric(max_target, "additive_group_contribution"),
        "healthy_additive_group_contribution_sum": sum(
            numeric(row, "additive_group_contribution")
            for row in max_group_healthy),
        "target_conditional_quadratic_increment":
            numeric(max_target, "conditional_quadratic_increment"),
        "healthy_conditional_quadratic_increment_sum": sum(
            numeric(row, "conditional_quadratic_increment")
            for row in max_group_healthy),
    }

    target_clean_nu = [numeric(c, "nu_m") for c, _, _ in target]
    target_injected_nu = [numeric(i, "nu_m") for _, i, _ in target]
    target_clean_marginal = [numeric(c, "marginal_z") for c, _, _ in target]
    target_injected_marginal = [numeric(i, "marginal_z")
                                for _, i, _ in target]
    target_summary = {
        "clean": describe(target_clean_z),
        "injected": describe(target_injected_z),
        "paired_delta": describe(target_delta_z),
    }
    target_components = {
        "nu_m": {
            "clean": describe(target_clean_nu),
            "injected": describe(target_injected_nu),
            "paired_delta": describe(
                [i - c for c, i in zip(target_clean_nu, target_injected_nu)]),
        },
        "marginal_z": {
            "clean": describe(target_clean_marginal),
            "injected": describe(target_injected_marginal),
            "paired_delta": describe([
                i - c for c, i in zip(target_clean_marginal,
                                      target_injected_marginal)]),
        },
        "conditional_z": target_summary,
    }
    healthy_delta_max = max(
        abs(item["paired_delta"]["Z_sum"])
        for item in healthy_by_anchor.values())
    healthy_positive_delta_max = max(
        0.0,
        max(item["paired_delta"]["Z_sum"]
            for item in healthy_by_anchor.values()))
    a_qualifying = [
        row for row in sweep_rows
        if float(row["p_fa"]) <= 0.05 and
        float(row["clean_empirical_alarm_fraction"]) <= 0.05 and
        float(row["affected_recall"]) >= 0.5
    ]
    b_signal = (
        target_summary["paired_delta"]["Z_sum"] >= 3.0 and
        target_summary["injected"]["nlos_direction_positive_fraction"] >= 0.70 and
        target_summary["paired_delta"]["Z_sum"] >=
        2.0 * max(1.0, healthy_positive_delta_max)
    )
    if a_qualifying:
        verdict = "GROUP_THRESHOLD_MISCONFIGURATION_DOMINANT"
        verdict_reason = "preregistered reasonable group-sweep criterion passed"
    elif b_signal:
        verdict = (
            "GROUP_STATISTIC_TASK_MISMATCH_"
            "PERSISTENT_PER_ANCHOR_SIGNAL_PRESENT")
        verdict_reason = "group sweep weak; preregistered target-specific signal criterion passed"
    else:
        verdict = "CONDITIONAL_INNOVATION_SIGNAL_NOT_SEPARABLE"
        verdict_reason = "neither preregistered group-threshold nor target-specific signal criterion passed"

    summary = {
        "schema": "uifgo_pl_threshold_signal_audit_v1",
        "verdict": verdict,
        "verdict_reason": verdict_reason,
        "truth_read_phase": "AFTER_PRODUCTION_STYLE_STATISTICS_SEAL",
        "truth_sha256": f"sha256:{sha256(args.truth)}",
        "statistics_seal_sha256":
            f"sha256:{sha256(root / 'sealed_statistics_hashes.sha256')}",
        "sealed_file_count": len(sealed_files),
        "source_preflight_seal_sha256":
            f"sha256:{SOURCE_PREFLIGHT_SEAL_SHA256}",
        "precheck": {
            "status": "PASS",
            "clean_group_count": len(clean_epochs),
            "clean_alarm_count": 0,
            "affected_group_count": len(affected_epochs),
            "affected_obs_count": len(covered),
            "affected_alarm_count": 0,
            "max_affected_T": max_t,
            "max_affected_keyframe": int(max_epoch["keyframe_id"]),
            "max_affected_dof": int(max_epoch["dof"]),
            "max_affected_threshold_pfa_1e5": float(max_epoch["threshold"]),
            "max_affected_upper_tail_probability":
                float(chi2.sf(max_t, int(max_epoch["dof"]))),
            "detector_forbidden_open_count":
                sum(len(hits) for hits in trace_hits.values()),
        },
        "sign_convention": {
            "factor_residual": "h+beta-z",
            "physical_innovation": "nu=-factor_residual=z-h-beta",
            "positive_excess_range_direction": "positive_nu_and_positive_z",
            "signed_evidence_multiplier_s": 1,
        },
        "threshold_sweep": sweep_rows,
        "target_conditional_z": target_summary,
        "target_paired_components": target_components,
        "healthy_conditional_z_by_anchor": healthy_by_anchor,
        "healthy_max_absolute_paired_delta_Z_sum": healthy_delta_max,
        "healthy_max_positive_paired_delta_Z_sum": healthy_positive_delta_max,
        "decomposition": decomposition,
        "verdict_criteria": {
            "A_qualifying_pfa_rows": [row["p_fa"] for row in a_qualifying],
            "B_target_paired_delta_Z_sum_ge_3":
                target_summary["paired_delta"]["Z_sum"] >= 3.0,
            "B_target_injected_positive_fraction_ge_0_70":
                target_summary["injected"][
                    "nlos_direction_positive_fraction"] >= 0.70,
            "B_target_delta_at_least_twice_healthy_or_one":
                target_summary["paired_delta"]["Z_sum"] >=
                2.0 * max(1.0, healthy_positive_delta_max),
        },
        "production_detector_modified": False,
        "support_created": False,
        "stage2": "NOT_RUN_DIAGNOSTIC_ONLY",
        "recovery_e2e": "NOT_RUN_DIAGNOSTIC_ONLY",
        "next_detector": "NOT_IMPLEMENTED",
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit-root", required=True, type=Path)
    parser.add_argument("--source-preflight-root", required=True, type=Path)
    parser.add_argument("--truth", required=True, type=Path)
    parser.add_argument("--locked-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError("refusing to overwrite evaluator output")
    try:
        summary = evaluate(args)
        exit_code = 0
    except PrecheckError as error:
        summary = {
            "schema": "uifgo_pl_threshold_signal_audit_v1",
            "verdict": f"PRECHECK_INVALID:{error}",
            "production_detector_modified": False,
            "support_created": False,
            "stage2": "NOT_RUN_PRECHECK_INVALID",
            "recovery_e2e": "NOT_RUN_PRECHECK_INVALID",
            "next_detector": "NOT_IMPLEMENTED",
        }
        exit_code = 3
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(summary["verdict"])
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
