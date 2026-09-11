#!/usr/bin/env python3
"""Truth-side evaluator for the sealed PL conditional RAIM preflight."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


LOCKED_MANIFEST_SHA256 = (
    "1660f7a1e8d1bd32bfabda394adf9e69675fb849917b028b66fff7ec0e87e820"
)
TARGET_PL_COMMIT = "ae54fb8ca55dfbfaf64fe45615b6bcd106548a93"


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
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"SEALED_HASH_MISMATCH:{path}")
        checked.append(str(path))
    return checked


def trace_forbidden_reads(path: Path):
    forbidden = ("/truth/", "/ground_truth", "/oracle", "/mocap", "/data/")
    hits = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        lower = line.lower()
        if " = -1 " in lower:
            continue
        if any(token in lower for token in forbidden):
            hits.append(line)
    return hits


def as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preflight-root", required=True, type=Path)
    parser.add_argument("--truth", required=True, type=Path)
    parser.add_argument("--locked-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    root = args.preflight_root.resolve()
    if args.output.exists():
        raise RuntimeError("refusing to overwrite evaluator output")
    manifest_hash = sha256(args.locked_manifest)
    if manifest_hash != LOCKED_MANIFEST_SHA256:
        raise RuntimeError("LOCKED_MANIFEST_HASH_MISMATCH")
    checked = verify_seal(root, root / "sealed_detector_hashes.sha256")

    clean_status = load_json(root / "clean/pl_conditional_status.json")
    injected_status = load_json(root / "injected/pl_conditional_status.json")
    for label, status in (("clean", clean_status), ("injected", injected_status)):
        if status.get("status") != "SUCCESS":
            raise RuntimeError(f"{label.upper()}_DETECTOR_NOT_SUCCESS")
        if status.get("pl_source_commit") != TARGET_PL_COMMIT:
            raise RuntimeError(f"{label.upper()}_PL_COMMIT_MISMATCH")
        if status.get("gt_read") is not False or status.get("oracle_read") is not False:
            raise RuntimeError(f"{label.upper()}_TRUTH_BLIND_STATUS_INVALID")

    trace_hits = {}
    for label in ("clean", "injected"):
        hits = trace_forbidden_reads(root / f"{label}_file_access.trace")
        trace_hits[label] = hits
        if hits:
            raise RuntimeError(f"{label.upper()}_FORBIDDEN_FILE_OPEN")

    truth = load_json(args.truth)
    spec = truth["specification"]
    planned_ids = {int(value) for value in truth["planned_affected_obs_ids"]}
    if (truth.get("planned_status") != "ACTUAL_PLAN" or len(planned_ids) != 30 or
            spec != {
                "amplitude_m": 0.5,
                "anchor_id": 20276,
                "anchor_subset": [7475, 9524, 10548, 15155, 20276],
                "enabled": True,
                "end_s": 1664959686.3077347,
                "interval": "closed",
                "seed": 911,
                "shape": "constant_step",
                "start_s": 1664959678.3077347,
                "tag_id": 27956,
                "version": "NLOS_INJECTION_V1",
            }):
        raise RuntimeError("LOCKED_TRUTH_SPECIFICATION_MISMATCH")

    clean_support = load_json(root / "clean/pl_conditional_support.json")
    injected_support = load_json(root / "injected/pl_conditional_support.json")
    epochs = load_csv(root / "injected/pl_conditional_epochs.csv")
    affected_epochs = []
    covered_ids = set()
    for row in epochs:
        ids = {int(value) for value in row["obs_ids"].split(";") if value}
        overlap = ids & planned_ids
        if overlap:
            affected_epochs.append(row)
            covered_ids.update(overlap)

    affected_alarm = [row for row in affected_epochs if as_bool(row["alarm"])]
    target_unique = [
        row for row in affected_epochs
        if row["outcome"] in (
            "UNIQUE_ISOLATION_POSITIVE", "UNIQUE_ISOLATION_NONPOSITIVE")
        and int(row["isolated_anchor_id"]) == spec["anchor_id"]
    ]
    overlap_segments = []
    for segment in injected_support["segments"]:
        ids = {int(value) for value in segment["obs_ids"]}
        time_overlap = not (
            float(segment["end_time"]) < spec["start_s"] or
            float(segment["start_time"]) > spec["end_s"]
        )
        if (int(segment["tag_id"]) == spec["tag_id"] and
                int(segment["anchor_id"]) == spec["anchor_id"] and
                (bool(ids & planned_ids) or time_overlap)):
            overlap_segments.append(segment["segment_id"])

    gate1 = len(clean_support["segments"]) == 0
    gate2 = gate1 and bool(affected_alarm)
    gate3 = gate2 and bool(target_unique)
    gate4 = gate3 and bool(overlap_segments)
    if not gate1:
        verdict = "PL_CONDITIONAL_PREFLIGHT_FAIL_CLEAN_SUPPORT"
    elif not gate2:
        verdict = "PL_CONDITIONAL_PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM"
    elif not gate3:
        verdict = "PL_CONDITIONAL_PREFLIGHT_FAIL_NO_TARGET_UNIQUE_ISOLATION"
    elif not gate4:
        verdict = "PL_CONDITIONAL_PREFLIGHT_FAIL_NO_PERSISTENT_TARGET_SUPPORT"
    else:
        verdict = "PL_CONDITIONAL_PREFLIGHT_PASS"

    def gate(status, passed, count):
        return {"status": status, "passed": passed, "observed_count": count}

    result = {
        "schema": "uifgo_pl_conditional_preflight_evaluation_v1",
        "verdict": verdict,
        "locked_manifest_sha256": f"sha256:{manifest_hash}",
        "sealed_detector_manifest_sha256":
            f"sha256:{sha256(root / 'sealed_detector_hashes.sha256')}",
        "sealed_file_count": len(checked),
        "truth_sha256": f"sha256:{sha256(args.truth)}",
        "truth_read_phase": "AFTER_DETECTOR_ARTIFACT_SEAL",
        "detector_forbidden_open_count": sum(map(len, trace_hits.values())),
        "planned_affected_id_count": len(planned_ids),
        "affected_id_covered_count": len(covered_ids),
        "affected_group_count": len(affected_epochs),
        "diagnostics": {
            "affected_alarm_group_count": len(affected_alarm),
            "affected_target_unique_isolation_count": len(target_unique),
            "injected_retained_segment_count": len(injected_support["segments"]),
            "target_overlap_segment_count": len(overlap_segments),
        },
        "gates": {
            "1_clean_zero_support": gate(
                "PASS" if gate1 else "FAIL", gate1,
                len(clean_support["segments"])),
            "2_affected_group_alarm": gate(
                "PASS" if gate2 else "FAIL", gate2,
                len(affected_alarm)),
            "3_target_unique_isolation": gate(
                ("PASS" if gate3 else
                 "NOT_RUN_PREVIOUS_GATE_FAILED" if not gate2 else "FAIL"),
                gate3 if gate2 else None, len(target_unique)),
            "4_persistent_target_overlap": gate(
                ("PASS" if gate4 else
                 "NOT_RUN_PREVIOUS_GATE_FAILED" if not gate3 else "FAIL"),
                gate4 if gate3 else None, len(overlap_segments)),
        },
        "production": "NOT_RUN_PREFLIGHT_FAILED" if not gate4 else "ADMITTED",
        "e2e": "NOT_RUN_PREFLIGHT_FAILED" if not gate4 else "ADMITTED",
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(verdict)
    return 0 if gate4 else 2


if __name__ == "__main__":
    raise SystemExit(main())
