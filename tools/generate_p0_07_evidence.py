#!/usr/bin/env python3
"""Run the frozen P0-07 replay and mechanically emit/verify its evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile


ATTEMPT_HEADER = [
    "attempt", "terminal_state", "detector_pass", "hypotheses",
    "generated_actions", "kernel_evaluated_actions", "coverage_status",
    "risk_all_validated", "risk_ledger_closes", "base_svd",
    "core_compute_ms", "analysis_completion_ms", "arrival_to_packet_ready_ms",
    "deadline_miss", "complete_work", "rss_kib",
]

ACTION_HEADER = [
    "action_id", "action_type", "removed_group_ids",
    "added_group_ids", "terminal", "kernel_evaluated", "valid",
    "post_detector_passed", "covers_plausible_set", "model_error_validated",
    "rank", "dof", "risk_allocation", "statistic", "threshold",
    "hpl_m", "vpl_m", "selected",
    "occurrence_identity_hash", "reason_hash",
]

BUNDLE_NAMES = (
    "alarm-actions.tsv", "evidence-command.txt", "frozen-input.json",
    "frozen-output.json", "frozen-truth.json", "input-attempts.tsv",
    "o12-correctness-smoke.json",
)


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def timing_summary(rows: list[dict[str, str]], field: str) -> dict[str, float]:
    values = [float(row[field]) for row in rows]
    return {
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values),
    }


def parse_run(stdout: str) -> tuple[list[dict[str, str]], list[list[str]],
                                    list[dict[str, str]], dict[str, str]]:
    attempts: list[dict[str, str]] = []
    raw_uwb: list[list[str]] = []
    actions: list[dict[str, str]] = []
    contract: dict[str, str] = {}
    for line in stdout.splitlines():
        if line.startswith("[P0-07-ATTEMPT]\t"):
            values = line.split("\t")[1:]
            if len(values) != len(ATTEMPT_HEADER):
                raise RuntimeError(f"attempt field count {len(values)} != {len(ATTEMPT_HEADER)}")
            attempts.append(dict(zip(ATTEMPT_HEADER, values)))
        elif line.startswith("[P0-07-RAW-UWB]\t"):
            raw_uwb.append(line.split("\t")[1:])
        elif line.startswith("[P0-07-ACTION]\t"):
            values = line.split("\t")[1:]
            if len(values) != len(ACTION_HEADER):
                raise RuntimeError(
                    f"action field count {len(values)} != {len(ACTION_HEADER)}")
            actions.append(dict(zip(ACTION_HEADER, values)))
        elif line.startswith("[P0-07-CONTRACT]\t"):
            contract = dict(item.split("=", 1) for item in line.split("\t")[1:])
    if len(attempts) != 24:
        raise RuntimeError(f"expected 24 attempts, observed {len(attempts)}")
    if len(raw_uwb) != 24 * 8:
        raise RuntimeError(f"expected 192 raw UWB rows, observed {len(raw_uwb)}")
    if contract.get("alarm_epoch") != "23":
        raise RuntimeError("missing epoch-23 production alarm contract")
    if len(actions) != int(contract.get("generated", "-1")):
        raise RuntimeError("per-action terminal journal does not match census")
    return attempts, raw_uwb, actions, contract


def write_json(path: pathlib.Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def derive(attempts: list[dict[str, str]], raw_uwb: list[list[str]],
           actions: list[dict[str, str]],
           contract: dict[str, str], input_snapshot: dict[str, object],
           reference: dict[str, object]) -> dict[str, object]:
    deadline_misses = sum(int(row["deadline_miss"]) for row in attempts)
    complete = sum(int(row["complete_work"]) for row in attempts)
    output_snapshot = {
        "schema": "p0-07-production-output-v3",
        "alarm": contract,
        "alarm_actions": actions,
        "terminal_attempt": attempts[-1],
        "all_attempts_retained": len(attempts),
        "formal_eligible": False,
        "protected_output": False,
    }
    summary = {
        "schema": "p0-07-o12-correctness-smoke-v2",
        "claim": "correctness smoke only; not P1 performance acceptance",
        "denominator": {"received": 24, "attempted": 24, "excluded": 0},
        "timing_boundaries": {
            "core_compute": "production core_total stage sampled before publication gate return",
            "analysis_completion": "test-driver processUwbBatch return sample",
            "arrival_to_packet_ready": "test-driver packet-ready/final immutable packet sample; no publisher was installed",
            "publish_outcome": "NOT_ATTEMPTED (no transport installed)",
        },
        "core_compute_ms": timing_summary(attempts, "core_compute_ms"),
        "analysis_completion_ms": timing_summary(attempts, "analysis_completion_ms"),
        "arrival_to_packet_ready_ms": timing_summary(attempts, "arrival_to_packet_ready_ms"),
        "deadline_misses": deadline_misses,
        "deadline_miss_rate": deadline_misses / 24.0,
        "complete_work_attempts": complete,
        "complete_work_rate": complete / 24.0,
        "rss_peak_kib": max(int(row["rss_kib"]) for row in attempts),
    }
    return {"input": input_snapshot, "truth": reference,
            "output": output_snapshot, "summary": summary}


def validate_derived(derived: dict[str, object], attempts: list[dict[str, str]],
                     raw_uwb: list[list[str]], actions: list[dict[str, str]],
                     reference: dict[str, object]) -> None:
    alarm = attempts[22]
    if alarm["detector_pass"] != "0" or int(alarm["generated_actions"]) <= 1:
        raise RuntimeError("real alarm/multi-action production contract absent")
    if len(actions) <= 1 or any(row["kernel_evaluated"] != "1" for row in actions):
        raise RuntimeError("every uncapped generated action was not actually evaluated")
    if any(row["terminal"] == "PLANNED_NOT_RUN" for row in actions):
        raise RuntimeError("non-terminal action occurrence in frozen evidence")
    if any(float(row["core_compute_ms"]) > float(row["analysis_completion_ms"])
           or float(row["analysis_completion_ms"]) > float(row["arrival_to_packet_ready_ms"])
           for row in attempts):
        raise RuntimeError("timing boundary ordering is invalid")
    if derived["summary"]["denominator"]["attempted"] != 24:  # type: ignore[index]
        raise RuntimeError("O12 denominator changed")
    expected_rows = derived["input"]["uwb_rows"]  # type: ignore[index]
    observed_rows = [
        {"epoch": int(row[0]), "measurement_id": int(row[1]),
         "anchor_id": int(row[2]), "timestamp_ns": int(row[3]),
         "range_m": float(row[4]), "sigma_m": float(row[5]),
         "anchor_position_m": [float(row[6]), float(row[7]), float(row[8])]}
        for row in raw_uwb
    ]
    if observed_rows != expected_rows:
        raise RuntimeError("binary raw replay differs from versioned frozen input")
    # The frozen reference is a regression observation only.  In particular,
    # do not turn its plausible/action/terminal/winner values into an oracle:
    # the test executable has already derived and checked those values from the
    # recipe, frozen factors and raw measurements before emitting this stream.
    if int(derived["output"]["alarm"]["generated"]) != len(actions):  # type: ignore[index]
        raise RuntimeError("generated action census is internally inconsistent")
    if int(derived["output"]["alarm"]["kernel_evaluated"]) != len(actions):  # type: ignore[index]
        raise RuntimeError("kernel-evaluated action census is internally inconsistent")
    action_ids = [int(row["action_id"]) for row in actions]
    if action_ids != list(range(1, len(actions) + 1)):
        raise RuntimeError("action occurrence identities are not unique and sequential")
    risk_values = [float(row["risk_allocation"]) for row in actions]
    if not risk_values or any(value <= 0.0 for value in risk_values):
        raise RuntimeError("action risk allocation is absent or non-positive")
    if any(not math.isclose(value, risk_values[0], rel_tol=1e-14,
                            abs_tol=1e-18) for value in risk_values[1:]):
        raise RuntimeError("per-occurrence action risk allocation is inconsistent")
    if derived["output"]["alarm"]["exception_publish_outcome"] != "NOT_ATTEMPTED":  # type: ignore[index]
        raise RuntimeError("unavailable publisher was not recorded honestly")


def load_json(path: pathlib.Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"{path} is not a JSON object")
    return value


def fsync_directory(path: pathlib.Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def resolve_authority(authority: pathlib.Path) -> pathlib.Path:
    """Resolve one atomic pointer; legacy flat bundles remain comparison-only."""
    pointer = authority / "current.json"
    if not pointer.exists():
        return authority
    value = load_json(pointer)
    if value.get("schema") != "p0-07-bundle-pointer-v1":
        raise RuntimeError("unsupported evidence authority pointer")
    digest = str(value.get("bundle_sha256", ""))
    if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        raise RuntimeError("invalid evidence authority digest")
    bundle = authority / "bundles" / digest
    manifest = load_json(bundle / "bundle-hashes.json")
    if hashlib.sha256((bundle / "bundle-hashes.json").read_bytes()).hexdigest() != digest:
        raise RuntimeError("authority identity does not match its hash manifest")
    if set(manifest) != set(BUNDLE_NAMES):
        raise RuntimeError("authority bundle manifest has missing/extra files")
    for name, wanted in manifest.items():
        if pathlib.PurePath(name).name != name or not isinstance(wanted, str):
            raise RuntimeError("invalid authority bundle manifest entry")
        if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != wanted:
            raise RuntimeError(f"authority bundle hash mismatch: {name}")
    return bundle


def validate_immutable(bundle: pathlib.Path, manifest_text: str,
                       files: dict[str, str]) -> None:
    """Validate identity, inventory, hashes and intended semantic bytes."""
    if bundle.name != hashlib.sha256(manifest_text.encode("utf-8")).hexdigest():
        raise RuntimeError("immutable bundle path has the wrong identity")
    manifest_path = bundle / "bundle-hashes.json"
    if manifest_path.read_text(encoding="utf-8") != manifest_text:
        raise RuntimeError("existing immutable manifest differs from identity")
    actual_names = {item.name for item in bundle.iterdir() if item.is_file()}
    if actual_names != set(files) | {"bundle-hashes.json"}:
        raise RuntimeError("existing immutable bundle inventory differs")
    manifest = json.loads(manifest_text)
    for name, content in files.items():
        raw = (bundle / name).read_bytes()
        expected = content.encode("utf-8")
        if raw != expected or hashlib.sha256(raw).hexdigest() != manifest[name]:
            raise RuntimeError(f"existing immutable semantic bytes differ: {name}")


def write_bundle_atomic(authority: pathlib.Path, files: dict[str, str],
                        inject_at: str = "") -> None:
    """Commit immutable content first, then replace only a tiny current pointer.

    At every pre-commit interruption an existing current.json remains present
    and readable. There is no old-directory rename and hence no absent-authority
    crash window.
    """
    if inject_at == "validation":
        raise RuntimeError("injected validation failure")
    authority.mkdir(parents=True, exist_ok=True)
    bundles = authority / "bundles"
    bundles.mkdir(exist_ok=True)
    stage = pathlib.Path(tempfile.mkdtemp(prefix=".stage-", dir=bundles))
    try:
        for index, (name, content) in enumerate(sorted(files.items())):
            target = stage / name
            target.write_text(content, encoding="utf-8")
            if inject_at == "write" and index == 1:
                raise RuntimeError("injected bundle write failure")
            with target.open("rb") as stream:
                os.fsync(stream.fileno())
        manifest = {
            name: hashlib.sha256(content.encode("utf-8")).hexdigest()
            for name, content in sorted(files.items())
        }
        manifest_text = json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        (stage / "bundle-hashes.json").write_text(manifest_text, encoding="utf-8")
        with (stage / "bundle-hashes.json").open("rb") as stream:
            os.fsync(stream.fileno())
        if inject_at == "flush":
            raise RuntimeError("injected pre-directory-fsync failure")
        fsync_directory(stage)
        identity = hashlib.sha256(manifest_text.encode("utf-8")).hexdigest()
        immutable = bundles / identity
        if immutable.exists():
            # Never trust a directory merely because its name is the desired
            # identity. Validate its manifest, exact inventory, hashes and
            # intended semantic bytes before it can become current.
            validate_immutable(immutable, manifest_text, files)
            for child in stage.iterdir():
                child.unlink()
            stage.rmdir()
        else:
            try:
                os.replace(stage, immutable)
            except OSError:
                # A same-content writer may have won the directory race.
                # Only accept that outcome after the same full validation.
                if not immutable.exists():
                    raise
                validate_immutable(immutable, manifest_text, files)
                for child in stage.iterdir():
                    child.unlink()
                stage.rmdir()
            fsync_directory(bundles)
        if inject_at == "pre_pointer":
            raise RuntimeError("injected failure before pointer creation")
        pointer_value = {
            "schema": "p0-07-bundle-pointer-v1",
            "bundle_sha256": identity,
            "relative_path": f"bundles/{identity}",
        }
        pointer_text = json.dumps(pointer_value, indent=2, sort_keys=True) + "\n"
        pointer_fd, pointer_name = tempfile.mkstemp(
            prefix=".current-", suffix=".tmp", dir=authority)
        temporary_pointer = pathlib.Path(pointer_name)
        with os.fdopen(pointer_fd, "w", encoding="utf-8") as stream:
            stream.write(pointer_text)
            stream.flush()
            os.fsync(stream.fileno())
        if inject_at == "pointer_rename":
            temporary_pointer.unlink()
            raise RuntimeError("injected pointer rename failure")
        os.replace(temporary_pointer, authority / "current.json")
        fsync_directory(authority)
        if inject_at == "post_commit_cleanup":
            raise RuntimeError("injected post-commit cleanup failure")
    finally:
        if stage.exists():
            for child in stage.iterdir():
                child.unlink()
            stage.rmdir()


def compare_non_timing(bundle: dict[str, object], canonical: pathlib.Path) -> None:
    canonical = resolve_authority(canonical)
    for name in ("frozen-input.json", "frozen-truth.json", "alarm-actions.tsv"):
        generated = bundle[name]
        expected = (canonical / name).read_text(encoding="utf-8")
        if generated != expected:
            raise RuntimeError(f"fresh {name} differs from frozen evidence")
    fresh_output = json.loads(str(bundle["frozen-output.json"]))
    frozen_output = load_json(canonical / "frozen-output.json")
    for value in (fresh_output, frozen_output):
        value["alarm"].pop("terminal_timing_digest", None)
        terminal = value["terminal_attempt"]
        for field in ("core_compute_ms", "analysis_completion_ms",
                      "arrival_to_packet_ready_ms", "rss_kib"):
            terminal.pop(field, None)
    if fresh_output != frozen_output:
        raise RuntimeError("fresh semantic output differs from frozen evidence")
    def stable_attempts(text_value: str) -> list[dict[str, str]]:
        lines = text_value.splitlines()
        header = lines[0].split("\t")
        result = []
        for line in lines[1:]:
            row = dict(zip(header, line.split("\t")))
            for field in ("core_compute_ms", "analysis_completion_ms",
                          "arrival_to_packet_ready_ms", "rss_kib"):
                row.pop(field, None)
            result.append(row)
        return result
    if stable_attempts(str(bundle["input-attempts.tsv"])) != stable_attempts(
            (canonical / "input-attempts.tsv").read_text(encoding="utf-8")):
        raise RuntimeError("fresh non-timing attempt journal differs from frozen evidence")
    fresh_summary = json.loads(str(bundle["o12-correctness-smoke.json"]))
    frozen_summary = load_json(canonical / "o12-correctness-smoke.json")
    for value in (fresh_summary, frozen_summary):
        for field in ("core_compute_ms", "analysis_completion_ms",
                      "arrival_to_packet_ready_ms", "rss_peak_kib"):
            value.pop(field, None)
    if fresh_summary != frozen_summary:
        raise RuntimeError("fresh non-timing O12 summary differs from frozen evidence")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--compare-dir")
    parser.add_argument("--inject-failure-before-swap", action="store_true")
    parser.add_argument("--inject-at", choices=("validation", "write", "flush",
                        "pre_pointer", "pointer_rename", "post_commit_cleanup"),
                        default="")
    args = parser.parse_args()
    command = [args.binary,
               "--gtest_filter=P007CorrectedExhaustive.SameRawReplayDrivesProductionBuilderGeneratorAndFinalRefusal"]
    run = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, check=False)
    sys.stdout.write(run.stdout)
    if run.returncode != 0:
        return run.returncode
    attempts, raw_uwb, actions, contract = parse_run(run.stdout)
    input_snapshot = load_json(pathlib.Path(args.input))
    reference = load_json(pathlib.Path(args.reference))
    if input_snapshot.get("schema") != "p0-07-production-raw-replay-v3":
        raise RuntimeError("unsupported frozen input schema")
    if reference.get("schema") != "p0-07-regression-observation-v4":
        raise RuntimeError("unsupported regression observation schema")
    derived = derive(attempts, raw_uwb, actions, contract,
                     input_snapshot, reference)
    validate_derived(derived, attempts, raw_uwb, actions, reference)
    output_dir = pathlib.Path(args.output_dir)
    json_text = lambda value: json.dumps(value, indent=2, sort_keys=True) + "\n"
    files = {
        "frozen-input.json": json_text(derived["input"]),
        "frozen-truth.json": json_text(derived["truth"]),
        "frozen-output.json": json_text(derived["output"]),
        "o12-correctness-smoke.json": json_text(derived["summary"]),
        "input-attempts.tsv": "\t".join(ATTEMPT_HEADER) + "\n" + "".join(
            "\t".join(row[field] for field in ATTEMPT_HEADER) + "\n"
            for row in attempts),
        "alarm-actions.tsv": "\t".join(ACTION_HEADER) + "\n" + "".join(
            "\t".join(row[field] for field in ACTION_HEADER) + "\n"
            for row in actions),
        "evidence-command.txt": (
            "command=" + " ".join(command) + "\nexit=0\n"
            "generator=tools/generate_p0_07_evidence.py\n"
            f"input={args.input}\nreference={args.reference}\n"),
    }
    if args.compare_dir:
        compare_non_timing(files, pathlib.Path(args.compare_dir))
    inject_at = args.inject_at or (
        "pre_pointer" if args.inject_failure_before_swap else "")
    write_bundle_atomic(output_dir, files, inject_at)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
