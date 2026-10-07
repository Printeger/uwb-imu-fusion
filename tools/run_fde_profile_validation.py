#!/usr/bin/env python3
"""Run the frozen five-profile validation protocol with resumable identities."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import yaml

from validate_simulation_calibrations import validate as validate_simulation_calibrations


ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("off", "uwb_order1", "imu_order1", "joint_order1", "joint_order2")
FAULT_OVERRIDE_NAMES = (
    "UWB_IMU_PL_UWB_BIAS_M",
    "UWB_IMU_PL_ACCEL_BIAS_MPS2",
    "UWB_IMU_PL_GYRO_BIAS_RADPS",
)
COMPLETION_FIELDS = frozenset(("exit_code", "runner_wall_s",
                               "terminal_artifacts"))
SOURCE_IDENTITY_ROOTS = (
    "CMakeLists.txt", "package.xml", "apps", "config", "include", "src",
    "tools", "test")
TERMINAL_ARTIFACTS = (
    "summary.json", "diagnostic_attempts.csv", "diagnostic_stages.csv",
    "integrity.csv", "timing.csv", "candidates.csv", "transactions.csv",
    "terminal_packets.csv", "fault_truth.csv", "resolved_config.yaml",
    "run_manifest.json", "stdout.log", "stderr.log")
GATE24_EQUIVALENCE_FILES = (
    "bridge.csv", "candidates.csv", "diagnostic_attempts.csv",
    "diagnostic_candidates.csv", "diagnostic_coverage.csv",
    "diagnostic_history_summary.csv", "diagnostic_snapshot_identity.csv",
    "diagnostic_square_root.csv", "diagnostic_state_steps.csv",
    "hypotheses.csv", "integrity.csv", "states.csv", "transactions.csv")
GATE24_GOLDEN_PREFIX = (
    "docs/evidence/p1-05-deterministic-concurrency/archived-raw/"
    "candidate-3x45")
GATE24_GOLDEN_ANCHOR = {
    "schema": "uwb-imu-pl/gate24-golden-anchor/v1",
    "manifest_algorithm": "sha256",
    "canonical_serialization": "canonical-json-sort-keys-compact-utf8-v1",
    "reference": "golden-p1-05-deterministic-concurrency",
    "annotated_tag_object": "02344301915590881b08810f1c34f67aed04b9e4",
    "peeled_commit": "4657cf87539cf7699231642d3356c08456b1833e",
    "tree": "8d2ee279074fb02b269f20041eb6b8806980ca06",
    "equivalence_runs": 3,
    "files_per_run": 13,
    "manifest_entries": 39,
    "manifest_sha256":
        "5d53385b8f019c12e33c0db25c200556c60ca5941dad06b6a0cfd0fdc68960dc",
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _resolve_golden_reference_manifest(reference: str, root=ROOT):
    """Resolve a tag and every Gate 2.4 input blob without trusting it."""
    if not re.fullmatch(r"[A-Za-z0-9._-]+", reference):
        raise ValueError("invalid golden reference name")
    def git(*arguments, text=True):
        return subprocess.check_output(["git", *arguments], cwd=root,
                                       text=text).strip() if text else \
            subprocess.check_output(["git", *arguments], cwd=root)
    if git("cat-file", "-t", reference) != "tag":
        raise ValueError("golden reference is not an annotated tag")
    payload = {
        "schema": "uwb-imu-pl/gate24-golden-reference/v1",
        "reference": reference,
        "tag_object": git("rev-parse", reference),
        "commit": git("rev-parse", f"{reference}^{{}}"),
        "tree": git("rev-parse", f"{reference}^{{tree}}"),
        "runs": {},
    }
    for index in (1, 2, 3):
        files = {}
        for name in GATE24_EQUIVALENCE_FILES:
            path = f"{GATE24_GOLDEN_PREFIX}/run{index}/{name}"
            blob = git("rev-parse", f"{reference}:{path}")
            if git("cat-file", "-t", blob) != "blob":
                raise ValueError(f"golden object is not a blob: {path}")
            content = git("cat-file", "blob", blob, text=False)
            files[name] = {
                "path": path,
                "blob": blob,
                "sha256": hashlib.sha256(content).hexdigest(),
                "bytes": len(content),
            }
        payload["runs"][f"run{index}"] = files
    return payload


def canonical_golden_manifest_sha256(payload) -> str:
    """Hash the complete 3x13 manifest using the versioned serialization."""
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def golden_anchor_from_protocol(protocol):
    anchor = protocol.get("golden_reference_anchor")
    if anchor != GATE24_GOLDEN_ANCHOR:
        raise ValueError("validation protocol golden anchor differs from pins")
    return anchor


def golden_reference_manifest(reference: str, root=ROOT, protocol=None):
    """Return the manifest only when tag, commit, tree and 39 blobs are pinned."""
    anchor = (golden_anchor_from_protocol(protocol) if protocol is not None
              else GATE24_GOLDEN_ANCHOR)
    if reference != anchor["reference"]:
        raise ValueError("golden reference name differs from pinned anchor")
    payload = _resolve_golden_reference_manifest(reference, root)
    actual = {
        "annotated_tag_object": payload["tag_object"],
        "peeled_commit": payload["commit"],
        "tree": payload["tree"],
    }
    for key, value in actual.items():
        if value != anchor[key]:
            raise ValueError(f"golden reference {key} differs from pinned anchor")
    entries = sum(len(files) for files in payload["runs"].values())
    if (len(payload["runs"]) != anchor["equivalence_runs"] or
            any(len(files) != anchor["files_per_run"]
                for files in payload["runs"].values()) or
            entries != anchor["manifest_entries"]):
        raise ValueError("golden reference manifest is not canonical 3x13")
    if canonical_golden_manifest_sha256(payload) != anchor["manifest_sha256"]:
        raise ValueError("golden reference manifest digest differs from pinned anchor")
    return payload


def git_identity():
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    return sha, source_tree_identity()["source_tree_sha256"]


def source_tree_identity(root=ROOT):
    """Hash executable inputs, never campaign output/evidence.

    The old dirty-diff hash accidentally became self-referential when evidence
    was written below the worktree.  This manifest hashes the complete set of
    tracked and untracked source/config/test inputs in fixed roots instead.
    """
    command = ["git", "ls-files", "-co", "--exclude-standard", "--",
               *SOURCE_IDENTITY_ROOTS]
    names = subprocess.check_output(command, cwd=root, text=True).splitlines()
    files = []
    for name in sorted(set(names)):
        path = root / name
        if (path.is_file() and "__pycache__" not in path.parts and
                not name.endswith((".pyc", ":Zone.Identifier"))):
            files.append((name, digest(path)))
    payload = "".join(f"{name}\0{sha}\n" for name, sha in files).encode()
    return {"source_tree_sha256": hashlib.sha256(payload).hexdigest(),
            "source_file_count": len(files)}


def loaded_dso_hashes(binary: Path):
    output = subprocess.check_output(["ldd", str(binary)], text=True,
                                     stderr=subprocess.STDOUT)
    result = {}
    for line in output.splitlines():
        match = re.search(r"=>\s+(/[^ ]+)\s+\(", line)
        if not match:
            match = re.match(r"\s*(/[^ ]+)\s+\(", line)
        if match:
            path = Path(match.group(1)).resolve()
            if path.is_file():
                result[str(path)] = digest(path)
    if not result:
        raise RuntimeError(f"no loaded DSO identity found for {binary}")
    return dict(sorted(result.items()))


def execution_identity(binary: Path):
    binary = binary.resolve()
    source = source_tree_identity()
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                  text=True).strip()
    return {
        "schema": "uwb-imu-pl/execution-identity/v1",
        "git_sha": sha,
        **source,
        "benchmark_source_sha256": digest(
            ROOT / "apps/realtime_performance_benchmark.cpp"),
        "benchmark_path": str(binary),
        "benchmark_sha256": digest(binary),
        "loaded_dso_sha256": loaded_dso_hashes(binary),
    }


def terminal_artifact_manifest(run: Path):
    manifest = {}
    for name in TERMINAL_ARTIFACTS:
        path = run / name
        if not path.is_file():
            raise ValueError(f"terminal raw artifact absent: {path}")
        manifest[name] = {"sha256": digest(path), "bytes": path.stat().st_size}
    return manifest


def terminal_artifacts_match(run: Path, recorded):
    try:
        return recorded == terminal_artifact_manifest(run)
    except (OSError, ValueError):
        return False


def resume_reusable(old, expected, run: Path, frozen_execution,
                    current_execution):
    return (current_execution == frozen_execution and old is not None and
            immutable_identity(old) == expected and
            old.get("exit_code") == 0 and
            isinstance(old.get("runner_wall_s"), (int, float)) and
            math.isfinite(old["runner_wall_s"]) and old["runner_wall_s"] >= 0 and
            terminal_artifacts_match(run, old.get("terminal_artifacts")))


def select_profiles(text: str, active):
    if text == "all": return list(PROFILES)
    if text == "active": return list(active)
    values = [item.strip() for item in text.split(",") if item.strip()]
    if not values or any(item not in PROFILES for item in values):
        raise ValueError("profiles must be all, active, or a comma-separated profile list")
    return values


def scenarios(protocol, gate, profile, explicit):
    if explicit: return [explicit]
    data = protocol["scenarios"]
    if gate == "fgo": return data["fgo"]
    if gate == "performance": return ["nominal"]
    return data[gate].get(profile, [])


def fault_schedule(scenario: str, epochs: int):
    """Mirror the benchmark's single versioned schedule calculation."""
    historical = scenario == "history_uwb"
    single_epoch = scenario in (
        "uwb_recovery", "imu_recovery", "joint_recovery")
    fault_expected = scenario != "nominal"
    begin_zero = 5 if historical else max(5, epochs // 3)
    end_zero = (begin_zero if single_epoch else
                max(5, epochs - 25) if historical else
                max(begin_zero, 2 * epochs // 3 - 1))
    if not fault_expected:
        return {"fault_expected": False, "epochs": epochs,
                "pre": epochs, "begin": None, "end": None,
                "fault": 0, "post": 0}
    return {"fault_expected": True, "epochs": epochs,
            "pre": begin_zero, "begin": begin_zero + 1,
            "end": end_zero + 1, "fault": end_zero - begin_zero + 1,
            "post": epochs - end_zero - 1}


def validate_profile_matrix_protocol(protocol):
    matrix = protocol.get("profile_matrix", {})
    original = int(protocol["epochs"].get("behavior_original", 0))
    scaled = int(protocol["epochs"]["behavior"])
    expected_runs = sum(len(protocol["scenarios"]["profiles"].get(profile, []))
                        for profile in protocol["profiles"])
    if (matrix.get("disposition") != "REVISED_SCOPE_38X20" or
            matrix.get("profile_scenario_runs") != expected_runs or
            matrix.get("attempts_per_run") != scaled or
            matrix.get("planned_attempts") != expected_runs * scaled or
            original != 300 or scaled != 20 or
            matrix.get("original_campaign", {}).get("status") !=
            "SUPERSEDED_BY_USER_SCOPE_CHANGE/NOT_CLAIMED" or
            matrix.get("schedule", {}).get("algorithm") !=
            "proportional_thirds_min5_history_tail25_v1"):
        raise ValueError("profile matrix scope/schedule contract mismatch")
    sections = {
        "standard_interval": set(matrix["schedule"]["standard_interval"]["scenarios"]),
        "historical_interval": set(matrix["schedule"]["historical_interval"]["scenarios"]),
        "recovery_impulse": set(matrix["schedule"]["recovery_impulse"]["scenarios"]),
        "no_fault": set(matrix["schedule"]["no_fault"]["scenarios"]),
    }
    declared = set().union(*sections.values())
    actual = {scenario for profile in protocol["profiles"]
              for scenario in protocol["scenarios"]["profiles"].get(profile, [])}
    if declared != actual or sum(map(len, sections.values())) != len(declared):
        raise ValueError("scenario schedule partition mismatch")
    for name, scenarios_in_class in sections.items():
        spec = matrix["schedule"][name]
        exemplar = next(iter(scenarios_in_class))
        expected_original = fault_schedule(exemplar, original)
        expected_scaled = fault_schedule(exemplar, scaled)
        if name == "no_fault":
            if (spec.get("original") != {"normal": original} or
                    spec.get("scaled") != {"normal": scaled}):
                raise ValueError("nominal schedule declaration mismatch")
        else:
            for label, expected in (("original", expected_original),
                                    ("scaled", expected_scaled)):
                recorded = spec.get(label, {})
                keys = ("pre", "begin", "end", "fault", "post")
                if any(recorded.get(key) != expected[key] for key in keys):
                    raise ValueError(f"{name} {label} schedule mismatch")
                if min(expected["pre"], expected["fault"], expected["post"]) <= 0:
                    raise ValueError(f"{name} {label} has an empty segment")
    return {"schema": "uwb-imu-pl/revised-profile-matrix/v1",
            "disposition": matrix["disposition"],
            "original_epochs": original, "scaled_epochs": scaled,
            "expected_runs": expected_runs,
            "planned_attempts": expected_runs * scaled,
            "schedule_algorithm": matrix["schedule"]["algorithm"]}


def immutable_identity(identity):
    """Return only fields fixed before execution.

    Completion fields are intentionally excluded so --resume accepts a marker
    written by this runner after a successful prior execution, while every
    input/config/source/calibration field remains immutable.
    """
    return {key: value for key, value in identity.items()
            if key not in COMPLETION_FIELDS}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--gate", required=True,
                        choices=("fgo", "profiles", "pl", "performance"))
    parser.add_argument("--profiles", default="all")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scale", choices=("smoke", "pilot", "full"), default="smoke")
    parser.add_argument("--scenario")
    parser.add_argument("--epochs", type=int)
    parser.add_argument("--repeats", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--golden-reference")
    parser.add_argument("--binary", type=Path,
                        default=Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--simulation-calibrations", type=Path,
        default=ROOT / "config/p1_06_simulation_calibrations.json")
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = yaml.safe_load(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("schema") != "uwb-imu-pl/fde-profile-validation/v1":
        raise SystemExit("unsupported validation protocol")
    try:
        matrix_contract = validate_profile_matrix_protocol(protocol)
    except (KeyError, TypeError, ValueError) as error:
        raise SystemExit(f"invalid revised profile matrix protocol: {error}")
    if not args.binary.is_file(): raise SystemExit(f"benchmark binary absent: {args.binary}")
    args.binary = args.binary.resolve()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(output)
    if usage.free < int(protocol["storage"]["minimum_free_bytes"]):
        raise SystemExit("free-space floor would be violated")
    protocol_hash = digest(protocol_path)
    simulation_calibrations = validate_simulation_calibrations(
        args.simulation_calibrations.resolve(), ROOT)
    simulation_by_profile = {
        item["profile"]: item for item in simulation_calibrations["profiles"]}
    frozen_execution = execution_identity(args.binary)
    campaign_identity_path = output / "campaign_identity.json"
    if campaign_identity_path.exists():
        try:
            recorded_execution = json.loads(
                campaign_identity_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise SystemExit(f"invalid frozen campaign identity: {error}")
        if recorded_execution != frozen_execution:
            raise SystemExit(
                "binary/source/DSO identity changed since campaign freeze")
    else:
        campaign_identity_path.write_text(
            json.dumps(frozen_execution, indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
    if args.golden_reference:
        try:
            anchor = golden_anchor_from_protocol(protocol)
        except ValueError as error:
            raise SystemExit(f"cannot bind golden reference: {error}")
        gate_identity = dict(
            frozen_execution, golden_reference=args.golden_reference,
            golden_annotated_tag_object=anchor["annotated_tag_object"],
            golden_peeled_commit=anchor["peeled_commit"],
            golden_tree=anchor["tree"],
            golden_manifest_sha256=anchor["manifest_sha256"],
            golden_anchor_schema=anchor["schema"])
        gate_identity_path = output / "gate24_run_identity.json"
        if gate_identity_path.exists():
            if json.loads(gate_identity_path.read_text(encoding="utf-8")) != gate_identity:
                raise SystemExit("gate24 identity changed since campaign freeze")
        else:
            gate_identity_path.write_text(
                json.dumps(gate_identity, indent=2, sort_keys=True) + "\n",
                encoding="utf-8")
        try:
            golden_manifest = golden_reference_manifest(
                args.golden_reference, protocol=protocol)
        except (OSError, subprocess.CalledProcessError, ValueError) as error:
            raise SystemExit(f"cannot bind golden reference: {error}")
        golden_manifest_path = output / "golden_reference_manifest.json"
        if golden_manifest_path.exists():
            try:
                recorded_manifest = json.loads(
                    golden_manifest_path.read_text(encoding="utf-8"))
            except (OSError, ValueError) as error:
                raise SystemExit(f"invalid golden reference manifest: {error}")
            if recorded_manifest != golden_manifest:
                raise SystemExit("golden reference tag/blob manifest changed")
        else:
            golden_manifest_path.write_text(
                json.dumps(golden_manifest, indent=2, sort_keys=True) + "\n",
                encoding="utf-8")
    active_overrides = {name: os.environ[name] for name in FAULT_OVERRIDE_NAMES
                        if name in os.environ}
    if active_overrides:
        raise SystemExit(
            "formal validation campaign refuses unbound synthetic fault "
            "overrides: " + ",".join(sorted(active_overrides)))
    snapshot = output / "protocol_snapshot.yaml"
    if snapshot.exists() and digest(snapshot) != protocol_hash:
        raise SystemExit("output has a different protocol snapshot")
    if not snapshot.exists(): shutil.copy2(protocol_path, snapshot)
    selected = select_profiles(args.profiles, protocol["active_profiles"])
    if args.gate == "fgo": selected = ["off"]
    if args.gate == "pl" and args.profiles == "all": selected = protocol["active_profiles"]
    if args.epochs:
        epochs = args.epochs
    elif args.gate == "fgo": epochs = int(protocol["epochs"]["fgo"])
    elif args.gate in ("profiles", "pl"):
        epochs = int(protocol["epochs"]["behavior"])
    else:
        epochs = int(protocol["epochs"][{"smoke": "smoke", "pilot": "pilot",
                                          "full": "performance_full"}[args.scale]])
    repeats = args.repeats or (int(protocol["performance"]["full_repeats"])
                               if args.gate == "performance" and args.scale == "full" else 1)
    identity_scale = ("full" if
                      (args.gate == "fgo" or
                       (args.gate == "profiles" and
                        epochs == int(protocol["epochs"]["behavior"])))
                      else args.scale)
    failures = 0; executed = 0
    for profile in selected:
        config = ROOT / "config" / f"fde_{profile}.yaml"
        config_hash = digest(config)
        for scenario in scenarios(protocol, args.gate, profile, args.scenario):
            for repeat in range(1, repeats + 1):
                seed = (args.seed if args.seed is not None else
                        int(protocol["input"]["seeds"][
                            (repeat - 1) % len(protocol["input"]["seeds"])]))
                run_id = f"{profile}__{scenario}__e{epochs}__s{seed}__r{repeat}"
                run = output / run_id
                command = ["/usr/bin/time", "-v", str(args.binary),
                           str(config), str(run), str(epochs)]
                identity = {"schema": "uwb-imu-pl/fde-validation-run/v1",
                            "run_id": run_id, "gate": args.gate, "scale": identity_scale,
                            "profile": profile, "scenario": scenario, "epochs": epochs,
                            "seed": seed, "repeat": repeat, "protocol_id": protocol["protocol_id"],
                            "protocol_hash": protocol_hash, "config_hash": config_hash,
                            "calibration_artifact_class": "simulation/synthetic",
                            "simulation_calibration_id":
                                simulation_by_profile[profile]["calibration_id"],
                            "complete_model_config_digest":
                                simulation_by_profile[profile][
                                    "complete_model_config_digest"],
                            "profile_matrix_contract": matrix_contract,
                            "fault_schedule_original": fault_schedule(
                                scenario, matrix_contract["original_epochs"]),
                            "fault_schedule_effective": fault_schedule(
                                scenario, epochs),
                            "hardware_formal_qualification": "CLOSED/NOT_CLAIMED",
                            "git_sha": frozen_execution["git_sha"],
                            "source_tree_sha256":
                                frozen_execution["source_tree_sha256"],
                            "source_file_count":
                                frozen_execution["source_file_count"],
                            "benchmark_source_sha256":
                                frozen_execution["benchmark_source_sha256"],
                            "benchmark_path": frozen_execution["benchmark_path"],
                            "benchmark_sha256":
                                frozen_execution["benchmark_sha256"],
                            "loaded_dso_sha256":
                                frozen_execution["loaded_dso_sha256"],
                            "fault_overrides": {}, "command": command}
                marker = run / "validation_run.json"
                if run.exists():
                    old = json.loads(marker.read_text()) if marker.exists() else None
                    current_execution = execution_identity(args.binary)
                    complete = resume_reusable(
                        old, identity, run, frozen_execution, current_execution)
                    if args.resume and complete:
                        print(f"RESUME {run_id}"); continue
                    raise SystemExit(f"refusing to overwrite or reuse mismatched run: {run}")
                run.mkdir(parents=True)
                marker.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
                if execution_identity(args.binary) != frozen_execution:
                    raise SystemExit(
                        "binary/source/DSO identity changed during campaign")
                env = os.environ.copy(); env["UWB_IMU_PL_SCENARIO"] = scenario
                env["UWB_IMU_PL_BENCHMARK_SEED"] = str(seed)
                start = time.monotonic()
                with (run / "stdout.log").open("w") as stdout, (run / "stderr.log").open("w") as stderr:
                    result = subprocess.run(command, cwd=ROOT, env=env, stdout=stdout, stderr=stderr)
                identity["exit_code"] = result.returncode
                identity["runner_wall_s"] = time.monotonic() - start
                marker.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
                for canonical, source in (("attempts.csv", "diagnostic_attempts.csv"),
                                          ("stages.csv", "diagnostic_stages.csv")):
                    if (run / source).exists(): (run / canonical).symlink_to(source)
                try:
                    identity["terminal_artifacts"] = terminal_artifact_manifest(run)
                except (OSError, ValueError) as error:
                    identity["terminal_artifact_error"] = str(error)
                    failures += 1
                marker.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
                executed += 1; failures += result.returncode != 0
                print(f"{'PASS' if result.returncode == 0 else 'FAIL'} {run_id} "
                      f"{identity['runner_wall_s']:.3f}s")
                if shutil.disk_usage(output).free < int(protocol["storage"]["minimum_free_bytes"]):
                    raise SystemExit("free-space floor reached during campaign")
    analyzer = ROOT / "tools" / "analyze_fde_profile_runs.py"
    analyzed = subprocess.run([sys.executable, str(analyzer), "--input", str(output)])
    print(f"completed={executed} failures={failures} output={output}")
    return 1 if failures or analyzed.returncode else 0


if __name__ == "__main__":
    raise SystemExit(main())
