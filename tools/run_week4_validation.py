#!/usr/bin/env python3
"""Phase-2 Week-4 experiment orchestrator and acceptance freezer."""

from __future__ import annotations

import argparse
import copy
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time
from typing import Any

from week4_common import (FAIL, INVALID, PASS, InvalidArtifact, atomic_json,
                          environment_record, git_metadata, load_protocol,
                          protocol_hash, resolve_config, shell_join, utc_now,
                          verify_checksums, write_checksums, sha256_file)

REPOSITORY = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = REPOSITORY / "config/week4_validation.yaml"


def executable(name: str, bin_directory: pathlib.Path | None) -> str:
    candidates = []
    if bin_directory: candidates.append(bin_directory / name)
    candidates.append(REPOSITORY.parents[1] / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl" / name)
    found = shutil.which(name)
    if found: candidates.append(pathlib.Path(found))
    for candidate in candidates:
        if pathlib.Path(candidate).is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    raise InvalidArtifact(f"cannot locate executable {name}")


def run_command(command: list[str], log: pathlib.Path, timeout: int | None = None) -> int:
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as stream:
        stream.write(f"$ {shell_join(command)}\n")
        stream.flush()
        try:
            completed = subprocess.run(command, cwd=REPOSITORY, stdout=stream,
                                       stderr=subprocess.STDOUT, timeout=timeout)
            return completed.returncode
        except subprocess.TimeoutExpired:
            stream.write(f"TIMEOUT after {timeout}s\n")
            return 124


def development_protocol(protocol: dict[str, Any]) -> dict[str, Any]:
    value = copy.deepcopy(protocol)
    offset = 700_000_000
    value["formal_execution"]["formal"] = False
    value["noncentral"]["root_seed"] += offset
    value["noncentral"]["h0"]["trials_per_job"] = 1000
    value["noncentral"]["alternatives"]["trials_per_job"] = 1000
    value["noncentral"]["global_test"]["parameter_bootstrap_trials"] = 200
    sweep = value["snapshot_sweep"]
    sweep["seeds"] = {"start": 70260901, "stop": 70260902}
    sweep["epochs_per_job"] = 5
    sweep["expected_jobs"] = 2052*2
    sweep["expected_rows"] = 2052*2*5
    value["imu_monte_carlo"]["root_seed"] += offset
    value["imu_monte_carlo"]["trials_per_scenario"] = 2000
    roc = value["roc"]
    for index, name in enumerate(("calibration_seeds", "nominal_evaluation_seeds",
                                  "fault_evaluation_seeds")):
        roc[name] = {"start": 71000000+index*100,
                     "count": 24 if name == "fault_evaluation_seeds" else 2}
    roc["epochs_per_sequence"] = 30
    roc["fault"]["onset_epoch"] = 10
    roc["temporal_ablation"]["rolling_windows"] = [1, 10]
    roc["gates"]["longest_window_min_valid_epochs_per_item"] = 1
    roc["bootstrap"]["trials"] = 100
    value["method_ab_shadow"]["root_seed"] += offset
    value["performance"]["seed"] += offset
    value["performance"]["epochs_per_repetition"] = 250
    value["performance"]["warmup_epochs"] = 10
    value["performance"]["timeout_s"] = 120
    return value


class Runner:
    def __init__(self, args):
        self.args = args
        self.protocol_path = args.protocol.resolve()
        self.protocol = load_protocol(self.protocol_path)
        self.git_sha, self.git_dirty = git_metadata(REPOSITORY)
        self.protocol_digest = protocol_hash(self.protocol_path)
        if args.formal:
            self.root = (args.results_root /
                         f"week4_{self.git_sha[:12]}_{self.protocol_digest[:12]}").resolve()
            self.used_protocol = self.protocol
        else:
            self.root = (args.results_root / "week4_development").resolve()
            self.used_protocol = development_protocol(self.protocol)
        self.manifest_path = self.root / "experiment_manifest.json"

    def require_formal_state(self):
        if self.args.formal and self.git_dirty:
            raise InvalidArtifact("formal execution requires a clean worktree")
        if self.manifest_path.exists():
            manifest = json.loads(self.manifest_path.read_text())
            if manifest["git_sha"] != self.git_sha:
                raise InvalidArtifact("resume code SHA differs from experiment manifest")
            if manifest["protocol_hash"] != self.protocol_digest:
                raise InvalidArtifact("resume protocol hash differs from manifest")

    def prepare(self) -> dict[str, Any]:
        self.require_formal_state()
        self.root.mkdir(parents=True, exist_ok=True)
        if self.manifest_path.exists():
            return json.loads(self.manifest_path.read_text())
        protocol_copy = self.root / "protocol.yaml"
        if self.args.formal:
            shutil.copy2(self.protocol_path, protocol_copy)
        else:
            protocol_copy.write_text(__import__("yaml").safe_dump(
                self.used_protocol, sort_keys=False), encoding="utf-8")
        base = REPOSITORY / self.used_protocol["base_config"]
        configs = {}
        definitions = {
            "noncentral": {"seed": self.used_protocol["noncentral"]["root_seed"]},
            "snapshot": {"seed": self.used_protocol["snapshot_sweep"]["seeds"]["start"]},
            "imu": {"seed": self.used_protocol["imu_monte_carlo"]["root_seed"]},
            "method_ab": {"seed": self.used_protocol["method_ab_shadow"]["root_seed"],
                          "incremental.fixed_lag_epochs": 200},
            "performance": {
                "seed": self.used_protocol["performance"]["seed"],
                "incremental.fixed_lag_epochs": 200,
                "output.write_global_diagnostics": False,
                "output.write_residuals": False, "output.write_timing": True,
                "output.root": str(self.root / "performance")},
        }
        for history in self.used_protocol["roc"]["graph_histories"]:
            definitions[f"roc_history_{history}"] = {
                "seed": self.used_protocol["roc"]["calibration_seeds"]["start"],
                "incremental.fixed_lag_epochs": history,
                "output.write_global_diagnostics": True,
                "output.write_residuals": False, "output.write_timing": False,
                "output.root": str(self.root / "roc")}
        for name, overrides in definitions.items():
            path = self.root / "configs" / f"{name}.yaml"
            _, config_hash = resolve_config(base, overrides, path)
            configs[name] = {"path": str(path.relative_to(self.root)),
                             "hash": config_hash, "overrides": overrides}
        manifest = {
            "schema_version": "uwb-imu-pl/week4-manifest/v1",
            "created_utc": utc_now(), "formal": self.args.formal,
            "protocol_path": str(self.protocol_path),
            "protocol_hash": self.protocol_digest,
            "git_sha": self.git_sha, "git_dirty": self.git_dirty,
            "base_config": self.used_protocol["base_config"],
            "resolved_configs": configs,
            "seed_ranges": {
                key: self.used_protocol[key].get("seeds", self.used_protocol[key].get("root_seed"))
                for key in ("noncentral", "snapshot_sweep", "imu_monte_carlo")},
            "environment": environment_record(), "attempts": [],
            "builds": {},
            "final_gate_verdict": INVALID,
        }
        atomic_json(self.manifest_path, manifest)
        return manifest

    def config(self, name: str) -> pathlib.Path:
        manifest = json.loads(self.manifest_path.read_text())
        return self.root / manifest["resolved_configs"][name]["path"]

    def attempt(self, stage: str, resume: bool) -> tuple[pathlib.Path, dict[str, Any]]:
        manifest = self.prepare()
        existing = [item for item in manifest["attempts"] if item["stage"] == stage]
        if resume and existing and existing[-1]["status"] == INVALID:
            record = existing[-1]
            directory = self.root / record["directory"]
            record["resumed_utc"] = utc_now()
        else:
            number = len(existing)+1
            directory = self.root / stage / f"attempt_{number:03d}"
            record = {"stage": stage, "attempt": number,
                      "directory": str(directory.relative_to(self.root)),
                      "started_utc": utc_now(), "status": INVALID,
                      "commands": [], "input_rows": {}, "output_rows": {}}
            manifest["attempts"].append(record)
        directory.mkdir(parents=True, exist_ok=True)
        atomic_json(self.manifest_path, manifest)
        return directory, record

    def finish_attempt(self, record: dict[str, Any], result: dict[str, Any]):
        manifest = json.loads(self.manifest_path.read_text())
        for item in manifest["attempts"]:
            if item["stage"] == record["stage"] and item["attempt"] == record["attempt"]:
                item.update(record)
                item["status"] = result["status"]
                item["completed_utc"] = utc_now()
                item["gate_result"] = str((self.root / item["directory"] /
                                           "gate_result.json").relative_to(self.root))
                break
        atomic_json(self.manifest_path, manifest)

    def command(self, record, command, log, timeout=None, accepted=(0,)):
        record["commands"].append(shell_join(command))
        code = run_command(command, log, timeout)
        if code not in accepted:
            raise InvalidArtifact(f"command exited {code}: {shell_join(command)}")

    def run_stage(self, stage: str, resume: bool) -> dict[str, Any]:
        self.require_formal_state()
        directory, record = self.attempt(stage, resume)
        log = directory / "execution.log"
        protocol_copy = self.root / "protocol.yaml"
        try:
            if stage == "noncentral":
                binary = executable("integrity_monte_carlo", self.args.bin_dir)
                spec = self.used_protocol[stage]
                command = [binary, str(self.config("noncentral")), str(directory / "raw"),
                           "--h0-trials", str(spec["h0"]["trials_per_job"]),
                           "--noncentral-trials", str(spec["alternatives"]["trials_per_job"]),
                           "--threads", str(self.args.threads)]
                self.command(record, command, log)
                command = [sys.executable, str(REPOSITORY/"tools/analyze_noncentral.py"),
                           str(directory/"raw/h0.csv"), str(directory/"raw/noncentral.csv"),
                           str(directory/"gate_result.json"), "--protocol", str(protocol_copy)]
                self.command(record, command, log, accepted=(0, 1))
            elif stage == "snapshot_sweep":
                binary = executable("snapshot_integrity_sweep", self.args.bin_dir)
                spec = self.used_protocol[stage]
                raw = directory / "raw"
                raw.mkdir(exist_ok=True)
                for seed in range(spec["seeds"]["start"], spec["seeds"]["stop"]+1):
                    command = [binary, str(self.config("snapshot")), str(raw/f"seed_{seed}.csv"),
                               "--seed-start", str(seed), "--seed-count", "1",
                               "--epochs", str(spec["epochs_per_job"]), "--threads", str(self.args.threads)]
                    if resume: command.append("--resume")
                    self.command(record, command, log)
                command = [sys.executable, str(REPOSITORY/"tools/summarize_snapshot_sweep.py"),
                           str(raw), str(directory/"snapshot_summary.csv"), "--protocol",
                           str(protocol_copy), "--inventory", str(directory/"raw_inventory.json"),
                           "--result", str(directory/"gate_result.json")]
                self.command(record, command, log, accepted=(0, 1, 2))
            elif stage == "imu_monte_carlo":
                command = [sys.executable, str(REPOSITORY/"tools/run_imu_bias_monte_carlo.py"),
                           str(self.config("imu")), str(directory/"gate_result.json"),
                           "--protocol", str(protocol_copy)]
                self.command(record, command, log, accepted=(0, 1))
            elif stage == "history_experiments":
                manifest = json.loads(self.manifest_path.read_text())
                roc_attempts = [item for item in manifest["attempts"]
                                if item["stage"] == "roc" and
                                item["status"] in (PASS, FAIL)]
                if not roc_attempts:
                    raise InvalidArtifact("history_experiments requires a complete roc attempt")
                source = self.root / roc_attempts[-1]["gate_result"]
                combined = json.loads(source.read_text())
                checks = {
                    "history_noninferiority": combined.get("checks", {}).get(
                        "history_noninferiority", False),
                    "outputs_complete": combined.get("checks", {}).get(
                        "outputs_complete", False),
                }
                result = {"status": PASS if all(checks.values()) else FAIL,
                          "checks": checks,
                          "history_noninferiority": combined.get(
                              "history_noninferiority", []),
                          "source_roc_result": str(source.relative_to(self.root))}
                atomic_json(directory/"gate_result.json", result)
            elif stage == "roc":
                # One raw collection closes both preregistered gates; each gate
                # keeps its own manifest attempt and verdict projection.
                binary = executable("sequential_detector_history", self.args.bin_dir)
                spec = self.used_protocol["roc"]
                raw = directory / "raw"
                definitions = (("calibration", spec["calibration_seeds"]),
                               ("nominal", spec["nominal_evaluation_seeds"]),
                               ("fault", spec["fault_evaluation_seeds"]))
                for history in spec["graph_histories"]:
                    for dataset, seeds in definitions:
                        target = raw / dataset / f"history_{history}.csv"
                        command = [binary, str(self.config(f"roc_history_{history}")),
                                   str(target), dataset, str(seeds["start"]), str(seeds["count"]),
                                   "--epochs", str(spec["epochs_per_sequence"]),
                                   "--fault-onset", str(spec["fault"]["onset_epoch"]),
                                   "--threads", str(self.args.threads)]
                        if resume: command.append("--resume")
                        self.command(record, command, log)
                command = [sys.executable, str(REPOSITORY/"tools/analyze_detector_roc.py"),
                           str(raw/"calibration"), str(raw/"nominal"), str(raw/"fault"),
                           str(directory/"gate_result.json"), "--protocol", str(protocol_copy)]
                self.command(record, command, log, accepted=(0, 1, 2))
            elif stage == "method_ab_shadow":
                command = [executable("method_ab_shadow", self.args.bin_dir),
                           str(self.config("method_ab")), str(directory/"comparison.csv"),
                           str(directory/"gate_result.json")]
                self.command(record, command, log, accepted=(0, 1))
            elif stage == "ros_topic_tests":
                for scenario in self.used_protocol[stage]["scenarios"]:
                    target = directory / scenario
                    command = ["roslaunch", "uwb_imu_pl", "week4_topic_test.launch",
                               f"scenario:={scenario}", f"run_directory:={target}",
                               f"inventory:={target/'publisher_inventory.json'}"]
                    self.command(record, command, log, timeout=180)
                command = [sys.executable, str(REPOSITORY/"tools/validate_week4_ros.py"),
                           str(directory), str(directory/"gate_result.json"),
                           "--protocol", str(protocol_copy)]
                self.command(record, command, log, accepted=(0, 1, 2))
            elif stage == "performance":
                spec = self.used_protocol[stage]
                binary = executable("realtime_performance_benchmark", self.args.bin_dir)
                for repetition in range(1, spec["repetitions"]+1):
                    command = ["taskset", "-c", ",".join(map(str, spec["cpu_affinity"])),
                               binary, str(self.config("performance")),
                               str(directory/f"run_{repetition}"),
                               str(spec["epochs_per_repetition"])]
                    env_command = ["env", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                                   "MKL_NUM_THREADS=1", *command]
                    self.command(record, env_command, log, timeout=spec["timeout_s"])
                command = [sys.executable, str(REPOSITORY/"tools/analyze_week4_performance.py"),
                           str(directory), str(directory/"gate_result.json"),
                           "--protocol", str(protocol_copy)]
                self.command(record, command, log, accepted=(0, 1, 2))
            else:
                raise InvalidArtifact(f"unknown stage {stage}")
            result = json.loads((directory/"gate_result.json").read_text())
            if result.get("status") not in (PASS, FAIL, INVALID):
                raise InvalidArtifact("gate result has invalid status")
            if self.args.formal and result["status"] in (PASS, FAIL) and \
                    stage in ("snapshot_sweep", "roc"):
                raw = directory/"raw"
                if stage == "roc":
                    inventory = []
                    for path in sorted(raw.rglob("*.csv")):
                        with path.open("rb") as stream:
                            row_count = max(0, sum(1 for _ in stream)-1)
                        inventory.append({"path": str(path.relative_to(raw)),
                                          "rows": row_count,
                                          "bytes": path.stat().st_size,
                                          "sha256": sha256_file(path)})
                    atomic_json(directory/"raw_inventory.json",
                                {"status": PASS, "files": inventory,
                                 "rows": sum(item["rows"] for item in inventory)})
                if raw.is_dir():
                    shutil.rmtree(raw)
                    record["raw_removed_after_summary"] = True
            inventory_path = directory/"raw_inventory.json"
            if inventory_path.is_file():
                inventory = json.loads(inventory_path.read_text())
                record["input_rows"] = {"raw": inventory.get("rows",
                    sum(item.get("rows", 0) for item in inventory.get("shards", [])))}
            if stage == "noncentral":
                record["input_rows"] = {"h0": result.get("h0_jobs", 0),
                                        "noncentral": result.get("noncentral_jobs", 0)}
            record["output_rows"] = {
                "gate_records": len(result.get("jobs", result.get(
                    "operating_points", result.get("scenarios", []))))}
        except (OSError, json.JSONDecodeError, InvalidArtifact) as error:
            result = {"status": INVALID, "reason": str(error)}
            atomic_json(directory/"gate_result.json", result)
        self.finish_attempt(record, result)
        return result

    def build_and_test(self) -> dict[str, Any]:
        self.require_formal_state()
        manifest = self.prepare()
        workspace = REPOSITORY.parents[1]
        results = {}
        for build_type in ("Debug", "Release"):
            log = self.root/"build"/f"{build_type.lower()}.log"
            log.parent.mkdir(parents=True, exist_ok=True)
            commands = [
                ["catkin", "clean", "uwb_imu_pl", "-y"],
                ["catkin", "config", "--cmake-args", f"-DCMAKE_BUILD_TYPE={build_type}"],
                ["catkin", "build", "uwb_imu_pl", "--no-status"],
                ["catkin", "run_tests", "uwb_imu_pl", "--no-status"],
                ["catkin_test_results", "--verbose"],
            ]
            status = PASS
            executed = []
            for command in commands:
                executed.append(shell_join(command))
                with log.open("a", encoding="utf-8") as stream:
                    stream.write(f"$ {shell_join(command)}\n")
                    completed = subprocess.run(command, cwd=workspace,
                                               stdout=stream,
                                               stderr=subprocess.STDOUT)
                if completed.returncode:
                    status = INVALID
                    break
            summary = {"status": status, "build_type": build_type,
                       "git_sha": self.git_sha, "commands": executed,
                       "log": str(log.relative_to(self.root))}
            atomic_json(self.root/"build"/f"{build_type.lower()}_summary.json",
                        summary)
            results[build_type] = summary
            if status != PASS:
                break
        manifest = json.loads(self.manifest_path.read_text())
        manifest["builds"] = {key: value["status"] for key, value in results.items()}
        atomic_json(self.manifest_path, manifest)
        return {"status": PASS if len(results) == 2 and
                all(item["status"] == PASS for item in results.values()) else INVALID,
                "builds": results}

    def finalize(self) -> dict[str, Any]:
        self.require_formal_state()
        manifest = self.prepare()
        required = self.used_protocol["acceptance"]["required_gates"]
        gates = {}
        for gate in required:
            attempts = [item for item in manifest["attempts"] if item["stage"] == gate]
            gates[gate] = attempts[-1]["status"] if attempts else INVALID
        all_pass = all(status == PASS for status in gates.values())
        builds_pass = manifest.get("builds") == {"Debug": PASS, "Release": PASS}
        clean_sha, dirty = git_metadata(REPOSITORY)
        traceability = clean_sha == manifest["git_sha"] and not dirty and not manifest["git_dirty"]
        status = PASS if all_pass and builds_pass and traceability and self.args.formal else (
            FAIL if any(value == FAIL for value in gates.values()) else INVALID)
        acceptance = {
            "schema_version": "uwb-imu-pl/week4-acceptance/v1",
            "status": status, "required_gates": gates,
            "protocol_hash": manifest["protocol_hash"], "git_sha": manifest["git_sha"],
            "git_dirty": dirty, "manifest_sha_matches": clean_sha == manifest["git_sha"],
            "checksum_verified": True,
            "builds": manifest.get("builds", {}),
            "claim": self.used_protocol["acceptance"]["success_token"]
                if status == PASS else None,
            "blocking_items": ([name for name, value in gates.items() if value != PASS] +
                               ([] if builds_pass else ["debug_release_builds"])),
            "excluded_claims": self.used_protocol["acceptance"]["excluded_claims"],
        }
        manifest["final_gate_verdict"] = status
        manifest["finalized_utc"] = utc_now()
        atomic_json(self.manifest_path, manifest)
        atomic_json(self.root/"acceptance_summary.json", acceptance)
        write_checksums(self.root)
        verified, errors = verify_checksums(self.root)
        if not verified:
            acceptance.update({"status": INVALID, "claim": None,
                               "checksum_verified": False,
                               "checksum_errors": errors})
            atomic_json(self.root/"acceptance_summary.json", acceptance)
            manifest["final_gate_verdict"] = INVALID
            atomic_json(self.manifest_path, manifest)
            write_checksums(self.root)
        return acceptance


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("prepare", "build", "run", "finalize", "status"))
    parser.add_argument("--stage", choices=("noncentral", "snapshot_sweep",
                        "imu_monte_carlo", "roc", "history_experiments",
                        "method_ab_shadow", "ros_topic_tests", "performance"))
    parser.add_argument("--protocol", type=pathlib.Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--results-root", type=pathlib.Path,
                        default=REPOSITORY/"results")
    parser.add_argument("--bin-dir", type=pathlib.Path)
    parser.add_argument("--threads", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--formal", action="store_true")
    args = parser.parse_args()
    try:
        runner = Runner(args)
        if args.action == "prepare": result = runner.prepare()
        elif args.action == "build": result = runner.build_and_test()
        elif args.action == "run":
            if not args.stage: raise InvalidArtifact("run requires --stage")
            result = runner.run_stage(args.stage, args.resume)
        elif args.action == "finalize": result = runner.finalize()
        else:
            result = json.loads(runner.manifest_path.read_text())
        print(json.dumps({"root": str(runner.root), "result": result},
                         sort_keys=True))
        if args.action in ("prepare", "status"):
            return 0
        status = result.get("status", result.get("final_gate_verdict", INVALID))
        return 0 if status == PASS else (1 if status == FAIL else 2)
    except (InvalidArtifact, OSError, json.JSONDecodeError) as error:
        print(f"INVALID: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
