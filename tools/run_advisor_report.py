#!/usr/bin/env python3
"""Prepare, run, analyze, and compile the isolated advisor-report evidence."""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys
from typing import Any

import yaml

from advisor_report_common import (artifact_root, atomic_json, environment,
                                   git_metadata, load_protocol, sha256_file,
                                   verify_checksums, write_checksums)

REPOSITORY = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = REPOSITORY / "config/advisor_report_experiments.yaml"
DEFAULT_BINARY = (REPOSITORY.parents[1] / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl/"
                  "advisor_paired_benchmark")


def set_nested(root: dict[str, Any], path: str, value: Any) -> None:
    node = root
    fields = path.split(".")
    for field in fields[:-1]:
        node = node[field]
    node[fields[-1]] = value


class Pipeline:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.protocol_path = args.protocol.resolve()
        self.protocol = load_protocol(self.protocol_path)
        self.root = args.root.resolve() if args.root else artifact_root(
            REPOSITORY, self.protocol_path)
        self.manifest_path = self.root / "experiment_manifest.json"

    def prepare(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / "raw").mkdir(exist_ok=True)
        (self.root / "summary").mkdir(exist_ok=True)
        (self.root / "configs").mkdir(exist_ok=True)
        shutil.copy2(self.protocol_path, self.root / "protocol.yaml")
        base_path = REPOSITORY / self.protocol["base_config"]
        base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
        for name, global_diagnostics in (("benchmark", False), ("roc", True)):
            config = json.loads(json.dumps(base))
            set_nested(config, "output.write_residuals", False)
            set_nested(config, "output.write_timing", True)
            set_nested(config, "output.write_global_diagnostics", global_diagnostics)
            set_nested(config, "output.root", str(self.root / "raw"))
            (self.root / "configs" / f"{name}.yaml").write_text(
                yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        git_sha, dirty = git_metadata(REPOSITORY)
        manifest = {
            "schema_version": "uwb-imu-pl/advisor-report-manifest/v1",
            "git_sha": git_sha,
            "git_dirty_at_prepare": dirty,
            "protocol_sha256": sha256_file(self.protocol_path),
            "protocol_path": str(self.protocol_path),
            "base_config": str(base_path),
            "artifact_root": str(self.root),
            "environment": environment(),
            "execution_profile": None,
            "commands": [],
            "suites": {},
            "week4_acceptance_modified": False,
        }
        atomic_json(self.manifest_path, manifest)
        print(self.root)

    def manifest(self) -> dict[str, Any]:
        if not self.manifest_path.is_file():
            self.prepare()
        return json.loads(self.manifest_path.read_text(encoding="utf-8"))

    def build(self) -> None:
        command = ["cmake", "--build", str(REPOSITORY.parents[1] / "build/uwb_imu_pl"),
                   "--target", "advisor_paired_benchmark", f"-j{self.args.threads}"]
        subprocess.run(command, cwd=REPOSITORY, check=True)

    def execute(self, suite: str, output: pathlib.Path, dataset: str,
                seed: int, count: int, epochs: int, histories: list[int],
                trajectories: list[str], extra: list[str] | None = None,
                config: str = "benchmark", resume: bool = True,
                output_range: tuple[int, int] | None = None,
                suite_profile: str = "complete") -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        command = [str(self.args.binary), str(self.root / "configs" / f"{config}.yaml"),
                   str(output), dataset, str(seed), str(count), "--epochs", str(epochs),
                   "--histories", ",".join(map(str, histories)), "--trajectories",
                   ",".join(trajectories), "--threads", str(self.args.threads)]
        if extra:
            command.extend(extra)
        if output_range:
            command.extend(["--output-start-epoch", str(output_range[0]),
                            "--output-end-epoch", str(output_range[1])])
        if resume:
            command.append("--resume")
        env = os.environ.copy()
        for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                    "NUMEXPR_NUM_THREADS"):
            env[key] = "1"
        manifest = self.manifest()
        log_path = self.root / "logs" / f"{len(manifest['commands'])+1:03d}_{suite}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        record = {"suite": suite, "profile": suite_profile, "command": command,
                  "output": str(output), "log": str(log_path),
                  "seed": {"start": seed, "count": count},
                  "executed_epochs": [0, epochs - 1],
                  "output_epochs": list(output_range or (0, epochs - 1)),
                  "returncode": None, "sha256": None}
        manifest["commands"].append(record)
        suite_record = manifest["suites"].setdefault(
            suite, {"profile": suite_profile, "status": "RUNNING", "runs": []})
        suite_record["profile"] = suite_profile
        suite_record["status"] = "RUNNING"
        suite_record["runs"].append(record.copy())
        atomic_json(self.manifest_path, manifest)
        with log_path.open("w", encoding="utf-8") as log:
            log.write("$ " + " ".join(command) + "\n")
            log.flush()
            completed = subprocess.run(command, cwd=REPOSITORY, env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
        manifest = self.manifest()
        manifest["commands"][-1]["returncode"] = completed.returncode
        if completed.returncode:
            manifest["suites"][suite]["status"] = "INVALID"
            manifest["suites"][suite]["runs"][-1]["returncode"] = completed.returncode
            atomic_json(self.manifest_path, manifest)
            raise subprocess.CalledProcessError(completed.returncode, command)
        digest = sha256_file(output)
        manifest["commands"][-1]["sha256"] = digest
        manifest["suites"][suite]["status"] = "COLLECTED"
        manifest["suites"][suite]["runs"][-1]["returncode"] = 0
        manifest["suites"][suite]["runs"][-1]["sha256"] = digest
        atomic_json(self.manifest_path, manifest)

    def run(self, suites: list[str]) -> None:
        p = self.protocol
        profile = "smoke" if self.args.smoke else self.args.profile
        baseline_smoke = profile in ("smoke", "focused-pl")
        pl_smoke = profile == "smoke"
        manifest = self.manifest()
        manifest["execution_profile"] = {
            "smoke": "SMOKE", "focused-pl": "FOCUSED_PL",
            "full": "PREREGISTERED_FULL"}[profile]
        atomic_json(self.manifest_path, manifest)
        if "nominal" in suites:
            spec = p["nominal"]
            self.execute("nominal", self.root/"raw/nominal.csv", "nominal",
                spec["seeds"]["start"], 2 if baseline_smoke else spec["seeds"]["count"],
                250 if baseline_smoke else spec["epochs_per_sequence"], spec["histories"],
                spec["trajectories"], suite_profile="smoke" if baseline_smoke else "complete")
        if "window_ablation" in suites:
            spec = p["window_ablation"]
            self.execute("window_ablation", self.root/"raw/window_ablation.csv", "nominal",
                spec["seeds"]["start"], 1 if baseline_smoke else spec["seeds"]["count"],
                250 if baseline_smoke else spec["epochs_per_sequence"], spec["histories"],
                spec["trajectories"], suite_profile="smoke" if baseline_smoke else "complete")
        if "roc" in suites:
            spec = p["roc"]
            epochs = 300 if baseline_smoke else spec["epochs_per_sequence"]
            onset = 100 if baseline_smoke else spec["fault_onset_epoch"]
            definitions = (("roc_calibration", "nominal", spec["calibration_seeds"]),
                           ("roc_nominal", "nominal", spec["nominal_evaluation_seeds"]),
                           ("roc_fault", "fault", spec["fault_evaluation_seeds"]))
            for name, dataset, seeds in definitions:
                smoke_count = spec["fault_evaluation_seeds"]["count"] \
                    if name == "roc_fault" else 5
                self.execute(name, self.root/f"raw/{name}.csv", dataset,
                    seeds["start"], smoke_count if baseline_smoke else seeds["count"], epochs,
                    spec["histories"], spec["trajectories"],
                    ["--fault-onset", str(onset), "--fault-magnitudes",
                     ",".join(map(str, spec["fault"]["magnitudes_m"]))], config="roc",
                    suite_profile="smoke" if baseline_smoke else "complete")
        if "performance" in suites:
            spec = p["performance"]
            repetitions = 1 if baseline_smoke else spec["repetitions"]
            epochs = 500 if baseline_smoke else spec["epochs_per_repetition"]
            for history in spec["histories"]:
                for repetition in range(1, repetitions+1):
                    self.execute("performance", self.root/
                        f"raw/performance_h{history}_r{repetition}.csv", "nominal",
                        spec["seed"], 1, epochs, [history], [spec["trajectory"]],
                        suite_profile="smoke" if baseline_smoke else "complete")
        if "representative_fault" in suites:
            spec = p["representative_fault"]
            epochs = 300 if baseline_smoke else spec["epochs"]
            onset = 100 if baseline_smoke else spec["onset_epoch"]
            self.execute("representative_fault", self.root/"raw/representative_fault.csv",
                "fault", spec["seed"], 1, epochs, spec["histories"],
                [spec["trajectory"]], ["--fault-onset", str(onset), "--fault-anchor",
                str(spec["anchor_id"]), "--fault-magnitude", str(spec["magnitude_m"])],
                suite_profile="smoke" if baseline_smoke else "complete")
        if "isolated_pl" in suites:
            spec = p["isolated_pl"]
            count = 64 if pl_smoke else spec["seeds"]["count"]
            output_range = tuple(spec["output_epoch_range"])
            self.execute("isolated_pl", self.root/"raw/isolated_pl.csv", "fault",
                spec["seeds"]["start"], count, spec["epochs_per_sequence"],
                spec["histories"], spec["trajectories"],
                ["--fault-onset", str(spec["fault_onset_epoch"]),
                 "--fault-duration-epochs", str(spec["fault_duration_epochs"]),
                 "--fault-magnitudes", ",".join(map(str, spec["fault"]["magnitudes_m"]))],
                config="benchmark", output_range=output_range,
                suite_profile="smoke" if pl_smoke else "complete")
        if "persistent_fault" in suites:
            spec = p["persistent_fault"]
            count = 8 if pl_smoke else spec["seeds"]["count"]
            output_range = tuple(spec["output_epoch_range"])
            self.execute("persistent_fault", self.root/"raw/persistent_fault.csv", "fault",
                spec["seeds"]["start"], count, spec["epochs_per_sequence"],
                spec["histories"], spec["trajectories"],
                ["--fault-onset", str(spec["fault_onset_epoch"]),
                 "--fault-duration-epochs", str(spec["fault_duration_epochs"]),
                 "--fault-magnitude", str(spec["fault"]["magnitude_m"])],
                config="benchmark", output_range=output_range,
                suite_profile="smoke" if pl_smoke else "complete")
        if "representative_trajectory" in suites:
            spec = p["representative_trajectory"]
            self.execute("representative_trajectory",
                self.root/"raw/representative_trajectory.csv", "nominal",
                spec["seed"], 1, spec["epochs_per_sequence"], spec["histories"],
                [spec["trajectory"]], suite_profile="complete")

    def analyze(self) -> None:
        command = [sys.executable, str(REPOSITORY/"tools/analyze_advisor_report.py"),
                   str(self.root), "--protocol", str(self.protocol_path)]
        subprocess.run(command, cwd=REPOSITORY, check=True)

    def slides(self) -> None:
        source = REPOSITORY / "doc/advisor_report/main.tex"
        output = source.parent
        for _ in range(2):
            subprocess.run(["pdflatex", "-interaction=nonstopmode", "-halt-on-error",
                            source.name], cwd=output, check=True)
        shutil.copy2(output/"main.pdf", self.root/"main.pdf")
        (self.root/"logs").mkdir(exist_ok=True)
        shutil.copy2(output/"main.log", self.root/"logs/latex.log")
        write_checksums(self.root)
        ok, errors = verify_checksums(self.root)
        if not ok:
            raise RuntimeError("checksum verification failed: " + "; ".join(errors))

    def status(self) -> None:
        manifest = self.manifest()
        summary_path = self.root / "summary/summary.json"
        print(json.dumps({"artifact_root": str(self.root),
                          "execution_profile": manifest.get("execution_profile"),
                          "suites": manifest.get("suites"),
                          "summary": json.loads(summary_path.read_text())
                              if summary_path.is_file() else None}, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "build", "run", "analyze",
                                             "slides", "status", "all"))
    parser.add_argument("--protocol", type=pathlib.Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--root", type=pathlib.Path)
    parser.add_argument("--binary", type=pathlib.Path, default=DEFAULT_BINARY)
    parser.add_argument("--threads", type=int, default=max(1, min(4, os.cpu_count() or 1)))
    parser.add_argument("--suite", action="append", choices=("nominal", "window_ablation",
                        "roc", "performance", "representative_fault", "isolated_pl",
                        "persistent_fault", "representative_trajectory"))
    parser.add_argument("--profile", choices=("smoke", "focused-pl", "full"),
                        default="full", help="focused-pl runs complete PL suites and smoke baselines")
    parser.add_argument("--smoke", action="store_true",
                        help="run a structurally complete reduced campaign, labelled SMOKE")
    args = parser.parse_args()
    pipeline = Pipeline(args)
    suites = args.suite or ["nominal", "window_ablation", "roc", "performance",
                            "isolated_pl", "persistent_fault", "representative_trajectory"]
    if args.command in ("prepare", "all"): pipeline.prepare()
    if args.command in ("build", "all"): pipeline.build()
    if args.command in ("run", "all"): pipeline.run(suites)
    if args.command in ("analyze", "all"): pipeline.analyze()
    if args.command in ("slides", "all"): pipeline.slides()
    if args.command in ("status", "all"): pipeline.status()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
