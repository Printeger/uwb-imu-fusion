#!/usr/bin/env python3
"""Non-destructive discrete nominal ablations; never overwrites old benchmarks.

Run stages before evaluation to freeze sensor-only decisions. GT is loaded only
by the explicit evaluate command. All failures, tails and coverage are retained.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import yaml
import benchmark_common as common
from run_dataset_benchmark import default_nominal_binary

ROOT = common.ROOT
DEFAULT = ROOT / "results/nominal_accuracy_v1"
STAGES = ("query", "covariance", "causal", "imu", "lm", "lag2", "lag5",
          "lag10", "threshold03", "threshold01", "robust", "final")


def tuning(stage):
    out = {"version": 1, "nominal_initial_guess": "cv"}
    if stage == "query":
        out["bias_integration_sigmas"] = [1.0]*6  # isolate old unit-matrix default
    if stage not in ("query", "covariance"):
        out["causal_bootstrap"] = True
    if stage not in ("query", "covariance", "causal"):
        out["nominal_initial_guess"] = "imu"
    if stage in ("lm", "lag2", "lag5", "lag10", "threshold03", "threshold01", "robust", "final"):
        out["nominal_lm_iterations"] = 5
    if stage.startswith("lag"):
        out["nominal_lag_s"] = float(stage[3:])
    if stage in ("threshold03", "threshold01", "robust", "final"):
        out["nominal_lag_s"] = 5.0
    if stage == "robust":
        out["nominal_robust_experimental"] = True
    return out


def run(args):
    output = args.output.resolve()
    for manifest_path in sorted(args.cache.glob("*/*/manifest.json")):
        manifest = json.loads(manifest_path.read_text())
        if args.sequence and args.sequence not in (manifest["dataset"], manifest["sequence"]):
            continue
        if not args.all and manifest["dataset"] not in ("SFUISE", "simulation"):
            continue
        directory = output/"runs"/args.stage/manifest["dataset"]/manifest["sequence"]
        directory.mkdir(parents=True, exist_ok=True)
        if (directory/"run_status.json").exists() and not args.rerun:
            print("EXISTS", args.stage, manifest["sequence"], flush=True)
            continue
        source = ROOT/"results/benchmark/runs/current"/manifest["dataset"]/manifest["sequence"]/"effective_nominal_config.yaml"
        config = yaml.safe_load(source.read_text())
        config["estimation_tuning"] = tuning(args.stage)
        config["estimation_tuning"]["bootstrap_prefer_below_anchors"] = bool(manifest.get("bootstrap_prefer_below_anchors", False))
        config["fault_models"]["manifest_path"] = str(ROOT/"config/integrity_fault_manifest.yaml")
        config["output"]["root"] = str(directory)
        config["realtime"]["lever_arm_body_m"] = manifest.get("lever_arm_body_m", config["realtime"]["lever_arm_body_m"])
        with (manifest_path.parent/"uwb.csv").open() as stream:
            for r in csv.DictReader(stream):
                if int(r["tag_id"]) == int(manifest["primary_tag"]):
                    config["realtime"]["lever_arm_body_m"] = [float(r[k]) for k in ("lever_x", "lever_y", "lever_z")]
                    break
        if args.stage in ("threshold03", "threshold01"):
            config["incremental"]["relinearize_threshold"] = 0.03 if args.stage == "threshold03" else 0.01
        if args.profile:
            profile = yaml.safe_load(args.profile.read_text())
            device_profile = profile.get("devices", {}).get(manifest["dataset"], {})
            for section in ("imu", "estimation_tuning", "incremental"):
                if section in device_profile:
                    profile.setdefault(section, {}).update(device_profile[section])
            for section in ("imu", "estimation_tuning"):
                config[section].update(profile.get(section, {}))
            if "incremental" in profile:
                config["incremental"].update(profile["incremental"])
        config_path = directory/"effective_nominal_config.yaml"
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        command = [str(args.binary), "--config", str(config_path), "--input",
                   str(manifest_path.parent), "--output", str(directory)]
        if args.export_historical:
            command.append("--export-historical")
        if args.local_oracle_times:
            command.extend(["--local-oracle-times", args.local_oracle_times])
        environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
                           MKL_NUM_THREADS="1", EIGEN_DONT_PARALLELIZE="1",
                           LD_LIBRARY_PATH=str(args.binary.parent.parent)+":"+os.environ.get("LD_LIBRARY_PATH", ""))
        with (directory/"console.log").open("w") as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, env=environment)
        record = dict(stage=args.stage, dataset=manifest["dataset"], sequence=manifest["sequence"],
                      tbb_threads=1, cpu_affinity=sorted(os.sched_getaffinity(0)),
                      exit_code=process.returncode, command=command,
                      effective_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                      binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                      cache_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                      library_sha256=hashlib.sha256((args.binary.parent.parent/"libuwb_imu_pl.so").read_bytes()).hexdigest())
        record["source"] = common.git_identity(ROOT)
        record["product_diff_sha256"] = hashlib.sha256(subprocess.check_output(
            ["git", "diff", "--binary", "--", "CMakeLists.txt", "apps", "config", "include", "src", "test", "tools"], cwd=ROOT)).hexdigest()
        (directory/"execution.json").write_text(json.dumps(record, indent=2)+"\n")
        print(json.dumps(record), flush=True)


def evaluate(args):
    records = []
    runs = args.runs_root or args.output/"runs"/args.stage
    for directory in sorted(runs.glob("*/*")):
        if args.sequence and args.sequence not in (directory.parent.name, directory.name):
            continue
        record = dict(stage=args.stage, dataset=directory.parent.name, sequence=directory.name)
        try:
            status = json.loads((directory/"run_status.json").read_text())
            if status["status"] != "SUCCESS":
                raise RuntimeError(status.get("reason", "RUN_FAILED"))
            manifest = json.loads((args.cache/record["dataset"]/record["sequence"]/"manifest.json").read_text())
            cache = args.cache/record["dataset"]/record["sequence"]
            bounds = []
            for name in ("imu.csv", "uwb.csv"):
                rows = common.read_csv(cache/name)
                bounds.extend(float(row["t"]) for row in (rows[0], rows[-1]))
            metrics, samples = common.evaluate_trajectories(
                common.load_tum(directory/"trajectory.tum"),
                common.load_gt(cache/"gt.csv", manifest["primary_tag"]),
                sensor_interval=(min(bounds), max(bounds)))
            record.update(metrics)
            record.update(output_semantics="ONLINE_CURRENT",
                          evaluation_semantics="OFFLINE_GRID_OF_ONLINE_OUTPUTS",
                          attitude_reference="FRAME_VERIFIED_SYNTHETIC" if record["dataset"] == "simulation" else "FRAME_UNVERIFIED",
                          reference_point="UWB_TAG", source_run=str(directory.resolve()),
                          execution=json.loads((directory/"execution.json").read_text()))
            record["aggregate_eligible"] = metrics["coverage"] >= 0.8
            samples_path = args.output/"samples"/args.stage/record["dataset"]/record["sequence"]
            samples_path.mkdir(parents=True, exist_ok=True)
            (samples_path/"evaluation_samples.json").write_text(json.dumps(samples)+"\n")
            timing = list(csv.DictReader((directory/"timing.csv").open()))
            # prepare + commit already includes nested preintegration, query,
            # marginal and warm-start work. Do not double-count nested columns.
            total = np.array([float(r["epoch_total_ms"]) if "epoch_total_ms" in r else float(r["prepare_wall_ms"])+float(r["commit_wall_ms"])+float(r["state_query_wall_ms"]) for r in timing])
            record.update(epoch_total_p99_ms=float(np.percentile(total,99)),
                          epoch_total_max_ms=float(total.max()), deadline_exceedances=int((total>40).sum()),
                          wall_ms=status["wall_ms"])
            states = np.genfromtxt(directory/"states.csv", delimiter=",", names=True)
            record["velocity_max_mps"] = float(np.max(np.sqrt(states["vx"]**2+states["vy"]**2+states["vz"]**2)))
        except Exception as error:
            record.update(status="FAIL", reason=str(error), aggregate_eligible=False)
        records.append(record)
        print(json.dumps(record), flush=True)
    target = args.output/"evaluation"; target.mkdir(parents=True, exist_ok=True)
    label = args.stage + ("_" + args.sequence if args.sequence else "")
    (target/(label+".json")).write_text(json.dumps({"records":records},indent=2)+"\n")
    common.write_csv(target/(label+".csv"), sorted({k for r in records for k in r}), records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("run", "evaluate"))
    parser.add_argument("--stage", choices=STAGES, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT)
    parser.add_argument("--cache", type=Path, default=ROOT/"results/benchmark/cache")
    parser.add_argument("--binary", type=Path, default=default_nominal_binary())
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--sequence")
    parser.add_argument("--runs-root", type=Path, help="Read-only existing stage directory for evaluation; new artifacts go to --output")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--export-historical", action="store_true")
    parser.add_argument("--local-oracle-times", default="")
    args = parser.parse_args()
    (run if args.command == "run" else evaluate)(args)

if __name__ == "__main__":
    main()
