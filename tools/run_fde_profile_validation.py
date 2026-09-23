#!/usr/bin/env python3
"""Run the frozen five-profile validation protocol with resumable identities."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILES = ("off", "uwb_order1", "imu_order1", "joint_order1", "joint_order2")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_identity():
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    diff = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=ROOT)
    untracked = subprocess.check_output(["git", "ls-files", "--others", "--exclude-standard"],
                                        cwd=ROOT, text=True).splitlines()
    h = hashlib.sha256(diff)
    for name in sorted(untracked):
        path = ROOT / name
        if path.is_file():
            h.update(name.encode()); h.update(b"\0"); h.update(path.read_bytes())
    return sha, h.hexdigest()


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
    parser.add_argument("--binary", type=Path,
                        default=Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark"))
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    protocol_path = args.protocol.resolve()
    protocol = yaml.safe_load(protocol_path.read_text(encoding="utf-8"))
    if protocol.get("schema") != "uwb-imu-pl/fde-profile-validation/v1":
        raise SystemExit("unsupported validation protocol")
    if not args.binary.is_file(): raise SystemExit(f"benchmark binary absent: {args.binary}")
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(output)
    if usage.free < int(protocol["storage"]["minimum_free_bytes"]):
        raise SystemExit("free-space floor would be violated")
    protocol_hash = digest(protocol_path)
    sha, dirty_hash = git_identity()
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
    failures = 0; executed = 0
    for profile in selected:
        config = ROOT / "config" / f"fde_{profile}.yaml"
        config_hash = digest(config)
        for scenario in scenarios(protocol, args.gate, profile, args.scenario):
            for repeat in range(1, repeats + 1):
                seed = int(protocol["input"]["seeds"][(repeat - 1) % len(protocol["input"]["seeds"])])
                run_id = f"{profile}__{scenario}__e{epochs}__s{seed}__r{repeat}"
                run = output / run_id
                identity = {"schema": "uwb-imu-pl/fde-validation-run/v1",
                            "run_id": run_id, "gate": args.gate, "scale": args.scale,
                            "profile": profile, "scenario": scenario, "epochs": epochs,
                            "seed": seed, "repeat": repeat, "protocol_id": protocol["protocol_id"],
                            "protocol_hash": protocol_hash, "config_hash": config_hash,
                            "git_sha": sha, "dirty_diff_hash": dirty_hash,
                            "fault_overrides": {
                                name: os.environ.get(name) for name in (
                                    "UWB_IMU_PL_UWB_BIAS_M",
                                    "UWB_IMU_PL_ACCEL_BIAS_MPS2",
                                    "UWB_IMU_PL_GYRO_BIAS_RADPS")
                                if os.environ.get(name) is not None}}
                marker = run / "validation_run.json"
                if run.exists():
                    old = json.loads(marker.read_text()) if marker.exists() else None
                    complete = old == identity and (run / "summary.json").exists()
                    if args.resume and complete:
                        print(f"RESUME {run_id}"); continue
                    raise SystemExit(f"refusing to overwrite or reuse mismatched run: {run}")
                run.mkdir(parents=True)
                marker.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
                command = ["/usr/bin/time", "-v", str(args.binary), str(config), str(run), str(epochs)]
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
