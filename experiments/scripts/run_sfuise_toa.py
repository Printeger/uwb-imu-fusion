#!/usr/bin/env python3
"""Run unmodified SFUISE ToA on ISAS Walk1/2/3 without playing GT."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[2]
SFUISE = ROOT / "experiments/compare_algorithm/SFUISE"
ADAPTER = ROOT / "experiments/compare_algorithm/sfuise_adapter_ros"
PRIVATE = ROOT.parents[1] / "evaluator_private/icra/sfuise_baseline"
BUILD_WS = PRIVATE / "build_ws"
EXPECTED_COMMIT = "75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d"
SEQUENCES = (1, 2, 3)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def sourced_environment(setup):
    command = ["bash", "-c", "source \"$1\" && env -0", "bash", str(setup)]
    raw = subprocess.check_output(command)
    return dict(item.split("=", 1) for item in raw.decode().split("\0") if item)


def ensure_link(path, target):
    if path.is_symlink() and path.resolve() == target.resolve():
        return
    if path.exists() or path.is_symlink():
        raise RuntimeError("build workspace path exists with unexpected identity: " + str(path))
    path.symlink_to(target, target_is_directory=True)


def build():
    if subprocess.check_output(["git", "-C", str(SFUISE), "rev-parse", "HEAD"], text=True).strip() != EXPECTED_COMMIT:
        raise RuntimeError("unexpected SFUISE commit")
    status = subprocess.check_output(
        ["git", "-C", str(SFUISE), "status", "--porcelain"], text=True).splitlines()
    allowed_missing_data = {" D dataset/ISAS-Walk%d.bag" % sequence for sequence in SEQUENCES}
    if set(status) - allowed_missing_data:
        raise RuntimeError("SFUISE source/config checkout is modified: " + repr(status))
    source = BUILD_WS / "src"
    source.mkdir(parents=True, exist_ok=True)
    for name in ("cf_msgs", "isas_msgs", "sfuise_msgs", "sfuise"):
        ensure_link(source / name, SFUISE / name)
    ensure_link(source / "sfuise_baseline_adapter", ADAPTER)
    log = PRIVATE / "build.log"
    command = ["catkin_make", "-DCMAKE_BUILD_TYPE=Release", "-DSFUISE_SOURCE_DIR=" + str(SFUISE)]
    build_env = dict(os.environ)
    # catkin_make consults PWD while locating its workspace.  subprocess cwd=
    # does not rewrite an inherited PWD, so make that relationship explicit.
    build_env["PWD"] = str(BUILD_WS)
    with log.open("w") as stream:
        result = subprocess.run(command, cwd=BUILD_WS, env=build_env,
                                stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError("SFUISE adapter build failed; see " + str(log))
    return sourced_environment(BUILD_WS / "devel/setup.bash"), command, log


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def wait_master(env):
    for _ in range(50):
        if subprocess.run(["rosnode", "list"], env=env, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            return
        time.sleep(0.1)
    raise RuntimeError("ROS master did not start")


def run_sequence(sequence, root, base_env):
    name = "ISAS-Walk" + str(sequence)
    run = root / name
    run.mkdir()
    bag = ROOT / "data/SFUISE" / (name + ".bag")
    config = SFUISE / "sfuise/config" / ("config_test_isas-walk%d.yaml" % sequence)
    cache = ROOT.parents[1] / ("res/nlos_injection_20260911_01/inputs/sfuise_walk%d_normal_clean/input_manifest.json" % sequence)
    cfg = yaml.safe_load(config.read_text())
    cache_doc = json.loads(cache.read_text())
    if cfg.get("if_tdoa") is not False or cfg.get("topic_uwb") != "/rtls_flares":
        raise RuntimeError("only the official absolute ToA mode is allowed")
    if cache_doc["base_source_sha256"] != "sha256:" + sha(bag):
        raise RuntimeError("SFUISE bag and our estimator clean cache do not share raw source")
    port = 11420 + sequence
    env = dict(base_env, ROS_MASTER_URI="http://127.0.0.1:%d" % port,
               ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1")
    trajectory = run / "trajectory.tum"
    metadata = run / "adapter_runtime.txt"
    logs = {key: (run / (key + ".log")).open("w") for key in ("roscore", "interface", "fusion", "adapter", "rosbag")}
    processes = []
    started = dt.datetime.now(dt.timezone.utc).isoformat()
    start_wall = time.monotonic()
    result = {"schema": "sfuise_toa_run_v1", "run_id": "sfuise-" + uuid.uuid4().hex,
              "sequence": name, "measurement_mode": "ABSOLUTE_TOA", "if_tdoa": False,
              "raw_bag": str(bag), "raw_bag_sha256": sha(bag), "our_cache_manifest": str(cache),
              "our_cache_base_source_sha256": cache_doc["base_source_sha256"],
              "sfuise_commit": EXPECTED_COMMIT, "config": str(config), "config_sha256": sha(config),
              "gt_topic_played": False, "played_topics": [cfg["topic_imu"], cfg["topic_uwb"], cfg["topic_anchor_list"]],
              "detector_or_recovery_input": False, "start_time": started, "status": "RUNNING"}
    write_json(run / "run_status.json", result)
    try:
        master = subprocess.Popen(["roscore", "-p", str(port)], env=env,
                                  stdout=logs["roscore"], stderr=subprocess.STDOUT)
        processes.append(master)
        wait_master(env)
        for namespace in ("/EstimationInterface", "/SplineFusion"):
            subprocess.check_call(["rosparam", "load", str(config), namespace], env=env)
        interface = subprocess.Popen([str(BUILD_WS / "devel/lib/sfuise/EstimationInterface"),
                                      "__name:=EstimationInterface",
                                      "/SplineFusion/sys_calib:=/sfuise_adapter/interface_calib_blocked"], env=env,
                                     stdout=logs["interface"], stderr=subprocess.STDOUT)
        fusion = subprocess.Popen([str(BUILD_WS / "devel/lib/sfuise/SplineFusion"),
                                   "__name:=SplineFusion"], env=env,
                                  stdout=logs["fusion"], stderr=subprocess.STDOUT)
        adapter = subprocess.Popen([str(BUILD_WS / "devel/lib/sfuise_baseline_adapter/sfuise_trajectory_adapter"),
                                    "__name:=sfuise_trajectory_adapter", "_output_path:=" + str(trajectory),
                                    "_metadata_path:=" + str(metadata)], env=env,
                                   stdout=logs["adapter"], stderr=subprocess.STDOUT)
        processes.extend([interface, fusion, adapter])
        time.sleep(2)
        play = subprocess.Popen(["rosbag", "play", "--quiet", str(bag), "--topics",
                                 cfg["topic_imu"], cfg["topic_uwb"], cfg["topic_anchor_list"]],
                                env=env, stdout=logs["rosbag"], stderr=subprocess.STDOUT)
        processes.append(play)
        play_code = play.wait(timeout=180)
        if play_code:
            raise RuntimeError("rosbag play failed with exit %d" % play_code)
        time.sleep(5)
        stop(adapter)
        if not trajectory.is_file() or trajectory.stat().st_size == 0:
            raise RuntimeError("adapter produced no trajectory")
        for process in (fusion, interface):
            if process.poll() not in (None, 0):
                raise RuntimeError("SFUISE process failed before export")
        result.update(status="SUCCESS", exit_code=0, trajectory_path=str(trajectory),
                      trajectory_sha256=sha(trajectory), metadata_path=str(metadata),
                      wall_time_s=time.monotonic() - start_wall)
    except Exception as exc:
        result.update(status="FAILURE", exit_code=1,
                      failure_reason=type(exc).__name__ + ": " + str(exc),
                      wall_time_s=time.monotonic() - start_wall)
    finally:
        for process in reversed(processes):
            stop(process)
        for stream in logs.values():
            stream.close()
        result["end_time"] = dt.datetime.now(dt.timezone.utc).isoformat()
        result["logs"] = {key: str(run / (key + ".log")) for key in logs}
        write_json(run / "run_status.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=ROOT / "experiments/results")
    args = parser.parse_args()
    PRIVATE.mkdir(parents=True, exist_ok=True)
    env, build_command, build_log = build()
    output = args.output_root.resolve() / ("sfuise-toa-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:12])
    output.mkdir(parents=True)
    manifest = {"schema": "sfuise_toa_batch_v1", "output": str(output),
                "build_command": build_command, "build_log": str(build_log), "runs": []}
    for sequence in SEQUENCES:
        manifest["runs"].append(run_sequence(sequence, output, env))
        write_json(output / "batch_manifest.json", manifest)
    ok = all(run["status"] == "SUCCESS" for run in manifest["runs"])
    manifest["status"] = "SUCCESS" if ok else "FAILURE"
    write_json(output / "batch_manifest.json", manifest)
    print(json.dumps({"status": manifest["status"], "output": str(output)}))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
