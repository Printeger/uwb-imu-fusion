#!/usr/bin/env python3
"""Build and run the pinned, unmodified SFUISE baseline in isolation."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import resource
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

import benchmark_common as common


ROOT = common.ROOT
SFUISE_ROOT = ROOT.parent / "SFUISE"
PIN = "75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d"
DEFAULT_OUTPUT = ROOT / "results/benchmark"
EPOCH_OFFSET_S = 1000.0


def checked_pin() -> str:
    commit = subprocess.check_output(
        ["git", "-C", str(SFUISE_ROOT), "rev-parse", "HEAD"], text=True).strip()
    if commit != PIN:
        raise RuntimeError(f"SFUISE commit mismatch: expected {PIN}, observed {commit}")
    dirty = subprocess.check_output(
        ["git", "-C", str(SFUISE_ROOT), "status", "--porcelain"], text=True)
    if dirty:
        raise RuntimeError("SFUISE source tree is dirty; baseline must remain read-only")
    return commit


def workspace(output: Path) -> Path:
    # A catkin workspace cannot be nested inside this repository because the
    # parent itself is a package.  Keep the isolated build beside the main
    # workspace while all run artifacts remain under results/benchmark.
    return ROOT.parents[1] / ".uwb_benchmark/sfuise_isolated_ws"


def build(output: Path) -> Path:
    checked_pin()
    ws = workspace(output)
    src = ws / "src"
    src.mkdir(parents=True, exist_ok=True)
    links = {
        "cf_msgs": SFUISE_ROOT / "cf_msgs",
        "isas_msgs": SFUISE_ROOT / "isas_msgs",
        "sfuise_msgs": SFUISE_ROOT / "sfuise_msgs",
        "sfuise": SFUISE_ROOT / "sfuise",
        "uwb_benchmark_sfuise_adapter": ROOT / "benchmark/sfuise_adapter_ros",
    }
    for name, target in links.items():
        link = src / name
        if link.is_symlink() and link.resolve() == target.resolve():
            continue
        if link.exists() or link.is_symlink():
            raise RuntimeError(f"unexpected isolated-workspace entry: {link}")
        link.symlink_to(target, target_is_directory=True)
    # Upstream unconditionally resets CMAKE_BUILD_TYPE to Debug after project().
    # Keep the source pinned/read-only and make that configuration
    # release-equivalent for benchmarking through the documented cache flag.
    cmake_args = ["-DCMAKE_BUILD_TYPE=Release",
                  "-DCMAKE_CXX_FLAGS_DEBUG=-O3 -DNDEBUG",
                  f"-DSFUISE_INCLUDE_DIR={SFUISE_ROOT / 'sfuise/include'}"]
    message_command = ["catkin_make", "-C", str(ws), "--pkg", "cf_msgs",
                       "isas_msgs", "sfuise_msgs", *cmake_args, "-j2"]
    command = ["catkin_make", "-C", str(ws), *cmake_args, "-j2"]
    log = output / "logs/sfuise_build.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as stream:
        environment = dict(os.environ, PWD=str(ws))
        messages = subprocess.run(message_command, cwd=ws, env=environment,
                                  stdout=stream, stderr=subprocess.STDOUT, text=True)
        if messages.returncode:
            raise RuntimeError(f"isolated SFUISE message build failed; see {log}")
        process = subprocess.run(command, cwd=ws, env=environment, stdout=stream,
                                 stderr=subprocess.STDOUT, text=True)
    if process.returncode:
        raise RuntimeError(f"isolated SFUISE build failed; see {log}")
    setup = ws / "devel/setup.bash"
    if not setup.is_file():
        raise RuntimeError("isolated SFUISE setup.bash missing")
    return setup


def source_environment(setup: Path) -> dict[str, str]:
    command = ["bash", "-c", f"source {shlex_quote(str(setup))} && env -0"]
    raw = subprocess.check_output(command)
    env = {}
    for item in raw.split(b"\0"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            env[key.decode()] = value.decode()
    return env


def shlex_quote(value: str) -> str:
    import shlex
    return shlex.quote(value)


def _write_bag(cache: Path, destination: Path, lossless_toa: bool = False):
    # Imports intentionally happen only in the subprocess sourced from the
    # isolated workspace so custom message resolution cannot leak from the
    # user's main catkin overlay.
    import rosbag
    import rospy
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64MultiArray
    from isas_msgs.msg import AnchorPosition, Anchorlist, RTLSRange, RTLSStick

    manifest = json.loads((cache / "manifest.json").read_text())
    primary = int(manifest["primary_tag"])
    with (cache / "anchors.csv").open() as stream:
        anchors = list(csv.DictReader(stream))
    with (cache / "imu.csv").open() as stream:
        imu_rows = list(csv.DictReader(stream))
    with (cache / "uwb.csv").open() as stream:
        range_rows = [row for row in csv.DictReader(stream)
                      if int(row["tag_id"]) == primary]
    # Reconstruct the same physical epochs used by the direct nominal runner.
    # Several public CSVs serialize one anchor per row/message, whereas
    # SFUISE's RTLSStick contract expects the contemporaneous anchor set.
    batch_window = float(manifest.get("epoch_batch_window_s", 0.0))
    groups = []
    begin = 0
    while begin < len(range_rows):
        end = begin + 1
        while end < len(range_rows) and (
                int(range_rows[end]["source_message"]) ==
                int(range_rows[begin]["source_message"]) or
                float(range_rows[end]["t"])-float(range_rows[begin]["t"])
                <= batch_window + 1e-12):
            end += 1
        groups.append(range_rows[begin:end])
        begin = end
    destination.parent.mkdir(parents=True, exist_ok=True)
    def exact_stamp(t):
        value_ns = float(t) * 1e9
        # Match TimestampNs::fromSeconds/std::round (ties away from zero).
        normalized_ns = math.floor(value_ns + .5) if value_ns >= 0 else math.ceil(value_ns - .5)
        ns = round(EPOCH_OFFSET_S * 1e9) + normalized_ns
        return rospy.Time(ns // 1000000000, ns % 1000000000)
    def row_time(row, column, fallback):
        if column not in row:
            return fallback
        ns = int(row[column]) + round(EPOCH_OFFSET_S * 1e9)
        return rospy.Time(ns // 1000000000, ns % 1000000000)
    with rosbag.Bag(str(destination), "w") as bag:
        anchor_msg = Anchorlist()
        for row in anchors:
            item = AnchorPosition(id=int(row["anchor_id"]))
            item.position = Point(float(row["x"]), float(row["y"]), float(row["z"]))
            anchor_msg.anchor.append(item)
        # The generic cache can begin with a negative IMU timestamp or with
        # UWB well before IMU. The upstream solver assumes both buffers are
        # populated when the static map unlocks initialization. Keep every
        # original sensor stamp/value; delay only static-map readiness under
        # one common protocol, with its real startup wait counted in playback.
        first_valid_range = min(float(r["t"]) for r in range_rows
                                if int(r["valid"]) != 0)
        map_ready = max(float(imu_rows[0]["t"]), first_valid_range) + .025
        for index in range(20):
            stamp = exact_stamp(map_ready + index * 1e-3)
            bag.write("/EstimationInterface/anchor_list", anchor_msg, stamp)
        for row in imu_rows:
            stamp = row_time(row, "_stamp_ns", exact_stamp(row["t"]))
            msg = Imu()
            msg.header.stamp = stamp
            msg.linear_acceleration.x = float(row["ax"])
            msg.linear_acceleration.y = float(row["ay"])
            msg.linear_acceleration.z = float(row["az"])
            msg.angular_velocity.x = float(row["gx"])
            msg.angular_velocity.y = float(row["gy"])
            msg.angular_velocity.z = float(row["gz"])
            bag.write("/EstimationInterface/imu_ds", msg, row_time(row, "_received_ns", stamp))
        for group in groups:
            stamp = row_time(group[-1], "_stamp_ns", exact_stamp(group[-1]["t"]))
            received = row_time(group[-1], "_received_ns", stamp)
            if "_stamp_ns" in group[-1]:
                # Upstream TOAData has one timestamp per batch. Reject an
                # unsupported asynchronous batch instead of silently retiming.
                if any(row_time(r, "_stamp_ns", stamp) != stamp or
                       row_time(r, "_received_ns", received) != received for r in group):
                    raise ValueError("SFUISE canonical batch has unequal per-row times")
            msg = RTLSStick()
            msg.header.stamp = stamp
            msg.id = primary
            for row in group:
                item = RTLSRange()
                item.id = int(row["anchor_id"])
                # Native SFUISE data keeps the official offset in its YAML.
                # Other adapters already froze known offsets into `range`.
                item.range = float(row["range_raw"] if manifest["dataset"] == "SFUISE" and not lossless_toa
                                   else row["range"])
                item.ra = int(row["valid"])
                item.pos = Point(float(row["anchor_x"]), float(row["anchor_y"]),
                                 float(row["anchor_z"]))
                msg.ranges.append(item)
            msg.noga = len(msg.ranges)
            msg.nora = sum(int(item.ra != 0) for item in msg.ranges)
            bag.write("/EstimationInterface/toa_ds", msg, received)
            if lossless_toa:
                # Official RTLSRange.range is float32, while the frozen raw
                # simulation and calibrated CSV ranges are doubles. Feed the
                # identical double observations into unchanged TOAData math.
                # The legacy message remains an observer-only timestamp feed;
                # the observed estimator explicitly unsubscribes from it.
                payload = [float(stamp.to_nsec()), float(len(group))]
                for row in group:
                    observed = row["range"]
                    payload.extend([float(row["anchor_id"]), float(observed), float(row["valid"])])
                bag.write("/EstimationInterface/toa_ds64", Float64MultiArray(data=payload), received)
    return {"imu": len(imu_rows), "uwb_messages": len(groups),
            "uwb_ranges": len(range_rows), "anchors": len(anchors),
            "map_ready_sensor_s": map_ready,
            "toa_transport": "FLOAT64_LOSSLESS" if lossless_toa else "OFFICIAL_FLOAT32",
            "toa_calibration": "CANONICAL_RANGE_PREAPPLIED_ONCE" if lossless_toa else "OFFICIAL_NATIVE_OFFSET",
            "clock_transport": "INTEGER_HEADER_AND_ORIGINAL_RECEIVE" if "_stamp_ns" in imu_rows[0] else "LEGACY_CSV_HEADER_RELEASE",
            "map_ready_policy": "both_sensor_streams_then_25ms; original sensor stamps unchanged"}


def write_bag_subprocess(setup: Path, cache: Path, bag: Path, lossless_toa: bool = False) -> dict:
    command = ["bash", "-c",
               f"source {shlex_quote(str(setup))} && "
               f"python3 {shlex_quote(str(Path(__file__).resolve()))} _write-bag "
               f"--cache {shlex_quote(str(cache))} --bag {shlex_quote(str(bag))}" +
               (" --toa64" if lossless_toa else "")]
    result = subprocess.check_output(command, text=True)
    return json.loads(result.strip().splitlines()[-1])


def frequency(path: Path) -> float:
    with path.open() as stream:
        times = np.asarray([float(row["t"]) for row in csv.DictReader(stream)])
    delta = np.diff(np.unique(times))
    delta = delta[(delta > 0) & np.isfinite(delta)]
    return float(1.0 / np.median(delta)) if delta.size else 1.0


def runtime_config(cache: Path, destination: Path, lossless_toa: bool = False) -> tuple[dict, list[float]]:
    manifest = json.loads((cache / "manifest.json").read_text())
    dataset = manifest["dataset"]
    source_name = "config_test_isas-walk1.yaml"
    if dataset == "SFUISE":
        source_name = {
            "ISAS-Walk1": "config_test_isas-walk1.yaml",
            "ISAS-Walk2": "config_test_isas-walk2.yaml",
            "ISAS-Walk3": "config_test_isas-walk3.yaml",
        }[manifest["sequence"]]
    config = yaml.safe_load((SFUISE_ROOT / "sfuise/config" / source_name).read_text())
    with (cache / "uwb.csv").open() as stream:
        uwb = list(csv.DictReader(stream))
    primary = int(manifest["primary_tag"])
    primary_rows = [row for row in uwb if int(row["tag_id"]) == primary]
    lever = [float(primary_rows[0][name]) for name in ("lever_x", "lever_y", "lever_z")]
    anchor_ids = sorted({int(row["anchor_id"]) for row in primary_rows})
    native_offsets = manifest.get("provenance", {}).get("toa_offset_by_anchor", {})
    batch_window = float(manifest.get("epoch_batch_window_s", 0.0))
    uwb_frequency = (1.0 / batch_window if batch_window > 0 else
                     frequency(cache / "uwb.csv"))
    config.update(topic_imu="/benchmark/imu", topic_uwb="/rtls_flares",
                  topic_anchor_list="/anchor_list", topic_ground_truth="/NO_GT",
                  imu_frequency=max(1, int(round(frequency(cache / "imu.csv")))),
                  uwb_frequency=max(1, int(round(uwb_frequency))),
                  imu_sample_coeff=1, uwb_sample_coeff=1, acc_ratio=False,
                  gyro_unit=False, if_tdoa=False, if_reject_uwb=True,
                  offset=lever)
    if dataset == "SFUISE" and not lossless_toa:
        config["toa_offset"] = [float(native_offsets.get(str(aid),
                                     native_offsets.get(aid, 0.0))) for aid in anchor_ids]
    else:
        config["toa_offset"] = [0.0] * len(anchor_ids)
    destination.write_text(yaml.safe_dump(config, sort_keys=False))
    return config, lever


def terminate(processes):
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 8.0
    for process in reversed(processes):
        try:
            process.wait(timeout=max(0.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def normalize_trajectory(path: Path):
    if not path.is_file():
        return
    rows = []
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) == 8:
            fields[0] = f"{float(fields[0]) - EPOCH_OFFSET_S:.17g}"
            rows.append(" ".join(fields))
    path.write_text("\n".join(rows) + ("\n" if rows else ""))


def run_one(setup: Path, cache: Path, run: Path, playback_rate: float = 2.0,
            fusion_executable: str = "SplineFusion", observer: bool = False) -> dict:
    manifest = json.loads((cache / "manifest.json").read_text())
    run.mkdir(parents=True, exist_ok=True)
    for stale in ("trajectory.tum", "adapter.meta", "run_status.json"):
        path = run/stale
        if path.exists():
            path.unlink()
    bag = run / "input.bag"
    counts = write_bag_subprocess(setup, cache, bag, lossless_toa=observer)
    config, lever = runtime_config(cache, run / "sfuise.yaml", lossless_toa=observer)
    env = source_environment(setup)
    env.update(OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
    if observer:
        env['SC8_SFUISE_OBSERVER'] = str(run / 'observer.csv')
        env['SC8_SFUISE_TOA64'] = '1'
    logs = []
    processes = []
    fusion_process = None
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    start = time.monotonic()
    try:
        for name, command in (
            ("roscore", ["roscore", "-p", "11342"]),
        ):
            stream = (run / f"{name}.log").open("w"); logs.append(stream)
            processes.append(subprocess.Popen(command, env=dict(env, ROS_MASTER_URI="http://127.0.0.1:11342"),
                                              stdout=stream, stderr=subprocess.STDOUT))
        time.sleep(2)
        run_env = dict(env, ROS_MASTER_URI="http://127.0.0.1:11342")
        for namespace in ("SplineFusion",):
            subprocess.check_call(["rosparam", "load", str(run / "sfuise.yaml"),
                                   f"/{namespace}"], env=run_env)
        commands = [
            ("fusion", ["rosrun", "uwb_benchmark_sfuise_adapter" if observer else "sfuise", fusion_executable,
                        "__name:=SplineFusion"]),
            ("adapter", ["rosrun", "uwb_benchmark_sfuise_adapter",
                         "sfuise_trajectory_adapter",
                         f"_output_path:={run / 'trajectory.tum'}",
                         f"_metadata_path:={run / 'adapter.meta'}",
                         f"_playback_rate:={playback_rate}",
                         "_lever_arm_body_m:=[" + ",".join(map(str, lever)) + "]"]),
        ]
        for name, command in commands:
            stream = (run / f"{name}.log").open("w"); logs.append(stream)
            process = subprocess.Popen(command, env=run_env, stdout=stream,
                                       stderr=subprocess.STDOUT)
            processes.append(process)
            if name == "fusion":
                fusion_process = process
        time.sleep(2)
        playback = subprocess.run(["rosbag", "play", "--clock", "--hz=1000", "--delay=1",
                                   "--rate", str(playback_rate),
                                   str(bag)], env=run_env, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True)
        (run / "play.log").write_text(playback.stdout)
        time.sleep(5)
        if playback.returncode:
            raise RuntimeError("rosbag playback failed")
    finally:
        terminate(processes)
        for stream in logs:
            stream.close()
    wall_s = time.monotonic() - start
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu_s = (after.ru_utime + after.ru_stime) - (before.ru_utime + before.ru_stime)
    normalize_trajectory(run / "trajectory.tum")
    normalize_trajectory(run / "trajectory.tum.online.tum")
    for view in ("online_final_calibration", "history_online_calibration", "history_final_calibration"):
        normalize_trajectory(run / ("trajectory.tum."+view+".tum"))
    samples = sum(1 for _ in (run / "trajectory.tum").open()) if (run / "trajectory.tum").is_file() else 0
    metadata = {}
    metadata_path = run/"adapter.meta"
    if metadata_path.is_file():
        for line in metadata_path.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1); metadata[key] = value
    received_toa = int(metadata.get("toa_timestamps", 0))
    count_match = received_toa == counts["uwb_messages"]
    solver_final = {}
    if observer:
        final_path=run/'observer.csv.final.json'
        if final_path.exists():solver_final=json.loads(final_path.read_text())
        count_match = count_match and solver_final.get('imu_received')==counts['imu'] and solver_final.get('toa_received')==counts['uwb_messages']
    fusion_returncode = fusion_process.returncode if fusion_process else None
    status = "SUCCESS" if samples >= 2 and count_match and fusion_returncode in (0, -signal.SIGINT) else "FAIL"
    if status == "SUCCESS":
        reason = ""
    elif fusion_returncode not in (None, 0, -signal.SIGINT):
        reason = f"SFUISE_PROCESS_EXIT_{fusion_returncode}"
    elif not count_match:
        reason = "PUBLISHED_RECEIVED_COUNT_MISMATCH"
    else:
        reason = "EMPTY_EXPORTED_TRAJECTORY"
    result = {"schema": "sfuise-run/v1", "status": status,
              "reason": reason,
              "dataset": manifest["dataset"], "sequence": manifest["sequence"],
              "sfuise_commit": checked_pin(), "source_tree_modified": False,
              "gt_topics_published": 0, "input_counts": counts,
              "trajectory_samples": samples, "wall_s": wall_s, "cpu_s": cpu_s,
              "received_uwb_messages": received_toa,
              "fusion_returncode": fusion_returncode,
              "cpu_affinity": ",".join(map(str, sorted(os.sched_getaffinity(0)))),
              "build_flags": "-O3 -DNDEBUG (upstream forces Debug type)",
              "playback_rate": playback_rate,
              "native_average_window_runtime_ms": float(
                  metadata.get("native_average_window_runtime", "nan")),
              "realtime_factor": wall_s / max(1e-9, common.csv_duration(cache / "imu.csv")),
              "official_uwb_rejection": bool(config["if_reject_uwb"])}
    result['solver_final_receipt'] = solver_final
    result['root_sensor_counts_match'] = count_match
    result['observer_enabled'] = observer
    result['fusion_executable'] = fusion_executable
    result['source_clock_resolution_s'] = 1e-9 if 'sc9_canonical' in manifest else .001
    (run / "run_status.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def command_build(args):
    print(build(Path(args.output).resolve()))
    return 0


def command_run(args):
    output = Path(args.output).resolve()
    setup = build(output)
    records = []
    for manifest_path in sorted((output / "cache").glob("*/*/manifest.json")):
        manifest = json.loads(manifest_path.read_text())
        if args.sequence and args.sequence not in (manifest["dataset"], manifest["sequence"]):
            continue
        try:
            record = run_one(setup, manifest_path.parent,
                             output / "runs/sfuise" / manifest["dataset"] / manifest["sequence"],
                             args.playback_rate)
        except Exception as error:
            record = {"dataset": manifest["dataset"], "sequence": manifest["sequence"],
                      "status": "FAIL", "reason": type(error).__name__ + ": " + str(error)}
        records.append(record)
        print(json.dumps(record), flush=True)
    indexed = {(row["dataset"], row["sequence"]): row for row in records}
    for status_path in sorted((output/"runs/sfuise").glob("*/*/run_status.json")):
        dataset, sequence = status_path.parent.parent.name, status_path.parent.name
        if (dataset, sequence) not in indexed:
            indexed[(dataset, sequence)] = json.loads(status_path.read_text())
    all_records = [indexed[key] for key in sorted(indexed)]
    (output / "execution_sfuise.json").write_text(json.dumps({"records": all_records}, indent=2) + "\n")
    return 0 if records and all(row["status"] == "SUCCESS" for row in records) else 1


def command_timing(args):
    output = Path(args.output).resolve(); setup = build(output)
    records = []
    representatives = []
    for manifest_path in sorted((output/"cache").glob("*/*/manifest.json")):
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("timing_representative"):
            representatives.append((manifest_path, manifest))
    for manifest_path, manifest in representatives:
        warmup_failure = None
        for repeat in range(6):
            run = output/"timing/sfuise"/manifest["dataset"]/manifest["sequence"]/("warmup" if repeat == 0 else f"repeat_{repeat}")
            if repeat == 0:
                natural_status = (output/"runs/sfuise"/manifest["dataset"]/
                                  manifest["sequence"]/"run_status.json")
                if natural_status.is_file():
                    row = json.loads(natural_status.read_text())
                    if float(row.get("playback_rate", -1)) == float(args.playback_rate):
                        run.mkdir(parents=True, exist_ok=True)
                        (run/"warmup_reference.json").write_text(json.dumps({
                            "schema": "sfuise-timing-warmup-reference/v1",
                            "source": str(natural_status), "run": row}, indent=2)+"\n")
                        if row.get("status") != "SUCCESS":
                            warmup_failure = row.get("reason", "unknown")
                        continue
            if repeat and warmup_failure is not None:
                row = {"status": "FAIL",
                       "reason": "WARMUP_FAILED: " + warmup_failure,
                       "dataset": manifest["dataset"],
                       "sequence": manifest["sequence"], "repeat": repeat}
                records.append(row); print(json.dumps(row), flush=True)
                continue
            try:
                row = run_one(setup, manifest_path.parent, run, args.playback_rate)
            except Exception as error:
                row = {"status": "FAIL", "reason": type(error).__name__+": "+str(error),
                       "dataset": manifest["dataset"], "sequence": manifest["sequence"]}
            row["repeat"] = repeat
            if repeat == 0 and row.get("status") != "SUCCESS":
                warmup_failure = row.get("reason", "unknown")
            if repeat:
                records.append(row); print(json.dumps(row), flush=True)
    target = output/"timing/sfuise_summary.json"; target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps({"schema": "sfuise-timing/v1", "records": records}, indent=2)+"\n")
    fields = sorted({key for row in records for key in row})
    common.write_csv(output/"timing/sfuise_summary.csv", fields, records)
    return 0 if records and all(row["status"] == "SUCCESS" for row in records) else 1


def parser():
    result = argparse.ArgumentParser()
    sub = result.add_subparsers(dest="command", required=True)
    for name in ("build", "run", "timing"):
        item = sub.add_parser(name)
        item.add_argument("--output", default=str(DEFAULT_OUTPUT))
        if name in ("run", "timing"):
            item.add_argument("--playback-rate", type=float, default=2.0)
        if name == "run":
            item.add_argument("--sequence")
    hidden = sub.add_parser("_write-bag")
    hidden.add_argument("--cache", required=True)
    hidden.add_argument("--bag", required=True)
    hidden.add_argument("--toa64", action="store_true")
    return result


def main():
    args = parser().parse_args()
    if args.command == "_write-bag":
        print(json.dumps(_write_bag(Path(args.cache), Path(args.bag), args.toa64)))
        return 0
    return globals()["command_" + args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
