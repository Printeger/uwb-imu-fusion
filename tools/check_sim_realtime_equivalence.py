#!/usr/bin/env python3
"""Replay the nominal simulation through the ROS node with fde.profile=off.

The replay bag/process publishes sensor inputs only.  Ground-truth and fault
truth subscriptions are explicitly disabled, and the output tag trajectory is
compared at identical estimator timestamps with the direct nominal runner.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rosgraph
import rospy
import yaml
from sensor_msgs.msg import Imu
from uwb_imu_pl.msg import LinktrackNode2, LinktrackNodeframe3, NavigationIntegrity


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CACHE = ROOT / "results/benchmark/cache/simulation/figure_eight_nominal_seed_20260901"
DEFAULT_DIRECT = ROOT / "results/benchmark/runs/current/simulation/figure_eight_nominal_seed_20260901"
DEFAULT_OUTPUT = ROOT / "results/benchmark/equivalence/simulation"
DEFAULT_NODE = Path("/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/uwb_imu_pl_realtime_node")


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def stamp(seconds):
    ns = round(float(seconds)*1e9)
    return rospy.Time(ns//1000000000, ns%1000000000)


def terminate(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache", type=Path, default=DEFAULT_CACHE)
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--node", type=Path, default=DEFAULT_NODE)
    parser.add_argument("--playback-rate", type=float, default=20.0)
    parser.add_argument("--position-tolerance-m", type=float, default=1e-5)
    parser.add_argument("--rotation-tolerance-deg", type=float, default=1e-5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    bootstrap = json.loads((args.direct / "bootstrap.json").read_text())
    config = yaml.safe_load((args.direct / "effective_nominal_config.yaml").read_text())
    config["realtime"]["initial_position_m"] = bootstrap["position_world_m"]
    config["realtime"]["initial_velocity_mps"] = bootstrap["velocity_world_mps"]
    # The ROS adapter combines configured sensor and anchor uncertainty.  The
    # direct cache already contains the desired total sigma of 0.1 m.
    anchor_sigma = max(float(item["sigma_m"]) for item in config["anchors"])
    total_sigma = float(rows(args.cache / "uwb.csv")[0]["sigma"])
    config["realtime"]["range_sigma_m"] = math.sqrt(
        max(0.0, total_sigma * total_sigma - anchor_sigma * anchor_sigma))
    config["realtime"]["max_queued_events"] = 65536
    replay_config = args.output / "realtime_fde_off_config.yaml"
    replay_config.write_text(yaml.safe_dump(config, sort_keys=False))
    causal = bool(config.get("estimation_tuning", {}).get("causal_bootstrap", False))
    if not causal:
        raise RuntimeError("equivalence requires shared causal_bootstrap; legacy startup is intentionally preserved")
    sensor_duration = float(rows(args.cache / "imu.csv")[-1]["t"])
    worker_start_delay = sensor_duration / args.playback_rate + 3.0

    env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:11389",
               ROS_HOSTNAME="127.0.0.1", OMP_NUM_THREADS="1",
               OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1",
               EIGEN_DONT_PARALLELIZE="1",
               LD_LIBRARY_PATH=str(args.node.parent.parent)+":"+os.environ.get("LD_LIBRARY_PATH", ""))
    roscore = subprocess.Popen(["roscore", "-p", "11389"], env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True)
    node = None
    received = []
    try:
        os.environ.update({"ROS_MASTER_URI": env["ROS_MASTER_URI"],
                           "ROS_HOSTNAME": env["ROS_HOSTNAME"]})
        deadline = time.time() + 15
        while time.time() < deadline and not rosgraph.is_master_online():
            time.sleep(.1)
        if not rosgraph.is_master_online():
            raise RuntimeError("ROS master did not start")
        node = subprocess.Popen([
            str(args.node), "__name:=uwb_imu_pl_realtime_equivalence",
            f"_config_path:={replay_config}", "_fde_profile:=off",
            "_enable_run_logging:=false", "_subscribe_truth_topics:=false",
            f"_worker_start_delay_s:={worker_start_delay}"],
            # Delay only the worker, not ROS callbacks, so the two input topic
            # streams are fully merged by the node's deterministic event key.
            # This is an offline equivalence barrier, not a production mode.
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        rospy.init_node("nominal_equivalence_replay", anonymous=True,
                        disable_signals=True)

        lever = np.asarray(bootstrap["lever_arm_body_m"], dtype=float)

        def solution(message):
            if not message.batch_committed or not message.state_valid:
                return
            pose = message.odometry.pose.pose
            q = np.asarray([pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w])
            # Quaternion rotation, written explicitly to avoid another runtime
            # dependency in this ROS-side audit tool.
            xyz = lever
            u, s = q[:3], q[3]
            rotated = 2 * np.dot(u, xyz) * u + (s*s - np.dot(u, u))*xyz + 2*s*np.cross(u, xyz)
            received.append([message.state_timestamp.to_sec(),
                             pose.position.x + rotated[0],
                             pose.position.y + rotated[1],
                             pose.position.z + rotated[2], *q])

        rospy.Subscriber("/uwb_imu_pl/solution", NavigationIntegrity, solution,
                         queue_size=10000)
        imu_pub = rospy.Publisher(config["realtime"]["imu_topic"], Imu,
                                  queue_size=10000)
        uwb_pub = rospy.Publisher(config["realtime"]["uwb_topic"],
                                  LinktrackNodeframe3, queue_size=3000)
        deadline = time.time() + 15
        while time.time() < deadline and (imu_pub.get_num_connections() < 1 or
                                           uwb_pub.get_num_connections() < 1):
            if node.poll() is not None:
                raise RuntimeError("realtime node exited before replay")
            time.sleep(.1)
        if imu_pub.get_num_connections() < 1 or uwb_pub.get_num_connections() < 1:
            raise RuntimeError("realtime node sensor subscriptions unavailable")

        imu_rows = rows(args.cache / "imu.csv")
        grouped = {}
        for item in rows(args.cache / "uwb.csv"):
            grouped.setdefault(int(item["source_message"]), []).append(item)
        events = [(float(item["t"]), 0, item) for item in imu_rows]
        for group in grouped.values():
            events.append((float(group[0]["t"]), 1, group))
        events.sort(key=lambda item: (item[0], item[1]))
        start_wall, previous_t = time.monotonic(), events[0][0]
        for event_t, kind, payload in events:
            delay = (event_t - previous_t) / args.playback_rate
            if delay > 0:
                time.sleep(delay)
            previous_t = event_t
            if kind == 0:
                message = Imu(); message.header.stamp = stamp(event_t)
                message.linear_acceleration.x = float(payload["ax"])
                message.linear_acceleration.y = float(payload["ay"])
                message.linear_acceleration.z = float(payload["az"])
                message.angular_velocity.x = float(payload["gx"])
                message.angular_velocity.y = float(payload["gy"])
                message.angular_velocity.z = float(payload["gz"])
                imu_pub.publish(message)
            else:
                message = LinktrackNodeframe3(); message.header.stamp = stamp(event_t)
                for item in payload:
                    value = LinktrackNode2(); value.id = int(item["anchor_id"])
                    value.dis = float(item["range"]); message.nodes.append(value)
                uwb_pub.publish(message)
        expected = sum(round(float(group[0]["t"])*1e9) > round(bootstrap["timestamp_s"]*1e9)
                       for group in grouped.values())
        deadline = time.time() + 60
        while time.time() < deadline and len(received) < expected:
            if node.poll() is not None:
                break
            time.sleep(.1)
        elapsed = time.monotonic() - start_wall

        direct = np.loadtxt(args.direct / "trajectory.tum", ndmin=2)
        online = np.asarray(received, dtype=float)
        np.savetxt(args.output / "trajectory.tum", online, fmt="%.17g")
        if len(online) != len(direct):
            reason = f"sample count mismatch direct={len(direct)} realtime={len(online)}"
            position_max = rotation_max = None
            status = "FAIL"
        else:
            time_error = np.max(np.abs(direct[:, 0] - online[:, 0]))
            position_error = np.linalg.norm(direct[:, 1:4] - online[:, 1:4], axis=1)
            dot = np.abs(np.sum(direct[:, 4:8] * online[:, 4:8], axis=1))
            rotation_error = np.rad2deg(2*np.arccos(np.clip(dot, -1, 1)))
            position_max = float(np.max(position_error))
            rotation_max = float(np.max(rotation_error))
            status = "PASS" if (time_error < 1e-9 and
                                  position_max <= args.position_tolerance_m and
                                  rotation_max <= args.rotation_tolerance_deg) else "FAIL"
            reason = "" if status == "PASS" else f"numeric mismatch, max dt={time_error}"
        master = rosgraph.Master(rospy.get_name())
        subscriptions = dict(master.getSystemState()[1]).get(
            "/uwb_imu_pl_realtime_equivalence", [])
        forbidden = [topic for topic in subscriptions
                     if topic in ("/sim/odom", "/uwb_sim/fault_truth")]
        if forbidden:
            status, reason = "FAIL", "forbidden GT subscriptions: " + ",".join(forbidden)
        result = {"schema": "nominal-realtime-equivalence/v1", "status": status,
                  "reason": reason, "direct_samples": int(len(direct)),
                  "realtime_samples": int(len(online)),
                  "max_tag_position_difference_m": position_max,
                  "max_orientation_difference_deg": rotation_max,
                  "position_tolerance_m": args.position_tolerance_m,
                  "rotation_tolerance_deg": args.rotation_tolerance_deg,
                  "fde_profile": "off", "truth_topics_published": 0,
                  "forbidden_truth_subscriptions": forbidden,
                  "playback_rate": args.playback_rate, "wall_s": elapsed}
        (args.output / "equivalence.json").write_text(json.dumps(result, indent=2)+"\n")
        print(json.dumps(result, indent=2))
        return 0 if status == "PASS" else 1
    finally:
        terminate(node)
        terminate(roscore)
        if node and node.stdout:
            (args.output / "realtime_node.log").write_text(node.stdout.read())
        if roscore.stdout:
            (args.output / "roscore.log").write_text(roscore.stdout.read())


if __name__ == "__main__":
    raise SystemExit(main())
