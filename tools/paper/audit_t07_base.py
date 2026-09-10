#!/usr/bin/env python3
"""Reproduce the T07 base-fixture audit without assigning provenance."""

import argparse
import bisect
import hashlib
import json
import math
import os
import statistics

import rosbag
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def resolve_config_path(config_path, value):
    if os.path.isabs(value):
        return os.path.realpath(value)
    return os.path.realpath(os.path.join(os.path.dirname(config_path), value))


def interpolate(times, positions, timestamp):
    index = bisect.bisect_left(times, timestamp)
    if index == 0:
        return positions[0]
    if index == len(times):
        return positions[-1]
    lo, hi = index - 1, index
    weight = (timestamp - times[lo]) / (times[hi] - times[lo])
    return tuple(positions[lo][axis] * (1.0 - weight) +
                 positions[hi][axis] * weight for axis in range(3))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--simulator-source", required=True)
    parser.add_argument("--simulator-config", required=True)
    args = parser.parse_args()
    config_path = os.path.realpath(args.config)
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    bag_path = resolve_config_path(config_path, config["bag"]["path"])
    anchors = {int(item["id"]): tuple(map(float, item["pos"]))
               for item in config["anchors"]}
    imu_topic = config["topics"]["imu"]
    uwb_topic = config["topics"]["uwb"]
    odom_topic = config["topics"]["gt_odom"]

    imu_times, frames, odom_times, odom_positions = [], [], [], []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[imu_topic, uwb_topic, odom_topic]):
            if topic == imu_topic:
                imu_times.append(message.header.stamp.to_sec())
            elif topic == odom_topic:
                odom_times.append(message.header.stamp.to_sec())
                point = message.pose.pose.position
                odom_positions.append((point.x, point.y, point.z))
            else:
                frames.append(message)
    if not imu_times or not frames or not odom_times:
        raise RuntimeError("base audit requires IMU, UWB and odometry")
    origin = min(min(imu_times), min(frame.header.stamp.to_sec()
                                     for frame in frames))
    counts, residuals = {}, {"los_like": [], "nlos_like": []}
    link_counts = {"1:1": 0, "1:2": 0}
    observations = 0
    for frame in frames:
        timestamp = frame.header.stamp.to_sec()
        position = interpolate(odom_times, odom_positions, timestamp)
        relative = timestamp - origin
        for node in frame.nodes:
            observations += 1
            difference = round(float(node.rx_rssi - node.fp_rssi), 6)
            key = f"{difference:.1f}"
            counts[key] = counts.get(key, 0) + 1
            link = f"{int(frame.id)}:{int(node.id)}"
            if 8.0 <= relative < 12.0 and link in link_counts:
                link_counts[link] += 1
            anchor = anchors[int(node.id)]
            geometric = math.sqrt(sum((position[axis] - anchor[axis]) ** 2
                                      for axis in range(3)))
            label = "nlos_like" if difference > 6.0 else "los_like"
            residuals[label].append(float(node.dis) - geometric)

    stat = os.stat(bag_path)
    output = {
        "schema": "t07_base_audit_observation_v1",
        "interpretation": "EMPIRICAL_CONSISTENCY_NOT_GENERATION_PROVENANCE",
        "base_source": {
            "path": bag_path,
            "size_bytes": stat.st_size,
            "sha256": sha256(bag_path),
        },
        "candidate_files": {
            "simulator_source_sha256": sha256(args.simulator_source),
            "simulator_config_sha256": sha256(args.simulator_config),
        },
        "selection": {
            "event_window": "sensor_time-origin in [8.0,12.0)",
            "links": ["1:1", "1:2"],
            "rssi_class": "rx_rssi-fp_rssi > 6 dB is NLOS-like",
            "residual_reference": "publish-time linearly interpolated /sim/odom",
        },
        "recording_time_origin_s": origin,
        "uwb_message_count": len(frames),
        "uwb_observation_count": observations,
        "event_link_counts": link_counts,
        "rssi_difference_counts": counts,
        "los_like_residual_mean_m": statistics.mean(residuals["los_like"]),
        "los_like_residual_std_m": statistics.pstdev(residuals["los_like"]),
        "nlos_like_residual_mean_m": statistics.mean(residuals["nlos_like"]),
        "total_latent_bias_status": "UNKNOWN",
    }
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
