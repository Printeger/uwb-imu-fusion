#!/usr/bin/env python3
"""Publish deterministic timestamped IMU/UWB/fault-truth Week-4 scenarios."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from dataclasses import asdict, dataclass
from typing import Iterator


ANCHORS = ((-5., -5., 1.), (-5., -5., 5.), (-5., 5., 1.), (-5., 5., 5.),
           (5., -5., 1.), (5., -5., 5.), (5., 5., 1.), (5., 5., 5.))


@dataclass(frozen=True)
class Epoch:
    index: int
    timestamp_ns: int
    publish_uwb: bool
    anchor_count: int
    fault_anchor_id: int
    fault_mode: str
    fault_active: bool
    outage: bool
    injected_bias_m: float


def schedule(scenario: str, epochs: int, start_ns: int = 1_000_000_000,
             period_ns: int = 50_000_000) -> list[Epoch]:
    valid = {"delayed_stale_uwb", "uwb_drop_resume", "imu_gap",
             "anchor_set_8_7_8", "fault_step", "fault_ramp",
             "fault_magnitude_sweep", "fault_outage"}
    if scenario not in valid or epochs < 12:
        raise ValueError("unknown scenario or fewer than 12 epochs")
    output = []
    for index in range(epochs):
        stamp = start_ns + index*period_ns
        publish_uwb = not (scenario == "uwb_drop_resume" and epochs//3 <= index < 2*epochs//3)
        anchor_count = 7 if scenario == "anchor_set_8_7_8" and \
            epochs//3 <= index < 2*epochs//3 else 8
        active = scenario.startswith("fault_") and index >= epochs//3
        mode = scenario[len("fault_"):] if scenario.startswith("fault_") else "none"
        outage = active and mode == "outage"
        if mode == "step": magnitude = 1.0 if active else 0.0
        elif mode == "ramp": magnitude = max(0.0, (index-epochs//3)*.02)
        elif mode == "magnitude_sweep":
            levels = (0.1, .25, .5, 1., 2.)
            magnitude = levels[min(len(levels)-1,
                max(0, index-epochs//3)*len(levels)//max(1, 2*epochs//3))] if active else 0.0
        else: magnitude = 0.0
        if scenario == "delayed_stale_uwb" and index == epochs//2:
            stamp -= 3*period_ns  # explicitly older than already published epochs
        output.append(Epoch(index, stamp, publish_uwb, anchor_count, 1, mode,
                            active, outage, magnitude))
    return output


def position(index: int) -> tuple[float, float, float]:
    t = index*.05
    return (0.2*t, .3*math.sin(.2*t), 1.2)


def ros_time_from_ns(rospy, timestamp_ns: int):
    return rospy.Time(timestamp_ns // 1_000_000_000,
                      timestamp_ns % 1_000_000_000)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=[
        "delayed_stale_uwb", "uwb_drop_resume", "imu_gap",
        "anchor_set_8_7_8", "fault_step", "fault_ramp",
        "fault_magnitude_sweep", "fault_outage"])
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--rate-scale", type=float, default=10.0,
                        help="wall-clock acceleration; timestamps remain 20/200 Hz")
    parser.add_argument("--inventory", type=pathlib.Path)
    # roslaunch appends private remapping arguments understood by rospy but not
    # by argparse.
    args = parser.parse_args([value for value in sys.argv[1:]
                              if not value.startswith("__")])
    import rospy
    from sensor_msgs.msg import Imu
    from uwb_imu_pl.msg import FaultTruth, LinktrackNode2, LinktrackNodeframe3

    rospy.init_node("week4_topic_publisher", anonymous=True)
    imu_pub = rospy.Publisher("/sim/imu", Imu, queue_size=1000)
    uwb_pub = rospy.Publisher("/nlink_linktrack_nodeframe3",
                              LinktrackNodeframe3, queue_size=100)
    truth_pub = rospy.Publisher("/uwb_sim/fault_truth", FaultTruth, queue_size=100)
    rospy.sleep(.5)
    start_ns = rospy.Time.now().to_nsec() + 100_000_000
    plan = schedule(args.scenario, args.epochs, start_ns)
    sequence = 1
    wall_rate = rospy.Rate(20*args.rate_scale)
    for epoch in plan:
        if rospy.is_shutdown(): break
        # Ten 200 Hz samples for every UWB epoch. imu_gap deliberately omits a
        # span longer than configured max_gap_s.
        for substep in range(10):
            if args.scenario == "imu_gap" and epoch.index == args.epochs//2 and \
                    2 <= substep <= 7:
                continue
            imu = Imu()
            imu.header.seq = sequence
            imu.header.stamp = ros_time_from_ns(
                rospy, start_ns + epoch.index*50_000_000 + substep*5_000_000)
            imu.header.frame_id = "base_link"
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.80665
            imu.orientation_covariance[0] = -1.0
            imu_pub.publish(imu)
            sequence += 1
        x, y, z = position(epoch.index)
        truth = FaultTruth()
        truth.header.seq = sequence
        truth.header.stamp = ros_time_from_ns(rospy, epoch.timestamp_ns)
        truth.sequence = sequence
        truth.anchor_id = epoch.fault_anchor_id
        truth.fault_mode = epoch.fault_mode
        truth.active = epoch.fault_active
        truth.outage = epoch.outage
        truth.injected_bias_m = epoch.injected_bias_m
        ax, ay, az = ANCHORS[epoch.fault_anchor_id-1]
        truth.true_range_m = math.sqrt((x-ax)**2+(y-ay)**2+(z-az)**2)
        truth_pub.publish(truth)
        sequence += 1
        if epoch.publish_uwb:
            frame = LinktrackNodeframe3()
            frame.header.seq = sequence
            frame.header.stamp = ros_time_from_ns(rospy, epoch.timestamp_ns)
            frame.header.frame_id = "world"
            frame.local_time = epoch.timestamp_ns
            for anchor_id, (ax, ay, az) in enumerate(ANCHORS[:epoch.anchor_count], 1):
                if epoch.outage and anchor_id == epoch.fault_anchor_id: continue
                node = LinktrackNode2()
                node.id = anchor_id
                node.dis = math.sqrt((x-ax)**2+(y-ay)**2+(z-az)**2)
                if anchor_id == epoch.fault_anchor_id:
                    node.dis += epoch.injected_bias_m
                node.fp_rssi = -60.0
                node.rx_rssi = -65.0
                frame.nodes.append(node)
            uwb_pub.publish(frame)
            sequence += 1
        wall_rate.sleep()
    if args.inventory:
        args.inventory.parent.mkdir(parents=True, exist_ok=True)
        args.inventory.write_text(json.dumps({
            "scenario": args.scenario, "epochs": [asdict(item) for item in plan],
            "completed": not rospy.is_shutdown()}, indent=2, sort_keys=True)+"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
