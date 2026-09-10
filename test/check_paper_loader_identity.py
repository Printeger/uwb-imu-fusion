#!/usr/bin/env python3
"""Audit paper ledger identities against a real rosbag and a nested crop."""

import argparse
import csv
import math
import sys
from pathlib import Path

import rosbag


MIN_RAW_TIME_ABS_TOL_SECONDS = 1e-9


def raw_time_abs_tolerance(lhs, rhs):
    """Allow only double-representation roundoff for epoch timestamps."""
    return max(
        MIN_RAW_TIME_ABS_TOL_SECONDS,
        2.0 * sys.float_info.epsilon * abs(lhs),
        2.0 * sys.float_info.epsilon * abs(rhs),
    )


def raw_times_match(lhs, rhs):
    return math.isclose(
        lhs,
        rhs,
        rel_tol=0.0,
        abs_tol=raw_time_abs_tolerance(lhs, rhs),
    )


def read_ledger(path):
    rows = []
    with Path(path).open(newline="") as stream:
        for row in csv.DictReader(stream):
            row["obs_id"] = int(row["obs_id"])
            row["source_message"] = int(row["source_message"])
            row["source_range"] = int(row["source_range"])
            row["raw_time"] = float(row["raw_time"])
            row["raw_tag_id"] = int(row["raw_tag_id"])
            row["anchor_id"] = int(row["anchor_id"])
            rows.append(row)
    if not rows:
        raise AssertionError(f"empty ledger: {path}")
    return rows


def read_bag_ranges(path, topic, interface):
    expected = {}
    message_index = 0
    with rosbag.Bag(path, "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for _, message, record_time in bag.read_messages(topics=[topic]):
            ranges = message.nodes if interface == "original" else message.ranges
            tag_id = int(message.id)
            raw_time = message.header.stamp.to_sec()
            for range_index, item in enumerate(ranges):
                anchor_id = int(item.id)
                expected[(message_index, range_index)] = (
                    raw_time,
                    tag_id,
                    anchor_id,
                    record_time.to_sec(),
                )
            message_index += 1
    if not expected:
        raise AssertionError(f"no ranges found on {topic} in {path}")
    return expected, bag_start, bag_end


def validate_rows(rows, expected, label):
    obs_ids = set()
    source_keys = set()
    mapping = {}
    for row in rows:
        key = (row["source_message"], row["source_range"])
        if key in source_keys:
            raise AssertionError(f"{label}: reused source identity {key}")
        source_keys.add(key)
        if row["obs_id"] in obs_ids:
            raise AssertionError(f"{label}: reused obs_id {row['obs_id']}")
        obs_ids.add(row["obs_id"])
        if key not in expected:
            raise AssertionError(f"{label}: source identity absent from bag: {key}")
        raw_time, tag_id, anchor_id, _ = expected[key]
        if not raw_times_match(row["raw_time"], raw_time):
            raise AssertionError(
                f"{label}: raw time mismatch for {key}: "
                f"{row['raw_time']} != {raw_time}"
            )
        if row["raw_tag_id"] != tag_id or row["anchor_id"] != anchor_id:
            raise AssertionError(
                f"{label}: tag/anchor mismatch for {key}: "
                f"{row['raw_tag_id']}:{row['anchor_id']} != "
                f"{tag_id}:{anchor_id}"
            )
        mapping[key] = row["obs_id"]
    return mapping


def expected_window_keys(expected, bag_start, bag_end, start, duration):
    begin = bag_start + start
    end = bag_end if duration < 0.0 else begin + duration
    return {
        key
        for key, (_, _, _, record_time) in expected.items()
        if begin <= record_time <= end
    }


def validate_trajectory(path, rows, label):
    times = []
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                times.append(float(line.split()[0]))
    if not times:
        raise AssertionError(f"{label}: empty trajectory")
    planned_times = [row["raw_time"] for row in rows if row["planned"] == "1"]
    if min(times) < min(planned_times) - 1e-9:
        raise AssertionError(f"{label}: trajectory begins before cropped UWB plan")
    if max(times) > max(planned_times) + 1e-9:
        raise AssertionError(f"{label}: trajectory ends after cropped UWB plan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", choices=("original", "sfuise"), required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--reference-csv", required=True)
    parser.add_argument("--reference-start", type=float, required=True)
    parser.add_argument("--reference-duration", type=float, required=True)
    parser.add_argument("--crop-csv", required=True)
    parser.add_argument("--crop-start", type=float, required=True)
    parser.add_argument("--crop-duration", type=float, required=True)
    parser.add_argument("--crop-trajectory", required=True)
    args = parser.parse_args()

    expected, bag_start, bag_end = read_bag_ranges(
        args.bag, args.topic, args.interface
    )
    reference_rows = read_ledger(args.reference_csv)
    crop_rows = read_ledger(args.crop_csv)
    reference = validate_rows(reference_rows, expected, "reference")
    crop = validate_rows(crop_rows, expected, "crop")

    expected_reference = expected_window_keys(
        expected,
        bag_start,
        bag_end,
        args.reference_start,
        args.reference_duration,
    )
    expected_crop = expected_window_keys(
        expected, bag_start, bag_end, args.crop_start, args.crop_duration
    )
    if set(reference) != expected_reference:
        raise AssertionError("reference ledger does not exactly match its bag window")
    if set(crop) != expected_crop:
        raise AssertionError("crop ledger does not exactly match its bag window")

    crop_keys = set(crop)
    if not crop_keys < set(reference):
        raise AssertionError("crop source identities must be a strict subset of reference")
    for key in crop_keys:
        if crop[key] != reference[key]:
            raise AssertionError(
                f"stable obs_id mismatch for {key}: {crop[key]} != {reference[key]}"
            )
    validate_trajectory(args.crop_trajectory, crop_rows, "crop")
    print(
        "PASS interface={} bag_ranges={} reference_rows={} crop_rows={} "
        "shared_stable_ids={}".format(
            args.interface,
            len(expected),
            len(reference_rows),
            len(crop_rows),
            len(crop_keys),
        )
    )


if __name__ == "__main__":
    main()
