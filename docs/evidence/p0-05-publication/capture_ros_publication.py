#!/usr/bin/env python3
"""Capture one identity-consistent P0-05 compound/legacy publication.

The process exits only after the compound message, legacy IntegrityStatus and
legacy Odometry agree for one attempt.  With --require-committed it rejects
diagnostic-only attempts and records a real committed (currently formally
unprotected) output.  JSON is canonical so its SHA-256 is reproducible.
"""

import argparse
import csv
import hashlib
import json
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from uwb_imu_pl.msg import IntegrityStatus, NavigationIntegrity


def stamp(value):
    return [int(value.secs), int(value.nsecs)]


def vector3(value):
    return [value.x, value.y, value.z]


def quaternion(value):
    return [value.x, value.y, value.z, value.w]


def status_record(value):
    return {
        "attempt_id": int(value.attempt_id),
        "transaction_id": int(value.transaction_id),
        "window_id": int(value.window_id),
        "solution_id": int(value.solution_id),
        "attempted_timestamp": stamp(value.attempted_timestamp),
        "state_timestamp": stamp(value.state_timestamp),
        "publish_timestamp": stamp(value.publish_timestamp),
        "batch_committed": bool(value.batch_committed),
        "deadline_missed": bool(value.deadline_missed),
        "publication_protected": bool(value.publication_protected),
        "publication_certificate_id": int(value.publication_certificate_id),
        "final_packet_protocol_version": int(value.final_packet_protocol_version),
        "final_packet_digest": int(value.final_packet_digest),
        "final_packet_authoritative": bool(value.final_packet_authoritative),
        "protected_frame_id": value.protected_frame_id,
        "protected_position_reference": value.protected_position_reference,
        "protection_packet_id": value.protection_packet_id,
        "state_valid": bool(value.state_valid),
        "fresh": bool(value.fresh),
        "pl_status": value.pl_status,
        "hpl_m": value.hpl_m,
        "vpl_m": value.vpl_m,
        "pl_xyz_m": list(value.pl_xyz_m),
        "formal_eligible": bool(value.formal_eligible),
        "risk_budget_valid": bool(value.risk_budget_valid),
        "allocated_hmi_risk": value.allocated_hmi_risk,
        "hmi_risk_requirement": value.hmi_risk_requirement,
        "reason_codes": list(value.reason_codes),
        "selected_action_id": int(value.selected_action_id),
        "selected_action_type": value.selected_action_type,
        "fde_status": value.fde_status,
        "reason": value.reason,
    }


def status_semantics(value):
    """Fields shared by an authoritative receipt and its legacy mirror."""
    record = status_record(value)
    del record["final_packet_authoritative"]
    return record


def odometry_record(value):
    return {
        "state_stamp": stamp(value.header.stamp),
        "frame_id": value.header.frame_id,
        "child_frame_id": value.child_frame_id,
        "position": vector3(value.pose.pose.position),
        "orientation": quaternion(value.pose.pose.orientation),
        "linear_velocity": vector3(value.twist.twist.linear),
        "pose_covariance": list(value.pose.covariance),
        "twist_covariance": list(value.twist.covariance),
    }


def odometry_equal(left, right):
    return odometry_record(left) == odometry_record(right)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--require-committed", action="store_true")
    parser.add_argument("--csv", help="integrity.csv produced from the same final packet")
    args = parser.parse_args(rospy.myargv()[1:])

    condition = threading.Condition()
    compound = {}
    legacy_status = {}
    legacy_odometry = []
    result = []

    def try_match():
        if result:
            return
        for attempt_id, message in sorted(compound.items()):
            if args.require_committed and not message.integrity.batch_committed:
                continue
            status = legacy_status.get(attempt_id)
            if status is None or status_semantics(status) != status_semantics(message.integrity):
                continue
            if (message.final_packet_authoritative or
                    message.integrity.final_packet_authoritative or
                    status.final_packet_authoritative or
                    message.integrity.publication_protected or
                    status.publication_protected):
                continue
            odometry = next(
                (item for item in legacy_odometry
                 if odometry_equal(item, message.odometry)), None)
            if odometry is None:
                continue
            record = {
                "compound_envelope": {
                    "run_id": message.run_id,
                    "attempt_id": int(message.attempt_id),
                    "transaction_id": int(message.transaction_id),
                    "solution_id": int(message.solution_id),
                    "scope_digest": message.scope_digest,
                    "manifest_digest": message.manifest_digest,
                    "attempted_timestamp": stamp(message.attempted_timestamp),
                    "state_timestamp": stamp(message.state_timestamp),
                    "arrival_steady_ns": int(message.arrival_steady_ns),
                    "finish_steady_ns": int(message.finish_steady_ns),
                    "frame_id": message.header.frame_id,
                    "final_packet_protocol_version":
                        int(message.final_packet_protocol_version),
                    "final_packet_digest": int(message.final_packet_digest),
                    "final_packet_authoritative":
                        bool(message.final_packet_authoritative),
                    "protected_frame_id": message.protected_frame_id,
                    "protected_position_reference":
                        message.protected_position_reference,
                    "protection_packet_id": message.protection_packet_id,
                    "pl_xyz_m": list(message.pl_xyz_m),
                    "hpl_m": message.hpl_m,
                    "vpl_m": message.vpl_m,
                    "formal_eligible": bool(message.formal_eligible),
                    "risk_budget_valid": bool(message.risk_budget_valid),
                    "reason_codes": list(message.reason_codes),
                },
                "compound_integrity": status_record(message.integrity),
                "compound_odometry": odometry_record(message.odometry),
                "legacy_integrity": status_record(status),
                "legacy_odometry": odometry_record(odometry),
            }
            result.append(record)
            condition.notify_all()
            return

    def compound_callback(message):
        with condition:
            compound[int(message.attempt_id)] = message
            try_match()

    def status_callback(message):
        with condition:
            legacy_status[int(message.attempt_id)] = message
            try_match()

    def odometry_callback(message):
        with condition:
            legacy_odometry.append(message)
            del legacy_odometry[:-256]
            try_match()

    rospy.init_node("p005_publication_capture", anonymous=True)
    rospy.Subscriber("/uwb_imu_pl/solution", NavigationIntegrity,
                     compound_callback, queue_size=256)
    rospy.Subscriber("/uwb_imu_pl/integrity", IntegrityStatus,
                     status_callback, queue_size=256)
    rospy.Subscriber("/uwb_imu_pl/odometry", Odometry,
                     odometry_callback, queue_size=256)

    deadline = time.monotonic() + args.timeout
    with condition:
        while not result and not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("timed out without a matching publication")
            condition.wait(min(remaining, 1.0))

    encoded = json.dumps(result[0], allow_nan=True, sort_keys=True,
                         separators=(",", ":")) + "\n"
    if result[0]["compound_envelope"]["final_packet_authoritative"]:
        raise RuntimeError("non-atomic ROS compound incorrectly claims authority")
    digest = result[0]["compound_envelope"]["final_packet_digest"]
    if digest == 0 or digest != result[0]["compound_integrity"]["final_packet_digest"]:
        raise RuntimeError("compound/embedded status final digest mismatch")
    if digest != result[0]["legacy_integrity"]["final_packet_digest"]:
        raise RuntimeError("compound/legacy status final digest mismatch")
    if result[0]["legacy_integrity"]["final_packet_authoritative"]:
        raise RuntimeError("legacy status incorrectly claims receipt authority")
    if args.csv:
        csv_deadline = time.monotonic() + min(args.timeout, 10.0)
        rows = []
        while not rows and time.monotonic() < csv_deadline:
            try:
                with open(args.csv, newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))
            except FileNotFoundError:
                pass
            if not rows:
                time.sleep(0.05)
        if not rows:
            raise RuntimeError("integrity CSV is empty")
        csv_row = rows[-1]
        if int(csv_row["final_packet_digest"]) != digest:
            raise RuntimeError("ROS/CSV final digest mismatch")
        status = result[0]["compound_integrity"]
        csv_checks = {
            "attempted_timestamp_ns": status["attempted_timestamp"][0] * 1000000000
                + status["attempted_timestamp"][1],
            "state_timestamp_ns": status["state_timestamp"][0] * 1000000000
                + status["state_timestamp"][1],
            "final_packet_protocol_version": status["final_packet_protocol_version"],
            "final_packet_authoritative": int(status["final_packet_authoritative"]),
            "formal_eligible": int(status["formal_eligible"]),
            "risk_budget_valid": int(status["risk_budget_valid"]),
        }
        for key, expected in csv_checks.items():
            if int(csv_row[key]) != expected:
                raise RuntimeError("ROS/CSV mismatch for " + key)
        for key, expected in zip(("pl_x", "pl_y", "pl_z"),
                                 status["pl_xyz_m"]):
            if float(csv_row[key]) != expected:
                raise RuntimeError("ROS/CSV mismatch for " + key)
        for key in ("hpl_m", "vpl_m"):
            if float(csv_row[key]) != status[key]:
                raise RuntimeError("ROS/CSV mismatch for " + key)
        if csv_row["reason_codes"].split(";") != status["reason_codes"]:
            raise RuntimeError("ROS/CSV mismatch for reason_codes")
        for key in ("protected_frame_id", "protected_position_reference",
                    "protection_packet_id"):
            if csv_row[key] != status[key]:
                raise RuntimeError("ROS/CSV mismatch for " + key)
        result[0]["csv_receipt"] = {
            key: csv_row[key] for key in (
                "attempted_timestamp_ns", "state_timestamp_ns", "pl_x", "pl_y",
                "pl_z", "hpl_m", "vpl_m", "formal_eligible",
                "risk_budget_valid", "reason_codes",
                "final_packet_protocol_version", "final_packet_digest",
                "final_packet_authoritative", "protected_frame_id",
                "protected_position_reference", "protection_packet_id")
        }
        encoded = json.dumps(result[0], allow_nan=True, sort_keys=True,
                             separators=(",", ":")) + "\n"
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write(encoded)
    print("canonical_sha256=" + hashlib.sha256(encoded.encode()).hexdigest())
    print("attempt_id=" + str(result[0]["compound_envelope"]["attempt_id"]))
    print("batch_committed=" +
          str(result[0]["compound_integrity"]["batch_committed"]).lower())
    print("publication_protected=" +
          str(result[0]["compound_integrity"]["publication_protected"]).lower())


if __name__ == "__main__":
    main()
