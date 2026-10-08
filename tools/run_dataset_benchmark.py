#!/usr/bin/env python3
"""Prepare, run, evaluate, and report the UWB/IMU dataset benchmark."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from rosbags.highlevel import AnyReader
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

import benchmark_common as common


ROOT = common.ROOT
DEFAULT_OUTPUT = ROOT / "results/benchmark"
CONFIG_DIR = ROOT / "config/benchmark"
SFUISE_ROOT = ROOT.parent / "SFUISE"
SFUISE_PIN = "75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d"


def default_nominal_binary() -> Path:
    devel = ROOT.parent.parent / "devel"
    candidates = [devel/"lib/uwb_imu_pl/nominal_dataset_runner",
                  devel/".private/uwb_imu_pl/lib/uwb_imu_pl/nominal_dataset_runner"]
    return next((path for path in candidates if path.is_file()), candidates[0])


def load_config(path: Path) -> dict:
    config = yaml.safe_load(path.read_text())
    if config.get("schema") != "uwb-imu-benchmark/v1":
        raise ValueError(f"unsupported benchmark config schema: {path}")
    config["_path"] = str(path.resolve())
    config["_data_root"] = str((ROOT / config["data_root"]).resolve())
    return config


def configs(arguments) -> list[dict]:
    if arguments.config:
        return [load_config(Path(arguments.config))]
    return [load_config(path) for path in sorted(CONFIG_DIR.glob("*.yaml"))]


def sequences(config: dict) -> list[dict]:
    if config["sequences"] != "all":
        return list(config["sequences"])
    params = json.loads((Path(config["_data_root"]) /
                         config["sequence_manifest"]).read_text())
    result = []
    for item in params:
        result.append({"id": item["name"], "path": "data/" + item["name"],
                       "setup": item["setup"], "landmarks": item["landmarks"],
                       "timing_representative": item["name"] == "loop-3d_s3"})
    return result


def selected(config: dict, sequence: dict, wanted: str | None) -> bool:
    return wanted is None or wanted == sequence["id"] or wanted == config["dataset"]


def rows_file_metadata(cache: Path) -> dict:
    result = {}
    for name in ("imu.csv", "uwb.csv", "anchors.csv", "gt.csv"):
        path = cache / name
        if path.exists():
            with path.open() as stream:
                count = max(0, sum(1 for _ in stream) - 1)
            result[name] = {"rows": count, "sha256": common.sha256(path)}
    return result


def source_file_hashes(paths) -> dict:
    """Hash only files that actually feed an adapter, never bulky side data."""
    return {str(Path(path).resolve()): common.sha256(Path(path))
            for path in paths if Path(path).is_file()}


def publish_cache(config: dict, sequence: dict, cache: Path, source: Path,
                  imu: list[dict], uwb: list[dict], anchors: dict[int, np.ndarray],
                  gt: list[dict], provenance: dict):
    if cache.exists():
        shutil.rmtree(cache)
    cache.mkdir(parents=True)
    imu.sort(key=lambda row: float(row["t"]))
    uwb.sort(key=lambda row: (float(row["t"]), int(row["source_message"]),
                              int(row["source_range"])))
    gt.sort(key=lambda row: (float(row["t"]), int(row["tag_id"])))
    common.write_csv(cache / "imu.csv", common.IMU_FIELDS, imu)
    common.write_csv(cache / "uwb.csv", common.UWB_FIELDS, uwb)
    common.write_csv(cache / "anchors.csv", common.ANCHOR_FIELDS,
                     ({"anchor_id": key, "x": value[0], "y": value[1], "z": value[2]}
                      for key, value in sorted(anchors.items())))
    common.write_csv(cache / "gt.csv", common.GT_FIELDS, gt)
    manifest = {
        "schema": common.CACHE_SCHEMA, "dataset": config["dataset"],
        "sequence": sequence["id"], "source": str(source),
        "source_sha256": common.sha256(source) if source.is_file() else None,
        "benchmark_config": config["_path"],
        "benchmark_config_sha256": common.sha256(Path(config["_path"])),
        "primary_tag": int(config["primary_tag"]),
        "bootstrap_prefer_below_anchors": bool(
            config.get("bootstrap", {}).get("prefer_below_anchor_median", False)),
        "epoch_batch_window_s": float(config["uwb"].get("epoch_batch_window_s", 0.0)),
        "timing_representative": bool(sequence.get("timing_representative", False)),
        "calibration_label": config["calibration"]["label"],
        "estimator_inputs": ["imu.csv", "uwb.csv", "anchors.csv"],
        "evaluator_only": ["gt.csv"], "provenance": provenance,
        "repository": common.git_identity(ROOT), "system": common.system_identity(),
    }
    manifest["files"] = rows_file_metadata(cache)
    (cache / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def v3(obj) -> np.ndarray:
    return np.array([obj.x, obj.y, obj.z], dtype=float)


def q4(obj) -> np.ndarray:
    return np.array([obj.x, obj.y, obj.z, obj.w], dtype=float)


def bag_messages(path: Path, topics: set[str]):
    with AnyReader([path]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        for connection, timestamp, raw in reader.messages(connections=connections):
            yield connection.topic, timestamp, reader.deserialize(raw, connection.msgtype)


def imu_row(t, message, accel_scale=1.0, gyro_scale=1.0,
            accel_rotation=np.eye(3), gyro_rotation=np.eye(3)) -> dict:
    accel = accel_rotation @ v3(message.linear_acceleration) * accel_scale
    gyro = gyro_rotation @ v3(message.angular_velocity) * gyro_scale
    return dict(zip(common.IMU_FIELDS, [t, *accel, *gyro]))


def uwb_row(t, source_message, source_range, tag_id, anchor_id, observed,
            sigma, lever, anchor, valid=True, raw=None) -> dict:
    return dict(zip(common.UWB_FIELDS,
                    [t, source_message, source_range, tag_id, anchor_id,
                     observed, observed if raw is None else raw, sigma, *lever,
                     *anchor, int(valid)]))


def prepare_huec(config, sequence, cache):
    source = Path(config["_data_root"]) / sequence["path"]
    topics = {config["imu"]["topic"], config["ground_truth"]["topic"],
              *config["uwb"]["topics"]}
    raw_imu, raw_uwb, raw_gt, anchors = [], [], [], {}
    index = 0
    lever = config["uwb"]["lever_arm_body_m"]
    for topic, bag_ns, message in bag_messages(source, topics):
        t = common.header_time(message, bag_ns)
        if topic == config["imu"]["topic"]:
            raw_imu.append((t, message))
        elif topic == config["ground_truth"]["topic"]:
            raw_gt.append((t, message.pose.pose))
        else:
            anchor_id = int(message.id)
            anchor = np.array([message.x, message.y, message.z], dtype=float)
            anchors[anchor_id] = anchor
            raw_uwb.append((t, index, 0, 0, anchor_id,
                            float(message.distanceFromTag), anchor))
            index += 1
    origin = min(raw_imu[0][0], raw_uwb[0][0], raw_gt[0][0])
    imu = [imu_row(t-origin, msg) for t, msg in raw_imu]
    uwb = [uwb_row(t-origin, i, j, tag, aid, value,
                   config["uwb"]["sigma_m"], lever, anchor)
           for t, i, j, tag, aid, value, anchor in raw_uwb]
    gt = [common.pose_row(t-origin, pose.position, pose.orientation, 0)
          for t, pose in raw_gt]
    publish_cache(config, sequence, cache, source, imu, uwb, anchors, gt,
                  {"acceleration": "source_mps2", "gyro": "source_radps_no_deg_conversion",
                   "lever_arm_body_m": lever, "gt_reference": "uwb_tag",
                   "source_file_sha256": source_file_hashes([source])})


def prepare_miluv(config, sequence, cache):
    source = Path(config["_data_root"]) / sequence["path"]
    imu_df = pd.read_csv(source / config["imu"]["file"])
    uwb_df = pd.read_csv(source / config["uwb"]["file"])
    gt_df = pd.read_csv(source / config["ground_truth"]["file"])
    anchors = {int(k): np.asarray(v, float) for k, v in config["anchors"].items()}
    levers = {int(k): np.asarray(v, float)
              for k, v in config["uwb"]["tag_levers_body_m"].items()}
    origin = min(imu_df.timestamp.iloc[0], uwb_df.timestamp.iloc[0],
                 gt_df.timestamp.iloc[0])
    imu = [dict(zip(common.IMU_FIELDS, [row["timestamp"]-origin,
             row["linear_acceleration.x"], row["linear_acceleration.y"],
             row["linear_acceleration.z"], row["angular_velocity.x"],
             row["angular_velocity.y"], row["angular_velocity.z"]]))
           for _, row in imu_df.iterrows()]
    uwb = []
    for index, row in uwb_df.iterrows():
        tag, anchor_id = int(row.from_id), int(row.to_id)
        if tag not in levers or anchor_id not in anchors:
            continue
        sigma = float(row.get(config["uwb"]["sigma_field"],
                              config["uwb"]["default_sigma_m"]))
        if not math.isfinite(sigma) or sigma <= 0:
            sigma = config["uwb"]["default_sigma_m"]
        uwb.append(uwb_row(float(row.timestamp-origin), index, 0, tag, anchor_id,
                            float(row[config["uwb"]["range_field"]]), sigma,
                            levers[tag], anchors[anchor_id], raw=float(row.range_raw)))
    gt = []
    for _, values in gt_df.iterrows():
        p = np.array([values["pose.position.x"], values["pose.position.y"],
                      values["pose.position.z"]])
        q = np.array([values["pose.orientation.x"], values["pose.orientation.y"],
                      values["pose.orientation.z"], values["pose.orientation.w"]])
        for tag, lever in levers.items():
            pt = common.apply_tag_lever(p, q, lever)
            gt.append(dict(zip(common.GT_FIELDS,
                               [values["timestamp"]-origin, *pt, *q, tag])))
    publish_cache(config, sequence, cache, source, imu, uwb, anchors, gt,
                  {"range": "MILUV_processed_range", "raw_range_retained": True,
                   "anchors": "official_constellation_0", "gt_reference": "tag_from_mocap_body",
                   "source_file_sha256": source_file_hashes([
                       source/config["imu"]["file"], source/config["uwb"]["file"],
                       source/config["ground_truth"]["file"]])})


def prepare_starloc(config, sequence, cache):
    source = Path(config["_data_root"]) / sequence["path"]
    imu_df = pd.read_csv(source / config["imu"]["file"])
    uwb_df = pd.read_csv(source / config["uwb"]["file"])
    calib = json.loads((source / "calib.json").read_text())
    def rot(item):
        return Rotation.from_quat([item["rot_x"], item["rot_y"],
                                   item["rot_z"], item["w"]])
    c_r_i = rot(calib["tf_cam_rig"]).inv() * rot(calib["tf_cam_imu"])
    imu = []
    for row in imu_df.itertuples(index=False):
        value = row._asdict()
        gyro = c_r_i.apply([value["angular_velocity_x"], value["angular_velocity_y"],
                            value["angular_velocity_z"]])
        imu.append(dict(zip(common.IMU_FIELDS,
                            [value["time_s"], value["linear_acceleration_x"],
                             value["linear_acceleration_y"], value["linear_acceleration_z"],
                             *gyro])))
    anchor_file = Path(config["_data_root"]) / config["anchors"]["files"][sequence["landmarks"]]
    anchors_df = pd.read_csv(anchor_file, index_col=0)
    marker_to_radio = {
        int(marker): int(radio) for marker, radio in
        config["anchors"]["radio_id_from_marker_id"][sequence["landmarks"]].items()
    }
    anchors = {marker_to_radio[int(index)]: row.to_numpy(float)
               for index, row in anchors_df.iterrows()}
    setup_levers = {int(k): np.asarray(v, float) for k, v in
                    config["uwb"]["tag_levers_by_setup"][sequence["setup"]].items()}
    range_field = (config["uwb"]["range_field_s5"] if sequence["setup"] == "s5"
                   else config["uwb"]["range_field_s1_s4"])
    uwb = []
    for index, row in uwb_df.iterrows():
        tag, aid = int(row.from_id), int(row.to_id)
        if tag not in setup_levers or aid not in anchors:
            continue
        observed = float(row[range_field])
        if not math.isfinite(observed):
            observed = float(row["range"])
        sigma = float(row.get("std", config["uwb"]["default_sigma_m"]))
        if not math.isfinite(sigma) or sigma <= 0:
            sigma = config["uwb"]["default_sigma_m"]
        uwb.append(uwb_row(float(row.time_s), index, 0, tag, aid, observed,
                            sigma, setup_levers[tag], anchors[aid], raw=float(row["range"])))
    gt = []
    primary = int(config["primary_tag"])
    for row in uwb_df[uwb_df["from_id"] == primary].itertuples(index=False):
        value = row._asdict()
        q = np.array([value["rot_x"], value["rot_y"], value["rot_z"], value["w"]])
        tag_p = np.array([value["tag_pos_x"], value["tag_pos_y"],
                          value["tag_pos_z"]])
        gt.append(dict(zip(common.GT_FIELDS, [value["time_s"], *tag_p, *q, primary])))
    publish_cache(config, sequence, cache, source, imu, uwb, anchors, gt,
                  {"gyro_rotation_C_r_i_xyzw": c_r_i.as_quat().tolist(),
                   "acceleration_frame": "rig_as_published", "range_field": range_field,
                   "setup": sequence["setup"], "landmarks": sequence["landmarks"],
                   "anchor_marker_id_to_radio_id": marker_to_radio,
                   "anchor_id_mapping_provenance":
                       "published_v1_room_geometry_correspondence_to_v2_radio_ids",
                   "gt_reference": "uwb_csv_official_tag_position_and_rig_orientation",
                   "source_file_sha256": source_file_hashes([
                       source/config["imu"]["file"], source/config["uwb"]["file"],
                       source/"calib.json", anchor_file])})


def prepare_sfuise(config, sequence, cache):
    source = Path(config["_data_root"]) / sequence["path"]
    topics = {config["imu"]["topic"], config["uwb"]["topic"],
              config["uwb"]["anchor_topic"], config["ground_truth"]["topic"]}
    raw_imu, raw_uwb, raw_gt, anchor_samples = [], [], [], defaultdict(list)
    source_message = 0
    for topic, bag_ns, message in bag_messages(source, topics):
        t = common.header_time(message, bag_ns)
        if topic == config["imu"]["topic"]:
            raw_imu.append((t, message))
        elif topic == config["uwb"]["anchor_topic"]:
            for item in message.anchor:
                anchor_samples[int(item.id)].append(v3(item.position))
        elif topic == config["ground_truth"]["topic"]:
            raw_gt.append((t, message.transform))
        else:
            raw_uwb.append((t, source_message, int(message.id), message.ranges))
            source_message += 1
    anchors = {key: np.mean(values, axis=0) for key, values in anchor_samples.items()}
    ordered_ids = sorted(anchors)
    offsets = config["uwb"]["toa_offsets_by_sequence"][sequence["id"]]
    offset_by_id = dict(zip(ordered_ids, offsets))
    origin = min(raw_imu[0][0], raw_uwb[0][0], raw_gt[0][0])
    imu = [imu_row(t-origin, msg) for t, msg in raw_imu]
    lever = config["uwb"]["lever_arm_body_m"]
    uwb = []
    for t, message_index, tag, ranges in raw_uwb:
        for range_index, item in enumerate(ranges):
            aid = int(item.id)
            if aid not in anchors:
                continue
            raw = float(item.range)
            corrected = raw + float(offset_by_id.get(aid, 0.0))
            uwb.append(uwb_row(t-origin, message_index, range_index, tag, aid,
                                corrected, config["uwb"]["sigma_m"], lever,
                                anchors[aid], valid=int(item.ra) != 0, raw=raw))
    gt = []
    for t, tf in raw_gt:
        quaternion = q4(tf.rotation)
        tag_position = v3(tf.translation) + Rotation.from_quat(quaternion).apply(lever)
        gt.append(dict(zip(common.GT_FIELDS,
                           [t-origin, *tag_position, *quaternion,
                            int(config["primary_tag"])])))
    publish_cache(config, sequence, cache, source, imu, uwb, anchors, gt,
                  {"range": "raw_plus_official_toa_offset", "toa_offset_by_anchor": offset_by_id,
                   "sfuise_rejection": "official_enabled",
                   "gt_reference": "vive_body_projected_to_physical_uwb_tag",
                   "source_file_sha256": source_file_hashes([source])})


def readable_bag(source: Path, recover: bool):
    if not recover:
        return source, None
    temp = Path(tempfile.mkdtemp(prefix="uwb_benchmark_reindex_"))
    copy = temp / source.name
    shutil.copy2(source, copy)
    process = subprocess.run(["rosbag", "reindex", str(copy)], text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if process.returncode:
        shutil.rmtree(temp)
        raise RuntimeError("temporary rosbag reindex failed: " + process.stdout[-1000:])
    return copy, temp


def prepare_own(config, sequence, cache):
    original = Path(config["_data_root"]) / sequence["path"]
    source, temporary = readable_bag(original, sequence.get("recover_unindexed_copy", False))
    topics = {config["imu"]["topic"], config["uwb"]["topic"],
              config["ground_truth"]["topic"]}
    anchor_topics = {config["anchors"]["source_topics"].format(id=value): value
                     for value in config["anchors"]["ids"]}
    topics.update(anchor_topics)
    raw_imu, raw_uwb, raw_gt, anchor_samples = [], [], [], defaultdict(list)
    source_message = 0
    try:
        for topic, bag_ns, message in bag_messages(source, topics):
            t = common.header_time(message, bag_ns)
            if topic == config["imu"]["topic"]:
                raw_imu.append((t, message))
            elif topic == config["uwb"]["topic"]:
                raw_uwb.append((t, source_message, int(message.id), message.nodes))
                source_message += 1
            elif topic == config["ground_truth"]["topic"]:
                raw_gt.append((t, message.pose))
            elif topic in anchor_topics:
                anchor_samples[anchor_topics[topic]].append(v3(message.pose.position))
    finally:
        if temporary:
            shutil.rmtree(temporary)
    anchors = {key: np.median(values, axis=0) for key, values in anchor_samples.items()}
    origin = min(raw_imu[0][0], raw_uwb[0][0], raw_gt[0][0])
    imu = [imu_row(t-origin, msg, accel_scale=config["imu"]["acceleration_scale"])
           for t, msg in raw_imu]
    lever = config["uwb"]["lever_arm_body_m"]
    uwb = []
    for t, message_index, tag, nodes in raw_uwb:
        for range_index, item in enumerate(nodes):
            aid = int(item.id)
            if aid in anchors:
                uwb.append(uwb_row(t-origin, message_index, range_index, tag, aid,
                                    float(item.dis), config["uwb"]["sigma_m"], lever,
                                    anchors[aid], valid=float(item.dis) > 0))
    gt = [common.pose_row(t-origin, pose.position, pose.orientation, 0)
          for t, pose in raw_gt]
    publish_cache(config, sequence, cache, original, imu, uwb, anchors, gt,
                  {"acceleration": "livox_g_times_9.80665", "gyro": "source_radps",
                   "anchor_aggregation": "component_median",
                   "extrinsic": "identity_colocated_seed_pending_GT_assisted_rotation_fit",
                   "source_reindexed_temporary_copy": bool(temporary),
                   "possibly_truncated": bool(sequence.get("recover_unindexed_copy", False)),
                   "source_file_sha256": source_file_hashes([original])})


def simulation_state(t, settings):
    static = settings.get("bootstrap_static_s", 0.0)
    takeoff = settings["takeoff_s"]
    radius, period, height = settings["radius_m"], settings["period_s"], settings["height_m"]
    if t < static:
        return np.zeros(3), np.zeros(3), np.zeros(3), 0.0
    if t < static + takeoff:
        u = (t-static) / takeoff
        z = height * u*u*(3-2*u)
        vz = height * (6*u-6*u*u) / takeoff
        az = height * (6-12*u) / (takeoff*takeoff)
        return np.array([0, 0, z]), np.array([0, 0, vz]), np.array([0, 0, az]), 0.0
    s = t-static-takeoff; w = 2*math.pi/period
    p = np.array([radius*math.sin(w*s), .5*radius*math.sin(2*w*s), height])
    v = np.array([radius*w*math.cos(w*s), radius*w*math.cos(2*w*s), 0])
    a = np.array([-radius*w*w*math.sin(w*s), -2*radius*w*w*math.sin(2*w*s), 0])
    # Fixed heading keeps this benchmark focused on nominal UWB/IMU fusion and
    # avoids injecting an artificial yaw discontinuity at trajectory start.
    return p, v, a, 0.0


def prepare_simulation(config, sequence, cache):
    settings = config["simulation"]
    rng = np.random.default_rng(int(settings["seed"]))
    anchors = {i+1: np.array(p, float) for i, p in enumerate([
        [-5,-5,1],[-5,-5,5],[-5,5,1],[-5,5,5],
        [5,-5,1],[5,-5,5],[5,5,1],[5,5,5]])}
    lever = np.asarray(config["uwb"]["lever_arm_body_m"], float)
    imu, gt, uwb = [], [], []
    imu_times = np.arange(0, settings["duration_s"]+1e-9, .005)
    yaws = np.unwrap(np.array([simulation_state(t, settings)[3] for t in imu_times]))
    yaw_rates = np.gradient(yaws, imu_times)
    for t, yaw, yaw_rate in zip(imu_times, yaws, yaw_rates):
        p, _, a, _ = simulation_state(t, settings)
        q = Rotation.from_euler("z", yaw)
        specific = q.inv().apply(a + np.array([0, 0, 9.80665]))
        imu.append(dict(zip(common.IMU_FIELDS, [t, *specific, 0, 0, yaw_rate])))
        tag_p = p + q.apply(lever)
        gt.append(dict(zip(common.GT_FIELDS, [t, *tag_p, *q.as_quat(), 0])))
    for message_index, t in enumerate(np.arange(0.1, settings["duration_s"]+1e-9, .05)):
        p, _, _, yaw = simulation_state(t, settings)
        tag_p = p + Rotation.from_euler("z", yaw).apply(lever)
        for range_index, (aid, anchor) in enumerate(sorted(anchors.items())):
            # LinktrackNode2.dis is float32.  Quantize the deterministic fixture
            # at the source-message boundary so direct and ROS replay paths
            # consume byte-equivalent ranges.
            observed = float(np.float32(
                np.linalg.norm(tag_p-anchor) +
                rng.normal(0, config["uwb"]["sigma_m"])))
            uwb.append(uwb_row(t, message_index, range_index, 0, aid, observed,
                                config["uwb"]["sigma_m"], lever, anchor))
    source = Path(config["_path"])
    publish_cache(config, sequence, cache, source, imu, uwb, anchors, gt,
                  {"generator": "analytic_figure_eight_contract_fixture",
                   "settings": settings, "faults": False,
                   "source_file_sha256": source_file_hashes([source])})


PREPARERS = {"rosbag_huec": prepare_huec, "miluv_csv": prepare_miluv,
             "rosbag_sfuise": prepare_sfuise, "rosbag_own_vicon": prepare_own,
             "starloc_csv": prepare_starloc, "generated_simulation": prepare_simulation}


def _own_gyro_pairs(cache_dirs, offset_s):
    sensor, body = [], []
    half_width = 0.03
    for cache in cache_dirs:
        imu = pd.read_csv(cache/"imu.csv")
        gt = pd.read_csv(cache/"gt.csv").drop_duplicates("t").sort_values("t")
        slerp = Slerp(gt.t.to_numpy(), Rotation.from_quat(
            gt[["qx", "qy", "qz", "qw"]].to_numpy()))
        times = imu.t.to_numpy()[::5]
        gyro = imu[["gx", "gy", "gz"]].to_numpy()[::5]
        query = times + offset_s
        valid = ((query-half_width > gt.t.iloc[0]) &
                 (query+half_width < gt.t.iloc[-1]))
        omega = (slerp(query[valid]-half_width).inv() *
                 slerp(query[valid]+half_width)).as_rotvec()/(2*half_width)
        measured = gyro[valid]
        finite = ((np.linalg.norm(omega, axis=1) < 3.0) &
                  (np.linalg.norm(measured, axis=1) < 3.0))
        sensor.append(measured[finite]); body.append(omega[finite])
    return np.vstack(sensor), np.vstack(body)


def _wahba(source, target):
    u, _, vt = np.linalg.svd(source.T @ target)
    matrix = vt.T @ u.T
    if np.linalg.det(matrix) < 0:
        vt[-1] *= -1
        matrix = vt.T @ u.T
    error = target - source @ matrix.T
    return matrix, float(np.sqrt(np.mean(np.sum(error*error, axis=1))))


def fit_own_vicon_calibration(cache_dirs):
    def objective(offset):
        source, target = _own_gyro_pairs(cache_dirs, offset)
        return _wahba(source, target)[1]
    coarse = [(objective(value), value) for value in np.linspace(-0.2, 0.2, 81)]
    _, seed = min(coarse)
    optimum = minimize_scalar(objective, bounds=(seed-0.01, seed+0.01),
                              method="bounded", options={"xatol": 1e-5})
    offset = float(optimum.x)
    source, target = _own_gyro_pairs(cache_dirs, offset)
    matrix, gyro_rmse = _wahba(source, target)

    design, observed = [], []
    rotation = Rotation.from_matrix(matrix)
    for cache in cache_dirs:
        imu = pd.read_csv(cache/"imu.csv")
        gt = pd.read_csv(cache/"gt.csv").drop_duplicates("t").sort_values("t")
        lo = max(gt.t.iloc[0]+1.0, imu.t.iloc[0]-offset+1.0)
        hi = min(gt.t.iloc[-1]-1.0, imu.t.iloc[-1]-offset-1.0)
        if hi <= lo + 2.0:
            continue
        times = np.arange(lo, hi, 0.01)
        position = np.column_stack([
            np.interp(times, gt.t, gt[key]) for key in ("px", "py", "pz")])
        acceleration = savgol_filter(position, 51, 3, deriv=2,
                                     delta=0.01, axis=0)
        slerp = Slerp(gt.t.to_numpy(), Rotation.from_quat(
            gt[["qx", "qy", "qz", "qw"]].to_numpy()))
        orientation = slerp(times)
        half_width = 0.03
        omega = (slerp(times-half_width).inv() *
                 slerp(times+half_width)).as_rotvec()/(2*half_width)
        omega = savgol_filter(omega, 31, 3, axis=0)
        alpha = savgol_filter(omega, 31, 3, deriv=1, delta=0.01, axis=0)
        specific_sensor = np.column_stack([
            np.interp(times-offset, imu.t, imu[key])
            for key in ("ax", "ay", "az")])
        specific_body = rotation.apply(specific_sensor)
        expected_at_tag = orientation.inv().apply(
            acceleration + np.array([0.0, 0.0, 9.80665]))
        response = specific_body - expected_at_tag
        for w, a, y in zip(omega, alpha, response):
            if np.linalg.norm(w) < 0.05:
                continue
            skew_w = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]],
                               [-w[1], w[0], 0]])
            skew_a = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]],
                               [-a[1], a[0], 0]])
            design.append(np.hstack([skew_a + skew_w@skew_w, np.eye(3)]))
            observed.append(y)
    design = np.vstack(design); observed = np.hstack(observed)
    solution, _, _, _ = np.linalg.lstsq(design, observed, rcond=None)
    residual = observed - design@solution
    dof = max(1, len(observed)-len(solution))
    covariance = (residual@residual/dof) * np.linalg.inv(design.T@design)
    imu_from_tag = solution[:3]
    lever_tag_from_imu = -imu_from_tag
    return {
        "schema": "own-vicon-interface-calibration/v1",
        "label": "GT-assisted interface calibration",
        "scope": "all_three_recordings_joint_fit",
        "rotation_body_from_imu_matrix": matrix.tolist(),
        "rotation_body_from_imu_xyzw": Rotation.from_matrix(matrix).as_quat().tolist(),
        "time_imu_to_gt_s": offset,
        "lever_tag_from_imu_body_m": lever_tag_from_imu.tolist(),
        "accelerometer_bias_body_mps2": solution[3:].tolist(),
        "lever_std_m": np.sqrt(np.diag(covariance)[:3]).tolist(),
        "gyro_fit_rmse_radps": gyro_rmse,
        "accel_fit_rmse_mps2": float(np.sqrt(np.mean(residual**2))),
        "gyro_samples": int(len(source)), "accel_scalar_samples": int(len(observed)),
        "gt_use": "interface_rotation_time_and_translation_only",
    }


def apply_own_vicon_calibration(cache_dirs, calibration, artifact):
    rotation = Rotation.from_matrix(calibration["rotation_body_from_imu_matrix"])
    lever = np.asarray(calibration["lever_tag_from_imu_body_m"], float)
    offset = float(calibration["time_imu_to_gt_s"])
    for cache in cache_dirs:
        imu = pd.read_csv(cache/"imu.csv")
        imu[["ax", "ay", "az"]] = rotation.apply(imu[["ax", "ay", "az"]])
        imu[["gx", "gy", "gz"]] = rotation.apply(imu[["gx", "gy", "gz"]])
        imu["t"] += offset
        common.write_csv(cache/"imu.csv", common.IMU_FIELDS, imu.to_dict("records"))
        uwb = pd.read_csv(cache/"uwb.csv")
        uwb[["lever_x", "lever_y", "lever_z"]] = lever
        common.write_csv(cache/"uwb.csv", common.UWB_FIELDS, uwb.to_dict("records"))
        manifest_path = cache/"manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["provenance"]["extrinsic"] = "joint_GT_assisted_frozen"
        manifest["provenance"]["calibration_artifact"] = str(artifact)
        manifest["provenance"]["rotation_body_from_imu_matrix"] = calibration[
            "rotation_body_from_imu_matrix"]
        manifest["provenance"]["time_imu_to_gt_s"] = offset
        manifest["provenance"]["lever_tag_from_imu_body_m"] = lever.tolist()
        manifest["files"] = rows_file_metadata(cache)
        manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")


def calibrate_own_vicon(output):
    family = output/"cache/own_vicon"
    cache_dirs = sorted(path.parent for path in family.glob("*/manifest.json"))
    if len(cache_dirs) != 3:
        raise RuntimeError("own_vicon joint calibration requires all three prepared caches")
    calibration = fit_own_vicon_calibration(cache_dirs)
    artifact = output/"calibration/own_vicon_joint.json"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    artifact.write_text(json.dumps(calibration, indent=2)+"\n")
    apply_own_vicon_calibration(cache_dirs, calibration, artifact)
    return calibration


def fit_sfuise_world_calibration(cache_dirs):
    source_gt, target_uwb = [], []
    sequence_counts = {}
    for cache in cache_dirs:
        uwb = pd.read_csv(cache/"uwb.csv")
        gt = pd.read_csv(cache/"gt.csv").drop_duplicates("t").sort_values("t")
        positions, times = [], []
        seed = uwb[["anchor_x", "anchor_y", "anchor_z"]].drop_duplicates().mean().to_numpy()
        seed[2] -= 1.0
        for _, group in uwb[(uwb.valid != 0)].groupby("source_message", sort=True):
            if group.anchor_id.nunique() < 4:
                continue
            anchors = group[["anchor_x", "anchor_y", "anchor_z"]].to_numpy(float)
            ranges = group["range"].to_numpy(float)
            solution = least_squares(lambda point: np.linalg.norm(
                point[None, :]-anchors, axis=1)-ranges, seed,
                loss="soft_l1", f_scale=.2, max_nfev=40)
            if solution.success and np.isfinite(solution.x).all():
                seed = solution.x
                positions.append(seed.copy()); times.append(float(group.t.max()))
        times = np.asarray(times); positions = np.asarray(positions)
        valid = (times >= gt.t.iloc[0]) & (times <= gt.t.iloc[-1])
        times, positions = times[valid], positions[valid]
        gt_position = np.column_stack([np.interp(times, gt.t, gt[key])
                                       for key in ("px", "py", "pz")])
        source_gt.append(gt_position); target_uwb.append(positions)
        sequence_counts[cache.name] = int(len(times))
    source = np.vstack(source_gt); target = np.vstack(target_uwb)
    keep = np.ones(len(source), dtype=bool)
    for _ in range(5):
        rotation, translation = common.rigid_align(source[keep], target[keep])
        residual = np.linalg.norm((rotation@source.T).T+translation-target, axis=1)
        median = np.median(residual[keep]); mad = np.median(abs(residual[keep]-median))
        threshold = median + max(.15, 4.0*1.4826*mad)
        new_keep = residual <= threshold
        if np.array_equal(new_keep, keep):
            break
        keep = new_keep
    rotation, translation = common.rigid_align(source[keep], target[keep])
    residual = np.linalg.norm((rotation@source[keep].T).T+translation-target[keep], axis=1)
    return {"schema": "sfuise-world-interface-calibration/v1",
            "label": "GT-assisted interface calibration",
            "scope": "all_three_walks_joint_fit_missing_VIVE_to_anchor_transform",
            "rotation_anchor_from_vive_matrix": rotation.tolist(),
            "rotation_anchor_from_vive_xyzw": Rotation.from_matrix(rotation).as_quat().tolist(),
            "translation_anchor_from_vive_m": translation.tolist(),
            "fit_samples": int(keep.sum()), "rejected_samples": int((~keep).sum()),
            "residual_rmse_m": float(np.sqrt(np.mean(residual**2))),
            "residual_median_m": float(np.median(residual)),
            "sequence_counts": sequence_counts,
            "gt_use": "missing_world_frame_transform_only"}


def apply_sfuise_world_calibration(cache_dirs, calibration, artifact):
    rotation = Rotation.from_matrix(calibration["rotation_anchor_from_vive_matrix"])
    translation = np.asarray(calibration["translation_anchor_from_vive_m"], float)
    for cache in cache_dirs:
        gt = pd.read_csv(cache/"gt.csv")
        gt[["px", "py", "pz"]] = rotation.apply(gt[["px", "py", "pz"]]) + translation
        gt[["qx", "qy", "qz", "qw"]] = (
            rotation * Rotation.from_quat(gt[["qx", "qy", "qz", "qw"]])).as_quat()
        common.write_csv(cache/"gt.csv", common.GT_FIELDS, gt.to_dict("records"))
        manifest_path = cache/"manifest.json"; manifest = json.loads(manifest_path.read_text())
        manifest["calibration_label"] = "GT-assisted_interface_calibration"
        manifest["provenance"]["gt_world_transform"] = "joint_GT_assisted_frozen"
        manifest["provenance"]["world_calibration_artifact"] = str(artifact)
        manifest["files"] = rows_file_metadata(cache)
        manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")


def calibrate_sfuise_world(output):
    cache_dirs = sorted(path.parent for path in (output/"cache/SFUISE").glob("*/manifest.json"))
    if len(cache_dirs) != 3:
        raise RuntimeError("SFUISE world calibration requires all three prepared caches")
    calibration = fit_sfuise_world_calibration(cache_dirs)
    artifact = output/"calibration/sfuise_vive_to_anchor.json"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    artifact.write_text(json.dumps(calibration, indent=2)+"\n")
    apply_sfuise_world_calibration(cache_dirs, calibration, artifact)
    return calibration


def command_prepare(arguments):
    output = Path(arguments.output).resolve() / "cache"
    records = []
    for config in configs(arguments):
        for sequence in sequences(config):
            if not selected(config, sequence, arguments.sequence):
                continue
            cache = output / config["dataset"] / sequence["id"]
            started = time.monotonic()
            try:
                PREPARERS[config["format"]](config, sequence, cache)
                check = common.validate_cache(cache)
                status, reason = check["status"], ";".join(check["errors"])
            except Exception as error:
                status, reason = "FAIL", type(error).__name__ + ": " + str(error)
            record = {"dataset": config["dataset"], "sequence": sequence["id"],
                      "status": status, "reason": reason,
                      "wall_s": time.monotonic()-started, "cache": str(cache)}
            records.append(record)
            print(json.dumps(record), flush=True)
    own_records = [row for row in records if row["dataset"] == "own_vicon"]
    if own_records:
        try:
            artifact = Path(arguments.output).resolve()/"calibration/own_vicon_joint.json"
            if len(own_records) == 3 and all(row["status"] == "PASS" for row in own_records):
                calibration = calibrate_own_vicon(Path(arguments.output).resolve())
            elif artifact.is_file():
                calibration = json.loads(artifact.read_text())
                apply_own_vicon_calibration(
                    [Path(row["cache"]) for row in own_records if row["status"] == "PASS"],
                    calibration, artifact)
            else:
                raise RuntimeError("single-sequence own_vicon preparation needs an existing joint calibration")
            for row in own_records:
                if row["status"] == "PASS":
                    check = common.validate_cache(Path(row["cache"]))
                    row["status"] = check["status"]
                    row["reason"] = ";".join(check["errors"])
            print(json.dumps({"dataset": "own_vicon", "calibration": calibration}),
                  flush=True)
        except Exception as error:
            for row in own_records:
                row["status"] = "FAIL"
                row["reason"] = "CALIBRATION: " + type(error).__name__ + ": " + str(error)
    sfuise_records = [row for row in records if row["dataset"] == "SFUISE"]
    if sfuise_records:
        try:
            artifact = Path(arguments.output).resolve()/"calibration/sfuise_vive_to_anchor.json"
            if len(sfuise_records) == 3 and all(row["status"] == "PASS" for row in sfuise_records):
                calibration = calibrate_sfuise_world(Path(arguments.output).resolve())
            elif artifact.is_file():
                calibration = json.loads(artifact.read_text())
                apply_sfuise_world_calibration(
                    [Path(row["cache"]) for row in sfuise_records if row["status"] == "PASS"],
                    calibration, artifact)
            else:
                raise RuntimeError("single-sequence SFUISE preparation needs existing world calibration")
            for row in sfuise_records:
                if row["status"] == "PASS":
                    check = common.validate_cache(Path(row["cache"]))
                    row["status"] = check["status"]
                    row["reason"] = ";".join(check["errors"])
            print(json.dumps({"dataset": "SFUISE", "calibration": calibration}), flush=True)
        except Exception as error:
            for row in sfuise_records:
                row["status"] = "FAIL"
                row["reason"] = "CALIBRATION: " + type(error).__name__ + ": " + str(error)
    path = Path(arguments.output).resolve() / "preparation.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"records": records}, indent=2)+"\n")
    return 0 if records and all(r["status"] == "PASS" for r in records) else 1


def command_calibrate(arguments):
    output = Path(arguments.output).resolve()
    own_artifact = output/"calibration/own_vicon_joint.json"
    sfuise_artifact = output/"calibration/sfuise_vive_to_anchor.json"
    calibration = {
        "own_vicon": (json.loads(own_artifact.read_text()) if own_artifact.is_file()
                      else calibrate_own_vicon(output)),
        "SFUISE": (json.loads(sfuise_artifact.read_text()) if sfuise_artifact.is_file()
                   else calibrate_sfuise_world(output)),
    }
    print(json.dumps(calibration, indent=2))
    return 0


def command_preflight(arguments):
    root = Path(arguments.output).resolve() / "cache"
    records = []
    for manifest in sorted(root.glob("*/*/manifest.json")):
        check = common.validate_cache(manifest.parent)
        doc = json.loads(manifest.read_text())
        check.update(dataset=doc["dataset"], sequence=doc["sequence"])
        records.append(check)
        print(json.dumps(check), flush=True)
    expected = sum(len(sequences(c)) for c in configs(arguments))
    status = "PASS" if len(records) == expected and all(r["status"] == "PASS" for r in records) else "FAIL"
    result = {"status": status, "expected": expected, "observed": len(records), "records": records}
    (Path(arguments.output).resolve()/"preflight.json").write_text(json.dumps(result, indent=2)+"\n")
    return 0 if status == "PASS" else 1


def command_run(arguments):
    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    binary = Path(arguments.binary or default_nominal_binary())
    if arguments.method == "sfuise":
        raise RuntimeError("SFUISE execution requires tools/run_sfuise_benchmark.py")
    if not binary.is_file():
        raise FileNotFoundError(f"nominal runner not built: {binary}")
    records = []
    for manifest in sorted((output/"cache").glob("*/*/manifest.json")):
        doc = json.loads(manifest.read_text())
        if arguments.sequence and arguments.sequence not in (doc["dataset"], doc["sequence"]):
            continue
        variant = "current_all_tags" if arguments.tag_policy == "all" else "current"
        run = output/"runs"/variant/doc["dataset"]/doc["sequence"]
        run.mkdir(parents=True, exist_ok=True)
        # The estimator validates physical anchor IDs against its configuration.
        # Freeze the cache's audited map into a per-run config while leaving all
        # estimator/noise/solver parameters at the reviewed fde=off values.
        effective = yaml.safe_load((ROOT/"config/fde_off.yaml").read_text())
        # Integrity components are not constructed, but the strict config
        # loader still validates this referenced file.
        effective["fault_models"]["manifest_path"] = str(
            (ROOT/"config/integrity_fault_manifest.yaml").resolve())
        imu_times = np.asarray([float(row["t"]) for row in
                                common.read_csv(manifest.parent/"imu.csv")])
        observed_max_gap = float(np.max(np.diff(imu_times)))
        # This is an interface validity bound derived solely from sensor
        # timestamps, not an accuracy-tuned algorithm parameter.
        effective["imu"]["max_gap_s"] = max(
            float(effective["imu"]["max_gap_s"]), observed_max_gap + 1e-6)
        effective["incremental"]["max_time_skew_s"] = max(
            float(effective["incremental"]["max_time_skew_s"]),
            float(doc.get("epoch_batch_window_s", 0.0)) + 1e-6)
        effective["incremental"]["epoch_bin_s"] = max(
            float(effective["incremental"]["epoch_bin_s"]),
            effective["incremental"]["max_time_skew_s"])
        anchor_rows = common.read_csv(manifest.parent/"anchors.csv")
        effective["anchors"] = [
            {"id": int(row["anchor_id"]),
             "position_m": [float(row["x"]), float(row["y"]), float(row["z"])],
             "sigma_m": 0.05, "frame": effective["realtime"]["world_frame"],
             "map_version": f"benchmark-{doc['dataset']}-{doc['sequence']}"}
            for row in anchor_rows]
        effective_config = run/"effective_nominal_config.yaml"
        effective_config.write_text(yaml.safe_dump(effective, sort_keys=False))
        command = [str(binary), "--config", str(effective_config),
                   "--input", str(manifest.parent), "--output", str(run),
                   "--tag-policy", arguments.tag_policy]
        env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
                   MKL_NUM_THREADS="1", EIGEN_DONT_PARALLELIZE="1")
        process = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, env=env)
        (run/"console.log").write_text(process.stdout)
        record = {"dataset": doc["dataset"], "sequence": doc["sequence"],
                  "status": "SUCCESS" if process.returncode == 0 else "FAIL",
                  "exit_code": process.returncode, "command": command}
        records.append(record); print(json.dumps(record), flush=True)
    execution_name = "execution_current_all_tags.json" if arguments.tag_policy == "all" else "execution_current.json"
    # A targeted rerun updates its entries without erasing the auditable
    # statuses of the other sequences.  Reconstruct from run directories as a
    # fallback when an older targeted invocation already truncated the index.
    indexed = {(row["dataset"], row["sequence"]): row for row in records}
    for status_path in sorted((output/"runs"/variant).glob("*/*/run_status.json")):
        dataset, sequence = status_path.parent.parent.name, status_path.parent.name
        if (dataset, sequence) in indexed:
            continue
        status_doc = json.loads(status_path.read_text())
        indexed[(dataset, sequence)] = {
            "dataset": dataset, "sequence": sequence,
            "status": status_doc.get("status", "FAIL"),
            "exit_code": 0 if status_doc.get("status") == "SUCCESS" else 1,
            "reason": status_doc.get("reason", ""), "command": "RECOVERED_FROM_RUN_STATUS"}
    all_records = [indexed[key] for key in sorted(indexed)]
    (output/execution_name).write_text(json.dumps({"records": all_records}, indent=2)+"\n")
    return 0 if records and all(r["status"] == "SUCCESS" for r in records) else 1


def command_evaluate(arguments):
    output = Path(arguments.output).resolve()
    records, samples_root = [], output/"evaluation/samples"
    samples_root.mkdir(parents=True, exist_ok=True)
    for manifest in sorted((output/"cache").glob("*/*/manifest.json")):
        doc = json.loads(manifest.read_text())
        if arguments.sequence and arguments.sequence not in (doc["dataset"], doc["sequence"]):
            continue
        methods = ["current", "sfuise"]
        if (output/"runs/current_all_tags"/doc["dataset"]/doc["sequence"]/"trajectory.tum").is_file():
            methods.append("current_all_tags")
        for method in methods:
            trajectory = output/"runs"/method/doc["dataset"]/doc["sequence"]/"trajectory.tum"
            base = {"dataset": doc["dataset"], "sequence": doc["sequence"],
                    "method": method, "primary_tag": doc["primary_tag"]}
            try:
                status_path = trajectory.parent/"run_status.json"
                if status_path.is_file():
                    run_status = json.loads(status_path.read_text())
                    if run_status.get("status") != "SUCCESS":
                        raise RuntimeError("RUN_FAILED: " + run_status.get("reason", "unknown"))
                if not trajectory.is_file():
                    raise FileNotFoundError("trajectory.tum missing")
                metrics, samples = common.evaluate_trajectories(
                    common.load_tum(trajectory), common.load_gt(manifest.parent/"gt.csv", doc["primary_tag"]))
                base.update(metrics)
                sample_path = samples_root/f"{doc['dataset']}__{doc['sequence']}__{method}.json"
                sample_path.write_text(json.dumps(samples)+"\n")
            except Exception as error:
                base.update(status="FAIL", reason=type(error).__name__+": "+str(error), coverage=0.0)
            if base.get("coverage", 0) < 0.8 and base["status"] == "SUCCESS":
                base["aggregate_eligible"] = False
                base["reason"] = "COVERAGE_BELOW_0.8"
            else:
                base["aggregate_eligible"] = base["status"] == "SUCCESS"
            records.append(base); print(json.dumps(base), flush=True)
    evaluation = output/"evaluation"; evaluation.mkdir(exist_ok=True)
    (evaluation/"metrics.json").write_text(json.dumps({"records": records}, indent=2)+"\n")
    fields = sorted({key for row in records for key in row})
    common.write_csv(evaluation/"metrics.csv", fields, records)
    return 0


def command_timing(arguments):
    output = Path(arguments.output).resolve()
    binary = Path(arguments.binary or default_nominal_binary())
    records = []
    representatives = []
    for manifest_path in sorted((output/"cache").glob("*/*/manifest.json")):
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("timing_representative"):
            representatives.append((manifest_path, manifest))
    for manifest_path, manifest in representatives:
        natural = output/"runs/current"/manifest["dataset"]/manifest["sequence"]
        effective = natural/"effective_nominal_config.yaml"
        if not effective.is_file():
            raise FileNotFoundError(f"natural run config missing: {effective}")
        root = output/"timing/current"/manifest["dataset"]/manifest["sequence"]
        for repeat in range(0, 6):
            run = root/("warmup" if repeat == 0 else f"repeat_{repeat}")
            run.mkdir(parents=True, exist_ok=True)
            command = [str(binary), "--config", str(effective), "--input",
                       str(manifest_path.parent), "--output", str(run),
                       "--tag-policy", "primary"]
            env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1",
                       MKL_NUM_THREADS="1", EIGEN_DONT_PARALLELIZE="1")
            process = subprocess.run(command, env=env, text=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            (run/"console.log").write_text(process.stdout)
            status = json.loads((run/"run_status.json").read_text())
            if process.returncode or status["status"] != "SUCCESS":
                records.append({"dataset": manifest["dataset"],
                                "sequence": manifest["sequence"],
                                "repeat": repeat, "status": "FAIL",
                                "reason": status.get("reason", "runner failed")})
                continue
            if repeat == 0:
                continue
            timing = pd.read_csv(run/"timing.csv")
            row = {"dataset": manifest["dataset"],
                   "sequence": manifest["sequence"], "repeat": repeat,
                   "status": "SUCCESS", "epochs": int(status["epochs"]),
                   "cpu_affinity": ",".join(map(str, sorted(os.sched_getaffinity(0)))),
                   "build_type": "Release",
                   "wall_ms": status["wall_ms"], "cpu_ms": status["cpu_ms"],
                   "adapter_ms": status["adapter_ms"],
                   "bootstrap_ms": status["bootstrap_ms"],
                   "sensor_duration_s": common.csv_duration(manifest_path.parent/"imu.csv")}
            row["throughput_epochs_s"] = row["epochs"]/(row["wall_ms"]*1e-3)
            row["real_time_factor"] = row["wall_ms"]*1e-3/max(row["sensor_duration_s"], 1e-9)
            for column in ("imu_preintegration_ms", "prepare_wall_ms",
                           "no_uwb_update_ms", "commit_wall_ms", "uwb_update_ms",
                           "state_query_wall_ms", "state_query_internal_ms",
                           "snapshot_extraction_ms"):
                values = timing[column].to_numpy(float)
                for label, percentile in (("p50", 50), ("p95", 95),
                                          ("p99", 99), ("max", 100)):
                    row[f"{column}_{label}"] = float(np.percentile(values, percentile))
            records.append(row)
            print(json.dumps(row), flush=True)
    target = output/"timing/current_summary.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps({"schema": "current-timing/v1", "records": records},
                                 indent=2)+"\n")
    fields = sorted({key for row in records for key in row})
    common.write_csv(output/"timing/current_summary.csv", fields, records)
    return 0 if records and all(row["status"] == "SUCCESS" for row in records) else 1


def command_report(arguments):
    output = Path(arguments.output).resolve()
    report_root = ROOT/"docs/benchmark"; figures = report_root/"figures"
    if figures.exists():
        shutil.rmtree(figures)
    figures.mkdir(parents=True, exist_ok=True)
    metrics_path = output/"evaluation/metrics.json"
    records = json.loads(metrics_path.read_text())["records"] if metrics_path.exists() else []
    passed = [r for r in records if r.get("aggregate_eligible")]
    sample_root = output/"evaluation/samples"
    figure_links = defaultdict(list)
    datasets = sorted({r["dataset"] for r in records})
    for dataset in datasets:
        rows = [r for r in passed if r["dataset"] == dataset]
        methods = [method for method in ("current", "sfuise")
                   if any(row["method"] == method for row in rows)]
        if not methods:
            continue
        # Per-sequence APE and 1 s RPE distributions.
        ape_values = [[row["ape_translation_m_rmse"] for row in rows
                       if row["method"] == method] for method in methods]
        fig, ax = plt.subplots(figsize=(6.4, 3.8))
        ax.boxplot(ape_values, labels=methods, showfliers=False)
        ax.set_ylabel("raw-frame APE RMSE [m]"); ax.grid(True, axis="y", alpha=.3)
        path = figures/f"{dataset}_ape_box.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
        figure_links[dataset].append(("逐序列 APE RMSE", path.relative_to(report_root)))
        rpe_values = [[row["rpe_1s_translation_m_rmse"] for row in rows
                       if row["method"] == method and "rpe_1s_translation_m_rmse" in row]
                      for method in methods]
        if all(rpe_values):
            fig, ax = plt.subplots(figsize=(6.4, 3.8))
            ax.boxplot(rpe_values, labels=methods, showfliers=False)
            ax.set_ylabel("1 s translation RPE RMSE [m]"); ax.grid(True, axis="y", alpha=.3)
            path = figures/f"{dataset}_rpe_box.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
            figure_links[dataset].append(("逐序列 1 s RPE", path.relative_to(report_root)))
        fig, ax = plt.subplots(figsize=(6.4, 3.8))
        for method in methods:
            values = []
            for row in rows:
                if row["method"] != method:
                    continue
                sample = sample_root/f"{dataset}__{row['sequence']}__{method}.json"
                if sample.is_file():
                    values.extend(json.loads(sample.read_text())["ape_translation_m"])
            if values:
                ordered = np.sort(values)
                ax.plot(ordered, np.arange(1, len(ordered)+1)/len(ordered), label=method)
        ax.set_xlabel("raw-frame APE [m]"); ax.set_ylabel("CDF"); ax.grid(True, alpha=.3); ax.legend()
        path = figures/f"{dataset}_ape_cdf.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
        figure_links[dataset].append(("pooled APE CDF", path.relative_to(report_root)))
        paired = []
        for sequence in sorted({row["sequence"] for row in rows}):
            current = next((row for row in rows if row["sequence"] == sequence and row["method"] == "current"), None)
            baseline = next((row for row in rows if row["sequence"] == sequence and row["method"] == "sfuise"), None)
            if current and baseline:
                paired.append((sequence, current["ape_translation_m_rmse"]-
                               baseline["ape_translation_m_rmse"]))
        if paired:
            fig, ax = plt.subplots(figsize=(max(6.4, .3*len(paired)), 3.8))
            ax.bar(range(len(paired)), [item[1] for item in paired])
            ax.axhline(0, color="black", linewidth=.8)
            ax.set_xticks(range(len(paired)), [item[0] for item in paired], rotation=70, ha="right")
            ax.set_ylabel("current − SFUISE APE RMSE [m]"); ax.grid(True, axis="y", alpha=.3)
            path = figures/f"{dataset}_paired_delta.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
            figure_links[dataset].append(("成对精度差", path.relative_to(report_root)))

    # Coverage for every run, including failures as zero.
    if records:
        labels = [f"{row['dataset']}/{row['sequence']}/{row['method']}" for row in records]
        fig, ax = plt.subplots(figsize=(10, max(4, .12*len(records))))
        ax.barh(range(len(records)), [row.get("coverage", 0) for row in records])
        ax.axvline(.8, color="red", linestyle="--", linewidth=1)
        ax.set_yticks(range(len(records)), labels, fontsize=6); ax.set_xlim(0, 1.02)
        ax.set_xlabel("coverage"); ax.grid(True, axis="x", alpha=.25)
        path = figures/"all_coverage.png"; fig.tight_layout(); fig.savefig(path, dpi=180); plt.close(fig)
        figure_links["all"].append(("全部运行 coverage", path.relative_to(report_root)))

    # Representative trajectory and APE time-series plots.
    for manifest_path in sorted((output/"cache").glob("*/*/manifest.json")):
        manifest = json.loads(manifest_path.read_text())
        if not manifest.get("timing_representative"):
            continue
        dataset, sequence = manifest["dataset"], manifest["sequence"]
        gt = common.load_gt(manifest_path.parent/"gt.csv", manifest["primary_tag"])
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        axes[0].plot(gt.position[:, 0], gt.position[:, 1], "k--", label="GT", linewidth=1)
        axes[1].plot(gt.time, gt.position[:, 2], "k--", label="GT z", linewidth=1)
        have = False
        for method in ("current", "sfuise"):
            trajectory_path = output/"runs"/method/dataset/sequence/"trajectory.tum"
            if trajectory_path.is_file() and trajectory_path.stat().st_size:
                trajectory = common.load_tum(trajectory_path)
                axes[0].plot(trajectory.position[:, 0], trajectory.position[:, 1], label=method, linewidth=.9)
                axes[1].plot(trajectory.time, trajectory.position[:, 2], label=method, linewidth=.9)
                have = True
        if have:
            axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]"); axes[0].axis("equal")
            axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("z [m]")
            for ax in axes: ax.grid(True, alpha=.25); ax.legend()
            path = figures/f"{dataset}_{sequence}_trajectory.png"; fig.tight_layout(); fig.savefig(path, dpi=160)
            figure_links[dataset].append((f"{sequence} XY/高度轨迹", path.relative_to(report_root)))
            fig3d = plt.figure(figsize=(6.4, 5.0)); ax3d = fig3d.add_subplot(111, projection="3d")
            ax3d.plot(gt.position[:, 0], gt.position[:, 1], gt.position[:, 2],
                      "k--", label="GT", linewidth=1)
            for method in ("current", "sfuise"):
                trajectory_path = output/"runs"/method/dataset/sequence/"trajectory.tum"
                if trajectory_path.is_file() and trajectory_path.stat().st_size:
                    trajectory = common.load_tum(trajectory_path)
                    ax3d.plot(trajectory.position[:, 0], trajectory.position[:, 1],
                              trajectory.position[:, 2], label=method, linewidth=.8)
            ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
            ax3d.legend(); ax3d.grid(True, alpha=.25)
            path3d = figures/f"{dataset}_{sequence}_trajectory_3d.png"
            fig3d.tight_layout(); fig3d.savefig(path3d, dpi=160); plt.close(fig3d)
            figure_links[dataset].append((f"{sequence} 3D 轨迹", path3d.relative_to(report_root)))
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(8, 3.5)); have = False
        for method in ("current", "sfuise"):
            sample = sample_root/f"{dataset}__{sequence}__{method}.json"
            if sample.is_file():
                values = json.loads(sample.read_text())
                ax.plot(values["time"], values["ape_translation_m"], label=method, linewidth=.8)
                have = True
        if have:
            ax.set_xlabel("t [s]"); ax.set_ylabel("APE [m]"); ax.set_yscale("log")
            ax.grid(True, alpha=.25); ax.legend()
            path = figures/f"{dataset}_{sequence}_ape_time.png"; fig.tight_layout(); fig.savefig(path, dpi=160)
            figure_links[dataset].append((f"{sequence} APE 时间序列", path.relative_to(report_root)))
        plt.close(fig)

    timing_path = output/"timing/current_summary.json"
    timing_records = json.loads(timing_path.read_text())["records"] if timing_path.is_file() else []
    timing_ok = [row for row in timing_records if row.get("status") == "SUCCESS"]
    if timing_ok:
        grouped = defaultdict(list)
        for row in timing_ok: grouped[(row["dataset"], row["sequence"])].append(row)
        labels, prepare, commit, query = [], [], [], []
        for key, values in grouped.items():
            labels.append("/".join(key))
            prepare.append(np.median([row["prepare_wall_ms_p50"] for row in values]))
            commit.append(np.median([row["commit_wall_ms_p50"] for row in values]))
            query.append(np.median([row["state_query_wall_ms_p50"] for row in values]))
        fig, ax = plt.subplots(figsize=(9, 4)); x = np.arange(len(labels))
        ax.bar(x, prepare, label="prepare"); ax.bar(x, commit, bottom=prepare, label="commit/solver")
        ax.bar(x, query, bottom=np.asarray(prepare)+commit, label="state query")
        ax.set_xticks(x, labels, rotation=35, ha="right"); ax.set_ylabel("per-epoch p50 [ms]")
        ax.grid(True, axis="y", alpha=.25); ax.legend()
        path = figures/"current_timing_stack.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
        figure_links["timing"].append(("current 阶段耗时", path.relative_to(report_root)))
        fig, ax = plt.subplots(figsize=(9, 4)); width = .18
        quantiles = ("p50", "p95", "p99", "max")
        for offset, quantile in enumerate(quantiles):
            values = [np.median([row[f"commit_wall_ms_{quantile}"]
                                 for row in grouped[key]])
                      for key in grouped]
            ax.bar(x + (offset-1.5)*width, values, width=width, label=quantile)
        ax.set_xticks(x, labels, rotation=35, ha="right")
        ax.set_ylabel("UWB commit/solver [ms]"); ax.set_yscale("log")
        ax.grid(True, axis="y", alpha=.25); ax.legend()
        path = figures/"current_timing_quantiles.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
        figure_links["timing"].append(("current solver 分位耗时", path.relative_to(report_root)))
    sfuise_timing_path = output/"timing/sfuise_summary.json"
    if sfuise_timing_path.is_file():
        sfuise_timing = json.loads(sfuise_timing_path.read_text())["records"]
    elif (output/"execution_sfuise.json").is_file():
        sfuise_timing = json.loads((output/"execution_sfuise.json").read_text())["records"]
    else:
        sfuise_timing = []
    sfuise_timing_ok = [row for row in sfuise_timing if row.get("status") == "SUCCESS"
                        and "wall_s" in row]
    natural_runs = []
    for method in ("current", "sfuise"):
        for status_path in sorted((output/"runs"/method).glob("*/*/run_status.json")):
            status = json.loads(status_path.read_text())
            natural_runs.append({"method": method,
                "dataset": status_path.parent.parent.name,
                "sequence": status_path.parent.name, **status})
    if sfuise_timing_ok:
        fig, ax = plt.subplots(figsize=(max(8, .22*len(sfuise_timing_ok)), 4))
        labels = [f"{row['dataset']}/{row['sequence']}" for row in sfuise_timing_ok]
        ax.bar(range(len(labels)), [row["wall_s"] for row in sfuise_timing_ok])
        ax.set_xticks(range(len(labels)), labels, rotation=70, ha="right", fontsize=7)
        ax.set_ylabel("SFUISE end-to-end wall [s]"); ax.grid(True, axis="y", alpha=.25)
        path = figures/"sfuise_wall.png"; fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)
        figure_links["timing"].append(("SFUISE 端到端墙钟", path.relative_to(report_root)))

    preflight_path = output/"preflight.json"
    preflight = json.loads(preflight_path.read_text()) if preflight_path.exists() else {"status":"NOT_RUN","records":[]}
    family_summary = []
    for dataset in datasets:
        for method in ("current", "sfuise"):
            rows = [row for row in passed if row["dataset"] == dataset and row["method"] == method]
            if not rows: continue
            pooled = []
            pooled_rpe = []
            for row in rows:
                sample = sample_root/f"{dataset}__{row['sequence']}__{method}.json"
                if sample.is_file():
                    sample_values = json.loads(sample.read_text())
                    pooled.extend(sample_values["ape_translation_m"])
                    pooled_rpe.extend(sample_values.get("rpe_1s_translation_m", []))
            sequence_rpe = [row["rpe_1s_translation_m_rmse"] for row in rows
                            if "rpe_1s_translation_m_rmse" in row]
            family_summary.append({"dataset": dataset, "method": method,
                "sequences": len(rows),
                "equal_sequence_ape_rmse_mean": float(np.mean([row["ape_translation_m_rmse"] for row in rows])),
                "pooled_ape_rmse": float(np.sqrt(np.mean(np.asarray(pooled)**2))) if pooled else None,
                "equal_sequence_rpe_rmse_mean": float(np.mean(sequence_rpe)) if sequence_rpe else None,
                "pooled_rpe_rmse": float(np.sqrt(np.mean(np.asarray(pooled_rpe)**2))) if pooled_rpe else None})
    own_cal_path = output/"calibration/own_vicon_joint.json"
    sf_cal_path = output/"calibration/sfuise_vive_to_anchor.json"
    own_cal = json.loads(own_cal_path.read_text()) if own_cal_path.is_file() else None
    sf_cal = json.loads(sf_cal_path.read_text()) if sf_cal_path.is_file() else None
    method_counts = {method: (sum(row["method"] == method for row in records),
                              sum(row["method"] == method and row.get("aggregate_eligible")
                                  for row in records))
                     for method in ("current", "sfuise", "current_all_tags")}
    lines = ["# UWB–IMU 主流程精度与耗时报告", "",
             "> 本报告由 `tools/run_dataset_benchmark.py report` 自动生成。",
             "> 标定口径：GT-assisted interface calibration；GT 未作为状态初值或算法调参输入。",
             "> SFUISE 保留官方 UWB 残差剔除，因此不是与 current nominal 完全对称的算法边界。", "",
             "## 结论摘要", "",
             f"- current 主表有效/总计：{method_counts['current'][1]}/{method_counts['current'][0]}；"
             f"官方 SFUISE baseline：{method_counts['sfuise'][1]}/{method_counts['sfuise'][0]}；"
             f"current 双标签补充：{method_counts['current_all_tags'][1]}/{method_counts['current_all_tags'][0]}。",
             "- 数值只汇总 coverage ≥80% 且运行成功的序列；有限但显著发散的轨迹仍按实验合同保留，"
             "可由 APE max、时间序列和轨迹图识别。", "",
             "## 数据接口审计", "",
             "| 数据族 | IMU 接口与单位 | 坐标/杆臂 | UWB/anchor | GT 与标定口径 |",
             "|---|---|---|---|---|",
             "| HUEC | `/imu/data`；acc m/s²、gyro rad/s（不执行错误的 deg→rad） | body；tag lever `[-0.055,-0.055,0.668]` m | 4 anchor topics，topic 自带坐标 | `/odometry/local_gps` tag pose；数据自带标定 |",
             "| MILUV | `imu_px4.csv`；m/s²、rad/s | PX4 body；官方 tag10/11 杆臂 | processed `range`；官方 constellation | mocap body pose 投影到 tag；官方标定 |",
             "| SFUISE | Waveshare IMU；m/s²、rad/s | IMU/body；lever `[0.1,-0.025,0]` m | `/rtls_flares` + anchor list；Walk ToA offset | VIVE body 投影到 tag 后拟合 VIVE→anchor world 固定变换（GT-assisted） |",
             "| own_vicon | Livox acc ×9.80665；gyro rad/s | 联合拟合 IMU→tag 固定旋转/杆臂/时延 | bag 内 anchor 1–4 Vicon 稳健中值 | tag0；三录制联合 GT-assisted interface calibration；原 bag 不修改 |",
             "| STAR-Loc | acc 已为 rig m/s²；gyro rad/s | `C_r_i=C_c_rᵀC_c_i`；CAD tag 杆臂 | setup 选择 v1/v2/v3；s1–s4 `range_calib`，s5 `range`；v1 marker ID 固定映射到 radio ID | `uwb.csv` 同时刻官方 `tag_pos_*`；官方几何，v1 ID 对应由 v2 房间几何恢复 |",
             "| simulation | 200 Hz m/s²、rad/s | body；冻结 tag 杆臂 | 8 anchors；seed 20260901；无丢包/NLOS | 解析真值仅供评价；measurement bootstrap |", "",
             f"Preflight：**{preflight['status']}**；发现 {len(preflight.get('records', []))} 条缓存。", "",
             "| 数据集 | 序列 | 状态 | IMU | UWB | IMU Hz | 初始 | GT覆盖 | 距离几何P95 [m] |", "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in preflight.get("records", []):
        val = lambda key, digits=3: "NA" if row.get(key) is None else f"{row[key]:.{digits}f}"
        lines.append(f"| {row['dataset']} | {row['sequence']} | {row['status']} | {row['imu_count']} | {row['uwb_count']} | {val('imu_frequency_hz',1)} | {val('initial_accel_norm_median')} | {val('gt_time_coverage')} | {val('range_geometry_abs_residual_p95')} |")
    lines += ["", "### GT-assisted 固定接口标定", ""]
    if own_cal:
        lines.append(
            f"- own_vicon：联合 3 条录制，时延 {own_cal['time_imu_to_gt_s']*1e3:.3f} ms，"
            f"杆臂 `{np.round(own_cal['lever_tag_from_imu_body_m'],6).tolist()}` m，"
            f"杆臂 1σ `{np.round(own_cal['lever_std_m'],6).tolist()}` m；"
            f"gyro/accel 拟合 RMSE 分别为 {own_cal['gyro_fit_rmse_radps']:.4f} rad/s、"
            f"{own_cal['accel_fit_rmse_mps2']:.4f} m/s²。")
    if sf_cal:
        lines.append(
            f"- SFUISE：联合 3 条 Walk 的缺失 VIVE→anchor world 变换，"
            f"保留/剔除 {sf_cal['fit_samples']}/{sf_cal['rejected_samples']} 个拟合样本，"
            f"残差 RMSE/median 为 {sf_cal['residual_rmse_m']:.4f}/"
            f"{sf_cal['residual_median_m']:.4f} m。")
    lines.append(
        "- STAR-Loc v1：`uwb_markers_v1.csv` 首列为 Vicon marker ID，数据量测使用 radio ID；"
        "按 v2 不变房间几何冻结映射 `10→11, 11→7, 12→4, 13→10, 15→5, 16→6, 6→12, 9→9`。"
        "`gt_range` 仅用于映射后的独立残差审计，未参与拟合。")
    lines += ["", "接口规则包括：HUEC 保持原始 m/s²、rad/s；MILUV 使用 PX4 与官方 constellation/tag 杆臂；SFUISE 使用官方 ToA offset；own_vicon 使用三条录制联合 GT 辅助接口标定；STAR-Loc 旋转角速度、恢复 v1 marker/radio ID 对应，并按 setup 选 anchor/range 字段。", "",
              "## 族级汇总", "", "| 数据集 | 方法 | 有效序列 | 序列等权 APE RMSE [m] | pooled APE RMSE [m] | 序列等权 RPE-1s RMSE [m] | pooled RPE-1s RMSE [m] |", "|---|---|---:|---:|---:|---:|---:|"]
    for row in family_summary:
        pooled = "NA" if row["pooled_ape_rmse"] is None else f"{row['pooled_ape_rmse']:.4f}"
        equal_rpe = "NA" if row["equal_sequence_rpe_rmse_mean"] is None else f"{row['equal_sequence_rpe_rmse_mean']:.4f}"
        pooled_rpe = "NA" if row["pooled_rpe_rmse"] is None else f"{row['pooled_rpe_rmse']:.4f}"
        lines.append(f"| {row['dataset']} | {row['method']} | {row['sequences']} | {row['equal_sequence_ape_rmse_mean']:.4f} | {pooled} | {equal_rpe} | {pooled_rpe} |")
    lines += ["", "## 逐序列精度结果", "",
              "主指标为原 anchor/world 坐标系、相同 UWB 标签点上的 APE；coverage < 80% 不进入汇总。", "",
              "| 数据集 | 序列 | 方法 | 状态/原因 | Coverage | APE RMSE | mean | median | P95 | max | 对齐 APE | RPE-1s 平移 RMSE/median/P95 | RPE-1s 旋转 RMSE/median/P95 [deg] | 姿态 APE RMSE [deg] |", "|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in records:
        def f(key):
            return "NA" if key not in row else f"{float(row[key]):.4f}"
        state = (row["status"] if row.get("aggregate_eligible") else
                 row.get("reason", row["status"]))
        state = str(state).replace("\n", " ").replace("|", "/")
        if len(state) > 180:
            state = state[:177] + "..."
        rpe = "/".join((f('rpe_1s_translation_m_rmse'),
                         f('rpe_1s_translation_m_median'),
                         f('rpe_1s_translation_m_p95')))
        rpe_rotation = "/".join((f('rpe_1s_rotation_deg_rmse'),
                                  f('rpe_1s_rotation_deg_median'),
                                  f('rpe_1s_rotation_deg_p95')))
        lines.append(f"| {row['dataset']} | {row['sequence']} | {row['method']} | {state} | {float(row.get('coverage',0)):.3f} | {f('ape_translation_m_rmse')} | {f('ape_translation_m_mean')} | {f('ape_translation_m_median')} | {f('ape_translation_m_p95')} | {f('ape_translation_m_max')} | {f('ape_se3_aligned_translation_m_rmse')} | {rpe} | {rpe_rotation} | {f('ape_rotation_deg_rmse')} |")
    paired_rows = []
    for dataset in datasets:
        for sequence in sorted({row["sequence"] for row in records if row["dataset"] == dataset}):
            current = next((row for row in passed if row["dataset"] == dataset and
                            row["sequence"] == sequence and row["method"] == "current"), None)
            baseline = next((row for row in passed if row["dataset"] == dataset and
                             row["sequence"] == sequence and row["method"] == "sfuise"), None)
            if current and baseline:
                paired_rows.append((dataset, sequence,
                    current["ape_translation_m_rmse"]-baseline["ape_translation_m_rmse"]))
    lines += ["", "### 公共有效序列的成对差值", "",
              "负值表示 current 的 raw-frame APE RMSE 更小。", "",
              "| 数据集 | 序列 | current − SFUISE [m] |", "|---|---|---:|"]
    for dataset, sequence, delta in paired_rows:
        lines.append(f"| {dataset} | {sequence} | {delta:.4f} |")
    lines += ["", "## 图表", ""]
    if figure_links:
        for dataset, items in figure_links.items():
            lines += [f"### {dataset}", ""]
            for title, path in items:
                lines += [f"{title}", "", f"![{title}]({path.as_posix()})", ""]
    else:
        lines += ["尚无同时通过 coverage 门限的精度结果。失败/缺失项已保留在上表。", ""]
    lines += ["## 耗时分析", "",
              "current runner 的 `timing.csv` 分解 adapter/bootstrap、IMU preintegration、prepare、commit、state query；SFUISE 使用其原生 average-window runtime 和进程墙钟，二者不作逐阶段直接排名。", "",
              "### 全量自然运行", "",
              "| 数据集 | 序列 | 方法 | 状态 | 墙钟 [s] | CPU [s] | epoch/轨迹样本 | 原因 |",
              "|---|---|---|---|---:|---:|---:|---|"]
    for row in natural_runs:
        wall_s = (float(row.get("wall_ms", 0.0))*1e-3 if row["method"] == "current"
                  else float(row.get("wall_s", 0.0)))
        cpu_s = (float(row.get("cpu_ms", 0.0))*1e-3 if row["method"] == "current"
                 else float(row.get("cpu_s", 0.0)))
        count = row.get("epochs", row.get("trajectory_samples", 0))
        reason = str(row.get("reason", "")).replace("\n", " ").replace("|", "/")
        if len(reason) > 120:
            reason = reason[:117] + "..."
        lines.append(f"| {row['dataset']} | {row['sequence']} | {row['method']} | {row.get('status','FAIL')} | {wall_s:.3f} | {cpu_s:.3f} | {count} | {reason} |")
    lines += ["", "### current 稳定计时（warm-up 后 5 次）", "",
              "| 数据集 | 序列 | repeat | CPU affinity | 墙钟 [ms] | CPU [ms] | adapter [ms] | bootstrap [ms] | throughput [epoch/s] | RTF |", "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in timing_ok:
        lines.append(f"| {row['dataset']} | {row['sequence']} | {row['repeat']} | {row.get('cpu_affinity','NA')} | {row['wall_ms']:.2f} | {row['cpu_ms']:.2f} | {row['adapter_ms']:.2f} | {row['bootstrap_ms']:.2f} | {row['throughput_epochs_s']:.1f} | {row['real_time_factor']:.4f} |")
    lines += ["", "current 端到端 5 次统计：", "",
              "| 数据集/序列 | wall p50/p95/p99/max [ms] | CPU p50 [ms] | throughput p50 [epoch/s] | RTF p50 |",
              "|---|---:|---:|---:|---:|"]
    lines += ["", "### current 每 epoch 阶段分位数（5 次 repeat 的中位数）", "",
              "| 数据集/序列 | 阶段 | p50 [ms] | p95 [ms] | p99 [ms] | max [ms] |", "|---|---|---:|---:|---:|---:|"]
    timing_groups = defaultdict(list)
    for row in timing_ok:
        timing_groups[(row["dataset"], row["sequence"])].append(row)
    # Populate the aggregate table after grouping.  It is inserted before the
    # per-epoch heading to keep the report's timing narrative compact.
    aggregate_rows = []
    for key, group in timing_groups.items():
        wall = np.asarray([row["wall_ms"] for row in group], float)
        aggregate_rows.append(
            f"| {'/'.join(key)} | "
            f"{np.percentile(wall,50):.2f}/{np.percentile(wall,95):.2f}/"
            f"{np.percentile(wall,99):.2f}/{np.max(wall):.2f} | "
            f"{np.median([row['cpu_ms'] for row in group]):.2f} | "
            f"{np.median([row['throughput_epochs_s'] for row in group]):.1f} | "
            f"{np.median([row['real_time_factor'] for row in group]):.4f} |")
    aggregate_heading_index = lines.index("### current 每 epoch 阶段分位数（5 次 repeat 的中位数）")
    aggregate_insert_index = (aggregate_heading_index - 1
                              if lines[aggregate_heading_index - 1] == ""
                              else aggregate_heading_index)
    lines[aggregate_insert_index:aggregate_insert_index] = aggregate_rows
    phase_labels = (("imu_preintegration_ms", "IMU preintegration"),
                    ("prepare_wall_ms", "prepare total"),
                    ("no_uwb_update_ms", "no-UWB update"),
                    ("commit_wall_ms", "UWB commit/solver"),
                    ("state_query_wall_ms", "state query"))
    for key, group in timing_groups.items():
        for field, label in phase_labels:
            values = [float(np.median([row[f"{field}_{q}"] for row in group]))
                      for q in ("p50", "p95", "p99", "max")]
            lines.append(f"| {'/'.join(key)} | {label} | {values[0]:.4f} | {values[1]:.4f} | {values[2]:.4f} | {values[3]:.4f} |")
    lines += ["", "### SFUISE 端到端耗时", "",
              "SFUISE 上游 CMake 强制写入 `Debug` 类型；隔离工作区不改源码，通过 `-O3 -DNDEBUG` 形成 Release-equivalent 二进制，报告保留该差异。", "",
              "| 数据集 | 序列 | repeat | CPU affinity | 墙钟 [s] | CPU [s] | RTF | 原生窗口均值 [ms] | 回放倍率 |", "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in sfuise_timing_ok:
        lines.append(f"| {row['dataset']} | {row['sequence']} | {row.get('repeat','natural')} | {row.get('cpu_affinity','NA')} | {row['wall_s']:.2f} | {row.get('cpu_s',float('nan')):.2f} | {row.get('realtime_factor',float('nan')):.4f} | {row.get('native_average_window_runtime_ms',float('nan')):.3f} | {row.get('playback_rate',1.0):.1f} |")
    sfuise_groups = defaultdict(list)
    for row in sfuise_timing_ok:
        sfuise_groups[(row["dataset"], row["sequence"])].append(row)
    lines += ["", "SFUISE 端到端 5 次统计：", "",
              "| 数据集/序列 | wall p50/p95/p99/max [s] | CPU p50 [s] | RTF p50 | 原生窗口均值 p50 [ms] |",
              "|---|---:|---:|---:|---:|"]
    for key, group in sfuise_groups.items():
        wall = np.asarray([row["wall_s"] for row in group], float)
        lines.append(
            f"| {'/'.join(key)} | "
            f"{np.percentile(wall,50):.2f}/{np.percentile(wall,95):.2f}/"
            f"{np.percentile(wall,99):.2f}/{np.max(wall):.2f} | "
            f"{np.median([row.get('cpu_s',float('nan')) for row in group]):.2f} | "
            f"{np.median([row.get('realtime_factor',float('nan')) for row in group]):.4f} | "
            f"{np.median([row.get('native_average_window_runtime_ms',float('nan')) for row in group]):.3f} |")
    failed_timing = [row for row in sfuise_timing if row.get("status") != "SUCCESS"]
    if failed_timing:
        lines += ["", "SFUISE 稳定计时失败项：", "",
                  "| 数据集 | 序列 | repeat | 原因 |", "|---|---|---:|---|"]
        for row in failed_timing:
            reason = str(row.get("reason", "unknown")).replace("|", "/")
            lines.append(f"| {row['dataset']} | {row['sequence']} | {row.get('repeat','NA')} | {reason} |")
    equivalence_path = output/"equivalence/simulation/equivalence.json"
    equivalence = json.loads(equivalence_path.read_text()) if equivalence_path.is_file() else None
    lines += ["", "## 仿真 direct nominal / ROS fde=off 等价性", ""]
    if equivalence:
        lines += [f"状态：**{equivalence['status']}**；direct/realtime 样本数 "
                  f"{equivalence['direct_samples']}/{equivalence['realtime_samples']}；"
                  f"标签位置最大差异 {equivalence.get('max_tag_position_difference_m')} m；"
                  f"姿态最大差异 {equivalence.get('max_orientation_difference_deg')} deg；"
                  f"禁止的 GT 订阅：{equivalence.get('forbidden_truth_subscriptions', [])}。",
                  "", f"原因：{equivalence.get('reason') or '数值容差内一致'}。", ""]
    else:
        lines += ["未运行。", ""]
    lines += ["## 失败与审计口径", "",
              "所有 FAIL/NA 均保留在逐序列表中；非有限轨迹、求解失败或 coverage < 80% 不进入族级数值汇总。估计器输入 allowlist 仅含 `imu.csv`、`uwb.csv`、`anchors.csv`，`gt.csv` 由 manifest 标为 evaluator-only。", "",
              "## 复现", "", "```bash",
              "python3 tools/run_dataset_benchmark.py prepare",
              "python3 tools/run_dataset_benchmark.py calibrate",
              "python3 tools/run_dataset_benchmark.py preflight",
              "cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target nominal_dataset_runner -j2",
              "python3 tools/run_dataset_benchmark.py run --method current",
              "python3 tools/run_sfuise_benchmark.py run",
              "taskset -c 4 python3 tools/run_dataset_benchmark.py timing",
              "taskset -c 4 python3 tools/run_sfuise_benchmark.py timing",
              "python3 tools/check_sim_realtime_equivalence.py --playback-rate 2",
              "python3 tools/run_dataset_benchmark.py evaluate",
              "python3 tools/run_dataset_benchmark.py report", "```", ""]
    path = report_root/"uwb_imu_accuracy_timing_report.md"
    path.write_text("\n".join(lines))
    # Keep the machine-readable, tracked deliverables beside the report.  Raw
    # per-sample arrays and logs remain in the ignored results directory.
    shutil.copy2(output/"evaluation/metrics.csv", report_root/"sequence_metrics.csv")
    shutil.copy2(output/"evaluation/metrics.json", report_root/"sequence_metrics.json")
    summary = {
        "schema": "uwb-imu-accuracy-timing-summary/v1",
        "generated_by": "tools/run_dataset_benchmark.py report",
        "method_counts": {key: {"total": value[0], "eligible": value[1]}
                          for key, value in method_counts.items()},
        "preflight": {"status": preflight.get("status"),
                      "expected": preflight.get("expected"),
                      "observed": preflight.get("observed")},
        "family_summary": family_summary,
        "paired_current_minus_sfuise_ape_rmse_m": [
            {"dataset": dataset, "sequence": sequence, "delta_m": delta}
            for dataset, sequence, delta in paired_rows],
        "equivalence": equivalence,
        "artifacts": {"sequence_metrics_csv": "sequence_metrics.csv",
                      "sequence_metrics_json": "sequence_metrics.json",
                      "report": path.name, "figures": "figures/"},
    }
    (report_root/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    common.write_csv(report_root/"summary.csv",
                     list(family_summary[0]) if family_summary else [], family_summary)
    print(path)
    return 0


def parser():
    result = argparse.ArgumentParser()
    sub = result.add_subparsers(dest="command", required=True)
    for name in ("prepare", "calibrate", "preflight", "run", "timing", "evaluate", "report"):
        item = sub.add_parser(name)
        item.add_argument("--config")
        item.add_argument("--sequence")
        item.add_argument("--output", default=str(DEFAULT_OUTPUT))
        if name in ("run", "timing"):
            item.add_argument("--binary")
        if name == "run":
            item.add_argument("--method", choices=("current", "sfuise"), default="current")
            item.add_argument("--tag-policy", choices=("primary", "all"), default="primary")
    return result


def main():
    arguments = parser().parse_args()
    return globals()["command_" + arguments.command](arguments)


if __name__ == "__main__":
    raise SystemExit(main())
