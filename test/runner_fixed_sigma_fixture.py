#!/usr/bin/env python3
"""Deterministic test-only fixed-sigma input for runner contract tests."""

import csv
import hashlib
import json
import math
import pathlib
import re


RECORDING_ID = "content-fnv1a64:gate05r-fixed-sigma-runner"
SOURCE_SHA256 = "sha256:" + hashlib.sha256(
    b"gate05r fixed-sigma runner fixture v1").hexdigest()
SIGMA_RANGE_M = 0.1
SEGMENT_START_S = 1.0
SEGMENT_END_S = 4.0
ANCHORS = (
    (1, -5.0, -5.0, 1.0),
    (2, -5.0, -5.0, 5.0),
    (3, -5.0, 5.0, 1.0),
    (4, -5.0, 5.0, 5.0),
    (5, 5.0, -5.0, 1.0),
    (6, 5.0, -5.0, 5.0),
    (7, 5.0, 5.0, 1.0),
    (8, 5.0, 5.0, 5.0),
)


def _sha256(path: pathlib.Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def _fnv1a64(text: str) -> int:
    value = 1469598103934665603
    for byte in text.encode("utf-8"):
        value ^= byte
        value = (value * 1099511628211) & 0xffffffffffffffff
    return value


def observation_id(message_index: int, range_index: int) -> int:
    identity = (
        f"{RECORDING_ID}|source_message={message_index}"
        f"|source_range={range_index}"
    )
    return _fnv1a64(identity)


def _write_csv(path: pathlib.Path, fieldnames, rows) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames,
                                lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def create_cache(root: pathlib.Path) -> pathlib.Path:
    """Create a compact analytic cache without GT/oracle payloads."""
    cache = root / "fixed_sigma_cache"
    cache.mkdir()
    imu_path = cache / "imu.csv"
    uwb_path = cache / "uwb_observations.csv"

    imu_rows = []
    for source_index in range(401):
        imu_rows.append({
            "source_index": source_index,
            "sensor_time_s": f"{source_index / 100.0:.2f}",
            "acc_x_mps2": "0", "acc_y_mps2": "0",
            "acc_z_mps2": "9.81",
            "gyro_x_radps": "0", "gyro_y_radps": "0",
            "gyro_z_radps": "0", "has_orientation": "1",
            "qw": "1", "qx": "0", "qy": "0", "qz": "0",
        })
    _write_csv(imu_path, (
        "source_index", "sensor_time_s", "acc_x_mps2", "acc_y_mps2",
        "acc_z_mps2", "gyro_x_radps", "gyro_y_radps",
        "gyro_z_radps", "has_orientation", "qw", "qx", "qy", "qz",
    ), imu_rows)

    uwb_rows = []
    source_observation = 0
    tag_position = (0.0, 0.0, 3.0)
    for message_index in range(5):
        for range_index, (anchor_id, x, y, z) in enumerate(ANCHORS):
            distance = math.sqrt(
                (x - tag_position[0]) ** 2 +
                (y - tag_position[1]) ** 2 +
                (z - tag_position[2]) ** 2)
            if anchor_id == 1 and message_index >= 1:
                distance += 0.4
            # Vary non-estimation payloads by source message so the production
            # stale-repeat integrity rule keeps all analytic measurements.
            fp_rssi = -70.0 - 0.01 * message_index
            uwb_rows.append({
                "obs_id": observation_id(message_index, range_index),
                "source_message_index": message_index,
                "source_range_index": range_index,
                "source_observation_index": source_observation,
                "sensor_time_s": str(float(message_index)),
                "tag_id": 1, "anchor_id": anchor_id,
                "observed_range_m": format(distance, ".17g"),
                "fp_rssi_dbm": format(fp_rssi, ".17g"),
                "rx_rssi_dbm": format(fp_rssi + 3.0, ".17g"),
                "source_valid": "1", "source_validity_reason_hex": "",
            })
            source_observation += 1
    _write_csv(uwb_path, (
        "obs_id", "source_message_index", "source_range_index",
        "source_observation_index", "sensor_time_s", "tag_id",
        "anchor_id", "observed_range_m", "fp_rssi_dbm", "rx_rssi_dbm",
        "source_valid", "source_validity_reason_hex",
    ), uwb_rows)

    imu_sha = _sha256(imu_path)
    uwb_sha = _sha256(uwb_path)
    canonical = (
        "t07_estimator_cache_v1\n"
        f"{RECORDING_ID}\n{SOURCE_SHA256}\n0\n"
        "imu_units=m/s^2,rad/s,quaternion_wxyz\n"
        "uwb_units=s,m,dBm\n"
        "time_basis=sensor_time_from_recording_origin\n"
        "grouping=source_message_index\n"
        f"{imu_sha}\n{uwb_sha}\n401\n40\n5\n"
    )
    manifest = {
        "schema": "t07_estimator_cache_v1",
        "cache_id": "sha256:" + hashlib.sha256(
            canonical.encode("utf-8")).hexdigest(),
        "base_recording_id": RECORDING_ID,
        "base_source_sha256": SOURCE_SHA256,
        "time_basis": "sensor_time_from_recording_origin",
        "recording_time_origin_s": 0.0,
        "imu_units": "acc_mps2,gyro_radps,orientation_quaternion_wxyz",
        "uwb_units": "time_s,range_m,rssi_dbm",
        "uwb_message_grouping": "source_message_index",
        "imu_file": "imu.csv", "imu_sha256": imu_sha, "imu_count": 401,
        "uwb_file": "uwb_observations.csv", "uwb_sha256": uwb_sha,
        "uwb_observation_count": 40, "uwb_message_count": 5,
    }
    manifest_path = cache / "input_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n",
                             encoding="utf-8")
    return manifest_path


def write_support(path: pathlib.Path, segment_id: str = "gate05r_segment") -> None:
    block = "\n".join("      " + line for line in segment_id.split("\n"))
    path.write_text(
        "schema: t04_oracle_support_v1\n"
        "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
        "segments:\n"
        "  - segment_id: |-\n"
        f"{block}\n"
        "    link: \"1:1\"\n"
        f"    start_time: {SEGMENT_START_S}\n"
        f"    end_time: {SEGMENT_END_S}\n",
        encoding="utf-8")


def make_config(source: pathlib.Path, destination: pathlib.Path,
                manifest: pathlib.Path,
                support: pathlib.Path = None) -> str:
    """Retain runner mode/thresholds while swapping only test input plumbing."""
    text = source.read_text(encoding="utf-8")
    text, dataset_count = re.subn(
        r"(?ms)^dataset:\n.*?(?=^anchors:)",
        "dataset:\n"
        "  interface: t07_cache\n"
        f"  cache_manifest: {manifest.resolve()}\n"
        "  cache_start_s: 0.0\n"
        "  cache_duration_s: -1.0\n\n",
        text)
    text, bag_count = re.subn(r"(?ms)^bag:\n.*?(?=^nlos:)", "", text)
    text, step_count = re.subn(
        r"(?m)^(  step:) 10$", r"\1 1", text)
    support_count = 0
    if support is not None:
        text, support_count = re.subn(
            r"(?m)^  oracle_support:.*$",
            f"  oracle_support: {support.resolve()}", text)
    expected_support = 1 if support is not None else 0
    if (dataset_count, bag_count, step_count, support_count) != \
            (1, 1, 1, expected_support):
        raise AssertionError("fixture config rewrite did not match exactly")
    destination.write_text(text, encoding="utf-8")
    return text


def assert_fixed_sigma_ledger(run_dir: pathlib.Path, expected_count=None) -> None:
    with (run_dir / "observations.csv").open(
            encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if expected_count is not None and len(rows) != expected_count:
        raise AssertionError(f"unexpected fixture observation count: {len(rows)}")
    if len({row["obs_id"] for row in rows}) != len(rows):
        raise AssertionError("fixture obs_id values are not unique")
    planned = [row for row in rows if row["planned"] == "1"]
    if not planned:
        raise AssertionError("runner produced no planned fixed-sigma observation")
    if any(float(row["nominal_sigma_m"]) != SIGMA_RANGE_M for row in planned):
        raise AssertionError("runner changed fixed configured sigma_range")
    if {row["strategy_used"] for row in planned} != {"1"}:
        raise AssertionError("runner changed fixed-sigma strategy")


def assert_fail_closed(run_dir: pathlib.Path) -> dict:
    status = json.loads((run_dir / "run_status.json").read_text("utf-8"))
    if status.get("status") != "FAILED":
        raise AssertionError(f"historical fixture did not fail closed: {status}")
    if status.get("valid_estimate_exported") not in (None, False):
        raise AssertionError(f"historical failure claimed a valid estimate: {status}")
    forbidden = {
        "trajectory.tum", "solver_certificate.json", "final_inference_summary.json"
    }
    present = {name for name in forbidden if (run_dir / name).exists()}
    if present:
        raise AssertionError(f"historical failure exported success artifacts: {present}")
    assert_fixed_sigma_ledger(run_dir)
    manifest = json.loads(
        (run_dir / "input_manifest.json").read_text(encoding="utf-8"))
    expected_manifest = {
        "raw_observations": 627,
        "planned_observations": 64,
        "keyframes": 8,
        "input_plan_hash_sha256":
            "sha256:cab614199660471fb1c01336f9fe9a3a1fa043ffcdd65949d0a4c80458795f67",
    }
    if any(manifest.get(key) != value
           for key, value in expected_manifest.items()):
        raise AssertionError(
            f"historical observation/state identity changed: {manifest}")
    preparation = json.loads(
        (run_dir / "common_preparation.json").read_text(encoding="utf-8"))
    physical_graph_contract = {
        "schema": "uifgo_t09_common_preparation_v2",
        "physical_graph_identity_policy":
            "PHYSICAL_GRAPH_AT_PRE_COMMON_REFERENCE_VALUES_V1",
        "physical_graph_linearization_sha256":
            "t08graphlin-sha256:"
            "a029fb09830efd4d17d24900cc594e25bdef7c317b5dea78602af2ccb85709e7",
        "physical_graph_reference_values_sha256":
            "t08values-sha256:"
            "51a986505c24566054d530e575f200abfa24bb785aa1d21edee712c5c6bc46bd",
        "raw_observation_ids_sha256":
            "sha256:f33599b958b25e81cabd1cd57536cd02c1ac01022aec67122b47edacfb61130d",
        "selected_observation_ids_sha256":
            "sha256:16bc33eb5af139d3a7b91ac08fc37f5b5e018df3824bb538d0545af17a93d7cb",
        "state_timeline_sha256":
            "sha256:58371f82a10240c8c10ff3f4974c6702ea68da65a2e5c2aa215c8342fb2d296e",
        "factor_count": 74,
        "uwb_factor_count": 64,
        "pim_factor_count": 7,
        "factor_type_sequence_sha256":
            "sha256:1a0c7033e7e0c2467612e6a565f7b3d45ee0befa9b74a563805e3565dabb8378",
        "factor_key_sequence_sha256":
            "sha256:c954453c8e090a25828d9e931e6ba99068707a21abf0ca8d477f94a37bea9be8",
        "physical_topology_sha256":
            "sha256:36f26aff489a4a09aefbcdf8b617cabf6dd87bb1c894437698f57992a72cea2a",
        "uwb_semantics_sha256":
            "sha256:f4e373840d075c04611ef55540d012a6dfcc88fd634f074db3398eb0d16532a6",
        "anchor_coordinates_sha256":
            "sha256:7c1b58312ea72d10580b00ad735885c74f0dac2f2ecdac0fb029de68dc6848dd",
        "lever_arm_sha256":
            "sha256:1751173c6b6189f0503d46be5ebdd1e388d97d8ee2c76b585c457af3f1aa6825",
        "pim_intervals_sha256":
            "sha256:047a70b8a27ca08d7fcbed9a01e33ef29485d194ca5d8424bc972aff74d96232",
    }
    physical_mismatches = {
        key: (expected, preparation.get(key))
        for key, expected in physical_graph_contract.items()
        if preparation.get(key) != expected
    }
    if physical_mismatches:
        raise AssertionError(
            f"historical physical graph changed: {physical_mismatches}")
    if preparation.get("initialization_linearization_identity_policy") != (
            "COMMON_INITIALIZER_OUTPUT_LINEARIZATION_V1"):
        raise AssertionError(f"initialization identity policy changed: {preparation}")
    if preparation.get("graph_linearization_sha256") != (
            "t08graphlin-sha256:"
            "cd46d08fca3a104e35fae843fbca68288dfa2e07e07039c6e6f071fa28d6cced"):
        raise AssertionError(
            f"current initializer linearization changed: {preparation}")
    if preparation.get("values_sha256") != (
            "t08values-sha256:"
            "1a6cb7d84db69d55b7533fe0bb1c087142981730f09fbacb51cbfa6efa5b8b76"):
        raise AssertionError(f"current initializer Values changed: {preparation}")
    return status
