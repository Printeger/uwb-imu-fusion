"""Independent reader for uwb-imu-pl/frozen-candidates/v4 replay binaries.

This module re-implements the candidate replay codec
(`src/uwb_imu_pl/estimation/candidate_replay.cpp`, schema v4) from the written
field order.  It deliberately shares no code with the C++ implementation and is
used for offline census / shadow diagnostics only.

Layout notes verified against candidate_replay.cpp `transfer()`:
  * all integers little-endian; doubles IEEE binary64 little-endian
  * strings are u64 length prefixed
  * matrices are (u64 rows, u64 cols) followed by column-major doubles
  * enums (FactorKind/SensorType/RowRole/BridgeMode/...) have no explicit
    underlying type and are serialized as 4-byte ints
  * bool is a single byte
"""

import struct

import numpy as np

SENSOR = {
    0: "Unknown", 1: "Uwb", 2: "Imu", 3: "ImuAccelerometer",
    4: "ImuGyroscope", 5: "Bridge", 6: "Prior",
}
FACTOR_KIND = {
    0: "Unknown", 1: "BoundaryPrior", 2: "CombinedImu", 3: "UwbBatch",
    4: "KinematicBridge", 5: "BiasContinuity", 6: "Regularizer",
}
ROW_ROLE = {0: "Measurement", 1: "TrustedPrior", 2: "Regularizer"}
DISPOSITION = {
    0: "ExplicitMeasurement", 1: "BoundaryInput", 2: "PendingExplicit",
    3: "UnrecoverableHistory",
}
BRIDGE_MODE = {0: "None", 1: "GenericKinematic", 2: "Dynamics"}
RECOVERABILITY = {
    0: "Recoverable", 1: "Marginalized", 2: "MissingProvenance",
    3: "OnsetBeforeRecoverableBoundary",
}


class Reader:
    def __init__(self, data):
        self.data = data
        self.off = 0

    def _scalar(self, fmt, size):
        value = struct.unpack_from(fmt, self.data, self.off)[0]
        self.off += size
        return value

    def u64(self):
        return self._scalar("<Q", 8)

    def i64(self):
        return self._scalar("<q", 8)

    def i32(self):
        return self._scalar("<i", 4)

    def f64(self):
        return self._scalar("<d", 8)

    def boolean(self):
        value = bool(self.data[self.off])
        self.off += 1
        return value

    def text(self):
        size = self.u64()
        value = self.data[self.off:self.off + size].decode("utf-8")
        self.off += size
        return value

    def matrix(self):
        rows = self.u64()
        cols = self.u64()
        count = rows * cols
        arr = np.frombuffer(self.data, dtype="<f8", count=count, offset=self.off)
        arr = arr.reshape((rows, cols), order="F").copy()
        self.off += 8 * count
        return arr

    def vector(self, item):
        size = self.u64()
        return [item() for _ in range(size)]

    def u64_vector(self):
        return self.vector(self.u64)

    def i32_vector(self):
        return self.vector(self.i32)


def _version(reader):
    return {
        "graph_version": reader.u64(),
        "ordering_version": reader.u64(),
        "noise_model_version": reader.u64(),
        "linpoint_version": reader.u64(),
    }


def _block(reader):
    block = {
        "group_id": reader.u64(),
        "kind": FACTOR_KIND.get(reader.i32(), "?"),
        "sensor": SENSOR.get(reader.i32(), "?"),
        "role": ROW_ROLE.get(reader.i32(), "?"),
        "jacobian_raw": reader.matrix(),
        "residual_raw": reader.matrix(),
        "covariance": reader.matrix(),
        "whitener": reader.matrix(),
        "jacobian_whitened": reader.matrix(),
        "residual_whitened": reader.matrix(),
        "window_column_indices": reader.i32_vector(),
        "fault_units": reader.u64_vector(),
        "effective_weight": reader.f64(),
        "whitening_model_id": reader.text(),
        "version": _version(reader),
    }
    return block


def read_replay(path):
    with open(path, "rb") as handle:
        reader = Reader(handle.read())
    schema = reader.text()
    if schema not in ("uwb-imu-pl/frozen-candidates/v4",
                      "uwb-imu-pl/frozen-candidates/v5"):
        raise ValueError(f"unsupported replay schema: {schema}")
    endian = reader.u64()
    if endian != 0x0102030405060708:
        raise ValueError("unexpected endian marker")
    replay = {
        "schema": schema,
        "input_attempt_id": reader.u64(),
        "input_timestamp_ns": reader.i64(),
        "transaction_id": reader.u64(),
    }
    window = {
        "id": reader.u64(),
        "version": _version(reader),
        "H": reader.matrix(),
        "z": reader.matrix().ravel(),
        "base_information": reader.matrix(),
        "base_information_rhs": reader.matrix().ravel(),
        "protected_state_map": reader.matrix(),
        "rank": reader.i32(),
        "dof": reader.i32(),
        "condition_number": reader.f64(),
        "model_valid": reader.boolean(),
        "reason": reader.text(),
        "detector_first_epoch": reader.u64(),
        "recovery_first_epoch": reader.u64(),
        "capabilities": {
            name: reader.boolean()
            for name in (
                "includes_boundary_prior", "includes_pending_imu",
                "includes_pending_uwb", "complete_factor_provenance",
                "history_provenance_valid", "fixed_lag_maturity_valid",
                "no_duplicate_rows", "every_active_factor_accounted_once",
                "frozen_slot_identity_valid",
            )
        },
    }
    window["state_layout"] = reader.vector(lambda: {
        "epoch": reader.u64(),
        "keys": reader.u64_vector(),
        "column_offset": reader.i32(),
        "dimension": reader.i32(),
        "protected_current_state": reader.boolean(),
    })
    window["slot_accounting"] = reader.vector(
        lambda: _slot_accounting(reader))
    window["factor_inventory"] = reader.vector(lambda: {
        "group_id": reader.u64(),
        "epoch": reader.u64(),
        "kind": FACTOR_KIND.get(reader.i32(), "?"),
        "sensor": SENSOR.get(reader.i32(), "?"),
        "disposition": DISPOSITION.get(reader.i32(), "?"),
        "keys": reader.u64_vector(),
        "slots": reader.u64_vector(),
    })
    window["blocks"] = reader.vector(lambda: _block(reader))
    dictionary = reader.vector(lambda: _block(reader))
    replay["actions"] = reader.vector(lambda: _action(reader, dictionary))
    replay["config"] = {
        "rank_tolerance": reader.f64(),
        "max_condition_number": reader.f64(),
        "max_linearization_step_norm": reader.f64(),
        "materialize_dense_oracle_fields": reader.boolean(),
        "exact_slow_path_condition": reader.f64(),
        "enable_numerical_certificate": reader.boolean(),
        "force_exact_condition_number": reader.boolean(),
    }
    replay["window"] = window
    if schema == "uwb-imu-pl/frozen-candidates/v5":
        replay["identity"] = {
            "snapshot_id": reader.text(),
            "source_revision": reader.text(),
            "config_digest": reader.text(),
            "manifest_digest": reader.text(),
            "state_solution_id": reader.text(),
            "sensor_timestamp_ns": reader.i64(),
            "frame_id": reader.text(),
            "position_reference": reader.text(),
            "tangent_convention": reader.text(),
            "state_scale": reader.text(),
            "output_jacobian_contract": reader.text(),
            "whitening_id": reader.text(),
            "noise_model_id": reader.text(),
            "boundary_summary_id": reader.text(),
            "history_lineage_id": reader.text(),
            "protected_reference_center": reader.text(),
            "active_observation_index": reader.text(),
            "coverage_epoch": reader.text(),
            "validity_assumptions": reader.text(),
            "identity_digest": reader.text(),
        }
    if reader.off != len(reader.data):
        raise ValueError(
            f"trailing bytes: {len(reader.data) - reader.off} at {reader.off}")
    return replay


def _slot_accounting(reader):
    slot = reader.u64()
    has = reader.boolean()
    group = reader.u64()  # always serialized (value_or default when absent)
    entry = {
        "slot": slot,
        "group_id": group if has else None,
        "explicit_window_block": reader.boolean(),
        "boundary_input": reader.boolean(),
        "pointer_identity_valid": reader.boolean(),
    }
    return entry


def _optional(reader):
    has = reader.boolean()
    value = reader.u64()  # always serialized (value_or default when absent)
    return value if has else None


def _action(reader, dictionary):
    action = {
        "id": reader.u64(),
        "covered_units": reader.u64_vector(),
        "covered_modes": reader.u64_vector(),
        "physical_source_ids": reader.vector(reader.text),
        "groups_to_remove": reader.u64_vector(),
        "groups_to_add": reader.u64_vector(),
        "bridge_mode": BRIDGE_MODE.get(reader.i32(), "?"),
        "exclusion_cardinality": reader.i32(),
        "action_model_id": reader.text(),
        "recoverability": RECOVERABILITY.get(reader.i32(), "?"),
        "recovery_epoch_begin": _optional(reader),
        "recovery_epoch_end": _optional(reader),
        "added_block_refs": reader.u64_vector(),
    }
    action["added_blocks"] = [dictionary[i] for i in action["added_block_refs"]]
    return action
