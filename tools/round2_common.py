#!/usr/bin/env python3
"""Shared deterministic protocol machinery for Integrity V2 round two."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import pathlib
import re
import tempfile
from dataclasses import dataclass
from typing import Any, Iterable, Iterator

PASS = "PASS"
FAIL = "FAIL"
INVALID = "INVALID"
STATUSES = {PASS, FAIL, INVALID}
MASK64 = (1 << 64) - 1


class InvalidCampaign(RuntimeError):
    pass


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: pathlib.Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(handle, "wb") as stream:
            stream.write(canonical_json(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_protocol(path: pathlib.Path) -> tuple[dict[str, Any], str]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise InvalidCampaign(f"cannot load protocol: {error}") from error
    if value.get("schema_version") not in {
            "uwb-imu-pl/integrity-round2-protocol/v1",
            "uwb-imu-pl/integrity-round3-protocol/v1"}:
        raise InvalidCampaign("unsupported integrity campaign protocol schema")
    if value.get("status_vocabulary") != [PASS, FAIL, INVALID]:
        raise InvalidCampaign("protocol status vocabulary is not frozen")
    parallelism = value.get("parallelism", {})
    if parallelism.get("max_workers") != 4 or parallelism.get("numeric_threads") != 1:
        raise InvalidCampaign("protocol must freeze four workers and one numeric thread")
    if (value.get("schema_version", "").find("round3") >= 0 and
            parallelism.get("jsonl_shards") != 4):
        raise InvalidCampaign("round-three protocol must freeze four JSONL shards")
    domains = value.get("seed_domains", {})
    ranges = []
    for name in ("calibration", "development", "test", "stress"):
        item = domains.get(name, {})
        begin, count = item.get("begin"), item.get("count")
        if not isinstance(begin, int) or not isinstance(count, int) or count <= 0:
            raise InvalidCampaign(f"invalid seed domain: {name}")
        ranges.append((begin, begin + count, name))
    for left, right in zip(sorted(ranges), sorted(ranges)[1:]):
        if left[1] > right[0]:
            raise InvalidCampaign(f"overlapping seed domains: {left[2]}/{right[2]}")
    return value, sha256_file(path)


def _splitmix64(value: int) -> int:
    value = (value + 0x9E3779B97F4A7C15) & MASK64
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & MASK64
    return (value ^ (value >> 31)) & MASK64


@dataclass(frozen=True)
class CounterRng:
    """Stateless counter RNG: output depends only on domain/key/counter."""

    seed: int
    domain: str
    key: str

    def uint64(self, counter: int, lane: int = 0) -> int:
        payload = canonical_json([self.seed, self.domain, self.key])
        base = int.from_bytes(hashlib.sha256(payload).digest()[:8], "little")
        return _splitmix64(base ^ _splitmix64(counter) ^ _splitmix64(lane))

    def uniform(self, counter: int, lane: int = 0) -> float:
        return ((self.uint64(counter, lane) >> 11) + 0.5) / float(1 << 53)

    def normal(self, counter: int, lane: int = 0) -> float:
        u1 = max(self.uniform(counter, 2 * lane), 2.0 ** -53)
        u2 = self.uniform(counter, 2 * lane + 1)
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def scenario_id(gate: str, dimensions: dict[str, Any]) -> str:
    normalized = {"gate": gate.upper(), **dimensions}
    digest = sha256_bytes(canonical_json(normalized))[:16]
    readable = "-".join(str(dimensions[key]).lower().replace("_", "-")
                        for key in sorted(dimensions) if key in
                        ("geometry", "sensor", "motion", "fault_mode", "name"))
    return f"{gate.lower()}-{readable or 'cell'}-{digest}"


def seed_for(protocol: dict[str, Any], domain: str, cell_id: str,
             ordinal: int) -> int:
    item = protocol["seed_domains"][domain]
    if not 0 <= ordinal < item["count"]:
        raise InvalidCampaign(f"seed ordinal outside {domain} domain")
    # A cell-specific cyclic permutation makes every ordinal in a cell unique.
    # Using an independent hash/modulo draw for every ordinal would admit
    # birthday collisions and would therefore not provide the frozen number of
    # independent seeds promised by the protocol.
    base = CounterRng(item["begin"], domain, cell_id).uint64(0) % item["count"]
    offset = (base + ordinal) % item["count"]
    return item["begin"] + offset


def boundary_amplitude(noncentrality: float, gram: list[list[float]],
                       direction: list[float], ratio: float) -> float:
    if noncentrality <= 0 or ratio <= 0 or len(gram) != len(direction):
        raise InvalidCampaign("invalid detection-boundary input")
    energy = sum(direction[i] * sum(gram[i][j] * direction[j]
                                    for j in range(len(direction)))
                 for i in range(len(direction)))
    if not math.isfinite(energy) or energy <= 0:
        return math.inf
    return ratio * math.sqrt(noncentrality / energy)


def metric_classification(truth_sources: set[str], removed_sources: set[str],
                          plausible_sources: set[str], nominal: bool) -> str:
    if nominal:
        return "nominal_keep" if not removed_sources else "nominal_false_exclusion"
    if not truth_sources.issubset(removed_sources):
        return "wrong_exclusion"
    extras = removed_sources - truth_sources
    if extras and extras.issubset(plausible_sources):
        return "ambiguous_exclusion"
    if extras:
        return "wrong_exclusion"
    return "union_exclusion" if len(truth_sources) > 1 else "correct_exclusion"


def enumerate_cells(protocol: dict[str, Any]) -> Iterator[dict[str, Any]]:
    ratios = protocol["boundary_ratios"]
    modes = protocol["estimator_modes"]
    spec = protocol["gate_e"]
    for geometry in spec["geometries"]:
        active = spec["anchors"][:{"normal8": 8, "poor7": 7, "poor6": 6}[geometry]]
        for anchor in active:
            for fault_mode in spec["fault_modes"]:
                for ratio in ratios:
                    for estimator_mode in modes:
                        dimensions = dict(geometry=geometry, anchor=anchor,
                                          fault_mode=fault_mode, ratio=ratio,
                                          estimator_mode=estimator_mode)
                        yield {"gate": "E", "id": scenario_id("E", dimensions), **dimensions}
        for estimator_mode in modes:
            dimensions = dict(geometry=geometry, fault_mode="nominal", ratio=0.0,
                              estimator_mode=estimator_mode)
            yield {"gate": "E", "id": scenario_id("E", dimensions), **dimensions}
    for geometry in spec["negative_controls"]:
        dimensions = dict(geometry=geometry, name="rank_negative_control",
                          estimator_mode="fixed_lag_200")
        yield {"gate": "E", "id": scenario_id("E", dimensions), **dimensions}

    spec = protocol["gate_f"]
    for sensor in spec["sensors"]:
        for axis in spec["axes"]:
            for motion in spec["motions"]:
                for window in spec["window_lengths"]:
                    for ratio in ratios:
                        for estimator_mode in modes:
                            dimensions = dict(sensor=sensor, axis=axis, motion=motion,
                                              window=window, ratio=ratio,
                                              estimator_mode=estimator_mode)
                            yield {"gate": "F", "id": scenario_id("F", dimensions), **dimensions}

    spec = protocol["gate_g"]
    for motion in spec["motions"]:
        for length in spec["bridge_lengths"] + [spec["timeout_epoch"]]:
            for envelope in ("in_envelope", "out_of_envelope"):
                dimensions = dict(motion=motion, bridge_length=length,
                                  envelope=envelope, estimator_mode="fixed_lag_200")
                yield {"gate": "G", "id": scenario_id("G", dimensions), **dimensions}

    spec = protocol["gate_h"]
    for anchor in spec["anchors"]:
        for imu_axis in spec["imu_axes"]:
            for uwb_mode in spec["uwb_modes"]:
                for onset in spec["relative_onsets"]:
                    for pair in spec["magnitude_pairs"]:
                        for ambiguity in spec["ambiguity"]:
                            for geometry in spec["union_geometries"]:
                                dimensions = dict(anchor=anchor, imu_axis=imu_axis,
                                    fault_mode=uwb_mode, relative_onset=onset,
                                    uwb_ratio=pair[0], imu_ratio=pair[1],
                                    ambiguity=ambiguity, geometry=geometry,
                                    estimator_mode="fixed_lag_200")
                                yield {"gate": "H", "id": scenario_id("H", dimensions), **dimensions}

    spec = protocol["gate_i"]
    for name in spec["noisy_scenarios"]:
        for estimator_mode in modes:
            dimensions = dict(name=name, noise="noisy",
                              estimator_mode=estimator_mode)
            yield {"gate": "I", "id": scenario_id("I", dimensions), **dimensions}
    for name in spec["deterministic_sequences"]:
        dimensions = dict(name=name, noise="zero", estimator_mode=(
            "fixed_lag_200" if "fixed_lag" in name else "full_history"))
        yield {"gate": "I", "id": scenario_id("I", dimensions), **dimensions}


def inventory(directory: pathlib.Path, relative_paths: Iterable[pathlib.Path]) -> dict[str, Any]:
    entries = []
    for relative in sorted(set(relative_paths), key=str):
        path = directory / relative
        entry: dict[str, Any] = {"path": relative.as_posix(),
                                 "bytes": path.stat().st_size,
                                 "sha256": sha256_file(path)}
        if path.suffix == ".csv":
            with path.open(newline="", encoding="utf-8") as stream:
                entry["rows"] = sum(1 for _ in csv.DictReader(stream))
        entries.append(entry)
    return {"schema_version": "uwb-imu-pl/raw-inventory/v1", "artifacts": entries}


def verify_checksums(directory: pathlib.Path,
                     checksum_name: str = "checksums.sha256") -> list[str]:
    path = directory / checksum_name
    if not path.is_file():
        return [f"missing {checksum_name}"]
    errors = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        if not match:
            errors.append(f"malformed checksum row {line_number}")
            continue
        relative = pathlib.PurePosixPath(match.group(2))
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"unsafe checksum path at row {line_number}")
            continue
        target = directory / pathlib.Path(relative)
        if not target.is_file() or sha256_file(target) != match.group(1):
            errors.append(f"checksum mismatch: {relative}")
    return errors


def clopper_pearson(successes: int, trials: int,
                    confidence: float = 0.95) -> tuple[float, float]:
    if not 0 <= successes <= trials or trials <= 0:
        raise InvalidCampaign("invalid binomial observation")
    try:
        from scipy.stats import beta
    except ImportError as error:
        raise InvalidCampaign("scipy is required for exact Clopper-Pearson intervals") from error
    alpha = 1.0 - confidence
    lower = 0.0 if successes == 0 else float(beta.ppf(alpha / 2, successes,
                                                       trials - successes + 1))
    upper = 1.0 if successes == trials else float(beta.ppf(1 - alpha / 2,
                                                           successes + 1,
                                                           trials - successes))
    return lower, upper
