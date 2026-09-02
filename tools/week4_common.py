#!/usr/bin/env python3
"""Shared, deterministic Week-4 validation primitives.

The module intentionally keeps gate states ternary: malformed/incomplete
artifacts are INVALID, while complete experiments that miss a scientific
criterion are FAIL.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import pathlib
import platform
import random
import shlex
import socket
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable, Iterator, Mapping, Sequence

import yaml

PASS = "PASS"
FAIL = "FAIL"
INVALID = "INVALID"
VALID_STATUSES = frozenset((PASS, FAIL, INVALID))


class InvalidArtifact(ValueError):
    """Raised when an experiment is incomplete or structurally invalid."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"),
                       ensure_ascii=False) + "\n").encode("utf-8")


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fnv1a64(payload: bytes) -> str:
    value = 1469598103934665603
    for byte in payload:
        value ^= byte
        value = (value * 1099511628211) & ((1 << 64) - 1)
    return f"{value:016x}"


def load_yaml(path: pathlib.Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise InvalidArtifact(f"cannot load YAML {path}: {error}") from error
    if not isinstance(value, dict):
        raise InvalidArtifact(f"{path} must contain a YAML mapping")
    return value


def load_protocol(path: pathlib.Path) -> dict[str, Any]:
    protocol = load_yaml(path)
    if protocol.get("schema_version") != "uwb-imu-pl/week4-protocol/v1":
        raise InvalidArtifact("unsupported Week-4 protocol schema")
    required = protocol.get("acceptance", {}).get("required_gates")
    if not isinstance(required, list) or not required or len(set(required)) != len(required):
        raise InvalidArtifact("acceptance.required_gates must be a unique nonempty list")
    for gate in required:
        if not isinstance(gate, str) or not protocol.get(gate, {}).get("required"):
            raise InvalidArtifact(f"required gate {gate!r} is not defined as required")
    return protocol


def protocol_hash(path: pathlib.Path) -> str:
    # Hash the committed bytes, not a parser-dependent YAML representation.
    return sha256_file(path)


def _set_dotted(root: dict[str, Any], dotted: str, value: Any) -> None:
    keys = dotted.split(".")
    node: dict[str, Any] = root
    for key in keys[:-1]:
        child = node.get(key)
        if not isinstance(child, dict):
            raise InvalidArtifact(f"override parent {'.'.join(keys[:-1])} is not a map")
        node = child
    if keys[-1] not in node:
        raise InvalidArtifact(f"override references unknown key {dotted}")
    node[keys[-1]] = value


def resolve_config(base_path: pathlib.Path, overrides: Mapping[str, Any],
                   output_path: pathlib.Path) -> tuple[str, str]:
    """Apply known dotted overrides and emit/hash the actual run config."""
    allowed = {
        "seed", "incremental.fixed_lag_epochs",
        "output.write_global_diagnostics", "output.write_residuals",
        "output.write_timing", "output.root",
    }
    unknown = set(overrides) - allowed
    if unknown:
        raise InvalidArtifact(f"unsupported config overrides: {sorted(unknown)}")
    config = load_yaml(base_path)
    for key, value in overrides.items():
        _set_dotted(config, key, value)
    lag = config.get("incremental", {}).get("fixed_lag_epochs")
    if isinstance(lag, bool) or not isinstance(lag, int) or lag == 1 or lag < 0:
        raise InvalidArtifact("incremental.fixed_lag_epochs must be 0 or >=2")
    if lag > 0xFFFFFFFF:
        raise InvalidArtifact("incremental.fixed_lag_epochs must fit uint32")
    root = config.get("output", {}).get("root")
    if not isinstance(root, str) or not root:
        raise InvalidArtifact("output.root must be a nonempty string")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # PyYAML and yaml-cpp format differently. The hash contract is over these
    # exact resolved bytes and validate_run_schema uses the same FNV algorithm.
    rendered = yaml.safe_dump(config, sort_keys=False, default_flow_style=False)
    output_path.write_text(rendered, encoding="utf-8")
    return rendered, fnv1a64(rendered.encode("utf-8"))


def git_metadata(repository: pathlib.Path) -> tuple[str, bool]:
    def run(*args: str) -> str:
        return subprocess.check_output(
            ["git", "-C", str(repository), *args], text=True).strip()
    return run("rev-parse", "HEAD"), bool(run("status", "--porcelain"))


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
        "+00:00", "Z")


def environment_record() -> dict[str, Any]:
    cpu = "unknown"
    try:
        for line in pathlib.Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    return {
        "hostname": socket.gethostname(),
        "os": platform.platform(),
        "python": sys.version.split()[0],
        "cpu": cpu,
        "cpu_affinity": sorted(os.sched_getaffinity(0))
            if hasattr(os, "sched_getaffinity") else [],
        "environment_threads": {
            key: os.environ.get(key) for key in (
                "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS")
        },
    }


def atomic_json(path: pathlib.Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(canonical_json(value))
    temporary.replace(path)


def beta_fraction(a: float, b: float, x: float) -> float:
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    d = 1e-300 if abs(d) < 1e-300 else d
    d = 1.0 / d
    h = d
    for m in range(1, 301):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        d = 1e-300 if abs(d) < 1e-300 else d
        c = 1.0 + aa / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        d = 1e-300 if abs(d) < 1e-300 else d
        c = 1.0 + aa / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 3e-14:
            break
    return h


def beta_cdf(x: float, a: float, b: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    front = math.exp(math.lgamma(a+b) - math.lgamma(a) - math.lgamma(b)
                     + a*math.log(x) + b*math.log1p(-x))
    if x < (a+1)/(a+b+2):
        return front * beta_fraction(a, b, x) / a
    return 1.0 - front * beta_fraction(b, a, 1-x) / b


def beta_quantile(probability: float, a: float, b: float) -> float:
    low, high = 0.0, 1.0
    for _ in range(100):
        middle = (low + high) / 2
        if beta_cdf(middle, a, b) < probability:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def clopper_pearson(successes: int, trials: int, alpha: float = .05,
                    one_sided: bool = False) -> tuple[float, float]:
    if trials <= 0 or not 0 <= successes <= trials:
        raise InvalidArtifact("invalid binomial counts")
    tail = alpha if one_sided else alpha / 2
    low = 0.0 if successes == 0 else beta_quantile(
        tail, successes, trials-successes+1)
    high = 1.0 if successes == trials else beta_quantile(
        1-tail, successes+1, trials-successes)
    return low, high


def exact_binomial_probability_ordering(successes: int, trials: int,
                                        probability: float) -> float:
    if trials < 0 or not 0 <= successes <= trials or not 0 < probability < 1:
        raise InvalidArtifact("invalid exact-binomial arguments")
    log_observed = (math.lgamma(trials+1) - math.lgamma(successes+1)
                    - math.lgamma(trials-successes+1)
                    + successes*math.log(probability)
                    + (trials-successes)*math.log1p(-probability))
    total = 0.0
    for value in range(trials + 1):
        log_mass = (math.lgamma(trials+1) - math.lgamma(value+1)
                    - math.lgamma(trials-value+1)
                    + value*math.log(probability)
                    + (trials-value)*math.log1p(-probability))
        if log_mass <= log_observed + 1e-12:
            total += math.exp(log_mass)
    return min(1.0, total)


def holm(p_values: Sequence[float], alpha: float) -> list[dict[str, Any]]:
    if not p_values or not 0 < alpha < 1:
        raise InvalidArtifact("Holm requires p-values and alpha in (0,1)")
    if any(not math.isfinite(p) or p < 0 or p > 1 for p in p_values):
        raise InvalidArtifact("Holm p-values must be finite in [0,1]")
    order = sorted(range(len(p_values)), key=lambda index: (p_values[index], index))
    rejected = True
    result: list[dict[str, Any] | None] = [None] * len(p_values)
    for rank, index in enumerate(order, start=1):
        threshold = alpha / (len(p_values) - rank + 1)
        reject = rejected and p_values[index] <= threshold
        if not reject:
            rejected = False
        result[index] = {"p_value": p_values[index], "rank": rank,
                         "threshold": threshold, "rejected": reject}
    return [item for item in result if item is not None]


def percentile(values: Sequence[float], probability: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = probability * (len(finite)-1)
    left = int(position)
    right = min(left+1, len(finite)-1)
    return finite[left] + (position-left) * (finite[right]-finite[left])


def condition_tolerance(condition: float) -> float:
    if not math.isfinite(condition) or condition < 1.0:
        raise InvalidArtifact("condition number must be finite and >=1")
    return min(1e-3, max(1e-9, 100.0*sys.float_info.epsilon*condition))


def rolling_sum(values: Sequence[float], window: int) -> list[float]:
    if window <= 0:
        raise InvalidArtifact("rolling window must be positive")
    output: list[float] = []
    total = 0.0
    invalid = 0
    for index, value in enumerate(values):
        if math.isfinite(value):
            total += value
        else:
            invalid += 1
        if index >= window:
            previous = values[index-window]
            if math.isfinite(previous):
                total -= previous
            else:
                invalid -= 1
        output.append(total if index+1 >= window and invalid == 0 else math.nan)
    return output


def sequence_rolling(rows: Iterable[Mapping[str, Any]], field: str, window: int,
                     block_fields: Sequence[str] = ("seed", "trajectory"),
                     epoch_field: str = "epoch") -> Iterator[tuple[Mapping[str, Any], float]]:
    """Roll independently per sequence and reject duplicate/out-of-order epochs."""
    current: tuple[str, ...] | None = None
    values: list[float] = []
    previous_epoch: int | None = None
    for row in rows:
        block = tuple(str(row[key]) for key in block_fields)
        epoch = int(row[epoch_field])
        if block != current:
            current, values, previous_epoch = block, [], None
        if previous_epoch is not None and epoch != previous_epoch + 1:
            raise InvalidArtifact(f"non-contiguous epochs in sequence {block}")
        previous_epoch = epoch
        values.append(float(row[field]))
        if len(values) > window:
            values.pop(0)
        value = sum(values) if len(values) == window and all(
            math.isfinite(item) for item in values) else math.nan
        yield row, value


def write_checksums(root: pathlib.Path) -> pathlib.Path:
    target = root / "checksums.sha256"
    entries = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path != target and ".tmp" not in path.parts:
            entries.append(f"{sha256_file(path)}  {path.relative_to(root)}")
    target.write_text("\n".join(entries) + ("\n" if entries else ""),
                      encoding="utf-8")
    return target


def verify_checksums(root: pathlib.Path) -> tuple[bool, list[str]]:
    checksum_path = root / "checksums.sha256"
    errors: list[str] = []
    if not checksum_path.is_file():
        return False, ["checksums.sha256 missing"]
    for line_number, line in enumerate(checksum_path.read_text().splitlines(), 1):
        if "  " not in line:
            errors.append(f"line {line_number}: malformed")
            continue
        expected, relative = line.split("  ", 1)
        path = root / relative
        if not path.is_file():
            errors.append(f"missing: {relative}")
        elif sha256_file(path) != expected:
            errors.append(f"mismatch: {relative}")
    return not errors, errors


def shell_join(arguments: Sequence[str]) -> str:
    return shlex.join(str(value) for value in arguments)
