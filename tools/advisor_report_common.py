#!/usr/bin/env python3
"""Shared helpers for the advisor-report pipeline (never mutates Week-4)."""

from __future__ import annotations

import hashlib
import json
import os
import pathlib
import platform
import socket
import subprocess
import sys
from datetime import datetime, timezone
from typing import Any

import yaml

SCHEMA = "uwb-imu-pl/advisor-report-protocol/v1"


def load_protocol(path: pathlib.Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != SCHEMA:
        raise ValueError(f"unsupported advisor-report protocol: {path}")
    return value


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: pathlib.Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def git_metadata(repository: pathlib.Path) -> tuple[str, bool]:
    sha = subprocess.check_output(
        ["git", "-C", str(repository), "rev-parse", "HEAD"], text=True).strip()
    dirty = bool(subprocess.check_output(
        ["git", "-C", str(repository), "status", "--porcelain"], text=True).strip())
    return sha, dirty


def environment() -> dict[str, Any]:
    return {
        "created_utc": datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "cpu_affinity": sorted(os.sched_getaffinity(0))
            if hasattr(os, "sched_getaffinity") else [],
        "thread_environment": {key: os.environ.get(key) for key in
            ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
             "NUMEXPR_NUM_THREADS")},
    }


def artifact_root(repository: pathlib.Path, protocol_path: pathlib.Path) -> pathlib.Path:
    protocol = load_protocol(protocol_path)
    git_sha, _ = git_metadata(repository)
    digest = sha256_file(protocol_path)
    rendered = protocol["artifact_template"].format(
        git12=git_sha[:12], protocol_hash12=digest[:12])
    return repository / rendered


def write_checksums(root: pathlib.Path) -> pathlib.Path:
    target = root / "checksums.sha256"
    entries = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and path != target and not path.name.endswith(".tmp"):
            entries.append(f"{sha256_file(path)}  {path.relative_to(root)}")
    target.write_text("\n".join(entries) + "\n", encoding="utf-8")
    return target


def verify_checksums(root: pathlib.Path) -> tuple[bool, list[str]]:
    errors: list[str] = []
    target = root / "checksums.sha256"
    if not target.is_file():
        return False, ["checksums.sha256 missing"]
    for line in target.read_text(encoding="utf-8").splitlines():
        expected, separator, relative = line.partition("  ")
        path = root / relative
        if not separator or not path.is_file():
            errors.append(f"missing or malformed: {relative}")
        elif sha256_file(path) != expected:
            errors.append(f"checksum mismatch: {relative}")
    return not errors, errors
