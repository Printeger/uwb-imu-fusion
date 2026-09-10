#!/usr/bin/env python3
"""Atomic, estimator-free attempt preparation for T10-A19-R05."""

import json
import os
from pathlib import Path


def write_json_atomic(path: Path, payload: dict) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def prepare_attempt(evidence: Path, attempt: str) -> dict:
    attempts = evidence / "attempts"
    attempts.mkdir(exist_ok=True)
    root = attempts / attempt
    root.mkdir(exist_ok=False)
    logs = root / "logs"
    logs.mkdir()
    pilot_parent = root / "pilot"
    pilot_parent.mkdir()
    output_leaf = pilot_parent / "output"
    if output_leaf.exists():
        raise RuntimeError("FRESH_OUTPUT_LEAF_ALREADY_EXISTS")
    status = root / "launch_status.json"
    write_json_atomic(status, {
        "schema": "A19_R05_LAUNCH_STATUS_V1",
        "attempt": attempt,
        "parent_exists": pilot_parent.is_dir(),
        "parent_writable": os.access(pilot_parent, os.W_OK),
        "output_leaf": str(output_leaf),
        "output_leaf_exists": False,
        "estimator": "NOT_RUN",
    })
    return {"root": root, "logs": logs, "pilot_parent": pilot_parent,
            "output_leaf": output_leaf, "status": status}
