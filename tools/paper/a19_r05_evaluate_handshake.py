#!/usr/bin/env python3
"""Independent consumer for the R05 engineering handshake artifacts."""

import argparse
import csv
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(handle, "w") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()

    handshake = json.loads((root / "handshake.json").read_text())
    summary = json.loads((root / "final/final_inference_summary.json").read_text())
    truth = {
        float(row["timestamp"]): tuple(float(row[name]) for name in ("x", "y", "z"))
        for row in csv.DictReader((root / "evaluation_only_truth.csv").open())
    }
    squared = []
    with (root / "final/trajectory.tum").open() as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 4:
                continue
            timestamp = float(fields[0])
            if timestamp not in truth:
                continue
            estimate = tuple(float(value) for value in fields[1:4])
            squared.append(sum((a - b) ** 2 for a, b in zip(estimate, truth[timestamp])))
    parent_ok = (
        handshake["development_schema"]
        == "A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY"
        and handshake["provider"] == "development_nonconsumable"
        and handshake["partition_hash"] == handshake["final_parent_partition_hash"]
        and handshake["solver_identity"] == handshake["final_solver_identity"]
    )
    result = {
        "schema": "T10_A19_R05_DIRECT_HANDSHAKE_EVALUATION_V1",
        "status": "PASS" if parent_ok and squared and summary.get("valid_estimate") else "FAIL",
        "parent_relations_ok": parent_ok,
        "matches": len(squared),
        "translation_rmse_m": math.sqrt(sum(squared) / len(squared)) if squared else None,
        "inputs": {
            str(path.relative_to(root)): digest(path)
            for path in (
                root / "handshake.json",
                root / "evaluation_only_truth.csv",
                root / "final/trajectory.tum",
                root / "final/final_inference_summary.json",
            )
        },
        "consumable": False,
    }
    atomic_json(args.output.resolve(), result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
