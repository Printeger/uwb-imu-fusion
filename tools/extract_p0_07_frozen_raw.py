#!/usr/bin/env python3
"""Freeze only raw/reference inputs from a production-probe acquisition."""

import argparse
import hashlib
from pathlib import Path


ALLOWED = {
    "BLOCK", "OWNER", "PROTECTED_MAP", "HISTORY_RESPONSE",
    "HISTORY_DETECTOR_RESPONSE", "HISTORY_D_PERP", "HISTORY_COLUMN",
    "ACTION_INPUT", "ADDED_BLOCK",
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--acquisition", required=True)
    parser.add_argument("--raw", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    lines = []
    for line in Path(args.acquisition).read_text().splitlines():
        if line.split("\t", 1)[0] in ALLOWED:
            lines.append(line)
    if not lines or not any(line.startswith("BLOCK\t") for line in lines):
        raise SystemExit("acquisition has no raw blocks")
    metadata = [
        "P007_FROZEN_RAW_V1",
        "raw_sha256\t" + digest(args.raw),
        "truth_sha256\t" + digest(args.truth),
        "config_sha256\t" + digest(args.config),
        "manifest_sha256\t" + digest(args.manifest),
    ]
    Path(args.output).write_text("\n".join(metadata + lines) + "\n")


if __name__ == "__main__":
    main()
