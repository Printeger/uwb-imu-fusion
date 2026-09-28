#!/usr/bin/env python3
"""Run the process-separated P0-07 B -> A -> C acceptance protocol."""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path


def run(command):
    completed = subprocess.run(command, check=False, text=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
    sys.stdout.write(completed.stdout)
    if completed.returncode:
        raise SystemExit(completed.returncode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", required=True)
    parser.add_argument("--source", required=True)
    args = parser.parse_args()
    source = Path(args.source).resolve()
    evidence = source / "docs/evidence/p0-07-corrected-exhaustive"
    replay = evidence / "authoritative-replay"
    with tempfile.TemporaryDirectory(prefix="p007-process-oracle-") as tmp:
        actual = Path(tmp) / "actual.tsv"
        expected = Path(tmp) / "expected.tsv"
        run([args.probe, str(source / "config/fde_uwb_order1.yaml"),
             str(replay / "frozen-input.json"),
             str(replay / "frozen-truth.json"), str(actual)])
        run([sys.executable, str(source / "tools/p0_07_offline_oracle.py"),
             "--capture", str(evidence / "process-oracle/frozen-raw-factors-v1.tsv"),
             "--raw", str(replay / "frozen-input.json"),
             "--truth", str(replay / "frozen-truth.json"),
             "--config", str(source / "config/fde_uwb_order1.yaml"),
             "--manifest", str(evidence / "oracle-manifest-v1.json"),
             "--canonical-manifest", str(evidence / "oracle-manifest-v1.json"),
             "--output", str(expected)])
        run([sys.executable, str(source / "tools/p0_07_compare_protocols.py"),
             "--oracle", str(expected), "--actual", str(actual)])


if __name__ == "__main__":
    main()
