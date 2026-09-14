#!/usr/bin/env python3
"""Serialization-only repair for the sealed own_vicon evaluator artifacts."""
import argparse
from pathlib import Path

import evaluate_own_vicon_flow as target


def union_save_csv(path, rows, fields=None):
    if fields is None:
        fields = list(dict.fromkeys(key for row in rows for key in row))
    Path(path).write_bytes(target.r.csv_bytes(rows, fields))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    target.e.save_csv = union_save_csv
    return target.evaluate(args.run.resolve())


if __name__ == '__main__':
    raise SystemExit(main())
