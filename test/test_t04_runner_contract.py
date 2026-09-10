#!/usr/bin/env python3
"""Runner-level regression for T04 CSV and JSON serialization contracts."""

import argparse
import csv
import json
import pathlib
import re
import subprocess
import tempfile


DEBUG_LABEL = "T04_ORACLE_SUPPORT_DEBUG_ONLY"
ESTIMATE_FILES = {
    "trajectory.tum",
    "segments.csv",
    "residuals.csv",
    "factor_metadata.csv",
    "imu_bias.csv",
}


def make_config(source: pathlib.Path, destination: pathlib.Path,
                support: pathlib.Path, bag: pathlib.Path) -> None:
    text = source.read_text(encoding="utf-8")
    text, support_count = re.subn(
        r"(?m)^  oracle_support:.*$",
        f"  oracle_support: {support}",
        text,
    )
    text, bag_count = re.subn(
        r"(?m)^  path:.*sim_circle_2026-06-15-16-04-35\.bag$",
        f"  path: {bag}",
        text,
    )
    if support_count != 1 or bag_count != 1:
        raise AssertionError("source config rewrite did not match exactly once")
    destination.write_text(text, encoding="utf-8")


def run(runner: pathlib.Path, config: pathlib.Path, output: pathlib.Path,
        run_id: str):
    return subprocess.run(
        [str(runner), "--config", str(config), "--output-root", str(output),
         "--run-id", run_id],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def read_csv(path: pathlib.Path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if reader.fieldnames is None:
            raise AssertionError(f"missing CSV header: {path}")
        for row in rows:
            if None in row or len(row) != len(reader.fieldnames):
                raise AssertionError(f"malformed CSV row: {path}: {row}")
        return reader.fieldnames, rows


def check_csv_round_trip(runner: pathlib.Path, source_config: pathlib.Path,
                         bag: pathlib.Path, root: pathlib.Path) -> None:
    expected_id = 'segment,with"quote\nand newline'
    support = root / "csv_support.yaml"
    support.write_text(
        "schema: t04_oracle_support_v1\n"
        f"{DEBUG_LABEL}: true\n"
        "segments:\n"
        "  - segment_id: |-\n"
        "      segment,with\"quote\n"
        "      and newline\n"
        "    link: \"1:1\"\n"
        "    start_time: 1781510684.4\n"
        "    end_time: 1781510688.4\n",
        encoding="utf-8",
    )
    config = root / "csv_config.yaml"
    make_config(source_config, config, support, bag)
    completed = run(runner, config, root, "csv_round_trip")
    if completed.returncode != 0:
        raise AssertionError(
            f"CSV runner failed ({completed.returncode}):\n{completed.stderr}"
        )
    run_dir = root / "csv_round_trip"
    _, segments = read_csv(run_dir / "segments.csv")
    _, factors = read_csv(run_dir / "factor_metadata.csv")
    _, residuals = read_csv(run_dir / "residuals.csv")
    if len(segments) != 1 or segments[0]["segment_id"] != expected_id:
        raise AssertionError(f"segment ID did not round-trip: {segments}")
    factor_rows = [row for row in factors if row["segment_id"]]
    residual_rows = [row for row in residuals if row["segment_id"]]
    if {row["segment_id"] for row in factor_rows} != {expected_id}:
        raise AssertionError("factor metadata segment linkage is inconsistent")
    if {row["segment_id"] for row in residual_rows} != {expected_id}:
        raise AssertionError("residual segment linkage is inconsistent")
    factor_obs = {row["obs_id"] for row in factor_rows}
    residual_obs = {row["obs_id"] for row in residual_rows}
    expected_count = int(segments[0]["obs_count"])
    if factor_obs != residual_obs or len(factor_obs) != expected_count:
        raise AssertionError(
            "segment/factor/residual obs_id association is inconsistent"
        )


def check_invalid_manifest_json(runner: pathlib.Path,
                                source_config: pathlib.Path,
                                bag: pathlib.Path,
                                root: pathlib.Path) -> None:
    support = root / "invalid_support.yaml"
    support.write_text(
        "schema: t04_oracle_support_v1\n"
        f"{DEBUG_LABEL}: true\n"
        "segments:\n"
        "  - segment_id: s0\n"
        "    link: \"1:1\"\n"
        "    start_time: 1781510684.4\n"
        "    end_time: 1781510688.4\n"
        "? |-\n"
        "  unknown\n"
        "  field\n"
        ": true\n",
        encoding="utf-8",
    )
    config = root / "invalid_config.yaml"
    make_config(source_config, config, support, bag)
    completed = run(runner, config, root, "invalid_manifest")
    if completed.returncode == 0:
        raise AssertionError("invalid manifest unexpectedly succeeded")
    run_dir = root / "invalid_manifest"
    status = json.loads((run_dir / "run_status.json").read_text("utf-8"))
    expected_reason = (
        "oracle manifest root contains forbidden/unknown field: unknown\nfield"
    )
    if status.get("reason") != expected_reason:
        raise AssertionError(f"error reason did not round-trip: {status!r}")
    if status.get("debug_label") != DEBUG_LABEL:
        raise AssertionError(f"debug label missing: {status!r}")
    if status.get("solver_status") != "INVALID_ORACLE_MANIFEST":
        raise AssertionError(f"wrong failure status: {status!r}")
    if status.get("valid_estimate_exported") is not False:
        raise AssertionError(f"invalid export status: {status!r}")
    present = ESTIMATE_FILES.intersection(path.name for path in run_dir.iterdir())
    if present:
        raise AssertionError(f"invalid manifest exported estimates: {present}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=pathlib.Path, required=True)
    parser.add_argument("--source-config", type=pathlib.Path, required=True)
    parser.add_argument("--bag", type=pathlib.Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="uifgo-t04-runner-test-") as temp:
        root = pathlib.Path(temp)
        check_csv_round_trip(args.runner.resolve(), args.source_config.resolve(),
                             args.bag.resolve(), root)
        check_invalid_manifest_json(
            args.runner.resolve(), args.source_config.resolve(),
            args.bag.resolve(), root
        )
    print("T04 runner CSV/JSON contract checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
