#!/usr/bin/env python3
"""Run and seal the preregistered v4 engineering gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET


BASELINE = "cf287b4fec9bc5689317f302ccf4cdd927bfba81"
FROZEN_BACKEND = (
    "src/nlos_refit.cpp",
    "include/uifgo/nlos_refit.h",
    "src/nlos_recoverability.cpp",
    "include/uifgo/nlos_recoverability.h",
)
FROZEN_DOCUMENTS = ("doc/v2/paper_structure.tex", "doc/v2/v2_roadmap.md")


def sha256(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str], cwd: Path, log: Path) -> dict:
    started = time.monotonic()
    with log.open("wb") as stream:
        process = subprocess.run(command, cwd=cwd, stdout=stream,
                                 stderr=subprocess.STDOUT, check=False)
    return {
        "command": command,
        "cwd": str(cwd),
        "exit_code": process.returncode,
        "wall_seconds": time.monotonic() - started,
        "log": str(log),
        "log_sha256": sha256(log),
    }


def xml_totals(root: Path) -> dict:
    totals = {"tests": 0, "errors": 0, "failures": 0, "skipped": 0,
              "files": 0}
    for path in sorted(root.rglob("*.xml")):
        node = ET.parse(path).getroot()
        totals["files"] += 1
        for name in ("tests", "errors", "failures", "skipped"):
            totals[name] += int(node.attrib.get(name, 0))
    return totals


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "engineering_gate.json"
    if summary_path.exists():
        raise SystemExit(f"refusing to overwrite {summary_path}")

    repo = Path(__file__).resolve().parents[2]
    workspace = repo.parents[1]
    head = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    evidence = {
        "schema": "uifgo_windowed_fde_engineering_gate_v1",
        "baseline": BASELINE,
        "observed_head_before_implementation": BASELINE,
        "observed_initial_git_status": ["?? doc/v2/ie_0911/"],
        "head_at_gate": head,
        "frozen_backend": {},
        "frozen_documents": {},
        "commands": [],
    }
    for name in FROZEN_BACKEND:
        path = repo / name
        diff = subprocess.run(
            ["git", "diff", "--exit-code", BASELINE, "--", name], cwd=repo,
            text=True, capture_output=True, check=False)
        evidence["frozen_backend"][name] = {
            "sha256": sha256(path), "diff_exit_code": diff.returncode,
            "diff": diff.stdout + diff.stderr,
        }
    for name in FROZEN_DOCUMENTS:
        evidence["frozen_documents"][name] = sha256(repo / name)

    commands = (
        ["catkin", "build", "uwb_imu_fgo", "--no-deps", "-j4"],
        ["catkin", "build", "uwb_imu_fgo", "--no-deps", "--make-args",
         "tests", "-j4"],
        ["catkin", "run_tests", "uwb_imu_fgo"],
        ["catkin_test_results",
         "build/uwb_imu_fgo/test_results/uwb_imu_fgo"],
    )
    for index, command in enumerate(commands, 1):
        label = Path(command[0]).name if index == 4 else command[1]
        item = run(command, workspace, output / f"{index:02d}_{label}.log")
        evidence["commands"].append(item)
        summary_path.write_text(json.dumps(evidence, indent=2) + "\n")
        if item["exit_code"] != 0:
            evidence["status"] = "FAIL_COMMAND"
            summary_path.write_text(json.dumps(evidence, indent=2) + "\n")
            return 1

    result_root = workspace / "build/uwb_imu_fgo/test_results/uwb_imu_fgo"
    evidence["test_totals"] = xml_totals(result_root)
    summary_text = Path(evidence["commands"][-1]["log"]).read_text(
        errors="replace")
    match = re.search(
        r"Summary: (\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped",
        summary_text)
    evidence["catkin_test_results_totals"] = ({
        "tests": int(match.group(1)), "errors": int(match.group(2)),
        "failures": int(match.group(3)), "skipped": int(match.group(4)),
    } if match else {})
    totals = evidence["test_totals"]
    reported = evidence["catkin_test_results_totals"]
    evidence["minimum_test_count"] = 370
    evidence["status"] = (
        "PASS" if reported.get("tests", 0) >= 370 and
        all(reported.get(name) == 0 for name in ("errors", "failures", "skipped")) and
        all(totals[name] == 0 for name in ("errors", "failures", "skipped"))
        else "FAIL_TEST_TOTALS")
    summary_path.write_text(json.dumps(evidence, indent=2) + "\n")
    return 0 if evidence["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
