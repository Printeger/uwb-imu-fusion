#!/usr/bin/env python3
"""A4-13 validation dispatcher (P2).

Reads validation-manifest.json, executes the commands of every MAPPED item
(or the subset requested with --only), and writes validation-report.json with
one row per item:

  base_sha, run_sha, config_digest, case_id, command, exit_code, status,
  metrics, artifact_paths

Statuses: PASS / FAIL / NOT_RUN / SKIP_WITH_REASON.

  * MAPPED items run their command list; a non-zero exit code is FAIL, a
    missing binary is NOT_RUN (environment), and a gtest filter that matches
    zero tests is SKIP_WITH_REASON.
  * Items that are not MAPPED never execute anything: they are reported as
    NOT_RUN with their owning work package, which is the honest state for the
    B/C/D packages of the roadmap.

This tool never modifies sources; it only reads the workspace and writes its
own report.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ROOT = os.path.join(REPO, "doc", "evidence", "integrity-kernel-refactor")
WORKSPACE = os.environ.get("UWB_IMU_PL_WORKSPACE", "/home/mint/ws_fusion_uwb")


def resolve(template, binary_dir):
    return (template
            .replace("{ws}", WORKSPACE)
            .replace("{bin}", binary_dir)
            .replace("{ev}", ROOT)
            .replace("{python}", sys.executable))


def run_command(command):
    started = time.time()
    try:
        completed = subprocess.run(command, shell=True, capture_output=True,
                                   text=True, timeout=3600)
    except subprocess.TimeoutExpired:
        return {"command": command, "exit_code": None, "status": "FAIL",
                "seconds": time.time() - started,
                "stdout_tail": "", "stderr_tail": "timeout after 3600 s",
                "metrics": {}}
    output = (completed.stdout or "") + (completed.stderr or "")
    metrics = {}
    ran = re.search(r"\[==========\] (\d+) tests? from \d+ test suites? ran\.", output)
    if ran:
        metrics["tests"] = int(ran.group(1))
    passed = re.search(r"\[  PASSED  \] (\d+) tests?", output)
    failed = re.search(r"\[  FAILED  \] (\d+) tests?", output)
    if passed:
        metrics["passed"] = int(passed.group(1))
    if failed:
        metrics["failed"] = int(failed.group(1))
    metrics["wall_seconds"] = round(time.time() - started, 2)
    status = "PASS" if completed.returncode == 0 else "FAIL"
    if status == "PASS" and "tests" in metrics and metrics["tests"] == 0:
        status = "SKIP_WITH_REASON"
    return {"command": command, "exit_code": completed.returncode,
            "status": status, "seconds": time.time() - started,
            "stdout_tail": "\n".join(output.splitlines()[-8:]),
            "stderr_tail": "", "metrics": metrics}


def git_sha(scope=None):
    # P7: the report must name the round's **code** commit.  When the evidence
    # commit already exists on top of it, pass it explicitly via
    # UWB_IMU_PL_VALIDATION_SHA (checked to be a real commit).
    override = os.environ.get("UWB_IMU_PL_VALIDATION_SHA", "").strip()
    if override:
        try:
            return subprocess.run(["git", "rev-parse", f"{override}^{{commit}}"],
                                  cwd=scope or REPO, capture_output=True,
                                  text=True, check=True).stdout.strip()
        except Exception:
            pass
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"],
                              cwd=scope or REPO, capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest",
                        default=os.path.join(ROOT, "validation-manifest.json"))
    parser.add_argument("--out",
                        default=os.path.join(ROOT, "validation-report.json"))
    parser.add_argument("--only", default="", help="comma separated item ids")
    parser.add_argument("--all", action="store_true",
                        help="run every item that has commands")
    parser.add_argument("--config", default=os.path.join(
        REPO, "config", "realtime_uwb_imu_pl_research.yaml"))
    # Moved here in P3 (B1): the dispatcher lives with the repository tools; the
    # manifest and reports stay in the evidence directory.
    args = parser.parse_args()

    with open(args.manifest) as handle:
        manifest = json.load(handle)
    binary_dir = resolve(manifest["binary_dir"], "")
    chosen = {item.strip() for item in args.only.split(",") if item.strip()}
    config_digest = None
    run_dir = os.path.join(ROOT, "raw", "runs_p2", "A_nominal")
    resolved = os.path.join(run_dir, "resolved_config.yaml")
    if os.path.exists(resolved):
        with open(resolved) as handle:
            for line in handle:
                match = re.match(r"\s*config_hash:\s*(\S+)", line)
                if match:
                    config_digest = match.group(1)
                    break

    report = {
        "schema": "uwb-imu-pl/validation-report/1",
        "produced_by": "tools/run_validation.py",
        "base_sha": "2772b3a1e3584b25c889239421404f550fba9f5d",
        "run_sha": git_sha(),
        "workspace_root": WORKSPACE,
        "config_digest": config_digest,
        "items": [],
    }
    for item in manifest["items"]:
        ident = item["id"]
        if chosen and ident not in chosen:
            continue
        row = {
            "case_id": ident,
            "description": item["description"],
            "owner_work_package": item["owner_work_package"],
            "manifest_status": item["status"],
            "commands": [],
            "status": None,
            "artifact_paths": [os.path.relpath(
                os.path.join(ROOT, path), ROOT)
                if not os.path.isabs(path) else path
                for path in item.get("evidence", [])],
            "note": item.get("note", ""),
        }
        if item["status"] not in ("MAPPED", "MAPPED_PARTIAL") or not item.get("commands"):
            row["status"] = "NOT_RUN"
            row["reason"] = (
                f"not started in P2; owning work package {item['owner_work_package']}")
            report["items"].append(row)
            continue
        statuses = []
        for template in item["commands"]:
            command = resolve(template, binary_dir)
            executable = command.split()[0]
            if executable.endswith(("test_integrity_config", "test_fault_manifest",
                                    "test_integrity_reference", "test_integrity_v2")) \
                    and not os.path.exists(executable):
                row["commands"].append({"command": command, "exit_code": None,
                                        "status": "NOT_RUN",
                                        "reason": "binary not built"})
                statuses.append("NOT_RUN")
                continue
            result = run_command(command)
            row["commands"].append(result)
            statuses.append(result["status"])
        if "FAIL" in statuses:
            row["status"] = "FAIL"
        elif all(status == "NOT_RUN" for status in statuses):
            row["status"] = "NOT_RUN"
        elif "SKIP_WITH_REASON" in statuses and "PASS" not in statuses:
            row["status"] = "SKIP_WITH_REASON"
        else:
            row["status"] = "PASS"
        report["items"].append(row)

    summary = {"PASS": 0, "FAIL": 0, "NOT_RUN": 0, "SKIP_WITH_REASON": 0}
    for row in report["items"]:
        summary[row["status"]] = summary.get(row["status"], 0) + 1
    report["summary"] = summary
    with open(args.out, "w") as handle:
        json.dump(report, handle, indent=1)
    for row in report["items"]:
        print(f"{row['case_id']}: {row['status']} "
              f"({row['manifest_status']}, WP {row['owner_work_package']})")
    print(f"summary: {summary}")
    print(f"written {os.path.relpath(args.out, ROOT)}")
    return 0 if summary["FAIL"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
