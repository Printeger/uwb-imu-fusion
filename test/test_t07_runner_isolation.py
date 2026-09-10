#!/usr/bin/env python3
"""Prove the real paper runner reaches Stage 1 using only a T07 cache."""

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def load_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def identity(path):
    stat = os.stat(path, follow_symlinks=False)
    return {
        "device": stat.st_dev,
        "inode": stat.st_ino,
        "mode": stat.st_mode,
        "size": stat.st_size,
    }


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--recipe", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--evidence-dir", required=True)
    parser.add_argument("--expected-cache-id", required=True)
    parser.add_argument("--expected-recording-id", required=True)
    args = parser.parse_args()

    evidence_dir = Path(args.evidence_dir).resolve()
    evidence_dir.mkdir(parents=True, exist_ok=False)
    runner_stdout = evidence_dir / "runner_stdout.log"
    runner_stderr = evidence_dir / "runner_stderr.log"
    runner_command_path = evidence_dir / "runner_command.json"
    result_path = evidence_dir / "isolation_result.json"
    runner_argv = [
        str(Path(args.runner).resolve()),
        "--config", str(Path(args.config).resolve()),
        "--output-root", str(Path(args.output_root).resolve()),
        "--run-id", args.run_id,
    ]
    run_dir = Path(args.output_root).resolve() / args.run_id
    require(not run_dir.exists(), "runner output directory already exists")

    moves = []
    process = None
    runner_exit_code = None
    during_checks = 0
    during_all_absent = True
    pre_spawn_all_absent = False
    restoration = []
    try:
        for original in (args.base, args.recipe, args.truth):
            source = Path(original).absolute()
            require(source.exists(), "isolation source missing: " + str(source))
            require(not source.is_symlink(),
                    "isolation source must not itself be a symlink: " + str(source))
            backup = source.with_name(
                source.name + ".t07_isolation_moved." + str(os.getpid()))
            require(not os.path.lexists(backup),
                    "isolation backup already exists: " + str(backup))
            before = identity(source)
            os.replace(source, backup)
            moves.append((source, backup, before))
        pre_spawn_all_absent = all(
            not os.path.lexists(source) for source, _, _ in moves)
        require(pre_spawn_all_absent,
                "one or more isolated inputs remained accessible before spawn")

        with runner_stdout.open("w", encoding="utf-8") as stdout_stream, \
                runner_stderr.open("w", encoding="utf-8") as stderr_stream:
            process = subprocess.Popen(
                runner_argv,
                cwd=os.getcwd(),
                text=True,
                stdout=stdout_stream,
                stderr=stderr_stream,
            )
            while process.poll() is None:
                during_checks += 1
                if any(os.path.lexists(source) for source, _, _ in moves):
                    during_all_absent = False
                time.sleep(0.01)
            runner_exit_code = process.returncode
    finally:
        if process is not None and process.poll() is None:
            process.wait()
            runner_exit_code = process.returncode
        for source, backup, before in reversed(moves):
            if os.path.lexists(backup):
                os.replace(backup, source)
            restored = os.path.lexists(source) and not os.path.lexists(backup)
            same_identity = restored and identity(source) == before
            restoration.append({
                "path": str(source),
                "restored": restored,
                "identity_preserved": same_identity,
            })

    runner_command = {
        "schema": "t07_isolation_runner_command_v2",
        "cwd": os.getcwd(),
        "argv": runner_argv,
        "exit_code": runner_exit_code,
        "stdout_path": str(runner_stdout),
        "stderr_path": str(runner_stderr),
        "run_directory": str(run_dir),
    }
    runner_command_path.write_text(
        json.dumps(runner_command, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    input_manifest = load_json(run_dir / "input_manifest.json")
    status = load_json(run_dir / "run_status.json")
    capability = load_json(run_dir / "capability_status.json")
    diagnostics = load_json(run_dir / "discovery_failure_diagnostics.json")
    discovery_path = run_dir / "discovery_iterations.csv"
    with discovery_path.open(newline="", encoding="utf-8") as stream:
        discovery_rows = list(csv.DictReader(stream))

    require(runner_exit_code == 1, "unexpected paper-runner exit code")
    require(during_checks > 0 and during_all_absent,
            "isolated inputs were accessible while runner was executing")
    require(all(item["restored"] and item["identity_preserved"]
                for item in restoration),
            "isolation input restoration failed")
    require(input_manifest.get("cache_id") == args.expected_cache_id,
            "paper runner used an unexpected cache_id")
    require(input_manifest.get("recording_id") == args.expected_recording_id,
            "paper runner did not inherit the base recording_id")
    require(input_manifest.get("input_interface") == "t07_cache",
            "paper runner did not use t07_cache")
    require(input_manifest.get("cache_unit_conversion_applied") is False,
            "paper runner applied a second cache unit conversion")
    require(len(discovery_rows) > 0,
            "Stage 1 trace is empty; manifest existence is insufficient")
    require(diagnostics.get("outer_trace_rows") == len(discovery_rows) and
            diagnostics.get("status") == "MAX_OUTER_ITERATIONS",
            "Stage 1 diagnostics do not match the nonempty trace")
    require(status.get("status") == "FAILED" and
            status.get("exit_code") == 1 and
            status.get("failure_stage") == "AUTOMATIC_DISCOVERY" and
            status.get("discovery_status") == "MAX_OUTER_ITERATIONS" and
            status.get("stage2_refit_run") is False and
            status.get("gate_or_fallback_run") is False,
            "paper runner failure status changed")
    require(capability.get("segment_refit") == "NOT_RUN" and
            capability.get("recoverability_score") == "NOT_RUN",
            "Stage 2 or score status changed")

    result = {
        "schema": "t07_isolation_result_v2",
        "runner_exit_code": runner_exit_code,
        "run_status": status["status"],
        "failure_stage": status["failure_stage"],
        "discovery_status": status["discovery_status"],
        "segment_refit": capability["segment_refit"],
        "recoverability_score": capability["recoverability_score"],
        "cache_id": input_manifest["cache_id"],
        "inherited_recording_id": input_manifest["recording_id"],
        "input_interface": input_manifest["input_interface"],
        "cache_unit_conversion_applied":
            input_manifest["cache_unit_conversion_applied"],
        "pre_spawn_all_isolated_inputs_absent": pre_spawn_all_absent,
        "during_process_checks": during_checks,
        "during_process_all_isolated_inputs_absent": during_all_absent,
        "restoration": sorted(restoration, key=lambda item: item["path"]),
        "stage1_trace_path": str(discovery_path),
        "stage1_trace_rows": len(discovery_rows),
        "stage1_diagnostics_path":
            str(run_dir / "discovery_failure_diagnostics.json"),
        "runner_stdout_path": str(runner_stdout),
        "runner_stderr_path": str(runner_stderr),
        "runner_command_path": str(runner_command_path),
    }
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    result_path.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # Preserve a machine-visible checker failure.
        print("T07 isolation check failed: " + str(error), file=sys.stderr)
        sys.exit(1)
