#!/usr/bin/env python3
"""Resumable Integrity V2 round-two campaign orchestrator.

Formal runs are deliberately refused on a dirty checkout. Reduced campaigns are
labelled SMOKE and can exercise the machinery, but can never finalize as PASS.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import pathlib
import platform
import random
import shutil
import subprocess
import sys
import time
from collections import Counter, defaultdict
from typing import Any

from round2_common import (FAIL, INVALID, PASS, STATUSES, CounterRng,
                           InvalidCampaign, atomic_json, canonical_json, clopper_pearson,
                           enumerate_cells, inventory, load_protocol,
                           seed_for, sha256_bytes, sha256_file, verify_checksums)

HERE = pathlib.Path(__file__).resolve().parent
REPOSITORY = HERE.parent
WORKSPACE = REPOSITORY.parent.parent
DEFAULT_PROTOCOL = REPOSITORY / "config/integrity_round2_protocol.json"
NUMERIC_ENV = {
    "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1", "BLIS_NUM_THREADS": "1",
}


def git(*arguments: str) -> str:
    return subprocess.check_output(["git", "-C", str(REPOSITORY), *arguments],
                                   text=True).strip()


def campaign_path(args: argparse.Namespace, protocol_sha: str) -> pathlib.Path:
    sha = git("rev-parse", "HEAD")
    return args.root / f"{sha[:12]}-{protocol_sha[:12]}"


def state_path(root: pathlib.Path) -> pathlib.Path:
    return root / "campaign_state.json"


def load_state(root: pathlib.Path) -> dict[str, Any]:
    try:
        return json.loads(state_path(root).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise InvalidCampaign(f"campaign is not prepared: {error}") from error


def save_state(root: pathlib.Path, state: dict[str, Any]) -> None:
    state["updated_unix_s"] = time.time()
    atomic_json(state_path(root), state)


def ensure_identity(root: pathlib.Path, protocol_sha: str) -> dict[str, Any]:
    state = load_state(root)
    if state["git_sha"] != git("rev-parse", "HEAD"):
        raise InvalidCampaign("campaign SHA differs from checkout; old evidence retained")
    if state["protocol_sha256"] != protocol_sha:
        raise InvalidCampaign("campaign protocol hash differs; old evidence retained")
    return state


def require_previous_stage(state: dict[str, Any], command: str) -> None:
    predecessor = {"build": "prepare", "calibrate": "build", "pilot": "calibrate",
                   "run": "pilot", "analyze": "run", "finalize": "analyze"}.get(command)
    previous_status = state.get("stages", {}).get(predecessor) if predecessor else None
    acceptable = STATUSES if command == "finalize" else {PASS}
    if predecessor and previous_status not in acceptable:
        raise InvalidCampaign(
            f"stage {command} cannot follow {predecessor}={previous_status}")


def command_prepare(args: argparse.Namespace, protocol: dict[str, Any],
                    protocol_sha: str) -> pathlib.Path:
    root = campaign_path(args, protocol_sha)
    root.mkdir(parents=True, exist_ok=True)
    cells = list(enumerate_cells(protocol))
    if len({cell["id"] for cell in cells}) != len(cells):
        raise InvalidCampaign("canonical scenario ID collision")
    atomic_json(root / "cell_inventory.json", {
        "schema_version": "uwb-imu-pl/round2-cell-inventory/v1",
        "protocol_sha256": protocol_sha, "cells": cells})
    atomic_json(root / "failure_catalog.json", protocol["failure_catalog"])
    protocol_copy = root / "frozen_protocol.json"
    shutil.copyfile(args.protocol, protocol_copy)
    dirty = bool(git("status", "--porcelain"))
    state = {
        "schema_version": "uwb-imu-pl/round2-campaign-state/v1",
        "status": INVALID,
        "profile": "FULL" if args.limit_cells is None else "SMOKE",
        "git_sha": git("rev-parse", "HEAD"), "git_dirty_at_prepare": dirty,
        "protocol_sha256": protocol_sha, "protocol_path": str(args.protocol),
        "cell_count": len(cells), "limit_cells": args.limit_cells,
        "stages": {name: "PENDING" for name in
                   ("prepare", "build", "calibrate", "pilot", "run",
                    "analyze", "finalize")},
        "environment": {"platform": platform.platform(),
                        "python": sys.version.split()[0], **NUMERIC_ENV},
        "commands": [" ".join(sys.argv)],
    }
    state["stages"]["prepare"] = PASS
    save_state(root, state)
    return root


def run_logged(command: list[str], log: pathlib.Path,
               cwd: pathlib.Path = WORKSPACE) -> None:
    environment = os.environ.copy()
    environment.update(NUMERIC_ENV)
    library_paths = [WORKSPACE / "devel/.private/uwb_imu_pl/lib",
                     WORKSPACE / "devel/lib", pathlib.Path("/opt/ros/noetic/lib")]
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, library_paths)) + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w", encoding="utf-8") as stream:
        result = subprocess.run(command, cwd=cwd, env=environment,
                                stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise InvalidCampaign(f"command failed ({result.returncode}); see {log}")


def test_xml_summary() -> dict[str, int]:
    import xml.etree.ElementTree as ET
    result = Counter(files=0, tests=0, errors=0, failures=0, skipped=0)
    for path in (WORKSPACE / "build/uwb_imu_pl/test_results").rglob("*.xml"):
        result["files"] += 1
        root = ET.parse(path).getroot()
        suites = [root] if root.tag == "testsuite" else root.findall(".//testsuite")
        for suite in suites:
            for field in ("tests", "errors", "failures"):
                result[field] += int(suite.attrib.get(field, 0))
            result["skipped"] += int(suite.attrib.get(
                "disabled", suite.attrib.get("skipped", 0)))
    return dict(result)


def command_build(root: pathlib.Path, state: dict[str, Any],
                  protocol: dict[str, Any]) -> None:
    if state["profile"] == "FULL" and git("status", "--porcelain"):
        raise InvalidCampaign("DIRTY_SHA: build evidence requires a clean checkout")
    summaries = {}
    shell_prefix = "source /opt/ros/noetic/setup.bash && "
    for build_type in ("Debug", "Release"):
        command = shell_prefix + (
            "catkin clean uwb_imu_pl -y && "
            f"catkin config --cmake-args -DCMAKE_BUILD_TYPE={build_type} && "
            "catkin build uwb_imu_pl --no-status && "
            "catkin run_tests uwb_imu_pl --no-status && "
            "catkin_test_results --verbose")
        run_logged(["bash", "-lc", command], root / "build" /
                   f"{build_type.lower()}.log")
        summary = test_xml_summary()
        summary["status"] = PASS if all(summary[key] == 0 for key in
                                         ("errors", "failures", "skipped")) else FAIL
        summaries[build_type] = summary
    atomic_json(root / "evidence/build_summary.json", summaries)
    performance_profile = "full" if state["profile"] == "FULL" else "smoke"
    subprocess.run([sys.executable, str(HERE / "run_round2_performance.py"),
                    str(root / "frozen_protocol.json"), str(root / "performance"),
                    "--profile", performance_profile], cwd=REPOSITORY,
                   env={**os.environ, **NUMERIC_ENV})
    state["stages"]["build"] = PASS if all(
        item["status"] == PASS for item in summaries.values()) else FAIL
    save_state(root, state)


def motion_sample(rng: CounterRng, index: int) -> dict[str, float]:
    return {
        "accel_x": 1.5 * rng.uniform(index, 0),
        "accel_y": 1.5 * rng.uniform(index, 1),
        "accel_z": 1.0 * rng.uniform(index, 2),
        "angular_rate_x": 0.8 * rng.uniform(index, 3),
        "angular_rate_y": 0.8 * rng.uniform(index, 4),
        "angular_rate_z": 1.5 * rng.uniform(index, 5),
        "angular_accel_x": 1.0 * rng.uniform(index, 6),
        "angular_accel_y": 1.0 * rng.uniform(index, 7),
        "angular_accel_z": 2.0 * rng.uniform(index, 8),
    }


def command_calibrate(root: pathlib.Path, state: dict[str, Any],
                      protocol: dict[str, Any]) -> None:
    count = protocol["sample_counts"]["gate_g_calibration"]
    raw = root / "raw/calibration/bridge_samples.csv"
    raw.parent.mkdir(parents=True, exist_ok=True)
    fields = ["seed", *motion_sample(CounterRng(0, "x", "x"), 0)]
    maxima = {field: 0.0 for field in fields[1:]}
    scalar_bounds = {"acceleration_bound_mps2": 0.0,
                     "angular_rate_bound_radps": 0.0,
                     "angular_acceleration_bound_radps2": 0.0}
    with raw.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(count):
            seed = seed_for(protocol, "calibration", "gate-g-bridge", index)
            sample = motion_sample(CounterRng(seed, "calibration", "bridge"), index)
            writer.writerow({"seed": seed, **sample})
            for key, value in sample.items():
                maxima[key] = max(maxima[key], abs(value))
            scalar_bounds["acceleration_bound_mps2"] = max(
                scalar_bounds["acceleration_bound_mps2"],
                math.sqrt(sum(sample[f"accel_{axis}"] ** 2 for axis in "xyz")))
            scalar_bounds["angular_rate_bound_radps"] = max(
                scalar_bounds["angular_rate_bound_radps"],
                math.sqrt(sum(sample[f"angular_rate_{axis}"] ** 2 for axis in "xyz")))
            scalar_bounds["angular_acceleration_bound_radps2"] = max(
                scalar_bounds["angular_acceleration_bound_radps2"],
                math.sqrt(sum(sample[f"angular_accel_{axis}"] ** 2 for axis in "xyz")))
    data_digest = sha256_file(raw)
    calibration_inputs = {
        "protocol_sha256": state["protocol_sha256"],
        "script_sha256": sha256_file(pathlib.Path(__file__)),
        "parameters": {"sample_count": count,
                       "seed_domain": protocol["seed_domains"]["calibration"]},
        "data_sha256": data_digest, "axis_bounds": maxima,
        "scalar_bounds": scalar_bounds,
    }
    digest = sha256_bytes(canonical_json(calibration_inputs))
    identifier = f"{protocol['gate_g']['calibration_prefix']}-{digest[:12]}"
    summary = {
        "schema_version": "uwb-imu-pl/bridge-calibration-summary/v1",
        "status": PASS, "research_only": True, "gate_j_product_calibration": False,
        "calibration_id": identifier, "protocol_sha256": state["protocol_sha256"],
        "sample_count": count, "calibration_digest": digest,
        **calibration_inputs,
        "inventory": [{"path": raw.relative_to(root).as_posix(),
                       "sha256": data_digest, "rows": count}],
    }
    atomic_json(root / "evidence/bridge_calibration_summary.json", summary)
    analytic = root / "evidence/analytic_mc_summary.json"
    profile = "full" if state["profile"] == "FULL" else "smoke"
    completed = subprocess.run(
        [sys.executable, str(HERE / "run_round2_analytic_mc.py"),
         str(root / "frozen_protocol.json"), str(analytic), "--profile", profile],
        cwd=REPOSITORY, env={**os.environ, **NUMERIC_ENV})
    state["bridge_calibration_id"] = identifier
    state["bridge_calibration_bounds"] = scalar_bounds
    state["analytic_mc_status"] = PASS if completed.returncode == 0 else (
        FAIL if completed.returncode == 1 else INVALID)
    state["stages"]["calibrate"] = (
        PASS if completed.returncode in (0, 1) else INVALID)
    save_state(root, state)


def task_inventory(protocol: dict[str, Any], profile: str,
                   cells: list[dict[str, Any]]) -> list[dict[str, Any]]:
    tasks = []
    if profile == "PILOT":
        for cell in cells:
            for ordinal in range(protocol["sample_counts"]["pilot_per_cell"]):
                tasks.append({"cell": cell, "domain": "development",
                              "ordinal": ordinal})
        return tasks
    for cell in cells:
        gate = cell["gate"]
        if gate == "G":
            continue
        count = (1 if gate == "I" and cell.get("noise") == "zero" else
                 protocol["sample_counts"]["gate_i_noisy_per_scenario"]
                 if gate == "I" else protocol["sample_counts"]["test_per_cell"])
        for ordinal in range(count):
            tasks.append({"cell": cell, "domain": "test", "ordinal": ordinal})
    gate_g = [cell for cell in cells if cell["gate"] == "G"]
    for domain, total, envelope in (
            ("test", protocol["sample_counts"]["gate_g_held_out"], "in_envelope"),
            ("stress", protocol["sample_counts"]["gate_g_stress"], "out_of_envelope")):
        selected = [cell for cell in gate_g if cell["envelope"] == envelope]
        for ordinal in range(total):
            tasks.append({"cell": selected[ordinal % len(selected)],
                          "domain": domain, "ordinal": ordinal})
    return tasks


def run_one(task: dict[str, Any], root: pathlib.Path, protocol: dict[str, Any],
            executable: pathlib.Path, code_sha: str,
            campaign_metadata: dict[str, Any]) -> tuple[str, str]:
    cell, domain, ordinal = task["cell"], task["domain"], task["ordinal"]
    seed = seed_for(protocol, domain, cell["id"], ordinal)
    destination = (root / "raw/outcomes" / cell["gate"] / cell["id"] /
                   f"{domain}-{ordinal:06d}-{seed}.json")
    if destination.is_file():
        try:
            existing = json.loads(destination.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return INVALID, destination.as_posix()
        identity = (existing.get("cell_id") == cell["id"] and
                    existing.get("seed_domain") == domain and
                    existing.get("ordinal") == ordinal and
                    existing.get("seed") == seed and
                    existing.get("git_sha") == code_sha)
        return ("SKIP" if identity else INVALID), destination.as_posix()
    destination.parent.mkdir(parents=True, exist_ok=True)
    runtime_cell = dict(cell)
    offsets = protocol["onset_offsets"]
    runtime_cell["onset_offset"] = offsets[ordinal % len(offsets)]
    if cell["gate"] == "F":
        runtime_cell["interval_offset"] = ordinal % int(cell["window"])
    if cell["gate"] == "G":
        runtime_cell["calibration_id"] = campaign_metadata.get(
            "bridge_calibration_id", "")
        runtime_cell.update(campaign_metadata.get("bridge_calibration_bounds", {}))
    command = [str(executable), str(REPOSITORY / "config/realtime_uwb_imu_pl_research.yaml"),
               json.dumps(runtime_cell, sort_keys=True, separators=(",", ":")), str(seed)]
    environment = os.environ.copy()
    environment.update(NUMERIC_ENV)
    library_paths = [WORKSPACE / "devel/.private/uwb_imu_pl/lib",
                     WORKSPACE / "devel/lib", pathlib.Path("/opt/ros/noetic/lib")]
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, library_paths)) + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    result = subprocess.run(command, cwd=WORKSPACE, env=environment,
                            text=True, capture_output=True)
    if result.returncode:
        atomic_json(destination, {"status": INVALID, "cell_id": cell["id"],
                    "gate": cell["gate"], "seed": seed, "seed_domain": domain,
                    "ordinal": ordinal, "git_sha": code_sha,
                    "failure_reason": "RUNNER_ERROR",
                    "stderr": result.stderr[-4000:]})
        return INVALID, destination.as_posix()
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError:
        payload = {"status": INVALID, "cell_id": cell["id"], "seed": seed,
                   "failure_reason": "MALFORMED_RUNNER_OUTPUT",
                   "stdout": result.stdout[-4000:]}
    payload.update({**cell, "cell_id": cell["id"], "gate": cell["gate"],
                    "seed": seed, "seed_domain": domain, "ordinal": ordinal,
                    "git_sha": code_sha,
                    "onset_offset": runtime_cell["onset_offset"],
                    "interval_offset": runtime_cell.get("interval_offset")})
    atomic_json(destination, payload)
    return payload.get("status", INVALID), destination.as_posix()


def command_samples(root: pathlib.Path, state: dict[str, Any],
                    protocol: dict[str, Any], stage: str,
                    limit_cells: int | None) -> None:
    if (stage == "run" and state["profile"] == "FULL" and
            git("status", "--porcelain")):
        raise InvalidCampaign("DIRTY_SHA: formal run requires a clean checkout")
    cells = json.loads((root / "cell_inventory.json").read_text())["cells"]
    if limit_cells is not None:
        cells = cells[:limit_cells]
        state["profile"] = "SMOKE"
    tasks = task_inventory(protocol, "PILOT" if stage == "pilot" else "FULL", cells)
    atomic_json(root / f"{stage}_task_inventory.json", {"tasks": tasks})
    executable = WORKSPACE / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl/integrity_round2_scenario"
    if not executable.is_file():
        raise InvalidCampaign(f"scenario runner is not built: {executable}")
    counts = Counter()
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(run_one, task, root, protocol, executable,
                                   state["git_sha"], state)
                   for task in tasks]
        for future in concurrent.futures.as_completed(futures):
            status, _ = future.result()
            counts[status] += 1
    atomic_json(root / f"evidence/{stage}_execution_summary.json",
                {"status": INVALID if counts[INVALID] else PASS,
                 "counts": dict(counts), "task_count": len(tasks),
                 "workers": 4, "numeric_threads": 1})
    state["stages"][stage] = INVALID if counts[INVALID] else PASS
    save_state(root, state)


def command_analyze(root: pathlib.Path, state: dict[str, Any],
                    protocol: dict[str, Any]) -> None:
    task_file = root / "run_task_inventory.json"
    if not task_file.is_file():
        raise InvalidCampaign("run task inventory is absent")
    tasks = json.loads(task_file.read_text(encoding="utf-8"))["tasks"]
    expected = Counter((task["cell"]["gate"], task["cell"]["id"],
                        task["domain"], task["ordinal"]) for task in tasks)
    observed = {}
    invalid_reasons = Counter()
    by_gate: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for path in (root / "raw/outcomes").rglob("*.json"):
        payload = json.loads(path.read_text(encoding="utf-8"))
        key = (payload.get("gate"), payload.get("cell_id"),
               payload.get("seed_domain"), payload.get("ordinal"))
        if key in expected:
            if key in observed:
                raise InvalidCampaign(f"duplicate outcome key: {key}")
            observed[key] = payload
            by_gate[payload["gate"]].append(payload)
            if payload.get("status") == INVALID:
                invalid_reasons[payload.get("failure_reason", "UNKNOWN")] += 1
    missing = list((expected - Counter(observed)).elements())
    missing_by_gate = Counter(item[0] for item in missing)
    verdicts = {}
    build_path = root / "evidence/build_summary.json"
    build = json.loads(build_path.read_text()) if build_path.is_file() else {}
    gate_a = PASS if build and all(build.get(kind, {}).get("status") == PASS
                                   for kind in ("Debug", "Release")) else INVALID
    atomic_json(root / "evidence/gate_a_verdict.json", {
        "schema_version": "uwb-imu-pl/round2-gate-verdict/v1",
        "gate": "A", "status": gate_a, "checks": build})
    verdicts["A"] = gate_a
    analytic_path = root / "evidence/analytic_mc_summary.json"
    analytic = json.loads(analytic_path.read_text()) if analytic_path.is_file() else {}
    for gate, family in (("B", "H0"), ("C", "NONCENTRAL")):
        family_cells = [cell for cell in analytic.get("cells", [])
                        if cell.get("family") == family]
        gate_status = analytic.get("status", INVALID) if family_cells else INVALID
        atomic_json(root / f"evidence/gate_{gate.lower()}_verdict.json", {
            "schema_version": "uwb-imu-pl/round2-gate-verdict/v1",
            "gate": gate, "status": gate_status,
            "checks": {"family": family, "cells": len(family_cells),
                       "global_parametric_bootstrap_p":
                           analytic.get("global_parametric_bootstrap_p"),
                       "holm_rejections": analytic.get("holm_rejections")}})
        verdicts[gate] = gate_status
    performance_path = root / "performance/performance_summary.json"
    performance = json.loads(performance_path.read_text()) if performance_path.is_file() else {}
    gate_d = performance.get("status", INVALID)
    atomic_json(root / "evidence/gate_d_verdict.json", {
        "schema_version": "uwb-imu-pl/round2-gate-verdict/v1",
        "gate": "D", "status": gate_d, "checks": performance})
    verdicts["D"] = gate_d
    for gate in "EFGHI":
        values = by_gate.get(gate, [])
        checks: dict[str, Any] = {"outcomes": len(values)}
        if (not values or missing_by_gate[gate] or
                any(value.get("status") == INVALID for value in values)):
            status = INVALID
        else:
            scientific_failures = [value for value in values
                                   if value.get("status") == FAIL]
            status = FAIL if scientific_failures else PASS
            safety = {
                "committed_action_without_post_detector": sum(
                    not value.get("post_fde_detector_passed", False) for value in values),
                "coverage_failures": sum(bool(value.get("coverage_failure")) for value in values
                    if not value.get("assumption_violation")),
                "hmi": sum(bool(value.get("hmi")) for value in values
                    if not value.get("assumption_violation")),
                "backend_update_violations": sum(
                    bool(value.get("backend_update_violation")) for value in values),
                "wrong_exclusion": sum(bool(value.get("wrong_exclusion"))
                                       for value in values),
                "unavailable_claimed_protected": sum(
                    bool(value.get("protected_available")) and
                    (not value.get("monitorable") or
                     value.get("ambiguous_exclusion") or
                     value.get("no_valid_candidate")) for value in values),
            }
            checks["safety_invariants"] = safety
            if any(safety.values()):
                status = FAIL
            eligible = [value for value in values if value.get("monitorable") and
                        value.get("ratio") in (2.0, 5.0)]
            if eligible:
                for metric in ("detected", "correct_exclusion"):
                    successes = sum(bool(value.get(metric)) for value in eligible)
                    interval = clopper_pearson(successes, len(eligible))
                    checks[metric] = {"successes": successes, "trials": len(eligible),
                                     "clopper_pearson_95": interval}
                    if interval[0] < protocol["statistical_gates"][
                            "monitorable_detection_lower" if metric == "detected"
                            else "monitorable_correct_exclusion_lower"]:
                        status = FAIL
                # Resample complete seed ordinals as blocks so correlations
                # among canonical cells sharing an ordinal are retained.
                blocks: dict[int, list[dict[str, Any]]] = defaultdict(list)
                for value in eligible:
                    blocks[int(value["ordinal"])].append(value)
                block_ids = sorted(blocks)
                bootstrap_rng = random.Random(int(state["protocol_sha256"][:16], 16) ^ ord(gate))
                for metric in ("detected", "correct_exclusion"):
                    estimates = []
                    for _ in range(2000):
                        sample = [blocks[bootstrap_rng.choice(block_ids)]
                                  for _ in block_ids]
                        flattened = [item for block in sample for item in block]
                        estimates.append(sum(bool(item.get(metric)) for item in flattened) /
                                         len(flattened))
                    estimates.sort()
                    checks[metric]["seed_block_bootstrap_95"] = [
                        estimates[49], estimates[1949]]
            boundary = [value for value in values if value.get("monitorable") and
                        value.get("ratio") == 1.0]
            if boundary:
                misses = sum(not bool(value.get("detected")) for value in boundary)
                interval = clopper_pearson(misses, len(boundary))
                boundary_blocks: dict[int, list[dict[str, Any]]] = defaultdict(list)
                for value in boundary:
                    boundary_blocks[int(value["ordinal"])].append(value)
                block_ids = sorted(boundary_blocks)
                boundary_rng = random.Random(
                    int(state["protocol_sha256"][16:32], 16) ^ ord(gate))
                bootstrap = []
                for _ in range(2000):
                    sample = [boundary_blocks[boundary_rng.choice(block_ids)]
                              for _ in block_ids]
                    flattened = [item for block in sample for item in block]
                    bootstrap.append(sum(not bool(item.get("detected"))
                                         for item in flattened) / len(flattened))
                bootstrap.sort()
                bootstrap_interval = [bootstrap[49], bootstrap[1949]]
                target = protocol["statistical_gates"]["p_md_boundary"]
                checks["boundary_miss"] = {"misses": misses,
                    "trials": len(boundary), "clopper_pearson_95": interval,
                    "seed_block_bootstrap_95": bootstrap_interval,
                    "target": target, "target_inside_interval":
                        bootstrap_interval[0] <= target <= bootstrap_interval[1]}
                if not bootstrap_interval[0] <= target <= bootstrap_interval[1]:
                    status = FAIL
            hmi_trials = [value for value in values
                          if not value.get("assumption_violation")]
            if hmi_trials:
                hmi_count = sum(bool(value.get("hmi")) for value in hmi_trials)
                hmi_interval = clopper_pearson(hmi_count, len(hmi_trials))
                checks["hmi_rate"] = {"events": hmi_count,
                    "trials": len(hmi_trials), "clopper_pearson_95": hmi_interval,
                    "zero_event_one_sided_95_upper":
                        (1.0 - 0.05 ** (1.0 / len(hmi_trials)))
                        if hmi_count == 0 else None,
                    "rare_event_certification": False}
            if gate == "E":
                nominal = [value for value in values if value.get("ratio") == 0.0 and
                           "normal8" in value.get("cell_id", "")]
                if nominal:
                    available = sum(bool(value.get("protected_available")) for value in nominal)
                    interval = clopper_pearson(available, len(nominal))
                    checks["normal8_nominal_availability"] = {
                        "available": available, "trials": len(nominal),
                        "clopper_pearson_95": interval}
                    if interval[0] < protocol["statistical_gates"][
                            "normal8_nominal_availability_lower"]:
                        status = FAIL
                regular = [value for value in values
                           if "near-rank" not in value.get("cell_id", "") and
                           value.get("ratio") != 0.0]
                monitorability_complete = all(value.get("monitorable") for value in regular)
                checks["regular_geometry_monitorability_complete"] = monitorability_complete
                conditions: dict[str, list[float]] = defaultdict(list)
                for value in values:
                    condition = value.get("geometry_condition")
                    if isinstance(condition, (int, float)):
                        cell_id = value.get("cell_id", "")
                        geometry = next((item for item in ("normal8", "poor7", "poor6")
                                         if item in cell_id), "")
                        if geometry:
                            conditions[geometry].append(float(condition))
                medians = {key: sorted(items)[len(items) // 2]
                           for key, items in conditions.items() if items}
                degradation = {key: medians[key] / medians["normal8"]
                               for key in ("poor7", "poor6")
                               if key in medians and "normal8" in medians}
                checks["geometry_condition_medians"] = medians
                checks["geometry_degradation"] = degradation
                if (not monitorability_complete or len(degradation) != 2 or
                        any(value < 2.0 for value in degradation.values())):
                    status = FAIL
            if gate in ("E", "F", "H"):
                coverable = [value for value in values if value.get("monitorable") and
                             value.get("ratio", 1.0) > 0.0]
                if coverable:
                    available = sum(bool(value.get("protected_available"))
                                    for value in coverable)
                    interval = clopper_pearson(available, len(coverable))
                    checks["coverable_fault_availability"] = {
                        "available": available, "trials": len(coverable),
                        "clopper_pearson_95": interval}
                    if interval[0] < protocol["statistical_gates"][
                            "coverable_fault_availability_lower"]:
                        status = FAIL
            if gate == "G":
                stress_valid = all(value.get("assumption_violation") is True
                                   for value in values
                                   if value.get("seed_domain") == "stress")
                timeout_verified = any(value.get("bridge_timeout_verified") is True
                                       for value in values)
                checks["stress_labeled_assumption_violation"] = stress_valid
                checks["bridge_timeout_verified"] = timeout_verified
                if not stress_valid or not timeout_verified:
                    status = FAIL
            if gate == "H":
                invariant = all(value.get("local_threshold_invariant") is True
                                for value in values)
                checks["local_threshold_invariant"] = invariant
                if not invariant:
                    status = FAIL
            if gate == "I":
                sequences = [value for value in values if value.get("seed_domain") == "test"
                             and value.get("ordinal") == 0 and
                             value.get("noise") == "zero"]
                state_machine = all(value.get("state_machine_verified") is True
                                    for value in sequences)
                checks["state_machine_verified"] = state_machine
                if not state_machine:
                    status = FAIL
        verdict = {"schema_version": "uwb-imu-pl/round2-gate-verdict/v1",
                   "gate": gate, "status": status, "checks": checks,
                   "failure_reasons": dict(invalid_reasons)}
        atomic_json(root / f"evidence/gate_{gate.lower()}_verdict.json", verdict)
        verdicts[gate] = status
    analysis_status = (INVALID if missing or INVALID in verdicts.values() else
                       FAIL if FAIL in verdicts.values() else PASS)
    atomic_json(root / "evidence/analysis_summary.json", {
        "status": analysis_status, "gate_verdicts": verdicts,
        "expected": len(expected), "observed": len(observed),
        "missing_count": len(missing), "missing_preview": missing[:100],
        "failure_reasons": dict(invalid_reasons)})
    state["stages"]["analyze"] = analysis_status
    save_state(root, state)


def command_finalize(root: pathlib.Path, state: dict[str, Any],
                     protocol: dict[str, Any]) -> None:
    verdicts = {}
    for gate in "ABCDEFGHI":
        path = root / f"evidence/gate_{gate.lower()}_verdict.json"
        if path.is_file():
            verdicts[gate] = json.loads(path.read_text())["status"]
        else:
            verdicts[gate] = INVALID
    dirty = bool(git("status", "--porcelain"))
    complete = (state["profile"] == "FULL" and
                not state.get("git_dirty_at_prepare", True) and not dirty and
                all(value == PASS for value in verdicts.values()))
    structurally_invalid = (state["profile"] != "FULL" or
                            state.get("git_dirty_at_prepare", True) or dirty or
                            INVALID in verdicts.values())
    status = (PASS if complete else INVALID if structurally_invalid else
              FAIL if FAIL in verdicts.values() else INVALID)
    acceptance = {
        "schema_version": "uwb-imu-pl/round2-acceptance/v1",
        "status": status, "git_sha": state["git_sha"],
        "git_dirty": dirty,
        "protocol_sha256": state["protocol_sha256"], "profile": state["profile"],
        "gates": verdicts, "gates_a_to_i_complete": complete,
        "implementation_maturity": "IMPLEMENTED_UNVERIFIED",
        "formal_eligible": False, "gate_j_complete": False,
        "bridge_calibration_research_only": True,
    }
    atomic_json(root / "round2_acceptance.json", acceptance)
    raw_files = [path.relative_to(root) for path in (root / "raw").rglob("*")
                 if path.is_file()]
    raw_inventory = inventory(root, raw_files)
    atomic_json(root / "raw_inventory.json", raw_inventory)
    manifest = {
        "schema_version": "uwb-imu-pl/round2-campaign-manifest/v1",
        "git_sha": state["git_sha"], "git_dirty": acceptance["git_dirty"],
        "protocol_sha256": state["protocol_sha256"],
        "protocol_path": "frozen_protocol.json",
        "raw_inventory_path": "raw_inventory.json",
        "raw_inventory_sha256": sha256_file(root / "raw_inventory.json"),
        "artifact_checksum_path": "checksums.sha256",
        "commands": state.get("commands", []),
        "environment": state["environment"],
        "seed_domains": protocol["seed_domains"], "attempt": 1,
        "failure_catalog_path": "failure_catalog.json",
        "bridge_calibration_id": state.get("bridge_calibration_id", ""),
        "gates_a_to_i_complete": complete, "formal_eligible": False,
        "gate_j_complete": False,
    }
    atomic_json(root / "campaign_manifest.json", manifest)
    evidence_files = [path.relative_to(root) for path in (root / "evidence").rglob("*")
                      if path.is_file()]
    atomic_json(root / "evidence_index.json", inventory(root, evidence_files))
    state["stages"]["finalize"] = status
    state["status"] = status
    save_state(root, state)
    checksum_targets = [path for path in root.rglob("*") if path.is_file() and
                        path.name != "checksums.sha256"]
    with (root / "checksums.sha256").open("w", encoding="utf-8") as stream:
        for path in sorted(checksum_targets):
            stream.write(f"{sha256_file(path)}  {path.relative_to(root).as_posix()}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "build", "calibrate", "pilot",
                                              "run", "analyze", "finalize", "status"))
    parser.add_argument("--protocol", type=pathlib.Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--root", type=pathlib.Path,
                        default=REPOSITORY / "results/integrity_round2")
    parser.add_argument("--limit-cells", type=int)
    args = parser.parse_args()
    try:
        protocol, protocol_sha = load_protocol(args.protocol)
        root = campaign_path(args, protocol_sha)
        if args.command == "prepare":
            root = command_prepare(args, protocol, protocol_sha)
        else:
            state = ensure_identity(root, protocol_sha)
            if args.command != "status":
                require_previous_stage(state, args.command)
            state.setdefault("commands", []).append(" ".join(sys.argv))
            save_state(root, state)
            if args.command == "build":
                command_build(root, state, protocol)
            elif args.command == "calibrate":
                command_calibrate(root, state, protocol)
            elif args.command in ("pilot", "run"):
                command_samples(root, state, protocol, args.command, args.limit_cells)
            elif args.command == "analyze":
                command_analyze(root, state, protocol)
            elif args.command == "finalize":
                command_finalize(root, state, protocol)
            else:
                finalized = (root / "checksums.sha256").is_file()
                errors = verify_checksums(root) if finalized else ["campaign is not finalized"]
                report = dict(state)
                report["artifact_verification"] = {
                    "status": PASS if finalized and not errors else INVALID,
                    "errors": errors}
                print(json.dumps(report, indent=2, sort_keys=True))
                return 0 if not finalized or not errors else 2
        print(f"{args.command}: {root}")
        return 0
    except (InvalidCampaign, OSError, subprocess.CalledProcessError) as error:
        print(f"INVALID: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
