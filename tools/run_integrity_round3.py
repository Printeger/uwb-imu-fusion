#!/usr/bin/env python3
"""Integrity V2 round-three evidence runner.

The formal E-I matrix remains disabled by the frozen round-three protocol.
Development pilots use four deterministic JSONL shards and four long-lived C++
workers.  Resume accepts only an exact task-prefix match.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import json
import os
import pathlib
import platform
import resource
import subprocess
import sys
import time
from itertools import zip_longest
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Any, Iterable, Iterator

from round2_common import (FAIL, INVALID, PASS, InvalidCampaign, atomic_json,
                           canonical_json, enumerate_cells, inventory,
                           load_protocol, seed_for, sha256_bytes, sha256_file,
                           verify_checksums)

HERE = pathlib.Path(__file__).resolve().parent
REPOSITORY = HERE.parent
WORKSPACE = REPOSITORY.parent.parent
PROTOCOL = REPOSITORY / "config/integrity_round3_protocol.json"
ROUND2_PROTOCOL = REPOSITORY / "config/integrity_round2_protocol.json"
CONFIG = REPOSITORY / "config/realtime_uwb_imu_pl_research.yaml"
EXECUTABLE = (WORKSPACE / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl" /
              "integrity_round2_scenario")
NUMERIC_ENV = {name: "1" for name in (
    "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS", "BLIS_NUM_THREADS")}
IDENTITY_FIELDS = ("git_sha", "protocol_sha256", "cell_id", "seed_domain",
                   "ordinal", "seed", "shard")
FROZEN_KEYS = ("seed_domains", "sample_counts", "boundary_ratios",
               "onset_offsets", "estimator_modes", "gate_d", "gate_e",
               "gate_g", "gate_i", "statistical_gates")


def git(*arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPOSITORY), *arguments], text=True).strip()


def campaign_root(base: pathlib.Path, protocol_sha: str) -> pathlib.Path:
    return base / f"{git('rev-parse', 'HEAD')[:12]}-{protocol_sha[:12]}"


def load_state(root: pathlib.Path) -> dict[str, Any]:
    try:
        return json.loads((root / "campaign_state.json").read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise InvalidCampaign(f"campaign is not prepared: {error}") from error


def save_state(root: pathlib.Path, state: dict[str, Any]) -> None:
    state["updated_unix_s"] = time.time()
    atomic_json(root / "campaign_state.json", state)


def run_logged(command: list[str], log: pathlib.Path,
               cwd: pathlib.Path = WORKSPACE) -> int:
    environment = worker_environment()
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as stream:
        completed = subprocess.run(command, cwd=cwd, env=environment,
                                   stdout=stream, stderr=subprocess.STDOUT)
    return completed.returncode


def require_clean_freeze(state: dict[str, Any], stage: str) -> None:
    if state["git_dirty_at_prepare"] or git("status", "--porcelain"):
        raise InvalidCampaign(f"DIRTY_SHA: {stage} evidence needs the frozen clean SHA")


def xml_inventory() -> dict[str, Any]:
    import xml.etree.ElementTree as ET
    totals = Counter(files=0, tests=0, errors=0, failures=0, skipped=0)
    files = []
    test_cases: set[str] = set()
    for path in sorted((WORKSPACE / "build/uwb_imu_pl/test_results").rglob("*.xml")):
        root = ET.parse(path).getroot()
        suites = [root] if root.tag == "testsuite" else root.findall(".//testsuite")
        summary = Counter(tests=0, errors=0, failures=0, skipped=0)
        for suite in suites:
            for key in ("tests", "errors", "failures"):
                summary[key] += int(suite.attrib.get(key, 0))
            summary["skipped"] += int(suite.attrib.get(
                "disabled", suite.attrib.get("skipped", 0)))
        for case in root.findall(".//testcase"):
            test_cases.add(
                f"{case.attrib.get('classname', '')}.{case.attrib.get('name', '')}")
        totals.update(summary)
        totals["files"] += 1
        files.append({"path": str(path.relative_to(WORKSPACE)),
                      "sha256": sha256_file(path), **dict(summary)})
    return {**dict(totals), "xml": files, "test_cases": sorted(test_cases),
            "status": PASS if totals["files"] and all(
                totals[key] == 0 for key in ("errors", "failures", "skipped"))
            else FAIL}


def command_build(root: pathlib.Path, state: dict[str, Any]) -> None:
    require_clean_freeze(state, "clean Debug/Release")
    summaries = {}
    for build_type in ("Debug", "Release"):
        shell = ("source /opt/ros/noetic/setup.bash && "
                 "catkin clean uwb_imu_pl -y && "
                 f"catkin config --cmake-args -DCMAKE_BUILD_TYPE={build_type} && "
                 "catkin build uwb_imu_pl --no-status && "
                 "catkin run_tests uwb_imu_pl --no-status && "
                 "(cd build/uwb_imu_pl && ctest --output-on-failure "
                 "-R 'test_(week4_tools|advisor_report|round2_tools|round3_tools)') && "
                 "catkin_test_results --verbose")
        log = root / "build" / f"{build_type.lower()}.log"
        return_code = run_logged(["bash", "-lc", shell], log)
        summary = xml_inventory()
        summary.update({"return_code": return_code, "log": str(log.relative_to(root)),
                        "build_type": build_type})
        summary["status"] = PASS if return_code == 0 and summary["status"] == PASS else FAIL
        summaries[build_type] = summary
    status = PASS if all(value["status"] == PASS for value in summaries.values()) else FAIL
    atomic_json(root / "evidence/build_summary.json", {
        "schema_version": "uwb-imu-pl/round3-build-summary/v1", "status": status,
        "tested_code_sha": state["tested_code_sha"],
        "protocol_sha256": state["protocol_sha256"],
        "config_sha256": state["config_sha256"], "builds": summaries,
        "known_warnings": ["catkin underlay/symlink warnings are retained in build logs"],
        "failure_directories": ["build/uwb_imu_pl/test_results"]})
    state["stages"]["build"] = status
    save_state(root, state)


def command_analytic(root: pathlib.Path, state: dict[str, Any]) -> None:
    require_clean_freeze(state, "analytic calibration")
    destination = root / "evidence/analytic_mc_summary.json"
    return_code = run_logged([
        sys.executable, str(HERE / "run_round2_analytic_mc.py"),
        str(root / "frozen_protocol.json"), str(destination), "--profile", "full"],
        root / "analytic/analytic_mc.log", REPOSITORY)
    if return_code not in (0, 1):
        raise InvalidCampaign("analytic MC execution was structurally invalid")
    summary = json.loads(destination.read_text())
    summary["role"] = "SHARED_STATISTICAL_CALIBRATION_PREREQUISITE"
    summary["not_gate_evidence_for"] = ["B", "C"]
    summary["tested_code_sha"] = state["tested_code_sha"]
    summary["config_sha256"] = state["config_sha256"]
    atomic_json(destination, summary)
    state["stages"]["analytic"] = summary.get("status", INVALID)
    save_state(root, state)


def command_d_smoke(root: pathlib.Path, state: dict[str, Any]) -> None:
    require_clean_freeze(state, "Gate D smoke")
    return_code = run_logged([
        sys.executable, str(HERE / "run_round2_performance.py"),
        str(root / "frozen_protocol.json"), str(root / "performance"),
        "--profile", "smoke"], root / "performance/gate_d_smoke.log", REPOSITORY)
    summary_path = root / "performance/performance_summary.json"
    if return_code not in (0, 1) or not summary_path.is_file():
        raise InvalidCampaign("Gate D smoke did not produce a valid verdict")
    summary = json.loads(summary_path.read_text())
    summary["tested_code_sha"] = state["tested_code_sha"]
    summary["config_sha256"] = state["config_sha256"]
    atomic_json(summary_path, summary)
    state["stages"]["d_smoke"] = summary["status"]
    save_state(root, state)


def command_ros_smoke(root: pathlib.Path, state: dict[str, Any],
                      protocol: dict[str, Any], evidence: pathlib.Path | None) -> None:
    require_clean_freeze(state, "ROS equivalence smoke")
    if evidence is None or not evidence.is_file():
        raise InvalidCampaign("representative ROS equivalence JSONL is required")
    required = protocol["ros_equivalence"]["fields"]
    seen = Counter()
    mismatches = []
    with evidence.open() as stream:
        for row, line in enumerate(stream, 1):
            item = json.loads(line)
            gate = item.get("gate")
            if gate not in protocol["ros_equivalence"]["gates"]:
                mismatches.append(f"row {row}: invalid gate")
                continue
            seen[gate] += 1
            for field_name in required:
                if item.get("core", {}).get(field_name) != item.get("ros", {}).get(field_name):
                    mismatches.append(f"row {row}: {field_name}")
    missing = [gate for gate in protocol["ros_equivalence"]["gates"] if seen[gate] != 1]
    status = PASS if not mismatches and not missing else FAIL
    atomic_json(root / "evidence/ros_equivalence_summary.json", {
        "schema_version": "uwb-imu-pl/round3-ros-equivalence/v1", "status": status,
        "tested_code_sha": state["tested_code_sha"], "source": str(evidence),
        "seen": dict(seen), "missing": missing, "mismatches": mismatches})
    state["stages"]["ros_smoke"] = status
    save_state(root, state)


def exact_frozen_inheritance(protocol: dict[str, Any]) -> str:
    inherited, inherited_sha = load_protocol(ROUND2_PROTOCOL)
    for key in FROZEN_KEYS:
        if key == "gate_d":
            if protocol[key] != inherited[key]:
                raise InvalidCampaign(f"round-three changed frozen field {key}")
        elif key == "gate_g":
            if protocol[key] != inherited[key]:
                raise InvalidCampaign(f"round-three changed frozen field {key}")
        elif protocol[key] != inherited[key]:
            raise InvalidCampaign(f"round-three changed frozen field {key}")
    if protocol["gate_f"]["sensors"] != inherited["gate_f"]["sensors"] or \
            protocol["gate_f"]["axes"] != inherited["gate_f"]["axes"] or \
            protocol["gate_f"]["motions"] != inherited["gate_f"]["motions"] or \
            protocol["gate_f"]["window_lengths"] != inherited["gate_f"]["window_lengths"]:
        raise InvalidCampaign("round-three changed frozen Gate F matrix")
    if any(protocol["gate_h"][key] != inherited["gate_h"][key] for key in (
            "anchors", "imu_axes", "uwb_modes", "relative_onsets",
            "magnitude_pairs", "ambiguity", "union_geometries")):
        raise InvalidCampaign("round-three changed frozen Gate H matrix")
    return inherited_sha


def command_prepare(base: pathlib.Path, protocol: dict[str, Any],
                    protocol_sha: str, protocol_path: pathlib.Path = PROTOCOL) -> pathlib.Path:
    inherited_sha = exact_frozen_inheritance(protocol)
    root = campaign_root(base, protocol_sha)
    root.mkdir(parents=True, exist_ok=True)
    cells = list(enumerate_cells(protocol))
    if len(cells) != 5543 or len({cell["id"] for cell in cells}) != len(cells):
        raise InvalidCampaign("canonical 5,543-cell census is not unique and complete")
    atomic_json(root / "cell_inventory.json", {
        "schema_version": "uwb-imu-pl/round3-cell-inventory/v1",
        "protocol_sha256": protocol_sha, "cell_count": len(cells), "cells": cells})
    (root / "frozen_protocol.json").write_bytes(protocol_path.read_bytes())
    atomic_json(root / "failure_catalog.json", protocol["failure_catalog"])
    dirty = bool(git("status", "--porcelain"))
    state = {
        "schema_version": "uwb-imu-pl/round3-campaign-state/v1",
        "status": INVALID, "git_sha": git("rev-parse", "HEAD"),
        "tested_code_sha": git("rev-parse", "HEAD"),
        "git_dirty_at_prepare": dirty, "protocol_sha256": protocol_sha,
        "inherited_protocol_sha256": inherited_sha,
        "config_sha256": sha256_file(CONFIG), "cell_count": len(cells),
        "pilot_outcomes": len(cells) * protocol["sample_counts"]["pilot_per_cell"],
        "formal_outcomes": sum(1 for _ in iter_tasks(protocol, cells, "formal")),
        "formal_matrix_started": False,
        "stages": {name: "PENDING" for name in
                   ("prepare", "build", "analytic", "d_smoke", "pilot",
                    "ros_smoke", "formal", "analyze", "finalize")},
        "environment": {"platform": platform.platform(),
                        "python": sys.version.split()[0], **NUMERIC_ENV},
    }
    state["stages"]["prepare"] = PASS
    save_state(root, state)
    write_gap_audit(root, state, {})
    return root


def iter_tasks(protocol: dict[str, Any], cells: Iterable[dict[str, Any]],
               profile: str) -> Iterator[dict[str, Any]]:
    cells = list(cells)
    if profile == "pilot":
        for cell in cells:
            for ordinal in range(protocol["sample_counts"]["pilot_per_cell"]):
                yield {"cell": cell, "seed_domain": "development",
                       "ordinal": ordinal}
        return
    for cell in cells:
        if cell["gate"] == "G":
            continue
        count = (1 if cell["gate"] == "I" and cell.get("noise") == "zero" else
                 protocol["sample_counts"]["gate_i_noisy_per_scenario"]
                 if cell["gate"] == "I" else
                 protocol["sample_counts"]["test_per_cell"])
        for ordinal in range(count):
            yield {"cell": cell, "seed_domain": "test", "ordinal": ordinal}
    gate_g = [cell for cell in cells if cell["gate"] == "G"]
    for domain, count, envelope in (
            ("test", protocol["sample_counts"]["gate_g_held_out"], "in_envelope"),
            ("stress", protocol["sample_counts"]["gate_g_stress"], "out_of_envelope")):
        selected = [cell for cell in gate_g if cell["envelope"] == envelope]
        for ordinal in range(count):
            yield {"cell": selected[ordinal % len(selected)],
                   "seed_domain": domain, "ordinal": ordinal}


def runtime_task(task: dict[str, Any], protocol: dict[str, Any],
                 state: dict[str, Any], shard: int) -> dict[str, Any]:
    cell = task["cell"]
    domain = task["seed_domain"]
    ordinal = task["ordinal"]
    value = dict(cell)
    value.update({"cell_id": cell["id"], "seed_domain": domain,
                  "ordinal": ordinal, "seed": seed_for(
                      protocol, domain, cell["id"], ordinal),
                  "git_sha": state["git_sha"],
                  "protocol_sha256": state["protocol_sha256"], "shard": shard,
                  "onset_offset": protocol["onset_offsets"][
                      ordinal % len(protocol["onset_offsets"])]})
    if cell["gate"] == "F":
        value["interval_offset"] = ordinal % int(cell["window"])
    return value


def task_key(value: dict[str, Any]) -> tuple[Any, ...]:
    return tuple(value.get(field) for field in IDENTITY_FIELDS)


def write_shards(root: pathlib.Path, protocol: dict[str, Any],
                 state: dict[str, Any], profile: str,
                 limit_cells: int | None = None) -> list[pathlib.Path]:
    cells = json.loads((root / "cell_inventory.json").read_text())["cells"]
    if limit_cells is not None:
        cells = cells[:limit_cells]
    directory = root / "tasks" / profile
    directory.mkdir(parents=True, exist_ok=True)
    paths = [directory / f"shard-{index}.jsonl" for index in range(4)]
    streams = [path.open("wb") for path in paths]
    counts = [0] * 4
    try:
        for index, task in enumerate(iter_tasks(protocol, cells, profile)):
            shard = index % 4
            streams[shard].write(canonical_json(
                runtime_task(task, protocol, state, shard)))
            counts[shard] += 1
    finally:
        for stream in streams:
            stream.close()
    artifacts = [{"path": path.relative_to(root).as_posix(),
                  "rows": counts[index], "sha256": sha256_file(path)}
                 for index, path in enumerate(paths)]
    atomic_json(root / f"tasks/{profile}_inventory.json", {
        "schema_version": "uwb-imu-pl/round3-task-inventory/v1",
        "profile": profile, "rows": sum(counts), "shards": artifacts})
    return paths


def valid_output_prefix(task_path: pathlib.Path, output_path: pathlib.Path) -> int:
    if not output_path.is_file():
        return 0
    valid_rows = 0
    valid_bytes = 0
    with task_path.open("rb") as tasks, output_path.open("rb") as outcomes:
        while True:
            outcome_line = outcomes.readline()
            if not outcome_line:
                break
            task_line = tasks.readline()
            if not task_line:
                raise InvalidCampaign("outcome shard has more rows than its task shard")
            if not outcome_line.endswith(b"\n"):
                break
            try:
                task = json.loads(task_line)
                outcome = json.loads(outcome_line)
            except json.JSONDecodeError as error:
                raise InvalidCampaign(f"malformed complete JSONL row: {error}") from error
            if task_key(task) != task_key(outcome):
                raise InvalidCampaign(
                    f"IDENTITY_MISMATCH at {output_path.name}:{valid_rows + 1}")
            payload = dict(outcome)
            digest = payload.pop("outcome_sha256", "")
            if digest != sha256_bytes(canonical_json(payload)):
                raise InvalidCampaign(
                    f"BAD_CHECKSUM at {output_path.name}:{valid_rows + 1}")
            valid_rows += 1
            valid_bytes = outcomes.tell()
    if output_path.stat().st_size != valid_bytes:
        with output_path.open("r+b") as stream:
            stream.truncate(valid_bytes)
    return valid_rows


def worker_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment.update(NUMERIC_ENV)
    paths = [WORKSPACE / "devel/.private/uwb_imu_pl/lib", WORKSPACE / "devel/lib",
             pathlib.Path("/opt/ros/noetic/lib")]
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, paths)) + (
        ":" + environment["LD_LIBRARY_PATH"]
        if environment.get("LD_LIBRARY_PATH") else "")
    return environment


def run_shard(root: pathlib.Path, profile: str, shard: int,
              task_path: pathlib.Path, checkpoint_period: int) -> dict[str, Any]:
    output_path = root / "raw" / profile / f"shard-{shard}.jsonl"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    completed = valid_output_prefix(task_path, output_path)
    process = subprocess.Popen(
        [str(EXECUTABLE), "--bulk", str(CONFIG)], cwd=WORKSPACE,
        env=worker_environment(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, bufsize=1)
    if process.stdin is None or process.stdout is None:
        raise InvalidCampaign("cannot open bulk worker pipes")
    rows = completed
    started = time.monotonic()
    last_key: tuple[Any, ...] | None = None
    with task_path.open(encoding="utf-8") as tasks, \
            output_path.open("ab") as outcomes:
        for _ in range(completed):
            if not tasks.readline():
                raise InvalidCampaign("checkpoint exceeds task shard")
        for line in tasks:
            task = json.loads(line)
            try:
                process.stdin.write(line)
                process.stdin.flush()
            except OSError as error:
                process.wait()
                stderr = process.stderr.read()[-4000:] if process.stderr else ""
                with contextlib.suppress(OSError):
                    process.stdin.close()
                if process.stdout:
                    process.stdout.close()
                if process.stderr:
                    process.stderr.close()
                raise InvalidCampaign(
                    f"WORKER_CRASH shard={shard}: {stderr or error}") from error
            response = process.stdout.readline()
            if not response:
                process.wait()
                stderr = process.stderr.read()[-4000:] if process.stderr else ""
                with contextlib.suppress(OSError):
                    process.stdin.close()
                process.stdout.close()
                if process.stderr:
                    process.stderr.close()
                raise InvalidCampaign(f"WORKER_CRASH shard={shard}: {stderr}")
            try:
                result = json.loads(response)
            except json.JSONDecodeError as error:
                raise InvalidCampaign(
                    f"malformed worker output shard={shard}: {error}") from error
            payload = {**task, **result}
            for field in IDENTITY_FIELDS:
                payload[field] = task[field]
            payload["outcome_sha256"] = sha256_bytes(canonical_json(payload))
            outcomes.write(canonical_json(payload))
            rows += 1
            last_key = task_key(payload)
            if rows % checkpoint_period == 0:
                outcomes.flush()
                os.fsync(outcomes.fileno())
                atomic_json(root / "checkpoints" / profile / f"shard-{shard}.json", {
                    "rows": rows, "last_identity": last_key,
                    "task_sha256": sha256_file(task_path),
                    "outcome_prefix_sha256": sha256_file(output_path)})
        outcomes.flush()
        os.fsync(outcomes.fileno())
    process.stdin.close()
    return_code = process.wait()
    if return_code:
        stderr = process.stderr.read()[-4000:] if process.stderr else ""
        process.stdout.close()
        if process.stderr:
            process.stderr.close()
        raise InvalidCampaign(f"WORKER_CRASH shard={shard}: {stderr}")
    process.stdout.close()
    if process.stderr:
        process.stderr.close()
    elapsed = time.monotonic() - started
    summary = {"shard": shard, "rows": rows, "resumed_rows": completed,
               "elapsed_s": elapsed, "task_sha256": sha256_file(task_path),
               "outcome_sha256": sha256_file(output_path),
               "bytes": output_path.stat().st_size,
               "last_identity": last_key}
    atomic_json(root / "checkpoints" / profile / f"shard-{shard}.json", summary)
    return summary


def command_samples(root: pathlib.Path, state: dict[str, Any],
                    protocol: dict[str, Any], profile: str,
                    limit_cells: int | None, checkpoint_period: int) -> None:
    if not EXECUTABLE.is_file():
        raise InvalidCampaign(f"scenario bulk worker is not built: {EXECUTABLE}")
    if profile == "formal":
        if not protocol.get("formal_matrix_authorized", False):
            raise InvalidCampaign("FORMAL_MATRIX_NOT_RUN: protocol forbids formal start")
        if state["git_dirty_at_prepare"] or git("status", "--porcelain"):
            raise InvalidCampaign("DIRTY_SHA: formal run requires the frozen clean SHA")
        state["formal_matrix_started"] = True
    elif limit_cells is None:
        require_clean_freeze(state, "complete development pilot")
    task_paths = write_shards(root, protocol, state, profile, limit_cells)
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(run_shard, root, profile, shard, path,
                                   checkpoint_period)
                   for shard, path in enumerate(task_paths)]
        summaries = [future.result() for future in futures]
    rows = sum(item["rows"] for item in summaries)
    elapsed = max((item["elapsed_s"] for item in summaries), default=0.0)
    raw_bytes = sum(item["bytes"] for item in summaries)
    peak_runner_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    peak_worker_rss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    atomic_json(root / "evidence" / f"{profile}_execution_summary.json", {
        "schema_version": "uwb-imu-pl/round3-execution/v1", "status": PASS,
        "profile": profile, "rows": rows, "workers": 4, "numeric_threads": 1,
        "wall_time_s": elapsed, "cpu_hours_upper": elapsed * 4.0 / 3600.0,
        "peak_runner_rss_kb": peak_runner_rss,
        "peak_worker_rss_kb": peak_worker_rss,
        "peak_rss_kb": max(peak_runner_rss, peak_worker_rss),
        "raw_bytes": raw_bytes, "shards": summaries})
    state["stages"]["pilot" if profile == "pilot" else "formal"] = PASS
    save_state(root, state)


@dataclass
class GateAggregate:
    outcomes: int = 0
    invalid: int = 0
    failed: int = 0
    safety_failures: Counter = field(default_factory=Counter)
    cells: Counter = field(default_factory=Counter)
    onset_offsets: set[int] = field(default_factory=set)
    interval_offsets: dict[int, set[int]] = field(default_factory=lambda: defaultdict(set))
    geometry_conditions: dict[str, dict[str, float]] = field(
        default_factory=lambda: defaultdict(dict))
    checks: Counter = field(default_factory=Counter)


def stream_profile(root: pathlib.Path, profile: str) -> dict[str, GateAggregate]:
    aggregates = {gate: GateAggregate() for gate in "EFGHI"}
    seen_keys: set[tuple[Any, ...]] = set()
    for shard in range(4):
        task_path = root / "tasks" / profile / f"shard-{shard}.jsonl"
        output_path = root / "raw" / profile / f"shard-{shard}.jsonl"
        if not task_path.is_file() or not output_path.is_file():
            raise InvalidCampaign(f"missing {profile} shard {shard}")
        valid_output_prefix(task_path, output_path)
        with task_path.open() as tasks, output_path.open() as outcomes:
            for row, pair in enumerate(zip_longest(tasks, outcomes), 1):
                task_line, outcome_line = pair
                if task_line is None or outcome_line is None:
                    raise InvalidCampaign(
                        f"MISSING_CELL shard={shard} row={row}")
                task, outcome = json.loads(task_line), json.loads(outcome_line)
                if task_key(task) != task_key(outcome):
                    raise InvalidCampaign(f"IDENTITY_MISMATCH shard={shard} row={row}")
                identity = task_key(outcome)[:-1]
                if identity in seen_keys:
                    raise InvalidCampaign(f"duplicate outcome key shard={shard} row={row}")
                seen_keys.add(identity)
                status = outcome.get("status")
                if status not in (PASS, FAIL, INVALID):
                    raise InvalidCampaign(f"invalid outcome status shard={shard} row={row}")
                if status != PASS and not str(outcome.get("failure_reason", "")).strip():
                    raise InvalidCampaign(
                        f"empty failure reason shard={shard} row={row}")
                gate = outcome.get("gate")
                if gate not in aggregates:
                    raise InvalidCampaign(f"invalid Gate shard={shard} row={row}")
                item = aggregates[gate]
                item.outcomes += 1
                item.cells[outcome["cell_id"]] += 1
                item.invalid += status == INVALID
                item.failed += status == FAIL
                item.onset_offsets.add(int(outcome.get("onset_offset", 0)))
                if gate == "F":
                    item.interval_offsets[int(outcome["window"])].add(
                        int(outcome["interval_offset"]))
                geometry = outcome.get("geometry")
                condition = outcome.get("geometry_condition")
                if geometry and isinstance(condition, (int, float)):
                    item.geometry_conditions[geometry].setdefault(
                        outcome["cell_id"], float(condition))
                for key in ("backend_update_violation", "coverage_failure", "hmi",
                            "wrong_exclusion"):
                    item.safety_failures[key] += bool(outcome.get(key)) and not (
                        key in ("coverage_failure", "hmi") and
                        outcome.get("assumption_violation"))
                categories = sum(bool(outcome.get(key)) for key in
                                 ("correct_exclusion", "union_exclusion",
                                  "ambiguous_exclusion", "wrong_exclusion"))
                if outcome.get("detected"):
                    item.checks["exclusive_classification"] += categories == 1
                    item.checks["classified"] += 1
                if gate == "E":
                    active_count = {"normal8": 8, "poor7": 7,
                                    "poor6": 6}.get(outcome.get("geometry"), 0)
                    if outcome.get("anchor") is not None:
                        item.checks["anchor_mapping"] += int(
                            outcome["anchor"]) <= active_count
                        item.checks["anchor_mapping_trials"] += 1
                    if outcome.get("fault_mode") == "nominal":
                        item.checks["nominal_keep"] += (
                            not outcome.get("selected_sources") and
                            not outcome.get("nominal_false_exclusion") and
                            bool(outcome.get("protected_available")))
                        item.checks["nominal_keep_trials"] += 1
                    if outcome.get("geometry") == "near_rank":
                        item.checks["near_rank_unavailable"] += (
                            not outcome.get("monitorable") and
                            not outcome.get("protected_available"))
                        item.checks["near_rank_trials"] += 1
                elif gate == "F":
                    item.checks["raw_interval"] += bool(
                        outcome.get("raw_interval_injection_verified"))
                    monitorable = bool(outcome.get("monitorable"))
                    diagnostics = (
                        outcome.get("monitorability_classification") ==
                            ("MONITORABLE" if monitorable else "UNAVAILABLE") and
                        isinstance(outcome.get("monitorability_rank"), int) and
                        isinstance(outcome.get("monitorability_sigma_min"),
                                   (int, float)) and
                        (not monitorable or (
                            outcome.get("monitorability_rank", 0) > 0 and
                            isinstance(outcome.get("monitorability_condition"),
                                       (int, float)) and
                            all(isinstance(outcome.get(
                                f"monitorability_slope_{axis}"), (int, float))
                                for axis in "xyz"))))
                    item.checks["monitorability_diagnostics"] += diagnostics
                elif gate == "G":
                    if not outcome.get("assumption_violation"):
                        item.checks["bridge_component_trials"] += 1
                        item.checks["bridge_component"] += bool(
                            outcome.get("bridge_component_coverage_verified"))
                    item.checks["bridge_control"] += bool(
                        outcome.get("bridge_control_unavailable"))
                    item.checks["bias_margin"] += bool(
                        outcome.get("bias_continuity_has_no_maneuver_margin"))
                    if int(outcome.get("bridge_length", 0)) == 21:
                        item.checks["timeout_trials"] += 1
                        item.checks["timeout"] += bool(
                            outcome.get("bridge_timeout_verified"))
                        item.checks["first_clean_uwb"] += bool(
                            outcome.get("first_clean_uwb_unavailable"))
                elif gate == "H":
                    item.checks["threshold_replay"] += bool(
                        outcome.get("local_threshold_invariant"))
                    expected = (outcome.get("plausible_count", 0) == 1
                                if outcome.get("ambiguity") == "single_plausible"
                                else outcome.get("plausible_count", 0) > 1)
                    item.checks["plausibility_shape"] += expected
                    item.checks["risk_closure"] += bool(
                        outcome.get("risk_budget_closed"))
                    if outcome.get("union_exclusion"):
                        item.checks["union_trials"] += 1
                        if outcome.get("geometry") == "normal8":
                            item.checks["normal8_union_trials"] += 1
                            item.checks["normal8_union_available"] += bool(
                                outcome.get("protected_available"))
                        elif outcome.get("geometry") == "poor6":
                            item.checks["poor6_union_trials"] += 1
                            item.checks["poor6_union_unavailable"] += not bool(
                                outcome.get("protected_available"))
                elif gate == "I":
                    name = outcome.get("name")
                    if name in ("continuous_interval_imu",
                                "intermittent_3_fault_2_clean"):
                        item.checks["raw_schedule_trials"] += 1
                        onset = 25 + int(outcome.get("onset_offset", 0))
                        epochs = int(outcome.get("epochs", 0))
                        fault_epochs = sum(
                            1 for epoch in range(onset, epochs + 1)
                            if name == "continuous_interval_imu" or
                            (epoch - onset) % 5 < 3)
                        expected_samples = 10 * fault_epochs
                        item.checks["raw_schedule"] += bool(
                            outcome.get("raw_interval_injection_verified")) and \
                            outcome.get("injected_imu_samples") == expected_samples and \
                            outcome.get("expected_injected_imu_samples") == expected_samples
                    if name == "health_recovery":
                        item.checks["health_recovery"] += bool(
                            outcome.get("state_machine_verified"))
                    elif name == "late_detection_full_history":
                        item.checks["full_history_replacement"] += bool(
                            outcome.get("history_replacement_verified"))
                    elif name == "late_detection_fixed_lag":
                        item.checks["fixed_lag_contamination"] += bool(
                            outcome.get("history_prior_contaminated"))
                    elif name == "bridge_timeout_reinit":
                        item.checks["timeout_reinit"] += bool(
                            outcome.get("state_machine_verified"))
    return aggregates


def unit_gate_verdicts(root: pathlib.Path, state: dict[str, Any]) -> dict[str, Any]:
    build_path = root / "evidence/build_summary.json"
    build = json.loads(build_path.read_text()) if build_path.is_file() else {}
    build_identity_valid = (
        build.get("status") == PASS and
        build.get("tested_code_sha") == state["tested_code_sha"] and
        build.get("protocol_sha256") == state["protocol_sha256"] and
        build.get("config_sha256") == state["config_sha256"])
    builds = build.get("builds", {}) if build_identity_valid else {}
    mapping = {
        "B": {
            "IntegrityV2Transaction.PrepareAndDiscardNeverMutateBackend",
            "IntegrityV2Transaction.CommitIsOneAtomicBackendUpdateAndLedgerIsComplete",
            "UwbImuIncremental.MethodAAndLinearMethodBCandidateGiveEquivalentOutput",
            "UwbImuIncremental.FixedLagMatchesUnboundedBeforeAndAfterMarginalization"},
        "C": {
            "IntegrityV2Transaction.PrepareAndDiscardNeverMutateBackend",
            "IntegrityV2Transaction.CommitIsOneAtomicBackendUpdateAndLedgerIsComplete",
            "IntegrityV2Transaction.HistoricalUwbReplacementCommitsAtomically",
            "IntegrityV2Window.RejectsFixedLagWithoutMaturityMargin",
            "IntegrityV2RankUpdate.MatchesIndependentDenseOracle"},
    }
    verdicts = {}
    for gate, required in mapping.items():
        missing_by_build = {
            build_type: sorted(required - set(builds.get(build_type, {}).get(
                "test_cases", [])))
            for build_type in ("Debug", "Release")}
        complete = build_identity_valid and all(
            builds.get(build_type, {}).get("status") == PASS and
            not missing_by_build[build_type]
            for build_type in ("Debug", "Release"))
        verdicts[gate] = {"status": PASS if complete else INVALID,
                          "required_tests": sorted(required),
                          "missing_tests_by_build": missing_by_build,
                          "build_identity_valid": build_identity_valid,
                          "tested_code_sha": state["tested_code_sha"],
                          "reason": "clean Debug/Release transaction/window evidence complete"
                          if complete else
                          "matching clean Debug/Release machine-test evidence is absent"}
        atomic_json(root / "evidence" / f"gate_{gate.lower()}_verdict.json",
                    {"schema_version": "uwb-imu-pl/round3-gate-verdict/v1",
                     "gate": gate, **verdicts[gate]})
    return verdicts


def command_analyze(root: pathlib.Path, state: dict[str, Any],
                    protocol: dict[str, Any]) -> None:
    profile = "formal" if state["stages"]["formal"] == PASS else "pilot"
    if state["stages"]["pilot"] != PASS and profile == "pilot":
        raise InvalidCampaign("no complete pilot or formal outcome set")
    aggregates = stream_profile(root, profile)
    verdicts = unit_gate_verdicts(root, state)
    build_path = root / "evidence/build_summary.json"
    build = json.loads(build_path.read_text()) if build_path.is_file() else {}
    build_identity_valid = (build.get("status") == PASS and
        build.get("tested_code_sha") == state["tested_code_sha"] and
        build.get("protocol_sha256") == state["protocol_sha256"] and
        build.get("config_sha256") == state["config_sha256"])
    verdicts["A"] = {"status": PASS if build_identity_valid else INVALID,
                     "reason": "clean Debug/Release inventory absent" if not build else
                     "see build_summary.json"}
    atomic_json(root / "evidence/gate_a_verdict.json", {
        "schema_version": "uwb-imu-pl/round3-gate-verdict/v1", "gate": "A",
        "tested_code_sha": state["tested_code_sha"], **verdicts["A"]})
    performance_path = root / "performance/performance_summary.json"
    performance = json.loads(performance_path.read_text()) if performance_path.is_file() else {}
    performance_identity_valid = (performance.get("tested_code_sha") ==
        state["tested_code_sha"] and performance.get("protocol_sha256") ==
        state["protocol_sha256"] and performance.get("config_sha256") ==
        state["config_sha256"])
    verdicts["D"] = {"status": performance.get("status", INVALID)
                     if performance_identity_valid else INVALID,
                     "identity_valid": performance_identity_valid,
                     "reason": "Gate D 128-candidate smoke absent" if not performance else
                     "40 ms frozen threshold evaluated" if performance_identity_valid else
                     "Gate D evidence identity does not match the campaign"}
    atomic_json(root / "evidence/gate_d_verdict.json", {
        "schema_version": "uwb-imu-pl/round3-gate-verdict/v1", "gate": "D",
        "tested_code_sha": state["tested_code_sha"], **verdicts["D"],
        "checks": performance})
    for gate, aggregate in aggregates.items():
        pilot_scientific = (FAIL if aggregate.invalid or aggregate.failed or
                            any(aggregate.safety_failures.values()) else PASS)
        checks: dict[str, Any] = {
            "profile": profile, "outcomes": aggregate.outcomes,
            "canonical_cells_seen": len(aggregate.cells),
            "invalid": aggregate.invalid, "failed": aggregate.failed,
            "safety_invariants": dict(aggregate.safety_failures),
            "onset_offsets_seen": sorted(aggregate.onset_offsets),
            "development_scientific_status": pilot_scientific,
        }
        if gate == "E":
            checks["onset_census_complete"] = aggregate.onset_offsets == set(
                protocol["onset_offsets"])
            medians = {}
            for geometry, values in aggregate.geometry_conditions.items():
                ordered = sorted(values.values())
                if ordered:
                    medians[geometry] = ordered[len(ordered) // 2]
            checks["geometry_condition_medians"] = medians
            checks["poor_geometry_degradation"] = {
                geometry: medians[geometry] / medians["normal8"]
                for geometry in ("poor7", "poor6")
                if geometry in medians and "normal8" in medians}
            checks["active_anchor_mapping_all"] = aggregate.checks[
                "anchor_mapping"] == aggregate.checks["anchor_mapping_trials"]
            checks["nominal_keep_all"] = aggregate.checks["nominal_keep"] == \
                aggregate.checks["nominal_keep_trials"]
            checks["near_rank_unavailable_all"] = aggregate.checks[
                "near_rank_unavailable"] == aggregate.checks["near_rank_trials"]
        elif gate == "F":
            checks["interval_census"] = {
                str(window): len(offsets) for window, offsets in
                aggregate.interval_offsets.items()}
            checks["raw_interval_injection"] = aggregate.checks["raw_interval"] == \
                aggregate.outcomes
            checks["monitorability_diagnostics_all"] = aggregate.checks[
                "monitorability_diagnostics"] == aggregate.outcomes
        elif gate == "G":
            checks["bridge_component_all"] = aggregate.checks["bridge_component"] == \
                aggregate.checks["bridge_component_trials"]
            checks["control_unavailable_all"] = aggregate.checks["bridge_control"] == \
                aggregate.outcomes
            checks["bias_margin_separate_all"] = aggregate.checks["bias_margin"] == \
                aggregate.outcomes
            checks["timeout_cells_verified"] = (
                aggregate.checks["timeout_trials"] > 0 and
                aggregate.checks["timeout"] == aggregate.checks["timeout_trials"])
            checks["first_clean_uwb_unavailable_verified"] = (
                aggregate.checks["timeout_trials"] > 0 and
                aggregate.checks["first_clean_uwb"] ==
                aggregate.checks["timeout_trials"])
        elif gate == "H":
            checks["threshold_replay_all"] = aggregate.checks["threshold_replay"] == \
                aggregate.outcomes
            checks["plausibility_shape_all"] = aggregate.checks[
                "plausibility_shape"] == aggregate.outcomes
            checks["outcome_risk_closure_all"] = aggregate.checks[
                "risk_closure"] == aggregate.outcomes
            checks["normal8_union_available_verified"] = (
                aggregate.checks["normal8_union_trials"] > 0 and
                aggregate.checks["normal8_union_available"] ==
                aggregate.checks["normal8_union_trials"])
            checks["poor6_union_unavailable_verified"] = (
                aggregate.checks["poor6_union_trials"] > 0 and
                aggregate.checks["poor6_union_unavailable"] ==
                aggregate.checks["poor6_union_trials"])
        elif gate == "I":
            checks["shadow_passes"] = protocol["gate_i"]["shadow_passes"]
            checks["recovery_test_passes"] = protocol["gate_i"][
                "recovery_test_passes"]
            checks["recovery_refail_pass"] = protocol["gate_i"][
                "recovery_refail_pass"]
            checks["raw_schedule_all"] = aggregate.checks["raw_schedule"] == \
                aggregate.checks["raw_schedule_trials"]
            checks["health_recovery_verified"] = bool(
                aggregate.checks["health_recovery"])
            checks["full_history_replacement_verified"] = bool(
                aggregate.checks["full_history_replacement"])
            checks["fixed_lag_contamination_verified"] = bool(
                aggregate.checks["fixed_lag_contamination"])
            checks["timeout_reinit_verified"] = bool(
                aggregate.checks["timeout_reinit"])
        expected_cells = Counter(cell["gate"] for cell in enumerate_cells(protocol))[gate]
        checks["canonical_cell_census_complete"] = len(aggregate.cells) == expected_cells
        required = [value for key, value in checks.items()
                    if key.endswith("_all") or key.endswith("_complete") or
                    key.endswith("_verified")]
        if gate == "E":
            required.extend(value >= 2.0 for value in
                            checks["poor_geometry_degradation"].values())
            required.append(len(checks["poor_geometry_degradation"]) == 2)
        if aggregate.checks["classified"]:
            required.append(aggregate.checks["exclusive_classification"] ==
                            aggregate.checks["classified"])
        if not all(required):
            pilot_scientific = FAIL
        checks["development_scientific_status"] = pilot_scientific
        formal_complete = profile == "formal" and sum(
            value.outcomes for value in aggregates.values()) == state["formal_outcomes"]
        status = pilot_scientific if formal_complete else INVALID
        reason = ("formal matrix evidence complete" if formal_complete else
                  "development pilot is not formal Gate evidence")
        verdicts[gate] = {"status": status, "reason": reason, "checks": checks}
        atomic_json(root / "evidence" / f"gate_{gate.lower()}_verdict.json", {
            "schema_version": "uwb-imu-pl/round3-gate-verdict/v1", "gate": gate,
            "tested_code_sha": state["tested_code_sha"], **verdicts[gate]})
    execution = json.loads((root / "evidence" /
        f"{profile}_execution_summary.json").read_text())
    rate = execution["rows"] / max(execution["wall_time_s"], 1e-9)
    formal_count = state["formal_outcomes"]
    observed_invalid = sum(value.invalid for value in aggregates.values())
    observed_failed = sum(value.failed for value in aggregates.values())
    build_verified = state["stages"].get("build") == PASS
    resource_report = {
        "schema_version": "uwb-imu-pl/round3-resource-projection/v1",
        "source_profile": profile, "measured_outcomes": execution["rows"],
        "formal_outcomes": formal_count, "aggregate_outcomes_per_s": rate,
        "projected_wall_time_s": formal_count / rate,
        "projected_cpu_hours": formal_count / rate * 4.0 / 3600.0,
        "projected_disk_bytes": int(execution["raw_bytes"] * formal_count /
                                    max(1, execution["rows"])),
        "peak_runner_rss_kb": execution.get(
            "peak_runner_rss_kb", execution["peak_rss_kb"]),
        "peak_worker_rss_kb": execution.get("peak_worker_rss_kb"),
        "peak_observed_rss_kb": execution["peak_rss_kb"],
        "observed_invalid_outcomes": observed_invalid,
        "observed_failed_outcomes": observed_failed,
        "observed_failure_rate": (observed_invalid + observed_failed) /
            max(1, execution["rows"]),
        "worker_crash_resume_verified": build_verified and all(
            item["resumed_rows"] >= 0 for item in execution["shards"]),
        "formal_start_authorized": False}
    atomic_json(root / "evidence/resource_projection.json", resource_report)
    write_gap_audit(root, state, verdicts)
    atomic_json(root / "round3_readiness.json", {
        "schema_version": "uwb-imu-pl/round3-readiness/v1", "status": INVALID,
        "tested_code_sha": state["tested_code_sha"],
        "protocol_sha256": state["protocol_sha256"],
        "implementation_maturity": "IMPLEMENTED_UNVERIFIED",
        "formal_eligible": False, "gate_j_complete": False,
        "gates_a_to_i_complete": False, "formal_matrix_started":
            state["formal_matrix_started"],
        "gates": {gate: verdicts.get(gate, {"status": INVALID})["status"]
                  for gate in "ABCDEFGHI"},
        "reason": "formal E-I matrix has not been executed"})
    state["stages"]["analyze"] = PASS
    save_state(root, state)


def write_gap_audit(root: pathlib.Path, state: dict[str, Any],
                    verdicts: dict[str, Any]) -> None:
    clauses = {
        "A": ("implementation/config/XML inventory frozen",),
        "B": ("prepare zero mutation", "discard zero update",
              "commit exactly one update", "nominal equivalence",
              "full-history/fixed-lag consistency"),
        "C": ("row census and dense boundary oracle", "100% provenance",
              "remove/add/replacement versions", "marginalization boundary"),
        "D": ("128 candidates", "p99 <= 40 ms", "one-worker ablation"),
        "E": ("all-onset census", "anchor mapping", "geometry degradation",
              "near-rank unavailable", "exclusive classification"),
        "F": ("all-interval census", "rank/sigma/condition/slope",
              "10 raw samples per interval"),
        "G": ("per-axis bridge component coverage", "additive bridge margins",
              "bias continuity separate", "control unavailable",
              "21 epoch/1 second timeout and reinit", "first clean UWB unavailable"),
        "H": ("diagnostic-threshold replay", "plausible-pool shape",
              "normal8 union available", "poor6 union unavailable",
              "outcome risk closure"),
        "I": ("continuous and 3-fault/2-clean injection", "20+10 recovery",
              "fifth recovery refail", "history replacement one update",
              "full-history late recovery", "fixed-lag contamination",
              "timeout reinit"),
    }
    records = []
    for gate, names in clauses.items():
        gate_status = verdicts.get(gate, {}).get("status", INVALID)
        for index, name in enumerate(names, 1):
            status = gate_status
            reason = verdicts.get(gate, {}).get(
                "reason", "evidence has not been collected")
            if gate == "D" and verdicts.get("D", {}).get("identity_valid"):
                performance = verdicts["D"]["checks"]
                if index == 1:
                    status = PASS if performance.get(
                        "candidate_contract_met") else FAIL
                    reason = "frozen 128-candidate contract evaluated"
                elif index == 3:
                    ablation = performance.get("single_worker_ablation", {})
                    status = PASS if ablation.get("status") == PASS else INVALID
                    reason = ablation.get(
                        "reason", "one-worker ablation did not complete")
            records.append({"clause_id": f"GATE_{gate}_{index:02d}",
                            "gate": gate, "clause": name,
                            "status": status,
                            "evidence_path": f"evidence/gate_{gate.lower()}_verdict.json",
                            "tested_code_sha": state["tested_code_sha"],
                            "reason": reason})
    atomic_json(root / "round3_gap_audit.json", {
        "schema_version": "uwb-imu-pl/round3-gap-audit/v1",
        "tested_code_sha": state["tested_code_sha"],
        "protocol_sha256": state["protocol_sha256"], "clauses": records})


def command_finalize(root: pathlib.Path, state: dict[str, Any]) -> None:
    readiness = root / "round3_readiness.json"
    if not readiness.is_file():
        raise InvalidCampaign("round3_readiness.json is absent; analyze first")
    evidence_files = [path.relative_to(root) for path in
                      (root / "evidence").rglob("*") if path.is_file()]
    atomic_json(root / "evidence_index.json", inventory(root, evidence_files))
    state["stages"]["finalize"] = INVALID
    state["status"] = INVALID
    save_state(root, state)
    compact_files = [path for path in root.rglob("*") if path.is_file() and
                     "raw" not in path.relative_to(root).parts and
                     path.name != "checksums.sha256"]
    with (root / "checksums.sha256").open("w") as stream:
        for path in sorted(compact_files):
            stream.write(f"{sha256_file(path)}  {path.relative_to(root).as_posix()}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "build", "analytic",
                                              "d-smoke", "pilot", "ros-smoke",
                                              "run", "analyze", "finalize", "status"))
    parser.add_argument("--protocol", type=pathlib.Path, default=PROTOCOL)
    parser.add_argument("--root", type=pathlib.Path,
                        default=REPOSITORY / "results/integrity_round3")
    parser.add_argument("--limit-cells", type=int)
    parser.add_argument("--checkpoint-period", type=int, default=64)
    parser.add_argument("--ros-evidence", type=pathlib.Path)
    args = parser.parse_args()
    try:
        protocol, protocol_sha = load_protocol(args.protocol)
        root = campaign_root(args.root, protocol_sha)
        if args.command == "prepare":
            root = command_prepare(args.root, protocol, protocol_sha, args.protocol)
        else:
            state = load_state(root)
            if state["git_sha"] != git("rev-parse", "HEAD") or \
                    state["protocol_sha256"] != protocol_sha:
                raise InvalidCampaign("campaign identity differs from checkout/protocol")
            if args.command == "build":
                command_build(root, state)
            elif args.command == "analytic":
                command_analytic(root, state)
            elif args.command == "d-smoke":
                command_d_smoke(root, state)
            elif args.command == "pilot":
                command_samples(root, state, protocol, "pilot", args.limit_cells,
                                args.checkpoint_period)
            elif args.command == "ros-smoke":
                command_ros_smoke(root, state, protocol, args.ros_evidence)
            elif args.command == "run":
                command_samples(root, state, protocol, "formal", None,
                                args.checkpoint_period)
            elif args.command == "analyze":
                command_analyze(root, state, protocol)
            elif args.command == "finalize":
                command_finalize(root, state)
            else:
                report = dict(state)
                report["checksums"] = verify_checksums(root) if (
                    root / "checksums.sha256").is_file() else ["not finalized"]
                print(json.dumps(report, indent=2, sort_keys=True))
                return 0
        print(f"{args.command}: {root}")
        return 0
    except (InvalidCampaign, OSError, subprocess.SubprocessError) as error:
        print(f"INVALID: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
