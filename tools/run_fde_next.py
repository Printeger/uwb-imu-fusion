#!/usr/bin/env python3
"""One research entry: build/check, synthetic run, scoped audit/bench, raw replay.
Only temporary CSV/matrix/log files are created; optional figures are explicit.
No protected publisher, deployment qualification or GT calibration is invoked.
"""
import argparse
from collections import Counter
import csv
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
BUILD = WORKSPACE / "build/uwb_imu_pl"
BIN = WORKSPACE / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl"


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def read_matrices(path):
    matrices, metadata = {}, []
    with path.open() as stream:
        for line in stream:
            if line.startswith("#"):
                metadata.append(line)
                continue
            key, rows, cols = line.split()
            shape = int(rows), int(cols)
            content = next(stream).strip()
            values = np.fromstring(content, sep=" ") if content else np.empty(0)
            if values.size != shape[0] * shape[1]:
                raise RuntimeError(f"malformed numeric audit: {key}")
            matrices[key] = values.reshape(shape)
    return matrices, metadata


def compare_rows(a, b):
    excluded = {"core_ms", "commit_ms", "cycle_ms", "end_to_end_ms", "cleanup_ms", "gaussian_decompositions", "source_products", "covariance_fallback"}
    for key in a.keys() - excluded:
        try:
            av, bv = float(a[key]), float(b[key])
        except ValueError:
            if a[key] != b[key]:
                raise RuntimeError(f"different result semantics: {key}")
            continue
        if not np.isclose(av, bv, rtol=1e-10, atol=1e-10, equal_nan=True):
            raise RuntimeError(f"different result: {key}: {av} vs {bv}")


def compare(left, right, left_dump, right_dump):
    compare_rows(read_rows(left)[0], read_rows(right)[0])
    max_delta = 0.0
    aa, am = read_matrices(left_dump)
    bb, bm = read_matrices(right_dump)
    if am != bm or aa.keys() != bb.keys():
        raise RuntimeError("different fault/consumer audit census")
    for key in aa:
        if aa[key].shape != bb[key].shape:
            raise RuntimeError(f"different matrix shape: {key}")
        delta = float(np.max(np.abs(aa[key] - bb[key]))) if aa[key].size else 0.0
        if key.endswith("reference_pl"):
            if np.any(bb[key] < aa[key] - 1e-12) or delta > 1e-8:
                raise RuntimeError(f"nonconservative or excessive PL guard: {key}")
        elif not np.allclose(aa[key], bb[key], rtol=1e-11, atol=1e-11, equal_nan=True):
            raise RuntimeError(f"different numeric payload: {key}")
        max_delta = max(max_delta, delta)
    return len(aa), max_delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["check", "run", "history", "recovery", "audit", "bench", "stream", "replay"])
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--config", type=Path, default=ROOT / "config/fde_next.yaml")
    parser.add_argument("--scene", default="U0")
    parser.add_argument("--scope", default="uwb")
    parser.add_argument("--seed", type=int, default=20261010)
    parser.add_argument("--epochs", type=int)
    parser.add_argument("--initial-law", choices=["configured", "gaussian"], default="configured")
    parser.add_argument("--numeric", choices=["reference", "optimized"], default="optimized")
    parser.add_argument("--position-functional", action="store_true", help="IMU-only recovery; current tag conditional, navigation unprotected")
    parser.add_argument("--history-reference", action="store_true", help="use the finite raw prefix reference for actual selection")
    parser.add_argument("--cache", type=Path)
    parser.add_argument("--plot", type=Path)
    args = parser.parse_args()
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = str(BIN.parent) + ":" + env.get("LD_LIBRARY_PATH", "")
    for name in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]:
        env[name] = "1"
    env.pop("FDE_NEXT_NUMERIC_AUDIT", None)
    if args.build:
        subprocess.run(["cmake", "-S", str(ROOT), "-B", str(BUILD)], cwd=ROOT, env=env, check=True)
        subprocess.run(["cmake", "--build", str(BUILD), "--target", "fde_next_runner", "fde_next_checks", "-j2"], env=env, check=True)
    with tempfile.TemporaryDirectory(prefix="fde-next-") as temp:
        tmp = Path(temp)
        if args.initial_law == "gaussian":
            config = tmp / "input-law.yaml"
            config.write_text(args.config.read_text() + "\ninitial_observation_law: independent-tangent-gaussian-v1\n")
            args.config = config
        if args.history_reference or args.command == "stream":
            env["FDE_NEXT_USE_PREFIX_REFERENCE"] = "1"
        if args.command == "history":
            env["FDE_NEXT_HISTORY_AUDIT"] = "1"
            env["FDE_NEXT_RAW_PREFIX_HISTORY"] = "1"
        if args.command == "recovery":
            env["FDE_NEXT_BOUNDED_RECOVERY"] = "1"
        if args.position_functional:
            env["FDE_NEXT_IMU_POSITION_FUNCTIONAL"] = "1"

        def run(name, scene, scope, epochs, numeric, attempts=None, audit=False, seed=None, continuous=False):
            path = tmp / (name + ".csv")
            cmd = [str(BIN / "fde_next_runner"), str(args.config), str(path), scene, scope,
                   str(args.seed if seed is None else seed), str(epochs), numeric]
            if attempts is not None:
                cmd.append(str(attempts))
            if continuous:
                cmd.extend(["1", "continuous"])
            current = dict(env)
            if audit:
                current["FDE_NEXT_NUMERIC_AUDIT"] = str(tmp / (name + ".matrices"))
            before = time.perf_counter()
            result = subprocess.run(cmd, cwd=ROOT, env=current, text=True, capture_output=True)
            process_seconds = time.perf_counter() - before
            (tmp / (name + ".log")).write_text(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
            return path, process_seconds

        if args.command == "check":
            subprocess.run([str(BIN / "fde_next_checks"), str(args.config)], cwd=ROOT, env=env, check=True)
        elif args.command in ["run", "history", "recovery"]:
            scene = "I2" if args.command == "recovery" else args.scene
            scope = "imu-diagnostic" if args.command == "recovery" else args.scope
            path, wall = run("run", scene, scope, args.epochs or (32 if args.command == "history" else 50), args.numeric)
            if args.command in ["history", "recovery"] or args.initial_law == "gaussian":
                for line in (tmp / "run.log").read_text().splitlines():
                    if line.startswith(("HISTORY", "RAW_PREFIX_HISTORY", "RAW_DOMAIN", "G_TAG_CONTRACTION", "BOUNDED_RECOVERY", "IMU_POSITION_FUNCTIONAL")):
                        print(line)
            rows = read_rows(path)
            print(f"{len(rows)} outputs; selected={sum(x['selected']=='1' for x in rows)}; "
                  f"exclusions={sum(x['exclusion']=='1' for x in rows)}; "
                  f"formal={sum(x['formal_eligible']=='1' for x in rows)}; "
                  f"protected={sum(x['publication_protected']=='1' for x in rows)}; process={wall:.3f}s")
            print("statuses:", dict(Counter(x["status"] for x in rows)))
            last = rows[-1]
            print("last output:", {key: last[key] for key in
                  ["model", "scope", "timestamp_ns", "protection_point", "commit_x", "commit_y", "commit_z",
                   "hpl", "vpl", "frozen_union_charge", "raw_total_risk", "raw_qualified", "future_state_status"]})
            errors = np.array([float(x["error"]) for x in rows])
            print(f"synthetic tag APE RMSE/P95/max m: {np.sqrt(np.mean(errors**2)):.6f}/"
                  f"{np.percentile(errors,95):.6f}/{np.max(errors):.6f}; "
                  f"local bound available={sum(x['final_local_available']=='1' for x in rows)}/{len(rows)}")
        elif args.command == "replay":
            if args.cache is None:
                parser.error("replay requires --cache with imu.csv and uwb_observations.csv")
            path, wall = run("replay", "replay", "off", args.epochs or 20, "reference", seed=args.cache)
            rows = read_rows(path)
            print(f"raw cache fragment: outputs={len(rows)}, backend_updates={sum(int(x['updates']) for x in rows)}, "
                  f"status={set(x['status'] for x in rows)}, process={wall:.3f}s; no GT accuracy/fault labels")
        elif args.command == "audit":
            for name, scene, epochs in [("normal", "U0", 14), ("alarm", "U1", 15)]:
                ref, _ = run(name + "-reference", scene, "uwb", epochs, "reference", 1, True)
                opt, _ = run(name + "-optimized", scene, "uwb", epochs, "optimized", 1, True)
                count, delta = compare(ref, opt, ref.with_suffix(".matrices"), opt.with_suffix(".matrices"))
                print(f"{name}: complete fields/recipes/fault maps/means/covariances/PL agree; "
                      f"matrices={count}, max_abs_delta={delta:.17g}")
        elif args.command == "stream":
            for scene in ["U0", "U1"]:
                reference_rows = None
                for numeric in ["reference", "optimized"]:
                    path, wall = run(scene + "-" + numeric, scene, "uwb", 48, numeric, continuous=True)
                    rows = read_rows(path)
                    for line in path.with_suffix(".log").read_text().splitlines():
                        if line.startswith("STREAM_PHASES"):
                            print(line, flush=True)
                    if len(rows) != 35 or [int(x["epoch"]) for x in rows] != list(range(14,49)):
                        raise RuntimeError("stream must measure35 consecutive epochs14–48")
                    if reference_rows is None:
                        reference_rows = rows
                    else:
                        for left, right in zip(reference_rows, rows):
                            compare_rows(left, right)
                        print(f"STREAM_PAIR {scene} all35 result semantics agree", flush=True)
                    values = lambda key: np.array([float(x[key]) for x in rows])
                    core, full = values("core_ms"), values("end_to_end_ms")
                    wait = np.zeros(35)
                    for k in range(1,35):
                        wait[k] = max(0., wait[k-1]+full[k-1]-50.)
                    stats = lambda x: "/".join(f"{v:.3f}" for v in (np.mean(x), np.median(x), np.percentile(x,95), np.max(x)))
                    print(f"STREAM {scene} {numeric} core={stats(core)} full={stats(full)} commit_median={np.median(values('commit_ms')):.3f} cleanup_median={np.median(values('cleanup_ms')):.3f} "
                          f"selected={sum(x['selected']=='1' for x in rows)}/35 updates={sum(int(x['updates']) for x in rows)} "
                          f"core40={sum(core>40)}/35 full50={sum(full>50)}/35 modeled_queue_max={max(wait):.3f} modeled_latency_max={max(wait+full):.3f} "
                          f"process_s={wall:.3f} setup_prewarm_teardown_s={wall-sum(full)/1000:.3f}", flush=True)
                    print("STREAM_ACTIONS", [(x["epoch"],x["selected_action"],x["removed_groups"],x["correct"]) for x in rows if x["alarm"]=="1"], flush=True)
                    print("STREAM_COUNTS", scene, numeric, {key: (min(values(key)), max(values(key))) for key in
                          ["rows", "cols", "noise_source_columns", "reference_faults", "actions", "post", "pl_evaluated", "integrations", "gaussian_decompositions", "source_products", "covariance_fallback"]},
                          f"rejected={sum(x['selected']!='1' for x in rows)} dropped=0 unfinished=0 queue_drained_after_last_arrival_ms={wait[-1]+full[-1]:.3f}", flush=True)
        else:
            results = {}
            print("| Workload | Mode | Core mean/median/P95/max ms | Full mean/median/P95/max ms | "
                  "Commit median | Cleanup median | Success/reject | Core>40 / full>50 | Decompositions | Process s |")
            print("|---|---|---|---|---:|---:|---|---|---:|---:|")
            for name, scene, epochs in [("P-N", "U0", 14), ("P-U", "U1", 15)]:
                for numeric in ["reference", "optimized"]:
                    path, wall = run(name + "-" + numeric, scene, "uwb", epochs, numeric, 35)
                    rows = read_rows(path)
                    for line in path.with_suffix(".log").read_text().splitlines():
                        if line.startswith("STREAM_PHASES"):
                            print(line, flush=True)
                    if len(rows) != 35 or any(x["selected"] != "1" or x["updates"] != "1" for x in rows):
                        raise RuntimeError("benchmark is not an effective committed workload")
                    if scene == "U1" and any(x["correct"] != "1" for x in rows):
                        raise RuntimeError("benchmark did not perform correct UWB recovery")
                    values = lambda key: np.array([float(x[key]) for x in rows])
                    core, full = values("core_ms"), values("end_to_end_ms")
                    stats = lambda x: "/".join(f"{v:.3f}" for v in (np.mean(x), np.median(x), np.percentile(x, 95), np.max(x)))
                    results[(name, numeric)] = (core, full)
                    print(f"| {name} | {numeric} | {stats(core)} | {stats(full)} | "
                          f"{np.median(values('commit_ms')):.3f} | {np.median(values('cleanup_ms')):.3f} | "
                          f"35/0 | {np.sum(core>40)}/35 / {np.sum(full>50)}/35 | "
                          f"{int(values('gaussian_decompositions')[0])} | {wall:.3f} |", flush=True)
                    first = rows[0]
                    print("CENSUS", name, numeric, {key: first[key] for key in
                          ["rows", "cols", "hypotheses", "actions", "numerical", "post", "pl_evaluated", "proof_entries", "selected_action", "removed_groups", "hpl", "vpl", "frozen_union_charge", "raw_total_risk"]}, flush=True)
            if args.plot:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, axes = plt.subplots(1, 2, figsize=(9, 3.5))
                for ax, index, title in zip(axes, [0, 1], ["core compute", "full cycle incl. cleanup"]):
                    labels, data = zip(*[(f"{name}\n{mode}", results[(name, mode)][index]) for name in ["P-N", "P-U"] for mode in ["reference", "optimized"]])
                    ax.boxplot(data, labels=labels, showmeans=True)
                    ax.set(title=title, ylabel="ms")
                    ax.grid(axis="y", alpha=.2)
                fig.suptitle("35 repeated mature snapshots each; actual commits; raw protection UNQUALIFIED")
                fig.tight_layout()
                fig.savefig(args.plot, dpi=140)
                plt.close(fig)


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        raise SystemExit(str(error))
