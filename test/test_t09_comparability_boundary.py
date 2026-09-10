#!/usr/bin/env python3
"""T09-R07: comparability corruption is infrastructure-invalid, not failure data."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
MOCK = r'''#!/usr/bin/env python3
import json, pathlib, sys
a=sys.argv[1:]
def v(k): return a[a.index(k)+1]
run=pathlib.Path(v("--output-root"))/v("--run-id"); run.mkdir()
method=v("--method")
common="t09common-sha256:"+("a" if method=="all_range" else "b")
(run/"common_preparation.json").write_text(json.dumps({"common_preparation_id":common}))
(run/"trajectory.tum").write_text("0 0 0 0 0 0 0 1\n")
(run/"run_status.json").write_text(json.dumps({"status":"OK","exit_code":0,"valid_estimate_exported":True}))
'''


def main():
    with tempfile.TemporaryDirectory(prefix="uifgo-t09-comparability-") as tmp:
        root = Path(tmp)
        runner = root / "runner.py"; runner.write_text(MOCK); runner.chmod(0o755)
        config = root / "input.yaml"
        config.write_text(yaml.safe_dump({"dataset": {"interface": "original"}}))
        manifest = root / "batch.yaml"
        manifest.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_batch_v2", "role": "development",
            "parameter_provenance": "T09_DEVELOPMENT_ENGINEERING_TEST_ONLY",
            "run_units": [{"run_unit_id": "u", "recording_id": "r",
                "base_trajectory_id": "b", "seed": 0,
                "prefix_identity": "full", "config": "input.yaml",
                "cells": [
                    {"mode": "all_range", "execution_type": "BASELINE_TRAJECTORY",
                     "path": "DIRECT_COMMON_PREPARATION"},
                    {"mode": "robust_huber", "execution_type": "BASELINE_TRAJECTORY",
                     "path": "DIRECT_COMMON_PREPARATION", "robust_scale": 1.0},
                ]}]}))
        output = root / "output"
        scheduled = subprocess.run([sys.executable, "-B",
            str(ROOT / "tools/paper/run_experiments.py"), "--manifest", str(manifest),
            "--runner", str(runner), "--output-root", str(output)],
            text=True, capture_output=True)
        assert scheduled.returncode == 2
        batch = json.loads((output / "batch_manifest.json").read_text())
        assert batch["status"] == "INVALID_COMPARABILITY"
        assert any(cell["status"] == "COMPARABILITY_INVALID" for cell in batch["cells"])
        assert (output / "batch_failure.json").is_file()

        evaluation = root / "evaluation.yaml"
        evaluation.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_evaluation_v1", "run_units": {}}))
        evaluated = subprocess.run([sys.executable, "-B",
            str(ROOT / "tools/paper/evaluate_runs.py"), "--batch-manifest",
            str(output / "batch_manifest.json"), "--evaluation-manifest",
            str(evaluation)], text=True, capture_output=True)
        assert evaluated.returncode == 2
        assert not (output / "evaluation.json").exists()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
