#!/usr/bin/env python3
"""Post-process locked detector outputs without changing detector policy."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess


START = 1664959678.3077347
END = 1664959686.3077347
BASELINE = "cf287b4fec9bc5689317f302ccf4cdd927bfba81"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    result = {
        "schema": "uifgo_windowed_fde_walk1_correctness_audit_v1",
        "policy_changed_after_result": False,
        "detector_parameters_changed_after_result": [],
        "conditions": {},
    }
    for condition in ("clean", "injected"):
        run = (args.root / "detector_screens" / condition /
               f"walk1_{condition}_detector_v4")
        with (run / "fde_local_windows.csv").open(newline="") as stream:
            windows = list(csv.DictReader(stream))
        with (run / "fde_observations.csv").open(newline="") as stream:
            observations = list(csv.DictReader(stream))
        def ratio(row):
            return float(row["statistic"]) / float(row["adjusted_threshold"])
        global_max = max(windows, key=ratio)
        target_windows = [row for row in windows
                          if row["tag_id"] == "27956" and
                          row["anchor_id"] == "20276" and
                          float(row["end_time"]) >= START and
                          float(row["start_time"]) <= END]
        target_max = max(target_windows, key=ratio)
        target_observations = [row for row in observations
                               if row["valid"] == "1" and row["planned"] == "1" and
                               row["tag_id"] == "27956" and
                               row["anchor_id"] == "20276" and
                               START <= float(row["sensor_time"]) <= END]
        residuals = [float(row["residual_m"]) for row in target_observations]
        fields = ("window_id", "tag_id", "anchor_id", "start_time", "end_time",
                  "count", "m", "rank", "statistic", "adjusted_threshold",
                  "gls_signed_residual_m", "adjusted_rejected", "significant", "status")
        result["conditions"][condition] = {
            "window_count": len(windows),
            "global_max_statistic_to_adjusted_threshold": {
                **{name: global_max[name] for name in fields}, "ratio": ratio(global_max)},
            "target_overlap_max_statistic_to_adjusted_threshold": {
                **{name: target_max[name] for name in fields}, "ratio": ratio(target_max)},
            "target_planned_observation_count": len(target_observations),
            "target_residual_m": {"minimum": min(residuals),
                                  "maximum": max(residuals),
                                  "mean": sum(residuals) / len(residuals)},
        }
    (args.root / "correctness_audit.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    repository = Path(__file__).resolve().parents[2]
    frozen = ["src/nlos_refit.cpp", "include/uifgo/nlos_refit.h",
              "src/nlos_recoverability.cpp", "include/uifgo/nlos_recoverability.h"]
    documents = ["doc/v2/paper_structure.tex", "doc/v2/v2_roadmap.md"]
    freeze = {
        "schema": "uifgo_windowed_fde_final_freeze_audit_v1",
        "baseline": BASELINE,
        "head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip(),
        "git_status_short": subprocess.check_output(
            ["git", "status", "--short"], cwd=repository, text=True).splitlines(),
        "protected_doc_v2_ie_0911_present": (repository / "doc/v2/ie_0911").is_dir(),
        "frozen_backend": {}, "frozen_documents": {},
    }
    for name in frozen:
        diff = subprocess.run(["git", "diff", "--exit-code", BASELINE, "--", name],
                              cwd=repository, capture_output=True, text=True)
        freeze["frozen_backend"][name] = {
            "diff_exit_code": diff.returncode,
            "sha256": "sha256:" + hashlib.sha256(
                (repository / name).read_bytes()).hexdigest(),
        }
    for name in documents:
        freeze["frozen_documents"][name] = "sha256:" + hashlib.sha256(
            (repository / name).read_bytes()).hexdigest()
    freeze["pass"] = (freeze["head"] == BASELINE and
                      all(item["diff_exit_code"] == 0
                          for item in freeze["frozen_backend"].values()))
    (args.root / "final_freeze_audit.json").write_text(
        json.dumps(freeze, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
