#!/usr/bin/env python3
"""Static dependency guards for the Gate-05 source split."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    entry = (ROOT / "tools/run_ie_paper.cpp").read_text(encoding="utf-8")
    app = (ROOT / "tools/paper/run_ie_app.cpp").read_text(encoding="utf-8")
    estimator_header = (ROOT / "include/uifgo/estimator_core.h").read_text(
        encoding="utf-8")
    estimator_source = (ROOT / "src/estimator_core.cpp").read_text(
        encoding="utf-8")
    common_header = (ROOT / "include/uifgo/common_initializer.h").read_text(
        encoding="utf-8")
    common_source = (ROOT / "src/common_initializer.cpp").read_text(
        encoding="utf-8")
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert len(entry.splitlines()) <= 10
    assert '#include "paper/run_ie_app.h"' in entry
    assert "RunIePaperApplication(argc, argv)" in entry
    assert "ConfigLoader" not in entry
    assert "GraphBuilder" not in entry
    assert "FinalInferenceEngine" not in entry

    assert "PrepareEstimatorCore(cfg, imu, raw_uwb, recording_id)" in app
    assert "RunPaperBaseline" in app
    assert "FinalInferenceEngine" in app
    assert "WriteInferenceArtifacts" in app

    combined_estimator = estimator_header + estimator_source
    for forbidden in (
            "T04", "T06", "T08", "T11", "A02", "A19", "R04", "R08",
            "oracle_debug", "scientific_lock", "tools/", "experiments/"):
        assert forbidden not in combined_estimator, forbidden
    assert "BuildPaperInputPlan" in estimator_source
    assert "Initializer(config).Run" in estimator_source
    assert "GraphBuilder builder" in estimator_source
    assert "ReplacePosePriorsForPaperPath" in estimator_source
    assert "CommonInitializer(config).Run" in estimator_source

    combined_common = common_header + common_source
    for forbidden in ("tools/", "experiments/", "ground_truth",
                      "ate_evaluator", "oracle"):
        assert forbidden not in combined_common, forbidden
    assert "CombinedImuFactor" in common_source
    assert "Huber::Create(kInitializationHuberScale)" in common_source
    assert "RunCheckedConditionalLm" in common_source

    assert "src/estimator_core.cpp" in cmake
    assert "src/common_initializer.cpp" in cmake
    runner_block = cmake.split(
        "add_executable(uwb_imu_fgo_paper_runner", 1)[1].split(")", 1)[0]
    assert "tools/run_ie_paper.cpp" in runner_block
    assert "tools/paper/run_ie_app.cpp" in runner_block
    print("Gate-05 architecture dependency guards passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
