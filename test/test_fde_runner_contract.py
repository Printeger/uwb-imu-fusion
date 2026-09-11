#!/usr/bin/env python3
"""Provider-aware Stage-2 cache admission contracts for residual FDE."""

import importlib.util
import json
from pathlib import Path
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_experiments", ROOT / "tools/paper/run_experiments.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write(path, value):
    path.write_text(value, encoding="utf-8")


def make_run(root: Path, fde: bool) -> Path:
    run = root / ("fde" if fde else "legacy")
    run.mkdir()
    write(run / "run_status.json", json.dumps({"exit_code": 0, "status": "OK"}))
    write(run / "capability_status.json", json.dumps({
        "discovery": "NO_CANDIDATES", "recoverability_score": "COMPLETE"}))
    provider = "imu_aided_residual_fde_v1" if fde else "automatic_discovery"
    write(run / "input_manifest.json", json.dumps({
        "source_hash_sha256": "sha256:source", "stage1_provider": provider}))
    write(run / "stage2_producer_context.json", json.dumps({
        "schema": "uifgo_t09_stage2_producer_context_v1",
        "stage1_config_sha256": "sha256:stage1-" + provider,
        "stage2_refit_config_sha256": "sha256:stage2",
        "stage3_score_config_sha256": "sha256:stage3"}))
    write(run / "stage2_content_identity.json", json.dumps({
        "graph_linearization_sha256": "sha256:graph",
        "values_sha256": "sha256:values"}))
    for name in MODULE.STAGE2_REPLAY_REQUIRED_PAYLOADS | {
            "trajectory.tum", "imu_bias.csv"}:
        path = run / name
        if path.exists():
            continue
        if name in {"groups.csv", "scores_decision.csv"}:
            write(path, "group_id\n")
        else:
            write(path, name + "\n")
    partition = json.dumps({"provider": provider}, sort_keys=True) + "\n"
    write(run / "partition.json", partition)
    if fde:
        write(run / "support_partition.json", partition)
        write(run / "fde_observations.csv", "obs_id,nlos_candidate\n")
        write(run / "fde_status.json", json.dumps({
            "provider": provider, "status": "SUCCESS", "gt_read": False}))
    return run


def publish(run: Path, cache: Path):
    return MODULE.publish_cache(
        run, cache, "AUTO_DISCOVERY", "common", "common-config",
        "stage2-config", False, {
            "producer_commit": "commit",
            "producer_binary_sha256": "binary",
            "producer_abi_sha256": "abi",
            "producer_toolchain_sha256": "toolchain"})


def main():
    with tempfile.TemporaryDirectory(prefix="uifgo-fde-contract-") as tmp:
        root = Path(tmp)
        fde = make_run(root, True)
        fde_manifest = publish(fde, root / "cache-fde")
        payloads = {item["name"] for item in fde_manifest["payloads"]}
        assert {"fde_status.json", "fde_observations.csv",
                "support_partition.json"}.issubset(payloads)
        assert fde_manifest["stage1_provider"] == "imu_aided_residual_fde_v1"
        assert not any("discovery_iteration" in name or "admm" in name or
                       "outer_trace" in name for name in payloads)

        legacy = make_run(root, False)
        legacy_manifest = publish(legacy, root / "cache-legacy")
        assert legacy_manifest["stage1_provider"] == "automatic_discovery"
        assert legacy_manifest["cache_id"] != fde_manifest["cache_id"]

        incomplete_root = root / "incomplete-run"
        incomplete_root.mkdir()
        incomplete = make_run(incomplete_root, True)
        (incomplete / "fde_observations.csv").unlink()
        try:
            publish(incomplete, root / "cache-incomplete")
        except RuntimeError as error:
            assert "incomplete" in str(error)
        else:
            raise AssertionError("incomplete FDE cache was admitted")
    print("FDE runner/cache contracts: PASS")


if __name__ == "__main__":
    main()
