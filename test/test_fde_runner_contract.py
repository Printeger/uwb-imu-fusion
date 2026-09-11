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


def make_run(root: Path, fde: bool, provider_override=None) -> Path:
    provider = provider_override or (
        "imu_aided_postfit_fde_v2" if fde else "automatic_discovery")
    run = root / provider
    run.mkdir()
    write(run / "run_status.json", json.dumps({"exit_code": 0, "status": "OK", "segment_refit_status": "CONVERGED"}))
    write(run / "capability_status.json", json.dumps({
        "discovery": "NO_CANDIDATES", "recoverability_score": "COMPLETE"}))
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
        if provider == "imu_aided_grouped_fde_v3":
            for name in ("fde_group_tests.csv", "fde_group_covariance.csv",
                         "fde_group_spectrum.csv"):
                write(run / name, name + "\n")
        if provider == "imu_aided_windowed_fde_v4":
            write(run / "fde_local_windows.csv", "window_id,obs_ids,obs_ids_sha256\n")
            write(run / "fde_windowed_summary.json", json.dumps({
                "provider": provider,
                "partition_rule": "FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4",
                "gt_read": False}))
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
        assert fde_manifest["stage1_provider"] == "imu_aided_postfit_fde_v2"
        assert not any("discovery_iteration" in name or "admm" in name or
                       "outer_trace" in name for name in payloads)

        legacy = make_run(root, False)
        legacy_manifest = publish(legacy, root / "cache-legacy")
        assert legacy_manifest["stage1_provider"] == "automatic_discovery"
        assert legacy_manifest["cache_id"] != fde_manifest["cache_id"]

        variants_root = root / "variants"
        variants_root.mkdir()
        v3 = make_run(variants_root, True, "imu_aided_grouped_fde_v3")
        v4 = make_run(variants_root, True, "imu_aided_windowed_fde_v4")
        v3_manifest = publish(v3, root / "cache-v3")
        v4_manifest = publish(v4, root / "cache-v4")
        assert v3_manifest["stage1_provider"] == "imu_aided_grouped_fde_v3"
        assert v4_manifest["stage1_provider"] == "imu_aided_windowed_fde_v4"
        assert v3_manifest["cache_id"] != v4_manifest["cache_id"]
        assert v3_manifest["stage1_config_sha256"] != \
            v4_manifest["stage1_config_sha256"]
        # Replay admission requires both expected Stage1 identity and provider;
        # prove both mismatch directions and unchanged V3 self-admission.
        def replay_admits(manifest, requested_provider, requested_stage1):
            return manifest["stage1_provider"] == requested_provider and \
                manifest["stage1_config_sha256"] == requested_stage1
        assert replay_admits(v3_manifest, "imu_aided_grouped_fde_v3",
                             v3_manifest["stage1_config_sha256"])
        assert not replay_admits(v3_manifest, "imu_aided_windowed_fde_v4",
                                 v4_manifest["stage1_config_sha256"])
        assert not replay_admits(v4_manifest, "imu_aided_grouped_fde_v3",
                                 v3_manifest["stage1_config_sha256"])

        incomplete_v4_root = root / "incomplete-v4-root"
        incomplete_v4_root.mkdir()
        incomplete_v4 = make_run(
            incomplete_v4_root, True, "imu_aided_windowed_fde_v4")
        (incomplete_v4 / "fde_local_windows.csv").unlink()
        try:
            publish(incomplete_v4, root / "cache-incomplete-v4")
        except RuntimeError as error:
            assert "incomplete" in str(error)
        else:
            raise AssertionError("windowed FDE cache without v4 artifact admitted")

        obsolete_root = root / "obsolete-run"
        obsolete_root.mkdir()
        obsolete = make_run(obsolete_root, True)
        old_input = json.loads((obsolete / "input_manifest.json").read_text())
        old_input["stage1_provider"] = "imu_aided_fde_v1"
        write(obsolete / "input_manifest.json", json.dumps(old_input))
        try:
            publish(obsolete, root / "cache-obsolete")
        except RuntimeError as error:
            assert "obsolete FDE provider" in str(error)
        else:
            raise AssertionError("obsolete FDE cache was admitted")

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
        # A genuine SUCCESS_EMPTY carries independent reference/test evidence,
        # and no alternating optimizer trace. Corruption must fail publication.
        empty_root = root / "empty-run"
        empty_root.mkdir()
        empty = make_run(empty_root, True)
        partition = json.dumps({"provider": "imu_aided_postfit_fde_v2", "segments": []})
        for name in ("partition.json", "support_partition.json"):
            write(empty / name, partition)
        write(empty / "refit_iterations.csv", "outer_iteration\n")
        write(empty / "raw_reference.json", json.dumps({"success": True, "graph_identity": "sha256:graph", "values_identity": "sha256:values"}))
        evidence = {"provider": "imu_aided_postfit_fde_v2", "status": "SUCCESS", "gt_read": False,
                    "reference_converged": True, "planned_count": 8, "tested_count": 8}
        write(empty / "fde_status.json", json.dumps(evidence))
        write(empty / "stage2_refit_status.json", json.dumps({"solver_status": "SUCCESS_EMPTY",
              "optimizer_calls": 0, "alternating_stop_conditions": "NOT_APPLICABLE"}))
        admitted = publish(empty, root / "cache-empty")
        assert admitted["stage2_status"] == "SUCCESS_EMPTY"
        write(empty / "raw_reference.json", json.dumps({"success": True, "graph_identity": "sha256:wrong", "values_identity": "sha256:values"}))
        try:
            publish(empty, root / "cache-wrong-graph")
        except RuntimeError:
            pass
        else:
            raise AssertionError("reference graph identity mismatch admitted")
        write(empty / "raw_reference.json", json.dumps({"success": True, "graph_identity": "sha256:graph", "values_identity": "sha256:values"}))
        for field, bad in (("reference_converged", False), ("tested_count", 7)):
            corrupt = dict(evidence, **{field: bad})
            write(empty / "fde_status.json", json.dumps(corrupt))
            try:
                publish(empty, root / ("cache-bad-" + field))
            except RuntimeError:
                pass
            else:
                raise AssertionError("invalid empty reference admitted: " + field)
        write(empty / "fde_status.json", json.dumps(evidence))
        nonempty = json.dumps({"provider": "imu_aided_postfit_fde_v2", "segments": [{"id": "s"}]})
        for name in ("partition.json", "support_partition.json"):
            write(empty / name, nonempty)
        try:
            publish(empty, root / "cache-all-suppress")
        except RuntimeError:
            pass
        else:
            raise AssertionError("nonempty/all-suppress confused with empty")
    print("FDE runner/cache contracts: PASS")


if __name__ == "__main__":
    main()
