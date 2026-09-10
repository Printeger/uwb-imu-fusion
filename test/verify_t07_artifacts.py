#!/usr/bin/env python3
"""Verify generated T07 component truth against estimator-only cache rows."""

import argparse
import csv
import hashlib
import json
import os


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def load(cache_manifest, truth_manifest):
    with open(cache_manifest, encoding="utf-8") as stream:
        cache_meta = json.load(stream)
    with open(truth_manifest, encoding="utf-8") as stream:
        truth_meta = json.load(stream)
    assert truth_meta["truth_semantics"] == "INJECTED_COMPONENT_ONLY"
    assert truth_meta["total_latent_bias_status"] == "UNKNOWN"
    assert truth_meta["cache_id"] == cache_meta["cache_id"]
    for forbidden in ("recipe", "truth", "support"):
        assert all(forbidden not in key.lower() for key in cache_meta)
    cache_dir, truth_dir = os.path.dirname(cache_manifest), os.path.dirname(truth_manifest)
    uwb_path = os.path.join(cache_dir, cache_meta["uwb_file"])
    truth_path = os.path.join(truth_dir, truth_meta["truth_file"])
    assert sha256(uwb_path) == cache_meta["uwb_sha256"]
    assert sha256(truth_path) == truth_meta["truth_sha256"]
    with open(uwb_path, newline="", encoding="utf-8") as stream:
        uwb = {int(row["obs_id"]): row for row in csv.DictReader(stream)}
    with open(truth_path, newline="", encoding="utf-8") as stream:
        truth = {int(row["obs_id"]): row for row in csv.DictReader(stream)}
    assert uwb.keys() == truth.keys()
    event_counts = {}
    for obs_id, row in truth.items():
        base = float(row["base_range_m"])
        final = float(row["observed_range_m"])
        delta = float(row["injected_bias_delta_m"])
        assert abs((base + delta) - final) <= 1e-12
        assert abs(float(uwb[obs_id]["observed_range_m"]) - final) <= 1e-12
        assert int(uwb[obs_id]["source_message_index"]) == int(row["source_message_index"])
        assert int(uwb[obs_id]["source_range_index"]) == int(row["source_range_index"])
        event = bytes.fromhex(row["event_id_hex"]).decode("utf-8")
        if event:
            event_counts[event] = event_counts.get(event, 0) + 1
        else:
            assert delta == 0.0 and row["shape"] == "none"
    assert sorted(event_counts.values()) == [77, 79]
    assert sorted(truth_meta["event_match_counts"]) == [77, 79]
    return cache_meta, uwb, truth, event_counts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step-cache", required=True)
    parser.add_argument("--step-truth", required=True)
    parser.add_argument("--ramp-cache", required=True)
    parser.add_argument("--ramp-truth", required=True)
    args = parser.parse_args()
    step = load(args.step_cache, args.step_truth)
    ramp = load(args.ramp_cache, args.ramp_truth)
    assert step[1].keys() == ramp[1].keys(), "stable obs_id changed across scenarios"
    for obs_id in step[1]:
        assert step[1][obs_id]["source_message_index"] == ramp[1][obs_id]["source_message_index"]
        assert step[1][obs_id]["source_range_index"] == ramp[1][obs_id]["source_range_index"]
    print(json.dumps({
        "schema": "t07_artifact_verification_v1",
        "step_cache_id": step[0]["cache_id"],
        "ramp_cache_id": ramp[0]["cache_id"],
        "observation_count": len(step[1]),
        "stable_obs_ids_across_scenarios": True,
        "conservation": "PASS_BASE_PLUS_INJECTED_COMPONENT_EQUALS_OBSERVED",
        "step_event_counts": step[3],
        "ramp_event_counts": ramp[3],
        "total_latent_bias_status": "UNKNOWN",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
