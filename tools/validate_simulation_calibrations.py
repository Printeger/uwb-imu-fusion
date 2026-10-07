#!/usr/bin/env python3
"""Validate synthetic calibration identities without opening production Gate J."""

import argparse
import hashlib
import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PROFILES = (
    "off", "uwb_order1", "imu_order1", "joint_order1", "joint_order2")
EXPECTED_FIELDS = (
    "artifact_class", "profile", "runtime_config.path",
    "runtime_config.sha256", "fault_manifest.path", "fault_manifest.sha256",
    "validation_protocol.path", "validation_protocol.sha256",
    "simulation_model.version", "simulation_model.source.path",
    "simulation_model.source.sha256", "profile_generator.version",
    "profile_generator.source.path", "profile_generator.source.sha256",
    "validation_runner.version", "validation_runner.source.path",
    "validation_runner.source.sha256", "acceptance_summarizer.version",
    "acceptance_summarizer.source.path",
    "acceptance_summarizer.source.sha256")

EXPECTED_SHARED_REFERENCES = {
    "simulation_model": {
        "version": "realtime-performance-benchmark/synthetic-model-v2-scheduled",
        "source": {"path": "apps/realtime_performance_benchmark.cpp"},
    },
    "profile_generator": {
        "version": "fde-profile-generator/v1",
        "source": {"path": "tools/generate_fde_profile_configs.py"},
    },
    "validation_runner": {
        "version": "fde-profile-validation-runner/v2-revised-38x20",
        "source": {"path": "tools/run_fde_profile_validation.py"},
    },
    "acceptance_summarizer": {
        "version": "p1-06-simulation-acceptance/v4-revised-38x20",
        "source": {"path": "tools/summarize_p1_06_simulation_acceptance.py"},
    },
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fail(message: str) -> None:
    raise ValueError(message)


def binding_payload(manifest: dict, profile: dict) -> dict:
    return {
        "artifact_class": manifest["artifact_class"],
        "profile": profile["profile"],
        "runtime_config": profile["runtime_config"],
        "fault_manifest": profile["fault_manifest"],
        "validation_protocol": profile["validation_protocol"],
        "simulation_model": manifest["simulation_model"],
        "profile_generator": manifest["profile_generator"],
        "validation_runner": manifest["validation_runner"],
        "acceptance_summarizer": manifest["acceptance_summarizer"],
    }


def validate(manifest_path: Path, root: Path = ROOT) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "uwb-imu-pl/simulation-calibration-manifest/v1":
        fail("unsupported synthetic calibration schema")
    if manifest.get("artifact_class") != "simulation/synthetic":
        fail("artifact_class must be exactly simulation/synthetic")
    contract = manifest.get("digest_contract", {})
    if (contract.get("algorithm") != "sha256" or
            contract.get("encoding") != "canonical-json-sort-keys-compact-utf8-v1" or
            tuple(contract.get("fields", ())) != EXPECTED_FIELDS):
        fail("unexpected digest contract")
    boundary = manifest.get("qualification_boundary", {})
    expected_boundary = {
        "simulation_acceptance_eligible": True,
        "hardware_formal_qualification": "CLOSED/NOT_CLAIMED",
        "production_formal_eligible_expected": False,
        "production_protected_output_expected": False,
        "may_populate_production_calibration_ids": False,
        "may_be_described_as_physical_calibration_or_hardware_certification": False,
    }
    if boundary != expected_boundary:
        fail("synthetic/hardware qualification boundary changed")
    for key, expected in EXPECTED_SHARED_REFERENCES.items():
        reference = manifest.get(key, {})
        if reference.get("version") != expected["version"]:
            fail(f"unexpected {key} version")
        source = reference.get("source", {})
        expected_path = expected["source"]["path"]
        if source.get("path") != expected_path:
            fail(f"unexpected {key} source path")
        if source.get("sha256") != sha256_file(root / expected_path):
            fail(f"stale {key} source sha256")
    profiles = manifest.get("profiles")
    if not isinstance(profiles, list) or tuple(
            item.get("profile") for item in profiles) != EXPECTED_PROFILES:
        fail("manifest must bind each of the five profiles exactly once in order")

    checked = []
    for item in profiles:
        profile = item["profile"]
        expected_paths = {
            "runtime_config": f"config/fde_{profile}.yaml",
            "fault_manifest": "config/integrity_fault_manifest.yaml",
            "validation_protocol": "config/fde_profiles_validation.yaml",
        }
        for key, expected_path in expected_paths.items():
            reference = item.get(key, {})
            if reference.get("path") != expected_path:
                fail(f"{profile}: unexpected {key} path")
            actual = sha256_file(root / expected_path)
            if reference.get("sha256") != actual:
                fail(f"{profile}: stale {key} sha256")
        payload = json.dumps(binding_payload(manifest, item), sort_keys=True,
                             separators=(",", ":")).encode("utf-8")
        complete_digest = hashlib.sha256(payload).hexdigest()
        if item.get("complete_model_config_digest") != complete_digest:
            fail(f"{profile}: complete model/config digest mismatch")
        expected_id = f"simcal-p1-06-v2-{profile}-{complete_digest[:16]}"
        if item.get("calibration_id") != expected_id:
            fail(f"{profile}: calibration ID is not bound to complete digest")
        checked.append({"profile": profile, "calibration_id": expected_id,
                        "complete_model_config_digest": complete_digest,
                        "runtime_config_sha256":
                            item["runtime_config"]["sha256"],
                        "validation_protocol_sha256":
                            item["validation_protocol"]["sha256"]})

    # The simulation IDs live only in this sidecar.  Production qualification
    # inputs remain absent, so loading this artifact cannot open Gate J.
    for profile in EXPECTED_PROFILES:
        text = (root / f"config/fde_{profile}.yaml").read_text(encoding="utf-8")
        for pattern in (
                r"noise_overbound_calibration_id:\s*['\"]?['\"]?\s*$",
                r"(?m)^\s{4}calibration_id:\s*['\"]?['\"]?\s*$",
                r"(?m)^\s{4}model_calibration_id:\s*['\"]?['\"]?\s*$",
                r"(?m)^\s{2}calibration_id:\s*['\"]?['\"]?\s*$"):
            if not re.search(pattern, text, flags=re.MULTILINE):
                fail(f"{profile}: production calibration input is no longer empty")

    return {"schema": manifest["schema"], "status": "PASS",
            "artifact_class": manifest["artifact_class"],
            "simulation_acceptance_eligible": True,
            "hardware_formal_qualification": "CLOSED/NOT_CLAIMED",
            "profiles": checked}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / "config/p1_06_simulation_calibrations.json")
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    print(json.dumps(validate(args.manifest.resolve(), args.root.resolve()),
                     indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
