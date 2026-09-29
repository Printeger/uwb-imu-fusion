#!/usr/bin/env python3
"""Small process-isolated leaf examples; expanded to all 174 after review."""

import argparse
import contextlib
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml


VALID_CLASSIFICATIONS = {"DERIVED", "FIXED", "OBSERVED"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def execute(command, expect=0):
    # Small dense oracle matrices are faster and more reproducible without a
    # machine-dependent BLAS thread fan-out.  This changes scheduling only;
    # every one of the 7 x 504 proofs is still computed and compared.
    environment = os.environ.copy()
    environment.update({"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1",
                        "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1"})
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, env=environment)
    if result.returncode != expect:
        raise RuntimeError("command status mismatch\n" + " ".join(command) +
                           "\n" + result.stdout)
    return result.stdout


def bound_capture(actual, raw, truth, config, manifest, output):
    metadata = (
        f"raw_sha256\t{digest(raw)}\n"
        f"truth_sha256\t{digest(truth)}\n"
        f"config_sha256\t{digest(config)}\n"
        f"manifest_sha256\t{digest(manifest)}\n")
    Path(output).write_text(metadata + Path(actual).read_text())


def run_derived(probe, source, tmp, name, mutate):
    replay = source / "docs/evidence/p0-07-corrected-exhaustive/authoritative-replay"
    evidence = replay.parent
    raw = json.loads((replay / "frozen-input.json").read_text())
    truth = json.loads((replay / "frozen-truth.json").read_text())
    config = yaml.safe_load((source / "config/fde_uwb_order1.yaml").read_text())
    config["fault_models"]["manifest_path"] = str(
        source / "config/integrity_fault_manifest.yaml")
    manifest = json.loads((evidence / "oracle-manifest-v1.json").read_text())
    mutate(raw, truth, config, manifest)
    raw_path, truth_path = tmp / f"{name}-raw.json", tmp / f"{name}-truth.json"
    config_path, manifest_path = tmp / f"{name}-config.yaml", tmp / f"{name}-manifest.json"
    raw_path.write_text(json.dumps(raw, sort_keys=True)); truth_path.write_text(json.dumps(truth, sort_keys=True))
    config_path.write_text(yaml.safe_dump(config, sort_keys=False)); manifest_path.write_text(json.dumps(manifest, sort_keys=True))
    actual, capture, oracle = tmp / f"{name}-actual.tsv", tmp / f"{name}-capture.tsv", tmp / f"{name}-oracle.tsv"
    execute([str(probe), str(config_path), str(raw_path), str(truth_path), str(actual)])
    bound_capture(actual, raw_path, truth_path, config_path, manifest_path, capture)
    execute([sys.executable, str(source / "tools/p0_07_offline_oracle.py"),
             "--capture", str(capture), "--raw", str(raw_path), "--truth", str(truth_path),
             "--config", str(config_path), "--manifest", str(manifest_path),
             "--canonical-manifest", str(evidence / "oracle-manifest-v1.json"),
             "--output", str(oracle)])
    execute([sys.executable, str(source / "tools/p0_07_compare_protocols.py"),
             "--oracle", str(oracle), "--actual", str(actual),
             "--skip-observed"])
    print(f"LEAF_DERIVED_PASS\t{name}")


def mutate_scalar(value):
    if isinstance(value, bool): return not value
    if isinstance(value, int): return value + 1
    if isinstance(value, float): return value * 2.0 if value else 1.0e-12
    if isinstance(value, str): return value + "__MUTATED"
    if isinstance(value, list) and not value: return ["__MUTATED"]
    raise TypeError(type(value))


def path_segments(path):
    result = []
    for part in path.split("."):
        if "[" in part:
            name, index = part[:-1].split("[")
            result.extend((name, int(index)))
        else:
            result.append(part)
    return result


def mutate_manifest_path(manifest, path):
    segments = path_segments(path); node = manifest
    for segment in segments[:-1]: node = node[segment]
    node[segments[-1]] = mutate_scalar(node[segments[-1]])


def audit_case_tables(source):
    """Prove that the declared cases and audited FIXED sources are exact.

    This check intentionally runs before any mutation.  It derives the legal
    leaf set from the canonical manifest and the DERIVED set from process A;
    neither set is inferred from the TSV under audit.
    """
    import p0_07_offline_oracle as offline

    evidence = source / "docs/evidence/p0-07-corrected-exhaustive"
    manifest = json.loads((evidence / "oracle-manifest-v1.json").read_text())
    legal = set(offline.flatten_leaves(manifest))
    observed = {path for path in legal
                if path.startswith("observed_non_authoritative.")}
    derived = set(offline.DERIVED_INPUT_PATHS)
    fixed = legal - derived - observed

    with (source / "test/p0_07_leaf_cases_v1.tsv").open() as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    paths = [row["path"] for row in rows]
    if len(rows) != 174 or len(set(paths)) != 174 or set(paths) != legal:
        raise RuntimeError(
            f"leaf table is not an exact 174-leaf partition: "
            f"rows={len(rows)} unique={len(set(paths))} "
            f"missing={sorted(legal - set(paths))} extra={sorted(set(paths) - legal)}")
    if any(row["classification"] not in VALID_CLASSIFICATIONS for row in rows):
        raise RuntimeError("leaf table contains an unknown classification")
    classified = {
        kind: {row["path"] for row in rows if row["classification"] == kind}
        for kind in VALID_CLASSIFICATIONS
    }
    for kind, expected in (("DERIVED", derived), ("FIXED", fixed),
                           ("OBSERVED", observed)):
        if classified[kind] != expected:
            raise RuntimeError(
                f"{kind} leaf set mismatch: missing={sorted(expected - classified[kind])} "
                f"extra={sorted(classified[kind] - expected)}")

    with (evidence / "fixed-contract-map.tsv").open() as stream:
        map_rows = list(csv.DictReader(stream, delimiter="\t"))
    mapped = [row["path"] for row in map_rows]
    if len(mapped) != 153 or len(set(mapped)) != 153 or set(mapped) != fixed:
        raise RuntimeError(
            f"FIXED source map mismatch: rows={len(mapped)} unique={len(set(mapped))} "
            f"missing={sorted(fixed - set(mapped))} extra={sorted(set(mapped) - fixed)}")
    for row in map_rows:
        expected_source = offline.fixed_contract_source(row["path"])
        if row["production_source_line_symbol_contract"] != expected_source:
            raise RuntimeError(f"stale FIXED source mapping: {row['path']}")
        file_name, line_text, symbol_contract = expected_source.split(":", 2)
        target = source / file_name
        if not target.is_file():
            raise RuntimeError(f"FIXED source does not exist: {file_name}")
        lines = target.read_text().splitlines()
        symbol = symbol_contract.split("/", 1)[0]
        symbol_token = symbol.rsplit("::", 1)[-1]
        if target.suffix == ".json":
            pattern = re.compile(rf'^\s*"{re.escape(symbol_token)}"\s*:')
        elif target.suffix == ".py":
            pattern = re.compile(rf'^def\s+{re.escape(symbol_token)}\s*\(')
        elif "::" in symbol:
            pattern = re.compile(re.escape(symbol) + r"\s*\(")
        elif symbol_token == "main":
            pattern = re.compile(r"\bint\s+main\s*\(")
        elif symbol_token == "buildRiskLedger":
            pattern = re.compile(r"\bRiskLedger\s+buildRiskLedger\s*\(")
        else:
            raise RuntimeError(f"FIXED source has no definition matcher: {expected_source}")
        occurrences = [index + 1 for index, text in enumerate(lines)
                       if pattern.search(text)]
        overload_markers = {
            "IntegrityConfigLoader::load": "IntegrityConfigOverrides&",
            "JointWindowDetector::evaluate": "FrozenWindowAdmission&",
            "FdeManager::decide": "FdeDecisionContextV1*",
            "buildRiskLedger": "CompleteRiskInputsV1&",
        }
        marker = overload_markers.get(symbol)
        if marker:
            matching_definitions = []
            for line in occurrences:
                declaration = []
                for text in lines[line - 1:line + 12]:
                    declaration.append(text)
                    if "{" in text:
                        break
                if marker in "\n".join(declaration):
                    matching_definitions.append(line)
            occurrences = matching_definitions
        if len(occurrences) != 1:
            raise RuntimeError(
                f"FIXED source symbol is absent or ambiguous: {expected_source}; "
                f"actual_lines={occurrences}")
        # The canonical P0 map is immutable and therefore retains its reviewed
        # historical line coordinate.  P1 validates the stable symbol in the
        # same file instead of weakening coverage when unrelated edits move it.
        int(line_text)
    print("LEAF_CASE_AUDIT_PASS\ttotal=174\tDERIVED=13\tFIXED=153\tOBSERVED=8")


def derived_mutation(path):
    def mutate(raw, truth, config, manifest):
        if path == "replay.epochs":
            raw["epochs"] = 23
            raw["uwb_rows"] = [r for r in raw["uwb_rows"] if int(r["epoch"]) <= 23]
            manifest["replay"]["epochs"] = 23
        elif path in {"replay.alarm_epoch", "factor_construction.state_columns.last_epoch"}:
            truth["fault"]["epoch"] = 24
            bias = float(truth["fault"]["bias_m"])
            fault_anchor = int(truth["fault"]["anchor_id"])
            for row in raw["uwb_rows"]:
                if int(row["anchor_id"]) != fault_anchor: continue
                if int(row["epoch"]) == 23: row["range_m"] -= bias
                if int(row["epoch"]) == 24: row["range_m"] += bias
            manifest["replay"]["alarm_epoch"] = 24
            manifest["factor_construction"]["state_columns"]["last_epoch"] = 24
        elif path == "replay.epoch_period_ns":
            raw["imu"]["period_ns"] = 4000000
            for row in raw["uwb_rows"]:
                row["timestamp_ns"] = int(row["epoch"]) * 40000000
            manifest["replay"]["epoch_period_ns"] = 40000000
        elif path == "replay.anchors[7]":
            for row in raw["uwb_rows"]:
                if int(row["anchor_id"]) == 8: row["anchor_id"] = 9
            for anchor in config["anchors"]:
                if int(anchor["id"]) == 8: anchor["id"] = 9
            manifest["replay"]["anchors"][7] = 9
        elif path == "replay.imu_samples_per_epoch":
            raw["imu"]["samples_per_epoch"] = 8
            raw["imu"]["period_ns"] = 6250000
            manifest["replay"]["imu_samples_per_epoch"] = 8
        elif path == "detector_and_action_recipe.detector.p_fa_per_test":
            config["detector"]["p_fa_per_test"] = 2.0e-6
            manifest["detector_and_action_recipe"]["detector"]["p_fa_per_test"] = 2.0e-6
        elif path.startswith("detector_and_action_recipe.risk."):
            key = path.rsplit(".", 1)[1]
            config_key = "p_hmi_total" if key == "total_decimal" else key
            old = config["risk"][config_key]
            if key == "nominal_axis_tail":
                config["risk"][config_key] = old * 0.5
            else:
                config["risk"][config_key] = mutate_scalar(old)
            value = config["risk"][config_key]
            manifest["detector_and_action_recipe"]["risk"][key] = (
                format(value, ".36f") if key == "total_decimal" else value)
        else:
            raise KeyError("no real input mutation: " + path)
    return mutate


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--probe", required=True); parser.add_argument("--source", required=True)
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--path")
    parser.add_argument("--keep", action="store_true")
    parser.add_argument("--fixed-only", action="store_true")
    parser.add_argument("--observed-only", action="store_true")
    args = parser.parse_args(); source = Path(args.source).resolve(); probe = Path(args.probe).resolve()
    audit_case_tables(source)
    temporary = (contextlib.nullcontext(tempfile.mkdtemp(prefix="p007-leaf-sample-"))
                 if args.keep else tempfile.TemporaryDirectory(prefix="p007-leaf-sample-"))
    with temporary as directory:
        tmp = Path(directory)
        if args.keep: print("LEAF_DEBUG_DIR\t" + str(tmp))
        derived = ([] if args.fixed_only or args.observed_only else [args.path] if args.path else
                   ["replay.epochs",
                    "detector_and_action_recipe.detector.p_fa_per_test"]
                   if not args.full else sorted(__import__(
                       "p0_07_offline_oracle").DERIVED_INPUT_PATHS))
        for path in derived:
            run_derived(probe, source, tmp, path, derived_mutation(path))

        # FIXED example: A parses the mutation and rejects through its own
        # typed invariant.  No reason string is read from the case TSV.
        evidence = source / "docs/evidence/p0-07-corrected-exhaustive"
        fixed_paths = ([] if args.path or args.observed_only else
                       ["factor_construction.history_raw.slot_rules.imu_multiplier"])
        if (args.full or args.fixed_only) and not args.observed_only:
            with (source / "test/p0_07_leaf_cases_v1.tsv").open() as stream:
                fixed_paths = [row["path"] for row in csv.DictReader(stream, delimiter="\t")
                               if row["classification"] == "FIXED"]
        for ordinal, fixed_path in enumerate(fixed_paths):
            manifest = json.loads((evidence / "oracle-manifest-v1.json").read_text())
            mutate_manifest_path(manifest, fixed_path)
            manifest_path = tmp / f"fixed-{ordinal}.json"; manifest_path.write_text(json.dumps(manifest, sort_keys=True))
            capture = tmp / f"fixed-{ordinal}-capture.tsv"
            base_capture = evidence / "process-oracle/frozen-raw-factors-v1.tsv"
            text = base_capture.read_text().splitlines()
            text[4] = "manifest_sha256\t" + digest(manifest_path)
            capture.write_text("\n".join(text) + "\n")
            output = execute([sys.executable, str(source / "tools/p0_07_offline_oracle.py"),
                "--capture", str(capture), "--raw", str(evidence / "authoritative-replay/frozen-input.json"),
                "--truth", str(evidence / "authoritative-replay/frozen-truth.json"),
                "--config", str(source / "config/fde_uwb_order1.yaml"), "--manifest", str(manifest_path),
                "--canonical-manifest", str(evidence / "oracle-manifest-v1.json"),
                "--output", str(tmp / f"fixed-{ordinal}.tsv")], expect=1)
            required = ["FIXED_TYPED_INVARIANT:" + fixed_path,
                        "FIXED_TYPED_TYPE:" + fixed_path]
            if fixed_path == "fault_contract.double_fault_families":
                required.append("FIXED_TYPED_SCHEMA_LEAF_SET:")
            if not any(code in output for code in required):
                raise RuntimeError("wrong FIXED error code: " + output)
            print("LEAF_FIXED_PASS\t" + fixed_path)

        if (args.full or args.observed_only) and not args.fixed_only:
            replay = evidence / "authoritative-replay"
            base_actual = tmp / "observed-base-actual.tsv"
            execute([str(probe), str(source / "config/fde_uwb_order1.yaml"),
                     str(replay / "frozen-input.json"),
                     str(replay / "frozen-truth.json"), str(base_actual)])
            base_oracle = tmp / "observed-base-oracle.tsv"
            execute([sys.executable, str(source / "tools/p0_07_offline_oracle.py"),
                     "--capture", str(evidence / "process-oracle/frozen-raw-factors-v1.tsv"),
                     "--raw", str(replay / "frozen-input.json"),
                     "--truth", str(replay / "frozen-truth.json"),
                     "--config", str(source / "config/fde_uwb_order1.yaml"),
                     "--manifest", str(evidence / "oracle-manifest-v1.json"),
                     "--canonical-manifest", str(evidence / "oracle-manifest-v1.json"),
                     "--output", str(base_oracle)])
            with (source / "test/p0_07_leaf_cases_v1.tsv").open() as stream:
                observed_paths = [row["path"] for row in csv.DictReader(stream, delimiter="\t")
                                  if row["classification"] == "OBSERVED"]
            for ordinal, observed_path in enumerate(observed_paths):
                manifest = json.loads((evidence / "oracle-manifest-v1.json").read_text())
                mutate_manifest_path(manifest, observed_path)
                manifest_path = tmp / f"observed-{ordinal}.json"
                manifest_path.write_text(json.dumps(manifest, sort_keys=True))
                mutated_oracle = tmp / f"observed-{ordinal}.tsv"
                execute([sys.executable, str(source / "tools/p0_07_offline_oracle.py"),
                         "--capture", str(evidence / "process-oracle/frozen-raw-factors-v1.tsv"),
                         "--raw", str(replay / "frozen-input.json"),
                         "--truth", str(replay / "frozen-truth.json"),
                         "--config", str(source / "config/fde_uwb_order1.yaml"),
                         "--manifest", str(manifest_path),
                         "--canonical-manifest", str(evidence / "oracle-manifest-v1.json"),
                         "--base-oracle", str(base_oracle),
                         "--output", str(mutated_oracle)])
                mismatch = execute([sys.executable, str(source / "tools/p0_07_compare_protocols.py"),
                                    "--oracle", str(mutated_oracle),
                                    "--actual", str(base_actual)], expect=1)
                if "same-run observed metadata mismatch" not in mismatch:
                    raise RuntimeError("wrong OBSERVED rejection: " + mismatch)
                print("LEAF_OBSERVED_PASS\t" + observed_path)


if __name__ == "__main__":
    main()
