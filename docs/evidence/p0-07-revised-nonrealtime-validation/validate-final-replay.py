#!/usr/bin/env python3
"""Mechanical guard for the final P0-07 replay authority."""

from __future__ import annotations

import hashlib
import json
import pathlib
import re
import sys


ROOT = pathlib.Path(__file__).resolve().parent
AUTHORITY = ROOT / "replay" / "current"
EXPECTED_DSO = "982348acefce41b124fe90cb48f73de6bbcd1cb3ebc3a17b0ca912f005314be3"
EXPECTED_BINARY = "ed8c5e5889e60059db1ae0ddb3a82ec1efaf00f527ef64824c09256597f951da"
EXPECTED_NORMAL = "2095395924847106241"
EXPECTED_EXCEPTION = "14520030921107668164"
FORBIDDEN = ("7175377120889463918", "2369560496406214301")


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fields(path: pathlib.Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line:
            continue
        key, value = line.split("=", 1)
        result[key] = value
    return result


def main() -> int:
    pointer = json.loads((AUTHORITY / "current.json").read_text(encoding="utf-8"))
    digest = pointer["bundle_sha256"]
    bundle = AUTHORITY / pointer["relative_path"]
    if pointer["schema"] != "p0-07-bundle-pointer-v1" or bundle.name != digest:
        raise RuntimeError("malformed current replay pointer")
    current_bundles = [entry for entry in (AUTHORITY / "bundles").iterdir()
                       if entry.is_dir()]
    if current_bundles != [bundle]:
        raise RuntimeError("current authority contains a non-current/pre-final bundle")
    manifest_path = bundle / "bundle-hashes.json"
    if sha256(manifest_path) != digest:
        raise RuntimeError("bundle identity does not match manifest")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_inventory = set(manifest) | {"bundle-hashes.json"}
    if {path.name for path in bundle.iterdir() if path.is_file()} != expected_inventory:
        raise RuntimeError("bundle inventory differs from manifest")
    for name, expected in manifest.items():
        if sha256(bundle / name) != expected:
            raise RuntimeError(f"bundle hash mismatch: {name}")
    output = json.loads((bundle / "frozen-output.json").read_text(encoding="utf-8"))
    alarm = output["alarm"]
    if alarm["terminal_semantic_digest"] != EXPECTED_NORMAL:
        raise RuntimeError("normal terminal digest is not golden-p1-05 equivalent")
    if alarm["exception_semantic_digest"] != EXPECTED_EXCEPTION:
        raise RuntimeError("exception terminal digest is not golden-p1-05 equivalent")
    current_text = "\n".join(path.read_text(encoding="utf-8") for path in bundle.iterdir()
                              if path.is_file())
    if any(value in current_text for value in FORBIDDEN):
        raise RuntimeError("pre-final terminal digest leaked into current authority")
    command = fields(bundle / "evidence-command.txt")
    if command.get("exit") != "0":
        raise RuntimeError("authoritative generation command did not exit zero")
    record = fields(ROOT / "replay-refresh-command.txt")
    if record.get("exit") != "0" or record.get("bundle_sha256") != digest:
        raise RuntimeError("refresh record does not bind the current successful bundle")
    if record.get("dso_sha256") != EXPECTED_DSO or \
            record.get("test_binary_sha256") != EXPECTED_BINARY:
        raise RuntimeError("refresh record does not bind the final DSO/test binary")
    if record.get("terminal_semantic_digest") != EXPECTED_NORMAL or \
            record.get("exception_semantic_digest") != EXPECTED_EXCEPTION:
        raise RuntimeError("refresh record digest mismatch")
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[+-]\d{2}:\d{2}",
                        record.get("generated_at", "")):
        raise RuntimeError("refresh record has no versioned timezone-aware timestamp")
    print(f"P007_FINAL_REPLAY_PASS bundle={digest} dso={EXPECTED_DSO} "
          f"normal={EXPECTED_NORMAL} exception={EXPECTED_EXCEPTION}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
