#!/usr/bin/env python3
"""Exercise every crash-safe evidence-authority commit boundary."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import importlib.util
import json
import pathlib
import subprocess
import tempfile


PRECOMMIT_FAILURES = (
    "validation", "write", "flush", "pre_pointer", "pointer_rename",
)


def pointer_bytes(root: pathlib.Path) -> bytes:
    return (root / "current.json").read_bytes()


def selected_digest(root: pathlib.Path) -> dict[str, str]:
    pointer = json.loads(pointer_bytes(root))
    bundle = root / pointer["relative_path"]
    manifest = json.loads((bundle / "bundle-hashes.json").read_text())
    observed = {
        name: hashlib.sha256((bundle / name).read_bytes()).hexdigest()
        for name in manifest
    }
    if observed != manifest:
        raise RuntimeError("selected authority bundle is incomplete")
    return observed


def process_boundary_read(root: pathlib.Path) -> None:
    script = r'''import hashlib,json,pathlib,sys
r=pathlib.Path(sys.argv[1]); p=json.loads((r/'current.json').read_text())
b=r/p['relative_path']; m=json.loads((b/'bundle-hashes.json').read_text())
assert all(hashlib.sha256((b/n).read_bytes()).hexdigest()==h for n,h in m.items())
'''
    subprocess.run(["python3", "-c", script, str(root)], check=True)


def invocation(args: argparse.Namespace, target: pathlib.Path,
               inject_at: str = "") -> subprocess.CompletedProcess[str]:
    command = [
        "python3", args.runner, "--binary", args.binary,
        "--input", args.input, "--reference", args.reference,
        "--compare-dir", args.canonical, "--output-dir", str(target),
    ]
    if inject_at:
        command += ["--inject-at", inject_at]
    return subprocess.run(command, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, check=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--canonical", required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("p007_runner", args.runner)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load evidence runner for authority tests")
    runner_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner_module)
    with tempfile.TemporaryDirectory(prefix="p007-authority-") as temp:
        target = pathlib.Path(temp) / "authority"
        initial = invocation(args, target)
        if initial.returncode != 0:
            raise RuntimeError("initial authority generation failed:\n" + initial.stdout)
        old_pointer = pointer_bytes(target)
        old_selected = selected_digest(target)
        old_relative = json.loads(old_pointer)["relative_path"]
        process_boundary_read(target)

        for stage in PRECOMMIT_FAILURES:
            failed = invocation(args, target, stage)
            if failed.returncode == 0:
                raise RuntimeError(f"injected {stage} failure unexpectedly succeeded")
            if pointer_bytes(target) != old_pointer:
                raise RuntimeError(f"{stage} changed current authority pointer")
            if selected_digest(target) != old_selected:
                raise RuntimeError(f"{stage} changed selected authority bytes")
            process_boundary_read(target)

        # Cleanup is after the atomic commit. A simulated cleanup error may
        # report failure, but the pointer must select a complete new bundle and
        # the prior immutable version must remain readable.
        post = invocation(args, target, "post_commit_cleanup")
        if post.returncode == 0:
            raise RuntimeError("post-commit cleanup injection unexpectedly succeeded")
        selected_digest(target)
        process_boundary_read(target)
        old = json.loads(old_pointer)
        if not (target / old["relative_path"]).is_dir():
            raise RuntimeError("post-commit cleanup removed prior immutable bundle")

        # Deterministic low-level content exercises isolate the storage
        # protocol from replay timing. Every required bundle file is present.
        def files(label: str) -> dict[str, str]:
            return {name: f"{label}:{name}\n" for name in runner_module.BUNDLE_NAMES}

        same = files("same-content")
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(runner_module.write_bundle_atomic,
                                   target, same) for _ in range(2)]
            for future in futures:
                future.result()
        selected_digest(target)
        process_boundary_read(target)

        different_a = files("different-a")
        different_b = files("different-b")
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = [
                pool.submit(runner_module.write_bundle_atomic, target,
                            different_a),
                pool.submit(runner_module.write_bundle_atomic, target,
                            different_b),
            ]
            for future in futures:
                future.result()
        selected_digest(target)
        process_boundary_read(target)

        # A corrupt directory at the exact requested identity must be rejected
        # without changing the current pointer.
        corrupt = files("corrupt-existing")
        corrupt_manifest = {
            name: hashlib.sha256(content.encode()).hexdigest()
            for name, content in sorted(corrupt.items())
        }
        corrupt_manifest_text = json.dumps(
            corrupt_manifest, indent=2, sort_keys=True) + "\n"
        corrupt_id = hashlib.sha256(corrupt_manifest_text.encode()).hexdigest()
        corrupt_dir = target / "bundles" / corrupt_id
        corrupt_dir.mkdir()
        for name, content in corrupt.items():
            (corrupt_dir / name).write_text(content)
        (corrupt_dir / "bundle-hashes.json").write_text(corrupt_manifest_text)
        (corrupt_dir / runner_module.BUNDLE_NAMES[0]).write_text("corrupted\n")
        before_corrupt = pointer_bytes(target)
        try:
            runner_module.write_bundle_atomic(target, corrupt)
        except RuntimeError:
            pass
        else:
            raise RuntimeError("corrupt existing identity was accepted")
        if pointer_bytes(target) != before_corrupt:
            raise RuntimeError("corrupt existing identity changed pointer")
        process_boundary_read(target)

        # A reader which captured an old pointer before later commits must
        # retain a complete immutable view after all writers finish.
        old_bundle = target / old_relative
        old_manifest = json.loads((old_bundle / "bundle-hashes.json").read_text())
        for index in range(4):
            runner_module.write_bundle_atomic(target, files(f"reader-{index}"))
            if any(hashlib.sha256((old_bundle / name).read_bytes()).hexdigest()
                   != digest for name, digest in old_manifest.items()):
                raise RuntimeError("old-pointer reader observed mutated bytes")
            process_boundary_read(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
