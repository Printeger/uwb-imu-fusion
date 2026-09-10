#!/usr/bin/env python3
"""Run one command and preserve exact argv/cwd/exit/stdout/stderr evidence."""

import argparse
import datetime
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-dir", required=True)
    parser.add_argument("--cwd", required=True)
    parser.add_argument("--expected-exit", required=True, type=int)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        raise RuntimeError("missing logged command")

    record_dir = Path(args.record_dir).resolve()
    record_dir.mkdir(parents=True, exist_ok=False)
    cwd = str(Path(args.cwd).resolve())
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    try:
        completed = subprocess.run(command, cwd=cwd, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        return_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except OSError as error:
        return_code = 127
        stdout = b""
        stderr = (type(error).__name__ + ": " + str(error) + "\n").encode()
    finished = datetime.datetime.now(datetime.timezone.utc).isoformat()
    (record_dir / "stdout.log").write_bytes(stdout)
    (record_dir / "stderr.log").write_bytes(stderr)
    metadata = {
        "schema": "t07_logged_command_v1",
        "cwd": cwd,
        "argv": command,
        "started_utc": started,
        "finished_utc": finished,
        "exit_code": return_code,
        "expected_exit_code": args.expected_exit,
        "stdout_path": str(record_dir / "stdout.log"),
        "stderr_path": str(record_dir / "stderr.log"),
    }
    (record_dir / "command.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    sys.stdout.buffer.write(stdout)
    sys.stderr.buffer.write(stderr)
    return 0 if return_code == args.expected_exit else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("logged command failed: " + str(error), file=sys.stderr)
        sys.exit(1)
