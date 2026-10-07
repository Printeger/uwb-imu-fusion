# Pause checkpoint commit allowlist

This is an allowlist, not an acceptance statement.  The supervisor should
stage only the following classes for the 2026-10-07 pause checkpoint.

## Include

- `.gitignore` and this evidence directory's `.gitignore`.
- `apps/realtime_performance_benchmark.cpp`.
- `config/fde_profiles_validation.yaml` and
  `config/p1_06_simulation_calibrations.json`.
- Modified headers and implementation under `include/` and `src/`.
- Modified tests under `test/`.
- Modified validation tools under `tools/`.
- `docs/review/P0_VERIFICATION_REPORT.md` and
  `docs/review/P1_06_PAUSED_STATE_2026-10-07.md`.
- This directory's `README.md`, `repair-frozen-v9.md`,
  `repair-focused-v10/README.md`, `targeted-kkt-v11/README.md`, the compact
  final-v8 aggregate/status files, and explicitly unignored summary,
  identity, command, hash and exit-code evidence.  For the v11 tree mutation,
  include only `tree-mutation-pass.{exit,log,time.txt}` (exit 0); the earlier
  `tree-mutation-final.*` attempt is stale failed evidence and remains local.

## Exclude

- The five exact untracked user-owned review inputs ignored at repository
  root: `doc/UWB_IMU_PL_系统审计与复现报告.md` and the four
  non-ADS files under `doc/review/`.  The audit exists locally and informed
  the Goal; exclusion from this transport checkpoint does not mean it is
  missing.  Windows alternate-data-stream metadata is also excluded.
- Every `*:Zone.Identifier` file.
- `profiles-38x300/` and all partial/stopped 38x300 output.
- Raw campaign CSV/output directories for 38x20, FGO, long runs, pilots and
  diagnostics; the 2.4 GiB time capture; raw/stale v4-v7/v8/v9 iterations;
  generated build trees; temporary ABI trees; logs not explicitly preserved
  by the nested allowlist; core files and caches.
- `docs/evidence/p1-06-position-quality-acceptance/` and the old untracked
  `docs/evidence/p0-07-corrected-exhaustive/` scratch bundle.  These are not
  needed to document this pause and must not be mixed into the checkpoint.

`artifact-hashes-final-v8.sha256` is immutable historical final-v8 evidence.
Its former `README.md` checksum is expected to fail after this pause index was
added and is not a current-checkpoint verifier.  Current transport documents
and compact v11 proof files are bound separately by
`pause-checkpoint-2026-10-07.sha256`.

The local raw evidence is intentionally not deleted.  It remains available
for a future resumed investigation but is not suitable for Git transport.
