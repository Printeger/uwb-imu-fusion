# P1-01 first independent review — FAILED

Verdict: **FAILED**, not BLOCKED. Do not commit.

## Decisive findings

1. The implementation did not achieve the R07 once-only content hash.  All
   three evidence runs and the independent reproduction had 45 windows but
   `window_content_hash_scans=450`, or ten full scans per window.  Production
   consumers still called the full fingerprint from snapshot validation,
   rank-update factorize/evaluate/router, joint detector and protection-level
   paths.
2. The typed handle held only a 64-bit fingerprint/proof hash plus IDs and
   versions.  It did not own an immutable window payload and provided no
   canonical-content secondary equality for a hash collision.  Removing the
   remaining scans in that design would allow mutation or stale reuse to evade
   identity invalidation.
3. The recorded benchmark command was not executable: its extra CLI arguments
   violate the benchmark's `argc == 4` contract and independently returned
   exit 2.  The actual required
   `UWB_IMU_PL_BENCHMARK_SEED=20260928` environment variable was omitted.
4. The comparator excluded broad publication refusal/unprotected fields even
   though one before repetition had a different terminal refusal after a real
   wall-deadline crossing.  Strict semantic comparison therefore needs a
   frozen/injected clock, with only mechanically proven wall-derived fields
   separated for live performance reporting.

## Independent passing evidence

- P101 focused: 4/4 PASS.
- O03: 24/24 PASS.
- Complete CTest: 34/34 PASS, exit 0, 376.88 seconds.
- Diff check, zero config diff, source/binary/input hashes and patch
  reverse-apply: PASS.
- Three raw before/after journals recomputed to 135/135 complete, zero excluded
  and zero incomplete work.
- With the missing seed supplied, an independent 45-attempt candidate run was
  45/45 complete with zero comparator mismatch and reproduced the speedup and
  work counters.  It also reproduced exactly 450 full hash scans for 45
  windows.
- P0-07 remained `VALIDATION_PAUSED`; no `golden-p0-07` tag existed.

## Minimum repair boundary

- Own the complete frozen payload through an immutable object/typed admission
  handle.
- Seal/hash canonical content once; production consumers validate immutable
  ownership, type and version in constant work.
- Mutation produces a new object/handle and cannot reuse an old handle.
- A hash collision requires full canonical equality or an equally strong
  secondary predicate.
- Reduce the production count to approximately one content scan per window
  without removing mutation defenses.
- Correct the exact benchmark command and seed environment in the evidence.
