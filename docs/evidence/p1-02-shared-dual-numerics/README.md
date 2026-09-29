# P1-02 shared dual numerics evidence

Status: `PASS`; the second independent reviewer and supervisor accepted the
ABI-stable sealed-sidecar repair.  The first independent review failure remains
recorded because it caused the design switch.

Baseline: annotated tag `golden-p1-01-identity-indexing`, resolved commit
`86013984e4a8c256db953743290cec767d27b661`.  P0-07 remains
`VALIDATION_PAUSED`; this result does not create or imply a P0-07 golden tag.

## Scope and implementation

This row implements R08 only.  Evidence now constructs one admission-owned,
sealed sidecar containing the immutable candidate response dictionary and
fixed-size (dimension <= 3) hypothesis
blocks containing total/current and history Gram material, score `t`, protected
response `G`, profile `J`, and the protected covariance factor.  The active
dual-channel KEEP_ALL PL path consumes those certified blocks instead of
rebuilding every hypothesis map and raw-factor product.

The first independent review was `FAILED`: the original context compared only
scalar fingerprints and could be transplanted between equal-content seals;
its new fields also changed the public P1-01 `FrozenHypothesisNumerics` ABI.
The repair switched designs instead of extending that gate.  The public type
is restored byte-for-byte to the P1-01 layout (320 bytes, `reason` offset 264),
while the new `FrozenHypothesisDualNumerics` sidecar owns the actual admission
control block and binds its payload address, base-context owner/address,
candidate census and every reused matrix/block under a recomputed digest.
Fingerprints are prefilters only.  Legacy/no-admission calls cannot enter the
dual fast path.

The fast path is deliberately conservative.  The established
`projectPostActionModes` implementation first proves the exact candidate
activity/census.  Frozen reuse is enabled only when that census fingerprint
matches, every retained hypothesis has a valid monitorability and raw-response
certificate, no bridges are present, and proof-audit capture is not requested.
Any miss uses the complete original `computeShared` path.  Modified actions,
uncommon dimensions, invalid/missing responses, proof-audit runs and census
mismatches therefore retain exact fallback.  No sensor, fault family,
hypothesis/action coverage, detector threshold, alert limit, risk, prior,
history, timeout or statistical denominator changed.

## Equivalence result

The formal workload ran three 45-attempt baseline runs and three 45-attempt
candidate runs with identical config, seed, affinity, thread limits and worker
count.  All six commands exited zero.  Both sides completed 135/135 attempts,
excluded zero attempts, and recorded 132 deadline misses.

Each paired comparison passed across 13 output files and all 45 attempts.
State, detector statistics/dof, coverage/census, terminal state, action result,
risk ledger, winner/commit/final packet, PL and certificates had no mismatch.
O12 additionally retained the authoritative 24-epoch replay, seven alarm
actions and the 504-mode census.  The focused P102 test compares the active
dual reference path and frozen path PL/certificate and validates the complete
proof payload.

Only four explicitly named performance/work-accounting fields are excluded
from semantic comparison: candidate covariance solve count, statistical cache
hits, statistical cache misses and cache entries.  Their complete raw arrays,
reason and before/after values are retained in
`ignored-performance-fields.json`; no terminal, census, risk, PL, certificate,
action, winner, commit or packet field is excluded.

## Performance result

All values below use the full 135-attempt denominator.

| Metric | Golden P1-01 | P1-02 candidate |
|---|---:|---:|
| core/analysis p50 (ms) | 3,831.17 | 3,274.54 |
| core/analysis p95 (ms) | 34,289.59 | 4,146.284 |
| core/analysis p99 (ms) | 56,960.16 | 4,494.577 |
| core/analysis max (ms) | 56,991.60 | 4,768.32 |
| arrival-to-publish p50 (ms) | 3,831.52 | 3,274.77 |
| arrival-to-publish p95 (ms) | 34,289.85 | 4,146.708 |
| arrival-to-publish p99 (ms) | 56,960.394 | 4,494.870 |
| arrival-to-publish max (ms) | 56,992.10 | 4,768.64 |
| deadline misses | 132/135 | 132/135 |
| complete work | 135/135 | 135/135 |
| peak RSS (KiB) | 10,048,520 | 9,529,764 |

Across the three runs, certified frozen reuse served 132 candidates with zero
frozen-context misses; three candidates used exact fallback.  Candidate PL
covariance solves fell from 135 to 3 and fault-Gram eigensolves from 5,996,817
to 5,776,308.  Low-dimensional evidence work stayed 5,774,400 on both sides,
showing that no hypothesis coverage was removed.

## Verification

- Focused P102 plus P002 boundary suite: 8/8 PASS.
- O02 dual-channel detector: 6/6 PASS.
- O03/O07 action search: 24/24 PASS.
- O05 `test_integrity_v2`: 99/99 PASS.
- O06 risk oracle: 13/13 PASS.
- O12 directed authoritative replay: 2/2 PASS.
- Complete CTest: 34/34 PASS, exit 0, 1133.99 seconds.
- Golden-header/current-DSO lifecycle client: current DSO construction,
  golden-header access and shared-context destruction all PASS, exit 0; layout
  is 320 bytes with `reason` at offset 264.  This stronger check was added
  after a stale old-layout test binary reproduced exit 139.
- Python comparator/summary syntax and `git diff --check`: PASS.

## Evidence index

- `performance-summary.json`: raw all-attempt timing arrays, outcomes, RSS and
  per-run aggregates for all six runs.
- `equivalence-run1.json` through `equivalence-run3.json`: comparison policy,
  hashes, row counts, wall-transition proofs and PASS results.
- `work-counter-summary.json`: per-run and aggregate numerical work counters.
- `acceptance-summary.json`: all-attempt census, coverage, risk-closure,
  terminal/publication and candidate-result distributions for both sides.
- `ignored-performance-fields.json`: exact ignore allowlist, reasons and every
  raw before/after value.
- `test-results.txt`, `ctest.log`, `o02.log`, `o03-o07.log`, `o05.log`,
  `o06.log`, `o12.log`, `focused-tests.log`: commands and raw test evidence.
- `abi-layout-failure-exit139.txt` retains the stale-binary ABI failure that
  triggered the design switch; `abi-context-client.log` records the repaired
  golden-header/current-DSO lifecycle check implemented by
  `test/p102_old_abi_context_client.cpp`.
- `failing-reproduction.md`: initial failure, strict-precheck failure and the
  conservative scheme switch that prevented a patch loop.
- `environment.txt`, `binary-hashes.sha256`, `loaded-library-hashes.txt`,
  `input-config-hashes.sha256`, `source-hashes.sha256`: provenance.
- `complete-diff.patch`: zero-context implementation/test/tool diff against
  the golden tag; evidence and the supervisor-owned master-table edit are not
  recursively embedded.
- `tools/compare_p1_02_outputs.py` is the row-local comparator; the historical
  P1-01 comparator remains byte-for-byte unchanged from its golden tag.
- `NOT_RUN.md`: explicit exclusions and claims not made.
- `reviewer-second-pass.md`: final independent PASS after the owner/seal and
  ABI repair.

The P1-02 golden tag certifies R08 only.  P0-07 remains
`VALIDATION_PAUSED`, and this evidence does not authorize a `golden-p0-07`
tag.
