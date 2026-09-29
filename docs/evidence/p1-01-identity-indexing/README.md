# P1-01 immutable identity and indexing evidence

Status: `PASS` (fourth independent reviewer and supervisor)

Development baseline: ordinary user-authorized checkpoint
`420cb00786b776d81b4ec9f5d430433b1378e7cb`.
The latest true golden remains `golden-p0-06-actions`; P0-07 independent
validation is paused and no `golden-p0-07` tag exists.

## Scope

This row implements R07 only:

- one typed frozen-window proof identity after full-content admission;
- constant-work exact owner/payload binding for every sealed candidate
  consumer, with no sealed-path hash-only equality fallback;
- stable mode and compact-pair descriptor indices;
- reuse of an already-built canonical action operation when constructing its
  semantic identity, while retaining complete byte-exact equality;
- one frozen-information solve for the concatenated unique-block RHS.

It does not change configuration, sensor/fault families, hypotheses, actions,
thresholds, alert limits, priors, risk budgets, statistical denominators,
fallbacks, timeout behavior, or P0-07 fail-closed qualification behavior.

## Acceptance contract

The mandatory Section 2.4 gate applies verbatim.  O03, O05, O09 and O12,
focused identity/index/dedup/batched-RHS tests, the complete CTest suite and
the same-input all-attempt comparison are required.  Every state/covariance,
detector statistic/dof, census/terminal state, per-hypothesis numerical result,
action/risk/selection/commit/final-packet field remains part of the comparison.

Performance evidence uses fixed CPU affinity, numerical thread counts, worker
count, source/config/input and all-attempt denominator.  It reports
core-compute, analysis-completion and arrival-to-publish p50/p95/p99/max,
deadline misses, complete-work rate, RSS and numerical work counters.  Runs
are retained even when slow or failed; no fast-reject or successful-only
filter is permitted.

## Implementer result

The candidate preserves all 135/135 attempts and complete-work results, with
zero excluded attempts, watchdog refusals, or unopened transactions.  The
three direct checkpoint comparisons are exact for every non-wall-derived field
covered by the comparator.  O12 independently retains 24/24 epochs, 7/7 alarm
actions, the 504-mode census, and the terminal semantic digest.

The real-wall runs deliberately retain deadline/staleness variability.  The
checkpoint repeated against itself did not have stable wall classifications,
so only explicit wall-clock deadline, sensor-lag/staleness, refusal reason and
wall-timeout fields are excluded from the strict semantic comparator.  They
remain included in the performance journal and aggregate below.

| All 135 attempts | Checkpoint | Candidate |
|---|---:|---:|
| core/analysis p50 (ms) | 28,230.80 | 3,756.70 |
| core/analysis p95 (ms) | 43,075.14 | 34,219.19 |
| core/analysis p99 (ms) | 66,268.55 | 56,952.51 |
| core/analysis max (ms) | 67,317.50 | 57,046.00 |
| publish p50/p95/p99/max (ms) | 28,231.00 / 43,075.54 / 66,268.88 / 67,318.00 | 3,756.97 / 34,219.39 / 56,952.81 / 57,046.20 |
| deadline misses | 133 | 132 |
| complete work | 135/135 | 135/135 |
| peak RSS (KiB) | 10,046,036 | 10,048,092 |

Each candidate run builds 45 frozen identities and reuses them 1,925,070
times, performs 3,827,760 direct descriptor lookups with zero descriptor
linear scans, and performs 45 full-content hash scans: exactly one seal-time
scan per window.  This benchmark's
selected actions are `KEEP_ALL`, so its unique-block batch count is zero; the
focused changed-block fixture proves five unique block columns are processed
by one certified solve and compares the complete result to the uncached path.
The sealed-owner regressions also prove: independent equal-content owners do
not accept each other's candidates; a forced equal-hash/different-payload
transplant is rejected by the real detector consumer without another content
scan; production move-sealing isolates later source changes; and a fixed
placement allocator cannot reuse the payload address while a candidate owns
the seal, then reuses that exact address only after release with a new token.

## Verification

- P101 focused identity/index/dedup/batched-RHS/owner binding: 13/13 PASS.
- O03 action-search suite: 24/24 PASS.
- O05 `test_integrity_v2`: 98/98 PASS.
- O09 deterministic/repeated/worker tests: 2/2, 1/1, 1/1 PASS.
- O12 authoritative replay: 24/24 epochs, 7/7 alarm actions, 504/504 census,
  exact non-timing comparison PASS.
- P0-07 corrected exhaustive: 8/8 PASS.
- Complete CTest: 34/34 PASS, exit 0, 1122.03 seconds.
- `git diff --check`: PASS.

The P1 edit shifted P0-07 source symbols without changing their contracts.
The tracked fixed-contract source map was mechanically rebased from 1198 to
1218, from 1846 to 1866, and from 3212 to 3218; the full 174-leaf/mutation
protocol then passed.  No contract value or P0-07 test behavior changed.

## Evidence index

- `performance-summary.json`: all raw timing arrays, per-run outcomes, RSS and
  work counters for all three before and all three after runs.
- `equivalence-run1.json` through `equivalence-run3.json`: exact comparison
  policy, hashes, row counts and PASS results.
- `baseline-comparison.txt`: independent checkpoint worktree/build/DSO binding,
  executable command, hashes and the accepted checkpoint raw run compared
  strictly against each of the three candidate runs.
- `abi-results.txt`: four checkpoint/current layouts and the
  golden-header/current-DSO canary result.
- `test-results.txt`: commands, counts, exits and the first-failure repair.
- `environment.txt`: compiler/dependency/runtime settings and hashes.
- `complete-diff.patch` and `source-hashes.sha256`: review material. The patch
  is deliberately stored with zero context so evidence itself passes the
  whitespace gate; validate it with
  `git apply --check --reverse --unidiff-zero complete-diff.patch`.
- `NOT_RUN.md`: explicit exclusions; none is claimed as PASS.

This bundle passed the fourth independent review and supervisor acceptance.
Its P1-01 tag records only the accepted R07 optimization; P0-07 remains
`VALIDATION_PAUSED` and no `golden-p0-07` tag is authorized.
