# Refactor Gate 03 Report — UWB Integrity and Robust Baseline

Status: `PASS / INTEGRITY_AND_ROBUST_BASELINE_ESTABLISHED / GATE_04_READY`

Scope: Prompt 3 in `doc/v3/v3.md`. Gate 02 prerequisites were satisfied before
this work. This gate changes paper-input integrity/correlation planning and the
existing robust-baseline initialization path only. It does not alter CUSUM, PL,
support discovery, refit, recoverability, recovery/suppression, solver-success
semantics, thresholds, scientific locks, or paper claims. No scientific or
full-matrix experiment was run.

## Integrity semantics

The raw ledger and estimator decisions remain separate. Each
`MeasurementPlanEntry` now exposes:

- `SOURCE_INVALID`: the source/protocol says invalid, or a required source
  field is non-finite;
- `ESTIMATOR_UNUSABLE`: source-valid data that cannot be used by the configured
  estimator, such as an unknown anchor or an out-of-policy range;
- `USABLE`: source-valid and estimator-usable, with no detected exact repeat;
- `STALE_REPEAT`: estimator-usable but correlated with an earlier exact payload;
- an explicit correlation group, raw representative `obs_id`, group size, and
  whether this row is the one instantiated as an independent likelihood.

`suspected_nlos` remains an orthogonal ledger diagnostic. It does not set source
invalidity or estimator unusability and does not remove a raw row.

## Exact duplicate definition and correlation behavior

The frozen policy identifier is
`EXACT_BINARY64_FULL_PAYLOAD_ADJACENT_MESSAGE_SAME_SLOT_V1`.

The relevant payload signature is exact, bit-for-bit binary64 range, FP RSSI,
RX RSSI, source-valid flag/reason, tag, and anchor. A group is formed only for:

1. an identical signature repeated inside the same source message; or
2. an identical signature in the same source-range slot and immediately
   adjacent source-message ordinal.

Adjacent identical rows chain transitively. Sensor timestamps and provenance
ordinals are intentionally not part of the payload signature: they identify and
preserve the distinct source messages whose payload may be stale. Approximate
values, later non-adjacent recurrence, temporal jumps, residual magnitude,
duration, and RSSI-based NLOS suspicion never create a duplicate group.

Every source observation remains in the ledger with its own stable `obs_id` and
complete provenance. The group representative ID is rooted at the earliest
source row. State association instantiates at most one selected row per group;
if the root's frame was state-subsampled, a selected correlated member may carry
the sole likelihood without changing the root identity. No general stochastic
correlation model was introduced.

The direct duplicate fixture demonstrates the count boundary:

| Count | Value |
|---|---:|
| raw ledger observations | 8 |
| source-valid / estimator-usable observations | 8 |
| exact correlation groups | 4 |
| materialized UWB likelihoods | 4 |
| preserved distinct raw `obs_id`s | 8 |

The runner appends integrity status/group fields to `observations.csv` and
publishes raw, source-valid, estimator-usable, stale-repeat, selected, and
independent-representative counts plus the policy/hash in its input manifests.
The input identity is now `paper_input_v4`; it includes an independent
`integrity_plan_v1` hash and a `measurement_plan_v3` hash.

## Robust baseline protection

Gate 03 reuses the repository's existing standard-loss `PaperRobustNoise` and
canonical `robust_huber` / `robust_cauchy` methods. Only UWB factor noise is
wrapped; raw ranges and the ledger are untouched. Robust scale retains its
existing dimensionless standardized-residual meaning and explicit provenance.
The existing Huber 1.345 and Cauchy 2.3849 experiment specifications are
documented/reused; no constant was invented or tuned against ground truth.

Previously the robust graph was optimized from a completed raw-Gaussian solve.
A catastrophic high-confidence range could therefore poison—or prevent—the
warm start before the robust loss helped. Robust methods now optimize the
robust graph directly from the common initial `Values`, with audit marker
`COMMON_INITIAL_VALUES_DIRECT_ROBUST_V1`. The command-line runner also skips
the unnecessary raw-Gaussian reference for these methods and records the
initialization path and parameter provenance in the baseline manifest.

`all_range` remains the explicit plain-Gaussian stress/ablation comparator.
`fixed_rejection` retains its required frozen raw-Gaussian residual reference.
IE preliminary graph/Values and all Stage-1 through Stage-4 mathematics remain
unchanged.

The synthetic 100-sigma conflict test uses two equally precise factors: the
plain Gaussian estimate moves more than 4 m toward the catastrophic UWB term,
while direct Cauchy remains within 0.1 m of the non-UWB constraint. This is an
engineering regression, not a localization accuracy claim.

## Persistent NLOS and Stage-1 evidence

A four-message synthetic persistent positive bias varies normally, carries the
RSSI suspected-NLOS label, and remains `USABLE`, selected, and present at its
unchanged raw range. No temporal-jump/persistence deletion was added.

FDE Stage-1 observation rows now copy `raw_range_m` directly from the immutable
ledger alongside source frame/message/range/observation ordinals. Existing
factor index, unwhitened residual, factor sigma, residual variance, and
standardized diagnostics still come from the same preliminary graph/Values.
The added field is evidence only and does not enter detector math.

## Tests and verification

Gate-specific regression coverage proves:

1. non-finite and protocol-malformed rows are source-invalid, while an unknown
   anchor is explicitly source-valid but estimator-unusable;
2. suspected NLOS alone preserves and can select a raw observation;
3. exact repeats preserve all provenance but instantiate one factor per group;
4. a 100-sigma isolated UWB factor does not dominate direct robust Cauchy;
5. persistent positive bias is not discarded by integrity logic;
6. FDE Stage-1 evidence can read the original raw range and all source ordinals;
7. the integrity plan remains invariant across `state_step` changes.

Verification commands and evidence:

```text
catkin build uwb_imu_fgo --no-deps --summarize
exit: 0
result: all requested packages succeeded (existing warnings only)
log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/build.make.200.log

catkin test uwb_imu_fgo --no-deps --summarize
exit: 0
result: 484 tests, 0 errors, 0 failures, 0 skipped
result log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/test.results.038.log
```

No test failure was waived. No real-data run, accuracy comparison, full
experiment matrix, threshold tuning, dependency upgrade, commit, or push was
performed.

## Files changed by Gate 03

- `include/uifgo/paper_input.h`, `src/paper_input.cpp`
- `include/uifgo/paper_methods.h`, `src/paper_methods.cpp`
- `include/uifgo/nlos_fde.h`, `src/nlos_fde.cpp`
- `tools/run_ie_paper.cpp`
- `test/test_paper_input.cpp`, `test/test_paper_methods.cpp`,
  `test/test_nlos_fde.cpp`
- `doc/ie_sprint/METHOD_CONTRACT.md`,
  `doc/ie_sprint/EXPERIMENT_CONTRACT.md`, `doc/ie_sprint/STATUS.md`
- `docs/refactor/IE_CORE_V2_CONTRACT.md` and this report

Other dirty-worktree files and existing result directories are user-owned and
were not overwritten.

## Gate decision

Gate 03 acceptance criteria are satisfied. Gate 04 may begin only after a new
explicit user instruction. Its scope is the solver certificate; Gate 03 does
not claim that current generic optimizer termination is scientific success.
