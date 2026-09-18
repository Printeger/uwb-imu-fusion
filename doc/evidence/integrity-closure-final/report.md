# Integrity decision, recovery, and cost closure — development report

Date: 2026-09-16
Status: **DEVELOPMENT_ONLY / PARTIALLY_CLOSED**
Formal eligibility: **false**
Gate D: **not declared PASS**

## Scope and evidence discipline

This round continues from the existing dirty workspace at HEAD
`e6ded3069bbc4cd5cf98f14257e96f61066685b8`. It preserves the roadmap,
previous reports, and previous failed raw runs. New raw artifacts are under
the ignored `results/integrity-closure-final/` tree. No commit, merge, or push
was performed.

The conclusions below use three labels:

- `IMPLEMENTATION_DEFECT`: corrected code behavior with regression evidence.
- `CONTRACT_LIMITATION`: fail-closed behavior required by the current complete
  plausible-set, cardinality, monitorability, or step contracts.
- `NOT_TESTED`: the requested end-to-end result was not demonstrated.

Old reports are historical observations only. In particular, their successful
UWB/IMU selections used the former minimal-plausible-set rule and are not
evidence for the corrected selection semantics.

## Answers to the six closure questions

1. **Complete plausible set:** restored. Selection, action construction,
   eligibility, and coverage audit use the same frozen complete set. A
   plausible strict superset can no longer be discarded because a plausible
   subset exists.
2. **Three unavailable frames after UWB discard:** this was an
   `IMPLEMENTATION_DEFECT`. Quarantine was being converted into probabilistic
   plausibility for every related historical mode. Mandatory health exclusion
   is now separate from evidence. A 65-input replay has one rejected fault
   input followed by 40 clean committed inputs, not the former repeated loop.
3. **The approximately 1265.6 m/s2 IMU number:** it is the present model's
   nominal noncentrality boundary in that physical direction, not a successful
   empirical detection amplitude. Analytic-versus-finite-difference error is
   below `1e-7`, and a half-boundary replay agrees with the local quadratic
   prediction to about 0.93%. The very high boundary is therefore a
   `CONTRACT_LIMITATION` of this noise/geometry/model combination, while reuse
   of the nominal boundary after repeated discard would be an invalid
   calibration assumption.
4. **Full positive sequences:** nominal and UWB post-rejection recovery were
   exercised. Under the corrected complete-plausible contract, the tested UWB,
   IMU, and joint alarm frames have no admissible action within physical-source
   cardinality two, or fail the unchanged step gate. Successful UWB exclusion,
   successful IMU bridge, successful joint exclusion, and raw bridge-timeout
   recovery are therefore **not demonstrated** in this round.
5. **Why alarm cost was enormous:** action kernels and PL were previously run
   before exact frozen coverage/recoverability qualification. In the paired
   UWB alarm, the exact precheck reduces candidate kernels from 45 to zero on
   the fault frame and candidate evaluation from 74,995.5 ms to 9.14 ms.
   KEEP_ALL also reuses the canonical frozen base and performs no candidate
   SVD. The remaining dominant cost is hypothesis evidence: 40,776 hypotheses
   produce 606,348 fault-Gram eigensolves and LDLT factorizations in 28 inputs.
6. **Equivalence and risk:** optimized and exhaustive runs on the same final
   executable have bit-identical state, integrity, and transaction CSVs for
   all 28 inputs. Dense candidate-oracle, 1/4-worker, risk-rounding, numerical
   contract, and transaction regressions pass. This is development evidence,
   not formal HMI-risk closure.

## Implementation defects fixed

### Complete selection semantics

`completePlausibleHypotheses()` returns all plausible hypotheses in frozen
model order. `HypothesisGenerator`, the eligibility precheck,
`FdeManager`, and coverage audit consume that same set. The regression
`PlausibleStrictSupersetRetainsCoverageDutyInSelection` constructs plausible
`H_U` and `H_U+I`; a U-only action is rejected by the real winner selection.

### Health barrier versus probability

Quarantined sources no longer force all associated evidence records plausible.
Health supplies a separate mandatory group set for the current transaction;
probabilistic evidence retains its own meaning. Candidate qualification and
winner selection require both complete probabilistic coverage and mandatory
group removal. Provenance/history contamination remains fail closed. The
regression `MandatoryGroupIsIndependentOfProbabilisticPlausibility` and the
65-input raw replay cover this separation.

### Exact action equivalence

The compact action key is now only a lookup bucket. Before merging coverage
metadata, code compares sorted remove groups, additions/replacements, bridge
semantics, recovery range, versions, units, whitening identity, weights,
covariance, whitener, Jacobians, and residual content. Same-shape/same-norm but
different-content actions do not merge; this is covered by
`ActionDedupRejectsSameShapeAndNormDifferentContent`.

### Exact eligibility before expensive work

After the complete plausible set and mandatory groups are frozen, each action
is checked for coverage, physical-source cardinality, recoverability, valid
remove targets, addition consistency, duplicate/conflicting replacement, and
content identity. An ineligible action remains in v8 audit as
`SKIPPED_INELIGIBLE: ...`, with no fabricated numerical validity or duration.
`UWB_IMU_PL_EXHAUSTIVE_CANDIDATES=1` retains the reference path.

The v8 diagnostic validator recognizes this explicit skip only for v8 and only
when every unexecuted numerical/timing field is empty. v1--v7 retain the old
all-kernels-executed validation rule.

### Shared base and corrected work counters

KEEP_ALL reuses the frozen canonical SVD solution, direct parity statistic,
rank/DOF/condition/logdet, and covariance operator. Near the 0.25 step boundary
it deliberately keeps the exact reference path. The shared numerical contract
fingerprint is checked before detector, evidence, KEEP_ALL, or rank-update
reuse.

Counters now distinguish base SVD/LLT, LLT state-solve calls, SVD state-solve
calls, spectral solves, covariance RHS calls, and RHS column counts. A nominal
window therefore records the two actual base state solves, rather than calling
them one logical solve.

### Development runner and IMU oracle

The runner reads scenario fault epochs, ramp choice, initial velocity, biases,
seed, and schema from `config/r0_r1_development_scenarios.yaml`; environment
variables remain explicit recorded overrides. Production uses analytic IMU
sensitivity with zero reintegration. `UWB_IMU_PL_IMU_FD_ORACLE=1` is an
explicit development oracle and records exactly 12 central-difference
reintegrations per checked interval. A targeted calibration audit avoids
formatting the complete 40,776-hypothesis table.

## Preserved earlier fixes

The following were retained and regressed: unique frozen boundary ownership,
explicit first-state UWB factors, default-ramp window-effective basis, strict
risk-budget rounding, analytic/oracle separation, recovery provenance, and
frozen numerical-contract identity. ADR 0003 remains the development contract
for boundary ownership and physical-versus-effective fault dimensions.

## Raw-stream behavior matrix

All streams use seed `20260901`, 200 Hz IMU, 20 Hz UWB, eight configured
anchors unless noted, raw measurement injection, and `formal_eligible=false`.

| Scenario | Actual result | Classification |
|---|---|---|
| Default ramp nominal, 22 inputs | 22 alarms absent, 22 finite PL values, 22 one-update commits; crossed the former epoch-21 failure | development PASS; PL remains above AL |
| UWB fault plus 40 clean inputs, 65 total | one raw anchor injection; input 25 rejected, inputs 26--65 all commit with finite PL; max consecutive rejection 1 | health-loop `IMPLEMENTATION_DEFECT` fixed; positive exclusion itself not demonstrated |
| IMU accel-x, 10 raw samples, 1582 m/s2 | alarm on four attempts; no bridge winner; two complete-coverage failures and two unchanged 0.25 step-gate failures | `CONTRACT_LIMITATION`, extreme software stress |
| UWB + IMU joint | no joint winner; coverage and step-gate failures retained | `CONTRACT_LIMITATION` for tested stream |
| Ambiguous complete-set UWB alarm | 364 plausible hypotheses; four actions have coverage gaps, 41 complete unions exceed cardinality two | reproducible fail-closed counterexample |
| Bridge epoch/time boundary | deterministic helper accepts 20 epochs / 1.0 s and rejects epoch 21 or the first value above 1.0 s | mechanism PASS; raw positive timeout sequence `NOT_TESTED` |
| Mature 226-input fixed lag and joint injection | 224 commits, 25 real marginalizations, then two zero-update rejects after 10 IMU plus one UWB raw injections | mature pipeline PASS; joint success not demonstrated |

The UWB replay's fault frame has 45 canonical candidate operations. Forty-one
cover the complete set but require more than two physical sources; the other
four leave a nonempty coverage gap. This is the concrete reason a correct UWB
exclusion cannot be selected for that stream. It is not repaired by candidate
pruning, and the implementation does not loosen cardinality.

For example, canonical action 5 must cover `uwb:1` plus accel axes 0/1/2 and
gyro axes 0/1/2 for interval 6 (seven configured fault units), so it is rejected
against cardinality two. Interval identity remains separate in provenance;
axes are the current ADR/config fault units and are not silently collapsed into
one IMU source. If cardinality is intended to count hardware devices instead of
configured physical fault units, that is an explicit contract change requiring
an ADR and new risk argument.

For the IMU replay, the first alarm has 350 plausible hypotheses and every one
of 128 generated actions leaves 347--350 uncovered. Subsequent pending frames
also demonstrate the unchanged step gate. For the retained joint replay, the
first alarm has three plausible hypotheses; one full-cover candidate reaches
the numerical path but fails the step gate, while the other two have coverage
gaps.

## IMU physical-direction calibration

For `imu_accel:0:interval:25` on the nominal frozen window:

- physical direction Gram `d^T G d = 0.000123569`;
- required noncentrality `lambda_required = 197.993`;
- `sqrt(lambda_required / (d^T G d)) = 1265.815 m/s2`;
- finite-difference oracle relative error `3.460561e-08`, verified, 12 raw
  reintegrations;
- predicted statistic at 632.8 m/s2: 49.4815;
- actual raw-injection statistic: 49.0208, a 0.93% difference.

The predefined scan retained all points: 126.56, 316.4, 632.8, 949.2, 1265.6,
1898.4, and 2531.2 m/s2. Their fault-input statistics were respectively about
2.13, 12.45, 49.02, 117.22, 220.30, 527.89, and 966.32 against threshold
269.932. Thus 1265.6 is not a deterministic threshold-crossing amplitude.
At 632.8 m/s2 the faulted/pending window itself has Gram `0.000164102` and
conditioned statistic 18.3016; this confirms that a nominal local calibration
must not be reused as a long-pending boundary.

The injected sensor reading and bridge truth envelope are recorded separately.
The trajectory truth remains inside the configured motion envelope; the large
sensor bias is not interpreted as real acceleration. The physical amplitude is
nevertheless far outside a credible sensor fault range, so this is not a
successful supported IMU bridge claim.

## Optimized versus exhaustive replay

Both runs used the final runner/core library, Release build, seed 20260901,
four candidate workers, `OMP_NUM_THREADS=1`, CPU affinity `0-3`, identical
configuration and log mode. The only difference was
`UWB_IMU_PL_EXHAUSTIVE_CANDIDATES=1`.

| Metric, 28-input UWB replay | Optimized | Exhaustive |
|---|---:|---:|
| state/integrity/transaction mismatches | 0 | reference |
| candidate kernels | 27 | 72 |
| PL evaluations | 27 | 70 |
| fault-frame kernels / PL | 0 / 0 | 45 / 43 |
| fault-frame candidate stage | 9.144 ms | 74,995.503 ms |
| base SVD / LLT | 28 / 28 | 28 / 28 |
| LLT / SVD base state solves | 28 / 28 | 28 / 28 |
| candidate reference SVD | 0 | 44 |
| candidate inner LLT | 0 | 132 |
| covariance RHS calls / columns | 28 / 8,268 | 94 / 8,913 |
| spectral RHS calls / columns | 28 / 8,268 | 94 / 8,913 |
| max RSS | 253,568 KiB | 433,264 KiB |
| process elapsed | 87.94 s | 158.05 s |

Both sides have 606,348 fault-Gram eigensolves and 606,348 LDLT
factorizations. Eligibility pruning therefore closes the wasted-candidate cost
but not the evidence census cost.

The separately pinned final-binary default-ramp nominal run has 22 samples,
core p50/p95/p99/max of 2312.810/7014.188/7543.762/7671.280 ms and outer
p50/p95/p99/max of 2314.605/7015.426/7545.339/7672.950 ms. RSS is 338,252
KiB. It performs 22 base SVDs, 22 base LLTs, 22 LLT plus 22 SVD state solves,
zero detector QR references, zero candidate SVD/LLT, 22 PL evaluations, and
541,788 fault-Gram eigen plus LDLT operations.

Whole-run timing distributions (28 samples; p99 is a development observation):

| Path | core p50 / p95 / p99 / max ms | outer p50 / p95 / p99 / max ms |
|---|---:|---:|
| optimized | 2692.300 / 8141.678 / 8778.719 / 8940.330 | 2694.560 / 8144.340 / 8781.683 / 8943.240 |
| exhaustive | 2821.475 / 6612.640 / 60883.657 / 80941.900 | 2823.045 / 6613.613 / 60900.168 / 80964.100 |

These distributions are not subtracted from one another, and stage quantiles
are not added. The 40 ms target is **not met**. The old 128-kernel formal stress
campaign was not rerun; the configured limit and old raw evidence remain, but
the paired alarm above has 45 canonical post-evidence actions. It must not be
presented as a new 128-kernel pass.

Additional behavior-run timings are retained only as unpaired development
observations (no RSS capture and no cross-row speedup claim):

| Behavior path | n | core p50 / p95 / p99 / max ms | outer p50 / p95 / p99 / max ms |
|---|---:|---:|---:|
| UWB reject then 40-input recovery | 65 | 3269.390 / 5453.030 / 6005.504 / 6204.410 | 3270.480 / 5455.368 / 6007.213 / 6206.560 |
| IMU extreme rejection | 28 | 1741.545 / 4297.478 / 4616.144 / 4714.940 | 1742.595 / 4298.631 / 4617.037 / 4715.860 |
| joint negative | 28 | 1230.171 / 4847.617 / 6641.588 / 7294.370 | 1231.110 / 4848.718 / 6642.754 / 7295.590 |

There are no honest timing rows for a successful IMU bridge or successful
joint action because neither exists under the corrected contract. The mature
run's timing is excluded for the separate contention reason below.

## Tests and numerical invariants

After a full ABI-consistent rebuild:

- CTest: 13/13 targets pass, including ROS and Python tools.
- `test_integrity_v2`: 45/45 tests pass.
- Focused numerical/reinitialization subset: 6/6 tests pass.
- v8 diagnostic tool tests: 12/12 pass.
- The paired, final nominal, and mature raw directories pass
  `validate_run_schema.py` (four validations).
- `git diff --check` passes.

Coverage includes prepare/discard zero updates, commit exactly one update,
correlated-UWB principal-covariance rewhitening, complete-plausible selection,
exact action equality, history replacement, ambiguity/history fail-closed,
frozen graph/order/noise/linpoint/content mutation, numerical-contract
mismatch, near-step reference behavior, dense action oracle at ADR 0002
tolerances, and 1/4-worker deterministic results.

An initial CTest attempt after changing the shared diagnostic structure used
stale test executables and failed due to an ABI mismatch. It is not treated as
an algorithm result; all affected binaries were explicitly rebuilt before the
13/13 result above.

## Mature fixed-lag run

The 226-input `H_mature_union` run completed and passes the raw schema
validator. Inputs 1--224 commit exactly once; the estimator reports 25 actual
marginalizations. Input 225 injects ten accel-x samples and one anchor range
measurement at the raw layer after marginalization. Inputs 225 and 226 are
discarded with zero backend updates (`AMBIGUOUS_UNAVAILABLE`, then
`NO_VALID_CANDIDATE`), maximum pending/state age 0.1 s, and no reinitialization
request. At the fault input, 21 full-cover union actions exceed cardinality two
and two actions leave coverage gaps; the pending retry reaches a candidate but
fails the unchanged step gate.

This proves real fixed-lag progression and post-marginalization fail-closed
behavior, not successful mature joint FDE. The 12:39.57 elapsed time and
256,072 KiB RSS are retained only as behavior-run observations: a short test
link overlapped part of this run, so its timing is excluded from performance
comparison.

## Identity

- HEAD: `e6ded3069bbc4cd5cf98f14257e96f61066685b8` (dirty).
- Final runner SHA-256:
  `92938d2864a05a50da5c784d422a95ce3442734d7d68d5655947e5c0ffc556e7`.
- Loaded `libuwb_imu_pl.so` SHA-256:
  `f12314e393b8235838c908cdf5d02f54cd28f37dd3a0606d20db68406dfe90b5`.
- `libgtsam.so.4` SHA-256:
  `00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da`.
- Scenario YAML SHA-256:
  `f556003bbe3dfc5c8f3991dc0bf5543cdddc258b4b00f42b776dbe03fcbd37e4`.
- Research config SHA-256:
  `7c465463a64299695ec40c580db7a2ff08a0db845697c5f5dfbed60b03ef9171`.
- Implementation-content SHA-256 (38 changed implementation/config/test/ADR
  files, excluding evidence and the user's roadmap):
  `c0ac4d6e66bab7394a13f1693ae3ecfb5a683fa66f7e04be4eca4dbc1d3885b1`.

`ldd` resolves the core library through
`/home/mint/ws_fusion_uwb/devel/lib/libuwb_imu_pl.so` to the private devel
library carrying the hash above. The final implementation-content hash is in
`summary.json`; it excludes this evidence directory and the user's roadmap.

## Reproduction

```sh
cd /home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/r0_r1_development
CFG=config/realtime_uwb_imu_pl_research.yaml
SCENARIOS=config/r0_r1_development_scenarios.yaml

ctest --test-dir /home/mint/ws_fusion_uwb/build/uwb_imu_pl --output-on-failure

OMP_NUM_THREADS=1 taskset -c 0-3 "$BIN" "$CFG" \
  results/integrity-closure-final/final-pair-v2/optimized 28 C_uwb_fde "$SCENARIOS"
OMP_NUM_THREADS=1 UWB_IMU_PL_EXHAUSTIVE_CANDIDATES=1 taskset -c 0-3 \
  "$BIN" "$CFG" results/integrity-closure-final/final-pair-v2/exhaustive \
  28 C_uwb_fde "$SCENARIOS"

UWB_IMU_PL_IMU_FD_ORACLE=1 UWB_IMU_PL_IMU_CALIBRATION_AUDIT=1 \
  OMP_NUM_THREADS=1 "$BIN" "$CFG" \
  results/integrity-closure-final/calibration/A_nominal_oracle \
  25 A_nominal "$SCENARIOS"
```

Every raw directory contains resolved configuration, scenario/run manifests,
raw fault truth, transactions, states, integrity, candidate and coverage
audits, numerical counters, and core/outer timing.

## Remaining limitations and decision points

- `NOT_TESTED`: a successful raw UWB exclusion under the restored complete-set
  contract; successful calibrated IMU bridge; successful two-source joint
  action; 20 committed bridges followed by raw timeout/reinitialization and
  clean recovery.
- `NOT_TESTED`: the requested independent full frozen-graph boundary rebuild
  comparison of information, RHS, covariance, residual constant, and DOF.
  Existing inventory ownership and candidate dense-oracle tests do not replace
  that mathematical oracle.
- `NOT_TESTED`: new formal 128-kernel smoke, 100+40 warm statistics, 3x12000,
  and full E--I campaign.
- `CONTRACT_LIMITATION`: for the tested UWB alarm, full coverage requires more
  than two physical sources. Changing that outcome requires an explicit
  contract/model decision, not an implementation shortcut.
- Remaining cost is the O(hypothesis-count) fault-Gram eigen/LDLT work and
  conservative spectral RHS fallback. A future optimization must reuse unique
  mode blocks/cross terms without dropping joint monitorability or risk and
  must retain exhaustive/dense equivalence.

The code is suitable for further development and focused contract work, but
the missing positive closure paths, boundary graph oracle, and multi-second
core times mean it is **not ready for R3 performance attribution or formal
validation**.
