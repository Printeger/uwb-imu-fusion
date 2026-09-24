# UWB/IMU FDE project audit master table

> This file is the project-level review ledger for the correction and
> optimization work that follows the fixed review baseline.  The detailed P0
> verification evidence remains below; Section 2 is the controlling execution
> and acceptance contract.

- Baseline: `c7fd4cc79a3a22cebeb46aaa03706a0c65938f39`
- Verification date: 2026-09-23
- Requested source review: `docs/review/UWB_IMU_FDE_Code_Review_c7fd4cc.md`
- Actual source review in this checkout: `doc/review/UWB_IMU_FDE_Code_Review_c7fd4cc.md`

## 1. Result

The verification target, and `HEAD` at the start of the verification pass, was
exactly `c7fd4cc79a3a22cebeb46aaa03706a0c65938f39`. No implementation fix was
made in that pass; the later audit-table commit changes documentation only.

The P0 correctness claim for the complete chain cannot be accepted at this baseline. D01 through D08 are **CONFIRMED**. The action-cap issue D09 is **CONFIRMED under the review's stated P0 condition**: the current selection path treats the returned action set as complete but carries no search-incomplete state or dominance proof. The IMU noise-density/trapezoid-correlation item is **DESIGN-DEPENDENT** because the checkout contains no calibration artifact proving the intended sigma semantics or an overbound. D11 has two confirmed output defects; the final-deadline/log divergence is the P0 part, while the Odometry velocity-frame defect is principally a navigation interface defect. D12, the absence of an independent raw-factor oracle, is **CONFIRMED**.

There are no `ALREADY FIXED` or `NOT REPRODUCIBLE` findings in the P0 set. “Confirmed” below means that the implementation or missing invariant is directly reachable/provable; it does not claim that every harmful magnitude has been measured on hardware.

### 1.1 Classification and effect matrix

`Y` means direct effect, `I` means indirect effect through candidate/selection/publication, and `—` means no direct effect was found.

| Item | Classification | Navigation estimate | Detector statistic | FDE recovery | PL | HMI/risk | Transaction commit |
|---|---|---:|---:|---:|---:|---:|---:|
| D01 duplicate historical observation | CONFIRMED | Y | Y | Y | Y | Y | I |
| D02 historical RHS sign | CONFIRMED | Y | Y | Y | Y | Y | I |
| D03-A response layout/initialization | CONFIRMED | — | I | I | I | I | I |
| D03-B missing history mode/census connection | CONFIRMED | I | Y | Y | Y | Y | I |
| D04 pooled post-detector fallback | CONFIRMED | I | Y | Y | Y | Y | Y |
| D05 incomplete risk closure | CONFIRMED | — | — | Y | Y | Y | Y |
| D06 premature selection-risk merging | CONFIRMED | I | — | Y | Y | Y | Y |
| D07-A post-mutation exception gap | CONFIRMED | Y | I | Y | I | I | Y |
| D07-B unbound committed PL reference | CONFIRMED | Y | — | Y | Y | Y | Y |
| D08 inconsistent numerical certificates | CONFIRMED | I | I | Y | Y | Y | I |
| D09 capped action search | CONFIRMED, conditional P0 | I | — | Y | I | Y | Y |
| N01 IMU noise/trapezoid qualification | DESIGN-DEPENDENT | Y | Y | Y | Y | Y | I |
| D11-A Odometry twist frame | CONFIRMED | Y | — | — | — | — | — |
| D11-B final deadline/log packet | CONFIRMED | — | — | — | — | Y | I |
| D12 self-referential oracle | CONFIRMED | I | I | I | I | I | I |

### 1.2 Coverage of the review's P0 recommendations

| Review recommendation | Verification coverage |
|---|---|
| R01 history unique rows, RHS, constant and carrier | D01, D02, D03-A, D03-B |
| R02 identical online all-in/post/PL dual-channel contract | D04 |
| R03 complete risk ledger and real selection events | D05, D06 |
| R04 unified RHS/covariance/PSD/nullspace numerics | D08 |
| R05 complete commit receipt, reference transfer and final packet | D07-A, D07-B, D11-A, D11-B |
| R06 independent raw-factor oracle and evidence bundle | D12; N01 records the remaining external qualification evidence |

D09 is kept as a separate conditional-P0 record because its review priority changes from P1 to P0 whenever exhaustive action coverage is claimed. No performance recommendations R07–R14 were implemented or evaluated as optimizations.

## 2. Project audit master table

### 2.1 Scope and status vocabulary

This plan has **13 development steps**: seven P0 correctness steps followed by
six P1 optimization/acceptance steps.  R13 and R14 are intentionally excluded:
they change the risk/selection contract or introduce research-only hierarchy
and anytime behavior, and therefore require a separate P2/P3 authorization.

The table status is one of:

- `NOT_STARTED`: no implementation claim has been made;
- `IN_PROGRESS`: work is occurring, but the previous golden baseline remains
  the only reference;
- `BLOCKED`: an explicit dependency or required artifact is absent;
- `FAILED`: the step was exercised and did not satisfy its exit gate;
- `GOLDEN`: the implementation commit, required tests and evidence bundle all
  passed, and an immutable golden tag points to that commit.

Current project state is `P0_REMEDIATION`; the latest immutable baseline is
`golden-p0-03-numerics`.  P1 is `CLOSED`.  P0-01 corrected the known
`test_round2_tools` v5/v6 evidence-harness mismatch, and its complete CTest
result is recorded in the evidence bundle; the historical
`catkin_test_results` count is not an acceptance substitute for the complete
CTest result.

### 2.2 Minimal ordered development plan

Every row is a separately reviewable change and ends in a commit.  Every P0
row additionally creates the named immutable annotated tag; that tagged commit
is the next row's only golden input.  A later row may not rewrite, squash or
move an earlier golden tag.

| Step | Required scope | Mandatory evidence / exit gate | Commit and golden baseline | Status |
|---|---|---|---|---|
| **P0-01 History raw-factor equivalence** | R01; D01, D02, D03-A, D03-B; establish the independent raw-factor portion of D12. Fix unique row ownership, the `Ax-b` RHS convention, initialized carrier dimensions, and history-to-physical-mode/census mapping. Also remove the current v5/v6 test-harness mismatch so the full suite has an honest result. | O01 and O03. Nonzero RHS, correlated covariance, multiple fixed-lag crossings and persistent/ramp history must match the unique raw system in objective, state, covariance, `T_b/F_b`, constant, dof, Gram and census. Any mismatch stops the step. | Commit: `fix(p0-01): restore history raw-factor equivalence`; tag: `golden-p0-01-history` | `GOLDEN` |
| **P0-02 Dual-channel contract** | R02; D04. Introduce one candidate detector certificate carrying row roles, current/history statistics, constant, rank/dof, accepted-event ID and numerical identity. Active v6 must not silently use pooled fallback. | O02. All-in, KEEP_ALL and modified candidates agree for dense and matrix-free paths; the pooled-pass/dual-fail and missing-constant examples reject; missing payload is `INVALID` and cannot produce a finite certified PL. | Commit: `fix(p0-02): enforce dual-channel candidate contract`; tag: `golden-p0-02-dual-channel` | `GOLDEN` |
| **P0-03 Unified numerical certificate** | R04; D08. Use one certified square-root solve/tolerance/proof identity for state, covariance, fault and bridge RHS; unify symmetry, PSD, rank, nullspace and `G ker(W)` decisions; retain exact fallback. | O05. State and every covariance/Gram/slope/PL quantity agree with an independent stable QR/SVD reference over rank/condition transitions. Tiny-map/unbounded-fault and indefinite-Gram examples fail closed. | Commit: `fix(p0-03): unify integrity numerical certificates`; tag: `golden-p0-03-numerics` | `GOLDEN` |
| **P0-04 Complete risk and selection events** | R03; D05, D06. Separate allocation validity, complete bound closure, validation status and formal eligibility. Charge or reject miss/escape/omitted/selection terms. Remove plausible-ID premerging without a numeric shared-event proof. | O06. High-precision ledger closes on the same total budget; unknown terms are not zero; disjoint action events are not merged; over-budget profiles become unavailable without changing budgets, alert limits or thresholds. | Commit: `fix(p0-04): close risk and selection accounting`; tag: `golden-p0-04-risk` | `NOT_STARTED` |
| **P0-05 Transaction, reference and final packet** | R05; D07-A, D07-B, D11-B. Extend the mutation guard through receipt creation, stage metadata, bind committed mean to the PL reference or transfer it, and make ROS/logging consume one immutable final packet. Fix D11-A in the same output boundary by rotating Odometry velocity/covariance to the child frame. | O04, O08 and O10. Exception injection at every mutation/query/ledger/publication boundary yields one receipt or poison; committed mean is covered; ROS, CSV and mirrors agree; deadline timestamps include the defined packet/publish boundary; 90-degree-yaw velocity test passes. | Commit: `fix(p0-05): make commit and publication certificates atomic`; tag: `golden-p0-05-publication` | `NOT_STARTED` |
| **P0-06 Action-search completeness** | Conditional-P0 D09. Eliminate silent action truncation. Exact duplicates may be merged; otherwise evaluate all actions or report `SEARCH_INCOMPLETE` with generated/evaluated/omitted identities. | O07. An uncapped reference and reordered enumeration produce the same winner/refusal. A resource-limited run cannot claim exhaustive selection or protected output. Increasing the cap alone does not pass. | Commit: `fix(p0-06): fail closed on incomplete action search`; tag: `golden-p0-06-actions` | `NOT_STARTED` |
| **P0-07 Corrected exhaustive closure** | R06; complete D12 evidence and resolve N01's noise/trapezoid semantics. Produce a corrected exhaustive baseline and an evidence bundle. Missing external calibration may remain an explicit qualification blocker only if it is represented by a fail-closed model gate and protected/formal output remains disabled. | O01–O12 correctness smoke, complete CTest exit status, sanitizer subset where available, replay snapshots, source/diff/binary/library/config/input/output hashes and explicit `NOT_RUN`. No P0 finding may remain an uncontrolled behavior. | Commit: `test(p0-07): establish corrected exhaustive baseline`; tag: `golden-p0-07-corrected-exhaustive` | `NOT_STARTED` |
| **P1-01 Immutable identity and indexing** | R07 only: frozen typed handles, one content hash, direct descriptor indices, compact pair descriptors, exact operation deduplication and batched unique-block RHS. | O03, O05, O09, O12 against `golden-p0-07-corrected-exhaustive`; census, decisions, risk and outputs unchanged; full-attempt time or work counters improve. | Commit: `perf(p1-01): remove redundant identity and lookup work` | `NOT_STARTED` |
| **P1-02 Shared dual numerics** | R08: one candidate root/context, shared unique-mode responses and fixed-size pair blocks across detector, evidence and PL. Keep every hypothesis and fallback. | O02, O03, O05, O06, O07, O12. Per-hypothesis `Gamma_c/Gamma_h/t/G/J/L`, worst PL, winner and certificate match the latest golden baseline within the numerical contract. | Commit: `perf(p1-02): share dual-channel hypothesis numerics` | `NOT_STARTED` |
| **P1-03 Incremental history root** | R09: update only the proven dependency closure while preserving raw row provenance, fault columns, constants and dof; retain full rebuild fallback. | O01, O03, O05, O09, O12. Every epoch matches full rebuild; recovery invalidates affected cache entries; long-run RSS is bounded. | Commit: `perf(p1-03): incrementally maintain history root` | `NOT_STARTED` |
| **P1-04 Complete streamed recovery search** | R10: replace the P0 fail-closed incomplete-search behavior with bounded-memory full action streaming and a complete rejection funnel. Do not change plausible thresholds or selection ordering. | O03, O07, O12. Same winner/refusal as uncapped reference, order-independent result, no wrong-exclusion increase, and no omitted action without proof. | Commit: `perf(p1-04): stream complete recovery search` | `NOT_STARTED` |
| **P1-05 Deterministic concurrency and timing** | R11: flat candidate×hypothesis tasks, fixed output slots, deterministic reduction, bounded scratch and one all-attempt timer. No nested use of the same worker pool. | O08, O09, O10, O12 plus TSan where available. One/two/four workers produce identical decisions; exceptions and deadline misses remain in the denominator; p99 and RSS are reported, not assumed. | Commit: `perf(p1-05): parallelize deterministic integrity work` | `NOT_STARTED` |
| **P1-06 Position-quality diagnosis and final acceptance** | R12 plus the complete P1 acceptance run. Diagnose off-profile attitude/long-run behavior in the required frame/observability/time/Jacobian/initialization order. Only proven implementation defects may be fixed here; a model, noise, prior or risk-contract change is moved to a separately authorized P2 step. | O04, O05, O11, O12. Re-run all profiles and all attempts on the same hardware/source/config/input; report core, analysis-completion and arrival-to-publish p50/p95/p99/max, complete-work rate, PL/recovery, navigation errors and RSS. | Commit: `test(p1-06): record full corrected performance acceptance` | `NOT_STARTED` |

The ordering is strict.  P1-01 may start only after every P0 row is `GOLDEN`
and `golden-p0-07-corrected-exhaustive` exists.  A P0 result that is merely
faster, finite, or internally self-consistent is not complete.

### 2.3 Golden-baseline protocol

For each P0 step, the implementing Codex instance must do all of the following
before marking the row `GOLDEN`:

1. start from the immediately previous golden commit and record its tag;
2. add an independent failing reproduction before relying on the fix;
3. make only the row's declared contract change—no threshold, alert limit,
   total risk, fault family, hypothesis census or sensor change;
4. run the row-specific oracle and the complete CTest suite, using the CTest
   exit code rather than only `catkin_test_results`;
5. save the evidence bundle required by Section 2.5;
6. update this row's status and evidence link, create the declared commit, and
   place the immutable annotated golden tag on that commit.

If a required result is `NOT_RUN`, unknown, numerically uncertified or missing,
the step is `BLOCKED` or `FAILED`, never `GOLDEN`.  The only external-evidence
exception is N01: absent hardware qualification may be closed for continued
research optimization only through an explicit fail-closed qualification gate,
while `formal_eligible=false` and protected output remain disabled.

### 2.4 Mandatory Codex gate for every P1 performance change

Every P1 task issued to Codex must include the following block.  Omitting an
item invalidates the task review.

```text
P1 task: <P1-ID and one optimization only>
Golden reference: <immutable latest golden tag>
Allowed implementation scope: <files/functions>
Forbidden contract changes:
  - no sensor/fault-family/hypothesis/action coverage reduction
  - no detector threshold, AL, total risk or prior change
  - no pooled fallback, missing history, disabled exact fallback or hidden timeout
Required equivalence:
  - state/covariance and current/history detector statistics + dof
  - expected/represented/evaluated census and every task terminal state
  - per-hypothesis Gram/profile/protected response/PL and nullspace class
  - action eligibility/post result/risk ledger/winner/commit/final packet
Required performance evidence:
  - all-attempt core_compute, analysis_completion and arrival_to_publish
  - p50/p95/p99/max, deadline misses, complete-work rate, RSS and work counters
Required tests: <applicable O01-O12 plus complete CTest>
Stop immediately if:
  - any required leaf/action becomes unknown or omitted without proof
  - a discrete decision differs from the golden reference
  - PL/risk moves downward without a proved mathematical equivalence
  - speedup comes from fast rejects, excluded failures or incomplete work
Commit only after all gates pass; otherwise report FAILED/BLOCKED and do not commit.
```

P1 equivalence uses quantity- and conditioning-aware absolute/relative
tolerances.  A single blanket tolerance is forbidden.  Rank, condition, step,
PSD and alert-limit boundary cases require certificate intervals or exact
fallback.  Discrete decisions must agree exactly unless both paths explicitly
return the same fail-closed indeterminate state.

### 2.5 Evidence bundle and commit record

Each completed row must retain:

- source SHA, base golden tag and complete diff;
- compiler, dependency, numerical environment and worker settings;
- binary and loaded-library hashes;
- resolved configuration, manifest, input and truth hashes;
- commands, exit codes, complete CTest result and explicit `NOT_RUN` list;
- all-attempt journal, census/coverage/risk/work-counter summaries;
- failing and boundary frozen replays needed to reproduce the decision;
- before/after performance only for P1 rows, using the same workload and
  denominator.

The completion record is maintained here:

| Step | Status | Commit | Golden tag / reference | Evidence bundle | Reviewer note |
|---|---|---|---|---|---|
| P0-01 | `GOLDEN` | this commit (`golden-p0-01-history^{}`) | `golden-p0-01-history` | [p0-01-history](../evidence/p0-01-history/README.md) | PASS: independent reviewer and supervisor; O01/O03 plus complete CTest 26/26 |
| P0-02 | `GOLDEN` | this commit (`golden-p0-02-dual-channel^{}`) | `golden-p0-02-dual-channel` | [p0-02-dual-channel](../evidence/p0-02-dual-channel/README.md) | PASS: independent reviewer and supervisor; O02 7/7, dual 5/5, complete CTest 26/26 |
| P0-03 | `GOLDEN` | this commit (`golden-p0-03-numerics^{}`) | `golden-p0-03-numerics` | [p0-03-numerics](../evidence/p0-03-numerics/README.md) | PASS: independent reviewer and supervisor; O05/proof/ABI gates plus complete CTest 26/26 |
| P0-04 | `NOT_STARTED` | — | `golden-p0-04-risk` | — | — |
| P0-05 | `NOT_STARTED` | — | `golden-p0-05-publication` | — | — |
| P0-06 | `NOT_STARTED` | — | `golden-p0-06-actions` | — | — |
| P0-07 | `NOT_STARTED` | — | `golden-p0-07-corrected-exhaustive` | — | P1 gate is closed until this row is GOLDEN |
| P1-01 | `NOT_STARTED` | — | use latest golden | — | — |
| P1-02 | `NOT_STARTED` | — | use latest golden | — | — |
| P1-03 | `NOT_STARTED` | — | use latest golden | — | — |
| P1-04 | `NOT_STARTED` | — | use latest golden | — | — |
| P1-05 | `NOT_STARTED` | — | use latest golden | — | — |
| P1-06 | `NOT_STARTED` | — | use latest golden | — | Final acceptance |

## 3. Verification method and chain trace

The pass followed the production chain in this order:

1. `IncrementalUwbImuEstimator::buildIntegrityWindow` and the fixed-lag/history summary code;
2. all-in and candidate detector construction;
3. fault-space generation, mode registry and exact-coverage certificate;
4. action census and preparation;
5. rank-update candidate solve and post-FDE detector;
6. PL and risk ledger;
7. recovery selection and shared-guarantee grouping;
8. transaction mutation, receipt, and ROS/log publication.

The reproductions below are deliberately unit-level or algebraic. They do not depend on runtime optimization, threshold relaxation, hypothesis removal, risk-budget changes, history removal, or post-FDE detector removal.

## 4. Detailed findings

### D01 — the history fault augmentation duplicates the nominal observation

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`, `IncrementalUwbImuEstimator::buildIntegrityWindow`, lines 1172–1770. The nominal boundary group factors are inserted into `augmented` at lines 1356–1365. Historical fault injections are then built in the loop at lines 1467–1525 and pushed into that same graph at line 1519; the nominal factor for the group is not replaced.
- `src/uwb_imu_pl/integrity/history_fault_parameterization.cpp`, `buildHistoricalFaultInjection`, lines 330–437. It obtains the same linearized group `A` and `b` at lines 390–399, copies the state blocks, appends fault-key blocks, and constructs a new full `JacobianFactor` at lines 407–431.

Thus the two rows represent the same random observation, not two independent measurements.

**Smallest reproduction.** For one scalar observation with fault parameter `f`, the required objective is

```text
J(x,f) = (x + f - 1)^2.
```

The constructed objective is

```text
Jdup(x,f) = (x - 1)^2 + (x + f - 1)^2.
```

At `f=1`, the correct minimum is `x=0, J=0`; the duplicated objective has `x=0.5, J=0.5`. At `f=0`, it doubles the observation information.

**Expected invariant and violation.** Every stochastic observation row and covariance ownership must appear exactly once. Adding fault columns augments/replaces the row; it must not create an independent nominal copy. The current graph contains both copies whenever historical fault columns are injected.

**Downstream consumers.** `extractBoundaryRows`, `buildHistoryFaultSummary`, the frozen `H/z`, square-root numerics, all-in detector, hypothesis response/Gram, candidate rank updates, post-detector, PL, selection, and the chosen commit plan all consume the contaminated summary.

**Effects.** It can directly change the frozen navigation increment/covariance, detector statistic, FDE recovery ranking, PL, and effective HMI accounting. It does not corrupt the transaction mechanism itself, but it can change which plan is committed.

**Smallest fix.** Give each raw random row a stable UID. For a group with historical fault columns, insert the augmented factor *instead of* its nominal factor; leave unaffected groups unchanged. Assert row-UID and covariance ownership uniqueness before condensation.

### D02 — the historical RHS has an extra sign reversal

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/integrity/history_summary_extraction.cpp`, `extractBoundaryRows`, lines 14–69, copies `JacobianFactor::getb()` unchanged at lines 56–57.
- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`, `IncrementalUwbImuEstimator::buildIntegrityWindow`, lines 1590–1597, comments that the factor uses `Ax+b` and assigns `summary_input.rhs` the negative of the extracted last column.
- `src/uwb_imu_pl/integrity/history_fault_summary.cpp`, `buildHistoryFaultSummary`, lines 140–249, declares the residual convention `H_o x_o + H_b x_b + A f - z`, copies `input.rhs` into `W` at lines 175–188, and derives `d_b/d_perp` from it at lines 235–246.
- GTSAM's installed `gtsam/linear/JacobianFactor.h`, lines 52–88, defines the error as `0.5 (A x - b)^T Sigma^-1 (A x - b)`.

**Smallest reproduction.** Combine a current row `x=3` with a historical row `x=1`. Correct:

```text
(x - 1)^2 + (x - 3)^2  ->  x*=2, T=2.
```

With only the history RHS reversed:

```text
(x + 1)^2 + (x - 3)^2  ->  x*=1, T=8.
```

**Expected invariant and violation.** Every row must use one convention, here `H x - z`. `getb()` is already `z=b`; negating it only in the history path makes history and current rows describe different measurements.

**Downstream consumers.** Boundary summary state/RHS, history constant, frozen state solution, all-in detector, profile/evidence values, candidate solves, post-detector, PL, and selection.

**Effects.** Direct effects are possible on navigation increment, detector statistic, FDE recovery and PL. The budget formula is unchanged, but the event being bounded is wrong, so HMI coverage and the chosen transaction are affected indirectly.

**Smallest fix.** Remove the extra negation and enforce a single `Ax-b` convention at extraction, condensation and reconstruction boundaries. Add a nonzero-RHS oracle starting from raw unique factors.

### D03-A — `history.response` has a contradictory shape and unwritten elements

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `include/uwb_imu_pl/estimation/integrity_window_snapshot.hpp`, `WindowHistorySummary`, lines 60–110, documents `response` as `T_b: boundary_columns x q`, `detector_response` as `F_b: nu_perp x q`, and a logical stacked response `[response.col(i); detector_response.col(i)]` at lines 93–97.
- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`, `buildIntegrityWindow`, lines 1692–1719, allocates `history.response` with `kept_rows.size()` rows at line 1699. The fill loop writes only `history.response.row(boundary_row)` for supported rows and writes perpendicular rows to `history.detector_response`; the unused tail of `response` is not initialized.
- `src/uwb_imu_pl/estimation/integrity_window_snapshot.cpp`, `integrityWindowFingerprint`, lines 186–223, hashes the entire `history.response` at line 222, including unwritten storage.
- `test/test_history_summary_pipeline.cpp`, lines 475–610, consumes the carrier produced by this same path. The rebuilt test printed `rows=333`, `boundary_rows=15`, `q=220`, `nu_perp=318`; this is concrete evidence that the allocated 333-row `response` does not match the documented 15-row `T_b`.

**Smallest reproduction.** With one supported row, one perpendicular row and `q=1`, the code allocates a `2x1` `response`, writes only element 0, and stores `F_b` separately. Element 1 is indeterminate but is fingerprinted. The declared carrier should instead be `T_b(1x1)` plus `F_b(1x1)`, or a fully initialized explicitly mapped `2x1` object.

**Expected invariant and violation.** A carrier must have one unambiguous dimension/row map, and every stored/fingerprinted element must be initialized. The implementation violates both requirements.

**Downstream consumers.** Snapshot fingerprint/cache identity, tests, dump/replay users, and any mode builder that starts consuming the documented carrier. Current production fault-mode generation largely ignores the carrier, which limits the direct present-day numerical propagation of the unwritten tail; that omission is D03-B.

**Effects.** No direct nominal-state change was found from the unwritten tail. It can destabilize identity/caching and, once/where consumed, detector response, recovery and PL. An identity mismatch can indirectly alter commit eligibility.

**Smallest fix.** Make `response` exactly `boundary_supported_rows x q`, zero-initialize it, and retain a separate exactly-sized `detector_response`; alternatively use one fully initialized emitted-row matrix with an explicit role/offset map. Add Eigen-NaN/MSan coverage and dimension assertions.

### D03-B — condensed-history fault columns do not enter the physical mode registry or census

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/integrity/hypothesis_generator.cpp`, `explicitlyMonitored`, lines 47–56, returns true only for `ExplicitMeasurement` and `PendingExplicit` blocks.
- The same file, mode generation at lines 652–943, filters UWB occurrences at lines 688–690 and 731–735 and IMU blocks at lines 812–816. It builds raw group maps only from explicit blocks and never consumes `history.response`, `history.detector_response`, or `history.column_ids`.
- The census at lines 855–943 is derived only from the generated `out.modes`.
- `src/uwb_imu_pl/integrity/integrity_monitor.cpp`, lines 2133–2142, passes those modes to `buildExactCoverageCertificate`.
- `src/uwb_imu_pl/integrity/coverage_envelope.cpp`, `buildExactCoverageCertificate`, lines 240–247, records `modes.size()` and marks the certificate exact/complete without reconciling it against a separately constructed expected physical scope.

**Smallest reproduction.** Let a persistent historical fault survive only through `T_b=[1]`, `F_b=[1]`, with one `column_id` and no explicit current block. `explicitlyMonitored(BoundaryInput)` is false, so zero physical modes are generated. The certificate reports `exact_count=0, complete=true`, although the expected physical count is one.

**Expected invariant and violation.** The independently resolved physical fault scope, instantiated raw parameters, condensed history columns, mode registry, hypothesis census, and evaluated hypotheses must reconcile. Explicit and historical portions of one persistent/ramp fault must be one physical parameter, not omitted or double counted. The current certificate proves only that it counted its own incomplete vector.

**Downstream consumers.** Coverage certificate, hypothesis evidence, plausible set, action construction, candidate fault Gram/PL, risk allocation/ledger, recovery selection and commit decision.

**Effects.** The all-in residual can contain the historical corruption while the hypothesis detector/profile and PL omit its response. This directly affects detector interpretation, FDE recovery, PL and HMI accounting; an untreated fault can contaminate navigation and lead to a wrong commit.

**Smallest fix.** Resolve `history.column_ids` into physical persistent/ramp modes and construct each boundary map as the stacked `[T_b c; F_b c]`; merge its explicit and historical portions under one physical ID. Build the expected physical census independently and fail closed with `UNREPRESENTED_MODE` on any mismatch.

### D04 — the online post-FDE detector silently changes from dual-channel to pooled

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/integrity/integrity_monitor.cpp`, candidate configuration at lines 2565–2580, sets `materialize_dense_oracle_fields=false`; the candidate is passed directly to `evaluateCandidate` at lines 2680–2703.
- `include/uwb_imu_pl/estimation/rank_update_kernel.hpp`, `CandidateEvaluation`, lines 57–97, defaults the retained Jacobian/residual views to empty; its getter does not lazily reconstruct them.
- `src/uwb_imu_pl/estimation/rank_update_kernel.cpp`, `RankUpdateEvaluator::evaluate`, lines 548–599 and 743–749, materializes retained rows only when the flag is true.
- `src/uwb_imu_pl/integrity/joint_window_detector.cpp`, `JointWindowDetector::evaluateCandidate`, lines 185–258, begins from pooled `baseResult` and performs the dual split only when retained rows are present. Missing rows leave a valid pooled result.
- `src/uwb_imu_pl/integrity/protection_level_v2.cpp`, `dualChannelAxisBounds`, lines 43–129, sets `legacy_single_channel = !channel_split_valid`; `computeShared`, lines 360–393, requires detector numerical validity but not `channel_split_valid`.
- The candidate carrier also lacks `history.constant_offset`; merely setting the materialization flag would not reproduce the all-in statistic exactly.

**Smallest reproduction.** At false-alarm probability `10^-6`, `chi2_1=23.92812698` and `chi2_2=27.63102112`. Use rows

```text
H = [1; 1; 0],  z = [0; 0; 5].
```

The pooled problem has rank 1, dof 2 and `T=25`, so it passes `25 < 27.631`. The third detector-only history row has rank 0, dof 1 and `T_h=25`, so the dual event rejects `25 > 23.928`. An empty retained-row carrier selects the first answer.

**Expected invariant and violation.** All-in detection, post-FDE detection and PL must use the same accepted event `T_c<=tau_c AND T_h<=tau_h`, including the history constant and explicit row roles. Missing active-v6 certificate material must be invalid, not an implicit pooled success.

**Downstream consumers.** Candidate eligibility, PL dual-channel bound, plausible/action selection, publication status and transaction commit.

**Effects.** Direct effects on detector result, FDE recovery, PL and risk event; the selected recovery state and committed plan can therefore change. Navigation is affected through the selected action.

**Smallest fix.** First restore a correctness reference path that materializes complete retained rows plus explicit current/history roles and constant offset. Then replace it with an immutable `CandidateDetectorCertificate` containing channel statistics, rank/dof, accepted-event ID and noise/linearization identity. Missing certificate is invalid; pooled operation remains an explicit legacy/offline mode only.

### D05 — `risk_budget_valid` is an allocation audit, not complete HMI closure

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/config/integrity_config.cpp`, `IntegrityConfigLoader::load`, lines 1104–1120, performs an anchor-oriented startup allocation check; it does not use the resolved mature current/history/IMU/pair census.
- `src/uwb_imu_pl/integrity/risk_budget_audit.cpp`, `axisTailSplit`, lines 61–99, defines the per-mode charge as `max(alpha,beta)`.
- `buildRiskLedger`, lines 102–275, computes the miss excess but labels it “reported, not charged” at lines 143–190. Escape, omitted-model, envelope and selection terms remain unvalidated/not implemented at lines 192–249. `charged_total` contains only validated terms at lines 251–274.
- `auditRiskBudget`, lines 278–320, sums allocations and configured terms but not the miss excess.
- `src/uwb_imu_pl/integrity/protection_level_v2.cpp`, `computeShared`, lines 387–393, exports that audit as `risk_budget_valid`; `src/uwb_imu_pl/integrity/fde_manager.cpp`, lines 74–79, gates on it. The fuller ledger assembled in `integrity_monitor.cpp`, lines 2157–2204, is diagnostic rather than a closing gate.
- `config/fde_joint_order2.yaml`, lines 49–83 and 115–125, supplies the relevant census policy, priors, `p_md`, cap and risk values.

**Smallest reproduction.** With the unchanged configuration, the available hypothesis allocation is

```text
4e-5 - 3*1e-5 - 1e-7 = 9.9e-6.
```

Charging the code's own `pi * max(alpha,p_md)` rule yields:

| Profile | Count | Equal allocation per mode | Complete max-tail total |
|---|---:|---:|---:|
| UWB-1 | 264 | `3.75e-8` | `5.65e-5` |
| IMU-1 | 60 | `1.65e-7` | `4.00e-5` |
| Joint-1 | 324 | `3.0555556e-8` | `5.8333333e-5` |
| Joint-2 | 16,164 | `6.1247216e-10` | `2.155e-4` |

For Joint-2 this includes 264 UWB modes at prior `1e-4` and 15,900 IMU/pair modes at prior `1e-5`, all with `p_md=1e-3`. It still excludes other unvalidated escape/selection channels.

**Expected invariant and violation.** A boolean named/used as risk-budget validity must mean that every nonzero or unknown event is charged by a validated bound and the same total closes. The current boolean means only that allocated slots and configured validated terms fit. Unknown terms do not force failure.

**Downstream consumers.** PL validity, FDE manager eligibility, integrity status/publication, run logs and commit gating.

**Effects.** It does not change the navigation solve or detector statistic. It can admit recovery/commit and a finite PL under an incomplete HMI bound. The code's separate `formal_eligible=false` mitigates a formal publication claim, but does not repair the misleading gate semantics.

**Smallest fix.** Split `allocation_valid`, `complete_bound_closes`, `all_terms_validated`, and `formal_eligible`. Charge detector miss, escape/omission and selection terms in one ledger; an unknown term fails the protected-output gate. Keep the existing budgets and thresholds unchanged, and label startup checking as a precheck rather than a complete audit.

### D06 — action risks are merged before the code has proof that their failure events are shared

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/integrity/fde_manager.cpp`, `FdeManager::select`, lines 231–300, groups eligible actions in `by_class` using only their plausible hypothesis IDs at lines 239–266. It keeps one class charge and passes only those classes to the guarantee grouping at lines 267–275. The generated `reference_certificate_id` is the event-class hash; flags for reference/event/time/output are set true while `triangle_transfer_evidence` is false at lines 251–259.
- `src/uwb_imu_pl/integrity/fde_post_selection.cpp`, `buildGuaranteeGroups`, lines 134–228, correctly requires a genuinely shared reference, accepted event, time and quantity plus triangle-transfer evidence at lines 164–205. Non-shareable actions become singleton charges at lines 207–225—but the caller has already discarded their distinct events.

**Smallest reproduction.** Two eligible actions have the same plausible IDs, each risk charge `3e-5`, total budget `4e-5`, and no transfer proof. Passing both directly to `buildGuaranteeGroups` makes two singleton events, `6e-5`, and fails. Premerging by ID keeps `max(3e-5,3e-5)=3e-5` and passes.

**Expected invariant and violation.** Data-dependent alternatives may share risk only when a numeric common-reference certificate proves the same accepted event/time/output and `L_a >= L_ref + |p_a-p_ref|`. Fault labels are not failure events. The caller bypasses this proof.

**Downstream consumers.** Selection-risk ledger, chosen recovery action, published PL/status and transaction commit.

**Effects.** Direct effects on FDE winner/availability, PL selection guarantee, HMI accounting and commit. Navigation can change through the selected action; detector statistics themselves are not changed.

**Smallest fix.** Pass every original eligible action and its actual numeric certificate to the guarantee layer. Do not premerge on plausible IDs. Share only through a valid common-reference transfer; otherwise charge the alternatives, reconciled with the already consumed base risk ledger.

### D07-A — exceptions after backend mutation are outside the transaction guard

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`, `IncrementalUwbImuEstimator::commitEpoch`, lines 2238–2524. Only `backendUpdate` is protected and poisons on exception at lines 2328–2338. `tx.backend_mutated=true` is set at line 2339. State/marginal queries at lines 2351–2355, estimate extraction at lines 2360–2371, ledger/slot binding at lines 2373–2418 (including an explicit throw at lines 2394–2395), and metadata/history pruning at lines 2423–2489 are outside that protection.
- `discardEpoch`, lines 2527–2532, rejects a transaction after `backend_mutated` is true.
- `src/uwb_imu_pl/integrity/integrity_monitor.cpp`, lines 3273–3291, catches commit failure and attempts ordinary discard when a pending transaction remains; that discard can itself throw after mutation and mask the original failure.
- `tools/run_realtime_integrity.cpp`, lines 497–539, has an outer reset/reinitialization path, which mitigates the ROS executable but does not establish library/API transaction safety.

**Smallest reproduction.** State machine: `backendUpdate` mutates graph `M`; `queryCurrentState` then throws. No post-mutation catch sets poison or creates a receipt. The caller calls `discardEpoch`, which rejects because `backend_mutated=true`. The graph changed, the transaction remains incomplete, and the backend is not explicitly poisoned.

**Expected invariant and violation.** From the first backend mutation through issuance of a complete receipt, either the transaction finishes atomically at the metadata/API level or every exception produces an explicit poisoned/committed-but-unprotected terminal state. It must never look discardable or reusable. Current protection ends too early.

**Downstream consumers.** Estimator graph/version/history, factor ledger, bridge/health state, future snapshots, current state publication and transaction callers.

**Effects.** Direct effects on backend/navigation state and commit semantics; later detector/FDE/PL can consume a graph whose metadata is inconsistent. Risk is affected indirectly.

**Smallest fix.** Add a mutation guard spanning the first mutation through immutable receipt creation. Prevalidate/preallocate before mutation; after mutation, any exception poisons and reports committed-but-unprotected. Stage bridge, health, cache, history and version changes, applying them only with the receipt. Do not pretend to roll back iSAM mutation.

### D07-B — the frozen candidate PL is not transferred to the nonlinear committed mean

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `src/uwb_imu_pl/estimation/rank_update_kernel.cpp` computes a frozen linear candidate `state_increment` and covariance.
- `src/uwb_imu_pl/integrity/integrity_monitor.cpp`, successful selection/commit path lines 3000–3190, constructs a factor-operation plan, performs nonlinear estimator commit, obtains the estimator's current state, and copies the selected candidate PL into the output at lines 3166–3182. It does not compare the committed protected position with the frozen PL reference position.
- `include/uwb_imu_pl/estimation/epoch_transaction.hpp`, `CommitReceipt`, lines 138–156, has identity/version fields but no protected-reference mean or reference-transfer certificate.

**Smallest reproduction.** A one-dimensional candidate certificate is centered at `p_ref=0` with `L=0.5`, covering truth `0`. The nonlinear commit emits `p_commit=1` while retaining `L=0.5`; the published interval `[0.5,1.5]` no longer covers truth. Triangle transfer requires at least `L'=0.5+|1-0|=1.5`.

**Expected invariant and violation.** PL must be centered on the same time, frame, body origin and numeric mean that is published after commit, or carry a proven conservative transfer. Matching transaction/solution IDs is not numeric equivalence.

**Downstream consumers.** Final state/PL packet, HMI decision, recovery output, transaction receipt and external users.

**Effects.** The nonlinear navigation state can differ from the certified reference; PL and HMI accounting are directly affected. Detector statistics are not changed. Recovery output and committed publication can be falsely bound.

**Smallest fix.** Put the protected reference mean and coordinate identity in the candidate certificate and receipt. After nonlinear commit, compute the same protected coordinates and add componentwise `|p_commit-p_ref|`, or recompute/validate a final-state certificate before protected publication.

### D08 — state, covariance, fault-nullspace and PSD decisions do not share one numerical certificate

**Classification: CONFIRMED.** Harmful occurrence frequency is not quantified, but the unsafe decision rules are present.

**Current implementation and exact locations.**

1. `src/uwb_imu_pl/estimation/integrity_window_snapshot.cpp`, `solveFrozenInformation`, lines 61–110, has residual/forward checks and fallback for a frozen state solve. `src/uwb_imu_pl/estimation/rank_update_kernel.cpp`, `CandidateEvaluation::covarianceTimes`, lines 269–290, directly uses LLT/corrections/reference storage without the same certificate. `ProtectionLevelV2::computeShared`, lines 438–451, consumes that path and chiefly tests finiteness/positive diagonal.
2. `src/uwb_imu_pl/integrity/hypothesis_generator.cpp`, `finalizeEffectiveBasis`, lines 58–135, drops basis directions using small absolute `D` and protected-response tolerances at lines 122–130, without a fault-amplitude bound. `src/uwb_imu_pl/integrity/integrity_monitor.cpp`, `projectPostActionModes`, lines 1022–1080, erases a projected mode when `maxAbs<=1e-12` at lines 1040–1059.
3. `src/uwb_imu_pl/integrity/dual_channel_detector.cpp`, `computeDualChannelBound`, lines 116–260, combines QR rank and SVD singular values at lines 211–251 without a unified symmetry/PSD negative-eigenvalue certificate. SVD singular values cannot prove that a symmetric Gram is PSD.
4. `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp`, lines 66–107 and 1224–1305, clamps negative eigenvalues/classifies nullspace with tolerances that differ from the PL reconstruction in `src/uwb_imu_pl/integrity/protection_level_v2.cpp`, lines 508–553. Production PL does not consume the same harmless-nullspace proof.

**Smallest reproductions.** A map `D=1e-13` is erased by the absolute gate, but with an unbounded fault `f=1e13`, `Df=1`; it is not harmless. Separately, `W=diag(1,-1e-6)` has finite positive SVD singular values and full SVD rank although it is indefinite, so an SVD-only rank/null test cannot certify a covariance/fault Gram.

**Expected invariant and violation.** State, every covariance/fault/bridge RHS, PSD/rank/nullspace and `G ker(W)` must use one square-root operator, tolerance policy and proof ID. A fault direction may be removed only if structurally zero/equivalent or if an explicit amplitude bound times truncation remainder is charged. The current modules can disagree and can classify small-but-unbounded response as absent.

**Downstream consumers.** Hypothesis census/evidence, post-action mode projection, candidate covariance, post-detector, PL, risk ledger and selection.

**Effects.** It can change monitorability, candidate eligibility/recovery, PL and HMI. Detector effects occur where channel/fault response classification diverges. Navigation state may remain finite while its covariance/PL is uncertified; selection can indirectly change commit.

**Smallest fix.** Route all RHS through one certified square-root solve with residual/forward checks and QR/SVD fallback. Replace absolute-small pruning with exact structural-null handling or a charged amplitude/remainder proof. Use one symmetric PSD eigencertificate and common rank/null/`G ker(W)` classification, and make PL consume that same certificate. Do not clamp a negative or indeterminate direction into safety.

### D09 — action enumeration is truncated without a winner/dominance proof

**Classification: CONFIRMED under the review's P0 condition; otherwise P1.** The condition applies to the current path because selection proceeds as if the supplied action set were complete and no `SEARCH_INCOMPLETE` state is propagated.

**Current implementation and exact locations.**

- `src/uwb_imu_pl/integrity/hypothesis_generator.cpp`, `ensureActionEntities`, lines 959–1005, stops adding actions at `actions.size()==128` at line 999.
- `actionsForPlausibleSet`, lines 1007–1155, truncates the returned vector to the configured maximum at lines 1150–1154. A full-union fallback is constructed only for the limited branch at lines 1122–1149 and is not a general dominance proof for all truncated actions.
- `config/fde_joint_order2.yaml`, line 83, sets `max_candidate_count: 128`.
- Downstream census/selection records evaluated actions, but carries neither the number/identity of omitted actions nor a proof that none can win.

**Smallest reproduction.** Generate 130 distinct admissible actions with cap 128 and place the only action satisfying post-detector/PL at index 128. The loop never evaluates it. The last retained action need not dominate the two omitted ones.

**Expected invariant and violation.** Hypothesis coverage must not be reduced. Action pruning is safe only with an exact duplicate/dominance/winner certificate; otherwise resource exhaustion must produce a fail-closed `SEARCH_INCOMPLETE`, not a complete-search result.

**Downstream consumers.** Candidate preparation/solve, recovery winner, selection-risk accounting, publication and commit.

**Effects.** It can lose a recoverable navigation action, change FDE outcome and commit, and invalidate selection completeness/HMI accounting. It does not change the detector or PL of actions that were actually evaluated.

**Smallest fix.** Stream or batch all actions through the existing bounded worker resources, retaining only provably sufficient winner state and exact deduplication. Record generated/evaluated/omitted counts. If the exact search cannot finish, fail closed as `SEARCH_INCOMPLETE`; do not increase the cap as a correctness argument.

### N01 — IMU noise semantics and shared trapezoid samples lack a qualification artifact

**Classification: DESIGN-DEPENDENT.** This is a P0 qualification blocker, not a proven formula defect.

**Current implementation and exact locations.**

- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`, estimator constructor lines 436–450, squares configured accelerometer/gyro sigmas into GTSAM preintegration covariance parameters.
- The same file, IMU preparation/integration lines 720–748, trapezoidally averages adjacent samples; consecutive intervals share a sample.
- `config/fde_joint_order2.yaml`, lines 21–25, supplies the sigma values, but no checked-in evidence here establishes whether they are continuous-time densities, per-sample deviations, already filtered effective values, or conservative overbounds.

**Smallest reproduction.** If independent samples have variance `sigma^2` and interval noise is `e_k=(n_k+n_{k+1})/2`, then `Var(e_k)=sigma^2/2` and `Cov(e_k,e_{k+1})=sigma^2/4`. Treating interval averages as independent requires a derivation or conservative bound; it is not implied by units alone.

**Expected invariant and current status.** The declared stochastic model must cover the actual filtered/shared-sample process, with a calibration ID and overbound evidence. The code may be correct if the configured sigma already encodes an equivalent/overbounded process, but that design fact is not present, so neither confirmation nor exoneration is possible.

**Downstream consumers.** Nominal FGO covariance/state, all detectors, fault-space whitening/Gram, candidate covariance, PL and risk calibration.

**Effects.** Potentially all numerical protection outputs and navigation. Transaction logic is affected only through the decisions it receives.

**Smallest fix.** Do not tune values in this pass. Specify the sigma semantics, attach calibration/overbound identity, and derive either the correlated discrete covariance or a proven conservative independent approximation for the trapezoid filter.

### D11-A — ROS Odometry publishes world velocity in a child-frame field

**Classification: CONFIRMED.** This is the P1/navigation-interface portion of D11, included because the review labels D11 P0/P1 and the output is a downstream consumer of commit.

**Current implementation and exact locations.**

- `tools/run_realtime_integrity.cpp`, `RealtimeIntegrityNode::publish`, lines 618–790, sets `odom.child_frame_id` to the body frame at line 641 and copies `output.state.velocity_world` directly to `odom.twist.twist.linear` at lines 649–651.
- ROS `nav_msgs/Odometry.msg` defines twist in `child_frame_id`.

**Smallest reproduction.** For body yaw `+90 deg` and world velocity `(1,0,0)`, body-frame velocity is `R^T v=(0,-1,0)`. The code publishes `(1,0,0)` while labelling it body-frame.

**Expected invariant and violation.** Odometry twist and its covariance use the child frame. The compound output may separately retain an explicitly named world-frame velocity.

**Downstream consumers and effects.** ROS odometry consumers receive a wrong navigation quantity. This path does not feed the detector, FDE, PL, HMI ledger or transaction decision.

**Smallest fix.** Rotate world velocity and its covariance into the child/body frame for Odometry; keep the world-frame value only in a field whose contract says world frame.

### D11-B — the final deadline mutation is made on a publication copy and is absent from the log packet

**Classification: CONFIRMED.** This is the P0 publication/accounting portion of D11.

**Current implementation and exact locations.**

- `tools/run_realtime_integrity.cpp`, `processUwb`, lines 428–437, stores `const auto output`, calls `publish(output, metrics)`, then logs the original `output`.
- `publish(IntegrityOutput output, ...)`, lines 618–790, takes `output` by value and changes deadline/status fields at lines 627–637. ROS messages are built from that local copy; the caller and logger retain the pre-mutation object.
- Finish time is sampled near publication entry at line 620, before message construction and ROS publish calls at lines 787–789, so the documented endpoint excludes part of final delivery work.

**Smallest reproduction.** `O.deadline=false; publish(O by value) { copy.deadline=true; } log(O);` makes the subscriber observe `true` and the audit log observe `false` for the same attempt.

**Expected invariant and violation.** One immutable final packet, produced after the final deadline gate and with a precisely defined timing endpoint, must feed ROS and persistent logging. Every attempt must be journalled consistently. The current outputs diverge.

**Downstream consumers.** ROS integrity subscribers, run logger, campaign/replay analysis, safety monitor and transaction audit trail.

**Effects.** No change to navigation, detector, recovery or PL calculation. It changes externally observed deadline/integrity status and prevents reliable HMI/transaction-accounting reconstruction; the backend commit itself is unchanged.

**Smallest fix.** Make finalization return one immutable packet (or mutate once before both sinks), then pass that exact packet to publisher and logger. Record arrival, compute complete, packet complete, publish-call and publish-return times so the deadline contract is explicit.

### D12 — existing history tests are self-consistency checks, not an independent raw-factor oracle

**Classification: CONFIRMED.**

**Current implementation and exact locations.**

- `test/test_history_summary_pipeline.cpp`, lines 175–223, recomputes pooled quantities from the already constructed `window.H/window.z`.
- The same test, lines 475–610, rebuilds results from the shipped summary/carrier rather than assembling the unique raw nonlinear/linearized factors independently.
- Its ledger checks at lines 377–411 verify group bookkeeping outside the condensed block; they cannot observe the duplicated rows internal to D01.
- The principal synthetic data around lines 46–110 is near-zero/clean, which does not expose the D02 nonzero-RHS sign reversal.
- `test/test_integrity_v2.cpp`, risk tests around lines 2898–2905, explicitly observe a nonzero miss channel while accepting that it is “reported, not charged”; the test therefore codifies D05 rather than closing it.

**Smallest reproduction.** Let builder `B(raw)` duplicate a row or negate its RHS, then define the test oracle as `T=||B.z-B.H*solve(B)||^2`. The test is algebraically self-consistent and passes even though `B(raw)` is not equivalent to the raw objective. The rebuilt baseline tests do exactly this class of downstream consistency check.

**Expected invariant and violation.** The oracle must be independent of the code under test and begin with a list of unique raw factors/covariance blocks. For arbitrary nonzero state/fault probes it must compare objective (including constants), optimum, covariance, detector split, fault Gram, dof, mode/action census, PL reference and commit transfer. Current tests start after the two earliest faults have already occurred.

**Downstream consumers.** This is an evidence defect rather than a runtime data edge. It permits regressions in every chain stage to be reported as passing and blocks a defensible correctness release.

**Effects.** Indirectly all six requested impact categories: defects in any can escape the suite. It does not itself numerically alter a runtime result.

**Smallest fix.** Add three layers: (1) independent raw unique-factor dense oracle, (2) correct dense end-to-end FDE reference with no action cap, and (3) optimized online equivalence tests. Use nonzero RHS, correlated covariance, historical faults across the lag boundary, multiple actions and injected post-mutation exceptions. Persist failure snapshots with raw row IDs and proof hashes.

## 5. Rebuilt test evidence and why it does not change the classifications

The four directly relevant targets were rebuilt from the baseline checkout with:

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl \
  --target test_history_summary_pipeline test_dual_channel_detector \
           test_fde_post_selection test_integrity_v2 -j2
```

All selected existing tests passed:

| Binary/filter | Result |
|---|---:|
| `test_history_summary_pipeline` | 9/9 passed |
| `test_dual_channel_detector` | 5/5 passed |
| `test_fde_post_selection` | 3/3 passed |
| selected transaction/fault/FDE/numerics/PL/risk suites in `test_integrity_v2` | 18/18 passed |

A separate complete CTest run on the same baseline discovered an additional
evidence-harness failure that the historical summary did not count:

| Complete-suite view | Result |
|---|---:|
| CTest targets | 25/26 passed |
| Actual cases | 273/274 passed; 1 Python error |
| Failure | `test_round2_tools`: v5 manifest with a v6 `integrity.csv` header |

`catkin_test_results build/uwb_imu_pl --all` reported 442 passing tests, but
that view duplicates the 221 GTest/rostest cases and omits the 53 Python cases
registered directly with CTest.  The project gate therefore uses the complete
CTest exit code and retains direct Python failures.

These results are useful connection evidence, but not counterexamples to the findings:

- history tests consume the already duplicated/sign-flipped construction;
- the history carrier test printed `333` emitted rows but only `15` boundary-supported rows, exposing D03-A while its assertions still passed;
- the dual-channel test proves pooled and split decisions differ, but does not exercise the production `materialize_dense_oracle_fields=false` handoff;
- post-selection tests validate the strict callee, not the caller's earlier premerge;
- risk tests accept the deliberately uncharged miss channel.

No sanitizer, hardware replay, pinned campaign artifact, or calibration package was available in this pass. Those gaps affect magnitude/frequency claims, not the static/algebraic confirmations above.

## 6. Required correctness order before performance work

No fixes are implemented here.  The controlling seven-step P0 order is the
one in Section 2.2:

1. history raw-factor equivalence (D01, D02, D03-A/B and the first independent
   oracle slice);
2. the dual-channel candidate contract (D04);
3. the unified numerical certificate (D08);
4. complete risk and selection events (D05/D06);
5. transaction, committed-reference and final-packet atomicity (D07/D11);
6. action-search completeness or explicit `SEARCH_INCOMPLETE` (D09);
7. corrected exhaustive evidence and qualification closure (D12/N01).

Each item ends in its own commit and immutable golden tag.  P1 remains closed
until all seven tags exist and the completion record in Section 2.5 marks all
seven rows `GOLDEN`.  This order does not reduce hypothesis coverage, loosen
thresholds, alter risk budgets, disable history or post-FDE detection, or turn
a fail-closed outcome into success.

## 7. Workspace changes made by this pass

Only this report was added and converted into the project audit master table
under `docs/review/`. No source, header, configuration, test, threshold, risk
budget or existing review file was modified.
