# REFACTOR-GATE-05R-A runner contract failure attribution

Date: 2026-09-13  
Scope: diagnostic only  
Verdict: **all three failures first arise from the Gate-02 fixed-sensor-noise
transition; Gate 05 is not causal**

## Decision

T04, T06 and T08 are all classified as
`EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT`.

Gate 02 intentionally changed the paper path from selected-gap-adaptive range
noise to `FIXED_SENSOR_SIGMA_V2`. The three runner fixtures retained the same
`sigma_range: 0.1` and the same expectation that their historical solves would
succeed. On this fixture, the transition leaves every observation and graph
topology decision unchanged but increases the aggregate UWB precision weight
by 5.069739093569201 times. The old tests were not actually exercised by the
Catkin aggregate used in the Gate-02/03/04 reports, because the runner tests are
registered with plain CMake `add_test`, not a Catkin xUnit test macro.

The current code with only the historical adaptive-sigma materialization
restored in a temporary shared library passes all three current runner contract
scripts. Conversely, a temporary reconstruction of the immediate pre-Gate-05
single-file runner, linked to the fixed-sigma Gate-04 library, fails all three
contracts at the same stages as Gate 05. This isolates Gate 02 as the first
causal transition and rejects a Gate-03, Gate-04 or Gate-05 implementation bug
for these fixtures.

This diagnosis does not authorize a repair. No checked-in source, configuration,
threshold, solver setting or test expectation was changed.

## Evidence boundary and history reconstruction

The five refactor gates are uncommitted working-tree stages on top of commit
`e9d821e960f725e064ca32fd9e475e529ac5f501`; there is no commit object for each
individual gate. History was therefore restricted as requested:

1. detached `e9d821e9` represents pre-Gate-02;
2. Gate-02 is the fixed-sigma source transition documented by its gate report;
3. Gate-03 adds input integrity and baseline-only robust protection;
4. Gate-04 adds the post-solve scientific certificate;
5. a reconstructed monolithic runner represents immediate pre-Gate-05;
6. the current thin-entry runner represents Gate-05.

The reconstruction was made in `/tmp/uifgo_gate05ra_pregate05.LykUVf` by
compiling the moved application orchestration back as `main` and replacing the
single `PrepareEstimatorCore` call with that function's exact inline sequence:
input plan, selection mask, keyframes, initializer, graph build, paper pose-prior
replacement and factor metadata. It was linked against the current Gate-04/05
library. The reconstruction changes application structure only; it does not
change the fixed-sigma scientific inputs.

The already-saved T08 true pre/post Gate-05 snapshot independently validates
the reconstruction premise. Ten stable artifacts plus
`stage2_refit_status.json` are byte-identical between
`/tmp/uifgo_gate05_before.mmpDKe/gate05` and
`/tmp/uifgo_gate05_after.D4Qou1/gate05`; `run_status.json` differs only in its
declared wall-clock fields.

### Why the earlier green gate reports did not establish runner success

`CMakeLists.txt` registers T04/T06/T08 through `add_test`. The Gate-02/03/04
reports recorded `catkin test` / `catkin run_tests` aggregate counts (480, 484
and 488), whose result logs contain only Catkin xUnit aggregation. They do not
contain execution evidence for these plain Python CTest registrations. The
first full `ctest --output-on-failure` audit at Gate 05 exposed the three red
contracts. Therefore those aggregate counts are not used as PASS points here.

## Last PASS and first FAIL

| Runner | Last actual PASS | First causally attributable FAIL | Immediate pre-Gate-05 | Current Gate-05 |
|---|---|---|---|---|
| T04 | detached pre-Gate-02 `e9d821e9`; Python contract exit 0 | Gate-02 `FIXED_SENSOR_SIGMA_V2` transition | contract exit 1, Stage-2 `CONDITIONAL_LM_MAX_ITERATIONS` | same |
| T06 | detached pre-Gate-02 `e9d821e9`; Python contract exit 0 (primary runner exit 1 was the contract-valid partial result) | Gate-02 `FIXED_SENSOR_SIGMA_V2` transition | contract exit 1, `AUTOMATIC_DISCOVERY / CONDITIONAL_LM_MAX_ITERATIONS` | same |
| T08 | detached pre-Gate-02 `e9d821e9`; Python contract exit 0 | Gate-02 `FIXED_SENSOR_SIGMA_V2` transition | contract exit 1, Stage-2 `CONDITIONAL_LM_MAX_ITERATIONS` | same |

No preserved Gate-02-only binary exists. Attribution to that transition is
nevertheless causal rather than chronological guesswork: a current
Gate-03/04/05 build with only `paper_input.cpp` temporarily changed back to the
pre-Gate-02 adaptive sigma passes T04, T06 and T08 (`0/0/0`), while the
pre-Gate-05 monolithic reconstruction with fixed sigma fails them (`1/1/1`).

## Exact common scientific-input comparison

The comparison used the same bag SHA-256
`038e8158c07da81d684e39d97f79d053c0c6603f48cad922e583e8096d1ba991`.
All three fixtures have the same input shape and the same Gate-02 delta:

| Quantity | pre-Gate-02 | Gate-02 through Gate-05 |
|---|---:|---:|
| raw observations | 627 | 627 |
| selected observations | 64 | 64 |
| selected `obs_id` sequence | reference | byte/value identical |
| keyframe/state count | 8 | 8 |
| base graph factor count | 74 | 74 |
| UWB factor count | 64 | 64 |
| stale repeats | policy absent; no exact repeats in data | 0 |
| correlation groups with size greater than one | policy absent | 0 |
| initial Values hash | `t08values-sha256:51a986...c46bd` | identical |
| graph-linearization hash | `t08graphlin-sha256:2ff447...a2aee` | `t08graphlin-sha256:a029fb...09e7` |
| selected sigma range, m | 0.1 to 0.34894357511995083 | exactly 0.1 |
| selected sigma mean, m | 0.3170075403154951 | 0.1 |
| observations whose sigma changed | — | 56 / 64 |
| sum of scalar UWB precisions, `sum(1/sigma^2)` | 1262.3923799388792 | 6399.999999999999 |
| aggregate precision ratio | 1 | 5.069739093569201 |
| maximum per-factor precision ratio | 1 | 12.176161861749277 |

Thus the first differing scientific input is the UWB Gaussian noise scale. It
is not observation membership, duplicate suppression, state density,
initialization, support-file identity, factor count or solver configuration.

The failed stages do not use a separately optimized baseline trajectory:

- T04 and T08 initialize oracle Stage 2 directly from `initial`;
- T06 initializes automatic Stage 1 directly from the same `initial`;
- the initial Values hash is identical before and after Gate 02;
- a live segment amplitude is inserted as 0.0 when the input Values do not
  already contain its `C(s)` key. T04/T08 therefore start their single segment
  at 0.0 m; current T06 never reaches Stage 2.

The solver options are unchanged: conditional LM maximum 50, relative
tolerance `1e-6`, absolute tolerance `1e-8`; T04/T08 Stage-2 maximum outer
iterations 20; T06 discovery and Stage-2 maximum outer iterations 50; refit
relative objective tolerance `1e-8`, scaled-step tolerance `1e-6`, projected
gradient/KKT tolerance `1e-8`, and navigation stationarity tolerance `1e-6`.

## Robust-baseline scope check

Robust-factor propagation is **rejected**.

- `src/paper_methods.cpp::RobustGraph` wraps UWB noise with
  `PaperRobustNoise` only inside the robust Huber/Cauchy branch of
  `RunPaperBaseline`.
- The application passes the original physical `graph`, not a baseline
  result's `final_graph`, into `AutomaticSupportProvider`, `SegmentRefitter`
  and `FinalInferenceEngine`.
- `SegmentRefitter` copies non-candidate UWB factors from that original graph
  and rebuilds candidate factors with `MakeSegmentUwbFactor` or
  `MakeFixedOffsetUwbFactor` using `measurement.sensor_sigma`.
- `RangeFactorMatchesExpected` explicitly requires both actual and expected
  range noise models to cast to `gtsam::noiseModel::Gaussian` and checks
  whitening `R(0,0) == 1/sigma`.
- recoverability and final inference consume the Stage-2/refit graph and rebuild
  from the same frozen raw graph; they never receive a robust baseline graph.

T04/T06/T08 do not invoke a robust baseline at all. Their base, Stage-1,
Stage-2, recoverability and final-inference range factors are plain Gaussian.
Gate 03 therefore did not introduce the observed solve change.

## Solver-iteration behavior

At the normal 50-iteration limit, the shared first conditional navigation solve
starts at objective `15862.745010999352`. T06's retained failure diagnostic
ends at `26.146366713196077` after 50 accepted updates and 54 rejected lambda
trials (`104` inner trials, lambda `0.1`). Its last absolute decrease is
`5.9071432239932165e-05`; relative decrease is
`2.2592545813531695e-06`, narrowly above the existing `1e-6` criterion. T04 and
T08 execute the same first conditional raw-navigation graph at zero segment
amplitude; their failure artifact does not export the last inner Values or
objective, but the isolated 200-iteration replay confirms the same initial
objective and deterministic continuation.

The immediate inner failure is therefore **PATTERN 4** for all three: finite,
strongly decreasing, non-divergent, and just short of the generic LM decrease
criterion at iteration 50. Stationarity is not evaluated on the aborted result,
so no stationarity PASS is inferred.

The isolated replay changed only `optimizer.lm_max_iter` from 50 to 200 in
temporary configuration copies under `/tmp/uifgo_gate05ra_extended`. It did
not change any checked-in file or outer-iteration limit. The first conditional
LM then reached its existing generic condition at iteration 63, but none of the
three end-to-end contracts reached the existing joint stop conditions:

| Runner | Extended result | Last objective evidence | Last stop evidence | Pattern after inner LM proceeds |
|---|---|---:|---|---|
| T04 | exit 1, Stage-2 `MAX_REFIT_ITERATIONS` at outer 20 | 25.165903897736914 | relative objective change `3.6364e-7`; scaled step `6.63999e-3`; navigation gradient `6.72609e-2` vs `1e-6`; KKT passes | PATTERN 2 |
| T06 | exit 1, Stage-1 `MAX_OUTER_ITERATIONS` at outer 50 | 10.473060088828067 | relative objective change `1.2891e-14`; scaled step `7.02272e-8`; KKT `9.66619e-9` passes `1e-8`; post-chain navigation gradient `4.92445e-6` vs `1e-6` | PATTERN 2 |
| T08 | exit 1, Stage-2 `MAX_REFIT_ITERATIONS` at outer 20 | 25.165903897736914 | identical T04 trace | PATTERN 2 |

For T06 the extended fixed-sigma active set stabilizes at 24 observations with
hash `sha256:c03fa2197...94219c8`; the historical adaptive-sigma solve converged
with 23 active observations, hash `sha256:80578165...9dc19cb1be`, and 17 frozen
segments. Additional inner iterations therefore do not merely reproduce the
historical support. The fixed-sigma solve remains finite and non-pathological,
but it plateaus with the navigation stationarity predicate false. This rejects
`SOLVER_ITERATION_BUDGET` as the primary root-cause label: raising only the
inner cap moves the failure outward rather than making the contracts valid.

## T04

1. **Last PASS:** detached pre-Gate-02 `e9d821e9`; contract exit 0 and primary
   runner exit 0.
2. **First FAIL:** Gate-02 fixed-sigma transition.
3. **Immediate pre-Gate-05:** reconstructed monolith primary/contract exit 1;
   `ORACLE_REFIT`, `CONDITIONAL_LM_FAILED /
   CONDITIONAL_LM_MAX_ITERATIONS`, zero completed refit outer iterations.
4. **Current Gate-05:** identical status and reason; primary/contract exit 1.
5. **Historical successful solve:** Stage-2 outer iterations 14; objective
   `1311.8406083746354 -> 5.1645403451937257`; termination
   `ALL_JOINT_STOP_CONDITIONS_SATISFIED`; 8 states, 74 final factors (64 UWB),
   25 Values; maximum exported position norm 3.8900681744464283 m.
6. **Support:** one `1:1` segment over `[1781510684.4,1781510688.4]`; support
   SHA-256 `8e2b67f2...ad56d0e4` before and after.
7. **Current failed solve:** initial conditional objective
   `15862.745010999352`; 50-iteration result is not exported as a valid stage
   result; the shared conditional diagnostic ends at `26.146366713196077`.
   Certificate and maximum position norm are `NOT_REACHED/NOT_AVAILABLE`.
8. **Extended replay:** first conditional LM converges at 63, then Stage 2
   reaches outer 20 with PATTERN 2 evidence above.
9. **Classification:** `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT`.
10. **Minimal repair recommendation:** retain fixed-sensor semantics and move
    the CSV/JSON serialization assertions to a deterministic fixed-sigma
    success fixture; retain this real sim-circle case as a separate explicit
    non-convergence regression. Do not restore adaptive sigma or declare
    `MAX_ITERATIONS` successful.

## T06

1. **Last PASS:** detached pre-Gate-02 `e9d821e9`; Python contract exit 0.
   Its primary automatic runner intentionally exited 1 with the
   contract-accepted `PARTIAL_STAGE2_OK_SCORE_UNAVAILABLE` status.
2. **First FAIL:** Gate-02 fixed-sigma transition.
3. **Immediate pre-Gate-05:** reconstructed monolith contract exit 1;
   `AUTOMATIC_DISCOVERY / CONDITIONAL_LM_FAILED /
   CONDITIONAL_LM_MAX_ITERATIONS`; Stage 2 not run.
4. **Current Gate-05:** identical stage, reason and exit.
5. **Historical successful Stage 1:** outer iterations 34; objective
   `1311.8406083746354 -> 3.1162352980016399`; final navigation gradient
   `9.063442717449632e-7`; 23 active observations, active hash
   `sha256:80578165...9dc19cb1be`; 17 segments, 13 short.
6. **Historical successful Stage 2:** outer iterations 32; objective
   `6.384958446542486 -> 2.610817183645759`; final navigation gradient
   `7.414973402786185e-7`; 8 states, 74 factors (64 UWB), 41 Values; maximum
   exported position norm 3.8861653664042763 m.
7. **Current failure:** first conditional solve reaches 50 iterations / 104
   inner trials and `26.146366713196077` from `15862.745010999352`; no support
   partition, bias refit, certificate or valid maximum position norm is reached.
8. **Extended replay:** first conditional LM proceeds at 63, but discovery
   reaches outer 50 with PATTERN 2 and a different 24-observation active set.
9. **Classification:** `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT`.
10. **Minimal repair recommendation:** retain fixed-sensor semantics and create
    a deterministic fixed-sigma automatic-discovery fixture whose success,
    support identity and Stage-2 behavior are frozen independently of this
    historical adaptive-sigma basin. Preserve this case as a negative
    convergence/conditioning regression.

## T08

1. **Last PASS:** detached pre-Gate-02 `e9d821e9`; contract and primary runner
   exit 0.
2. **First FAIL:** Gate-02 fixed-sigma transition.
3. **Immediate pre-Gate-05:** both the true saved snapshot and reconstructed
   monolith exit 1 at Stage 2 with `CONDITIONAL_LM_FAILED /
   CONDITIONAL_LM_MAX_ITERATIONS`, zero completed refit outer iterations.
4. **Current Gate-05:** identical failure; the true pre/post stable artifacts
   are byte-identical except declared timing fields in `run_status.json`.
5. **Historical successful Stage 2/final inference:** Stage-2 outer iterations
   14, objective `1311.8406083746354 -> 5.1645403451937257`; final recovery
   refit one iteration; no fallback; final graph 74 factors / 25 Values;
   covariance available; maximum position norm 3.8900681744959349 m.
6. **Support:** the same single oracle candidate file before/after, SHA-256
   `48ee93d5...d41b72a`; candidate group count 1 in the historical success.
7. **Current failure:** same conditional objective evidence as T04; Stage 3,
   final inference and the Gate-04 certificate are not reached, so no valid
   trajectory, covariance or maximum position norm is available.
8. **Extended replay:** first conditional LM proceeds at 63, then Stage 2
   reaches outer 20 with the same PATTERN 2 trace as T04.
9. **Classification:** `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT`.
10. **Minimal repair recommendation:** retain fixed-sensor semantics and run
    the strict final-artifact/identity assertions on a deterministic
    fixed-sigma success fixture; keep the present sim-circle input as a
    fail-closed Stage-2 regression. Do not weaken the solver certificate.

## Diagnostic commands and outputs

Representative commands actually executed:

```text
# Detached pre-Gate-02 build (isolated build/devel spaces)
catkin build uwb_imu_fgo --no-deps ...
python3 -B test/test_t04_runner_contract.py --runner <detached-runner> ...
python3 -B test/test_t06_runner_contract.py --runner <detached-runner> ...
python3 -B test/test_t08_runner_contract.py --runner <detached-runner> ...
# contract exit codes: 0, 0, 0

# Immediate pre-Gate-05 monolith reconstruction
python3 -B test/test_t04_runner_contract.py \
  --runner /tmp/uifgo_gate05ra_pregate05.LykUVf/uwb_imu_fgo_paper_runner ...
python3 -B test/test_t06_runner_contract.py --runner <same> ...
python3 -B test/test_t08_runner_contract.py --runner <same> ...
# contract exit codes: 1, 1, 1

# Current code plus temporary pre-Gate-02 adaptive-sigma object/library
LD_LIBRARY_PATH=/tmp/uifgo_gate05ra_adaptive/lib:<current> \
  python3 -B test/test_t0{4,6,8}_runner_contract.py ...
# individual contract exit codes: 0, 0, 0

# Isolated extended replay; only optimizer.lm_max_iter changed 50 -> 200
<current-runner> --config /tmp/uifgo_gate05ra_extended/t04/config.yaml ...
<current-runner> --config /tmp/uifgo_gate05ra_extended/t06/config/paper/config.yaml ...
<current-runner> --config /tmp/uifgo_gate05ra_extended/t08/config/paper/config.yaml ...
# primary exit codes: 1, 1, 1
```

Primary retained diagnostic outputs are under:

- `/tmp/uifgo_gate05ra_head_t04`, `..._head_t06`, `..._head_t08`;
- `/tmp/uifgo_gate05ra_pregate05_outputs/{t04,t06,t08}`;
- `/tmp/uifgo_gate05ra_current_t04`, `..._current_t06`, and
  `/tmp/uifgo_gate05_after.D4Qou1/gate05`;
- `/tmp/uifgo_gate05ra_extended/{t04,t06,t08}/out/extended`.

An early detached-run attempt that mixed the detached runner with the current
shared library exited `-11`; loader inspection identified the ABI mismatch and
that run was discarded. It is not used as numerical evidence.

## Final attribution

| runner | last_pass | first_fail | root_cause | minimal_repair | scientific_behavior_change_required |
|---|---|---|---|---|---|
| T04 | pre-Gate-02 `e9d821e9` | Gate-02 fixed sensor sigma | `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT` | fixed-sigma deterministic serialization success fixture; preserve current case as fail-closed regression | no |
| T06 | pre-Gate-02 `e9d821e9` | Gate-02 fixed sensor sigma | `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT` | fixed-sigma deterministic automatic/Stage-2 success fixture; preserve current case as conditioning regression | no |
| T08 | pre-Gate-02 `e9d821e9` | Gate-02 fixed sensor sigma | `EARLIER_INTENDED_SEMANTIC_CHANGE_STALE_CONTRACT` | fixed-sigma deterministic final-artifact success fixture; preserve current case as fail-closed regression | no |

Gate 05 remains structurally complete but its regression acceptance remains
blocked until a separately authorized repair makes the full CTest suite green.
Prompt 6 remains **not ready**.
