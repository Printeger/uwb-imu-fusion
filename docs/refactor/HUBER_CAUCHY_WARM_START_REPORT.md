# COMPLETE_HUBER_TO_CAUCHY_WARM_START report

## Verdict

`ALL_UWB_CORRECTNESS_FAIL`

The missing orchestration contract is implemented and all engineering gates
pass. The locked Walk1 chain reached the authorized unchanged Cauchy final
solve, but `PAPER_SOLVER_CERTIFICATE_V1` rejected that final Values object for
navigation non-stationarity. No trajectory was exported and GT was not read.

## Contract implemented

`INTERMEDIATE_OPTIMIZATION_SEED` is now a distinct, explicitly non-publishable
role. A Huber terminal retains its original raw termination, stationarity
diagnostics, and Solver Certificate status. Its Values may be handed only to
the subsequent Cauchy solve after
`INTERMEDIATE_OPTIMIZATION_SEED_QUALITY_V1` passes. The role has hard false
capabilities for trajectory export and GT evaluation.

The truth-free intermediate gate requires:

- exact graph/Values key compatibility and a finite increasing state timeline;
- finite contiguous X/V/B states and finite Huber input/terminal objectives;
- terminal Huber objective no greater than its input plus a 64-epsilon
  binary64 roundoff allowance;
- finite scalar UWB residuals and positive finite sensor sigmas;
- position and velocity inside the same catastrophe envelopes already derived
  for the initializer full-seed audit from anchors, `max_range`, `v_max`, and
  the frozen 0.25 s horizon.

It does not require stationarity or a successful Solver Certificate. It adds
no optimizer parameter, threshold sweep, retry, or fallback. The initializer,
fixed-lag implementation, graph builder, raw ledger, repeat handling,
association, IMU factors, priors, and IE Stage1--4 were not changed.

The final Cauchy solve is still executed by `RunPaperBaseline` and still clears
its publishable `final_values` whenever the unchanged Solver Certificate
fails. Only `cauchy_final.valid` denotes `CERTIFIED_FINAL_ESTIMATE`.

## Tests

The focused `PaperMethods` suite now has 12 tests. The three new tests prove:

1. a finite Huber terminal with a failed certificate can be retained as an
   intermediate seed;
2. that seed remains ineligible for trajectory export and GT evaluation;
3. non-finite terminal X/V/B cannot become a Cauchy initial guess;
4. the physical input graph is not mutated by the two-stage chain;
5. the final Cauchy result still exports no Values when its certificate fails.

Existing initializer and graph tests cover the unchanged initializer behavior.
There is no source diff in `common_initializer.h/.cpp` from this task.

| Gate | Result |
|---|---|
| Focused paper-method tests | PASS, 12/12 |
| Initializer tests | PASS, 13/13 |
| T04/T06/T08 runner contracts | PASS, 3/3 |
| Core regression (`catkin test`) | PASS, 522 tests, 0 failures |
| Gate05 architecture guard | PASS |
| Full CTest | PASS, 36/36 |
| `git diff --check` | PASS |

Commands included:

```text
ctest --output-on-failure -R '_ctest_uwb_imu_fgo_gtest_test_paper_methods|_ctest_uwb_imu_fgo_gtest_test_common_initializer|test_t0(4|6|8)_runner_contract'
catkin test uwb_imu_fgo --no-deps --summarize
python3 -B test/test_refactor_gate05_architecture.py
ctest --output-on-failure
```

## Locked Walk1 chain

The isolated evidence directory is:

`experiments/icra2027/dev/HUBER_CAUCHY_WARM_START/locked_walk1_chain/`

The single command used the existing locked effective config and mode
`warm_start`. It rebuilt the same 913-state, 3,191-factor physical graph with
2,276 UWB factors. The audit reports `physical_graph_unchanged=true`, fixed
sensor sigma 0.15 m, Huber scale 1.345, Cauchy scale 2.3849, and the unchanged
100-iteration budget. Command exit was 1 because the final certificate failed.

### A. Initialization seed

- finite X/V/B: PASS, 913 states;
- objective under the entering Huber solve: 1144.5296768789888;
- position norm mean/median/P95/max: 3.1877219734 / 3.1096892144 /
  4.3483857514 / 4.4710339953 m;
- velocity norm mean/median/P95/max: 0.6299395679 / 0.5983348121 /
  1.2748614370 / 1.8041706304 m/s;
- absolute raw UWB residual mean/median/P95/max: 0.1082745679 /
  0.0774723946 / 0.2877554014 / 20.1930262792 m;
- absolute standardized residual mean/median/P95/max: 0.7218304527 /
  0.5164826306 / 1.9183693427 / 134.6201751950;
- entering Huber weight mean/P05/median/P95/min: 0.9694458227 /
  0.7011191550 / 1 / 1 / 0.0099910730;
- termination/certificate: not applicable to the already-qualified seed.

For continuity with the prior locked initialization evidence, the physical
Gaussian graph objective was 10119.985478341385 and
`INITIALIZATION_SEED_QUALITY_V1` passed before this task.

### B. Huber intermediate seed

- raw termination: `CONDITIONAL_LM_CONVERGED`, 14 accepted iterations;
- Huber objective: 844.0200102086894;
- finite X/V/B: PASS;
- position norm mean/median/P95/max: 3.2068897048 / 3.1195259150 /
  4.3692529755 / 4.4944663105 m;
- velocity norm mean/median/P95/max: 0.4434049398 / 0.4524499036 /
  0.9076839256 / 1.1322889251 m/s;
- absolute raw UWB residual mean/median/P95/max: 0.0991264183 /
  0.0757639564 / 0.2309612573 / 20.2244386838 m;
- absolute standardized residual mean/median/P95/max: 0.6608427886 /
  0.5050930430 / 1.5397417152 / 134.8295912253;
- Huber weight mean/P05/median/P95/min: 0.9829251894 / 0.8735238357 /
  1 / 1 / 0.0099755550;
- max scaled navigation gradient: 3.6007261482860535;
- `SOLVER_CERTIFICATE = CERTIFIED_FAILURE`;
- `INTERMEDIATE_SEED_QUALITY = PASS`.

The gate's position/velocity maxima 4.4944663105 m and 1.1322889251 m/s are
inside the frozen 65.8251609285 m and 242 m/s catastrophe envelopes. The
objective fell by about 300.51 and is well below its input plus the
1.6264744955e-11 roundoff allowance. Thus the failed scientific certificate
was not relabeled; the independently valid optimization-seed role alone
authorized Cauchy.

### C. Unchanged Cauchy final solve

- raw termination: `CONDITIONAL_LM_CONVERGED`, 43 accepted iterations;
- final Cauchy objective: 621.775894312065;
- finite X/V/B: PASS;
- position norm mean/median/P95/max: 3.2197416134 / 3.1322128683 /
  4.3899368309 / 4.5017126251 m;
- velocity norm mean/median/P95/max: 0.4426001296 / 0.4565410411 /
  0.8960835720 / 1.1533978274 m/s;
- absolute raw UWB residual mean/median/P95/max: 0.0983894712 /
  0.0724856639 / 0.2318275048 / 20.2491289320 m;
- absolute standardized residual mean/median/P95/max: 0.6559298080 /
  0.4832377592 / 1.5455166987 / 134.9941928800;
- Cauchy weight mean/P05/median/P95/min: 0.9204614496 / 0.7042458252 /
  0.9605627020 / 0.9996727070 / 0.0003120144;
- stationarity audit: valid but `NOT_STATIONARY`;
- max scaled navigation gradient: 4.531465978234358;
- Solver Certificate:
  `CERTIFIED_FAILURE / NAVIGATION_STATIONARITY_FAILED:NOT_STATIONARY`;
- valid trajectory exported: no.

This is the first final numerical failure of the completed warm-start chain.
Per the task STOP condition, there was no additional repair, tuning, budget
increase, kernel change, or fallback.

## GT boundary

`gt_truth_oracle_read=false`. Since no certified Cauchy trajectory was
exported, the evaluator was not invoked. ATE RMSE, P50, and P95 are therefore
`NOT_RUN_NO_CERTIFIED_FINAL_ESTIMATE`.

## Final verdict

`ALL_UWB_CORRECTNESS_FAIL`
