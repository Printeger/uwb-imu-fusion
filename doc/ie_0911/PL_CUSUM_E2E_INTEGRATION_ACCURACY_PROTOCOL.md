# PL CUSUM production integration and accuracy protocol

Status: `LOCKED_BEFORE_PRODUCTION_ACCURACY / IN_PROGRESS`.

Date: 2026-09-12. Start HEAD:
`9c592ba9a4e4624ecd9f131d0435f57e42ce08d7`.

Evidence root:
`/home/mint/ws_fusion_uwb/res/pl_cusum_e2e_integration_accuracy_20260912T041820Z`.

## 1. Scope and immutable inputs

This task integrates the already admitted offline detector as the sole new mode
`nlos.mode=pl_bidirectional_cusum`, then runs the existing Stage2, recoverability,
final inference, factor audit and repository evaluator. It does not redesign the
detector, support, Stage2, gate, LCB, final graph or evaluation mathematics.

The prior evidence root is
`/home/mint/ws_fusion_uwb/res/pl_bidirectional_cusum_support_20260912T024240Z`.
Its `sealed_artifacts.sha256` hashes to
`bcff967b1be8c2725b4e4013ba52ce50087a8336faa8c1997d05cc55c8c059a0` and all
126 listed artifacts verified before implementation. The sealed detector protocol
hash is `fd93ae202be1cc4893ef7e8c8df399a4629bc747b6409edfdfea780c75a4df80`.

Controlled inputs are the existing locked `sfuise_walk1_normal_clean` and
`sfuise_walk1_normal_injected` T07 caches. Their input-manifest hashes are
`7af404c8580ecf0b9f9d2742bbc56d03ed9a592172603d3df0e8a0d569e22c0e` and
`fc3c37aa75a6141a90126cdc9ed14a7b4adace586aa091d0f601c419deee61d6`;
cache IDs are respectively `sha256:4330902d2f741459038f9072e3cc6f7ade2cb6fd1a19c1929679e7c8eb44bcfe`
and `sha256:d967b14b27c45ca90cfcc3c357458efc082e6e6e3e3bd35168966942079b4f82`.
The original clean/injected input-plan hashes are respectively
`sha256:c1ae9da39e5bd651269a2f5e0d88502a842b05a816c53492c7c950b18118eb18`
and `sha256:396c106d05cdeaf7ff4dcd35dd6fc2d9b874204fa04a5ab5700e7a6e9aa0eda0`.

Missing evidence, any seal/hash mismatch, or detector identity mismatch gives
`E2E_FAIL_INPUT_PROVENANCE`; no detector evidence may be regenerated as a substitute.

## 2. Frozen detector and provider identity

The scientific signal is only PL per-anchor `conditional_z` at the admitted
pre-commit, always-commit runtime location. Forward parameters are
`kappa=0.5`, `h=7.0234689587858723`; backward parameters are `kappa=0.5`,
`h=7.0234689587858714`; gap is 1.0 s. Existing recurrence, invalid/gap reset,
inclusive crossing, forward last-zero onset and backward reverse-excursion rules are
unchanged. Final support is exact same-link/same-obs-id forward AND backward.

Provider identity is `PL_BIDIRECTIONAL_CUSUM_V1`. Parameter provenance is
`PL_BIDIRECTIONAL_CUSUM_SUPPORT_20260912_LOCKED`. Config carries all five numeric
parameters explicitly and fail-closed; oracle/truth/GT support inputs are forbidden.
Provider identity binds signal, parameter, forward, backward and support identities.
Detection freezes a source-neutral `SupportPartition` before Stage2. Stage2 and final
inference cannot rerun or modify CUSUM/support.

The admitted injected regression oracle is one segment on link `27956:20276`, 30
candidate obs IDs, no healthy candidate, first alarm affected index 15, and support
TP/FP/FN 30/0/0. Clean has zero alarm and zero final segment.

## 3. Frozen downstream policy

Stage2 and final inference use the repository's current production classes and the
existing numerical settings in the controlled input config: Stage2 outer cap 50,
the existing objective/step/KKT/navigation-stationarity rules, recoverability score,
`tau_eta=0.1`, `tau_s_m=0.1`, `tau_gamma=1.0`, and
`T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION`. No result-dependent adjustment is
allowed.

Methods are `robust_cauchy`, `suppress_all`, `structured_debias`, `lcb_partial`, and
`lcb_fixed_full`. Primary recovery is preregistered as `lcb_fixed_full`; rejection is
`suppress_all`; the other recovery policies are secondary. All four candidate-aware
policies consume one immutable production support/cache. Existing fixed compensation
is unchanged: full uses `delta=c_hat_stage2`; partial uses
`max(0,c_hat_stage2-2 sigma_c_local)`. Existing final factor audit and at most one
fallback remain authoritative.

## 4. Serial gates and stop rules

1. **P0 provenance:** prior seals, controlled input identities and frozen parameters
   verify. Failure: `E2E_FAIL_INPUT_PROVENANCE`.
2. **P1 production support:** clean forward/backward/final are zero; injected support
   is exact to the admitted shadow by IDs, links, segments, endpoints and first alarm.
   Failure: `E2E_FAIL_PRODUCTION_SUPPORT_REGRESSION`.
3. **P2 clean no-op:** empty support follows valid `SUCCESS_EMPTY`/equivalent Stage2
   and final paths, candidate/compensation counts are zero, and policy trajectories
   are byte-identical or numerically within `1e-12`. Failure: `E2E_FAIL_CLEAN_NOOP`.
4. **P3 Stage2:** nonempty injected Stage2 succeeds with finite nonnegative target
   amplitude and complete iteration/KKT/boundary evidence. Failure:
   `E2E_FAIL_STAGE2_RECOVERY`.
5. **P4 factor audit:** all four final policies obey their existing factor semantics;
   no duplicate candidate, lost noncandidate or wrong segment mapping. Failure:
   `E2E_FAIL_FINAL_FACTOR_AUDIT`.
6. **P5 truth blind:** before scientific artifact seal, successful forbidden
   GT/truth/oracle/clean-pair opens are zero. Failure: `E2E_FAIL_TRUTH_LEAKAGE`.
7. **P6 primary use:** primary accepts and applies at least one target recovery. A
   valid gate decision not to recover gives `E2E_CHAIN_PASS_TARGET_NOT_RECOVERABLE`.
8. **P7 range:** evaluator pairs exactly 30 affected clean/injected observations by
   obs_id; all used delta are nonnegative and recovered range is no greater than
   injected. Require recovered RMSE < raw RMSE - 1e-12. Failure:
   `E2E_FAIL_RANGE_EFFECTIVENESS`.
9. **P8 localization:** use `tools/paper/evaluate_runs.py`, scale-fixed SE(3), and
   common matched GT timestamps (at least 3). Primary endpoint is paired aligned ATE
   RMSE. No RMSE improvement gives `E2E_CHAIN_PASS_NO_LOCALIZATION_BENEFIT`; RMSE
   improvement with worse p95 gives `E2E_CHAIN_PASS_MIXED_LOCALIZATION_RESULT`.
   RMSE improvement and p95 non-worse permits full pass.
10. **P9 determinism:** repeat controlled production E2E twice with identical input,
    config, seed and provider. Canonical scientific support, Stage2, compensation,
    factor audit, trajectory and evaluation fields must be exact. Failure:
    `E2E_FAIL_NONDETERMINISM`.

Only all applicable P0--P9 passing yields
`E2E_FULL_SYSTEM_PASS_DEVELOPMENT`. The earliest failure is the sole main verdict.
Six-input accuracy is a non-gating diagnostic run only after production integration,
support, Stage2, factor audit, range artifacts and valid final trajectories exist;
its results cannot tune anything or select another primary.

## 5. Metrics and truth boundary

After scientific artifacts are sealed, the independent range evaluator pairs clean
and injected raw observations by exact obs_id and reports raw/recovered mean, median,
MAE, RMSE, absolute p95; absolute/percent RMSE and MAE improvement; compensation
mean/median/min/max; and the known +0.5 m amplitude error as diagnostic only.

Trajectory evaluation reuses the repository evaluator's GT association, scale=1
SE(3) aligned ATE, horizontal/vertical errors and RPE. It reports common matched count
and interval, aligned ATE mean/p50/p95/p99/max/RMSE, horizontal/vertical RMSE,
translation/rotation RPE and raw-frame ATE only when provenance is valid. Primary gain
is `ATE_suppress-ATE_lcb_fixed_full`; p95 is evaluated analogously. Clean accuracy is
integration/no-op evidence. Injection penalties are explanatory only.

Detector, provider, Stage2, scoring, gate, compensation and final optimizer may not
read GT, truth support, target/injection metadata, injection amplitude, oracle labels
or clean paired ranges. File-access traces cover the injected production chain and
post-seal evaluator separately.

## 6. Reproducibility and result interpretation

The result preserves every failed, fallback, zero and unavailable run. Scientific
artifacts include explicit chain status, support/Stage2/score/compensation/decision/
factor-audit/range/trajectory files. Tests cover fail-closed config, provider
equivalence, empty support, injected mapping, Stage2/final factor semantics and
determinism, followed by package build and full CTest.

Full pass supports only the locked controlled Walk1 development statement: admitted
support, useful range recovery, and paired localization improvement versus rejecting
the same observations. It is not a general superiority, certified integrity, causal
endpoint or formal held-out claim. The next action is chosen exactly from the verdict
mapping in the user-authorized task; no tuning follows a negative or mixed result.
