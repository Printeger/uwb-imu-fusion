# REFACTOR-GATE-01 Report

Result: `PASS / CONTRACT_AND_CHARACTERIZATION_ONLY / GATE_02_READY`

Gate 01 mapped and froze the current production behavior. No production source,
configuration, algorithm, threshold, solver option, scientific lock, result, or
artifact name was changed. The dense-state conclusion remains a description of
the current confounded pipeline; it is not evidence that dense UWB measurements
are intrinsically harmful.

## Frozen context

- Branch: `feature/uwb-imu-fusion-ie-postprocessing`
- Inspected HEAD: `e9d821e960f725e064ca32fd9e475e529ac5f501`
- Frozen paper structure SHA256:
  `8ac373b301380468d577266a703301e45e3080b4ba1e95cf1f3764a736467144`
- Frozen roadmap SHA256:
  `b9bb63c7f2155124602716eb50ce51ae14c6a7c3ab242978f4805fe85633bdb`
- The worktree was already dirty. Existing user changes and experiment outputs
  were preserved; this gate did not clean, overwrite, stage, commit, or push them.

## Current architecture call graph

```text
tools/run_ie_paper.cpp::main
  -> LoadData
     -> DataLoader::{LoadUwb,LoadImu,...}
  -> BuildPaperInputPlan
     -> StableObservationId / source ledger construction
     -> protocol/domain validity + RSSI suspected-NLOS label
     -> keyframe.step + keyframe.min_interval selection
     -> planned/keyframe_id + AdaptiveSigma snapshot
  -> strategy mask -> MaterializePaperKeyframes
  -> Initializer::Run
  -> GraphBuilder::Build
     -> X/V/B states + priors + CombinedImuFactor
     -> AddUwbFactorsForFrame + FactorMeta(obs_id,factor_index,keys)
  -> PrepareRawGaussianReference
     -> RunCheckedConditionalLm
  +-> baseline: RunPaperBaseline -> final graph/Values/export
  `-> IE Stage 1 provider
        -> AutomaticSupportProvider (legacy/development), or
           ImuAidedFdeSupportProvider (current production), or
           PlBidirectionalCusumSupportProvider
      -> SegmentRefitter / frozen candidate policy
      -> ScoreRefitRecoverability
      -> FinalInferenceEngine::Run
         -> use/suppress decisions, one fallback, final graph/Values
      -> inference/residual/covariance/export artifacts
```

## Files and functions inspected

| Area | Files / principal functions |
|---|---|
| Loading and source types | `include/uifgo/types.h`, `include/uifgo/data_loader.h`, `src/data_loader.cpp` |
| Legacy filtering | `include/uifgo/outlier_filter.h`, `src/outlier_filter.cpp::PreFilter` |
| Ledger and planning | `include/uifgo/paper_input.h`, `src/paper_input.cpp::{StableObservationId,BuildPaperInputPlan,MaterializePaperKeyframes,AdaptiveSigma}` |
| Graph construction | `include/uifgo/graph_builder.h`, `src/graph_builder.cpp::{Build,AddUwbFactorsForFrame,AdaptiveSigma}` |
| Baselines | `include/uifgo/paper_methods.h`, `src/paper_methods.cpp::RunPaperBaseline` |
| Solver checks | `include/uifgo/nlos_solver_utils.h`, `src/nlos_solver_utils.cpp::{RunCheckedConditionalLmImpl,AuditNavigationStationarity}` |
| Stage 1 | `nlos_discovery`, `nlos_fde`, `pl_bidirectional_provider`, `pl_bidirectional_support`, `pl_persistent_cusum` headers/sources |
| Stages 2--4 | `nlos_refit`, `nlos_scoring`, `nlos_recoverability`, `nlos_inference` headers/sources |
| Orchestration | `tools/run_ie_paper.cpp::{LoadData,main}` |
| Tests | `test/test_paper_input.cpp`, `test/test_graph_builder.cpp`, `test/test_nlos_discovery.cpp` |
| Frozen governance | `doc/ie_sprint/{STATUS,METHOD_CONTRACT,EXPERIMENT_CONTRACT}.md`, `doc/v2/{paper_structure.tex,v2_roadmap.md}`, `doc/v3/v3.md` |

## Exact coupling points

| Concept | Current owner/coupling | Consequence |
|---|---|---|
| Raw identity | `BuildPaperInputPlan` creates stable `obs_id`, but stores source and estimator fields in one `ObservationRecord`. | Raw ledger cannot be consumed as a purely source-layer object. |
| State creation | `BuildPaperInputPlan` applies `cfg.kf_step`/`kf_min_interval`; `MaterializePaperKeyframes` carries that plan into `GraphBuilder::Build`. | State density and measurement planning share one selector. |
| UWB selection | Only valid observations in selected keyframe frames become `planned`; materialization masks only this subset. | Changing `kf_step` changes both state count and UWB factor population. |
| UWB noise | `paper_input.cpp::AdaptiveSigma` uses `last_link_time`, updated only for planned observations. `GraphBuilder` consumes the snapshot; legacy NaN takes a second adaptive path. | Selection cadence changes the sigma of a common observation; there are two sigma owners. |
| Observation validity | `BuildPaperInputPlan` combines source protocol flag, finiteness, known-anchor and range bounds into `valid`. | Protocol validity and model usability are not independently represented. |
| Suspected NLOS | Same function sets `suspected_nlos` from RSSI while retaining it; legacy `OutlierFilter::PreFilter` discards it. | Paper and legacy frontends have different, insufficiently explicit boundaries. |
| Independence | No duplicate/stale/correlation field or policy exists. Each distinct provenance row materializes as a separate factor. | Repeated payloads are counted as independent likelihood terms. |
| Solver termination | default `RunCheckedConditionalLmImpl` trusts linked `gtsam::checkConvergence` under `GTSAM_CHECK_ONLY_V1`; stationarity is optional. | Generic termination can occur without the independent stationarity audit passing. |
| Scientific success | `RunPaperBaseline` maps `final.converged` plus graph/Values key equality to `valid=true`, `status=OK`. | Numerical termination and scientific eligibility are conflated at the baseline boundary. |
| Stage-1 evidence | Factor metadata links `obs_id` to factors and residual readers expose raw residual/noise; providers assemble their own views. | Contract G is only partial; provenance and standardized residuals lack one shared typed interface. |

The normative future contracts and current compliance classification are in
[`IE_CORE_V2_CONTRACT.md`](IE_CORE_V2_CONTRACT.md).

## Characterization tests added

All added tests describe current behavior and intentionally do not repair it:

- `PaperInputCharacterization.KeyframeStepCurrentlyChangesSelectionAndSameObservationSigma`
  freezes the current `kf_step` coupling while confirming raw identities/payloads
  are stable.
- `PaperInputCharacterization.DuplicatePayloadsCurrentlyMaterializeAsIndependentFactors`
  freezes the absence of a duplicate/stale independence policy.
- `CheckedConditionalLmCharacterization.DefaultGenericConvergenceDoesNotCertifyStationarity`
  demonstrates that default generic termination is not a stationarity
  certificate.

Verification performed:

```text
catkin build uwb_imu_fgo --no-deps --summarize
  exit 0 (production build; existing compiler/symlink warnings only)

catkin test uwb_imu_fgo --no-deps --summarize
  exit 0; 472 tests, 0 errors, 0 failures, 0 skipped

test_paper_input --gtest_filter=PaperInputCharacterization.*
  exit 0; 2/2 passed

test_nlos_discovery --gtest_filter=CheckedConditionalLmCharacterization.*
  exit 0; 1/1 passed
```

No experiment matrix or scientific run was executed because Gate 01 changes no
production behavior. Therefore no scientific numeric result changed in this
gate.

## Bounded edit surface proposed for REFACTOR-GATE-02

Gate 02 may implement only ledger/planning separation and direct adapters needed
to keep existing behavior selectable:

- `include/uifgo/paper_input.h`
  - split immutable source observation fields from state/measurement plan fields;
  - introduce explicit state-plan, association/use decision and reason-code
    types;
  - provide separate canonical hashes for ledger and plans.
- `src/paper_input.cpp`
  - build the raw ledger without reading `kf_step`;
  - build state and measurement plans in separate functions;
  - assign fixed sensor sigma independently of selection;
  - retain current adaptive behavior only through an explicitly named legacy
    compatibility adapter.
- `include/uifgo/types.h`
  - carry source identity/sensor sigma into materialized ranges without
    conflating them with state creation.
- `include/uifgo/graph_builder.h`, `src/graph_builder.cpp`
  - consume an explicit measurement plan and remove the hidden paper-path sigma
    fallback; keep legacy fallback behind its compatibility entry point.
- `tools/run_ie_paper.cpp`
  - wire ledger -> state plan -> measurement plan -> materialization explicitly;
  - preserve downstream method inputs and artifact names.
- `test/test_paper_input.cpp`, `test/test_graph_builder.cpp`
  - replace the undesirable density/sigma characterization with new A/B/C/E
    invariant tests while retaining a legacy-adapter regression.

Gate 02 must not edit Stage-1 detector/CUSUM logic, Stage-2 refit mathematics,
Stage-3 scoring/recoverability, Stage-4 recovery/suppression, solver parameters,
scientific locks or experiment conclusions. If compatibility cannot be preserved
within the files above, stop and record an amendment rather than expanding scope.

## Blockers and commit-ready summary

No Gate-01 blocker remains. The production build and complete package test suite
pass, contracts A--G are explicit, current violations are localized, and the
Gate-02 edit surface is bounded. The work is commit-ready as a documentation plus
characterization-test change; it was intentionally not committed or pushed in
the pre-existing dirty worktree.
