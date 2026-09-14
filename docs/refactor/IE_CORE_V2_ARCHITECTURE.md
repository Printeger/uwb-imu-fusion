# IE Core V2 architecture after Gate 05

Date: 2026-09-13

## Dependency direction

```text
tools/run_ie_paper.cpp                     executable entry (5 lines)
        |
        v
tools/paper/run_ie_app.cpp                 application orchestration / compatibility
        |
        +--> loader + observation/integrity plan
        |
        +--> uifgo::PrepareEstimatorCore
        |       |
        |       +--> BuildPaperInputPlan
        |       +--> MaterializePaperKeyframes
        |       +--> Initializer
        |       +--> GraphBuilder
        |       `--> paper Pose3-prior replacement
        |
        +--> RunPaperBaseline + solver certificate
        |
        +--> Stage-1 support providers
        +--> SegmentRefitter
        +--> RecoverabilityScorer
        +--> FinalInferenceEngine + solver certificate
        |
        `--> paper_run_io / nlos_inference_io artifact writers

tools/run_*_forensics.cpp                  experiment/diagnostic applications
        `--------------------------------> application/core (one-way only)
```

The production library has no dependency on `tools/`, task-card scripts, or
experiment directories. Application and experiment targets depend on the
library; the reverse edge is forbidden.

## Responsibilities

| Boundary | Responsibility | Must not own |
|---|---|---|
| Data/input plan | Raw observation ledger, integrity status, state timeline, measurement association | Solver decisions or experiment labels |
| `estimator_core` | Deterministic construction of the plan, keyframes, initialization, physical graph, initial `Values`, UWB factor indices and metadata | CLI, YAML provenance locks, cache protocol, artifact files, detector/refit policy |
| Baseline solver | Baseline graph solve and shared solver certificate | Support discovery or recovery policy |
| IE modules | Support production, bias refit, recoverability scoring, recover-or-suppress final inference | Dataset loading or experiment orchestration |
| Application | Validate runtime request, call the above modules in contract order, retain compatibility parsing, write artifacts | New estimator mathematics |
| Experiments/forensics | Build locked requests and inspect outputs | Calls from production core |

## Configuration boundary

Algorithm modules use typed runtime values (`Config`, solver/refit/inference
option structs), not task-card names. The semantic production names introduced
at Gate 05 are `kDevelopmentGateProvenance`,
`kOracleSupportDebugProvenance`, and
`kAutomaticDiscoveryDevelopmentProvenance`.

The old `T04`/`T06`/`T08` identifiers remain aliases with identical string
payloads so historical YAML and artifacts continue to load. They are
compatibility/provenance vocabulary at the application edge, not dependencies
of `estimator_core`. Existing A/R-labelled forensic programs remain separate
executables. Removing their legacy include-based access to application helpers
would require a later mechanical extraction of loader/artifact services; the
production library does not depend on them today.

## Gate-05 extraction and remaining debt

`tools/run_ie_paper.cpp` changed from 5,432 lines at the pre-refactor snapshot
to a five-line executable entry. The former implementation is now an explicit
application boundary, and the reusable source-neutral preparation sequence is
in `include/uifgo/estimator_core.h` / `src/estimator_core.cpp`.

The application compatibility unit remains large because it retains historical
CLI validation, cache identities and artifact schemas. Further splitting it is
safe only as a subsequent behavior-preserving gate with the currently failing
runner regressions first restored; it is not estimator-core code and no core
module includes it.

