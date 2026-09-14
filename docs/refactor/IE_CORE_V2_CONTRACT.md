# IE Core V2 Contract

Status: `REFACTOR-GATE-04 / SOLVER_CERTIFICATE_ESTABLISHED`

This document freezes the architectural invariants required before repairing the
dense-UWB input path. It does not change an estimator, detector, threshold,
scientific lock, or published result. `doc/ie_sprint/METHOD_CONTRACT.md` and
`doc/ie_sprint/EXPERIMENT_CONTRACT.md` remain authoritative for scientific
definitions. A conflict requires a recorded amendment before implementation.

## Vocabulary and layer boundaries

- **Raw observation ledger**: one immutable row per received range observation,
  including stable `obs_id`, recording/source ordinals, sensor time, tag/anchor,
  raw range and radio metadata. Ledger membership and identity are properties of
  input provenance, not of an estimator timeline.
- **Protocol-valid**: the payload is structurally parseable and its required
  fields are finite and in protocol/domain bounds.
- **Estimator-usable**: a protocol-valid observation can be associated with the
  estimator model and timeline under an explicitly named policy. This is a
  downstream decision with a reason code; it is not source validity.
- **Suspected NLOS**: a diagnostic or detector label. It is neither invalidity
  nor an automatic instruction to discard the observation.
- **Statistically independent**: the noise model is permitted to treat an
  observation as an independent likelihood term. Distinct messages or `obs_id`s
  do not establish independence by themselves; duplicates/stale payloads need an
  explicit policy and audit trail.
- **Sensor sigma**: uncertainty assigned from the sensor/noise model and the
  observation itself. Timeline association/interpolation uncertainty, if used,
  is a separate named quantity and may not silently rewrite sensor sigma.
- **State plan**: estimator state timestamps and state keys.
- **Measurement plan**: immutable references from observations to a state or an
  explicit interpolation/association policy, plus selection and reason codes.
- **Solver termination**: the numerical optimizer's stop condition.
- **Scientific success**: an explicitly certified result satisfying the locked
  numerical, graph/Values, provenance and output checks for its method.

## Required invariants

### Contract A — ledger/state independence

The Raw Observation Ledger is independent of `state_step` (and equivalent state
density controls). Changing the state plan must leave ledger row count, ordering,
stable identity, provenance and raw payload byte-for-byte/canonically unchanged.

### Contract B — sensor-noise independence

A UWB observation's sensor sigma must not depend on how many other observations
the estimator selected, their order of selection, or the elapsed time since the
last selected/planned observation. Any association uncertainty must be separately
named, stored and reported.

### Contract C — distinct classifications

`protocol-valid`, `estimator-usable`, `suspected-NLOS`, and
`statistically-independent` are four separate fields/decisions. No one field may
be inferred solely from another. Every exclusion or decorrelation decision must
have a stable reason code and retain the raw ledger row.

### Contract D — NLOS retention

Suspected persistent NLOS must not be automatically removed by the frontend on
the paper path. It remains available to IE Stage 1 and later frozen selection
policies. A legacy/development prefilter may retain its behavior only behind an
explicitly named compatibility boundary.

### Contract E — density invariance

Changing state density must not silently alter raw observation identity or sensor
sigma. It may change state keys and an explicitly recorded association outcome.
All resulting measurement-use differences must be visible in the measurement
plan and its hash, not folded into the raw ledger.

### Contract F — success qualification

Solver termination is not equivalent to certified scientific success. A result
must expose at least separate statuses for input/build validity, solver
termination, numerical qualification, final graph/Values consistency, and
scientific eligibility. A generic GTSAM convergence predicate alone may never be
reported as scientific success.

### Contract G — Stage 1 evidence access

IE Stage 1 must receive, through a shared typed interface:

1. raw observation identity and complete provenance;
2. raw range and sensor sigma;
3. the exact factor index/state association used by the preliminary graph;
4. actual unwhitened residual and standardized residual, with sign convention;
5. validity/availability flags and reason codes.

The evidence must come from the same preliminary graph/`Values` pair identified
in the artifacts. A provider-specific reconstruction is not a substitute for the
shared contract.

## Compatibility and governance

- Gate 01 deliberately freezes current undesirable behavior in characterization
  tests; it does not make current code compliant with A--G.
- Legacy behavior stays available until a later gate explicitly moves it behind
  a compatibility adapter. Legacy output/artifact names are not renamed here.
- No detector, CUSUM, support grouping, refit, recoverability, inference,
  threshold, solver parameter, or scientific lock may change as a side effect of
  implementing these interfaces.
- Every later gate must state which contracts it establishes, demonstrate the
  invariants with tests, and report any scientific-output comparison separately.

## Gate 01 compliance snapshot

| Contract | Current state | Evidence in current production path |
|---|---|---|
| A | Partial / structurally violated | Stable source identity is created first, but `ObservationRecord` also stores `planned`, `keyframe_id`, and `nominal_sigma`; `PaperInputPlan` hashes ledger and state plan together. |
| B | Violated | `paper_input.cpp::AdaptiveSigma` uses time since the last *planned* same-link observation. |
| C | Partial | Source validity and RSSI suspicion are separate, but estimator use is represented by `planned`; statistical independence has no explicit state or duplicate/stale policy. |
| D | Met on paper path; violated by legacy prefilter | `BuildPaperInputPlan` retains suspected NLOS, while `OutlierFilter::PreFilter` removes RSSI-suspected observations. |
| E | Violated | `keyframe.step` changes planned observations and changes `nominal_sigma` for observations common to both plans. |
| F | Violated at baseline success boundary | default `GTSAM_CHECK_ONLY_V1` can set `CONDITIONAL_LM_CONVERGED`; `RunPaperBaseline` then sets `valid=true/status=OK` after only convergence and graph/key checks. |
| G | Partial | Factor metadata and residual readers exist and current FDE receives them, but there is no universal Stage-1 evidence object joining raw provenance, association, raw and standardized residuals. |

## Gate 02 compliance snapshot

Gate 02 establishes the input and measurement-semantics portion of this
contract. It does not certify success semantics or introduce the shared Stage 1
evidence interface planned for later gates.

| Contract | Gate 02 state | Evidence in production path |
|---|---|---|
| A | Met | `ObservationRecord` contains source-ledger data only. `BuildPaperInputPlan` constructs the ledger before the state timeline, and tests require identical ledger hashes, rows, and `obs_id`s for `state_step` 1, 2, and 4. |
| B | Met | Every paper-path `MeasurementPlanEntry::sensor_sigma` is copied directly from `Config::sigma_range`. The former selected-gap adaptive calculation remains only in the explicitly marked legacy noise mode. |
| C | Partial | `source_valid`, `estimator_usable`, and `suspected_nlos` are separate values with reason fields where an exclusion exists. Statistical-independence and duplicate/stale-observation policy remain deliberately unresolved for a later gate. |
| D | Met on paper path; explicit legacy boundary retained | Source-valid suspected-NLOS rows remain in the ledger and can be selected. The legacy prefilter and legacy graph-noise mode remain compatibility behavior rather than paper-path semantics. |
| E | Met | State density changes the state-plan hash and may change explicitly recorded measurement selection/association; it does not change ledger identity, ledger hash, or sensor sigma. With `state_step=1`, all estimator-usable UWB observations are selected. |
| F | Unchanged / not yet met | Gate 02 does not change solver termination or scientific-success qualification. |
| G | Unchanged / partial | Gate 02 supplies separated provenance, measurement selection, state association, and sensor sigma, but the common preliminary-graph Stage 1 evidence object is not part of this gate. |

The composite paper-input identity is versioned as `paper_input_v3` and includes
separate ledger, state-plan, and measurement-plan hashes. This intentional cache
identity change prevents a new fixed-sigma plan from being mistaken for a
legacy selected-gap-adaptive plan.

## Gate 03 compliance snapshot

Gate 03 establishes the integrity/statistical-independence boundary for exact
stale payloads and protects the existing robust baseline from a poisoned
raw-Gaussian warm start. It deliberately does not change solver-success
qualification, detector mathematics, or final IE decisions.

| Contract | Gate 03 state | Evidence in production path |
|---|---|---|
| A | Met | Every source row, including an exact repeat, remains in `ObservationRecord` with its own stable `obs_id` and provenance. Integrity has a separate, state-density-invariant hash. |
| B | Met | Integrity grouping does not rewrite `MeasurementPlanEntry::sensor_sigma`; the fixed configured sensor sigma remains unchanged. |
| C | Met for the Gate-03 integrity scope | `source_valid`, `estimator_usable`, `integrity_status`, `independent_likelihood_representative`, and `suspected_nlos` are independent fields. Exact repeats share a correlation group and instantiate at most one likelihood. |
| D | Met on paper path | Persistent or RSSI-suspected NLOS remains in the ledger and is selected unless a separate source/model/integrity rule applies. No jump or persistence rejection exists. |
| E | Met | The integrity plan is derived from source ordinals and exact payload only, independently of `state_step`; state association records which group member represents the likelihood. |
| F | Unchanged / not yet met | Generic optimizer termination is still not a scientific solver certificate. This is the sole focus of Gate 04. |
| G | Partial, strengthened | FDE Stage-1 evidence rows now expose the immutable raw range together with full source ordinals, factor mapping, unwhitened residual, factor sigma, and standardized diagnostics. A single authoritative cross-provider evidence/certificate boundary remains later work. |

The composite identity is versioned as `paper_input_v4` and includes the
independent `integrity_plan_v1` hash. The measurement plan is versioned as
`measurement_plan_v3`; legacy and Gate-02 caches cannot be silently reused.

## Gate 04 compliance snapshot

Gate 04 establishes Contract F with one shared, fail-closed paper solver
certificate. It does not change the optimizer, its termination conditions, or
any detector/recovery mathematics.

| Contract | Gate 04 state | Evidence in production path |
|---|---|---|
| A--E | Preserved | The certificate consumes the already-built final graph, `Values`, declared factor audit, and state timestamps; it does not rebuild or select input observations. |
| F | Met for Base FGO and final IE | `CertifySolverResult` separately retains raw termination and requires finite supported states/objective, exact graph/`Values` keys, caller factor integrity, applicable temporal integrity, caller-specific checks, and the existing navigation-stationarity audit. Only `CERTIFIED_SUCCESS` enables trajectory validity/export. |
| G | Unchanged / partial | Gate 04 does not alter Stage-1 evidence construction. The authoritative cross-provider evidence boundary remains later work. |

The certificate policy is `PAPER_SOLVER_CERTIFICATE_V1`. It accepts no GT,
ATE, truth, oracle, or trajectory-error input. Maximum position norm is emitted
as a finite diagnostic only and is not a success threshold. Base FGO records
the certificate in its run status/manifest and writes
`solver_certificate.json`; final IE does the same for recovery and, when used,
the existing one-shot suppression fallback. A certificate failure clears or
withholds the selected final `Values`, so it cannot be exported as a valid
trajectory. Final IE content identity and affected runner policy identities are
versioned to prevent historical termination-only artifacts from being treated
as newly certified evidence.

## Gate 05 compliance snapshot

Gate 05 introduces a source-neutral estimator preparation boundary and a thin
executable entry. It makes no change to Contracts A--G or their scientific
definitions.

| Boundary | Gate 05 state | Evidence |
|---|---|---|
| Executable/application | Established | `tools/run_ie_paper.cpp` only delegates to the application; CLI, compatibility provenance and artifacts remain outside the estimator library. |
| Estimator preparation | Established | `PrepareEstimatorCore` owns the existing input-plan, materialization, initialization, graph-build and paper-prior-replacement sequence, returning typed graph state and metadata. |
| Experiment dependency direction | Established for production core | `estimator_core` contains no task-card/experiment names and has no dependency on `tools` or experiment directories. Legacy forensic binaries remain downstream application consumers. |
| Historical configuration/artifacts | Preserved | T04/T06/T08 strings remain value-identical aliases at compatibility boundaries. |
| Scientific behavior | Snapshot-equivalent but regression gate incomplete | The pre/post T08 failure snapshot is identical except timings; core fixtures pass. Three runner contract tests reach solver iteration-limit failures, so Gate-05 acceptance remains blocked. |

This snapshot does not authorize Prompt 6. The failed runner contracts must be
resolved under an explicit follow-up scope before dense scientific execution.
