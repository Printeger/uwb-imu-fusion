# T10-A10 validation admission proposal

Status: **PROPOSED_FOR_REVIEW / EXECUTION_DENIED**. This is a concrete future synthetic
validation plan, not a locked gate, executed validation, test evidence or claim upgrade.
A10 development has completed 9/9 processes, all failing at first conditional navigation
before a chain solve. No support candidate can be selected from those runs. The first
admission blocker is an accepted development path to automatic Stage 1/2 and auditable
scores; the pilot budget is closed and this document does not authorize more diagnosis.
A08 graph FD remains 15/18, with three smallest-step discrepancies unresolved. An admission
review must explicitly accept the numerical support domain and that limitation or resolve it
with separately authorized evidence. A05 V2/default behavior is not silently migrated.

## Split and provenance

Machine-readable proposal: [T10_A10_SPLIT_PROPOSAL.json](T10_A10_SPLIT_PROPOSAL.json).
All variants/windows/prefixes sharing a base trajectory, recording ancestry or seed stay in
one role. The split manifest hashes complete motion/sensor/scenario specifications; actual
recording/cache content IDs are null until generated. Proposed recording keys are logical
reservation keys, not fabricated content hashes. No held-out input/GT has been generated,
opened or run this round. Seed 10102 used in generator checks is permanently development.

One development base and its metamorphic seed are observed. Two different complete base
trajectories with seeds 20101/20102 are reserved for validation; three others with seeds
30101/30102/30103 are reserved for future test. Each base's six scenario variants are paired,
not six independent motion repetitions. They share a modest analytic turning family with
different rate, geometry amplitudes, initial height and turn angle. Their numeric motion
parameters and seeds do not overlap development; this supports within-family generalization,
not arbitrary motion/hardware generalization. Two validation bases cannot establish statistical
significance. Display each base; any interval must resample whole base clusters, not segments.

An independent synthetic calibration reservation (base cal01, seed 40101, LOS only) is
listed but NOT_GENERATED/NOT_RUN and excluded from the compute schedule. Current synthetic
beta/noise/geometry are exact model assumptions; no calibration estimate or precision is
claimed. This optional recording is not necessary for exact-assumption synthetic validation.
Real independent beta, surveyed geometry/lever/clock, GT point and license requirements
remain unresolved and are not satisfied by this proposal.

Six reserved main scenarios: LOS; links 1/1–2/1–3 sustained steps .6/.9/.7 m;
two-link ramps starting .2/.3 m with slopes .05/.075 (gentle) and .2/.25 (steep) m/s.
All burst intervals [3,6] s, recording 0..8 s. The generator needs a reviewed parameter-input
extension to express these profiles and scenarios; its currently implemented CLI intentionally
permits only A10 development generation. Three planned one-factor checks (weak geometry,
range sigma .10 m, UWB 2.5 Hz) and an alternate duration are **DEFERRED, NOT_SCHEDULED**,
not silently dropped evidence or implied completed RQ3. Current scope cannot support full
noise/count/generalization claims from the six main cells alone. Test execution needs its
own budget and review after locking; none of the test labels may participate here.

## Stage order and finite budgets

Proposed validation-only `B_total=14400 s` (4 h serial wall allocation):
`B_scheduled_max=10800 s`, reserve `3600 s` (25%). Every estimator process hard cap 120 s;
concurrency=1; no automatic retries, seed replacement, parameter expansion, solver/tolerance
change or partial-group splitting. New profiling may reduce scheduled work only through a
pre-execution amendment; it cannot remove bad seeds/failures. Reserve use requires a separately
logged reason and review; no success-chasing extension.

| Work | Processes | Worst-case seconds | Dependency |
|---|---:|---:|---|
| 3 support candidates × 12 validation inputs, AUTO Stage1/2/score | 36 | 4320 | Admission accepted and split-bound generator ready |
| fixed-partition DEBUG, 12 inputs | 12 | 1440 | Scripted support frozen before outputs, separate namespace |
| selected operating point final replay: fit_only/s_fit/full_gate × LOS/step2/steep-ramp × 2 bases | 18 | 2160 | Same selected AUTO parent cache, three policies equally budgeted |
| all_range LOS reference, 2 bases | 2 | 240 | Same input/init/noise/solver |
| generator, checks, publishing, 1152 cached policy evaluations, metric audit | 0 | 2640 allowance | Explicitly measured separately |
| **Scheduled cap** | **68** | **10800** | No failure refund into extra trials |
| **Unallocated reserve** | — | **3600** | 25% of total |

The 1152 decision evaluations are 12 inputs × 2 namespaces × 4 policies × 12 points.
No threshold causes Stage1/2 to rerun. Fixed DEBUG does not execute discovery, does not
supply amplitudes or GT initialization and never repairs/masks an AUTO failure. Its partition
is the scripted link interval intersected with the frozen input plan. LOS has empty support.
The `nominal_curvature` registry currently supports cache diagnostics only; its 12 decision
points have equal budget, while final replay is deliberately only the three supported policies.
This is not a downstream comparison of all four policies. RQ2/T11/T12 full matrices are outside
this proposed allocation, not represented as complete.

A10 measured process walls are about 1 s, but only failed first-block solves were measured;
these cannot estimate successful Stage2/scoring/final cost. Accordingly 120 s is a conservative
resource ceiling, not a promised runtime extrapolation. Failure, timeout, zero candidate,
zero eligible and missing parent remain rows and consume their scheduled slots.

## Support candidates and selection

Exactly P1/P2/P3 from [T10_A10_PROTOCOL.md](T10_A10_PROTOCOL.md):
(lambda1,lambdaTV,b_min,change,merge) = (4,20,.10,.15,.10),
(8,40,.15,.20,.15), (12,80,.20,.25,.20), in stated metre/objective units.
No full Cartesian product. All other solver/scientific definitions remain the admitted A10
settings, including short count2/duration .01, boundary1e-9, closed-interval grouping and
single-pass merge. Validation support choice is shared by all four policies, never selected
separately to help full_gate or eta.

Evaluate each candidate on all 12 inputs. Rank lexicographically by:
1. number of Stage1/2 failures (ascending; timeouts included);
2. mean per-base full-observation bias-field RMSE (ascending): Stage2 c for candidates,
   zero for noncandidates, and the full synthetic b_total per obs_id as truth;
3. total producer wall time (ascending), then P1/P2/P3 ID order.
A failed base/scenario has no invented RMSE; candidate ranking uses item 2 only when compared
candidates have the same successful cell domain, otherwise ID breaks the tie. Selectability
requires zero Stage1/2 failures and at least one numerically valid eligible AUTO group in
**each validation base**. If no candidate qualifies, terminate **NO_ADMISSIBLE_SUPPORT**,
produce all rows, do not lock support/gate, and do not consume final replay cells. Eligibility
is a readiness criterion, not a reason to edit truth, discard short segments or bypass discovery.

## Equal gate candidate budget

All policies share selected-support AUTO caches; fixed DEBUG caches form a separate analysis.
Required score availability and frozen group eligibility remain prerequisites. Curvature is
lambda_min(N), m^-2, with >= threshold; it is not navigation Hessian trace or a full prior method.
Gamma uses original nominal sigma and Stage2 post-fit residuals, not GT/noise-inflated weights.

| Policy | Exactly 12 candidate points |
|---|---|
| fit_only | tau_gamma={.25,.5,.75,1,1.25,1.5,2,2.5,3,4,6,9} |
| s_fit | tau_s={.05,.10,.20} m × tau_gamma={.5,1,2,4} |
| full_gate | same 12 (s,gamma) points; eta=.02 for s=.05, eta=.10 for s=.10, eta=.30 for s=.20 |
| nominal_curvature | tau_N={25,50,100,200,400,800,1200,1600,2400,3200,4800,6400} m^-2 |

The full grid is explicitly coupled, not a hidden 36-point search. It explores a restricted
policy family; a negative result is only about these points. Curvature thresholds correspond
to conditional scales 1/sqrt(N)=.2.. .0125 m; s thresholds span one to four nominal range
sigmas; eta thresholds are illustrative fractions, not calibrated safety bounds. All equalities
pass according to existing policy. Nominal curvature does not also test gamma.

Use three declared candidate-coverage targets {0.25,0.50,0.75}. For each policy/target choose
among its same 12 points using validation only: minimum absolute deviation of mean per-base
candidate coverage from target (require deviation <=.10), then mean per-base accepted squared
bias error ascending, then harmful-acceptance frequency ascending, then point ID. Only points
with nonempty acceptance in both bases and successful parent metrics qualify. If no point
matches within tolerance, report UNATTAINABLE_TARGET; no interpolation/new threshold.
The deployment operating point is the .50 target selection; other targets are locked curve
points. Test reports actual achieved coverage, never retunes thresholds to hit targets.
Selection uses AUTO evidence only; DEBUG does not select a different gate.
A .50 operating point is admissible only if bad-correction rate <=.10 in each base; otherwise
NO_ADMISSIBLE_GATE, no forced all-suppress winner. Full vs s_fit is evaluated at matched achieved
coverage, not presumed eta superiority. Test labels never enter selection.

## Metric protocol and downstream checks

Proposed `epsilon_bad_m=.20 m` (application-level excess-range tolerance, distinct from s).
Per observation error e_i=c_hat_i-b_total_i; bad iff |e_i|>.20 (equality good).
Report accepted bias-field RMSE, bad-correction numerator/accepted-observation denominator,
good-correction rejection (suppressed candidate with |e_i|<=.20 over all good candidates),
plus count/length distributions of all segments/groups and statuses. Never substitute
beta+b_total or post-fit residual for b_total. The generation manifest proves base errors are
known; actual truth is evaluation-only, paired on stable obs_id.

Report candidate_use_coverage including ineligible observations; eligible_use_coverage;
overall_retained_fraction from actual final factor audit over full fixed-valid ledger.
No candidates or eligible yields explicit undefined denominator. Zero acceptance yields
undefined conditional risk, not zero risk. Stage1/2 failure yields no defined candidate
coverage and stays in run-level failure denominator. Missing parent/corruption is a separate
integrity failure. Stage2 decision-time risk and final-time bias/ATE are separate tables.

Synthetic GT represents IMU origin; map-to-GT and body-to-GT transforms exactly identity by
construction. Raw-frame ATE is primary; per-run SE3-aligned ATE is additional. Pair exact
keyframe times to stored GT at tolerance 1e-9 s, no extrapolation; RPE horizon 1 s, exact pairs.
Report RMSE/P95/P99/max, matched count, per-axis errors, and translation/rotation RPE.
Trajectory failure: solver invalid, missing/nonfinite output or <100% planned keyframe matches;
ATE>1 m separately flags unacceptable trajectory accuracy without relabeling numerical status.
LOS degradation unacceptable if final raw-frame ATE RMSE exceeds matched all_range by >.05 m
or P95 by >.10 m, or a new estimation failure occurs. These are proposed application tolerances,
not retrospectively inferred from test. LOS failure blocks deployment lock. Final recovery
failure and successful/failed single fallback are separate rates; keep stage-specific costs/RSS.
Bad correction is not called trajectory harm without paired downstream Use/Suppress evidence.

## Minimal scheduler/role modification (not implemented in A10)

Actual current restrictions: `run_experiments.py::load_manifest` only permits development;
policy provenance is limited to TEST_ONLY/PENDING_VALIDATION. C++ common preparation currently
hashes `recording_id + ':development'`. Merely allowing a new Python role would be incorrect.

1. Add closed schema `uifgo_t10_validation_admission_v1`, explicit `role=validation`, approved
   admission-document SHA, split SHA, budget ID, metric implementation SHA and immutable source,
   configuration, runner/core/GTSAM producer IDs. Existing development schema/behavior stays.
   A standalone validator must reject unknown roles, test, role ancestry overlap, fake seed-only
   independence, missing consent/provenance and unscheduled cells before any process/label read.
2. Pass a **truth-free** C++ input-context manifest carrying role/split/base/recording/seed,
   nominal-noise and synthetic-assumption hashes; C++ independently verifies input/cache binding.
   Replace hard-coded development identity only in the new opt-in path. Include context in
   common preparation, producer and final request identities. Full evaluation manifest/truth
   paths never enter runner config or CLI; no GT orientation or init values.
3. Version Stage2 envelope schema (v3 proposed), preserving canonical v2 reader for development.
   Bind role/split/context/producer provenance in canonical ID, reject cross-role cache reuse and
   namespace swaps in both Python and C++. Gate-only points share the same verified cache;
   never change the parent ID to a threshold request ID. Content-addressed raw bytes can be
   physically reused only when the ancestry manifest permits; dev ancestry can never become val.
4. Add durable budget ledger: reserve each planned process before spawn; count failure/timeout,
   capped 120 s and no retries; reject unscheduled support/threshold points and reserve use.
   Diagnostic point accounting is separate and capped at 12 per strategy/input/namespace.
5. Export explicit curvature value/threshold/unit/pass (currently diagnostic output omits them),
   require complete gamma list for every group's segments, and produce explicit missing-parent/
   unavailable status rows. Evaluator requires matching split, selected support, total truth
   completeness and artifact graph/hash validation before computing any metric.
6. Before admitting validation, test positive development compatibility plus negative role
   mismatch, same base across splits, missing metric/provenance, over-budget/retry, test input,
   truth-contaminated CLI/config, mismatched parent context and changed library IDs. Existing
   cache/evaluator/final regressions and actual eligible-score provenance checks must pass.

Only after review and these implementations/checks may the validation allocation execute.
Gate lock requires a separate review of selection, coverage, LOS and failure evidence.
Test stays refused until a further locked test manifest and budget are accepted. A10 does not
claim any of these proposed interfaces exist, nor that its development pilot passed admission.
