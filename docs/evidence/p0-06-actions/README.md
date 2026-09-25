# P0-06 action-search completeness evidence

Base golden tag: `golden-p0-05-publication`  
Peeled base SHA: `02ae9f6b92bf2afc852cda9eac18c50aec6686b1`  
Run date: 2026-09-25 (Asia/Shanghai)

## Contract implemented

- Removed both silent truncation sites: the hard-coded 128 break during action
  entity materialization and the `resize(max_candidate_count)` replacement in
  plausible-set generation.
- Added `ActionSearchResultV1`/`ActionSearchCensusV1` sidecars and a separate
  `GeneratedActionSnapshotV1` trusted input. Every raw
  KEEP_ALL/request occurrence enters the single census boundary and retains a
  stable occurrence identity, complete canonical operation bytes, terminal
  record, reason and duplicate-of proof where applicable.
- Graph-operation equality and downstream semantic equality are separate.
  Safe duplicate equality includes action ID/model, coverage and cardinality in
  addition to canonical graph-mutation/factor-block bytes. Canonical census
  order and representatives are independent of raw enumeration order.
  Floating values are serialized by bit pattern;
  `+0/-0` remain distinct, NaN/Inf are explicitly invalid, and compact-key or
  digest collisions cannot authorize merging.
- `validateActionSearchCensusV1` derives completeness from protocol, checked
  uint64 arithmetic, unique records, the separately supplied trusted raw
  snapshot, actual evaluated-action byte/order binding and duplicate proofs
  recomputed per occurrence. FDE never trusts the producer's `exhaustive`
  boolean or a raw vector embedded beside the certificate.
- A resource-cap omission is `RESOURCE_CAP_UNPROVEN` and makes the census
  non-exhaustive. The FDE V2 context returns `SEARCH_INCOMPLETE` before any
  winner, complete-risk, commit or integrity claim. The legacy vector-only API
  throws on an incomplete search because it cannot carry the census.
- The complete action-search certificate and every occurrence are staged in
  the production attempt before candidate kernel/post/PL. The certificate
  distinguishes planned evaluations from actual candidate/post/PL counts and
  binds cap, lifecycle, trusted snapshot, generator and validation. Explicit
  explicitly installed test-only throwing dependencies wrapping the real production evaluator,
  detector and PL implementation at those three fallible stages preserve every
  raw occurrence, exact-semantic duplicate reason and duplicate-of identity
  through the outer catch, final packet and CSV. The release DSO contains no
  environment-variable fault hook or ambient/globally activatable throw state.
  The dependency pointer is held in a mutex-protected external sidecar keyed by
  pipeline address, with constructor/destructor erase and no default entry, so
  the public `RealtimeIntegrityPipeline` object retains the P0-05 ABI and an
  address cannot inherit a stale seam; an
  NDEBUG regression proves the legacy variable cannot change the raw census,
  decision or packet. Production
  refuses with actual `evaluated=0`; safe duplicate records remain distinct
  from resource/invalid/aborted omissions without double counting. Production
  carries the refusal through candidate audit, risk result,
  discard/commit gate, `fde_status`, reason codes, final packet, ROS adapter
  fields and CSV/logger fields. The final-packet boundary independently forces
  unavailable/unprotected/formal-false/risk-invalid for `SEARCH_INCOMPLETE`.
- Existing public V1 structs and P0-05 publication layouts were not extended;
  the action census and FDE context are additive V1/V2 sidecars. The pipeline
  layout is restored exactly: golden and current portable headers both report
  size/alignment 5056/16; with the DSO's `-march=native` Eigen ABI they both
  report 5184/32. A source-retained golden-header canary constructs the pipeline
  in guarded storage, calls reference and by-value APIs across the current DSO,
  destroys it and verifies both guards.
- The rejected `FdeDecisionContextV2::complete_risk_inputs` entry and its
  thread-local ledger substitution were completely removed. Production and
  tests both use the unchanged P0-04 `buildRiskLedger`/selection inputs; source
  search finds no override symbol or thread-local risk ledger.
- Production candidate evaluation now uses `CandidateEvaluationRouterV1`.
  Before invoking the optimized rank evaluator it independently checks the
  frozen proof identity, exact remove/add group mapping, changed-block solve
  shape/finiteness, final raw-Jacobian SVD rank/dof/condition/state-step margin
  and update shape. Unknown or boundary cases terminate through a router-local
  raw-H SVD state/covariance result with `DENSE_ROUTER_EXACT`; the existing
  public `DenseCandidateOracle` semantics are unchanged. A permitted rank
  result is still compared against the raw-H SVD state, rank/dof, statistic
  and full covariance before use; mismatch switches to dense. Production stage
  diagnostics record rank/dense counts and every fallback reason.

No detector threshold, alert limit, risk budget, prior, fault family,
hypothesis/action coverage contract, statistical denominator or formal Gate J
setting was changed. Increasing the cap is not used as the repair.

## Independent red reproduction and O07

The first checked build of the new independent test failed at link time because
`censusAndCapActionsV1` and `exactActionOperationIdentityV1` did not yet exist.
That frozen failure is summarized in `first-failing-results.txt`.

The final directed target passes 24/24. It includes:

- 130 distinct admissible actions under cap 128;
- exact action census/cap reporting every omitted occurrence identity;
- one immutable semantic universe built by `HypothesisGenerator::generate()`
  from 128 real historical UWB epochs for one anchor. The unchanged production
  evidence evaluator declares all 128 generated hypotheses plausible; no test
  writes `plausible` or `profile_j`. `actionsForPlausibleSetV1()` therefore
  produces exactly KEEP_ALL + 128 single-epoch requests + its standard
  `FULL_PLAUSIBLE_UNION_RECOVERY` = 130 actions. The union is action ID 130 at
  zero-based index 129 and is the only action covering the complete 128-member
  plausible universe. Cap 128 omits both action 129 and that sole full-union
  candidate; no hypothesis or obligation is deleted after generation;
- a test-local independent dense oracle starts only from each retained block's
  `jacobian_raw`, `residual_raw` and covariance. It independently computes a
  stable Cholesky whitener (positive-eigen fallback), asserts its contract
  against production on non-diagonal correlated covariance and nonzero RHS,
  then performs its own SVD,
  computes state/full covariance/parity rank/dof/threshold, projects the full
  semantic hypothesis set, and independently computes the scalar-fault and
  nominal PL with Boost distributions. It does not call
  `CandidateEvaluationRouterV1`, `JointWindowDetector::evaluateCandidate`, or
  `ProtectionLevelV2` for expected values. Production and oracle agree that
  only action 130 is complete and numerically viable; production assigns it
  the sole pre-risk `UseInReferenceEstimate` disposition. With the unchanged P0-04 contract,
  production FDE honestly returns `RiskBudgetInvalid`, selects no action and
  does not commit. Three reorderings preserve the refusal, state/full
  covariance, PL, full plausible coverage, risk result and every disposition.
  The cap-128 run separately refuses `SEARCH_INCOMPLETE` before candidate
  evaluation;
- exact-semantic duplicate proof-safe merging; operation-equivalent actions
  with different IDs/models remain distinct, same-ID semantic conflict fails
  closed, and forward/reverse order produces the same FDE status, winner,
  commit, risk and final-packet digest;
- reordered enumerations with invariant uncapped eligible action set, FDE
  status/commit decision and complete generated census;
- a 54-cell amplitude/onset/axis/joint grid where every cell constructs a real
  window/transaction and UWB/IMU subspaces, invokes
  `HypothesisGenerator::generate()` and the real
  `HypothesisEvidenceEvaluator`, then uses the complete computed plausible set
  for action generation/census, nonempty-hypothesis PL and FDE. Every plausible
  hypothesis and every affected group is checked, including 2,295 joint
  hypotheses. The 54 cells contain 2,863 plausible hypotheses and honestly
  end in 54 refusals (18 detector alarms): 22 cells lack a valid frozen
  numerical proof for evidence and 13 contain plausible records whose real
  profile is invalid. No threshold or profile was synthesized to turn these
  into winners. Across all actions the automatic router records 567 certified
  rank results and 2,384 dense terminals. Every 50 m/s^2 cell reaches automatic
  dense fallback without SIGSEGV; safe 0.25/4.0 actions may use rank. The
  router result matches an independent raw-factor SVD, and every cell's final
  status/commit/availability/plausible set/dispositions/refusal/risk result
  matches the uncapped dense reference;
- explicit large-step replacement, ill-conditioned window and unknown group
  mapping cases automatically use dense and expose their route reasons;
- cap 0/1/128, duplicate operation/action IDs, `+0/-0`, NaN, compact collision,
  checked count overflow, fake-safe and tampered-sidecar boundaries;
- zero-cap/oversize input, manager/risk/commit refusal, normal/deadline/queue
  overflow/exception final packets, and CSV omission identities. The real
  `RealtimeIntegrityPipeline` exception test duplicates a genuine generated
  occurrence at its explicitly installed external generator-to-census seam,
  then injects each
  candidate/post/PL exception. All generated/planned/actual/omitted counts and
  every occurrence/operation/semantic/duplicate-of/terminal row are identical
  across `lastAttemptOutput`, final packet, `integrity.csv`, and
  `candidates.csv`;

See `directed-results.txt`, `boundary-replay.json` and
`action-census-summary.json`.

## Acceptance

```text
normal clean: PASS
normal full all-target build: PASS (integrity_round2_scenario present)
tests target build: PASS
P0-06 directed/O07: 24/24 PASS
inherited integrity_v2: 85/85 PASS; FDE post-selection: 3/3 PASS
P0-04 risk oracle: 13/13 PASS
P0-05 publication regression: 13/13 PASS
complete CTest: 29/29 PASS, 98.56 s, exit 0
round2 gate: PASS as CTest test_round2_tools
golden-header cross-DSO ABI: PASS
git diff --check: PASS
```

The eighth independent reviewer passed the complete P0-06 gate after a fresh
normal build: directed 24/24, inherited regressions, complete CTest 29/29 in
97.65 s, the correlated-raw oracle, production 130-action chain, pipeline ABI
canary, 14-path patch replay and all five hash manifests. Supervisor acceptance
independently repeated clean/full/tests, directed 24/24, complete CTest 29/29
in 99.01 s, selected regressions, the golden-header pipeline canary and rebuilt
binary/loaded-library hashes. P0-06 is therefore `GOLDEN` at
`golden-p0-06-actions`.

## Explicit NOT_RUN

- Sanitizers/TSan/MSan: `NOT_RUN` (not configured for this P0 row).
- Physical UWB/IMU hardware: `NOT_RUN` (D09 is deterministic search-state
  correctness; no hardware qualification claim).
- Live ROS simulator capture: `NOT_RUN` (no standard configuration reaches
  130 distinct recovery actions; production ROS field-copy wiring is unchanged
  and final-packet/CSV propagation is directly tested).
- Before/after performance: `NOT_RUN` (P0 correctness row; no speed claim).
- Formal deployment qualification: `NOT_RUN` and unchanged; Gate J remains
  fail-closed.
