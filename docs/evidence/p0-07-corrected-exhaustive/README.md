# P0-07 corrected-exhaustive evidence

> **Checkpoint notice (2026-09-28): validation is paused and this work is not
> GOLDEN.** The eleventh independent review remains FAILED. The subsequent
> process-isolated oracle work was checkpointed before its full acceptance run
> completed. See [`checkpoint-status.md`](checkpoint-status.md) for the exact
> interrupted state. Older results below are retained as attempt history and
> must not be read as a current PASS claim. No `golden-p0-07` tag is authorized.

Baseline: `golden-p0-06-actions` / `8ca1d713704e47b922b84d7cc854da97c27dc20b`.

The first independent review was **FAILED** (not blocked).  Its complete
findings and preserved execution facts are in
[`reviewer-first-failed.md`](reviewer-first-failed.md).  The repair evidence
below must be read together with that retained failure record.

The second independent review was also **FAILED** (not blocked).  Its complete
findings are retained in
[`reviewer-second-failed.md`](reviewer-second-failed.md).  In particular, it
rejected the monitor-entry N01 gap, the use of two different replay fixtures,
and the hand-assembled O12 evidence. The third independent review was also
**FAILED** (not blocked); its complete findings are retained in
[`reviewer-third-failed.md`](reviewer-third-failed.md). It accepted N01/ABI but
rejected the partial raw oracle, self-reported action reference, bypassed
exception/publication chain, and non-atomic runner. The historical
LeakSanitizer result is unchanged. The fourth independent review was also
**FAILED** (not blocked); its complete findings, including its unbounded
`AddressSanitizer:DEADLYSIGNAL` attempt, are retained in
[`reviewer-fourth-failed.md`](reviewer-fourth-failed.md). It rejected the
third-repair oracle because expected row/mode/action identities still depended
on production metadata, and rejected the two-rename authority update because
it admitted a no-authority crash window.

The fifth independent review was also **FAILED** (not blocked); its complete
findings and preserved clean-build/test/sanitizer/ABI results are retained in
[`reviewer-fifth-failed.md`](reviewer-fifth-failed.md). It rejected the fourth
repair because the manifest still stored plausible/action/terminal/winner
outcomes, candidate construction consumed production added blocks, PL/risk/
selection were not independently recomputed, the MissingProvenance case was
hand-authored, and an existing content-addressed identity was trusted without
full revalidation. The sixth independent review was also **FAILED** (not
blocked); its full result, including the sanitizer macro compile failure and
sanitized ABI `DEADLYSIGNAL`, is retained in
[`reviewer-sixth-failed.md`](reviewer-sixth-failed.md). It rejected the
remaining recipe/config split, production-derived raw inventory, missing
candidate PL/risk oracle, unexecuted recipe declarations and conflicting
frozen outputs. The seventh independent review was also **FAILED** (not
blocked); its full findings and execution facts are retained in
[`reviewer-seventh-failed.md`](reviewer-seventh-failed.md). It rejected
unconsumed recipe/config/fault-manifest fields, production-fed hypothesis risk
inputs, incomplete prior/IMU row-owner binding, and inconsistent O05/O09/O11
evidence references. The eighth independent review was also **FAILED** (not
blocked); its complete verdict, eleven stale artifact-manifest failures and
fresh LSan `AddressSanitizer:DEADLYSIGNAL` failure are retained in
[`reviewer-eighth-failed.md`](reviewer-eighth-failed.md). It rejected legal
recipe leaves that were declared but not typed and consumed. The eighth repair
below is awaiting a new independent reviewer; no earlier failure has been
removed or relabelled.

## Result

Eighth-repair implementation candidate for the reproducible software boundary;
final PASS still requires the next independent review. This change does **not** claim
that the configured IMU noise is hardware-calibrated.  All shipped profiles
declare `UNQUALIFIED_CONFIGURED_PARAMETER` and
`SHARED_SAMPLE_CORRELATION_UNQUALIFIED`; therefore the production pipeline
forces `formal_eligible=false`, `Availability::Unavailable`, reason code
`IMU_MODEL_UNQUALIFIED`, and no protected publication.  The directed test
also enables the platform publication switch in memory, proving the model
qualification gate independently closes the protected path.

The acceptance chain is one frozen deterministic 24-epoch raw IMU/UWB replay,
with a +2 m fault on epoch 23 / anchor 1, after the fixed-lag boundary has
crossed several times. `oracle-manifest-v1.json` is the versioned independent
row/owner/group/fault/action contract. From that manifest, the frozen nonlinear
factors, and frozen linearization values, the oracle reconstructs every prior,
IMU, UWB and history row from parsed slot/group/row-identity/order rules and
the frozen raw replay before consulting production transaction objects. The
transaction inventory, slots, covariance and source IDs are comparison
targets; only the shared frozen nonlinear-factor numerical primitive is
linearized. It eliminates the old variables from 268 raw history rows, obtains the
103-row effective summary over the declared 15-column separator, and compares
the complete explicit and summary H/z, raw UWB H/z/covariance/whitener,
information/RHS/constant, objective, state/full covariance, rank/DOF and every
R/d/T_b/F_b coordinate. It never reads production H/z to construct expected
values. This oracle
exposed and caused repair of a real correlated-covariance coordinate bug:
GTSAM supplies an upper information root for the UWB linearization, while the
historical UWB fault map had used a lower inverse-Cholesky root.  H, z and F
now consistently remain in GTSAM's upper-root coordinate.  The configured
covariance, thresholds and risk values are unchanged.

The identical replay then drives the real production
`buildIntegrityWindow`/history summary over fixed-lag crossings 22, 23 and 24,
then production hypothesis generation, detector, evidence and uncapped action
census.  At the real alarm it has 504/504/504
expected/represented/evaluated modes, seven generated actions and seven actual
kernel evaluations.  A second independent dense path builds each operation
set from frozen raw truth. For every action and every plausible hypothesis it
independently computes the candidate fault map, Gram, protected response,
nullspace class, total and dual-channel slopes, finite/infinite base PL, full
risk-term ledger, eligibility and selection tuple before applying N01 as the
final formal gate. Expected hypothesis prior, `p_md` and HMI allocations come
only from the parsed recipe; production hypotheses are used only as the actual
side of a later field comparison. These fields match the available production certificates;
mathematically unbounded actions are checked as such before the production
path's expected early refusal. The plausible set is derived at runtime from each independently
computed profile and the configured chi-square threshold; the manifest's
observed list is never loaded as expected truth. KEEP_ALL, each plausible
occurrence, exact operation identity and the single-anchor union are then
derived from the recipe. Only action 7 covers the complete plausible set; every PL remains
infinite and invalid, so FDE reaches an honest refusal.  Separately, a
MissingProvenance fixture removes the recipe-named current replacement factor
before generation, then drives the real generator, uncapped action search,
dense candidate evaluator and FDE manager. It naturally produces nonzero
MissingProvenance actions and refuses them instead of dropping a mode/action.

Every one of the 504 hypotheses is independently enumerated as eight anchors
times 21 onset epochs times three fault families. Native maps are built from
the manifest and frozen raw factors/whiteners, including boundary-onset
expansion and the recipe-declared one-column effective numerical basis for a
terminal-onset ramp (while preserving its two-coordinate physical identity).
Independent dense SVD
algebra checks Gram, profile statistic, protected response and three-axis PL
slope. Each comparison tolerance is derived from that quantity's dimension,
scale and measured condition certificate; there is no blanket `2e-6` gate.
The oracle also derives and unconditionally compares profile validity and
finiteness, PL validity, Gram rank, nullspace class and the exported discrete
detection class for all 504 entries; an invalid production entry cannot skip
its expected classification or finite/infinite contract.

A fresh estimator replays the identical raw stream through
`RealtimeIntegrityPipeline`, including selection/commit/publication, and
matches detector outcome and hypothesis/action census before ending in the
same honest N01 refusal.  A final replay-family epoch arms the production-only
`BeforeReceiptCreation` test seam and calls the real pipeline.  The test reads
the actual catch/`lastAttemptOutput`, backend receipt and poison state, then
finalizes that output.  There is no ROS publisher in this executable, so both
normal and exception packets honestly record `NOT_ATTEMPTED` and the third
timing boundary is packet-ready, not publish arrival.  The packet carries a
stable semantic digest separate from the intentionally variable timing
digest.

The older nine-row dense/SVD/high-precision/order-invariance test remains only
a supplemental numerical unit test.  It is not combined with the production
replay and is not the basis of the D12/O12 acceptance claim.

The N01 test constructs the exact adjacent trapezoid covariance (diagonal
`sigma^2/2`, adjacent off-diagonal `sigma^2/4`) and verifies the conservative
independent interval bound `sigma^2 I - C` is positive semidefinite.  V1 is
deliberately closed because this repository contains no authenticated hardware
artifact verifier or artifact.  An arbitrary ID and even both recognized
declaration strings remain unqualified.  Old profiles lacking the two
documentation-only declarations still load and default unqualified.  No
numeric sigma, detector threshold, alert limit, risk, prior, action,
hypothesis, or statistical denominator changed.

`tools/generate_p0_07_evidence.py` consumes a versioned frozen input and a
separate versioned regression observation, directly runs the recipe-checking
test binary, and validates the emitted raw rows and stream-internal census,
identity and risk consistency. It deliberately never reads the observation's
plausible/action/terminal/winner fields as expected truth; those expectations
are derived inside the test from the recipe before production comparison. A
fresh authority is an
immutable content-addressed `bundles/<sha256>/` directory selected by one
atomically renamed `current.json` pointer. Every file and containing directory
is flushed before the pointer commit. Validation, write, flush, pre-pointer and
pointer-rename failures leave the old pointer and selected bundle byte-identical;
a post-commit-cleanup failure leaves the new selected bundle complete and the
old immutable bundle readable. Existing identities are accepted only after
manifest identity, exact inventory, every file hash and intended semantic
bytes are revalidated. The expanded matrix covers same-content and
different-content concurrent writers, corrupt-existing-identity rejection,
an old-pointer reader and post-commit cleanup while `current.json` always
selects a complete bundle. A fresh-process reader verifies every injection.
The retained `authoritative-replay/` is the sole frozen epoch-23 / 504-mode /
seven-action acceptance snapshot. The superseded epoch-10 / 240-mode /
six-action files live only under the explicitly non-authoritative
`legacy-attempt/epoch10-240modes-6actions/` directory. A second fresh run
must match all non-timing/non-RSS fields, including the stable semantic packet
digest.  Timing and RSS remain present and are recomputed, not frozen.
The directed acceptance test also reads and validates the snapshots.
`authoritative-replay/input-attempts.tsv` retains
all 24 production replay inputs and its terminal/census/coverage/risk/work,
three timing boundaries, deadline, complete-work and RSS fields.
`authoritative-replay/alarm-actions.tsv` retains every generated action's terminal, candidate/post/
coverage/PL/risk/selection values and stable identity/reason hashes.
`authoritative-replay/o12-correctness-smoke.json` mechanically reports all-attempt p50/p95/p99/max,
deadline misses, 24/24 complete work and peak RSS.  Its boundary names state
that no transport publication was attempted.  These slow results are
retained: this is a correctness smoke, not a P1 optimization or 40 ms pass.

Reproduce from the package source directory:

```sh
python3 tools/generate_p0_07_evidence.py \
  --binary /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_p0_07_corrected_exhaustive \
  --input docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-input.json \
  --reference docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-truth.json \
  --compare-dir docs/evidence/p0-07-corrected-exhaustive/authoritative-replay \
  --output-dir /tmp/p0-07-fresh-bundle
```

The retained authoritative invocation exits 0; see
`authoritative-replay/evidence-command.txt`. There is no second current file
set with the same names at the evidence root.

## Verification summary

- clean package build after third-review repair: PASS, exit 0 (Release, `-j2`,
  5m23.8s)
- explicit tests target after that clean: PASS, exit 0 (3m37.2s)
- strict configuration regression: 15/15 PASS
- P0-07 directed tests after third-review repair: 7/7 PASS
- P0-07 directed tests after fourth-review repair: 7/7 PASS, 15.28 s
- content-addressed authority failure matrix: PASS for five pre-commit
  injection points and post-commit cleanup, including fresh-process readers
- selected third-repair CTest gate: 7/7 PASS, 134.87 s
- complete pre-clean third-repair CTest: 32/32 PASS, 240.61 s
- complete post-clean third-repair CTest: 32/32 PASS, 239.95 s
- complete pre-clean fourth-repair CTest: 32/32 PASS, 332.51 s
- fourth-repair clean Release build: PASS, 5m22.5 s; explicit tests target:
  PASS, 3m39.4 s
- complete post-clean fourth-repair CTest: 32/32 PASS, 334.66 s
- final independent-classification directed test: 7/7 PASS, 15.26 s;
  final P0-07/runner/authority matrix: 3/3 PASS, 138.81 s; final complete
  CTest: 32/32 PASS, 332.40 s
- fresh seven-case ASan+UBSan directed-test client: 7/7 PASS in 148.203 s;
  fresh leak-enabled N01/ABI subset: 2/2 PASS, exit 0; see
  `sanitizer-results.txt`
- fourth-repair fresh ASan+UBSan directed-test client: 7/7 PASS in 150.767 s;
  fourth-repair LSan-enabled N01/ABI subset: 2/2 PASS; one abort-on-error LSan
  attempt first repeated the retained stackless `DEADLYSIGNAL` failure
- `git diff --check`: see `verification.txt`
- fresh golden-header/current-DSO config loader/by-value ABI canary:
  `ImuNoiseConfig 80/8`, `IntegrityConfig 2544/8`, PASS
- fifth-repair final directed binary: 7/7 PASS in 16.487 s; runner consecutive
  same-identity writes and expanded concurrent/corrupt/old-reader authority
  matrix: PASS
- fifth-repair clean Release build and explicit tests target: PASS; final
  post-clean CTest: 32/32 PASS in 335.47 s
- fifth-repair fresh ASan+UBSan client: 7/7 PASS in 160.061 s; leak-enabled
  N01/ABI subset: 2/2 PASS in 98 ms, exit 0
- fifth-repair fresh golden-header/current-clean-DSO ABI canary: 80/8,
  2544/8, offsets, load/by-value/destruct and legacy/V1/V2 exports PASS
- sixth-repair recipe/config/raw-ledger/action-math directed binary: 7/7 PASS
  in 17.239 s; fresh sole-authority runner comparison: PASS in 15.934 s.
  Complete pre-clean CTest: 32/32 PASS in 339.12 s; clean Release package
  build and explicit tests target: PASS; post-clean P0-07/runner/authority
  matrix: 3/3 PASS in 148.51 s; post-clean golden ABI/export census: PASS.
  The fresh full ASan+UBSan client hung with retained stackless DEADLYSIGNAL;
  isolated non-main cases passed 4/4 and leak-enabled N01/ABI passed 2/2, but
  the full sanitizer attempt remains FAIL.
- seventh reviewer result is retained in `reviewer-seventh-failed.md`,
  including its first-startup sanitizer failure and same-binary 7/7 retry
- seventh-repair strict consumed-field/config/fault-manifest binding and
  independent hypothesis-risk/506-row owner oracle: directed same-replay PASS
  in 16.203 s; complete CTest 32/32 PASS in 342.98 s
- final production row-ownership certificate uses only the estimator's actual
  transaction and FactorLedger inventory.  The recipe/frozen-replay side is
  generated first and then compares all 506 stable row IDs, owner groups,
  group-local rows and covariance placements, including prior and IMU rows.
  It is a non-virtual read-only diagnostic method and adds no object field or
  vtable/layout change.
- seventh-repair fresh ASan+UBSan client: 7/7 PASS in 167.966 s, exit 0,
  without sanitizer finding; leak-enabled N01/config-ABI subset: 2/2 PASS in
  96 ms, exit 0
- seventh-repair golden-header/current-DSO ABI and export census: PASS; 80/8,
  2544/8, offsets, by-value load/destruct and all three decide overloads
- seventh-repair package-scoped clean full Release build and explicit tests
  target: PASS; post-clean P0-07/runner/authority matrix: 3/3 PASS in 148.37 s
- seventh-repair post-clean complete CTest: 32/32 PASS in 341.87 s
- final-certificate directed binary: 7/7 PASS in 17.190 s; full current-source
  CTest: 32/32 PASS in 342.15 s
- final-certificate fresh ASan+UBSan: 7/7 PASS in 176.177 s; leak-enabled
  N01/config-ABI subset: 2/2 PASS in 105 ms
- final-certificate package-scoped clean Release build and tests target: PASS;
  post-clean P0-07/runner/authority matrix: 3/3 PASS in 149.23 s
- post-clean DSO/direct-test hashes begin `1a3fc601` / `e59bdaac`; the complete
  values are retained in `binary-hashes.sha256`
- final complete patch covers 65 intended files and passes fresh-golden
  apply/byte/reverse validation; user-owned docs remain excluded
- eighth-repair strict recipe gate enumerates all 174 legal leaves; mutating
  each leaf must reject parsing/semantics or alter the independent expected
  fingerprint. Root/nested unknown and missing-key mutations also reject.
  Authority, all history/served partitions, boundary onset, missing-provenance
  recipe/recoverability, the exact six classification comparisons, and all
  four observed-only fields are typed and consumed; observed fields remain
  non-authoritative.
- eighth-repair directed suite: 8/8 PASS in 17.168 s; complete pre-clean CTest:
  32/32 PASS in 340.61 s. Package-scoped clean Release build PASS in 5m13.7s,
  from-zero tests target PASS in 3m38.7s, and post-clean complete CTest 32/32
  PASS in 341.09 s.
- eighth-repair freshly compiled current-source sanitizer binary: ASan+UBSan
  full suite 8/8 PASS in 172.869 s; its first bounded leak-enabled subset run
  2/2 PASS in 104 ms, exit 0, with no finding. The eighth reviewer's earlier
  LSan startup failure remains FAIL.
- eighth-repair golden-header/current-DSO ABI canary: 80/8, 2544/8, offsets,
  load/by-value/destruct PASS; exactly three decide exports and one non-virtual
  read-only ownership diagnostic export.
- ninth review is frozen verbatim in `reviewer-ninth-failed.md`: it FAILED the
  generic all-member fingerprint mutation pseudo-gate, a fresh ASan startup,
  and the row-owner audit's read of the live ledger. None is relabelled.
- ninth repair replaces the fingerprint with
  `semantic-leaf-classification.tsv`: all 174 leaves are explicitly classified
  as 50 construction semantic sinks, 116 strict/semantic rejections, or 8
  observed-only validations. Construction mutations independently re-derive
  named canonical artifacts; observed mutations preserve every expected
  artifact byte-for-byte and are rejected by the actual-observation validator.
- row ownership/covariance provenance is frozen during prepare into a private,
  immutable, lifetime-keyed sidecar. Audit no longer reads the live ledger.
  Copy, concurrent audit-versus-commit, post-commit byte stability and
  same-address reuse are covered without a public ABI field or exported helper.
- ninth-repair directed suite passed 8/8 in 17.361 s. An initial full run
  honestly failed 31/32 on auxiliary rows in a test UWB group; after explicitly
  classifying those rows, P0-05 passed and pre-clean CTest passed 32/32 in
  339.99 s. Clean build passed in 5m18.8s, from-zero tests in 3m41.6s and
  post-clean CTest 32/32 in 341.74 s.
- sanitizer history remains complete: SHA `066f5aaa...` first passed full8 but
  its first LSan run reported external legacy-TBB 12,312 bytes. With an explicit
  sanitizer-only TBB lifetime, fresh SHA `3a4631bd...` passed its first full8
  8/8 in 171.247 s and first leak-enabled run 2/2 in 131 ms, exit 0/no finding.
- ninth-repair post-clean golden-header/current-DSO ABI passed 80/8, 2544/8,
  offsets and load/by-value/destruct. Exports remain exactly three decide plus
  one audit, with no private certificate builder export. DSO SHA begins
  `392d4ec3`.
- ninth-repair complete patch covers 67 intended files and passes fresh-golden
  apply, byte-for-byte comparison and reverse-check; user docs and recursive
  generated artifacts remain excluded.

External real-hardware calibration and physical HIL remain `NOT_RUN`: no
calibration dataset, sensor, or certified rig is present.  This is not
reported as a pass; the production-reachable fail-closed behavior is what is
verified.  TSan/MSan full-DSO runs are also `NOT_RUN` because the supplied
ROS/GTSAM dependency stack is not instrumented; ASan+UBSan is run on the
directed client without pretending that it instruments those prebuilt DSOs.

Tenth-review repair closure (pending independent review):
- `reviewer-tenth-failed.md` freezes the complete FAILED-not-BLOCKED verdict:
  the old 174-leaf check serialized inputs into strings, used catch-all fixed
  rejection, and hard-coded the observed actual side.
- `test/p0_07_leaf_cases_v1.tsv` now declares exactly 174 unique path cases:
  38 DERIVED, 128 FIXED and 8 OBSERVED. DERIVED cases execute a structured
  typed evaluator for raw/served H-z-C and ownership, 504 mode maps/profiles/
  plausibility, derived actions/replacements, action-by-hypothesis PL/risk/
  eligibility/selection/refusal, missing provenance and tolerance classes.
- FIXED cases name an invariant and exact path-specific rejection reason;
  OBSERVED cases keep the expected typed bundle unchanged and reject against
  an `ActualObservation` captured from the same production replay.
- a source meta-test prevents the typed evaluator from consuming production
  window/model/search/audit objects as expected artifacts. Directed Release
  suite PASS 9/9 in 18.18 s.
- package-scoped clean Release build and from-zero tests target PASS; complete
  post-clean CTest PASS 32/32 in 341.99 s after the support-axis correction.
- fresh ASan+UBSan PASS 9/9 in 179.97 s and leak-enabled subset PASS 2/2 in
  0.18 s. The pre-fix Eigen assertion and one bounded stackless redirected
  sanitizer loop remain retained failures.
- golden-header/current-DSO ABI PASS: 80/8, 2544/8, frozen offsets and
  load/by-value/destruct; exports remain 3 decide + 1 audit + 0 private builder.

`attempt-journal.tsv` includes every failed command attempt.  The initial
incremental run against a stale pre-clean test executable exposed a header
layout mismatch (exit 139), the first independent review FAILED, both early
ABI-canary link failures, and both production replay repair failures remain
visible; later successful runs do not erase them.

`complete-diff.patch` excludes itself, `artifact-hashes.sha256`, and
`verification.txt` to avoid a recursive generated-artifact cycle.  It is
checked by applying to an archive of the baseline and by reverse-apply
checking the resulting tree.  `artifact-hashes.sha256` is the self-checking
manifest for the retained evidence files.
