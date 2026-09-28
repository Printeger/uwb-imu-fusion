# P0-07 fourth independent reviewer — FAILED

Verdict: **FAILED**, not BLOCKED. This review covered the third P0-07 repair
on top of `golden-p0-06-actions`. It is retained permanently and is not
superseded by a later repair. The master table was not updated, no commit or
tag was created, and P1 remains closed.

## Preserved independent execution results

- clean Release build PASS in 5m23.2s; explicit tests target PASS in 3m38.7s;
- complete CTest 32/32 PASS in 240.35s, including P0-07, P0-06, history and
  configuration gates;
- two fresh runner invocations and the pre-swap atomic-failure case PASS;
- source/artifact/binary/library/config/input/output hashes, JSON, complete
  patch apply/byte/reverse, diff checks and golden-header/current-DSO ABI PASS;
- HEAD/tag/status and user documentation were correct and no contract value,
  threshold, risk budget, denominator, hypothesis or action coverage changed;
- the reused sanitizer client entered a repeated ASan `DEADLYSIGNAL` loop
  without a usable stack and was interrupted. It is a retained failure, not a
  PASS, and requires a fresh compile and bounded ASan/UBSan/LSan rerun.

The review accepted N01: all public monitor, pipeline, watchdog and exception
exits fail closed, there is no environment bypass, and old-profile/config ABI
compatibility passes. It also accepted the real pipeline exception chain
through `lastAttemptOutput`, commit receipt/backend poison, final packet and
honest `NOT_ATTEMPTED` publication outcome. Those facts do not close the
independent-reference defects below.

## 1. Expected raw inventory still comes from production metadata

`completeRawReference` selects boundary rows from production
`window.slot_accounting` (around the then-current lines 539 onward), enumerates
explicit groups from `window.blocks` (around lines 709 onward), and obtains
fault columns and selected groups from transaction metadata. A production
omission or relabeling can therefore produce the same omission in the alleged
reference. There is no independently versioned complete row-ID/owner ledger,
nor a value-by-value comparison of the complete raw and whitened `H`, `z` and
`C`. The `T_b`/`F_b` checks compare only normal products and cross-products
(then-current lines 676–686), not their values and coordinate contract. The
README and O01 claims are consequently stronger than their evidence.

## 2. Hypothesis oracle is production-defined and uses a blanket tolerance

`verifyEveryHypothesisNumerics` constructs expected results from production
`models.modes`, `models.hypotheses`, each mode's `raw_group_maps` and
`effective_parameter_basis`, plus production `window.H/z` (then-current lines
791 onward and 826 onward). It does not independently enumerate the 504 modes
and hypotheses from frozen raw truth plus the fault manifest. Its uniform
`2e-6` gate is not derived per quantity from conditioning/certificates, and
several comparisons run only when production `profile_valid` or `entry.valid`
is already true. It therefore does not independently establish validity,
nullspace/discrete class, or absence of downward PL movement.

## 3. Action/FDE reference consumes production decisions

`independentDenseActionReferences` directly accepts the production `window`,
`raw_actions`, `models.hypotheses` and `evidence` (then-current lines 404–485
and call around 1450). Its plausible set, operation and covered modes are all
production inputs. It independently recomputes only rank, DOF, statistic,
post-test and coverage; it does not independently compute PL, risk ledger,
eligibility, selection tuple or final winner/refusal. `frozen-truth.json`
copies those production results. MissingProvenance likewise needs an
independent fixture/manifest-derived expected action and terminal rather than
production-optimal fields supplied by the test.

## 4. Bundle authority has a crash window

The runner performs two renames: old authority to a backup, then staging to
the authority path. Between them the authority does not exist and a crash can
leave no readable current bundle. The only injected failure precedes the
first rename, and no test covers validation/write/flush/pre-pointer/pointer
rename failures, process-boundary behavior, or directory fsync. Authority
must instead use immutable content-addressed/versioned bundles written and
fsynced completely before atomically replacing one small current pointer (or
manifest), so the old pointer remains readable until the commit point.

## Required repair

Add an independently versioned oracle manifest and parser driven only by the
frozen raw replay, configuration/fault manifest and explicit factor
construction specification. It must enumerate complete prior/IMU/UWB/history
row identities and ownership, modes/hypotheses/native maps/plausible set, and
deduplicated operations. It may use frozen nonlinear factor objects and
linearization values for numerical assembly, but must not use production
window/model/search/evidence/audit metadata to define expected entities.
Independently assemble and compare complete raw/whitened systems and history
elimination, derive per-quantity tolerances, classify every hypothesis even
when production says invalid, and compute every action's dense terminal,
post-test, coverage, PL, risk, eligibility, selection tuple and refusal/winner.
Generate the signed reference from that oracle rather than copying production.
Replace the authority protocol with immutable bundles plus one atomically
renamed pointer and cover all specified failure points. Finally compile a
fresh sanitizer client and retain this review's `DEADLYSIGNAL` failure.
