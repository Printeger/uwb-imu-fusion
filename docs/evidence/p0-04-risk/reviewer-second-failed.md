# P0-04 second independent review — FAILED

Reviewer role: read-only independent reviewer; no worktree modifications.

Verdict: **FAILED**. The step was not eligible for the master-table update,
commit, or `golden-p0-04-risk` tag.

## Blocking findings

1. The risk ledger still accepted negative/illegal terms. An independent probe
   with `omitted_bound=-2`, `known=true`, `validated=true` produced:

   ```text
   negative_complete=1 all_validated=0 formal=0 charged=-2
   ```

   A rejected hypothesis was also skipped while the aggregate fault/miss terms
   remained numerically known, treating the rejected risk as zero.
2. `ActionSharedEventV1` remained a caller-populated assertion. The consumer
   checked only fields supplied together by the same caller, without validating
   a real P0-03 proof payload, P0-02 accepted-event certificate, or frozen
   window/time/output certificate. Fully invented values produced:

   ```text
   forged_valid=1 groups=1 charge=3e-05
   ```

   The correct fail-closed result is two singleton events totaling `6e-5`,
   which exceeds the unchanged `4e-5` budget. The existing test copied the
   sidecar from the action and was not independent evidence.
3. The evidence bundle did not retain this review's predecessor FAILED record;
   the journal contained only a repair summary.

## Positive evidence that did not clear the blockers

- Exact quad boundary, risk-before-selection gate, no-sidecar production
  singleton behavior, final authoritative ledger, and old-header ABI passed.
- Directed tests (9/9, 3/3, inherited 10/10), clean build, complete CTest
  27/27 (102.84 s), hashes, prior P0 regression, and explicit NOT_RUN records
  passed.
- No budget, alert limit, threshold, prior, or coverage change was found.

## Required repair

Reject negative/NaN/infinite or semantically unknown bounds, make rejected
hypothesis risk UNKNOWN or provide an explicit conservative bound, reject
caller-forged shared-event claims, add independent invalid-input/forgery tests,
and retain both FAILED reviews verbatim in the evidence history.
