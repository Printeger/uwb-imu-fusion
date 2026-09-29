# P1-03 failed attempt 1: assembled-matrix suffix update

Status: `FAILED`; no commit or tag is authorized from this attempt.

Baseline: `golden-p1-02-shared-dual-numerics`
(`435d682440f34fdeec22c237e4490b18c412cc74`).  P0-07 remains
`VALIDATION_PAUSED`.

## Scheme and stop condition

The attempted cache retained the explicit two-stage square-root state and
used Givens rotations only when the complete dependency proof established a
strict raw-row suffix.  A repaired variant also allowed fault columns to be
added as a suffix when they were exactly zero on every retained row.  Equal
content under a different owner, changed row provenance, payload tampering,
update/remove, reordering, rewhitening, marginalization and recovery-version
changes all rebuilt or failed closed.

The module-level non-zero append reproduction passed against an independent
full Householder rebuild for `R_b`, `T_b`, `F_b^T F_b`, `F_b^T d_perp`,
`kappa` and `nu_perp`.  Production did not satisfy that closure.

## Results

- `HistoryFaultSummary.P103*`: 2/2 PASS.
- Full `test_history_fault_summary`: 13/13 PASS before the suffix-column
  extension; the two P103 cases passed again after it.
- Full `test_history_summary_pipeline`: 11/11 PASS before the suffix-column
  extension, including the independent O01 raw-factor oracle.
- O03 action search: 24/24 PASS.
- O05 `test_integrity_v2`: 99/99 PASS.
- O09 deterministic/repeated/worker: 2/2, 1/1, 1/1 PASS.
- O12 directed corrected-exhaustive: 2/2 PASS.
- One 45-attempt strict precheck against the retained P1-02 formal run:
  13 files, 855 rows, zero semantic mismatch; 45/45 complete.
- Production closure audit over 30 consecutive epochs:
  `requests=30`, `incremental_appends=0`, `exact_hits=0`,
  `full_rebuilds=30`, `invalidations=29`, retained rows `467`, retained bytes
  `2002496`; final reason `linearization version changed`.

The 45-attempt precheck timing is not claimed as R09 performance evidence,
because the production audit proves that no non-zero incremental update was
served.  Formal 3x45 performance, long soak/RSS, complete CTest and ABI client
were therefore `NOT_RUN` after this stop condition.

## Required design switch

The next attempt must move below assembled-window memoization: maintain the
factor-group square-root state at backend commit/marginalization time and use
the actual changed-key, relinearized-factor, removed-slot and boundary-factor
closure.  Unknown closure must still perform the complete rebuild.  Repeating
or broadening assembled-matrix suffix matching is not an acceptable retry.
