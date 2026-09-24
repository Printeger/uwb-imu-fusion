# P0-04 complete risk and selection-event evidence

Status: implementer `PASS`, pending independent reviewer and supervisor.

## Frozen identity and scope

- Base: annotated `golden-p0-03-numerics^{}` =
  `9d81a25f68e7552aa14f83b74e9a4585d8ebef82`.
- Uncommitted `HEAD` remained that same SHA throughout implementation.
- Scope is only P0-04 / R03 / D05 / D06 / O06. The master verification
  table was not edited. The user-owned untracked Chinese audit and
  `doc/review/` were read and not modified.
- No risk total, allocation, prior, detector threshold, alert limit, fault
  family, hypothesis/action coverage, denominator, timeout, or exact fallback
  was changed.

## Red reproduction

The independent pre-fix oracle failed both cases (`exit 1`, 2/2 failed):

1. the miss term was `ASSUMED_UNVALIDATED` and omitted from the charged total
   (`3.13e-6` rather than the independent quad result `3.2e-6`);
2. two eligible actions with identical plausible labels but no numeric shared
   event proof were premerged into one `3e-5` event instead of two singleton
   events totaling `6e-5`, so an unchanged `4e-5` budget incorrectly admitted
   commit.

The exact assertions are retained in `first-failing-results.txt`; the repaired
oracle is `test/test_p0_04_risk_oracle.cpp`.

## Contract repair

- Allocation precheck, complete-bound closure, term validation, and formal
  eligibility are four explicit states. Unknown escape/omitted/selection terms
  are NaN/UNKNOWN and fail the protected-output gate; finite unvalidated bounds
  are still charged.
- The high-precision ledger charges
  `pi_h * max(alpha_h, beta_h)` as allocation plus miss excess, and charges
  escape, omitted, envelope, and selection terms in the unchanged total.
- Selection receives every original eligible action. Plausible labels are not
  grouping keys. The compatibility sidecar contains only caller claims, not a
  recomputable P0-03 proof payload, P0-02 accepted-event certificate, or frozen
  window/time/output digest. It therefore cannot authorize sharing. Production
  and the sidecar overload both charge every action as a singleton until a real
  independently verifiable common-reference producer is wired end-to-end.
- Every explicit risk bound must be finite and non-negative. A structural
  no-event proof accepts only its canonical known/validated zero sidecar;
  dormant nonzero evidence is never overwritten. Contradictory known/validated
  flags, negative/NaN/infinite bounds, overflowed exports, and rejected
  hypothesis tails make the aggregate UNKNOWN and fail closed; no rejected
  risk is continued as a known zero. An independent Cartesian oracle exercises
  112 omitted, 112 envelope, and 56 selection representations, including
  `-Inf`, negative, IEEE `-0`, `+0`, positive, `+Inf`, and NaN values.
- Each action is charged against the complete represented hypothesis
  allocation, including no-alarm outcomes. Disjoint groups exceeding the
  unchanged total disable commit and protected availability.
- P0-01 census, P0-02 dual-channel certificate, and P0-03 numerical proof
  consumers remain exercised by the directed and complete suites.

## Acceptance

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target tests -j2
exit 0

P0-04 independent oracle: 13/13 PASS
FDE post-selection grouping: 3/3 PASS
P0-01/P0-02/P0-03 directed preservation set: 10/10 PASS
round2 tool unittest: 7 discovered, 5 PASS and 2 optional-runner SKIP;
the authoritative complete CTest wrapper PASS is recorded below

Golden-header ABI client: link/run PASS; legacy six-argument
`FdeManager::decide` symbol and all golden public-struct sizes/offsets retained.

cd /home/mint/ws_fusion_uwb/build/uwb_imu_pl
ctest --output-on-failure
exit 0; 27/27 targets PASS; 103.01 s (retained full output after clean rebuild)

git diff --check
exit 0
```

See `directed-results.txt`, `ctest-results.txt` (complete console output), `attempt-journal.tsv`, and
`boundary-replay.json` for retained summaries and decisions.
The three independent FAILED reviews are retained in
`reviewer-first-failed.md`, `reviewer-second-failed.md`, and
`reviewer-third-failed.md`; all reviewers were read-only and did not modify
the worktree.

## Explicit NOT_RUN

- Sanitizers/TSan/MSan: `NOT_RUN` (not configured in this build).
- Hardware/ROS sensor campaign and rare-event qualification: `NOT_RUN`
  (outside O06; production remains fail-closed for unknown qualification).
- Before/after performance: `NOT_RUN` (P0 correctness row, no speed claim).
- Deployment calibration approval: `NOT_RUN` (Gate J remains separate and
  formal/protected output remains disabled without complete qualification).
