# P1-06 revised non-realtime evidence index

Current status (2026-10-07): `GOAL PAUSED / P1-06 NOT ACCEPTED`.

The authoritative handoff is the
[paused-state record](../../review/P1_06_PAUSED_STATE_2026-10-07.md).  The
checkpoint allowlist is [recorded here](PAUSE_COMMIT_ALLOWLIST.md).  No
`golden-p1-06` tag may be created from this state.

`artifact-hashes-final-v8.sha256` is a frozen historical manifest and has not
been rewritten.  Its checksum for this `README.md` no longer verifies because
the evidence index necessarily evolved to document the later v9-v11 work and
the pause.  That single historical README entry is therefore not used for
current verification; all other final-v8 entries remain independently
checkable.  The current pause documents and compact v11 artifacts are bound
by `pause-checkpoint-2026-10-07.sha256`.

The latest focused diagnosis is
[targeted KKT/history fallback v11](targeted-kkt-v11/README.md).  It is a
focused diagnostic result, not acceptance evidence: the current v11 source
identity is `NOT_RUN` for 38x20, FGO, long quality, complete CTest,
sanitizers and Section 2.4.  The complete campaigns summarized below are
frozen final-v8 historical evidence from an earlier source/DSO identity and
must not be inherited by the current worktree.

Historical final-v8 status: `FROZEN / REVISED_NONREALTIME_ACCEPTANCE_FAIL`.

The subsequent bounded repair investigation is recorded in
`repair-frozen-v9.md`.  It repairs the attempt-511 hierarchical history-root
failure via the certified full-row fallback.  Its single-epoch claim that the
fixed single-tag model makes the body-pose gate structurally impossible was
later disproved by the complete-graph v10 check and must not be used as the
current explanation.  v11 found no implementation defect that explains the
still-failed quality gate; Section 2.4 strict equivalence also remains failed.

This is the independent final evidence namespace for the user-authorized
2026-10-06 revised scope.  Realtime thresholds remain measured and are
reported as `FAILED / DEFERRED_TO_REALTIME_GOAL` when missed.  They do not
replace or waive any non-realtime correctness, coverage, publication-safety,
simulation-quality, crash-free, or RSS-below-1-GiB gate.

The fixed mathematical/discrete comparison reference is
`golden-p1-05-deterministic-concurrency`.  The source baseline for this run is
`validated-p0-07-nonrealtime` (`4e99d4d8aa9712300389a7caed9d7809afc87d5a`).
Hardware qualification is `CLOSED/NOT_CLAIMED`, and all calibration metadata
is strictly `simulation/synthetic`.

## 2026-10-06 matrix-size amendment

The user replaced the profile campaign only from 38 scenarios x 300 attempts
to 38 x 20 (760 planned attempts).  The accepted matrix namespace is
`profiles-38x20`; it must be generated afresh and must not resume or reuse any
run from `profiles-38x300`.  The protocol scales the original fault schedule
deterministically, preserving non-empty pre-fault, fault-active and
post-fault/recovery denominators in every fault scenario.  Nominal runs retain
20 normal attempts.  This reduction does not alter fault families,
magnitudes, the sensor model, noise, priors, risk, thresholds, dof,
hypothesis/action coverage, FGO 4x200, long-run/20-seed work, O01--O12,
sanitizers, complete CTest, or Section 2.4.

The earlier `profiles-38x300` directory and archived partial resume are
historical evidence with status
`SUPERSEDED_BY_USER_SCOPE_CHANGE/NOT_CLAIMED`.  Their complete and partial raw
records remain preserved, but none contributes to the revised 760-attempt
denominator and the original 11,400-attempt campaign is not claimed passed.

## Frozen implementation identity

The final source identity used by every accepted final campaign below is
`a488d2272d54c0813b65cc4608b88445aea1163817f63021837ae3e31b26dd31`;
the loaded production DSO identity is
`455f01fe81a4cf47a05891cb0c544a9f793e842ebc9dad31a9bc03350912ea26`.
Earlier `final-v4` through `final-v7` directories are retained history and are
not the final denominator.  The mistakenly started `final-v8` PL campaign was
stopped with SIGINT and is also historical; the correctly parameterized
4x600 campaign is `active-alarm-pilot-4x600-final-v9`.

## Final results

| Item | Result | Frozen evidence |
|---|---|---|
| Revised profile matrix | `PASS` | `profiles-38x20-final-v8`: 38/38 exit 0, 760/760 terminal, 705 committed, 55 fail-closed rejects, 534 deadline misses retained; fault cells have non-empty pre/fault/post denominators; peak RSS 775.188 MiB |
| Fault/transaction/publication contract | `PASS` | acceptance summarizer reports `all_attempt_contract_status=PASS`; no partial protected output, crash, or omitted terminal work |
| Resource bound | `PASS` | acceptance summarizer reports `all_attempt_resource_status=PASS`; matrix peak RSS 775.188 MiB and 4x600 peak RSS 574.906 MiB, both below 1 GiB |
| FGO 4x200 | `PASS` | `fgo-4x200-final-v8`, 4/4 exit 0; all six configured FGO quality checks pass |
| Active nominal 4x600 execution | `PASS` | `active-alarm-pilot-4x600-final-v9`, 2400/2400 terminal, 2130 committed and 270 explicit fail-closed rejects; no crash |
| Active nominal quality | `FAIL` | three profiles become history-invalid after attempt 510 and finish with 90 fail-closed rejects each; finite protected PL remains zero under the closed hardware qualification boundary |
| 20-seed off quality | `FAIL` | position RMSE 20/20, position P95 20/20, velocity RMSE 20/20, attitude RMSE only 4/20 |
| 3x12000 off quality | `FAIL` | position RMSE 2/3, position P95 3/3, velocity RMSE 3/3, attitude RMSE 0/3 |
| Complete CTest | `PASS` | `ctest-complete-final-v8.log`: 35/35, 547.36 seconds |
| ASan+UBSan subset | `PASS` | P0-07 8/8 plus final maturity/identity/arena subset 4/4; no sanitizer finding |
| LeakSanitizer focused check | `PASS` | leak detection enabled, 1/1; prior startup-loop evidence remains historical |
| TSan / MSan | `NOT_RUN` | retained toolchain probes: TSan runtime link prerequisite unavailable; GCC MSan unsupported |
| Simulation calibration metadata | `PASS` | five versioned `simulation/synthetic` IDs bind the protocol/model/config digest; hardware remains `CLOSED/NOT_CLAIMED`, `formal_eligible=false` |
| Section 2.4 work completeness | `PASS` | `gate24-final-v8`: 135/135 complete, zero excluded, peak RSS 554,836 KiB |
| Section 2.4 strict golden equivalence | `FAIL` | all three canonical comparisons against `golden-p1-05-deterministic-concurrency` fail; the failures are retained |
| Realtime numeric gate | `FAILED / DEFERRED_TO_REALTIME_GOAL` | Section 2.4 core p99 6791.344 ms, arrival-to-publish p99 6791.360 ms, 132/135 deadline misses; no attempts removed |
| Original Section 2 / P1-06 full acceptance | `NOT_CLAIMED` | revised scope and 38x20 amendment do not represent original 38x300 or realtime acceptance |
| Revised non-realtime Goal | `FAIL` | non-realtime off-profile attitude/long-run quality exits remain failed, so the revised Goal cannot be declared complete |

The off-profile failure was independently checked for timestamp alignment,
quaternion normalization and geodesic-error evaluation.  Its dominant
attitude error is coupled to the body-origin position error with mean
position/attitude slope about 0.268 m/rad, close to the configured lever-arm
norm 0.281 m.  With one UWB antenna the range model observes `p + R*l`; its
rotational Jacobian has a null direction along `l`, so rotation about that
axis can be exchanged against body-origin translation.  No implementation- or
evaluation-contract defect was found that can correct this without changing
the model, sensor arrangement, prior or noise, all prohibited in this Goal.

The machine-readable aggregate is `acceptance-final-v8.json`.  Its `FAIL` is
intentional and preserves both the strict golden mismatch and realtime
measurements; it must not be rewritten as original Section 2 acceptance.
