# P1-06 paused state — 2026-10-07

Status: `GOAL PAUSED / P1-06 NOT ACCEPTED`.

The user stopped this Goal on 2026-10-07.  This record freezes the state for
handoff; it is not an acceptance report.  The original Section 2 contract and
P1-06 have **not** passed, no `golden-p1-06` tag is authorized, and the current
uncommitted implementation must not be treated as a golden baseline.

The original audit input
`doc/UWB_IMU_PL_系统审计与复现报告.md` exists in the local
worktree and informed this Goal.  It is user-owned, untracked material and is
deliberately excluded from the pause checkpoint by an exact `.gitignore`
rule; it is not missing and this checkpoint does not replace it.

## Repository anchor and worktree identity

- Branch: `feature/realtime-uwb-imu-pl`.
- Checked-in anchor before the paused worktree:
  `4e99d4d8aa9712300389a7caed9d7809afc87d5a`
  (`validated-p0-07-nonrealtime`, a scope-limited non-golden reference).
- Mathematical/discrete comparison reference:
  `golden-p1-05-deterministic-concurrency`
  (`4657cf87539cf7699231642d3356c08456b1833e`).
- Paused product/config/test/tool diff SHA-256 (computed as
  `git diff --binary -- apps config include src test tools | sha256sum`):
  `df614ed52f0684dfb61217b30d1a96fe647493dbe7adce5f7897b4485c3a992a`.
- The worktree is intentionally dirty until the supervisor creates the pause
  checkpoint.  The hash above identifies only the product/config/test/tool
  diff; this document and evidence-index edits are outside it.

The unaccepted worktree contains: the user-authorized 38x20 scaled fault
schedule and its manifest/calibration validation; synthetic calibration IDs;
acceptance summarizer hardening; a certified full-row fallback and reset for
an invalid hierarchical history root; fail-closed handling for a frozen
window whose invalid model has no completed numerics; private test-only
history-root mutation hooks; and a diagnostic-only read-only graph/KKT
snapshot in the benchmark.  These changes preserve the public P0 ABI/API but
have not completed a single current-source full acceptance campaign.

## Frozen result ledger

Results from `final-v8` belong to its frozen earlier source/DSO identity.
They remain valid historical observations but cannot be inherited as PASS by
the later v9-v11 worktree.

| Gate | Frozen result | Scope and evidence |
|---|---|---|
| Revised 38x20 profile matrix | `PASS` (historical final-v8 identity) | 38/38 profiles, 760/760 terminal attempts, every fault cell has non-empty pre/fault/post segments, no omitted action coverage, peak RSS 775.188 MiB |
| Fault, transaction and publication safety | `PASS` (historical final-v8 identity) | Complete terminal work; rejects remain fail-closed; no partial protected output |
| FGO 4x200 | `PASS` (historical final-v8 identity) | 4/4 runs and all six configured FGO checks |
| Complete CTest | `PASS` (historical final-v8 identity) | 35/35 |
| ASan+UBSan / focused LSan | `PASS` (historical final-v8 identity) | 8/8 plus 4/4 focused; LSan 1/1 |
| TSan / MSan | `NOT_RUN` | Required runtime/toolchain was unavailable; no PASS is claimed |
| Synthetic calibration metadata | `PASS` (historical final-v8 identity) | Five versioned `simulation/synthetic` IDs bind model/config/protocol; this is not hardware calibration |
| 20-seed, 1200-attempt off-profile quality | `FAIL` | position RMSE 20/20, position P95 20/20, velocity RMSE 20/20, attitude RMSE 4/20 |
| 3-seed, 12000-attempt off-profile quality | `FAIL` | position RMSE 2/3, position P95 3/3, velocity RMSE 3/3, attitude RMSE 0/3 |
| Section 2.4 work completeness | `PASS` (historical final-v8 identity) | 135/135 attempts, zero exclusions |
| Section 2.4 strict golden equivalence | `FAIL` | 3/3 canonical comparisons failed and remain failed |
| Realtime numeric gate | `FAILED / DEFERRED_TO_REALTIME_GOAL` | core p99 6791.344 ms, arrival-to-publish p99 6791.360 ms, 132/135 deadline misses; full denominator retained |
| Full 38x300 matrix | `NOT_RUN / SUPERSEDED_BY_USER_SCOPE_CHANGE / NOT_CLAIMED` | User replaced only this matrix with 38x20; preserved partial raw is not an acceptance denominator |
| Current paused v11 source: 38x20, FGO, long quality, full CTest, sanitizer, Section 2.4 | `NOT_RUN` | Focused v11 diagnostics do not transfer prior PASS results to the current source identity |
| Original Section 2 / original P1-06 | `FAIL / NOT_CLAIMED` | Quality, strict equivalence and realtime gates are not passed |
| Revised non-realtime Goal | `PAUSED`, last acceptance status `FAIL` | Non-realtime quality and strict golden equivalence remain open |

Hardware qualification remains `CLOSED/NOT_CLAIMED`,
`formal_eligible=false`, and protected output disabled.  Nothing in this
checkpoint is a physical calibration or hardware certification.

## Four diagnostic/review rounds

1. **final-v8 — acceptance review failed.**  Coverage, bounded RSS, FGO,
   CTest and sanitizer evidence passed on that identity, but off-profile
   attitude/long-run quality failed, strict Section 2.4 equivalence failed
   3/3, and realtime remained deferred.
2. **repair-v9 — proposed explanation not accepted as closure.**  The
   invalid hierarchical history root was reproduced and a bounded certified
   full-row fallback was added.  The single-epoch UWB null-direction analysis
   did not prove a complete-model structural impossibility, so no acceptance
   rerun or PASS was allowed.
3. **focused-v10 — blocked/incomplete.**  A complete frozen graph was full
   column rank, central-difference Jacobian checks passed, and a bounded
   production-versus-batch fixture passed.  The actual failed long-run window
   and the valid-fallback downstream oracle were still `NOT_RUN`; final-v8
   failures remained unchanged.
4. **targeted-v11 — focused diagnosis only.**  Actual 1200/12000 failed
   windows were captured.  Batch LM improved their local objective but kept
   essentially the same failed long-run attitude/body-origin quality;
   selected-inverse/covariance and active-graph rank defects were not found.
   The current-instance valid full-row fallback mutation passed all downstream
   invariants.  No implementation defect explaining the quality failure was
   proven, and no full current-source acceptance campaign was run.

The detailed last round is
[targeted KKT v11](../evidence/p1-06-revised-nonrealtime-final/targeted-kkt-v11/README.md).
Earlier frozen conclusions remain in the evidence index; they are not
silently upgraded by later diagnostics.

## Blockers at pause

1. The fixed-model non-realtime attitude/long-run quality gate is still
   failed.  v11 rules out several local solver/Jacobian/covariance hypotheses
   but does not prove a repairable implementation defect.
2. Section 2.4 strict comparison against
   `golden-p1-05-deterministic-concurrency` remains 3/3 failed.  Any semantic
   mapping or deterministic-clock replacement baseline requires explicit
   authority and must retain the old strict failures.
3. After v9-v11 source changes, 38x20, FGO, long quality, complete CTest,
   sanitizers and Section 2.4 are `NOT_RUN` for the current source identity.
4. Realtime remains a separate deferred Goal.  Hardware eligibility remains
   closed and is not a route to pass this simulation Goal.

## Minimum safe resume sequence

1. Start from the pause checkpoint and verify its diff/hash against this
   record; do not use it as a golden baseline.
2. Obtain explicit authority for the Section 2.4 path: either a versioned
   semantic mapping that retains strict 3/3 FAIL, or a separately approved
   deterministic-clock baseline procedure.
3. If continuing the quality investigation without a model-contract change,
   build an independent long-horizon marginalization oracle from original
   measurements that does not consume the production marginal prior.  Do not
   tune the model, noise, priors, thresholds, risk, dof, coverage or quality
   denominator.
4. Review any resulting implementation change independently, then rerun on
   one frozen source/DSO identity: 38x20, FGO 4x200, prescribed 20-seed and
   12000-attempt quality runs, O01-O12, complete CTest, sanitizer subset, and
   full Section 2.4 comparison.  Preserve every failure, rejection, timeout
   and denominator.
5. Only all revised non-realtime exits passing can close the revised Goal.
   The original Section 2/P1-06 and realtime performance must remain
   separately reported.  Do not create `golden-p1-06` from this pause state.

## Evidence retention and commit boundary

The local 8.3 GiB evidence tree is retained in place but raw CSVs, campaign
directories, historical/stale iterations, logs and build artifacts are
ignored and must not be pushed.  Compact status, summaries, identities,
hashes, commands and exit codes allowed in the pause checkpoint are listed in
[the commit allowlist](../evidence/p1-06-revised-nonrealtime-final/PAUSE_COMMIT_ALLOWLIST.md).
The frozen `artifact-hashes-final-v8.sha256` is preserved verbatim; its
historical README checksum is not a current verifier after this pause index
evolved.  Current checkpoint documents and compact v11 proof files are
instead bound by `pause-checkpoint-2026-10-07.sha256` in that evidence root.
