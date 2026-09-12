# PL CUSUM Production E2E Integration and Accuracy Result

## Verdict

`E2E_FULL_SYSTEM_PASS_DEVELOPMENT`

This verdict is limited to the locked controlled SFUISE Walk1 persistent-NLOS
experiment. It is not a general superiority or certified-integrity claim. The
non-gating six-input diagnostic is reported separately below.

## Frozen provenance

- HEAD and origin at start: `9c592ba9a4e4624ecd9f131d0435f57e42ce08d7`.
- Pre-existing worktree item: untracked `doc/v2/ie_0911/`; it was not read,
  changed, or added.
- Prior sealed evidence:
  `/home/mint/ws_fusion_uwb/res/pl_bidirectional_cusum_support_20260912T024240Z`.
- Prior evidence verification: 126/126 artifacts verified.
- Admitted support identity:
  `plbidirectional-sha256:5123ce9e940d7a4c6b1c46e8cc09944827c169cbb191f1b2018287b995121cde`.
- Sealed protocol snapshot SHA-256:
  `52823678511d266e1c30eb62c6e6cb974adba3b9933cb4b2f953a1f63c33d71d`.
- New evidence root:
  `/home/mint/ws_fusion_uwb/res/pl_cusum_e2e_integration_accuracy_20260912T041820Z`.

The protocol was sealed before the formal injected production evaluation. No
detector, Stage2, gate, compensation, primary-policy, association, or alignment
parameter was changed after observing the accuracy result.

## Production integration

The production mode is `pl_bidirectional_cusum`; its immutable provider label is
`PL_BIDIRECTIONAL_CUSUM_V1`. ConfigLoader requires explicit finite/domain-valid
forward/backward kappa and thresholds, gap threshold, provenance,
`score_recoverability=true`, and `final_inference_enabled=true`. Missing values,
oracle support, and an illegal final-inference combination fail closed.

The provider reuses the admitted conditional-z, forward CUSUM, and backward
support cores. It collects conditional-z from the same pre-commit runtime
location as the admitted dynamic shadow, freezes the bidirectional partition
before Stage2, and passes that immutable partition through the existing cache,
Stage2, recoverability, final-inference, and factor-audit paths.

| Check | Clean Walk1 | Injected Walk1 |
| --- | ---: | ---: |
| Forward alarms | 0 | 1 |
| Backward alarms | 0 | 1 |
| Final segments | 0 | 1 |
| Candidate observations | 0 | 30 |
| Healthy candidate observations | 0 | 0 |

The injected production segment is link `27956:20276`, interval
`[1664959678.3077347, 1664959686.289086]`, with the same 30 obs IDs, segment
boundaries, and segment ID as the admitted shadow. All semantic equivalence
checks passed. The forward first alarm is affected observation 15 (one-based),
with locked latency `3.6047780513763428 s`; controlled precision and recall are
both 1.0.

## Stage2 and primary recovery

Stage2 returned `CONVERGED`, `ALL_JOINT_STOP_CONDITIONS_SATISFIED`, after 42
iterations. The one target segment has:

- 30 observations on tag/anchor `27956:20276`;
- `c_hat = 0.47134578518036691 m`;
- objective gradient and KKT violation
  `4.3673122412192361e-14`;
- `boundary = false`.

For preregistered primary `lcb_fixed_full`, one of one segment was accepted and
none suppressed:

- `sigma_c_local = 0.075073707107864776 m`;
- `delta_c_fixed = 0.47134578518036691 m`;
- `use = true`;
- reason `USE_POSITIVE_LCB_FIXED_OFFSET`.

The exported invariant `0 <= delta_c_fixed <= c_hat_stage2` passed. The known
`+0.5 m` amplitude and paired clean ranges were unavailable to detector,
Stage2, scoring, gate, compensation, and final optimizer.

## Range-level effectiveness

The post-seal evaluator paired all 30 affected observations by exact `obs_id`.

| Metric | Raw injected | Recovered | Improvement |
| --- | ---: | ---: | ---: |
| Mean error | 0.5000000000 m | 0.0286542148 m | 0.4713457852 m |
| Median error | 0.5000000000 m | 0.0286542148 m | 0.4713457852 m |
| MAE | 0.5000000000 m | 0.0286542148 m | 0.4713457852 m (94.269157%) |
| RMSE | 0.5000000000 m | 0.0286542148 m | 0.4713457852 m (94.269157%) |
| P95 absolute error | 0.5000000000 m | 0.0286542148 m | 0.4713457852 m |

Mean/median/min/max compensation are all `0.47134578518036691 m`. The
evaluator-only amplitude diagnostic is `c_hat - 0.5 = -0.028654214819633095 m`;
it was not a tuning input. All deltas were nonnegative and every recovered range
was no greater than its injected range. The strict range RMSE gate passed.

## Final factor audit

All 1,074 planned observations were accounted for by each policy. Every audit
row has `ok=1`:

| Policy | Noncandidate raw factor | Candidate handling | Result |
| --- | ---: | ---: | --- |
| `suppress_all` | 1,044 x exactly one | 30 suppressed / zero final factor | PASS |
| `structured_debias` | 1,044 x exactly one | 30 x exactly one raw-with-live-C factor | PASS |
| `lcb_partial` | 1,044 x exactly one | 30 x exactly one fixed-offset factor | PASS |
| `lcb_fixed_full` | 1,044 x exactly one | 30 x exactly one fixed-offset factor | PASS |

There were no duplicate candidate factors, corrected pseudo-ranges, lost
noncandidates, reference conversions, or segment-mapping errors.

## Controlled localization accuracy

The repository evaluator used SE(3) alignment with scale fixed to one. All
methods used the same 229 matched GT samples over
`[1664959676.9893188, 1664959736.082964]`.

| Method | Support | Recovery | ATE RMSE (m) | ATE p95 (m) | Horizontal RMSE (m) | Vertical RMSE (m) | RPE trans. (m) | RPE rot. (rad) | Status |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `robust_cauchy` | N/A | robust only | 0.1969043591 | 0.3563120422 | 0.1437141590 | 0.1346015123 | 0.6961699054 | 0.3801266704 | AVAILABLE |
| `suppress_all` | same 30 obs | reject | 0.1816368661 | 0.3464682650 | 0.1291073641 | 0.1277624344 | 0.7015738809 | 0.3802035508 | AVAILABLE |
| `structured_debias` | same 30 obs | live C | 0.1631494889 | 0.2902918312 | 0.1193051525 | 0.1112835852 | 0.6994374798 | 0.3803253964 | AVAILABLE |
| `lcb_partial` | same 30 obs | partial fixed | 0.1700862308 | 0.2920034090 | 0.1239817297 | 0.1164382094 | 0.6997984119 | 0.3801913731 | AVAILABLE |
| `lcb_fixed_full` | same 30 obs | primary fixed | 0.1631494891 | 0.2902918337 | 0.1193051528 | 0.1112835851 | 0.6994374798 | 0.3803253964 | AVAILABLE |

For primary recovery versus suppression:

- aligned ATE RMSE improves by `0.018487376951714013 m`
  (`10.17820740394057%`);
- ATE p95 improves by `0.05617643125209176 m`
  (`16.214019271634847%`);
- horizontal RMSE improves by `0.009802211334933739 m`;
- vertical RMSE improves by `0.016478849281547928 m`;
- translation RPE improves by `0.0021364011477048583 m`;
- rotation RPE changes by `-0.0001218455412406172 rad` under the
  suppress-minus-recover convention.

The primary RMSE direction and p95 non-worsening gates both pass. Secondary ATE
mean/p50/p99/max for `lcb_fixed_full` are respectively
`0.1482238431/0.1303859539/0.3570016750/0.3858437577 m`; raw-frame ATE remains
`UNAVAILABLE_FRAME_OR_POINT_PROVENANCE` rather than being inferred.

## Clean no-op and injection penalty

Clean production support is empty and Stage2 returns `SUCCESS_EMPTY`. All four
recovery policies have zero candidate factors and zero compensations. Their
trajectories are byte-identical; clean `suppress_all` and `lcb_fixed_full` share
SHA-256 `ebe85bd6c591493d8ebac1ba08f79b001d869c96ec7956763f4f5c658fda2fb9`.

Clean robust ATE is `0.16384723094111373 m`; clean production-policy ATE is
`0.1638532681570079 m`. Relative to the corresponding clean trajectory, the
injected robust penalty is `0.03305712813623615 m`, the suppress penalty is
`0.01778359792371753 m`, and the recovery penalty is
`-0.00070377902799648 m`. These are explanatory diagnostics, not the primary
endpoint.

## Truth blindness and determinism

The final injected primary production process was traced. Successful forbidden
truth/GT/oracle **data** opens before scientific artifact sealing: 0. Three
filename hits were repository source files opened by the scheduler while hashing
producer identity and are separately recorded. The detector status also reports
`gt_or_oracle_read=false`. GT and clean-pair reads occur only in post-seal
evaluators.

Two complete controlled runs finished 12/12 cells. Ninety-two canonical
scientific comparisons passed, including byte-exact support, Stage2 values,
fixed compensation, factor metadata, masks, trajectories, and canonical exact
decision/evaluation fields. Runtime timing/RSS and run/cache/inference IDs were
explicitly excluded as metadata.

## Engineering tests

- Package build: PASS (one package succeeded; existing warnings retained).
- Direct affected GTests: 100/100 PASS.
- Full CTest: 34/34 PASS.
- Direct paper-runner Python contract tests: PASS.
- `pytest` wrapper: `NOT_AVAILABLE` because the environment has no pytest
  module; the same test files passed through direct execution/CTest.
- An initial stale test executable/updated-library ABI mismatch was discarded as
  an invalid test invocation; test targets were rebuilt before the authoritative
  100/100 and 34/34 runs.

## Six-input non-gating diagnostic

All 36 manifest cells reached a recorded terminal state; the batch status is
`COMPLETE_WITH_RUN_FAILURES`. This result does not alter the controlled gate.

| Run unit | Candidates / segments | Stage2 | Primary accepted | Robust ATE | Suppress ATE | Recovery ATE | Recovery minus suppress direction |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- |
| `sfuise_walk1` | 0 / 0 | SUCCESS_EMPTY | 0 | 0.163847231 | 0.163853268 | 0.163853268 | tied |
| `sfuise_walk2` | unavailable | MAX_REFIT_ITERATIONS | unavailable | unavailable | unavailable | unavailable | unavailable |
| `sfuise_walk3` | 89 / 2 | CONVERGED | 2 | 0.175866603 | 0.168536791 | 0.171750848 | worse by 0.003214057 m |
| `miluv_random` | unavailable | producer failed | unavailable | unavailable | unavailable | unavailable | unavailable |
| `miluv_circular` | unavailable | producer failed | unavailable | unavailable | unavailable | unavailable | unavailable |
| `vicon_test` | unavailable | producer failed | unavailable | unavailable | unavailable | unavailable | unavailable |

Counts are 0 better, 1 worse, 1 tied, and 4 unavailable. Mean and median
`ATE_suppress - ATE_recover` over the two available run units are both
`-0.001607028528655366 m`. This is negative generalization evidence and prevents
any claim that recovery generally beats rejection; no parameter was changed or
rerun in response.

## Interpretation and next action

The three controlled layers pass independently: persistent NLOS is found with
the admitted support; fixed recovery reduces paired range error; and using that
recovered range improves paired Walk1 localization relative to rejecting the
same observations. Therefore the exact next action is:

`FREEZE_METHOD_AND_RUN_FINAL_MULTIDATASET_EXPERIMENTS`
