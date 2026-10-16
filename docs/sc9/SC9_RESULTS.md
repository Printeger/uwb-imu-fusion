# SC9 results

Status: IN_PROGRESS. No SC9 system capability is awarded yet. Hardware raw risk remains UNQUALIFIED; production configuration and publication gates remain unchanged.

## W0 — actual route and frozen compatibility baseline

Entry HEAD `25a4372af550c6b20d907ddc1696100de4fd36bd`, branch `feature/realtime-uwb-imu-pl`. No applicable AGENTS.md found. User files `docs/sc8/SC8_RESULTS.html` and `.pdf` remain untouched. The supplied contract is actually at `docs/requirements/sc9/SC9_REALWORLD_INTEGRATION_REQUIREMENTS.md`, SHA256 `91d2deb7a93c40e5676bce04a906606ac188bf0dc4fd06f8498fdd5cb4e1b07e`; no substitute contract was created.

Actual simulation route: `runSc6System → Sc6Provider::certify → Sc6Service::step → RollingRecovery::prepare / RollingIntegrity::propose → IncrementalUwbImuEstimator::commitEpoch[RollingResearchV1] → RollingIntegrity::finalize`. Actual Walk route: `nominal_dataset_runner → RealtimeIntegrityPipeline`, explicitly constructing the older engine. The new common implementation will be extracted from Sc6Service, retaining the native backend and all rolling operators. CSV parsing alone does not change this route.

| Asset | Actual consumer and migration obligation |
|---|---|
| SC0 current UWB candidates / Gaussian reference | `epoch_raim.hpp`, `numerical_candidate.cpp`: dynamic matrices already exist; exclusion currently means one row, must become physical source; local roots do not prove global uniqueness |
| SC1 raw range contraction | `raw_range_contraction.*`: fixed8 shells,9 masks; source guards also encode ID/time/noise law and must stay for legacy evidence |
| SC2 IMU bridge and source scope | `sc2_imu_bridge.*`, `open_fault_pool.*`: sample-grid episodes, source support and bridge pairs must follow real parents/time |
| SC3 joint FDE / native ledger | `rolling_recovery.cpp`, estimator/ledger: cache9, actions18, replacement7; onset timestamp arithmetic and held row index require identity/time migration |
| SC4 delivery / native transactions | `sc6_system.cpp`, `sc6_delivery.hpp`: single commit, sink gate and full denominator retained; old producer uses 5/50ms |
| SC5 rolling boundary / risk | `rolling_policy.hpp`, `rolling_boundary.*`, `rolling_event.hpp`: slice11, fixed anchor arrays, birth128/21, actual marginal-key verification retained |
| SC6 service / provider / temporal | `sc6_service.*`, `sc6_provider.*`, `sc6_temporal_monitor.hpp`: provider has privileged truth/draw qualification; service guard10, replay10*window+1 and fixed RW; provider must leave core dependency |
| SC7 joint raw / terminal support | `joint_raw.cpp`, `tag_support.cpp`: integral10 at5ms, interval50ms, sample0 healthy ID1/time0; terminal mask≤7; preserve all branches/frontier/receipt seals |
| SC8 / Walk input and scoring | `apps/nominal_dataset_runner.cpp`, `tools/sc8_walk_followup.py`: reuse causal input/CAL/scoring, replace old default in explicit new mode; OFF must retain monitoring |

`raw_source_law.hpp` finite50 is a separate old model, not an open-service horizon. Backend input uses 64-bit StrongIds already; ROS Linktrack narrowing remains a separate adapter defect. Native backend permits bounded asynchronous input but current range residuals still need a justified time model. Retained marginal keys remain the authority for window transfer.

Built current `epoch_raim_runner`; actual loaded DSO resolved with the execution environment. ELF SHA256 `380887d44f372b943791494fd806d74ee1ee607946b7a9cb5a14b4dc77c580be`; DSO `16fa08239067d7bbaeec654bc5f3c444b9b164065fe5d56c873acca4400900b9` at `/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/libuwb_imu_pl.so`.

Frozen seed20261010, config/sc7.yaml, offline tight-optimized: S0/35, F0/76, F3/76, S3(G2)/35. All222/222 native commits and conditional raw bounds valid; all four runs retire x0; F0/F3 each actually replace a boundary prior once. These short prefixes have zero reported rejoin events; no rejoin claim is inferred. State, action, source/risk, boundary, support and H/V fields are frozen in `.cache/sc9/legacy-baseline.json`, alongside full traces. This is functional baseline evidence, not a pacing test.

Reproduce baseline before numerical changes: `python3 tools/sc9_realworld.py legacy-freeze` (refuses overwrite). Compare after changes: `python3 tools/sc9_realworld.py legacy-parity`. No SC8 matrix rerun.

## Remaining work and frozen scope

W1–W5 and RI02–RI24 remain open. Matrix follows contract B0–B3/R0–R2/L: dynamic input points1/2/3/4/5/6/8/12/32/65; actual5-anchor nonuniform≈82Hz and12-anchor alternate-rate raw simulations; Walk1 CAL, Walk2/3 frozen TEST; original ON/OFF plus Walk2 U/A/G/J and Walk3 J; lossless same-wire ROS1×;5000 healthy/10000 repeated-fault generalized streams. Default30/30 plus short20/20,40/40,30/10. Final safety regression only after implementation freeze. Intermediate stage results do not award system completion.

Next: remove provider dependency from common service, generalize the existing range operator to physical source branches, validate legacy exact semantics and dynamic independent containment fixtures, then integrate variable source/time evidence through the same rolling service. No source guard is being bypassed.
