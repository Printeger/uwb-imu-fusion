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

## W1/W2 foundation — common implementation, dynamic range operator and event

`UnifiedRollingIntegrityCore` now owns the former Sc6Service implementation; `Sc6Service` is a source-compatible alias, not another algorithm. The shared envelope header has no provider/truth/draw dependency. Simulation runners include their provider explicitly. Input guards and legacy probability law remain intact pending full time/evidence integration; CSV/ROS still need routing to this core, so RI02 is not awarded.

The existing raw range contractor now consumes variable-length shells and enumerates KEEP plus each distinct physical anchor. Every observation of an excluded anchor is omitted together, including repeated observations with different times. Legacy8-array callers delegate to this same kernel. Unprocessed branches retain their entire input frontier. Terminal support validation uses actual row count. Asynchronous transport still must be supplied by a justified shell producer; this kernel change alone does not authorize retimestamping.

Added bounded `AnchorRegistry`: sparse uint64 identity, immutable calibration revisions, explicit disconnect/reconnect, separate registered/visible/measurement counts, validity and duplicate-ID checks. Resource limits are runtime parameters. This is an input component, not yet end-to-end topology qualification.

Added V2 `SourceLayoutEvent`, separate from untouched V1. For distinct newly born random coordinates in block k, allocate delta0=epsilon/2, deltak=epsilon/[2k(k+1)]. Choose integer b with 2*d*2^-b≤directed-lower(delta), and integer q with 5q²≥7b. Gaussian marginal Chernoff tails give 2*d*exp(-q²/2)≤2*d*2^-b because ln2<0.7. Sum over coordinates/blocks requires no independence. Reused coordinates retain their original birth/q; empty/retry blocks spend nothing. The implementation stages allocations before mutating the ledger, checks representation/capacity, and never resets the task budget. Its current directory is O(total admitted coordinates), with an explicit capacity; it is not an unlimited-memory claim. Marginal tail evidence and predetermined/potential-source layout are separate prerequisites; empirical/unknown selection cannot become statistically qualified. This accountant is not yet integrated into the service or a new source-law certificate.

Validation: `python3 tools/sc9_realworld.py check` PASS. Dynamic tests N=0/1/2/3/4/5/6/8/12/32/65 cover sparse64 including UINT64_MAX, distinct counts, invalid/duplicate sources, disconnect/reconnect, calibration conflicts, repeated-source faults, analytic ±z mirrors and budget0/7/256 frontier containment. V2 tests cover10000 varying-dimension blocks with an independent long-double erfc tail oracle, telescoping risk, startup/retry reuse, zero-new, capacity/overflow and selection qualification. V1-equivalent layouts retain their old q/tail.

Legacy comparison:222/222 frozen rows are exactly equal in all registered state/action/source/risk/boundary/support/HV fields. Changed ELF `74b87c40bf20c1df75575ba424488ed0ca07c32ca331d579571dd2b6fe27c696`, unchanged loaded private DSO. `sc7_support_checks` passes40 actual native epochs, strict same-center H/V improvements40/40, seal/tamper/deadline-independent single-commit checks. These are offline functional checks, not performance evidence; the legacy comparison overlapped a test-only compilation.

Development failures retained in `.cache/sc9`: first target build needed CMake regeneration; first extraction build exposed missing provider-specific includes, then fixed; a direct support-test invocation loaded the stale public devel DSO and failed symbol resolution. Re-running with the same private DSO environment as the registered runner passes. New check CLI always uses that environment.

Next: actual-duration physical/RW/dynamics operators and independent containment checks; then wire V2 evidence through rolling boundary/joint/fault/recovery and adapter inputs. W1 and W2 are not complete; Walk and ROS validation have not started. No source guard was bypassed.
