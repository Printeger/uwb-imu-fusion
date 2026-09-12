# 0912 canonical +1m/10s Walk1 pipeline smoke

CANONICAL_PIPELINE_PASS，仅单案例工程验证：10 backend成功、无fallback；独立clean/corrupted前端；
TP36/FP0/FN0，c_hat=1.058388062m；同229 GT样本，M4−M3 ATE=-0.014782224m，
窗口39样本差=-0.066876872m。36条恢复观测注入分量RMSE 1→0.058388062m。
几何range假设误差反增，不能以注入分量代替真实range truth；不升级C1–C3/held-out结论。
[完整结果、负结果、协议和证据](../experiments/CONTROLLED_INJECTION.md)。无调参/sweep/核心改动。

# 0912 Walk1 identity/common-clock evaluator diagnostic

用户授权单位外参与同一时基假设，evaluator exit0，每方法4266有效range样本，
RMSE=1.696637133m；四方法无动态补偿，raw/corrected 相同。
仅 ASSUMED_GEOMETRY_AND_CLOCK 条件诊断，非独立校准的range truth，不升级C1–C3。
[协议、机器产物和限制](../experiments/RANGE_IDENTITY_ASSUMPTION.md)。无estimator重跑/调参。

# UWB-IMU-IE claim–evidence ledger

## 0912 per-range evaluator and physical isolation

[Range evaluator evidence](../experiments/RANGE_EVALUATOR.md) supports geometry/lever/gap/
compensation engineering fixtures, 15 range/isolation tests and 5 harness tests. Actual bwrap
backend smoke reproduces the previous four-method trajectory metrics exactly; GT and derived
range CSV paths are not mounted. Official SFUISE constants establish evaluator-only
beta=-toa_offset and ID mapping, not an independently validated full GT geometry chain.
19,400 observation rows and 80 summaries are exported with zero available geometric range
errors because GT-to-anchor, marker-to-IMU and clock calibration remain missing. This is
UNAVAILABLE_CALIBRATION, not a real-range accuracy result. No core estimator/CUSUM/recovery
changes, no oracle labels or GT feedback, and no C1–C3 / T10=C2-C / T11=C upgrades.

## 0912 ICRA Walk1 harness

[Harness result](../experiments/HARNESS_RESULT.md): M0/M1/M3/M4 use the existing scheduler,
runner and evaluator; the final clean smoke has the same 229 matched GT samples, all four
methods successful, and no fallback. M3/M4 are empty-support no-ops. Five engineering tests
pass. Prior preflight and cache-namespace wiring failures remain recorded. These are tracker-point
development proxy metrics with unresolved extrinsics, not formal benchmark or recovery-benefit
claims. No estimator/CUSUM/recovery mathematics or parameters changed; T10=C2-C, T11=C and
C1–C3 limitations remain. The previously paused five-family data validation was later resumed and
completed under the separately bounded infrastructure claim below.

## 0912 ICRA dataset infrastructure

[Dataset contract and inventory](../experiments/DATASET_MANIFEST.md) v2 records 39 actual local
recordings: HUEC 8, MILUV 3, own_vicon 3, SFUISE 3, and starloc 22. The read-only content artifact,
manifest and all constituent files are SHA-bound; schema/semantics/hash validation has 0 errors and
15 engineering rejection tests pass. Absolute ToA/TWR/range only; TDoA and range differences are
rejected. [Validation result](../experiments/DATA_VALIDATION.md) preserves one nonfinite HUEC auxiliary
RSSI and two unindexed own_vicon source bags whose content was scanned only through temporary
reindexed copies. Six known development recordings are DEV; 33 unknown-exposure recordings remain
UNASSIGNED, with no VAL/TEST. Missing calibration, GT extrinsics, clock and data permission provenance
remain explicit; all 39 entries are NOT_ADMITTED. This supports only five-family manifest infrastructure
validation, not dataset scientific admission, an estimator/accuracy claim, or a C1–C3 upgrade;
T10=C2-C and T11=C remain.

## 0912 repository truth audit index

[`01_REPO_TRUTH.md`](../doc/paper_writing/01_REPO_TRUTH.md) anchors the actual production
paper path to HEAD `439282c2790ab1414ca9b276dd61a18b5e1643a8`: PL conditional-z,
bidirectional CUSUM support, Stage2 and explicit primary `lcb_fixed_full`.
This is a source/configuration/test-source/report audit, with no new experiment or test execution.
It records discrepancies with the supplied v4 blueprint and bounds fallback and uncertainty claims.
The [existing production E2E report](../doc/ie_0911/PL_CUSUM_E2E_INTEGRATION_ACCURACY_RESULT.md)
supports its controlled Walk1 development case, while its six-input diagnostic reports
0 better / 1 worse / 1 tied / 4 unavailable. Older sections below are historical snapshots,
including the failed forward-only admission result; they do not describe the later integrated
bidirectional path. No new scientific claim upgrade: T10=C2-C, T11=C and C1–C3 limits remain.

状态：`T10_FROZEN_C2_C / T11_FROZEN_T11_C_LIMITED_SCOPE_U13_INCOMPLETE / 0911_ORACLE_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX`

阅读范围：下方按 T00–T10、Axx/Rxx 命名的阶段证据节均为**历史快照**。其中“当前状态”“本轮”、`IN_PROGRESS`、`NOT_RUN` 和“下一步”仅描述当时阶段；任务状态及后续安排已由本文当前裁决、汇总表和准入检查取代，不构成新的执行计划。

## 0912 PL persistent CUSUM admission evidence boundary

[冻结协议](../doc/ie_0911/PL_PERSISTENT_CUSUM_PREFLIGHT_PROTOCOL.md)先于统计量封存；
[完整结果](../doc/ie_0911/PL_PERSISTENT_CUSUM_PREFLIGHT.md)只支持 source-neutral per-link one-sided CUSUM
实现和 bounded frozen replay。clean temporal calibration 与 held-out validation 由 1 s guard 分离，
`kappa=0.5`、`G_cal_max=6.023469`、`h=7.023469`；held-out clean 0 alarm/0 segment。injected target 在
affected #15 报警，30/30 recall、healthy 0 alarm/0 segment，但 last-zero excursion 额外覆盖 37 个
unaffected target rows，precision 仅 0.447761，未达到预登记 0.80。因此唯一裁决
`CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`，不满足 production admission。

该结果不支持 dynamic production-commit detectability、observer non-interference、recovery 或 localization
收益；F4 后 dynamic、Stage2/Rc/final/ATE/RMSE 均按协议 NOT_RUN。无 parameter rescue，Processes A--D
truth/GT/oracle successful forbidden opens 为0。C1--C3、T10=C2-C、T11=C均不升级。

## 当前 0911 oracle-support backend claim 边界

[stationarity repair 完整结果](../doc/ie_0911/STAGE2_STATIONARITY_REPAIR_RESULT.md) 取代此前
[未修复负结果](../doc/ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md) 作为当前 backend 状态。实际 HEAD
`3db4f317` 的封存诊断确认 `x186[4]` 的 `0.0750243859` 梯度是真实 objective derivative；原
50/100/200 outer 在 C update 前后都不变，根因是 fixed-C conditional LM 的 generic numerical stop，
不是 C coupling 或 gradient bug。唯一 D2 修复复用既有 V2 和相同 graph/Values checkpoint recovery，
未改 objective、model、budget、任何 threshold 或 FDE。

同一 Walk1 normal +0.5m、30-ID oracle 场景中，estimator 仍无 amplitude/GT 输入；Stage2 在 outer42
满足原 joint contract，得到 `c_hat=0.471346m`，Rc rank1、`sigma_c=0.075074m`，LCB correction
`0.321198m`。suppress/structured/full/LCB final 均有效且无 fallback。冻结 evaluator 的 LCB RMSE
`0.170086m` 比 suppress `0.181637m` 低 `0.011551m`（`6.3592%`），故当前限定裁决为
`RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX`，并单独记录
`LOCALIZATION_BENEFIT_OBSERVED`。这只支持单一锁定 oracle-support development sanity check；不支持
detector E2E、泛化收益、formal held-out/RQ 或 calibrated uncertainty。T10=C2-C、T11=C、C1–C3不升级。

## 当前 0911 FDE forensic / grouped V3 claim 边界

[完整取证与固定验收](../doc/ie_0911/FDE_FORENSIC_AND_FIX_RESULT.md)：111 raw / 30 planned 的注入、physical measurement、prefit sign与identity核验通过。原30条truth-window aggregate显著，但一次共同hold-out求解达到原迭代上限，masking与功效结论保持INCONCLUSIVE。按预先封存裁决仅实现opt-in C grouped provider，默认V2不变；完整CTest31/31及双向cache拒绝通过。

Walk1 clean/injected各六方法和共享producer实际完成，均零segment。clean aligned RMSE约0.164m；injected all-range及四recovery约0.199m、Cauchy约0.197m。冻结最大连续组未检出新增偏置，真阳性0/30；扩大矩阵NOT_RUN_WALK1_GATE_FAILED。只支持本输入plumbing与受测数学/工程正确性，不支持检测召回改善、独立masking确认、calibrated integrity或LCB恢复收益。T10=C2-C、T11=C、C1–C3不升级；保留全部历史负结果。

## 当前 0911 Recovery / post-fit FDE v2 claim 边界

定位门通过：fresh legacy（GT读取禁用）/paper all-range/Cauchy full Walk1锁定aligned RMSE为0.169642149/0.163853268/0.163847231m，均229匹配、0未匹配。paper raw Gaussian LM共同reference及paper-only标准robust loss修复有受控运行与梯度测试证据。FDE provider更新为`imu_aided_postfit_fde_v2`，完整Gaussian白化与一次稀疏QR的残差方差归一化已验证；旧provider结果不复用。

最终CTest29/29；固定[8,11]s smoke与full Walk1五方法均完成，55/1074条planned全检、零fault/零segment。实际SUCCESS_EMPTY Stage2及四final零优化，graph/Values一致，轨迹、bias、residual和可计算协方差一致。full四final RMSE均0.163853268m，仅支持真实空集合工程链；没有非空补偿证据，不支持恢复收益、策略等价或calibrated integrity claim。六输入30项prepare通过，其它输入精度矩阵NOT_RUN。旧六输入失败证据保留，不以本轮Walk1覆盖。T10=C2-C、T11=C、C1–C3不升级。见[本轮结果与完整命令](../doc/ie_0911/RECOVERY_FDE_V2_RESULT.md)。

## 历史 0911 residual-FDE v1 claim 边界

论文主 Stage1 已改为 `imu_aided_residual_fde_v1`：reference graph residual 按 factor sigma 做固定
$p=0.99$/1-DoF 双边检测，只聚合 `r<0` 的 positive-excess fault；legacy `automatic_discovery` 未删除。
构建、29/29 CTest 和 Walk1 `[8,11]s` smoke 通过。smoke 为合法 zero-candidate，Stage2 cache 正常发布，
四个 candidate-dependent final 复用同一 cache；FDE artifacts 完整且无 discovery/ADMM/outer trace。

六输入冻结矩阵的结果为负/不完整：3 个 SFUISE FDE reference 成功但均为 zero-candidate，随后 unchanged
raw Stage2 stop test 失败；MiLUV random/circular 与 Vicon 在 preliminary LM 达到最大迭代。6/6 producer
均未发布 cache，24 个 candidate-dependent final 因而为 `PARENT_CACHE_UNAVAILABLE`。独立 Cauchy 4/6
产生轨迹，另外 2/6 final LM 失败。锁定 GT hash 已由 evaluator 核验，但 FDE candidate-dependent 方法没有
可配对轨迹，RMSE/P95/horizontal/vertical、coverage 和 LCB 相对 suppress 改变量均 `UNAVAILABLE`。
该轮只支持 FDE 工程实现、fail-closed/cache 隔离与完整失败记账；不支持精度收益、完整 ARAIM、certified
integrity、正式 held-out/RQ/U14 或 C1–C3 升级。T10=C2-C、T11=C 保持不变。见
[`FDE_STAGE1_RESULT.md`](../doc/ie_0911/FDE_STAGE1_RESULT.md)。

## 当前 0911 STEP2 claim 边界

第二个且最后一个 development 工作包已执行六条 full 输入、两个真实 anchor 低冗余条件及独立 GT 评价。
六个主 producer 和两个低冗余 producer 全部在 Stage1 失败，未生成任何可供 suppress/structured/LCB/full
使用的 Stage2 cache；因此 LCB 相对 suppress/structured 的轨迹收益、低冗余恢复收益和预指定 MILUV
circular 全额补偿消融差异全部为 `UNAVAILABLE`，不支持正向、负向或等价 claim。严格的“至少四个主
producer 同为 MAX_OUTER”规则没有触发（2/6），但候选方法精度矩阵事实上被 Stage1 完全阻塞。

独立 robust Cauchy 仅 5/8 条件生成轨迹；其 full-record conditional SE(3)-aligned ATE 为数百至上千米，
表明这些成功输出也发生严重漂移。固定 frame/刚体点和独立 range-reference provenance 未闭合，raw-frame
与真实测距参考均不报告。该结果只支持可追溯的负/缺失证据与失败记账，不升级 C1、C2 或 C3，不改变
T10=C2-C、T11=C，也不构成 formal held-out/RQ/U14 验收。见
[`STEP2_RESULTS.md`](../doc/ie_0911/STEP2_RESULTS.md) 和隔离的 38 行机器表。


## 当前 T11 claim 边界

限定预登记工作包已执行并冻结，唯一裁决 **T11-C / FROZEN NEGATIVE/INCOMPLETE RESULT**：六个 fresh 科学前缀均已尝试，4/6完成全链；20101 H=0/H=1在Stage1失败且未重试。20102完整U13精确通过；20101仅已到达失败阶段一致，下游未到达，完整U13为NOT_RUN_INCOMPLETE_CHAIN，因此整体U13未通过。已完成比较未显示未来数据泄漏，不能把未完成链视为完整通过。

两条fresh H2的固定模型六行数值/行语义/SVD核验通过：**SUPPORTED（限定 mechanistic/diagnostic 域）**，提供有效且机制一致的信息证据；不能覆盖U13准入或作为端到端收益证据。正式E2E RQ4验证未成功完成。预登记truth gate未满足，truth未读，历史bias/轨迹评价为 **NOT RUN BY PROTOCOL / NOT_RUN_U13_NOT_PASSED**，不填论文数字。不计划T11重试或rescue。

限定工程证据包括物理前缀、原始ID、初始化/积分边界、compact数值一致和一条完整U13；这些是system/implementation证据，与上述mechanistic/diagnostic证据及未获支持的E2E empirical claim分开记账。C1/C2/C3不整体升级，T10 C2-C不变。T12为 **NOT_STARTED / NOT_RUN**，是下一科学执行任务：既有输入语义核实与限定评估。[T11完整收口](../doc/ie_sprint/T11_CLOSEOUT.md)。

## 当前 T10 claim 裁决（覆盖下方历史状态）

**T10已按用户收缩范围执行并冻结，C2-C：recoverability characterization / exploratory structured correction。**
Rc/η/s machinery 在已测试域内实现并验证；改变 noise/count 后 N=6400I/1600I/3200I，
跨条件原固定映射不再成立，但六输入 s_fit/full_gate 实际决定相同。
Structured recovery 的历史 RMSE 为1好5差、P95为0好6差；稳定轨迹收益和 η 增量操作收益均 **NOT SUPPORTED**。
不得再以“safe recovery policy”或“η 是必需操作门控”作为已验证主贡献；保留数学诊断及探索性代码。

- **SUPPORTED（限定工程域）：** Rc/η/s计算、signed-zero metadata修复、真实零候选生产接口。
- **PARTIALLY SUPPORTED：** 有限合成全链；LOS20102独立all-range对照通过，20101对照数值失败且相对代价UNAVAILABLE。
- **NOT SUPPORTED：** 稳定恢复轨迹收益、η门控增量、安全操作恢复策略、两条LOS均无退化，以及由实现存在推导的性能主张。
- **NOT YET TESTED：** T12新真实/公共数据及原完整正式held-out RQ3；完整C1/C3验证要求尚未满足，已有有限工程与诊断证据保留。T11已执行限定工作包，状态以上方T11-C边界为准。

重启损坏的新run原始冻结/评价sidecar不能回溯为完整盲测证据；现存最终产物经重建清单后独立复算。
用户授权精简34.15GiB可再生成中间证据，旧完整archive状态不再作为当前可用性承诺。
[最终收口和全部数值来源](../doc/ie_sprint/T10_CLOSEOUT.md)。本次完成仅指用户授权收缩范围，非原完整T10实验验收。


## T10-A19-R05 independent review boundary（历史快照，状态已取代）

Read-only review cross-checked per-block counts, recorded terminal thresholds and current mapped binary hashes.
The outer24 joint gradient is 6.3933990901e-7 and all four recorded conditions pass after one outer15 handoff.
This is a bounded execution milestone, not an independently recomputed final-state or benefit result: final Values
are incomplete (246 B rows/41 keys only, no X/V/C), so the terminal graph/Values cannot be independently audited here.
Twenty-three handoff audits are invalid JSON and disagree with their valid block status on inner convergence.
The engineering handshake bypassed the failing production Stage2 exporter. See the
[independent review and R06 task](../doc/ie_sprint/T10_A19_R05_REVIEW_NEXT.md).
The disclosed premature truth read remains recorded; another ticket on the same input cannot restore blinded
independence. Future comparisons on this input must be labeled unblinded development. Scores/finals/benefit remain
NOT_RUN and C1–C3 do not change. No new optimization or evaluation-truth access occurred during this review.

## T10-A19-R05 evidence boundary

R05 supports the bounded engineering fact that the precise R04 development schema now has one shared request factory
and validator, is accepted by the actual Stage1 producer, remains nonconsumable/default-off, and passes a direct small
C++/GTSAM producer→partition→Stage2→score→final→independent-evaluator handshake. Complete raw prepare constructed and
validated the exact production request after 371-factor/123-value initialization with zero optimizer calls.

The sole fresh development run adds a narrower execution fact: Stage1 converged at outer90 and Stage2 at outer24 with
the unchanged four-way AND, crossing the historical outer15 and producing 2 candidate segments/32 observations. It
failed before `ScoreRefitRecoverability` when an old X/V/B-only serializer encountered the first live scalar C in Stage2
Values. Scores, eligibility, decisions, all five finals, fallback, trajectory/bias metrics and benefit are
`NOT_RUN/UNAVAILABLE`; no candidate was actually classified Use or Suppress. A disclosed premature agent read of the
evaluation manifest/partial bias-truth table independently bars benefit claims from this run, while estimator strace
records zero truth opens. This evidence does not lock a gate, enter validation, or upgrade C1--C3. See the
[R05 record](../doc/ie_sprint/evidence/t10_a19_r05_identity_score_compare_20260910T083223Z/VERIFICATION.md).

## T10-A19-R04 指挥复审边界

只读复核封存GDB栈、Eigen配置、prepare状态及Stage1拒绝源码，支持R04的限定工程修复记录；没有重跑。
Stage1输出INVALID_INPUT且0 outer/call/trial，原因是runner R04 schema不在producer白名单，prepare在
请求构造前返回而未发现。该观察不支持或否定handoff数值有效性、eta或修正收益。
[复审与最小下一任务](../doc/ie_sprint/T10_A19_R04_REVIEW_NEXT.md)。完整输入Stage2、score、五策略和
科学评价仍NOT_RUN，C1–C3保持原状态。

## T10-A19-R04 evidence boundary

R04 adds bounded software evidence only. The real original-binary stack and
ABI probes support an Eigen alignment mismatch between the manually built R03
runner and CMake-built core. A CMake-matched runner completed the formerly
crashing full raw prepare/content-identity path. A real C++ mixed final and a
separate evaluator process also proved the local score/Use-Suppress/final
artifact consumption path. These are development engineering results.

The sole fresh scientific ticket failed at the Stage1 input-identity guard:
the R04 schema was absent from the producer allowlist. It made 0 outer and 0
conditional calls, so Stage2, scores, five scientific finals, evaluation truth,
ATE, bias risk, coverage and benefit remain `NOT_RUN/UNAVAILABLE`. It does not
show whether the R03 handoff improves or harms a trajectory, and it adds no
support to C1--C3. See the
[R04 record](../doc/ie_sprint/evidence/t10_a19_r04_crash_score_compare_20260910T072815Z/VERIFICATION.md).

## T10-A19-R03 failure evidence boundary

R03 implemented the default-off, nonconsumable Stage2 inexact-handoff guard and the explicit five-policy development
adapter.  The archived 12-trial guard, negative cases, real mixed graph `c_s` update/four-way joint audit, final artifact
path, evaluator fixture, and 8/8 startup checks passed.  This supports only the bounded engineering behavior.

The fresh scientific run did not yield evidence for any claim.  After one confirmed pre-initialization launcher defect was
fixed, the fresh attempt opened the frozen config and complete raw input but terminated by SIGSEGV before any Stage1 trace.
The execution stage was not provably pre-algorithm, so no retry was made.  Stage2, real score, eta/s/gamma, policy decisions,
five finals, truth evaluation, ATE and correction risk are `NOT_RUN/UNKNOWN`.  It did not cross outer15 or establish the
joint four-way AND.  C1--C3 remain unchanged.  See the
[R03 record](../doc/ie_sprint/evidence/t10_a19_r03_inexact_handoff_20260910T061142Z/VERIFICATION.md).

论文工作稿：[`main.tex`](main.tex)

方法合同：[`../doc/ie_sprint/METHOD_CONTRACT.md`](../doc/ie_sprint/METHOD_CONTRACT.md)

实验合同：[`../doc/ie_sprint/EXPERIMENT_CONTRACT.md`](../doc/ie_sprint/EXPERIMENT_CONTRACT.md)

状态词：`PLANNED`、`PARTIAL_LEGACY_ONLY`、`PARTIAL_T02_FOUNDATION`、`PARTIAL_T03_GOLDEN_ONLY`、`PARTIAL_T04_ORACLE_DEBUG_ONLY`、`PARTIAL_T05_ORACLE_DEBUG_VALIDATED_DOMAIN`、`PARTIAL_T06_AUTOMATIC_DEVELOPMENT_ONLY`、`PARTIAL_T07_INPUT_ENGINEERING_LOCAL_ONLY`、`PARTIAL_T08_FINAL_INFERENCE_DEVELOPMENT_ONLY`、`PARTIAL_T09_ENGINEERING_ONLY`、`BLOCKED_EXTERNAL`、`SUPPORTED`、`REJECTED`。
只有锁定代码与实验产物可以把 C1–C3 标成 `SUPPORTED`。T00 证据只支持旧后端构建、测试实际执行、
单条旧 baseline 运行和重跑事实，不支持 v2 NLOS 方法或正式 RQ 结果。

| Claim | Implementation/source expected | Experiment/run IDs expected | Figure/table expected | Current status and limitations |
|---|---|---|---|---|
| C1：可复现的 full-trajectory raw-range UWB–IMU pipeline，策略可替换，输出来自一致 final graph/Values | T02 paper input/fixed-beta path；T08 `InferenceResult`；T09 isolated runner/evaluator；source commit + build log | RQ1 simulation、controlled UGV、一个 public dataset；至少一次完整 rerun；run manifests/hashes | `tab:system_results`；architecture figure；artifact manifest | `PARTIALLY_SUPPORTED_ENGINEERING_ONLY`：FDE Stage1、隔离 artifacts、provider-aware cache 和 zero-candidate smoke 全链有工程证据；六输入 6/6 producer 未发布 cache，不能新增完整端到端方法或精度证据。 |
| C2：recoverability characterization / exploratory structured correction；保留 Rc/η/s 与 live-c 分段联合图 | T03 golden matrices；T04 segment factor/constrained refit；T05 `F/G/N/R,eta,s,gamma`；FDE/legacy partition；T08 final/fallback；U01–U12 | RQ2 repeated LOS + sustained 1/2/3-link/ramp；RQ3 automatic-discovery end-to-end matched-cache gate comparison；fixed-partition constant/ramp 只作 `DEBUG_DIAGNOSTIC_ONLY`；failure/fallback runs | `tab:main`；`fig:headline`；automatic E2E decision/final score CSV；另列 fixed-partition DEBUG 表 | `C2-C / NOT_SUPPORTED_OPERATIONAL_BENEFIT`：T10负裁决保持。当前 FDE 六输入无 Stage2 cache 或 suppress/structured/LCB 配对，四项 paired 指标和 coverage 变化均 unavailable；原完整 formal held-out RQ3仍未完成。 |
| C3：受控证据与 artifact 分离 portability、decision quality、multi-link NLOS 和 future-context，命令/配置可追溯 | T07 deterministic truth sidecar；T09 manifests/cache/evaluator；T10 locked gate；T11 no-leak prefix；T12 locked metrics；T13 generated figures/tables/release | RQ1–RQ4 locked run IDs；RQ3 两个隔离 cache namespace；RQ4 common-linearization 独立 manifest/cache；U13/U14；failed/zero-coverage/fallback ledger；artifact reproduction run | `fig:headline`、`fig:context`、system/main tables、machine-readable metrics | `PARTIALLY SUPPORTED（基础设施） / FROZEN NEGATIVE/INCOMPLETE RESULT`：当前轮新增 FDE status/observation/partition artifacts、provider-aware cache、六份锁定 GT 核验和 30 行全分母聚合；但 6/6 producer 无 cache、无 candidate-dependent final、无 U14/formal held-out，故不升级完整 C3。 |

## T10-A19-R03 崩溃后复审边界（历史快照，状态已取代）

指挥会话核对封存SIGSEGV、源码与当前runner/core/GTSAM/MPFR/GMP身份；原始崩溃栈未提供，根因未定。
启动预检未覆盖真实raw初始化建图正路径，五final实际共用自动流程900s timeout，既有T09 evaluator
自造输入回归不支持已消费R03真实final的连贯链路。真实3项refit和2项final/policy测试记录仍成立；另一次
0项GTest不能计作通过。见[复审及下一轮任务](../doc/ie_sprint/T10_A19_R03_REVIEW_NEXT.md)。
本轮只读审查，无新estimator/数值修复/score/final收益或truth评价。R03未证明handoff在完整输入有效，
也未否定该方案或eta；C1–C3原状态不变。

## T10-A19-R08 limited validation evidence boundary

2026-09-10独立复审核验1320项冻结文件，重算36份有效final轨迹及可用配对差值和两阶段bias指标一致。
8组N均为6400I，eta与s满足确定映射；当前同N矩阵不能证明eta的额外价值。LOS完整链工程门仍缺，
两种ramp失败与中断缺失计数限制保留，不接受gate准入或C1–C3升级。汇总CSV的accepted_bias字段来自
decision_time，不能称final bias。见 [复审](../doc/ie_sprint/T10_A19_R08_REVIEW_NEXT.md) 与
[AUDIT](../doc/ie_sprint/evidence/t10_a19_r08_independent_review_20260910/AUDIT.json)。

R08 adds bounded, split-bound synthetic validation characterization. The implementation binds validation role,
reservation, ancestry and content identity through the real producer, Stage2 request, frozen decisions and final
content identities while retaining `consumable=false`. Twelve fixed P1 cells were attempted once: eight completed
automatic discovery, joint refit, scoring and five final attempts; three retained first algorithm failures and one
retained a host interruption. All estimator traces show zero truth opens. A 1320-payload pre-evaluation freeze was
rehash-verified with zero mismatch before eight independent evaluators opened validation truth.

Across the eight scored cells, fit_only, s_fit and full_gate always made the same Use/Suppress decision. Seven cells
had valid suppress_all and structured finals: structured historical RMSE improved once and worsened six times, while
P95 worsened in all seven. No LOS cell produced a valid final, so LOS cost and its tolerance check remain unavailable.
This does not lock a gate, distinguish eta, prove correction harm/benefit in general, establish LOS safety, or upgrade
C1--C3. Test/held-out reservations were not used. See the
[R08 record](../doc/ie_sprint/evidence/t10_a19_r08_validation_compare_20260910T115109Z/VERIFICATION.md).

## T10-A19-R07 配对 development 证据边界

2026-09-10 指挥独立复审核验387项冻结payload并在核验后独立重算五策略的同时间关联ATE、
配对差值和structured final bias误差，与下述结果一致。接受限定development配对事实，
不升级C1–C3；结束单seed求解器支线。复审没有重新优化、重建完整图或独立重算333-bit证书。
见 [R07复审](../doc/ie_sprint/T10_A19_R07_REVIEW_NEXT.md) 和
[独立AUDIT](../doc/ie_sprint/evidence/t10_a19_r07_independent_review_20260910/AUDIT.json)。

R07只把既有certified P/D接受策略接到default-off development final的全部抑制/无C recovery和fallback；
legacy/default、非空C Stage2及其handoff、score/gate、阈值、预算和原四项AND不变。真实candidate-excluded
小图、受控fallback、identity错误、无C handoff拒绝及完整raw prepare工程门通过。唯一fresh P1 step从raw
完成Stage1 90 outer、Stage2 24 outer、1/1 eligible评分和五个有效final；共同参考为339 factors/123
X-V-B keys/0 C，structured_debias为371 factors/125 keys/2 C，均无fallback或corrected pseudo-range。

独立truth评价仅支持这一条`UNBLINDED_DEVELOPMENT`配对观察：suppress_all在历史`[3,6]s`的ATE RMSE/P95为
`0.0151169771/0.0210468736 m`，structured_debias为`0.0158797893/0.0229025775 m`，相对差
`+0.0007628122/+0.0018557039 m`，因此在该seed上使用两段/32条修正略差。structured的accepted bias
RMSE为`0.005767514545 m`且bad correction 0/32；这不改变轨迹配对结论，也不能推广为安全性。

fit_only、s_fit、full_gate都因max gamma超过1抑制同一组，与suppress_all同终态；本输入没有区分三个gate，
没有eta增量证据。R05 truth事件永久保留。本条不支持泛化、统计显著性、LOS无退化、正式gate lock、
validation/test准入或C1--C3升级。下一证据缺口是预登记有限validation与held-out未见数据；本seed不再调gate。
[完整记录](../doc/ie_sprint/evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/VERIFICATION.md)。

## T10-A19-R06 independent review boundary

The review verified 119 frozen payload hashes before reading evaluation truth and independently recomputed the N/R
spectrum, eta/s, and structured_debias trajectory/bias metrics. Full-record ATE RMSE is 0.024065220556 m;
[3,6]s RMSE is 0.015879789319 m; final accepted bias RMSE is 0.005767514545 m, with 0/32 bad corrections under
the frozen 0.20 m definition. This is absolute performance on one unblinded development input, not paired benefit.
All three gates suppress the one group because max gamma exceeds 1; eta and s pass. Under the decision-time
bias-error label they reject 32/32 good candidate corrections, which is not proof of trajectory harm or eta value.

All four suppressed-policy recovery/fallback numerical traces match. Their gradient stops at 1.1159186055e-6
from outer2 through outer200, and the empty-accepted/no-C/fallback code paths bypass the development certified
callback. This supports a bounded policy-coverage amendment proposal, not a claim that the new route will converge.
No new estimator run or implementation occurred during review. C1–C3 and UNBLINDED_DEVELOPMENT boundaries remain.
See [review and R07 task](../doc/ie_sprint/T10_A19_R06_REVIEW_NEXT.md) and its reproducible audit.

## T10-A19-R06 自动评分与配对参考失败边界

R06 在预登记后只修完整 X/V/B/C production export、逐位 reader 和严格 handoff 审计；真实含非零/零边界
C 的同路径回归、失败不发布、同图数值审计、直接 score→decision→final→独立 evaluator、完整 raw prepare
及 14/14 受影响测试通过。共享 development schema、solver、333-bit、handoff 和工作点均未改变。

一次明确预初始化目录失败经新 ticket 重试后，fresh P1 step seed10101 从原 raw 完成 Stage1 90 outer、
Stage2 24 outer、125-key完整终态导出及 1/1 eligible group 评分。eta为`0.58397847413582771`、s为
`0.016357299043480211 m`，两段gamma为`0.1431107780213654/1.4164987078762303`；完整F/G/N/R/E、
obs/factor masks、谱复算和linearization身份已保存。estimator truth/GT打开0。

structured_debias 使用2段/32候选并产生有效final；raw-frame全段ATE RMSE/P95为
`0.0240652206/0.0394080403 m`，`[3,6]s`为`0.0158797893/0.0229025775 m`。suppress_all与
fit_only/s_fit/full_gate均抑制且其recovery和唯一fallback都在冻结200 outer失败，所以共同参考trajectory
缺失，配对ATE差值全部UNAVAILABLE；不能声称修正优于、劣于或等于抑制。三个gate未区分此输入，零接受
risk记为UNDEFINED。

R05 truth暴露事件保留，R06评价统一为`UNBLINDED_DEVELOPMENT`。本条仅支持默认关闭开发路径在单一已见
seed上走完自动评分，并支持其失败/空接受审计；不支持泛化、统计显著性、eta增量、LOS无退化、validation
准入、正式gate lock或C1--C3升级。[完整证据](../doc/ie_sprint/evidence/t10_a19_r06_live_c_score_compare_20260910T092530Z/VERIFICATION.md)。

## T10-A19-R02 outer15 静态复审边界

2026-09-10 本指挥会话新增[定向复审](../doc/ie_sprint/evidence/t10_a19_r02_outer15_review_20260910/REVIEW.md)：
12/12封存trial的原生状态/常数/分支逐位复现、J/r/delta文件逐字节复现；Python MPFR区间重算与原C++证书
一致确认实际端点非下降。高精度理想端点及方向差分支持这些微小试步被位姿端点运算误差盖过，不能将此
扩展为全Jacobian或收敛证明。未执行新的线性求解、LM iterate、完整估计或truth评价。

唯一后续提案为development Stage2非空分段的受限inexact handoff，保留最终joint四项AND；
`PROPOSED_NOT_IMPLEMENTED_NOT_RUN`，未证明可收敛。Stage2成功、真实score、五策略final与修正收益仍
NOT_RUN，C1--C3状态保持不变。审计代码不是新的estimator，负D没有被接受或解释成成功。

## T10-A19-R02 自动流程失败证据边界

R02 在运行前冻结 development 协议、工作点、预算和身份。修正仅影响 estimator-free 启动 checker 的动态库
symlink 比较；第二轮 9/9 启动预检通过后，唯一 fresh P1 step seed10101 从原 raw 初始化。Stage1 在
90 outer、327 calls/468 trials后收敛，冻结2段/32候选；Stage2 完成14个 outer 后在 outer15 第3
conditional call 以 `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED` 首次算法失败（累计83 calls/170 trials）。

Stage2 未产生有效 final graph/Values，`ScoreRefitRecoverability` 未运行；group/eligible/eta/s/gamma、五种
Use/Suppress decision、final/fallback 及 evaluation-only truth 均 `NOT_RUN/UNAVAILABLE`。因此本记录不支持
“修正优于全部抑制”、三种 gate 有差异、零风险或零 eligible，也不进入 validation/held-out。无 retry、
无 threshold sweep，C1--C3 状态不变。
[完整证据](../doc/ie_sprint/evidence/t10_a19_r02_auto_score_compare_20260910T041247Z/VERIFICATION.md)。

## T10-A19 Stage2接口失败证据边界

本轮登记并执行 `REVIEW_ACCEPTED_A18_DEVELOPMENT_STAGE1_SCOPE` 后的限定A19。工程门通过，但唯一fresh P1 step只完成Stage1：90outer/327calls/468trials与四项AND通过。Stage2 outer1在零optimizer call前因共同参考raw range缺certificate metadata而明确 `UNKNOWN_FACTOR` exit2；这是A19接口覆盖缺陷，不是Stage2收敛、eligibility或评分结果。Stage2 amplitudes/boundary/groups/eligible/unavailable为NA，eta/s/gamma及score为NOT_RUN，无retry且truth/GT打开0。

[完整证据](../doc/ie_sprint/evidence/t10_a19_stage2_score_20260910T014316Z/VERIFICATION.md)。本结果没有向门控比较提供有效下游score，C1-C3状态不变；正式validation/test/cache/final/gate/T11/scheduler均NOT_RUN。唯一下一步需另行授权：补齐所有common-reference range的fixed-beta metadata并加入真实mixed-reference/candidate certified Stage2 fixture。

## T10-A19-R01 工程修复与 fresh 入口失败边界

R01 接受并修复上述 Stage2 接口 finding：所有 reference/candidate range metadata 一一覆盖，真实 mixed
CertifiedLm/native retract、非零 fixed beta/live `c_s`、同图 scoring 以及 constructor/partial 异常状态均通过
预登记工程门。小图有限 eta/s/gamma 只属于 engineering fixture，不是论文数据或政策收益证据。

唯一 fresh P1 step 运行在 Stage1 前因 launcher 未创建隔离父目录 exit2；Stage1、Stage2、scoring、
segment/group/eligible/gamma 均 `NOT_RUN`/`null`，无 retry、无 truth/GT 读取。失败后的 launcher 修正未运行，
所以 C1--C3、正式 RQ、locked metrics、validation/test/cache/final/gate 全部不升级。
[R01完整证据](../doc/ie_sprint/evidence/t10_a19_r01_stage2_integration_20260910T024847Z/VERIFICATION.md)。

## T10-A18 完整Stage1 development证据边界

A18限定交付完成待review：登记 REVIEW_ACCEPTED_A17_CERTIFIED_FIRST_BLOCK_PROTOTYPE_SCOPE，来源本指挥会话及独立复审 `/tmp/t10-a17-commander-review-j5heubp8/REVIEW.md`。默认关闭C++进程内333bit证书保留A17数值语义；真实AutomaticSupportProvider全Stage1接线覆盖实际beta+非零u，开发输出schema隔离且旧cache reader拒绝。43-pair全证书/有理P/保守fidelity通过，完整批次4.783429s；C++104项/原discovery39项通过，全部工程失败保留。
唯一fresh P1 step seed10101运行exit0、60.5083s：90outer/327calls/468trials，327接受141拒绝0unresolved；证书累计31.1697s。首block完整复现A17数值；Stage1最终原objective/step/KKT/navigation四项AND通过，chain后尺度梯度9.520208e-7。实际2段/32candidate/short0（每段16条、3–6s）；boundary/group/eligible/unavailable为NA，Stage2/score/cache/final/gate/validation/test/T11/scheduler NOT_RUN。
仅本development输入Stage1观察，非端到端/held-out证据；T10 IN_PROGRESS，A14失败/A15全部FD失败/A12负结果/A08历史15/18与C1–C3限制保留，不锁gate、不升级claim。完成归档后停止等待review。

[完整A18交付](../doc/ie_sprint/evidence/t10_a18_certified_stage1_20260910T010000Z/VERIFICATION.md)。C1–C3原状态不变。

## T10-A17 首block原型证据边界

登记指挥接受 `REVIEW_ACCEPTED_A16_FIXED_ENDPOINT_PRECISION_AUDIT_SCOPE`；本指挥会话与独立复审 `/tmp/t10-a16-commander-review-7d4bmy57/REVIEW.md` 为来源，仅接受A16限定审计。
[A17限定amendment/冻结工程门与预算](../doc/ie_sprint/T10_A17_AMENDMENT_PROTOCOL.md)先于实现/测试；默认关闭的独立C++ PAPER_CERTIFIED_PAIR_REDUCTION_V1原型复用原GTSAM solve/native retract，333bit P/D与fidelity比较/转换均保守舍入。模型/初始化/priors/原generic AND stationarity、lambda预算保持，core/GTSAM/legacy未改。
工程门通过：A16固定端点/615维方向严格一致，D证书复现，独立精确有理P包络通过；23个区间/失败/转换反例、15个原策略相关GTest、原型小图2calls计数/原驻点通过；实际旧Stage2 reader拒绝原型schema。首次Python nextafter缺失、空分支CSV工程失败及实施前lambda勘误全部保留。
唯一fresh P1 step seed10101原raw初始化pilot exit0、external89.2440s/RSS35880KiB：20calls/43solves/43trials，20accepted/23rejected/0unresolved；最终尺度梯度2.2196421412e-7，原阈值1e-6/roundoff5.0389874292e-9，lambda0.010000000000000005，满足原generic AND stationarity。
这只是首conditional block合格，不是Stage1收敛；程序立即停止，无chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler，未评candidate/segment/group/eligible/unavailable为NA。证书累计求值81.2431s，尚无全链路成本/收益证据。
[完整交付](../doc/ie_sprint/evidence/t10_a17_certified_prototype_20260910T002211Z/VERIFICATION.md)含全部P/D区间、逐factor/原始端点/实际命令/失败/库身份。原型独立策略身份与consumable=false，禁止旧缓存消费；不铺开Stage2/cache/final集成，不升级claim/准入。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。A17本地限定交付完成待review，完成后停止等待review，不自行扩展下一轮。

## T10-A16 历史固定端点精度证据边界

登记指挥接受 `REVIEW_ACCEPTED_A15_TERMINAL_NUMERIC_DIAGNOSTIC_SCOPE`，来源为本指挥会话，非外部REVIEW.md、非estimator成功或validation准入。
[A16预登记协议](../doc/ie_sprint/T10_A16_PROTOCOL.md)先于实现/计算；唯一call17 trial1恢复初始/末态graph、Values、371factor/123keys/615delta及两端1886native残差严格一致，原生retract一次后封存binary64端点。
零optimizer iterate/新estimator进程；固定50/100位参考与有向区间表明该端点对下降 `+2.59734275148281994e-13`；两档差`3.20911e-48`，独立区间半宽`9.16821e-46 / 7.22877e-96`满足预登记门，分支核对通过。
原生残差求值/白化将下降高估`4.00953e-13`，解释原白化恒等式与A15稳定GN差异的约79.26%；仍剩`D_ref-GN=+1.04901e-13`，未进一步区分端点舍入/非正交、J/r误差与GN余项。不宣称全域导数正确或solver会收敛。
[完整证据](../doc/ie_sprint/evidence/t10_a16_endpoint_precision_20260909T160252Z/VERIFICATION.md)含公式/独立误差依据、逐factor分解、原始端点、命令/失败及实际库身份；core/GTSAM/模型/生产策略/合同不改，无truth/GT读取。
唯一[solver amendment草案](../doc/ie_sprint/T10_A16_SOLVER_AMENDMENT_DRAFT.md)为paired reduction与误差证书/resolution判据，明确接受/失败/身份失效/回归；PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED，本轮不实施或追加实验。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评计数NA。
A16本地限定交付完成待审，完成本次审计后停止。

## T10-A15 历史末态数值诊断证据边界

指挥接受 `REVIEW_ACCEPTED_A14_OPT_IN_IMU_MODEL_AND_BOUNDED_PILOT_SCOPE`，来源为本指挥会话，非外部 REVIEW.md，非 estimator 成功或 validation 准入。
[A15预登记协议](../doc/ie_sprint/T10_A15_PROTOCOL.md)先于实现/捕获。默认关闭被动TRYDELTA模式无额外solve/InspectFirstLinkedLmTry/extension；只捕获首block并阻止chain。
唯一P1 step seed10101新模型捕获exit1，external1.75785s/RSS31160KiB；初始graph/Values/common严格复现A14，末态17calls/16accepted/42trials及E/五类梯度/lambda复现，仍不驻点。
全部42个trial 615维完整；静态零optimizer恢复371factor、16accepted native retract及内容身份通过。call17的19trials为18次线性差负/1次零，均未计算model fidelity。
指定首/末拒绝方向的稳定线性下降为+1.54833e-13/+5.80613e-15，而linked总目标相减为-1.36424e-12/-3.18323e-12；两正值仍低于原5.16324e-13 resolution门。
三条实际完整方向及末态最大梯度坐标在预登记连续步长判据下支持导数一致性；13356个factor残差FD中124失败全部保留，不能写全步长通过，A08历史15/18限制不消除。
结论支持线性化差值数值分辨力与linked分支限制；非线性真实下降/残差求值误差与非线性余项分离仍证据不足，不宣称改算术即可收敛。
唯一[最小后续提案](../doc/ie_sprint/T10_A15_NEXT_ACTION_PROPOSAL.md)为同call17 trial1零iterate非线性残差精度审计，本轮NOT_RUN，无solver修复或数值语义amendment实施。
[A15完整证据](../doc/ie_sprint/evidence/t10_a15_terminal_numeric_20260909T152633Z/VERIFICATION.md)保留编译失败、中止重叠构建、唯一pilot失败、完整静态结果和source/raw/config/actual runtime身份；truth/GT未读，GTSAM不变。
T10 IN_PROGRESS；A14失败、A12负结果、A08历史限制及C1–C3原状态保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评candidate/segment/group/eligible/unavailable计数NA。
A15限定诊断本地完成待审；完成后停止，不自行执行下一轮。

## T10-A14 显式模型与负结果证据边界

REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE为指挥限定接受，不是A14效果/validation准入。
A14按[预登记amendment](../doc/ie_sprint/T10_A14_AMENDMENT_PROTOCOL.md)实施显式conditional live-bias K0；default/legacy I6保持。
实际参数/重力/native convention绑定及跨模型common/support/Stage2/request/final消费拒绝已验证；原A12内容身份精确复现。
最终1680静态检查、127 C++工程fixture及Python mock-runner合同通过，只支持该输入/方向/接口的工程正确性范围。
[唯一development pilot](../doc/ie_sprint/evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md)退出1：outer1首block17calls、16accepted，lambda耗尽，梯度1.20549e-5未达原条件。
chain/Stage2/score/cache未到，候选/段/组/eligible/不可评分组数NA，不能写成零eligible成功或正式coverage证据。
不跨模型比较objective宣称性能改进；无truth/GT读取、其他scene、held-out、scheduler、gate或final sweep。
A12负结果、A08历史15/18/UNKNOWN、C1–C3原PARTIAL/BLOCKED状态保留；T10 IN_PROGRESS，A14限定交付完成后停止。

## T10-A13 历史协方差审计证据边界

以下为A13封存时状态；草案在A14获技术接受，生效实施及证据边界见上文。

指挥接受 `REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE`，仅A12限定诊断，原负结果不变。
A13零iterate恢复同P1 step call50，原graph/Values门通过；实际40PIM的biasAccOmegaInt=I6，逐sample作为额外协方差源注入，
与A10已声明模型不一致（该项及尺度来源未声明）。不将GTSAM默认或大条件数直接定性为通用bug/唯一非收敛根因。
完整1600步重放、原白化及冻结方向814检查通过；IMU曲率0.198580/总0.244713，K的完整cov方向敏感性归因99.9997925%，
不冒充可相加的独立信息。唯一[amendment草案](../doc/ie_sprint/T10_A13_AMENDMENT_DRAFT.md)为PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED，
未修改生产协方差/默认/solver；其回归、chain/Stage2/validation/test/T11全部NOT_RUN。
[A13证据](../doc/ie_sprint/evidence/t10_a13_imu_covariance_20260909T135328Z/VERIFICATION.md)保留命令/失败/库身份及truth隔离。
A12负结果、A08历史15/18/UNKNOWN及全部历史限制保留。C1–C3不升级；T10 IN_PROGRESS，完成这次定向审计后停止。

## T10-A12 诊断证据边界

登记指挥接受 `REVIEW_ACCEPTED_A11_BOUNDED_FIRST_BLOCK_DIAGNOSTIC_SCOPE`，只接受A11限定诊断，不接受estimator成功或validation准入。
A12仅封存P1 step seed10101 call50：graph/Values/371factor/五类梯度/实际方向证据复现；物理尺度J满秩、列和阻尼作用明显不均，
满足预登记单次对照前提，但不构成根因结论。A150次精确复现A11；B仅已有diagonalDamping，最终Gmax由0.073059237到0.048386890，
两臂均未达原generic AND stationarity，B未达预登记至少减半改善判据。两进程各150次、30s内；负结果保留，不推广生产策略。
本轮未读/hash truth、不生成数据；chain/Stage2/gate/held-out/scheduler NOT_RUN，无eligible/eta/s/gamma或新cache。
[A12完整证据](../doc/ie_sprint/evidence/t10_a12_checkpoint_scale_20260909T131349Z/VERIFICATION.md)。
A08历史图级15/18及UNKNOWN限制保留。T10 IN_PROGRESS，C1–C3保持原PARTIAL/BLOCKED状态，准入方案仍PROPOSED_NOT_ADMITTED。
最小后续提案仅同checkpoint最弱旋转模式的逐factor白化曲率/IMU协方差贡献审计，零iterate；本轮NOT_RUN。

## T00 可引用事实的严格边界

证据目录：
[`../doc/ie_sprint/evidence/t00_newenv_20260905T154344Z/`](../doc/ie_sprint/evidence/t00_newenv_20260905T154344Z/)
及 [`REPO_AUDIT.md`](../doc/ie_sprint/REPO_AUDIT.md)。

| 事实 | 证据 | 可支持 | 不可支持 |
|---|---|---|---|
| 当前 package build exit 0 | `04_build_and_test_after_cleanup.log` | 当前提交/环境可构建旧后端 | v2 模块存在、正确或可发布 |
| tests：38 total、2 failures | 同上 | 测试确实执行；失败语义已知 | “测试全通过”；依赖 IMU 语义的新实验可靠 |
| SFUISE Walk1 两次 legacy node exit 0 | `05_baseline_wrapper.log`、`09_baseline_rerun_wrapper.log` | 单条 legacy 输入在本机运行 | 其他 adapter、v2 paper path、controlled NLOS 已验证 |
| 两次 trajectory/GT/calibration 字节一致 | `10_rerun_comparison.log` | 该输入/二进制/配置的产物重跑一致 | 并行求解普遍 bitwise deterministic |
| aligned ATE RMSE `0.169642 m` | 两个 run 的 metrics/log 与审计 | legacy、per-run SE(3)-aligned 指标 | v2 主结果、raw-frame ATE、independent calibration benchmark |
| covariance CSV 为 `-1` 占位，legacy residual 与 final object 不一致 | source/actual output audit | 当前缺口与 T02/T08 动机 | 任何 uncertainty/covariance claim |

## Claim 准入检查

### C1

- [x] paper path 复用 loader/initializer/IMU/GraphBuilder/UWB factor 且正常退出；T02 smoke exit 0
- [x] stable observation/mask/keyframe/sigma contract；T02 fixtures、4850-row ledger，以及 original/SFUISE 真实 loader 嵌套裁剪通过
- [x] final trajectory/bias/residual/covariance 同 graph/Values；T08 已在 development engineering scope 独立接受，仍非正式 RQ 支持
- [ ] simulation + controlled + one public v2 run；RQ1 `NOT_RUN`
- [ ] measured runtime/RSS/scoring/fallback/rerun；RQ1 `NOT_RUN`
- [ ] release/archive 与许可可核验；`BLOCKED_EXTERNAL`

### C2

- [x] T03 U01–U05 NumPy golden correctness 复审；T03-R01–R04 已接受，最终独立复核默认/Sandybridge 各 22/22、跨 BLAS/字节复现/独立 Schur 均通过
- [ ] U06 fixed/online beta score-column 完整性；test-only online `Z(m)` + real prior/去 prior混淆与 fixed-beta 单次残差在 T05 测试支持域 `PASS_REVIEW_ACCEPTED`，但真实 fixed value 缺失
- [ ] U07 projector/Schur/C++ sparse 全对照；T03 Python 与 T05 C++ sparse 支持域对照均已接受，临界/不可证明秩显式不评分；仍不等于通用误差证明
- [x] U08–U12 C++ factor/score/final correctness；U08–U10 既有范围已复审，U11 `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；T08 R01/R03 保留限定 scope，R02 为 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`
- [ ] automatic discovery，不读 oracle/GT；T06限定工程证据保留；T10有限全链与T11的4/6科学前缀已完成，未完成原完整正式RQ验证，不能以实现或有限执行升级完整claim。
- [ ] RQ3 fixed-partition constant/ramp mismatch diagnostic 使用预声明分段并全程标 DEBUG；T09 仅运行 development smoke 并验证 cache namespace，正式 mismatch/RQ `NOT_RUN`，不能替代上一项
- [ ] RQ3 automatic discovery 使用独立 cache，报告全部自动段的数量/长度和 decision/final `gamma`；T10用户收缩范围六输入、30/30 final有效，s_fit/full_gate决定相同，稳定structured收益 **NOT SUPPORTED**；LOS20102独立对照成功，20101失败且相对代价UNAVAILABLE。原完整正式held-out RQ3未完成或获验收；有限比较不替代该验证程序。
- [ ] independent fixed beta calibration；`BLOCKED_EXTERNAL`
- [ ] held-out gate increment at matched coverage/cost；RQ3 `NOT_RUN`
- [ ] no unacceptable LOS degradation under prelocked tolerance；**NOT SUPPORTED**：T10 LOS20102独立对照通过，20101对照数值失败；原完整RQ2验证未完成。
- [ ] accepted `c_s` retained and fallback traced；T08 R01 已在 development engineering scope 独立接受，但仍非正式 RQ；本轮 R02 不修改该实现

若 full gate 对 `s+fit` 无增量，按冻结材料收窄/简化政策 claim；若完整 gate 也无效，将诊断降为探索性。
不能用 C1 系统事实替代失败的 C2 方法证据。

### C3

- [ ] split/locked gate/test labels 无泄漏；T10有限validation已执行并冻结C2-C，未取得原完整gate lock/held-out验收；原始冻结sidecar缺口不能回溯为完整盲测证据。T11冻结T11-C，整体U13未通过，truth gate未满足且未读取truth。
- [ ] deterministic truth sidecar 与相同缓存 gate comparison；T07 input/truth isolation 与复现已独立复审接受，T10有限same-cache比较已执行；原完整正式held-out RQ3未完成或获验收。
- [x] prefix 在初始化前物理裁剪；T11限定工程核验通过。
- [ ] 完整U13；20102完整通过，20101因Stage1失败未到达下游而不完整，整体未通过；已完成比较未显示未来数据泄漏。
- [ ] development-only zero-candidate/eligible/accepted、failure/fallback 分母与状态保留；T09限定工程证据保留；T10有限validation与T11前缀尝试已执行并保留失败/零覆盖记账，不能据此宣称正式U14通过。
- [ ] locked metrics 生成论文数字与图；T12/T13 `NOT_RUN`
- [ ] 数据发表权限与 artifact 许可；`BLOCKED_EXTERNAL`

## 引用与结果状态

- `paper/main.tex` 中保留的 bibliographic entries 尚未完成全文与一手来源核验；当前不宣称 related-work
  对比已经验证。
- System 段只加入 T00 可定位的 legacy 实测事实；其 aligned ATE 没有填入 v2 RQ 表。
- T10有限比较已执行并冻结C2-C；T11限定工作包及固定模型诊断已执行并冻结T11-C，诊断证据有效，正式E2E RQ4验证未完成，truth指标 **NOT RUN BY PROTOCOL**。原完整RQ1–RQ4验证程序未完成，T12为NOT_STARTED；不得把这些有限证据写成完整v2实证支持，也不得统称为全部NOT_RUN。
- A01 已由本轮用户指挥/审查会话技术接受，T06-R01–R05 修复与 U11 fixture 已在 development engineering
  scope 内独立复审接受；当前没有
  claim 将其写成正式已验证能力。未来任何 claim 收窄/变更先更新本表与 `STATUS.md`。

## T01 review 覆盖与证据边界（历史快照，状态已取代）

| Review 项 | 合同落点 | 对 claim 的约束 | 当前状态 |
|---|---|---|---|
| RQ3 fixed-partition constant/ramp | `EXPERIMENT_CONTRACT.md` 8.1 | 仅解释 model mismatch 与 `gamma/eta/s`；不得计入 C2 automatic end-to-end 证据 | `NOT_RUN` |
| RQ3 automatic support discovery | `EXPERIMENT_CONTRACT.md` 8.2 | 只有不读 scripted support/GT、从 Stage 1 重跑的隔离 run 才能支持 C2 | `NOT_RUN` |
| RQ3 公平性与 RQ4 隔离 | `EXPERIMENT_CONTRACT.md` 8.3、9 | gate 比较只在各路径同一 cache 内成立；RQ4 future diagnostic 不能与 RQ3 fixed-partition 合表或共用 cache | `NOT_RUN` |
| 冻结 baseline/ablation 全清单 | `EXPERIMENT_CONTRACT.md` 4 | all-range、Huber/Cauchy、fixed rejection、structured bias only、structured + debias、fit-only、s+fit、full、simulation eta-only 均保留；nominal-curvature 另作 roadmap comparator。执行分为主矩阵、预声明代表子集和缓存诊断 | 全部 `NOT_RUN` |
| 一次 segment merge | `METHOD_CONTRACT.md` 4.1；`STATUS.md` A01 | deterministic merge 会改变 partition/hash；规则已技术接受并在 immutable snapshot 上实现和独立复审，但科学参数未锁定，仍不能写成正式 C2 partition correctness | U11 `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；A01 `TECHNICALLY_ACCEPTED_IMPLEMENTED_REVIEW_ACCEPTED` |

本轮 review 只补合同与 ledger，没有生成新的 run、指标或 claim 支持。`paper/main.tex` 未修改，沿用初次
T01 已核验的双遍编译证据；本轮未重新编译。

## T02 工程证据边界

证据目录：
[`../doc/ie_sprint/evidence/t02_20260906T065545Z/`](../doc/ie_sprint/evidence/t02_20260906T065545Z/)，
总记录见 [`VERIFICATION.md`](../doc/ie_sprint/evidence/t02_20260906T065545Z/VERIFICATION.md)。

- T02 的 60/60 package tests 和 development smoke 只支持输入账本、固定 beta 语义、IMU correctness、
  薄 runner、隔离产物及同一 final graph/Values 导出基础。
- synthetic `beta=0.3 m` 只验证接口与单次修正；它不是实际标定值，也不支持 C2 效果 claim。
- development smoke 不读取 GT/oracle，且不是正式 RQ 实验；其数值不能填入论文结果表。
- 在 T02 证据边界内，discovery/refit/score/gate/fallback/covariance 当时均未实现或未运行；后续
  T04/T05 debug 进展另见下文，仍不足以使 C1–C3 达到 `SUPPORTED`。

T02 review 修复证据：
[`../doc/ie_sprint/evidence/t02_review_20260906T090000Z/`](../doc/ie_sprint/evidence/t02_review_20260906T090000Z/)，
总记录见 [`VERIFICATION.md`](../doc/ie_sprint/evidence/t02_review_20260906T090000Z/VERIFICATION.md)。

- package tests 为 `68 tests, 0 errors, 0 failures`。新增检查覆盖规范 beta link key、非空配置的使用链路
  完整性、range 级时间/tag/provenance 和 message/range 身份。
- 实际 original bag 的 627 条裁剪观测、SFUISE bag 的 820 条裁剪观测均严格属于各自参考窗口；检查脚本
  逐条读取真实 rosbag，验证源 message/range、raw time、tag、anchor，并确认所有共享 `obs_id` 不变。
- 正常 full-recording development smoke 在 126 次 LM 后按配置收敛并 exit 0；1-iteration 上限 run 为
  `FAILED/MAX_ITERATIONS_REACHED_WITHOUT_CONVERGENCE`，记录目标和迭代详情且不导出有效估计。
- malformed/partial fixed beta、非零 SFUISE group window、非零 `td_init` 和零 LM iterations 均实际非零
  退出。空 beta map 只作为 `MISSING_CALIBRATION_DEVELOPMENT_ONLY` smoke，仍不支持任何实际标定 claim。
- legacy SFUISE trajectory 与复审前 SHA-256 相同；该回归仍只支持 legacy 行为，没有升格为 v2 正式实验。

T02 第二轮复审证据：
[`../doc/ie_sprint/evidence/t02_rereview_20260906T081734Z/`](../doc/ie_sprint/evidence/t02_rereview_20260906T081734Z/)，
总记录见 [`VERIFICATION.md`](../doc/ie_sprint/evidence/t02_rereview_20260906T081734Z/VERIFICATION.md)。

- supplied overflow-objective run 保持 `FAILED`/exit `1` 和明确原因，非有限目标诊断写为 JSON `null`；
  strict parser 通过，且 trajectory/bias/residual 均未产生。
- 时间比较明确使用 `rel_tol=0` 与 double 精度绝对容差；真实 SFUISE epoch 时间 `+0.5 s` 后 checker
  exit `1`。未篡改的 original 627 条和 SFUISE 820 条裁剪观测仍逐条匹配真实 bag 且跨窗口 ID 稳定。
- package tests 仍为 68/68，正常 development smoke exit `0`。这些结果只收紧 T02 foundation 的失败审计和
  provenance 检查，不支持尚未运行的 v2 方法或正式 RQ claim。

T02 最终独立复核收口证据：
[`../doc/ie_sprint/evidence/t02_final_review_20260906T083514Z/`](../doc/ie_sprint/evidence/t02_final_review_20260906T083514Z/)，
总记录见 [`VERIFICATION.md`](../doc/ie_sprint/evidence/t02_final_review_20260906T083514Z/VERIFICATION.md)。

- 本轮指挥/审查会话最终接受 T02-R01–R07，T02 状态关闭为 `DONE`。
- 独立复核包保存 normal smoke、strict-JSON overflow failure、real-loader identity/time negative check 的
  汇总、日志、状态与配置；R01–R05 的详细执行证据仍由前两轮归档承担。
- 该接受只证明 T02 输入/runner/失败审计基础，不把 C1、C2 或 C3 标为 `SUPPORTED`；T03 及四阶段方法、
  covariance、formal RQ1–RQ4 仍不由 T02 证据支持。

## T03 数值参考证据边界

当前状态：`DONE/REVIEW_ACCEPTED`。复核来源 `/tmp/t03_review_5co105jp` 发现 T03-R01–R04；修复后由
`/tmp/t03_final_review_va5vu4tl` 完成最终独立复核并由本轮指挥/审查会话接受。

证据目录：
[`../doc/ie_sprint/evidence/t03_20260906T084306Z/`](../doc/ie_sprint/evidence/t03_20260906T084306Z/)，
总记录见 [`VERIFICATION.md`](../doc/ie_sprint/evidence/t03_20260906T084306Z/VERIFICATION.md)。

- `tools/paper/reference_recoverability.py` 对已白化 `F/G` 使用 scaled-`F` SVD 列空间投影，导出
  `E/N/R/eta/s`、F/N/R 数值秩、完整谱、实际阈值、审计和状态；没有 GTSAM/Python estimator。
- U01–U05 的解析条件已覆盖；`N=R=1e-4 m^-2` 实得 `eta=1,s=100 m`；static-beta 平移歧义、
  full-rank projector/Schur 等价和固定模型未来信息增加也通过。
- 近退化 fixture 显式保存 rank-threshold sweep，并标
  `TOLERANCE_SENSITIVE_NOT_GATE_ELIGIBLE`；没有以放宽容差、LM damping、jitter、人工 prior 或伪逆有限
  方差把不可辨识方向写成通过。
- 定向 unittest 为 15/15，9 个矩阵的 fixture 可 byte-identical 重生成并通过 strict JSON 解析。U07 的
  C++ sparse 对照仍为 `NOT_RUN`；T03 L0 结果不能支持 C2 四阶段已实现，更不支持任何正式 RQ 数字。

T03 review 修复证据：
[review evidence directory](../doc/ie_sprint/evidence/t03_review_20260906T093843Z/)，
总记录见 [VERIFICATION.md](../doc/ie_sprint/evidence/t03_review_20260906T093843Z/VERIFICATION.md)。

- 极端 `1e-200/1e200` 非零 nuisance 列均稳定归一化并保留 `col(F)`；关键中间量非有限时显式
  `NUMERICAL_FAILURE`，所有结果保持 strict JSON。
- `N-R` PSD 改用 binary64 `gamma_k` 与 `N/R` 谱尺度的误差界，无固定绝对地板；保存的
  `-1.826067290073587e-12` 正交舍入反例在界内，超过界十倍的注入错误被回归条件拒绝。
- `F=[1,1,1]^T,G=1e10F` 的 `R=7.275957614183426e-12 m^-2` 低于 projector-derived
  `8.51969777638694e-9 m^-2` 信息地板，结果为 `RANK_DEFICIENT,s=inf`；真实弱信息仍为
  `N=R=1e-4 m^-2,eta=1,s=100 m,OK`。
- fixtures 扩充为 13 个矩阵；状态/维度/case/结构严格比较、浮点字段按预声明 `atol/rtol`。默认和
  Sandybridge 各 22/22 tests 且跨环境数值比较通过；同环境重生成 byte-identical 单独通过。
- 最终独立复核精简包见
  [`t03_final_review_20260906T110914Z`](../doc/ie_sprint/evidence/t03_final_review_20260906T110914Z/)：
  默认/Sandybridge 各 22/22、跨 BLAS 容差比较、同环境 byte-identical 和 100 个独立 Schur case 均通过。
- Python GTSAM、C++ sparse 对照和正式 RQ1–RQ4 仍 `NOT_RUN`；这些 L0 结果不把 C1–C3 标为 `SUPPORTED`。

## T04 oracle-debug 证据边界

当前状态：`DONE/REVIEW_ACCEPTED`；R01–R03 均由最终独立复核接受。初始证据目录：
[`t04_20260906T111500Z`](../doc/ie_sprint/evidence/t04_20260906T111500Z/)；review 修复证据目录：
[`t04_20260906T121005Z`](../doc/ie_sprint/evidence/t04_20260906T121005Z/)，总记录见
[`VERIFICATION.md`](../doc/ie_sprint/evidence/t04_20260906T121005Z/VERIFICATION.md)。
最终独立复核归档见
[`t04_final_review_20260906T143023Z`](../doc/ie_sprint/evidence/t04_final_review_20260906T143023Z/VERIFICATION.md)。

- `MakeSegmentUwbFactor` 在非平凡 Pose3/lever/anchor/fixed-beta/`C(s)` 下通过实际 GTSAM retract 的白化
  Jacobian 中心差分；残差为 `h+beta+c-z`，key 集只含独立 `X/C`。
- oracle manifest 只接受 debug 标签、唯一 segment ID、规范 link 和互斥的 obs IDs/闭区间；不接受幅值、
  GT、pose、initialization、未知字段或重复/跨 link/无效归属。
- 成功 4 秒 debug run 的最终 joint graph 为 74 factors/25 values，8 个 raw obs 由 8 个 segment factors
  共享唯一 `c0`，没有 amplitude prior 或 corrected pseudo-range。闭式 `c` 更新后在同一最终 graph/Values
  对所有自由 `X/V/B` 检查单位尺度化梯度；第 13 轮旧 objective/step/KKT 三项已通过但 navigation
  stationarity 未通过，第 14 轮梯度从 `1.4538511614592409e-6` 降至 `8.1725042155866845e-7` 后才成功。
- segment ID 含逗号、双引号和换行时，segments/factor metadata/residual 三表均经标准 CSV parser 保持列数、
  ID 和 obs 关联；未知 manifest 字段含换行时 exit 1，状态 strict JSON 保真、debug 标签正确且不导出估计。
- 一轮上限 run 明确 `MAX_REFIT_ITERATIONS`/exit 1，且没有有效 trajectory/amplitude/residual 导出。
  T02 all-range 与 legacy SFUISE trajectory 均和旧证据 byte-identical。
- 最终复核原样保留：`-0.4` fixture 在 20 轮后仍为 `MAX_REFIT_ITERATIONS`、`c=0`、navigation
  gradient `7.928690804792637e-06`；`-0.2/-0.1` 收敛到约束边界；嵌入 NUL 的非法 YAML reason
  会经 exception `what()` 截断，但 exit 1/失败状态/strict JSON/无估计导出正确。没有为制造成功调整阈值。
- 输入缺独立 fixed `beta`，只标 `MISSING_CALIBRATION_DEVELOPMENT_ONLY`。T04 不包含 T05 score、T06 automatic
  discovery、T08 gate/fallback/covariance 或正式 RQ，因此 C2 只到 `PARTIAL_T04_ORACLE_DEBUG_ONLY`，C1–C3
  均不标 `SUPPORTED`。

## T05 sparse score 证据边界

当前状态：`DONE/REVIEW_ACCEPTED_T05_ORACLE_DEBUG_VALIDATED_DOMAIN`；Gate A/Gate B 及 R01–R05 已在
当前已测试 sparse 支持域完成独立复审。Gate A 证据见
[`t05_sparse_prototype_20260906T143023Z`](../doc/ie_sprint/evidence/t05_sparse_prototype_20260906T143023Z/VERIFICATION.md)，
Gate B 证据见
[`t05_20260906T144556Z`](../doc/ie_sprint/evidence/t05_20260906T144556Z/VERIFICATION.md)，review 修复见
[`t05_review_fix_20260906T160000Z`](../doc/ie_sprint/evidence/t05_review_fix_20260906T160000Z/VERIFICATION.md)。
最终复核包见
[`t05_final_review_20260907T022011Z`](../doc/ie_sprint/evidence/t05_final_review_20260907T022011Z/VERIFICATION.md)。

- 原 T03 golden 未改；13 个 fixture 在 sparse 支持域内对照。冻结 SVD rank 定义不变，naive QR 只作
  诊断；review 后用完整 scaled-F 列数量/谱界与 triangular inverse-norm bound 证明支持域，不能证明时
  返回 `SPARSE_RANK_UNCERTAIN`。指挥、上三角和大量重复列反例均不再导出错误有效分数。
- `N/R` 数值审计不再使用 unit scale floor；真实弱 `N=R=5e-11` 与 weak-R 均保留有限有效结果，
  `N-R` error bound 无固定绝对地板，rank/PD/最终状态阈值一致，归一化正交残差使用归一化容差。
- 真实 GTSAM scoring graph 排除全部 candidate 后每组只加回自身 factor；已白化 Jacobian 只使用一次，
  RHS 独立。factor index/实际 keys/类型/obs ownership/candidate coverage/group mask 均在评分前校验；
  linearization ID 对实际 mask、ordering/mapping 和白化 F/G/RHS 作规范 SHA-256。
- U06 test-only online beta、U09 physical information 对 LM damping 不变、U10 跨组排除，以及端点传递
  overlap、RHS 排除、short/boundary 阶段隔离均有定向测试。
- 最终 oracle debug 短输入只有 1 group，得到 `eta=0.81570732433217819`、
  `s=0.088141539221235826 m`、`gamma=0.042596852206918272`。该输入缺独立 fixed `beta`，且
  condition/roundoff 仍为 `PENDING_NUMERICAL_PROPOSAL`，所以这些数字不进入论文正式表图或 claim。
- short/boundary 的主 `eta/s` 字段为 unavailable，原始数学诊断只进入 `debug_*`/audit；这与 rank-deficient
  的 `s=+inf` 分开。review 修复最终定向 6/6 与 19/19、package 120/120、T03 两环境各 22/22，
  T04 default-off、T02 all-range 和 legacy 轨迹回归通过。
- 独立复核为 recoverability 6/6、refit/scoring 19/19；100 个 seeded case 中 85 个支持域结果匹配、15 个
  明确 unavailable。未独立重跑的 package/T03 双 BLAS/T04/T02/legacy 项按原样保留。此收口不覆盖
  automatic discovery、gate、fallback、final covariance、locked RQ 或 multi-link test；C1–C3 均不标
  `SUPPORTED`。

## T06 automatic discovery 验收证据边界

当前状态：`DONE/REVIEW_ACCEPTED_T06_AUTOMATIC_DEVELOPMENT_ENGINEERING_SCOPE`。证据目录：
[`t06_20260907T025122Z`](../doc/ie_sprint/evidence/t06_20260907T025122Z/VERIFICATION.md) 与
[`t06_review_fix_20260907T035905Z`](../doc/ie_sprint/evidence/t06_review_fix_20260907T035905Z/VERIFICATION.md)，
最终独立复核见
[`t06_final_review_20260907T043727Z`](../doc/ie_sprint/evidence/t06_final_review_20260907T043727Z/VERIFICATION.md)。

- C++ Stage 1 实现冻结非负 L1/TV 目标；chain ADMM 以 `p=rho*q` 审计最终可行 `u` 的 primal、dual、
  原始 KKT 与 `Du` TV 次梯度，外层以原始目标、navigation 与逐观测 bias 的组合尺度步长、navigation
  驻点停止。独立 SciPy epigraph reference 另查
  可行性与 KKT，而不只看 solver status；多个 rho、非均匀权重、单点、零正则、constant/ramp 均本地通过。
- automatic/oracle 输入分离，共同下游复用 T04 refit 和 T05 score；automatic 强制 scoring、显式科学参数并
  前置拒绝任何 oracle 字段。A01 immutable snapshot 使用稳定加权累计并保存 SHA-256 context、父子 ID、
  `mu_merge_snapshot`、obs 唯一归属和 partition hash，Stage 2 后冻结。无候选仍执行无 `C(s)` 的 raw Stage 2，
  只有目标/步长/最终驻点等全部适用条件通过才返回 `NO_CANDIDATES`；score 为空/not-applicable，没有 T08 fallback。
- 实际 4 秒 development run 的 manifest 明确 `gt_read=false/oracle_support_read=false`，发现 17 段，其中
  short 13、boundary 0；Stage 2 收敛并导出 graph/Values 中间结果。3 个 overlap group 均含 short，主评分
  不适用，`valid_score_exported=false`，run 按设计 exit 1。缺独立 fixed beta，不能用于正式结果或调参。
- 无 oracle/GT 的自动工程小图产生非空 eligible candidate，经共享 refit 进入有效 score；这只验证工程闭环，
  不是正式 RQ。Stage 1 故意令 ADMM 一轮超限的实跑 exit 1，仍保留 manifest/effective config/capability/
  diagnostics、snapshot/partition 与 trace，且不运行 Stage 2。
- 独立复审重跑 discovery 14/14、refit/scoring 23/23、recoverability 6/6、config 8/8、5 cases x 3 rho
  reference 与 14/14 preflight probes；保留旧默认空 fixture 的 20 轮非收敛。full package/build、T03 双 BLAS、
  T04/T05/T02/legacy 未由该复审重跑，只核验开发者证据。
- T06 证据自身不含 gate/fallback/final covariance/T08、locked parameters、formal RQ 或 final decision。
  该产物只证明 reviewed development engineering scope 内的 discovery/refit/score 工程闭环，不支持任何
  C1–C3 `SUPPORTED` 状态。

## T07 deterministic input engineering 证据边界

当前状态：`DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`。接口说明见
[`T07_IMPLEMENTATION.md`](../doc/ie_sprint/T07_IMPLEMENTATION.md)，本地证据见
[`t07_20260907T053301Z`](../doc/ie_sprint/evidence/t07_20260907T053301Z/VERIFICATION.md)，
R01--R03 修复证据见
[`t07_review_fix_20260907T062126Z`](../doc/ie_sprint/evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md)，
最终独立复审收口见
[`t07_final_review_20260907T070102Z`](../doc/ie_sprint/evidence/t07_final_review_20260907T070102Z/VERIFICATION.md)。

- step/ramp 只在已有 full base UWB range 上加入确定性非负分量，不仿真动力学；完整 base 固定
  SHA-256、recording ID、sensor-time origin 与原始 source ordinals，fixture 的 `1:1=79`、`1:2=77`
  只作 development 计数约束。
- estimator cache 锁定 normalized IMU 单位、UWB 单位、时间基准、消息分组、payload hash/count 与
  recipe-independent cache ID；paper runner 只读 cache，继承 base recording ID 并复核 stable `obs_id`，
  crop 在初始化前完成且不重复单位转换。review-fix 后 reader/writer 同时强制 UWB 非递减时间、严格递增
  source-message/source-observation ordinal、group 唯一连续与 range ordinal，并强制 IMU row/time 顺序；交换
  完整 group 或 IMU 行后重算 payload hash/cache ID 仍被拒绝。
- injected-component truth 独立存放并明确 `total_latent_bias_status=UNKNOWN`。基础 RSSI/residual 与当前
  simulator source/config 的吻合只标 empirical consistency；缺 runtime parameter snapshot/realized seed，
  不构成 generation provenance，也不计算未知总 bias 的恢复指标。
- generator 对 root 与最终目录做物理 canonical/包含关系检查，拒绝 exact-same、symlink alias 和重叠路径；
  双侧 staging/cleanup 保留 no-overwrite，成功 cache 精确为三文件且 truth 只在独立两文件目录。该清理保证只覆盖
  可捕获异常，不声明 `SIGKILL`、进程死亡或断电下双目录发布具有 crash atomicity。
- 12498 行 component conservation、step/ramp 各 79/77、跨 scenario stable obs IDs、prefix 零匹配/共有历史、
  payload/manifest 篡改拒绝和 byte-identical step 重生成在本地通过。base、recipe、truth 同时移走后，真实
  paper runner 仍从 cache 进入 Stage 1；checker 核对实际 cache/recording/interface/no-conversion，运行期持续
  检查三路径不可达并在 finally 恢复，以 50 行 discovery trace 与 diagnostics 证明 Stage 1 而非只看早期 manifest。
- step/ramp 两个真实 smoke 均在未调整 T06 development 参数的情况下于 Stage 1
  `MAX_OUTER_ITERATIONS` 失败；refit 与 recoverability score 均 `NOT_RUN`。这些输入工程事实不是 RQ
  结果，不支持 C1–C3 `SUPPORTED`；它们在后续 T08 本地任务中仍原样保留为 T08 不可达。
- 独立复审确认 T07-R01--R03 均为 `REVIEW_ACCEPTED`；接受范围仅是 T07 输入工程。base 总 latent bias
  仍为 `UNKNOWN`，runtime realized seed/完整参数 provenance 仍不完整，formal RQ 仍 `NOT_RUN`。T08 后续已按
  development engineering scope 独立复审收口；T09--T13 仍 `NOT_STARTED`。

## T08 final inference 证据边界

当前状态：`DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；R01
`REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`、R03 `REVIEW_ACCEPTED_TESTED_CONTENT_IDENTITY_SCOPE`，
R02 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`。初次证据见
[`t08_20260907T083437Z`](../doc/ie_sprint/evidence/t08_20260907T083437Z/VERIFICATION.md)，review-fix 证据见
[`t08_review_fix_20260907T092037Z`](../doc/ie_sprint/evidence/t08_review_fix_20260907T092037Z/VERIFICATION.md)，
R02 最终定向修复证据见
[`t08_r02_review_fix_20260907T104154Z`](../doc/ie_sprint/evidence/t08_r02_review_fix_20260907T104154Z/VERIFICATION.md)，
最终独立复审收口见
[`t08_final_review_20260907T121510Z`](../doc/ie_sprint/evidence/t08_final_review_20260907T121510Z/VERIFICATION.md)。

- `FinalInferenceEngine` 返回唯一 `InferenceResult`，持有实际 final graph/Values、冻结 masks、decision/final
  score sets、状态、timing、factor audit 与 full-final-graph covariance。所有最终 state/bias/residual/covariance
  exporter 只接收该对象；T08 不调用 legacy GNC/rejection。
- full frozen candidate set 与 accepted ordinal set 分开传入 final graph reconstruction。定向测试在同一图内
  验证 accepted raw range 一次并绑定 live `C`、suppressed candidate 零次且无 `C`、noncandidate reference
  一次、corrected pseudo-range 为零。final score 使用独立 linearization ID 且不重新接纳 frozen Suppress。
- review-fix 以实际 final re-score/acceptance failure 触发一次 all-suppressed fallback；success 为
  `FALLBACK_OK`，再次失败为 `ESTIMATION_FAILED`。recovery group/segment score、失败原因、linearization ID
  与 solver/iteration trace 独立保留，fallback-final 无 `C` 的 score 另写不适用且不覆盖 recovery。失败结果
  为空 final graph/Values、无有效 trajectory；covariance success/unavailable 均测试且不写数值占位。
- 自动工程 fixture 通过实际 `AutomaticSupportProvider` 进入 T08，其 API 不接收 oracle/GT/label/reference
  trajectory。另一个 oracle-development runner 仅用于产物验收，exit 0/`OK`，74 factors/25 Values、8 accepted/
  56 reference、25 covariance blocks。review-fix runner 8 JSON/21 CSV strict parse；external/runner/Stage-4
  timing 为 `9.474669116/9.463173210/0.007065469 s`；一轮 Stage-2 失败 runner 也通过同一墙钟语义的外部核验。content identity 绑定实际 graph/Values 数值与必要
  input/config context，相同输入稳定、不同 Values/不同 factor 数值但等结构/等总 error 可区分，exporter
  拒绝 seal 后篡改。完整 CTest 17/17、T08 定向 18/18。
- runner gate 数值 `0/1000000 m/1000000` 明确为
  `T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION`，不是 T10 locked gate，也未依据 smoke 调参。该输入缺独立
  fixed `beta`；T07 step/ramp 仍在 Stage 1 `MAX_OUTER_ITERATIONS` 且 T08 `NOT_RUN`。没有正式 RQ、性能或
  held-out evaluation，因此这里只增加已独立复审的 development engineering 证据，不把 C1–C3 标成 `SUPPORTED`。
- R02 独立反例的非法 oracle manifest 在修复前后都保持 exit 1、原 reason、`ORACLE_SUPPORT_LOAD` 和
  `INVALID_ORACLE_MANIFEST`。修复后 runner/external wall 为 `9.442706742/9.455848487 s`，Stage 1–4
  未执行项均为 `null` 且无有效估计；normal、Stage-2 failure、`LmFailure` 使用同一 elapsed semantics。
  最终独立复审重放得到 runner/external wall `9.441450969/9.453941426 s`、差 `0.012490457 s`，并确认
  R02 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`。本轮未重跑 package build、完整 CTest、T07 step/ramp、
  正式 RQ、held-out validation 或性能/RSS；不把旧验证冒充本轮运行。

## T09 batch/cache/evaluator 证据边界

当前状态：
`DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE / AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`。
本地证据见
[`t09_20260907T143701Z`](../doc/ie_sprint/evidence/t09_20260907T143701Z/VERIFICATION.md)；R01–R08 review-fix 见
[`t09_review_fix_20260907T164311Z`](../doc/ie_sprint/evidence/t09_review_fix_20260907T164311Z/VERIFICATION.md)；第二轮
R01-A/B、R02-A、R03-A/B/C、R04-A 定向修复见
[`t09_rereview_fix_20260907T180000Z`](../doc/ie_sprint/evidence/t09_rereview_fix_20260907T180000Z/VERIFICATION.md)。

- T09 提供 canonical method registry、独立 baseline、Stage 1 regularized snapshot、common/cache/final-request
  identity、automatic/fixed cache namespace、batch DAG、独立 GT exporter 和 run-unit evaluator。T08
  `inference_id` 的内容语义未改变，写后增加了 live identity、graph/Values count、CSV row ID 和 artifact hash
  核验；cache publisher/reader 的 ID 编码由 C++/Python 共用固定向量验证。
- development batch 的 19 个预登记 cell 全部终态。T07 的 all-range/Huber/fixed rejection 和 ramp
  all-range 不受 Stage 1 影响并成功；step/ramp automatic producer 仍为 `MAX_OUTER_ITERATIONS`，相应目录无
  Stage 2/cache/recovery/final 产物。T06 automatic cache 保留完整 group 表和 partial-score 状态；fixed
  partition cache 仅以精确 DEBUG label 发布到另一 namespace。
- common preparation comparability 使用 runner 导出的实际 graph/Values/context 内容身份；最终 replay 对每个
  run unit 的执行方法均一致，没有 `COMPARABILITY_INVALID`，预登记 request hash 未冒充实际身份。
- evaluator 不把 diagnostic 或 fixed cache producer计入 trajectory/automatic 重复。automatic producer 为
  3 个 run unit，失败 2 个；三个 all-range、各一个 Huber/Cauchy/fixed-rejection 的 development trajectory
  failure 均为 0。所有比例携带 numerator/denominator/status/reason，零分母不是数值 0。
- 本轮 evaluation manifest 不含 GT/frame transform，因此 ATE/RPE 不产生论文数字；T07 只有
  `INJECTED_COMPONENT_ONLY` truth，因此 total latent bias 指标保持 unavailable。development threshold 仅为
  `T09_DEVELOPMENT_ENGINEERING_TEST_ONLY`，不是 T10 lock。
- review-fix 后，真实 fixed-partition DEBUG 链路由 1 个 producer 发布含完整 `X/V/B/C` 的 cache，2 个 diagnostic
  operating point 和 `fit_only/full_gate` 2 个 final 消费同一 cache；final 明确不重跑 Stage 1/2 optimizer，并在
  进入未改变的 T08 engine 前复核 Stage-2 graph/Values 内容身份；归档 cache 另由第二个 scheduler 进程复用为
  新 diagnostic cell。该链路只证明 DEBUG replay，不替代 automatic。
- evaluator 现按 parent cache 与 inference ID 连接 decision/final masks、segment bias、score 和 summary；分别报告
  recovery solve/final-audit 与 fallback 分母，RPE 使用同一刚体的 SE(3) relative pose。comparability invalid 结果
  不能进入聚合；validation/test 未获准请求在预检拒绝；ABI 身份不含 ASLR 地址。
- 第二轮修复进一步用 binary64 区分相邻阈值 request，cache v2 绑定 producer common/Stage1/Stage2-refit/Stage3-score
  配置，并在 external consumer 与 C++ replay 拒绝 discovery 变化；真实 AUTO producer 的两个相邻精度 final 共用
  cache 但 request ID 不同。evaluator 对 final 封存文件复算 SHA-256、以 factor audit 计 raw-factor usage，并按
  policy/point/path 与唯一 run-unit 分层；普通 upstream/recovery/fallback failure 保留 row/分母。decision-time bias
  不再被 final amplitude 覆盖，final accepted amplitude 缺失不会回填 Stage 2。replay 成功/失败均恢复统一 runner
  total wall 与 Stage-4 nullable breakdown。
- 首轮 package build、focused suites、完整 CTest 24/24、Python contract tests、batch/evaluator 和 diff check
  均实际 exit 0；第二轮又完成 build/tests target、focused 10/10、完整 CTest 25/25、真实 AUTO 多消费者链、
  stale-cache/tamper 反例和 diff check。
- 最终独立复审见
  [`t09_final_review_20260908T050201Z`](../doc/ie_sprint/evidence/t09_final_review_20260908T050201Z/VERIFICATION.md)：
  被审 source/runner/core identity 匹配，focused 9/9；两个 AUTO-cache final consumer 均 exit 0、request 区分、
  Stage 1/2 optimizer `NOT_RUN_CACHE_REPLAY` 且 export `VERIFIED`；scheduler/direct stale-cache 分别 exit 2/1，
  普通 sealed-file 篡改 exit 2，failure/bias accounting probes 通过。因此 R01-A/B、R02-A、R03-A/C、R04-A
  与 R05–R08 在当前 development-engineering 范围接受。
- R03-B 最小本地修复见
  [`t09_r03b_fix_20260908T053929Z`](../doc/ie_sprint/evidence/t09_r03b_fix_20260908T053929Z/VERIFICATION.md)：
  evaluator 现验证 parent ledger identity、本地副本一致性、ledger/mask/audit 三层 observation domain，严格
  核对 accepted/suppressed/noncandidate classification、flags 与 actual/expected count；retained numerator 与
  固定有效观测 denominator 只取已验证结果。真实 final 保持 `41/627`，orphan/missing/duplicate audit、错误
  classification、矛盾 expected count/flags 与本地 ledger 篡改均 exit 2；failure/NA/run-unit 分母保持。
  该修复的历史状态为待独立复审；最终结论由下述收口复审取代。没有 held-out、正式 RQ、独立 fixed beta、
  数据许可或论文表图证据，C1–C3 不提升为 `SUPPORTED`。
- R03-B parent-cache 内容身份续修见
  [`t09_r03b_parent_identity_fix_20260908T062741Z`](../doc/ie_sprint/evidence/t09_r03b_parent_identity_fix_20260908T062741Z/VERIFICATION.md)：
  evaluator 在接受 parent 前直接复用既有 scheduler canonical v2 定义重算 Stage-2 cache ID，并核对 manifest、
  cell 与 final binding；既有 payload、ledger、mask/audit 校验和合同分母未改变。真实归档反例中，payload/ledger
  hash 一致但 cache ID 过期，以及 parent 重算新 ID 后替换旧 final，均 exit 2 且不生成 evaluation；正常仍为
  `41/627`，七个既有负例与 failure accounting 保持。该续修的历史状态为本地通过待独立复审；最终结论由
  下述收口复审取代。
- 最终独立复审与文档收口见
  [`t09_final_rereview_closeout_20260908T065646Z`](../doc/ie_sprint/evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md)：
  独立复审接受 R03-B，并结合此前接受的 R01–R08，将 T09 关闭为
  `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`。归档复审确认正常真实 final 为 `41/627`，完整
  canonical-v2 parent/cache/final identity 链、ledger/mask/audit 双向完整性、七个既有负例与 failure/NA/run-unit
  accounting 均通过；两个 fresh C++ AUTO final consumer 的 request 不同、parent cache 相同，Stage 1/2 optimizer
  均为 `NOT_RUN_CACHE_REPLAY`，export 为 `VERIFIED`。
- 该接受范围不包含 automatic accepted-candidate 科学闭环、正式 RQ、T10 准入或 claim 支持。本轮 package build、
  完整 CTest 25、19-cell batch、T07 step/ramp、新 discovery、validation/test、locked gate 和 T10–T13 均
  `NOT_RUN`。T07 step/ramp 仍在 Stage 1 `MAX_OUTER_ITERATIONS`，故保留
  `AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`；T06 partial-score 与 fixed-partition DEBUG 均不能替代 automatic E2E。
  数据许可、独立 fixed beta 及标定/GT 等外部 provenance 缺口仍在，C1–C3 继续不为 `SUPPORTED`。

## T10 precheck 与 Stage 1 诊断证据边界（历史快照，状态已取代）

当前状态：`IN_PROGRESS/A04_REVIEW_ACCEPTED/A05_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE/A06_REVIEW_ACCEPTED_BOUNDED_SCOPE/A07_REVIEW_ACCEPTED_BOUNDED_SCOPE/A08_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE/A09_REVIEW_ACCEPTED_BOUNDED_SCOPE_A10_REVIEW_ACCEPTED_BOUNDED_SCOPE_A11_REVIEW_ACCEPTED_BOUNDED_SCOPE_A12_REVIEW_ACCEPTED_BOUNDED_SCOPE_A13_LOCAL_DIRECTED_AUDIT_COMPLETE_AWAITING_REVIEW`。准入报告见
[`T10_READINESS.md`](../doc/ie_sprint/T10_READINESS.md)，前置检查证据见
[`t10_precheck_20260908T073803Z`](../doc/ie_sprint/evidence/t10_precheck_20260908T073803Z/VERIFICATION.md)，
诊断实施证据见
[`t10_stage1_diagnostics_20260908T081328Z`](../doc/ie_sprint/evidence/t10_stage1_diagnostics_20260908T081328Z/VERIFICATION.md)。

- 真实 AUTO 的可用程度被分成三层记录：T06 短输入能完成 Stage 1/2，但 3 个 group 全不 eligible；T09 可从
  该 partial cache 完成两个 final replay，结果为 `NO_ELIGIBLE_CANDIDATES`；T07 step/ramp 则仍在 Stage 1
  终止，没有 Stage 2/cache/decision/final。automatic engineering fixture 只证明有效评分接口，fixed-partition
  cache 只属于 `FIXED_PARTITION_DEBUG`，二者都不替代 accepted-candidate AUTO 科学证据。
- Stage 1 新增只读诊断记录实际 linked-GTSAM LM check 的 initial/previous/current error、有效三类 tolerance、
  全部触发谓词、既有 inner iterations/lambda，以及同一 `lm.values` 上旧 bias conditional graph 和更新 bias graph
  的既有 navigation stationarity audit；旧字段保持前缀兼容，诊断不驱动 solver。focused discovery 17/17、
  refit 23/23、runner serialization contract 与加载库 predicate-order probe 均通过。
- T07 step/ramp 使用原输入、原科学配置、原停止条件各定向运行一次，均保持 50 轮
  `FAILED/AUTOMATIC_DISCOVERY/MAX_OUTER_ITERATIONS`，Stage 2/final `NOT_RUN`；与历史 trace 的 18 个旧字段
  逐行最大浮点差为 0，终态完全一致。100/100 次 conditional LM 均只由 relative-decrease 谓词停止，但旧
  bias 下 pre-chain navigation stationarity 为 0/50、0/50，末轮仍分别为容差的 268.48/188.28 倍；更新 bias
  后梯度在两条 run 的全部轮次增大，末轮 post/pre 为 23.86/46.38 倍。
- 这些事实支持 generic LM 停止不足与 block coupling 放大的混合诊断；不证明 linked GTSAM 实现错误，也不把
  `MAX_OUTER_ITERATIONS` 当根因。末 10 轮三个外层失败量继续下降，未观察到明确平台；更多 outer budget 是否
  成功和数据/模型/calibration 贡献仍为 `UNKNOWN`。下一项只提交 stationarity-qualified conditional solve 的
  可审查方案，本阶段不改变 solver 行为。
- T07 truth 继续严格标作 `INJECTED_COMPONENT_ONLY`，total latent bias 为 `UNKNOWN`；seed 只进入上下文身份。
  post-fit residual 不是 bias truth。step/ramp 共用且已经使用同一 development 基础记录，不能改标为 unseen
  validation/test。
- scheduler 当前仍硬拒绝非 `development`。split、合成生成 provenance 或真实独立 calibration、待审数值、
  operating points 与 `B_total` 均未闭合。只读诊断修改了产品源码中的观测字段和序列化，但未修改科学配置、
  停止条件、objective、block order、gate 或冻结合同，也没有登记 amendment。
- 正式 validation、sweep、matched-cache gate、locked gate、test、完整 19-cell batch、正式 RQ/U14、T11–T13、
  locked metrics 和论文图表均 `NOT_RUN`。C1–C3 的表格状态和复选框保持未满足，不升级为 `SUPPORTED`。

## T10 A02 development policy 证据边界（历史快照，含以下各阶段记录）

A02 完整证据见
[`t10_conditional_stationarity_policy_20260908T091614Z`](../doc/ie_sprint/evidence/t10_conditional_stationarity_policy_20260908T091614Z/VERIFICATION.md)。

- 默认关闭、显式版本化的 `GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1` 已实现；actual linked GTSAM generic
  check 与既有 conditional navigation stationarity audit 同时通过才返回成功。未通过时继续同一 optimizer，
  不重置 lambda、不绕过原 inner cap；invalid/nonfinite/exception/cap 不导出有效估计。
- config 11/11、discovery 23/23、refit 23/23 及 runner contract 通过。policy 进入 effective config、trace、
  failure 及 solver/support/cache identity；默认策略的 canonical identity 保持兼容。cap strict JSON 产物保留
  最后 audit，旧封存 verifier 未修改。
- 原 step/ramp development 输入各运行一次。两条 100 个 outer round 的 conditional qualification 均通过，
  但 chain 更新后末轮 navigation gradient 仍为 `6.317840e-3/8.691964e-3`，relative objective 与 combined
  step 也未通过；两条终态仍为 50 轮 `MAX_OUTER_ITERATIONS`，Stage 2/score/decision/final `NOT_RUN`。
- conditional LM iteration 总数由 `115/118` 增为 `538/513`，Stage 1 墙钟约为诊断基线的 `3.87/3.93`
  倍。末段三个失败指标仍单调下降，故更多 outer budget 的结果仍为 `UNKNOWN`。这支持 post-chain block
  coupling/外层 fixed-point 推进仍是直接阻塞，不证明数据/模型因素不存在，也不支持默认迁移。
- A02 是已登记的 `DEVELOPMENT_IMPLEMENTATION_AND_DIAGNOSTIC_RUNS_ONLY` 数值策略，不是 correctness fix、
  正式 validation、gate lock、RQ 或 claim evidence。T07/T09 保持 `DONE`；C1–C3 状态、复选框与证据等级均
  不升级。

### A02-R01 参数一致性修复与 A03 限定预算诊断

修复与诊断证据见
[`t10_a02_r01_budget_diagnostic_20260908T120339Z`](../doc/ie_sprint/evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/VERIFICATION.md)。

- 独立复审输入复现了 A02 outer/support canonical 与 conditional solve 可使用不同 tolerance、roundoff 和五类
  navigation scale、同时仍得到相同 support identity 的反例。A02-R01 在 provider 与独立 partition 两个入口
  对七类参数逐一强制精确一致；冲突明确拒绝，一致输入可用。默认关闭策略行为/身份兼容，独立
  `RunCheckedConditionalLm` 用途不变。focused build、config 11/11、discovery 26/26、refit/scoring 23/23
  和 runner contract 均实际通过；A02-R01/A03 已由独立复审接受，但只限 development scope。
- A03 在运行前登记为 `DEVELOPMENT_BUDGET_DIAGNOSTIC`。step/ramp 两份隔离配置各自唯一语义差异为 outer cap
  `50 -> 500`；每条原输入只运行一次并受整进程 120 秒限制，无 timeout、重跑或追加 cap。两条前 50 轮的
  84 列 trace 中所有非计时字段与 cap=50 精确一致，最大绝对差为 0；计时字段单列，config/solver/support
  与潜在 Stage-2 producer-config identity 均已分离。
- step/ramp 分别完成 120/126 个 outer round，在第 121/127 次 conditional solve 用尽不变的 50-check inner
  budget，终态均为 `FAILED/AUTOMATIC_DISCOVERY/CONDITIONAL_LM_STATIONARITY_NOT_REACHED`。不存在四项 AND
  同时通过的轮次；Stage 1 失败，Stage 2 未运行，eligible candidate 未评价。完整进程耗时分别为
  `0.831115288/0.806571784 s`，reported optimizer iterations 为 `914/865`，实际 convergence-check calls 为
  `963/914`。
- 该诊断只排除“outer cap=50 是唯一最终阻塞”这一窄解释。输入仍为 development-only，truth 仅为
  `INJECTED_COMPONENT_ONLY`，base total latent bias 为 `UNKNOWN`，独立 fixed `beta` 与科学 provenance 未闭合；
  因而不构成有效评分、默认迁移、正式 validation/RQ 或 C1–C3 升级依据。A03 不授权进一步增加 inner/outer
  budget、修改 tolerance 或重复运行。

### conditional LM 停滞定位

证据见
[`t10_conditional_lm_stall_20260908T130400Z`](../doc/ie_sprint/evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md)。

- 唯一 A03 step replay 未 timeout，在 outer 121 重现原失败。50 次 wrapper check 中只有第 1 次 accepted
  state update；其后 49 次 linked GTSAM candidate 的 model fidelity 为负且 cost change 极小，`tryLambda`
  由 small-relative-cost 分支直接返回但不更新 Values/state/lambda/iteration。实际为 50 条 try、49 条该返回、
  0 次 lambda-search rejection，故 optimizer iterations 保持 1。
- 最大梯度 `x1` translation coordinate 4 为 `-2.29030094e-6 objective/m`。预声明 13 点中央差分网格中
  `1e-4..3e-8` 连续 8 点满足固定误差界，不支持该坐标存在 Jacobian/objective graph 不一致；tentative
  objective difference 很小，但 Values 停止变化的控制流原因是候选未接受后的无更新返回。
- 前 120 轮 79 个非计时字段与 A03 baseline 精确相同。Stage 1 仍失败，Stage 2 未运行，eligible candidate
  未评价；truth 仍为 `INJECTED_COMPONENT_ONLY`、base total latent bias 和独立 calibration provenance 未闭合。
  本诊断不是 scientific data、新 estimator、validation 或 claim 支持；C1--C3 状态不变。
- 建议的最小修复只是 A02 qualified policy 首次检测到“linked iterate 无 accepted progress 且驻点失败”时
  显式 fail fast，避免重复浪费 check budget。它尚未实施且需新版本 identity/独立审查；不会把失败写成成功，
  也不授权覆盖 GTSAM stop、增加 lambda/cap 或迁移默认策略。

### A04 fixed-checkpoint LM 恢复可行性

证据见
[`t10_fixed_checkpoint_lm_recovery_20260908T142233Z`](../doc/ie_sprint/evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/VERIFICATION.md)。

- A04 在运行前登记且只执行一次 A03 step 活图重捕获；ramp 0 次。A/B 从同一 73-factor graph、24-key
  Values、objective `4.8408701715814653` 与 lambda `1e-6` 启动。原 estimator 仍 outer-121 Stage-1 failure，
  shadow Values 未写回。
- A 当前行为为 50 calls/50 trials、0 accepted、0 objective decrease，最终 gradient
  `2.29030094e-6`，不驻点。B 唯一变化是 optimizer 内部 `relativeErrorTol 1e-6->0`；这明确改变 search
  termination 数值语义，external generic 仍为 rel `1e-6`/abs `1e-8`。
- B 首 call 在 `lambda=1e-6` 拒绝后增加至 `1e-5` 并真实接受下降，第二 call 在 `1e-6` 再接受下降；
  共 2 calls/3 trials、2 accepted strict descents，总 objective 下降 `3.86357613e-13`，最终 gradient
  `3.45174471e-7` 并通过原 stationarity。model-fidelity 门槛、lambda upper `1e5`、50-call cap 均不变。
- 前 120 轮 79 个非计时字段和 authoritative 50-call capture 与已接受停滞 replay 精确一致。affected
  discovery 28/28、config 11/11、重链接后 refit/scoring 23/23 及 T06 runner contract 通过。
- 这只支持“固定 checkpoint 存在 bounded recovery path”的工程结论，不是 A02/default 集成、Stage-1
  成功、validation、scientific data 或 claim 支持。随后 A05 已另行登记并完成版本化集成及定向运行；
  独立复审前仍不迁移默认，C1--C3 保持不升级。

### A05 默认关闭 V2 集成与定向验证

证据见
[`t10_v2_conditional_lm_recovery_20260908T151942Z`](../doc/ie_sprint/evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/VERIFICATION.md)。

- A05 在实现和运行前登记。V2 只把 optimizer 内部 `relativeErrorTol` 设为 0；外部 generic rel
  `1e-6`/abs `1e-8`、stationarity、roundoff、五类尺度、GTSAM 接受规则、lambda upper `1e5`、50-call cap
  和外层四项 AND 均不变。默认 V1 不迁移。
- focused checks 最终为 discovery 30/30、config 11/11、refit 23/23、inference 18/18、paper methods 5/5、
  Stage-2 cache 7/7，runner 与 scheduler identity checks 通过。V1/V2 的 config、solver、support context、
  snapshot 与 scheduler producer identity 分离；两条 Stage 1 失败，未产生实际 Stage-2 cache。
- step/ramp 隔离配置相对 A03 V1 基线只有 policy 不同，outer=500、inner=50，严格各执行一次。V2 分别恢复
  旧 outer 121/127 checkpoint 并通过原 conditional stationarity；随后在 outer 123/168 因原 lambda 上界
  搜索耗尽而失败。完成 outer 数为 122/167，runner 墙钟 `0.820071/0.868272 s`，均未 timeout。
- step 失败 conditional 为 2 calls/12 trials/11 rejected/1 accepted，最后 gradient `1.9922351e-6`；ramp
  为 1/10/10/0，未进入驻点资格。两条都没有首次同时满足外层四项停止条件，Stage 1 为 0/2，Stage 2
  `NOT_RUN`，eligible `NOT_EVALUATED`。
- fixed beta 仍缺失，truth 只覆盖 injected component，base total latent 与独立 calibration provenance 未闭合。
  这不是科学数据、validation、gate/test/RQ 或 C1--C3 支持。A05 仅可保留为 default-off development
  candidate；原独立复审未接受且当前结果拒绝默认迁移与扩大预算。

### A05 review-fix 证据边界

review-fix 证据见
[`t10_a05_review_fix_20260909T040832Z`](../doc/ie_sprint/evidence/t10_a05_review_fix_20260909T040832Z/VERIFICATION.md)。

- 独立复审 `CHANGES_REQUESTED` 的 F1/F2 稳定映射为 `T10-A05-RF1`、`T10-A05-RF2`。RF1 只修复异常诊断计数：
  pre-trial linearize fixture 对 V1/V2 都独立证明实际 trial/rejection 为 0；trial 内 exception fixture 证明
  linked inner counter 可能尚未提交，因此总数必须为 `INCOMPLETE`。实现只输出 confirmed completed lower
  bound，不重复求解、不改变正常 optimizer 行为；generic/stationarity 未执行且失败不导出 Values。
- 三态 `NOT_EXECUTED/COMPLETE/INCOMPLETE_EXCEPTION_DURING_ITERATE` 已写入 CSV 与 strict JSON，failure
  diagnostic schema 升为 v2。普通成功、V1 small-change/no-update、V2 recovery、call cap、lambda exhaustion、
  failure no-Values、runner serialization 与 config/solver/support/snapshot/scheduler producer identity 均回归通过。
- RF2 为本轮 runner、6 个实际 focused test executable、core、GTSAM 保存 realpath、SHA-256、build ID、记录时间、
  raw `ldd` 与去 ASLR normalized resolution；测试前后关键文件身份一致。它只证明本轮测试加载关系，A05 历史
  step/ramp 没有共享库绑定的缺口继续保留，当前哈希/事后 `ldd` 不作为历史运行时证明。
- V2 对初值最优、非零 residual、`J^T r=0` 的合法图仍先以 lambda-upper exhaustion 失败，external generic/
  stationarity 不运行、无 Values；这是已登记 A05 边界，不是本轮 correctness fix，也不解释 outer 123/168。
- 本轮 step/ramp、Stage 2、validation/sweep/gate lock/test/RQ/T11--T13 均 `NOT_RUN`。本地修复本身不构成 A05
  独立接受，也不使 C1--C3 升级。
- 指挥追加的 `T10-A05-RF1-N01` 已由默认/A02 V1 的最小二次 factor 与 linked branch log 确认：一次 wrapper
  调用实际为 2 trials、1 search rejection、0 accepted、1 normal no-update，原 wrapper 漏计 terminal
  small-change trial 却标 `COMPLETE`。本地修复只校正诊断：normal return 在 lambda upper 以下补计未写入
  inner counter 的 small-change trial；exhaustion 的末次 rejection 已在 inner counter 中，不重复增加；异常和
  未执行语义不变，聚合 `INCOMPLETE` 不会被覆盖。独立前后 probe、discovery 34/34、focused suites、runner
  contract 与本轮库身份稳定性通过，证据见
  [`t10_a05_counter_review_fix_20260909T060028Z`](../doc/ie_sprint/evidence/t10_a05_counter_review_fix_20260909T060028Z/VERIFICATION.md)。
  随后本轮指挥会话实际核对三个 manifest、三个当前源码快照、最终运行身份、discovery 34/34、双策略 post
  probe 和 final audit，并明确接受
  `REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE`，限定关闭 RF1/RF2/RF1-N01。该接受不是
  外部 `REVIEW.md`，指挥未重建或重跑 step/ramp；历史 A05 动态库身份缺口、V2 最优点 exhaustion 边界、
  Stage 1 0/2 与全部正式 `NOT_RUN` 保持，C1--C3 不升级。

### A06 新失败 checkpoint 诊断证据边界

A06 证据见
[`t10_a06_failed_checkpoint_diagnostic_20260909T065842Z`](../doc/ie_sprint/evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。

- A06 在运行前登记为 `DEVELOPMENT_V2_FAILED_CHECKPOINT_DIAGNOSTIC`，复制 A05 step/ramp 配置并保持逐字节一致；
  只更换隔离 run ID/output 并观察 outer 123/168。两条 estimator 各实际运行一次、exit 1、无 timeout，无 retry、
  A04 shadow recovery 或 solver 数值变化。
- step/ramp 的 authoritative terminal Values 均实际评价为 `NOT_STATIONARY`：最大 scaled gradient 分别为
  `1.9922351334744626e-6`、`1.512759625631882e-6`，原 tolerance 为 `1e-6`。主导坐标解析梯度与预声明有限
  差分分别 7/13、8/13 点一致；这排除“当前 Values 已驻点”并不支持主导坐标 mismatch，但不证明完整 Jacobian。
- step 终止调用的 11 个 trial、ramp 的 10 个 trial 均为 predicted decrease 正、actual decrease 负；model
  fidelity 因此为负，ramp 最后一个 predicted decrease 低于 linked 可分辨条件而不可用。方向范数均随 lambda
  缩小，最终按未改规则在 `lambda=1e5` 失败。浮点相减敏感判据只覆盖 step 2/11、ramp 7/10，不能单独解释全部。
- 历史 A05 的 86 个公共非计时字段在 122/167 个完成行及公共终态精确一致；当前 A06 runner/core/GTSAM 身份
  前后稳定，但不追溯为历史 A05 加载证明。两条仍无 Stage-2 cache、有效估计或科学结果。
- 完整 LM delta 与逐 factor old/tentative objective change 尚不可用，故 full-direction model-fidelity mismatch
  的底层归因写为 `UNKNOWN`。唯一建议是另行授权 actual tryLambda 内不参与决策、不重复求解的 observer，并在
  身份/公共 trace/终态不变、方向有限差分与逐 factor 求和一致时验收。本轮未实施。T10 保持 `IN_PROGRESS`，
  C1--C3 不升级。
- 本轮指挥会话随后以 `REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE` 接受上述限定范围；实际核验
  manifest `247/247`、源码 `4/4`、当前身份 `9/9`、discovery `34/34`、历史 `122/167 x 86` 与终止拒绝
  `11/10`，未重建、重跑 estimator 或改仓库，也不存在外部 REVIEW。`64*epsilon` 不构成完整浮点误差上界；
  A06 未新增此类 solve，但既有 `InspectFirstLinkedLmTry` 本身含 diagnostic solve。

### A07 actual LM 方向诊断证据边界

A07 证据见
[`t10_a07_exact_lm_direction_diagnostic_20260909T082434Z`](../doc/ie_sprint/evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)。

- A07 在真实运行前登记为 `DEVELOPMENT_EXACT_LM_DIRECTION_DIAGNOSTIC`；不修改 `/usr/local` GTSAM、不新增
  diagnostic solve、不复制 optimizer。小图验证 actual TRYDELTA 的 3 key/15 维完整、max-digits 解析、
  accepted delta/`retract` round-trip 和三点方向 FD 一致。
- step outer 123/ramp outer 168 各唯一一次、exit 1、无 timeout/retry；22/22 个 authoritative trial 均为
  24 key/120 维完整方向。预登记 6 个方向的 linked prediction 与事后 linear model 逐值一致，但 18 个
  nonlinear FD 全部不满足解析 `g^T u`；`lambda=1e4` 两方向还从解析下降变为实际一阶上升。
- selected trial 的逐 factor fsum/long-double difference 与 direct graph change 同号且均为目标上升，故不支持
  “只有总目标直接相减改变符号”。factor absolute changes/net change 为 `5.28e4--6.85e6`，UWB expression
  最大并与 pose prior/IMU 强对消；A06 单一主导坐标在同三步长仍通过。唯一缺失证据是逐 factor analytic
  directional derivative 与 FD，因而不能把某一 factor class 定为 bug，也不支持改变 LM 接受/停止规则。
- 历史 122/167 行的 86 个公共非计时字段及失败终态精确一致；A07 当时 Stage 1 仍 0/2，Stage 2/cache/
  有效估计均 `NOT_RUN`。本轮指挥会话随后限定接受 A07；T10 保持 `IN_PROGRESS`，C1--C3 不升级。

### A08 冻结方向逐 factor 定位与 Jacobian correctness fix 证据边界

A08 证据见
[`t10_a08_factor_jacobian_correctness_20260909T095029Z`](../doc/ie_sprint/evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/VERIFICATION.md)。

- 本轮指挥会话接受 `REVIEW_ACCEPTED_A07_BOUNDED_EXACT_DIRECTION_DIAGNOSTIC_SCOPE`；这是会话验收，不是
  外部 `REVIEW.md`，指挥未重建、重跑或修改仓库。A08 在修改和新运行前登记。
- 两次 pre-fix 捕获只覆盖 A07 冻结的 step outer 123/ramp outer 168、六个 actual direction 和原三个 FD
  步长。共 1314 个逐 factor 点保留；`h=1e-5` 时单个 Pose3 prior mismatch 解释图级 mismatch 的
  `99.9666%--100.1990%`，UWB/Combined IMU/velocity prior/bias prior 没有同量级不一致。
- linked GTSAM `PriorFactor<Pose3>` 保持 `-Local(x,prior)` residual，却提供 identity Jacobian；独立大位姿
  反例三步长均失败。paper-only factor 保持 residual、objective、noise、prior mean/weight，使用实际
  `-D_x Local(x,prior)` 后三步长均通过。legacy GraphBuilder 与 `/usr/local` GTSAM 未改，producer identity
  因 linearization 语义变化而版本化。
- 修复后冻结 factor 点 `1314/1314` 通过；图级仍为 `15/18`，三个 `h=1e-6` 残差原样保留，精确来源
  `UNKNOWN`。这里不把 `64*epsilon` 当完整浮点上界，也没有事后放宽判据。
- step/ramp post-fix 各唯一一次、120 秒上限、无 retry；原失败点分别接受 3/1 个下降更新，Stage 1 在
  outer 178/173 满足原四项 AND。两条随后实际运行 Stage 2，但都在未变的 50 轮上限返回
  `MAX_REFIT_ITERATIONS`，无 Stage-2 cache、有效估计或可用于 claim 的输出。
- 本轮指挥会话接受 `REVIEW_ACCEPTED_A08_PAPER_POSE_PRIOR_JACOBIAN_FIX_SCOPE`；该接受不是外部
  `REVIEW.md`，指挥未重建、重跑或修改仓库。正式 validation、sweep、gate lock、test、19-cell、RQ、
  T11--T13 均 `NOT_RUN`；C1--C3 不升级。

### A09 Stage 2 有界 budget 证据边界

A09 证据见
[`t10_a09_stage2_budget_diagnostic_20260909T111641Z`](../doc/ie_sprint/evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/VERIFICATION.md)。

- 运行前 amendment 只授权把 A08 两份隔离 development 配置的 `max_refit_iterations` 从 50 改为 200；
  结构化 preflight 证明除此之外没有有效数值参数变化。没有修改 solver 源码、默认配置、停止/接受规则、
  容差、LM policy 或依赖，也没有重建。
- step/ramp 各唯一一次、120 秒硬限、无 retry/observer。Stage 1 的 `178/173 x 87` 公共非计时字段、
  Stage 2 前 50 轮的 `50 x 22` 字段均与 A08 精确一致；Stage 2 分别在 outer 66/124 首次满足
  objective/step/KKT/navigation stationarity 四项 AND 并导出同一 refit graph/Values 的 estimate。
- 两个规范封存的 Stage-2 cache 均为 `CONVERGED/COMPLETE_WITH_SCORE_UNAVAILABLE`。step 为 24 段、
  17 short、0 boundary、1 group；ramp 为 27 段、21 short、1 boundary、3 groups。eligible group 为
  `0/1` 与 `0/3`，valid score 都是 0，因此两进程均以 `ONE_OR_MORE_GROUP_SCORES_UNAVAILABLE` exit 1。
- 运行前/step 后/ramp 后 runner、core、GTSAM 的 realpath/SHA-256/build ID 均一致且等于 A08 最终身份。
  A09 说明原 50 轮 Stage-2 budget 对这两个 development 输入不足；它不提供有效评分、accepted decision、
  final trajectory 或正式 validation，C1--C3 不升级。A09 状态为
  `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`（本轮指挥会话，不是外部 REVIEW.md）。

### A10 完整合成与 development pilot 证据边界

本轮用户/指挥会话接受 A09 为 `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`，不是外部 REVIEW.md。
T10 保持 IN_PROGRESS；A08 图级 FD 15/18、三个最小步长限制继续保留。A10不是held-out证据。

| 事实 | 实现/证据 | claim 限制 |
|---|---|---|
| 同一解析运动生成UWB/IMU，完整total dynamic latent truth，PCG64实际随机源，raw/truth分离 | [generator](../tools/paper/generate_synthetic_input.py)、[协议](../doc/ie_sprint/T10_A10_PROTOCOL.md)、[数值与随机验证](../doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/GENERATOR_CHECKS.json) | 只支持已测试合成输入工程；已知beta/noise/geometry是合成假设，不弥补真实标定/许可缺口；旧bag未知误差不归零 |
| 3support × 3scenario/seed10101实际9次进程，全部首次conditional LM 50-call后未达驻点 | [完整证据](../doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/VERIFICATION.md)、[逐次统计](../doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/PILOT_SUMMARY.csv) | Stage2/score/final NOT_RUN，0新Stage2 cache，eligible NOT_EVALUATED；无参数赢家、无eta/s/gamma或risk证据，C2不升级 |
| raw ledger/common init/actual maps和库hash/truth隔离9/9通过；反例5/5 | [审计](../doc/ie_sprint/evidence/t10_a10_synthetic_pilot_20260909T115705Z/PILOT_AUDIT.json) | 支持可追溯development失败；不是跨数据正式性能/科学验证；首轮checker失败和配置字段拼写缺口保留 |
| 独立split、四gate各12点、selection/metrics、B_total14400s/25%reserve和scheduler改造方案 | [准入提案](../doc/ie_sprint/T10_A10_VALIDATION_ADMISSION.md)、[split](../doc/ie_sprint/T10_A10_SPLIT_PROPOSAL.json) | PROPOSED_NOT_ADMITTED；新schema/role接口尚未实现，validation/test/gate lock NOT_RUN；development不得升格 |

C1–C3原状态不变；本轮没有论文数字、图表、正式风险/轨迹指标或成功AUTO科学闭环。

### A11 首次conditional LM有限定位证据边界

本轮指挥接受 `REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE`，
仅A10限定交付验收，不是外部REVIEW.md、estimator成功或validation准入。T10仍IN_PROGRESS。

| 事实 | 证据 | claim边界 |
|---|---|---|
| 只用冻结P1 LOS/step/ramp seed10101，各一次/120s；无输入、truth、noise、初始化或support变更 | [协议](../doc/ie_sprint/T10_A11_PROTOCOL.md)、[实际命令/结果](../doc/ie_sprint/evidence/t10_a11_first_block_20260909T122715Z/VERIFICATION.md) | development诊断；P2/P3、held-out和scheduler未运行/实现 |
| call50的A10已保存数值与共同准备身份复现，实际accepted delta完整123 keys/615维且原生retract匹配 | [审计](../doc/ie_sprint/evidence/t10_a11_first_block_20260909T122715Z/AUDIT.json) | A10未保存完整call50 Values，不能声称历史完整状态逐元素比较 |
| 六个实际方向原步长/判据图级18/18、factor6678/6678，最大梯度坐标9/9通过 | [汇总](../doc/ie_sprint/evidence/t10_a11_first_block_20260909T122715Z/SUMMARY.csv) | 仅所检查方向支持一致性；不替代完整方向的单坐标检查，不消除A08历史15/18/UNKNOWN限制 |
| 全部获准同optimizer续至200次，600次接受更新与记录目标下降，但仍未达原驻点 | [进展](../doc/ie_sprint/evidence/t10_a11_first_block_20260909T122715Z/PROGRESS_SUMMARY.csv) | 振荡缓慢接近驻点；不声称必然收敛或默认200有效。Stage1 completed outer/chain=0，Stage2/gate/final NOT_RUN，无eta/s/gamma证据 |
| 原输入/truth/配置/合同hash保持、file-open trace无truth、实际加载库身份闭合；92/92工程fixture通过 | [交付证据](../doc/ie_sprint/evidence/t10_a11_first_block_20260909T122715Z/VERIFICATION.md) | 支持限定可追溯诊断，不是科学收益或validation资格；首轮build语法失败与三次estimator失败保留 |

A11不提出无依据的Jacobian修复。唯一最小后续提案是封存P1 step call50的静态线性系统尺度/条件数与
阻尼比例审计，不调用iterate；本轮NOT_RUN。C1–C3原状态不变，A10 validation提案不升格为准入。

## 0912 Windowed FDE v4：工程证据与负 Walk1 gate

[协议](../doc/ie_sprint/WINDOWED_FDE_E2E_PROTOCOL.md)与
[结果](../doc/ie_0911/WINDOWED_FDE_E2E_RESULT.md)支持以下有限工程事实：显式 v4 provider 使用 FullGraph
covariance 子块、chain-local Bonferroni、负 GLS 和确定性 exact-union merge；dilution/clean/merge/
multiplicity/correlated/sign/ambiguity/cache 边界随完整 384/384 测试通过。v2/v3 默认及 replay 保留，四个
Stage2/Rc backend 相对实施基线无差异。

锁定 Walk1 clean 的 1010 windows 未产生支持；injected 在 30 个 planned added-component IDs 上
TP=0、FP=0、FN=30，目标 link 最大 adjusted ratio 为 0.674515，故 detector gate 失败。六方法 E2E、
Stage2 接收、local sigma/LCB 与定位收益均 NOT_RUN/NOT_EVALUABLE。该单次 clean 不是总体误报保证，负 gate
也不授权调参或扩大矩阵；不是 held-out/正式论文数字。T10=C2-C、T11=C、C1--C3 均不升级。


## 0911 第一步固定部分补偿：工程证据，非 claim 升级

用户授权仅 `lcb_partial` / 同 LCB 集合 `lcb_fixed_full` 的固定 offset amendment，旧联合 live-C 方法和默认配置保留。
[STEP1_RESULT](../doc/ie_0911/STEP1_RESULT.md) 记录实际 catkin 构建、CTest 26/26（GTest 161 项）、
24/24 六输入四方法 prepare-only、实际 certified production 非零补偿与独立 16 条最终残差重算。
局部 sigma、Stage2 幅值和冻结补偿分列；不把固定补偿称为最终 bias 后验或已校准安全概率。

真实 Walk1 起始后 [8,11]s robust Cauchy 有 development aligned 评价，但自动 Stage1 原 50 outer 上限失败，
Stage2 和候选 final 未运行；不存在真实 LCB 精度收益证据。标定/GT 点/许可限制、T10=C2-C、T11=C 保留，
C1/C2/C3 均不升级。第二步完整矩阵、正式指标/held-out、论文数字与 release NOT_RUN。

## 0911 frozen-algorithm real-data injection experiment

DONE / BENEFIT_EVIDENCE_INSUFFICIENT。授权与结果见 `doc/ie_sprint/NLOS_INJECTION_PROTOCOL.md`、`doc/ie_0911/NLOS_INJECTION_EXPERIMENT_RESULT.md`。HEAD46d37f6算法和科学参数hash未变。6 clean、4 admitted injected完成；2 injected保留clean Cauchy失败而跳过。72方法条目中57成功、3失败、12跳过。用户暂停中断另列，续跑未重试算法失败或已完成项。

全部10个场景temporal support为空，4 injected新增偏置检测TP=0；LCB/suppression轨迹相同来自空support行为，没有实际恢复补偿，不能作为检测后LCB优越性证据。唯一有效normal/low配对Walk3收益均0，未观察到低冗余放大收益；不推断统计显著性。完整CTest30/30、指标工程测试7/7、四对实际初值/计划/噪声和共享上游检查通过仅支持工程正确性。Range provenance缺口仍UNAVAILABLE。T10=C2-C、T11=C、C1–C3不升级。

## 0912 PL conditional RAIM/FDE：preflight 工程证据与停止裁决

[协议](../doc/ie_sprint/PL_CONDITIONAL_RAIM_PROTOCOL.md)与
[结果](../doc/ie_0911/PL_CONDITIONAL_RAIM_PREFLIGHT.md)支持限定工程事实：锁定 PL conditional equations、
detect-before-commit 的 15 维 prior、unique LOAO、healthy-subset commit、PL gap reinitialize、连续
support 与 truth-blind shadow artifacts 已实现；独立 fixture 和 8/8 定向测试通过。该代码仍是
preflight-only，不是 production provider。

锁定 Walk1 clean retained support 为 0；injected 的 30/30 planned affected IDs 全部被检测，但目标
affected group alarm 为 0，最大统计量 `4.86052305024` 低于对应门限 `30.8561899404`，故正式裁决
`PL_CONDITIONAL_PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM`。target unique isolation/persistent overlap
后续门、production/cache、Stage2、六方法和定位指标均 NOT_RUN/NOT_EVALUABLE。本结果不支持检测召回、
完整 ARAIM、certified integrity 或恢复收益；不产生论文数字，不升级 T10=C2-C、T11=C 或 C1--C3。

## 0912 PL threshold / persistent-signal：root-cause diagnostic

[协议](../doc/ie_sprint/PL_THRESHOLD_SIGNAL_AUDIT_PROTOCOL.md)与
[结果](../doc/ie_0911/PL_THRESHOLD_SIGNAL_AUDIT.md)只支持一个 locked Walk1 diagnostic 事实：保持原
group T/DoF 时，从 `P_FA=1e-5` 放宽至 0.10 仍是 clean 0/224、affected 0/30；最大 T 的 DoF=5
tail probability 为 0.4331。相同 30 identities 上，target conditional z injected mean=1.0926、
`Z_sum=5.9846`，相对 clean 的 `Z_delta=6.6220`，且 healthy 无同等级正向 paired shift。因此本数据
支持“single-epoch group statistic 与 persistent per-anchor candidate task 不匹配”的诊断裁决。

该证据不是新 detector、阈值标定、总体误报/召回保证或 localization benefit；没有运行 Stage2/final，
也不能把 30 帧描述统计当作已验证 sequential test。CUSUM/GLR 仅是需另立任务的可能方向；C1--C3、
T10=C2-C 与 T11=C 均不升级。

## 0912 PL bidirectional CUSUM support admission

[协议](../doc/ie_0911/PL_BIDIRECTIONAL_CUSUM_SUPPORT_PROTOCOL.md)与
[结果](../doc/ie_0911/PL_BIDIRECTIONAL_CUSUM_SUPPORT_RESULT.md)支持一个限定 development admission：
既有 forward detector 参数、identity、alarm 与 onset 不变；clean-only calibrated backward non-causal
closure 和 exact link/obs-ID intersection 在 locked Walk1 frozen 与 always-commit dynamic shadow 中均得到
TP/FP/FN=30/0/0、healthy segment=0。Backward held-out clean 为 0 alarm/0 segment；dynamic
CONTROL/SHADOW 的 commit、measurement、state、trajectory 和 conditional artifacts 字节一致，最大 state
差为 0。forward first alarm 仍为 affected #15。

这只支持 `BIDIRECTIONAL_CUSUM_SUPPORT_PASS_FOR_PRODUCTION_ADMISSION`，不等于 production provider 已接入，
也不是 causal online endpoint、formal integrity/general-dataset detector guarantee 或 localization benefit。
Stage2/Rc/recovery/final/ATE/RMSE 均未运行，C1--C3、T10=C2-C 与 T11=C 不升级。

## 0912 PL bidirectional production E2E controlled evidence

`E2E_FULL_SYSTEM_PASS_DEVELOPMENT` is supported only for the locked controlled
SFUISE Walk1 `+0.5 m` persistent-NLOS experiment. The production provider finds
the admitted 30-observation support without truth access; Stage2 converges at
`c_hat=0.47134578518036691 m`; primary fixed compensation reduces exact-obs-id
paired range RMSE from `0.5 m` to `0.02865421481963315 m`; and, on the same 229
GT samples, primary aligned ATE RMSE is `0.16314948912901142 m` versus
`0.18163686608072543 m` for suppressing the same observations, with p95 also
lower. See
[`PL_CUSUM_E2E_INTEGRATION_ACCURACY_RESULT.md`](../doc/ie_0911/PL_CUSUM_E2E_INTEGRATION_ACCURACY_RESULT.md)
and evidence root
`/home/mint/ws_fusion_uwb/res/pl_cusum_e2e_integration_accuracy_20260912T041820Z`.

This does not upgrade C2 to a general recovery-superiority claim: the non-gating
six-input diagnostic has 0 better, 1 worse, 1 tied, and 4 unavailable run units.
T10 remains `C2-C`; formal final multidataset evidence remains required.

## 0912 SFUISE ToA baseline adapter

The development comparison in [SFUISE_BASELINE.md](../experiments/SFUISE_BASELINE.md) establishes an
adapter and reproducible evaluation path, rather than a paper claim. Unmodified SFUISE commit `75bf5a32`
ran successfully in absolute-ToA mode on ISAS Walk1/2/3 without GT playback or detector/recovery input.
Its adapter trajectories and four sealed method trajectories were evaluated by the same evaluator,
per-sequence interval, nearest-0.02 s association, tracker/body assumption and scale-fixed SE3 alignment.
SFUISE aligned ATE RMSE was 0.109076557/0.075431056/0.079883493 m; Walk2 Robust FGO remains an explicit
estimation failure and NA. The body/tracker extrinsic is still an assumed identity and the comparison is
not held-out, so these values do not upgrade C1--C3, T10 or T11.
