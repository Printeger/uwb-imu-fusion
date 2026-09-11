# Stage2 navigation stationarity forensic and correctness repair

最终裁决：**`RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX`**。

定位结果单独裁决：**`LOCALIZATION_BENEFIT_OBSERVED`**。在这个单一、锁定的
oracle-support development 场景中，LCB 的 RMSE 比 suppression 低
`0.011550630 m`（`6.3592%`）。这是 detector 后端 sanity check，不是 detector
端到端成绩，也不改变既有 C1–C3 或 T10/T11 裁决。

## 1. 基线、范围与身份

实际实验基线 Git HEAD 是 `3db4f3175fc647ac5a24e3de771e7e41ceddf4e7`。本次
correctness repair 在该基线上实施。实验基线仅为
[`ORACLE_SUPPORT_BACKEND_RESULT.md`](ORACLE_SUPPORT_BACKEND_RESULT.md) 及其锁定输入：
SFUISE Walk1 normal redundancy、seed 911、link `27956:20276`、闭区间
`[1664959678.3077347, 1664959686.3077347]`、constant `+0.5 m` injection、30 个
planned oracle-support observations。实际进入 graph 的最后一个 support 时间是
`1664959686.2890861`。

正式修复后运行位于：

```text
/home/mint/ws_fusion_uwb/res/stage2_stationarity_repair_20260911_01/
  forensics_sealed_v2/
  oracle_rerun_normal_delivery/
  final_evaluation_delivery_v2/
  engineering/
```

权威 batch manifest SHA-256 为
`e8132eb135f74e6411f014198129e1c2013eaa7205b81a08abe210cbe01f7fec`；runner
binary SHA-256 为
`62b63436625811fd66e3e28c4a30936916033355450975c821841a1a7b062c8b`，动态 core library
SHA-256 为 `ceedc6e2c69b3e7af06ed6ddca7b8ca4d1c62e1f3e76e90ccc76b8a9d7ba7ef2`；Stage2
cache ID 为
`t09stage2cache-sha256:d970dd4af2766d58f79f1be965b847fbbffacd130eaea5aac7eb53f6fbfdbaac`。
producer ABI identity 为
`t09abi-sha256:928c31396e5c8af0551590d5d403d613286db51fbe6e34909e0db4ed16ce1eda`。
7/7 preregistered cells 到达 terminal，0 run failures，未做算法重试。

## 2. A1：三个解的同定义 stationarity audit

三者都使用 production `AuditNavigationStationarity` 的同一 graph linearization、物理尺度、
`1e-6` 容差和 binary64 roundoff accounting。

| Values | pose rot | pose trans | velocity | accel bias | gyro bias | max scaled | roundoff |
|---|---:|---:|---:|---:|---:|---:|---:|
| clean raw-reference final | 0.013063083 | 0.135282016 | 0.004034965 | 0.000884018 | 0.000995635 | 0.135282016 | 2.24013e-11 |
| injected raw-reference final | 0.245504042 | 3.952021070 | 0.664544417 | 0.016524672 | 0.009443099 | 3.952021070 | 2.23475e-11 |
| pre-fix oracle Stage2 terminal | 0.005440109 | 0.075024386 | 0.001325904 | 0.000303891 | 0.000357754 | 0.075024386 | 2.23557e-11 |

由此得到辅助裁决 **`STATIONARITY_CONTRACT_INCONSISTENT_WITH_BASE_SOLVER`**：两个由既有
generic GTSAM success 判据接受的 raw-reference 解本身也远高于 `1e-6`。这说明 `0.075` 不能单凭
绝对数值被解释为 Stage2 特有的模型 coupling；也说明 Stage2 明确要求的严格 stationarity 比基础
raw solver 的 success contract 更强。本任务仍完整保留 Stage2 的 `1e-6`，没有用这一观察放宽它。

## 3. A2：dominant coordinate

pre-fix Stage2 terminal 的最大项是：

| 字段 | 值 |
|---|---|
| key / coordinate | `x186`, local coordinate `4` |
| symbol / category | `X` / pose translation |
| native gradient | `+0.075024385862432652` objective/m |
| physical scale | `1 m` |
| scaled gradient | `0.075024385862432652` |
| absolute factor-gradient sum | `9.5174948299773163` |
| coordinate roundoff allowance | `1.1039922973978534e-11` |

因此 `0.075` 不是 rotation、velocity 或 IMU bias 项，也不是 roundoff 造成的假阳性。

## 4. A3：有限差分 derivative audit

forensic 入口直接复用 `nlos_solver_utils` 的 local-coordinate central-difference 实现。dominant
`x186[4]` 的 analytic gradient 为 `0.075024385862432652`：

| perturbation | central difference | absolute mismatch | pass |
|---:|---:|---:|---|
| 1e-2 | 0.0750057753152 | 1.86105e-5 | yes |
| 1e-3 | 0.0750241997451 | 1.86117e-7 | yes |
| 1e-4 | 0.0750243840741 | 1.78829e-9 | yes |
| 3e-5 | 0.0750243856373 | 2.25098e-10 | yes |
| 3e-7 | 0.0750244074273 | 2.15649e-8 | yes |

dominant 的 10/10 尺度通过。固定 seed 911 另取 8 个 navigation coordinates；其中 5 组
10/10、3 组 9/10 通过，三项失败都只出现在最小 perturbation、解析梯度已接近机器差分消去尺度时，
其较大尺度均一致。结论是 analytic gradient 与 objective 有限差分一致，
**`STATIONARITY_GRADIENT_IMPLEMENTATION_BUG` 被拒绝**；没有走 D1。

## 5. B：最后 15 个原始 outer 的 C 前后行为

完整 50 行在 `forensics_sealed_v2/stage2_original_50.csv`。最后 15 行（outer 36–50）完全相同：

| outer | C before | nav grad before C | LM accepted / inner | C after | delta C | nav grad after C | C KKT | obj/step/KKT/nav |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 36 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 37 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 38 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 39 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 40 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 41 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 42 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 43 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 44 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 45 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 46 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 47 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 48 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 49 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |
| 50 | 0.4687545133 | 0.0750243859 | 0 / 3 | 0.4687545133 | 0 | 0.0750243859 | 1.83e-14 | 1/1/1/0 |

conditional objective 与 joint objective 都是 `128.79193030992778`，lambda 是 `0.01`，full
scaled step 是 `2.61483e-17`。因此 fixed-C navigation 子问题在 C update **之前**已经保留真实
`0.075` 梯度；C update 为零且没有重新引入梯度。问题不符合 D3 前提，
**`BLOCK_COORDINATE_COUPLING_CONFIRMED` 被拒绝**。

## 6. C：diagnostic-only 200 outer

保持全部接受标准不变的 nonpublishing continuation 给出：

| outer | objective | C | delta C | nav gradient | scaled step | C KKT |
|---:|---:|---:|---:|---:|---:|---:|
| 50 | 128.791930310 | 0.4687545133 | 0 | 0.0750243859 | 2.61e-17 | 1.83e-14 |
| 100 | 128.791930310 | 0.4687545133 | 0 | 0.0750243859 | 2.61e-17 | 1.83e-14 |
| 200 | 128.791930310 | 0.4687545133 | 0 | 0.0750243859 | 2.61e-17 | 1.83e-14 |

观测结果为 **`BLOCK_COORDINATE_COUPLING_PLATEAU`**，但根因位于上游 conditional solver，
不是 old-C stationary / new-C nonstationary 的 coupling 模式。单纯增加 outer budget 没有作用。

## 7. 唯一根因与修复

唯一根因裁决为 **D2 / `CONDITIONAL_LM_FIXED_C_NUMERICAL_CONVERGENCE_FAILURE`**。旧 generic
LM 把固定-C 状态判为 converged，即使 production audit 仍为 `0.075`；其后每个 outer 都以 0 accepted
updates 重复同一状态。独立 V2 probe 在同一 terminal fixed-C graph 上得到：100 calls 时
`7.13995e-4`，200 时 `9.18115e-5`，340 时 `9.65420e-7` 并通过原容差。另一个 lambda-exhausted
checkpoint 在同 graph、同 Values、默认 lambda 重建后 6 calls 达到 `7.40375e-7`。

最小修复保留既有 alternating trajectory。只有上一 outer 已同时满足 objective、scaled-step 和 C-KKT，
唯独 navigation stationarity 失败时，下一个 fixed-C conditional solve 才启用已有
`GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2`。若一个 V2 block 因
stationarity 未达、lambda search exhaustion 或其固有 call block 上限结束，则在完全相同 graph 和最后
accepted Values 上重建同一 V2；checkpoint 只作为起点，从不作为成功结果。每次检查：

- graph/Values keys 精确匹配、Values 和 objective 有限；
- objective 不超过现有 binary64 allowance；
- success 同时要求 V2 generic convergence 和原 navigation stationarity；
- restart 受原 `max_refit_iterations` 约束，所有普通及严格 conditional calls 共同受
  `max_refit_iterations * lm_max_iterations` 总预算约束；
- 不接受 `PAPER_STAGE2_INEXACT_HANDOFF_V1`。

同一严格 terminal qualification 也用于无 live-C/fixed-offset 的 production refit，避免 final
recovery 在相同 numerical stop 上失败。Stage2 objective、factor、measurement、noise、初始化、LM
参数、linear solver 和全部阈值均未改变。新 policy/trigger/recovery budget 已进入 cache identity
`terminal-stationarity-recovery-v3`。

D3 joint active-set polish **未实现**，因为其必要证据条件为假；因此没有新增 solver mechanism、C prior、
active-set boundary rule 或 `JOINT_ACTIVE_SET_POLISH_*` success reason。

## 8. 实现与测试

代码改动包括：stationarity audit 导出 dominant coordinate 和 factorwise/roundoff 证据；公开复用既有
finite-difference coordinate diagnostic；Stage2 forensic 入口；before/after-C trace；checked-LM fixed
checkpoint recovery；production cache identity 和机器 evaluator。FDE、Rc、sigma、LCB、final factor 与
GT evaluator 源码未改。

已有 coupled live-C toy `JointSmallGraphConvergesWithOneSharedLiveCAndNoPrior` 验证正 interior C、联合
navigation stationarity、C gradient/KKT 和无 C prior；`ZeroAmplitudeStopsAtBoundaryWithValidKkt` 验证
C=0 boundary KKT 且无负值；factor metadata 检查验证原 joint graph 每条候选恰有一个 X/C physical
factor、相同 measurement/noise/keys。新增
`FixedCheckpointRecoveryKeepsObjectiveAndReachesStationarity` 验证单个 V2 block 失败后，同 graph/Values
恢复能使 objective 下降并达到原 stationarity，且无预算时 fail closed。由于没有选择 D3，这些是 D2
及既有 Stage2 objective identity 测试，不把它们描述成 joint-polish 测试。

最终工程命令：

```text
catkin build uwb_imu_fgo --no-deps --make-args tests -j4
make -C /home/mint/ws_fusion_uwb/build/uwb_imu_fgo -j4 uwb_imu_fgo_paper_runner
catkin run_tests uwb_imu_fgo
python3 tools/paper/run_oracle_support_backend.py \
  --output /home/mint/ws_fusion_uwb/res/stage2_stationarity_repair_20260911_01/oracle_rerun_normal_delivery \
  --regime normal
python3 tools/paper/report_stage2_stationarity_repair.py \
  --run .../oracle_rerun_normal_delivery \
  --forensics .../forensics_sealed_v2 \
  --output .../final_evaluation_delivery_v2
```

全部最终命令 exit 0；CTest/GTest 汇总为 **370 tests, 0 errors, 0 failures, 0 skipped**。首次在扩展
`RefitIteration` 结构后直接运行陈旧 test binary 曾 SIGSEGV；重建测试目标后消失，说明是本地 stale
test-object ABI，不是算法失败。后续完整 tests build 和两次完整 `catkin run_tests` 均通过。构建只保留
仓库既有的三个 devel symlink warning 与两个原有 dangling-else 编译 warning。

## 9. 锁定 oracle support 与 truth 隔离

`oracle_support.json` 仍只含 segment/link/interval/30 obs IDs 和
`support_source=oracle_injection_interval`。30/30 均映射到 valid、planned、目标 link，没有其他 link。
estimator 在 bwrap 内看不到 injection truth/GT；strace 记录 8 次允许的 oracle support 打开，0 次
`nlos_injection_truth.json` 或 GT 打开。support/effective config 中没有 amplitude、`c_true`、bias truth
或 GT 字段。`0.5 m` 仅在 estimator 全部结束后由独立 evaluator 读取。

## 10. 修复后 Stage2、Rc 与 correction

Stage2 未走 `SUCCESS_EMPTY`，实际结果如下：

| 字段 | 值 |
|---|---:|
| status / reason | `CONVERGED` / `ALL_JOINT_STOP_CONDITIONS_SATISFIED` |
| outer conditional solves / LM optimizer instances | 42 / 45 |
| LM iterate calls / inner trials | 566 / 1161 |
| objective initial → final | 2029099755.925515 → 128.788586274973 |
| final relative objective change | 4.63439e-15 |
| final scaled state step | 2.37663e-7 |
| final navigation gradient | 7.10213e-7 |
| tolerance + roundoff | 1e-6 + 2.23559e-11 |
| C gradient / KKT | 7.61484e-14 / 7.61484e-14 |
| objective / step / KKT / stationarity | true / true / true / true |

修复路径实际使用一次 fixed-checkpoint recovery（outer 14，3 restarts）；最终没有 inexact handoff。
Stage2 独立估计 `c_hat=0.4713457858280814 m`。evaluation 阶段才读取 `c_true=0.5 m`，得到 signed
error `-0.0286542141719186 m`、absolute error `0.0286542141719186 m`。

production Rc 结果为 `OK`，`R_rank=1`、frozen rank certified，
`lambda_min(N)=552.5750231410673 m^-2`、condition-1 `1185.059348580189`、rcond-1
`8.438395943613227e-4`，并给出有限 `sigma_c=0.07507370718106174 m`。该 sigma 保持原 local
diagnostic 语义，不声称校准置信保证。

固定 `kappa=2`：

| correction | 值 | vs 0.5 m |
|---|---:|---|
| full `delta=c_hat` | 0.4713457858 m | under-correction；不越界 |
| LCB `max(0,c_hat-2 sigma)` | 0.3211983715 m | positive under-correction；不越界 |
| LCB residual injected bias | 0.1788016285 m | — |

## 11. Final optimization 与 factor delivery

四种方法使用同一 oracle support。全部 final 从 recovery path 获得有效 estimate，covariance
`AVAILABLE`，fallback count 均为 0。

| method | run status | optimizer calls | objective initial → final | final nav grad | factors | candidate physical factors |
|---|---|---:|---:|---:|---:|---|
| suppress_all | `ZERO_ACCEPTED` | 5 | 127.996369046 → 126.755127027 | 9.46763e-7 | 1275 | 30 suppressed / 0 retained |
| structured_debias | `OK` | 2 | 128.788586275 → 128.788586275 | 5.01553e-7 | 1305 | 30 raw with live C |
| lcb_fixed_full | `OK` | 2 | 128.788586275 → 128.788586275 | 6.38807e-7 | 1305 | 30 raw with fixed 0.471345786 m offset |
| lcb_partial | `OK` | 5 | 135.017279913 → 130.992148547 | 8.61743e-7 | 1305 | 30 raw with fixed 0.321198371 m offset |

`ZERO_ACCEPTED` 是 suppress 语义状态，并非 solver failure；其 final summary 明确
`valid_estimate=true`、solver `CONVERGED`。两个 fixed 方法的 `fixed_compensations.csv` 均为
`decision_use=1`、`final_use=1`，factor audit 各有 30 条
`ACCEPTED_CANDIDATE_RAW_WITH_FIXED_OFFSET`，证明 correction 真正进入 physical UWB factors。

## 12. 冻结 evaluator 的定位结果

所有轨迹 coverage 均为 1.0。

| method | RMSE m | P95 m | horizontal RMSE m | vertical RMSE m |
|---|---:|---:|---:|---:|
| clean raw | 0.163853268 | 0.298978070 | 0.123735494 | 0.107412388 |
| injected raw | 0.198923089 | 0.343576674 | 0.132749869 | 0.148148126 |
| robust Cauchy | 0.196904359 | 0.356312042 | 0.143714159 | 0.134601512 |
| suppress_all | 0.181636878 | 0.346468328 | 0.129107385 | 0.127762431 |
| structured_debias | 0.163149485 | 0.290291838 | 0.119305148 | 0.111283585 |
| lcb_fixed_full | 0.163149489 | 0.290291831 | 0.119305154 | 0.111283584 |
| lcb_partial | 0.170086249 | 0.292003454 | 0.123981748 | 0.116438216 |

RMSE 配对变化（candidate minus reference；负数表示 candidate 更好）：

| comparison | absolute change m | improvement |
|---|---:|---:|
| LCB vs suppress | -0.011550630 | +6.3592% |
| full vs suppress | -0.018487389 | +10.1782% |
| structured vs suppress | -0.018487393 | +10.1782% |
| LCB vs full | +0.006936759 | -4.2518% |

所以 LCB 在本场景优于 outright suppression，但因保守欠补偿而比 full correction 差。full 与 live-C
structured 的轨迹几乎一致；这与 `c_hat` 接近 0.5 m、且 anchor redundancy 足以让 suppression 仍保持
有限精度的观察一致。该单场景结果不支持泛化的“recovery 总是优于 suppression”。

## 13. 最终科学裁决

Stage2 现在真实满足原始 joint stationarity/KKT，随后 production `c_hat -> Rc -> sigma -> LCB -> final`
全部执行成功，因此二选一裁决是：

```text
RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX
```

RMSE 单独裁决是：

```text
LOCALIZATION_BENEFIT_OBSERVED
```

本任务未修改或新增 detector/FDE/grouping/CUSUM/solution-separation/RAIM；未改 injection、bias model、
Rc、sigma、LCB、final semantics、evaluator 或任何 tolerance；未运行 low-redundancy 或大矩阵；没有创建
新 phase 或 roadmap。

权威机器结果为
`final_evaluation_delivery_v2/stage2_repair_result.json` 和
`localization_metrics.csv`。`oracle_rerun_normal_01` 保留了首次 repair 后 suppress/full/LCB final
仍因旧 no-C qualification 失败的结果；`normal_02`、`normal_final` 是后续开发运行。最后的
`oracle_rerun_normal_delivery` 才是本文引用的权威运行，没有用早期结果替换失败记录。
