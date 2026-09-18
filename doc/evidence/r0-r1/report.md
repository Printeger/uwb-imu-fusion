# UWB–IMU PL V2 R0/R1 开发实施报告

本轮结论为 **`DEVELOPMENT_ONLY / PARTIALLY_IMPLEMENTED`**。R1 的冻结窗口基础
数值共享、生产 analytic sensitivity、风险预算审计、计数器和诊断已实现并完成
开发级回归。R0 已建立独立 raw-stream 场景、正常提交、连续拒绝、长 pending、
真实边缘化和 128-action 压力基线，并修复两个实际缺陷；但 C/D/E 的成功排除和
G 的 bridge timeout 尚未在完整 raw-stream pipeline 中实现。成熟窗口仍按现有
历史假设/完整覆盖合同 fail closed。本报告不宣称 Gate D PASS、formal eligible
或正式风险闭合。

机器摘要见 [summary.json](summary.json)。原始开发产物保存在 ignored 目录
`results/r0-r1/final-current/`；旧 evidence 和失败日志均未覆盖。

## 1. 身份与固定条件

- 基线 HEAD：`e6ded3069bbc4cd5cf98f14257e96f61066685b8`。
- 最终工作区：同一 HEAD，`git_dirty=true`；用户原有未跟踪文件
  `doc/UWB_IMU_PL_V2_NEXT_ROADMAP.md` 保持未覆盖。
- 编译输入 patch SHA-256（不含该用户文件、本报告和 summary）：
  `cf805ddb227d71f7d398bda6df29ea3668a05a4555a3b216eef48916473253eb`。
- 最终二进制 SHA-256：`r0_r1_development` 为
  `781b2f9d923583376827c8765bb4339485fb2f9ab015667eff3a655bfe8209a2`；
  `realtime_performance_benchmark` 为
  `c2814d9544145a7031506ff18969aa6b0fdccdec4fa1e033734dd2cba2c9cf05`。
- 场景清单 SHA-256：
  `42f60c01b562473bf91aa2bebfe889eef52f194c899851d9d1f71f27835316a1`。
- Release，GCC 9.4.0，GTSAM 4.2a5，Eigen 3.3.7，Linux WSL2；CPU 为
  Intel Core Ultra 5 125H。
- raw-stream：seed `20260901`，IMU 200 Hz，UWB 20 Hz，8-anchor（低冗余用例
  为 4-anchor），candidate workers=4，数值库线程=1，CPU affinity `0-3`。
- research base config 未修改。正向 development runner 仅在独立解析后的配置中
  关闭 UWB ramp，并记录 resolved config；所有 scenario 强制
  `formal_eligible=false`。A 的 resolved config hash 为 `76c991b0319cf2cf`，
  128-action smoke 为 `92405b09ab4accc5`。

## 2. 代码审计：已确认事实与假设验证

修改前的静态审计确认，同一个有效冻结窗口的基础路径执行
SVD×2、LLT×2、QR×1 和三次基础状态求解：`finalizeIntegrityWindow()` 的 SVD、
detector 的 QR、evidence 的 LLT/solve，以及 rank-update base 的 SVD/LLT/solve。
`ImuFaultSubspaceBuilder::build()` 也会对六轴逐轴做正负扰动，每 interval 固定
12 次 raw-IMU 重新预积分。

以下线索经最小重现后得到区分后的结论：

- 原 benchmark 真初速为 `(0.45, 0.45, 0) m/s`，初始化速度为零。将
  `relinearize_skip=200` 单独改成 1 后，0.25 全窗口步长拒绝仍然复现；二者是
  贡献因素而不是唯一根因。A/B 因此分别保留真值一致和零初速回归。
- 基线第 3 个窗口出现 `RISK_BUDGET_INVALID` 时，逐项二进制和只比上限多约
  `1.58e-18`，来源是等分 allocation 的 round-to-nearest 与逐项累加，而非真实
  风险分配超支。修复后的同类历元 margin 为正，例如 A attempt 3 为
  `1.8065624578158124e-21`；所有最终运行均没有 `RISK_BUDGET_INVALID`。
- 默认最新历元只有一个时间点，二维 affine-ramp 的 slope 列为零；按既有满秩
  合同该 mode 必然 unmonitorable。F-ramp 保留此默认行为，5/5 历元均没有有限
  PL，证明仍然 fail closed。
- risk 修复后曾暴露另一个实际阻塞：成功候选的 commit plan 会为未实际加入的
  generic bridge 组记录 health/replacement relation，使 backend 更新后
  `FactorLedger::setHealth()` 因未知 group 抛出。现在只为
  `groups_to_add` 中的组记录关系，避免“更新后异常”。事务零/一次更新不变量由
  Integrity V2 回归覆盖。
- 成熟窗口中，压缩到边界先验的 epoch-independent UWB occurrence 已没有显式
  `raw_group_maps`，按既有 monitorability 合同为 unmonitorable。若直接丢弃该
  mode，原 ambiguity fail-closed 回归会错误变成可提交的 union；该尝试已撤销。
  因而这不是可以在 R0/R1 内通过阈值调整修复的问题。
- C/E/H 报警历元各生成 128 actions，但只有 45 个进入内核；C 有 43 个、E/H
  各有 3 个 post/PL 通过，仍没有一个 action 覆盖完整 plausible set，最终为
  `AMBIGUOUS_UNAVAILABLE`，backend update=0。这是当前正向 post-FDE 的直接阻塞。
- D 和 G-timeout 的 accel-x 原始样本注入已真实发生，但 `12.5 m/s²` 没有触发
  joint detector。因此未选择 bridge，也未触发 timeout/reinit。计划中的
  `1.25×` nominal-counterfactual 幅值校准本轮没有完成，不能把该幅值称为检测
  边界。
- 连续 UWB 拒绝用例中 pending 从 0.05 s 增到 1.85 s，state age 增到 1.80 s，
  raw IMU 样本增到 371，连续拒绝达到 36；说明这些量与未提交时长同步增长。

## 3. 实施内容

### R0

- 新增 `r0_r1_development` 和带哈希场景清单。故障只改原始 UWB range 或 200 Hz
  IMU samples，`fault_truth.csv` 记录 source、axis、值和精确时间区间。
- 新增统一 `RiskBudgetAudit`。最终判定使用高精度二进制累加；等分 allocation
  若因 double 舍入越过剩余预算，就向零调整一个 ULP。真实超支仍严格失败；
  新测试显式验证这一点。
- FDE 和 post-PL 复用同一 risk audit，并记录 nominal、`p_nm`、bridge、history、
  model、hypothesis sum、upper bound 和 margin。
- attempt diagnostics 升级到 v5：detector、数值原因、base/selected step 及每 epoch
  rotation/position/velocity/accel-bias/gyro-bias 分量、action 各阶段、输入/输出
  时间、state age、pending、raw IMU、连续拒绝、backend update、边缘化和分解
  计数均可追溯。旧 v1–v4 reader/validator 保持兼容。
- 修复 commit plan 为未加入 bridge group 写 relation 的问题；未改变 0.25、risk、
  rank、condition、detector 或 AL 门限。

### R1

- `LinearizedIntegrityWindow` 现在持有不可变 `FrozenWindowNumerics`。身份指纹绑定
  window ID、四项 version、H/z、information/RHS、protected map、state/key 排序、
  block 顺序/内容、noise/whitening ID 和 covariance；数值合同阈值另有指纹。
- finalize 一次构造薄 SVD、rank/DOF/condition/spectrum、information LLT、基础状态
  increment、直接 parity/statistic、logdet、solve residual 和 block row offsets。
- detector、evidence、KEEP_ALL 和 rank-update base 共用该上下文。正常良态窗口
  不再执行 detector QR；接近 condition gate 一数量级内保留独立 QR reference，
  mismatch 时 fail closed。变化 candidate 的精确 SVD/inner LLT、相关 UWB 主协方差
  重白化和 post-FDE 重算保持原语义。
- 实际 Eigen 构造点分别计数，不用函数调用近似。正常 30-window 场景实测为
  base SVD=30、base LLT=30、base solve=30、detector reference QR=0，即每窗口
  `1/1/1/0`；修改前静态基础路径为每窗口 `2/2/3/1`。
- 冻结内容任一图、排序、noise、linpoint、H、protected map、state ordering 或
  covariance 变化都会拒绝复用；replacement/marginalization/reinit 依赖 version 和
  content identity 失效。

### Analytic sensitivity 与审计开销

- 生产接口拆为 `buildAnalytic()`；显式 `verifyFiniteDifferenceOracle()` 才执行
  六轴中央差分。字段区分 analytic input/computation、oracle executed/error/
  verified，旧字段仅保留兼容含义，没有把 verified 常量化为 true。
- oracle 测试覆盖 stationary、constant velocity、constant acceleration、constant
  yaw、3D rotation、0.02–0.50 s 积分和 nonzero bias。生产路径均为 0 次重积分，
  显式 oracle 为 12 次。G 长 pending 的 371 raw samples 仍记录
  `oracle_reintegrations=0`。
- hypothesis/health 的字符串和 CSV 构造受 output flags 控制；必要 factor ledger、
  candidate、风险和事务证据保留。benchmark 不再先构造完整 hypothesis audit 后
  清空。core 计时边界未改；新增 `outer_epoch`（pipeline 输入到 CSV flush 完成）
  和其子项 `outer_logging_flush`。写入计时记录本身在测量后的第二次 flush，不纳入
  被测 epoch，避免自引用。

## 4. Raw-stream 场景结果

“有限 PL”列为最终 selected output；括号内是整段中有限 output 的历元数。所有
`PL≤AL` 和 formal eligible 计数均为 0。早期有限 PL 仍超过 HPL/VPL alert limit
`2/3 m`，是合法 unavailable。

| 场景 | 原始注入 | detector/事务结果 | 正确排除 | 故障历元有限 PL | PL≤AL | formal |
|---|---:|---|---|---|---|---|
| A nominal, 30 | 0 | 0 alarm，30 commit | N/A | N/A（20/30） | false | false |
| B 初速误差, 30 | 0 | 0 alarm，30 commit | N/A | N/A（18/30） | false | false |
| C UWB, 30 | anchor 1×1，2.25 m | 1 alarm，`AMBIGUOUS_UNAVAILABLE` | **false** | false（20/30） | false | false |
| D IMU, 30 | accel-x×10，12.5 m/s² | 0 alarm，KEEP_ALL | **false** | false（20/30） | false | false |
| E union, 30 | anchor×1 + accel-x×10 | 1 alarm，`AMBIGUOUS_UNAVAILABLE` | **false** | false（20/30） | false | false |
| F ramp, 5 | 0 | ramp mode unmonitorable | expected fail closed | false（0/5） | false | false |
| F low redundancy, 10 | 0 | unavailable by AL | N/A | N/A（10/10） | false | false |
| G reject, 45 | 8 anchors×36=288 | 36 consecutive rejects | expected reject | false（5/45） | false | false |
| G timeout, 52 | accel-x×260 | 0 alarm；未进入 bridge | **false / NOT_REACHED** | false（20/52） | false | false |
| H mature, 226 | epoch 225 联合 11 samples | 225 commit、1 reject、26 marginalizations | **false** | false（20/226） | false | false |

H 使用了 226 次真实 integrity pipeline 输入而非直接 commit 预热。第 225 个故障
输入发生在 25 次边缘化之后，该历元 backend `224→224`；第 226 个输入恢复提交，
最终边缘化计数为 26。它满足“经过 200+ 决策与提交并发生边缘化后再故障”的机制
覆盖，但不满足成功联合排除。

## 5. 数值 replay、并发和不变量

使用已有成熟 v3 冻结快照
`results/gate_d_p3_p6/fixed_lag_development/after-205-commits.bin`
（SHA-256 `2cef39153865a05e46ee80fc486cbedeac0d95c309090abc35a617af5d131f0e`）
重放 128 actions：4-worker fast 322.691 ms，1-worker fast 814.073 ms，1-worker
dense oracle 12498.1 ms。fast 的 1/4-worker 除 wall/scratch 字段外逐行一致；fast
证书与 oracle 的 128/128 valid、rank、DOF 一致，exact condition 或证书上下界均
覆盖 oracle，最大相对 statistic/logdet/step 差分别为
`9.67e-44 / 5.65e-13 / 5.56e-29`，满足 ADR 0002 分级容差。

回归还覆盖：prepare/discard 零 backend update、commit 恰好一次、统一冻结基线、
相关 UWB principal covariance 重白化、post detector/covariance/DOF/PL/risk 重算、
ambiguity/history contamination fail closed、缓存身份失效和原 128 个实际内核。
本轮没有实施 coverage 前置资格、候选剪枝、compact sparse window 或 touched-key
求解。

## 6. 开发性能

以下均为 Release、seed 20260901、affinity 0-3、4 candidate workers、numeric
threads=1。p99 在短场景中只作开发观测。`outer` 是完整 epoch 到首次 CSV flush，
不能与 core 或日志子项的分位数相加。C/D/E 并非成功 FDE，所以只作为失败/未触发
路径观察，不能冒充成功 UWB/IMU/union 性能。

| 路径 | n | core p50/p95/p99/max (ms) | outer p50/p95/p99/max (ms) | RSS KiB |
|---|---:|---:|---:|---:|
| nominal A | 30 | 298.02 / 565.30 / 619.97 / 626.11 | 298.28 / 565.58 / 620.30 / 626.49 | 243468 |
| UWB alarm C（未成功排除） | 30 | 307.51 / 608.76 / 3608.75 / 4810.70 | 307.81 / 609.04 / 3609.27 / 4811.32 | 384912 |
| IMU D（未触发） | 30 | 296.77 / 525.23 / 590.92 / 616.24 | 297.08 / 525.47 / 591.20 / 616.54 | 235280 |
| union E（未成功排除） | 30 | 291.08 / 574.23 / 1127.43 / 1336.12 | 291.37 / 574.51 / 1127.97 / 1336.77 | 322592 |
| continuous reject G | 45 | 13.09 / 30.37 / 44.20 / 47.91 | 13.27 / 30.56 / 44.40 / 48.12 | 45096 |
| mature H | 226 | 517.27 / 542.78 / 583.21 / 1372.17 | 517.56 / 543.02 / 583.44 / 1373.40 | 322924 |
| forced 128-action | 140 | 958.44 / 1189.42 / 1215.07 / 1246.61 | 959.90 / 1191.05 / 1216.58 / 1248.42 | 583600 |

原 benchmark 的同条件 forced-140 基线保存在
`results/r0-r1/baseline-e6ded306/forced-140/`，其独立二进制 SHA-256 为
`0dbd6fc2bf03549ec496c42ff5f464fa6f8ec720622358a8db6f10543eb49ba6`。
两边外部 audit CSV 行数一致；修改后 resolved hash 额外记录了 runtime output
override，因此 hash 不相同，不能隐瞒为字节相同配置。基线没有 `outer_epoch`，
故 outer 前后对比为 `NOT_AVAILABLE`。

| forced-140 core | 基线 p50/p95/p99/max (ms) | 修改后 p50/p95/p99/max (ms) | 变化 |
|---|---:|---:|---:|
| 全 140 | 1047.74 / 1307.18 / 1341.79 / 1357.68 | 958.44 / 1189.42 / 1215.07 / 1246.61 | -8.52% / -9.01% / -9.44% / -8.18% |
| 前 60 | 697.98 / 734.64 / 763.03 / 792.89 | 544.21 / 573.02 / 587.57 / 588.27 | p50 -22.03% |
| 后 80（128 actions） | 1178.41 / 1318.40 / 1353.87 / 1357.68 | 1080.23 / 1193.50 / 1222.89 / 1246.61 | p50 -8.33% |

总 core 从 126846 ms 降为 111375 ms（-12.20%），RSS 从 618644 降为
583600 KiB（-5.66%）。修改后实际生成 9728 个 kernel evaluations，其中 69 个
历元各 128 个，PL 实际执行 1 次；candidate wall n=9728，p50/p95/p99/max 为
`7.684/13.620/18.285/270.541 ms`。分解计数为 base SVD/LLT/solve
`140/140/140`、detector reference QR `0`、candidate reference SVD `80`、
candidate inner LLT `28764`、covariance RHS solve `140`、oracle reintegration `0`。
基线没有运行时分解计数器；其 `280/280/140 QR` 仅由旧源码每窗口构造点静态推导，
不是伪装成运行时测量。

## 7. 构建、测试和复现命令

最终构建：

```bash
cd /home/mint/ws_fusion_uwb
catkin build uwb_imu_pl --no-deps --cmake-args -DCMAKE_BUILD_TYPE=Release
```

最终包级 `catkin run_tests uwb_imu_pl --no-deps` 汇总为 178 tests、0 errors、
0 failures、0 skipped。另直接复跑的受影响 C++ suites 共 88/88 通过：snapshot 7、
strict config 10、logger 4、realtime incremental 24、deterministic event 2、dense
oracle 2、Integrity V2 39。独立 Python 46/46 通过，ROS rostest 1/1 通过：

```bash
for t in test_snapshot_integrity test_integrity_config test_run_logger \
  test_realtime_incremental test_deterministic_event test_dense_oracle \
  test_integrity_v2; do
  /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/$t
done
PYTHONPATH=test python3 -m unittest -q test_advisor_report test_gate_d_tools \
  test_round2_tools test_round3_tools test_week4_tools
rostest uwb_imu_pl realtime_integrity_viz.test
cd /home/mint/ws_fusion_uwb && catkin run_tests uwb_imu_pl --no-deps
```

场景示例；其余名称和预期见
`config/r0_r1_development_scenarios.yaml`：

```bash
cd /home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl
taskset -c 0-3 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 UWB_IMU_PL_CANDIDATE_WORKERS=4 \
  /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/r0_r1_development \
  config/realtime_uwb_imu_pl_research.yaml \
  results/r0-r1/reproduce/A-30 30 A_nominal

taskset -c 0-3 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  MKL_NUM_THREADS=1 UWB_IMU_PL_CANDIDATE_WORKERS=4 \
  UWB_IMU_PL_BENCHMARK_FORCE_ALARM=1 \
  /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark \
  config/realtime_uwb_imu_pl_research.yaml \
  results/r0-r1/reproduce/forced-140 140
```

所有 12 个最终运行目录均经 `tools/validate_run_schema.py` 验证为 v5 PASS。
工作区 `git diff --check` 通过。构建只报告工作区原有的三个 nodelet library
symlink 目标冲突 warning，没有编译失败。

## 8. 未闭合事项与决策点

- **成功 UWB FDE、IMU generic bridge、联合/union raw-stream 路径：未完成。**
  当前成熟窗口的 boundary/compressed occurrence monitorability 与 complete plausible
  coverage 共同阻塞 selection。解决它需要明确历史风险/eligibility 语义或 R2 的
  coverage 前置规则，属于本轮禁止改变的研究合同，不能在这里偷偷删 hypothesis。
- **bridge timeout→controlled reinit 的 raw-stream 到达性：未完成。** 当前 IMU
  幅值未触发 detector。下一阶段应先实现 nominal-counterfactual directional
  boundary calibration，再选择确实在 analytic fault model 和 bridge envelope 内的
  原始 IMU 注入；不能简单增大到任意幅值。
- **AL 可用性：未完成。** 当前所有有限 PL 均超过 2/3 m alert limit。它是合法
  unavailable，不通过放宽 AL 或风险预算修复。
- 正式 3×12000 campaign、E–I 统计矩阵、正式 CPU 性能合同和独立评审均为
  `NOT_RUN`。R2/R3、新 fault model 和新 bridge model 均未实施。

建议下一决策先回答：压缩进边界先验且已无显式 measurement rows 的 occurrence，
应由非零 history-contamination 风险覆盖、保留可计算的边界 sensitivity，还是在
正式 eligibility 前置阶段从当前 action coverage census 中剥离。确定合同后再做
校准过的 C/D/E/G 正向场景，才能闭合 R0；R1 本身已具备继续演进的共享数值基础。
