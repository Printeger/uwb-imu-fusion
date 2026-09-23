# UWB–IMU FGO/FDE 五模式实现与验证报告

状态：`IMPLEMENTED_WITH_VALIDATION_FAILURES`
日期：2026-09-23（Asia/Shanghai）

本报告只使用本轮最终二进制产生的证据。结论不是“全部验收通过”：五模式、统一事务/FGO/FDE/PL 接口、ROS 输出和验证工具已实现；配置、单元、集成、ROS 和证据 schema 回归通过；但随机噪声姿态精度、长程位姿、四个 active profile 的实时性、告警限内可用性和预定义恢复动作均未达到设计门限。正式保护资格因真实校准、罕见事件 campaign、Gate J 和独立评审缺失而保持 `false`。

## 0. 可恢复工程基线补充

本文件所在的新本地 Git 提交固化了原 dirty workspace，并补充修复了
README、ROS execution identity 和默认日志策略。该提交没有重跑或替代本
报告后文的完整算法 campaign；历史实验仍严格由原 SHA、dirty diff hash、
二进制 hash 和五个 summary hash 标识，`FINAL_STATUS.json` 的算法结论未改。

- 整个系统只需一次 Release 编译；五个 profile 均使用
  `devel/lib/uwb_imu_pl/uwb_imu_pl_realtime_node`，通过配对的 v6 配置在
  重启节点时切换，不支持进程内热切换。
- `realtime.launch`、`realtime_integrity_sim.launch` 和被 include 的
  `realtime_integrity_stack.launch` 统一新增 `enable_run_logging`，默认
  `false`。关闭时不构造 `RunLogger`，不创建 online 目录、resolved config、
  manifest 或 CSV；topic、diagnostics、RViz 和算法仍运行。
- 显式开启日志时，指定目录必须尚不存在，否则 fail closed；未指定目录时
  在 resolved `output.root` 下创建唯一 `online_<wall-time-ns>` 目录。
- manifest 的 `execution_command` 在写入前绑定节点实际解析出的
  `config_path`、`fde_profile`、fixed lag、seed、日志开关、最终目录和输出
  overrides；仿真 launch 另外完整保留 trajectory、fault、packet-loss、NLOS
  和 RViz 参数。未展开的 `$()`/`${}` 占位符被拒绝。
- 权威输出仍是 `/uwb_imu_pl/solution`；odometry 和 `IntegrityStatus` 只是
  后发兼容镜像。

本次最小充分工程回归结果：

| 项目 | 结果 |
|---|---|
| Release 增量构建 | PASS；同一二进制用于全部 smoke |
| 全测试 | 442 tests，0 errors / failures / skipped；高于原 436 |
| 五模式默认日志 smoke | PASS；profile 与配置一致；off=`NOT_COMPUTED`；active hypothesis census 为 24/6/27/111，并进入 PL/FDE 路径 |
| 默认零运行文件 | PASS；隔离 ROS_HOME、仓库和工作目录均无新增 CSV/manifest/resolved config/online 目录 |
| RViz 接线 | PASS；realtime node、visualizer 和 RViz 均由同一 launch 解析 |
| 显式日志 | PASS；v6 schema、resolved config、manifest、config hash、scope digest 和完整 execution command 可读 |
| 覆盖保护 | PASS；已有显式目录被拒绝；自动目录唯一且 command 记录最终实际路径 |

工程 smoke 二进制 SHA-256 为
`13e75c06fd786aa276be5d309293747475961b41d45db128d3c420ad62528a24`。
它只认证本节接口回归，不反向冒充后文大型 campaign 的 benchmark 二进制。
本次未重跑长期性能/MC campaign，所有 smoke 临时 CSV 和 ROS 日志均在验证后
删除。

## 1. 实现身份与复现

| 项目 | 最终结果 |
|---|---|
| 基线 SHA / 分支 | `5a808ebda2d25fd13ee6af824c5230fcee73ffcc` / `feature/realtime-uwb-imu-pl` |
| 运行时 dirty diff hash | `62948a51f0eea7b28367bf41778a097ab0047f48f57bc93d46679ffe80a617f0`；报告与 README 在实验后更新，因此不反向冒充运行身份 |
| benchmark / shared library SHA-256 | `ede37caf...f93460` / `779ab37e...07fa`，完整值见 `results/fde_profiles_p0_p7_20260923/FINAL_STATUS.json` |
| protocol | `fde-profiles-20260923-v1`；SHA-256 `93662f3c0e7083e446c3ffc1c52926ee2e5626a959123d1b38a9438b9a6b83dc` |
| profile config SHA-256 | off `937133ac...07d2d`；UWB `cc6ec410...9f95`；IMU `4855cd14...966d`；joint1 `95f0b25c...154`；joint2 `fc0a453f...7ff8` |
| resolved scope digest | off `e8fc5ca8f8aaeeeb`；UWB `15f126bec131810f`；IMU `f7d31e6776e6dff0`；joint1 `88d5401651284008`；joint2 `fbbf232a0a941ee9` |
| 数据 | runner 内生成的确定性原始 IMU/UWB 流与独立 ground truth；每个 run 的 `ground_truth.csv`、`fault_truth.csv`、resolved config 和 SHA-256 在 `validation.json` 中 |
| 固定输入 | IMU 200 Hz、UWB 20 Hz、8 个非共面锚点、lag 30 epoch、window 10、recovery margin 10、lever arm `[0.25,-0.10,0.08]` m |
| 噪声 | 离散 accel `0.01 m/s²`、gyro `0.001 rad/s`，按 `sqrt(dt)` 换算为连续密度；UWB/IMU 物理噪声为完整性 overbound 的 0.05/0.01 |
| 编译/并发 | Release `-O3 -DNDEBUG`；候选 worker=4；未设置 OMP/BLAS 线程环境变量；WSL2 无 governor 接口 |
| 主机 | Intel Core Ultra 5 125H，18 logical CPU，15 GiB RAM，WSL2 Linux 6.18.33.2，g++ 9.4.0，ROS Noetic，GTSAM 4.2a5，Eigen 3.3.7 |
| 最终 schema | 69/69 run directories PASS；JSON 使用 `null` 而非非标准 Infinity |
| 全回归 | 436 tests，0 errors / 0 failures / 0 skipped；含 ROS rostest；Gate-D Python 工具 14/14 PASS |

最终证据集合及摘要 SHA-256：

| 集合 | 规模 | 摘要 SHA-256 |
|---|---:|---|
| `p7/fgo_final_v4` | 4 runs | `e02d41df...8e8` |
| `p7/off_mc20_v3` | 20×60 s | `d2032efa...ab6` |
| `p7/off_full_v3` | 3×600 s | `7862e165...bb8` |
| `p7/active_pilot_v3` | 4×600 epoch | `4ff72e43...16ce` |
| `p7/profile_matrix_v3` | 38 fault/profile runs | `b26dbc3f...6eb` |

完整索引为 `results/fde_profiles_p0_p7_20260923/FINAL_STATUS.json`。每个 run 还包含 `validation_run.json`、`run_manifest.json`、resolved config、逐 epoch compact timing、状态/真值、故障真值及文件 hash。

## 2. 最终架构与接口

```mermaid
flowchart LR
  RAW[原始 IMU/UWB] --> Q[有界队列与输入校验]
  Q --> PRE[统一 prepareEpoch / 预积分 / nominal factors]
  PRE --> OFF{resolved profile}
  OFF -->|off| COM[统一 commitEpoch + fixed lag]
  OFF -->|active| WIN[当前窗口 + scope 限定历史摘要]
  WIN --> DET[双通道 current/history detector]
  DET --> HYP[scope 限定 singles/pairs 与 actions]
  HYP --> CAND[post-FDE detector + PL + risk]
  CAND --> TX[原子 commit/discard]
  COM --> PUB[身份/证书/结束时 deadline]
  TX --> PUB
  PUB --> ATOMIC[/uwb_imu_pl/solution]
```

关键实现事实：

- 五个 profile 由 `ResolvedFaultScope` 统一解析；没有五套 pipeline。`off` 仍执行同一 IMU 预积分、UWB 因子、事务、后端和 fixed lag，只跳过完整性材料，输出优化状态并明确 `FDE_DISABLED / detector NOT_RUN / PL NOT_COMPUTED`。
- inactive provider 不构建其故障映射、历史故障列、sensitivity 或恢复 action；nominal IMU/UWB factor 始终保留。UWB-only 的历史 nominal IMU provenance 回归已覆盖。
- `joint_order2` 包含全部一阶 UWB/accel/gyro 假设，并组合支持的 UWB×accel、UWB×gyro 二阶族；不构造未支持的同族 pair。
- current/history 双通道统计量同时用于生产 detector、post-FDE detector 和 PL。修复了 fixed-lag history constant 重复扣除、风险预合并、scope/cache/certificate 身份和末端 deadline reason 覆盖。
- 历史摘要、cache key、风险证明、候选、PL、日志和发布都携带同一 scope digest；错配或篡改 fail closed。
- v6 只有 `fde.profile` 一个模式源；与 legacy order/enable 字段并存时报错。v5 确定性迁移并写出 warning；double-only 这种不能映射到五模式的配置拒绝迁移。
- 新原子 ROS 消息 `msg/NavigationIntegrity.msg` 同时携带 state、velocity、bias、attempt/state/publish time、commit、scope、PL、证书和 deadline。`/uwb_imu_pl/solution` 先发布；旧 odometry 与 `IntegrityStatus` 只是兼容镜像。
- IMU queue overflow/IMU gap 触发可见的 controlled reinitialization；UWB overflow 按显式 drop-oldest policy 记录序号和计数；无静默预积分跨越。

主要文件：`fault_scope.*`、`integrity_config.*`、`incremental_estimator.*`、`dual_channel_detector.cpp`、`protection_level_v2.cpp`、`publication_identity.*`、`run_realtime_integrity.cpp`、五份 `config/fde_*.yaml`、统一 runner/analyzer 和本报告。README 已给出实际 launch 与验证命令。

## 3. 五模式验收总表

| profile | 实现/事务/scope | 精度/freshness | PL 数值 | 恢复与可用性 | 实时 | 正式保护资格 |
|---|---|---|---|---|---|---|
| off | PASS | freshness PASS；姿态/长程 FAIL | PASS：正确地 NOT_COMPUTED | N/A | PASS | N/A（FDE 关闭） |
| uwb_order1 | PASS；U=264, I=0, pair=0 | 单轨迹位置 PASS；姿态沿用主线 FAIL | PASS：finite 98.5%，误差覆盖 100% | FAIL：within 7.83%，无预期排除 action | FAIL：p99 135.7 ms | BLOCKED / `false` |
| imu_order1 | PASS；U=0, accel=30, gyro=30 | 同上 | PASS：finite 100%，误差覆盖 100% | FAIL：within 0.17%，恢复段 finite 67.9%，action 错误 | FAIL：p99 110.2 ms | BLOCKED / `false` |
| joint_order1 | PASS；U=264, accel=30, gyro=30 | 同上 | PASS：finite 99%，误差覆盖 100% | FAIL：within 0%，UWB/IMU recovery 未选排除 action | FAIL：p99 203.6 ms | BLOCKED / `false` |
| joint_order2 | PASS；一阶同 joint1；U×A=7920, U×G=7920 | 同上 | PASS：finite 99%，误差覆盖 100% | FAIL：overlap recovery 未选二阶恢复 action，within 0% | FAIL：p99 409.5 ms | BLOCKED / `false` |

这里“误差覆盖 100%”的分母只是 seed 20260901 的 finite-PL epoch（591/600/594/594），序列相关，不能转写为独立 Bernoulli 风险或 HMI 证明。

## 4. FGO 位姿、时效与 fixed lag

### 4.1 解析、batch 与短段场景

| 场景 | commit/fresh/marg | position RMSE/P95 m | attitude RMSE rad | velocity RMSE m/s | state age P99/max | 判定 |
|---|---|---:|---:|---:|---:|---|
| noiseless 10 s | 200/200/171 | 0.000022 / 0.000033 | 0.001114 | 0.000240 | 0/0 | PASS |
| nominal 10 s | 200/200/171 | 0.01638 / 0.03051 | 0.05725 | 0.02495 | 0/0 | PASS（短段） |
| initial_error 10 s | 200/200/171 | 0.04668 / 0.06304 | 0.16667 | 0.05160 | 0/0 | FAIL：10 s 末姿态误差 0.1768 rad；且未完成各轴/符号网格 |
| degenerate 10 s | 200/200/171 | 0.01844 / 0.03106 | 0.06786 | 0.02382 | 0/0 | 独立报告；不声称完整姿态可观 |

`UwbImuIncremental.FiveSecondIsamMatchesBatchPoseAndMarginal` 用相同因子、先验和观测比较 iSAM 与独立 batch LM：Pose3 Logmap norm `<1e-5`，joint marginal 相对误差 `<1e-5`，PASS。`FixedLagMatchesUnboundedBeforeAndAfterMarginalization` 同时比较 pose、velocity、bias、detector 与 PL，PASS。

世界坐标直接比较，无 SE(3) 对齐；UWB 测量使用旋转真值与非零 tag lever arm；状态 timestamp 等于实际优化 state time。truth-at-state 与 truth-at-attempt 在这些同步、全 fresh 运行中相同。RPE 以及所有初值轴/符号组合未单独执行，标 `NOT_RUN`。

### 4.2 20-seed × 60 s nominal

| 指标（每 seed 门限） | mean | worst | 通过 seed | 判定 |
|---|---:|---:|---:|---|
| position RMSE ≤0.20 m | 0.04450 | 0.07866 | 20/20 | PASS |
| position P95 ≤0.50 m | 0.08244 | 0.17925 | 20/20 | PASS |
| attitude RMSE ≤0.10 rad | 0.16597 | 0.29567 | 4/20 | FAIL |
| velocity RMSE ≤0.20 m/s | 0.02577 | 0.02686 | 20/20 | PASS |
| accel-bias norm RMSE | 0.000927 m/s² | 0.001063 | — | 记录值 |
| gyro-bias norm RMSE | 1.67e-5 rad/s | 2.96e-5 | — | 记录值 |
| fresh/commit | 100% / 100% | — | 20/20 | PASS |

因此 Gate FGO 不能整体标 PASS。位置、速度、freshness 和 fixed-lag 正确，但姿态随噪声 seed 漂移。

### 4.3 3×600 s sustained

| repeat | commit/fresh/marg | position RMSE/P95 m | attitude RMSE rad | velocity RMSE | core/outer p99 ms | RSS peak MiB | 判定 |
|---|---|---:|---:|---:|---:|---:|---|
| r1 | 12000/12000/11971 | 0.1834 / 0.3962 | 0.7396 | 0.0308 | 2.576 / 2.736 | 29.34 | realtime PASS，pose FAIL |
| r2 | 12000/12000/11971 | 0.3559 / 0.4939 | 1.5696 | 0.0424 | 2.183 / 2.355 | 26.64 | position/attitude FAIL |
| r3 | 12000/12000/11971 | 0.0878 / 0.1866 | 0.3311 | 0.0269 | 2.224 / 2.400 | 26.58 | realtime PASS，attitude FAIL |

三次均无 reject、deadline miss 或 >40 ms mature core epoch；fixed lag 确实边缘化。新增 ledger pruning 回归将固定滞后 factor provenance 限制为 ≤30 entries。逐 epoch RSS 曲线和 queue depth 未采集，因此“末两个等长区间 RSS 增长≤10%”为 `NOT_RUN`；只报告 `/usr/bin/time` 峰值与结构有界测试，不用峰值替代曲线证明。

## 5. FDE、PL 与故障矩阵

### 5.1 nominal PL

| profile | finite/600 | HPL P50/P95/max m | VPL P50/P95/max m | actual-error coverage | within 2m/3m | 结果 |
|---|---:|---:|---:|---:|---:|---|
| uwb_order1 | 591 | 2.866 / 3.064 / 3.257 | 1.144 / 1.395 / 1.408 | 100% | 47/600 = 7.83% | PL_NUMERICS_PASS / AVAILABILITY_FAIL |
| imu_order1 | 600 | 3.529 / 3.814 / 6.664 | 2.094 / 2.186 / 4.648 | 100% | 1/600 = 0.17% | PL_NUMERICS_PASS / AVAILABILITY_FAIL |
| joint_order1 | 594 | 3.891 / 4.197 / 6.754 | 2.217 / 2.310 / 4.707 | 100% | 0/600 | PL_NUMERICS_PASS / AVAILABILITY_FAIL |
| joint_order2 | 594 | 4.622 / 4.978 / 6.950 | 2.856 / 3.170 / 4.806 | 100% | 0/600 | PL_NUMERICS_PASS / AVAILABILITY_FAIL |

finite 比例四模式均达到 ≥95%；2 m/3 m 告警限内 ≥90% 的目标全部失败。`inf` 没有被算入 finite 正向功能。

### 5.2 恢复段

| profile/scenario | fault epoch | post commit/finite | post HPL P50/P95 m | post within | 动作与判定 |
|---|---|---:|---:|---:|---|
| uwb_order1 / uwb_recovery | 0/1 commit，AMBIGUOUS | 53/53 | 2.151 / 2.479 | 37.7% | fault 被 discard，但未选 UWB exclusion；FAIL |
| imu_order1 / imu_recovery | 1/1 best-effort commit | 53/36 | 3.138 / 3.282 | 0% | 17 epoch NO_VALID，未选 IMU bridge/exclusion；FAIL |
| joint_order1 / uwb_recovery | 0/1 commit | 53/53 | 3.458 / 3.685 | 0% | 无 exclusion action；FAIL |
| joint_order1 / imu_recovery | 1/1 best-effort commit | 53/35 | 3.424 / 3.589 | 0% | 无 bridge/exclusion；FAIL |
| joint_order2 / joint_recovery | 0/1 commit | 53/35 | 4.013 / 4.261 | 0% | 未选 UWB×IMU 二阶恢复 action；FAIL |

UWB、IMU 六轴、联合、历史跨窗、持续拒绝和恢复均在最终 38-run 矩阵中执行。普通 accel/gyro 区间注入多数仍 `KEEP_ALL`；UWB/联合强故障多 fail closed 为 `AMBIGUOUS_UNAVAILABLE`；持续拒绝场景 fault 段 27/27 reject，之后能恢复提交，但告警限内仍失败。历史 UWB 场景在 joint1/2 仅提交 6/80，post 0/24，说明范围贯穿但正向历史恢复未通过。

### 5.3 数学与输出合同

PASS 的独立/反例测试包括：dense rank-update oracle、analytic IMU sensitivity 对 finite-difference sweep、dual-channel `pooled pass + channel fail`、zero-history dof、rank/condition 边界、post-FDE recomputation、risk ledger closure、scope/cache/certificate mutation、交换 epoch、PL tamper、watchdog deadline、clock rollback、ROS state/PL pairing。fixed-lag history constant 修复有直接单测，避免 current statistic 在边缘化后变负。

这些单测证明合同和数值接线；端到端恢复失败仍按 FAIL 保留，不能由 oracle PASS 抵消。

## 6. 性能与热点

### 6.1 nominal pilot

| profile | commit/fresh/marg | finite PL | core p50/p95/p99/max ms | outer p99/max ms | >40 ms mature | deadline miss | RSS peak MiB | 判定 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| off（3×full 范围） | 12000/12000/11971 | N/A | p99 2.18–2.58；max 7.37–11.37 | p99 2.36–2.74；max 7.56–11.49 | 0/11900 每 repeat | 0% | 26.6–29.3 | PASS |
| uwb_order1 | 600/600/571 | 591 | 104.5/112.7/135.7/143.9 | 136.0/144.1 | 500/500 | 97.7% | 41.97 | REALTIME_FAIL |
| imu_order1 | 600/600/571 | 600 | 85.6/93.7/110.2/170.8 | 110.5/171.2 | 500/500 | 97.0% | 36.62 | REALTIME_FAIL |
| joint_order1 | 600/600/571 | 594 | 171.2/179.9/203.6/252.0 | 203.9/252.3 | 500/500 | 98.0% | 48.71 | REALTIME_FAIL |
| joint_order2 | 600/600/571 | 594 | 378.2/396.6/409.5/467.4 | 409.8/467.7 | 500/500 | 99.3% | 79.81 | REALTIME_FAIL |

`outer_epoch` 是离线同步 runner 的全链路代理，包含 logging flush，但不是 ROS 本机 arrival timestamp 到实际 publish 的在线量测；真正的 `arrival_to_publish`、queue P99/max 和 sensor-to-publish 为 `NOT_RUN`。没有把 outer 代理冒充线上指标。

active profiles 在 pilot 已全部明显超过 40/50 ms，且每个 mature epoch 都超限。依照设计的“pilot 先定位、单帧高成本先修复”规则，未继续执行 active 12000×3 或 20-seed PL MC；二者标 `NOT_RUN`，不是 PASS，也没有缩样本冒充完整实验。

### 6.2 active 各阶段 p99 wall ms

| stage | uwb_order1 | imu_order1 | joint_order1 | joint_order2 |
|---|---:|---:|---:|---:|
| prepare | 1.858 | 1.086 | 2.320 | 3.312 |
| integrity_window | 90.722 | 69.312 | 120.068 | 108.418 |
| window_boundary_provenance | 68.684 | 41.096 | 94.466 | 83.342 |
| window_fingerprint | 8.081 | 17.734 | 12.550 | 12.455 |
| window_svd | 12.514 | 12.720 | 13.380 | 12.724 |
| all_in_detector | 9.228 | 18.165 | 16.223 | 16.465 |
| current_sensitivity | 0.022 | 0.589 | 1.359 | 1.350 |
| historical_sensitivity | 0.000 | 0.163 | 0.223 | 0.237 |
| model_generation | 2.574 | 0.345 | 2.669 | 33.299 |
| hypothesis_evidence | 13.271 | 7.457 | 23.922 | 69.437 |
| health_actions | 0.406 | 0.187 | 0.490 | 3.716 |
| candidate_evaluation | 17.643 | 9.127 | 30.515 | 180.176 |
| fde_decision | 0.150 | 0.095 | 0.197 | 3.763 |
| commit | 3.591 | 2.628 | 2.845 | 3.132 |
| logging flush | 0.395 | 0.429 | 0.385 | 0.491 |
| core_total（全部 epoch） | 135.731 | 109.408 | 203.648 | 409.228 |

完整 31-stage count/P50/P95/P99/max 在 `active_pilot_v3/validation_summary.json`，未将阶段 p99 相加成 total。未采集 exclusive CPU 与 worker 累计 CPU，表中均为 wall；父阶段包含子阶段。

热点结论：

1. joint_order2 的候选求值 p99 180.2 ms，受 15,840 个 UWB×IMU pair 以及完整 post-detector/PL 影响，是第一热点。
2. 所有 active profile 的 window boundary/provenance 41–94 ms，是共同热点；需要增量历史摘要/指纹且逐帧证明统计量、rank、PL 和动作等价。
3. joint_order2 hypothesis evidence 69.4 ms、model generation 33.3 ms，是第二阶组合特有热点。
4. off 最初存在全 ledger 扫描和未裁剪 provenance 的 O(N²)/RSS 增长；修复为无 frozen graph 不扫描、ledger prune、benchmark 关闭全 dump，600 s 从中止/243 MiB 降为约 16 s wall 和 <30 MiB。

## 7. 回归、修复链与差异

最终回归：

- `catkin run_tests uwb_imu_pl --no-status`：436/436 PASS，包括配置、事务、fixed lag、detector/PL/oracle、publication、logger 与 ROS rostest。
- `PYTHONPATH=tools python3 test/test_gate_d_tools.py`：14/14 PASS。
- 69 个最终 run：`tools/validate_run_schema.py` 全部 PASS。
- profile generator 重建后五份配置 hash 稳定；runner 对 protocol/config/code/seed 不一致拒绝 resume。

本轮主要缺陷及修复：

| 缺陷 | 最小证据 | 修复与回归 |
|---|---|---|
| UWB-only 历史遗漏 nominal IMU provenance | 下一窗口报告 provenance incomplete | nominal IMU group 始终进入历史；仅故障列受 provider gate；profile 回归 PASS |
| 双通道 fixed-lag current statistic 变负 | epoch≈45 后 active 停滞 | pooled statistic 加 history constant 后再拆分；独立 split 单测和 600 epoch PASS |
| deadline 覆盖上游 reason | timeout 只剩 deadline 文本 | append upstream reason；publication tests PASS |
| factor ledger 长程 O(N²)/RSS 增长 | 早期 12000-run >7 min、RSS 243 MiB | frozen-only scan、ledger prune、关闭 benchmark 全 dump；最终 3×600 s PASS realtime |
| v6 validator 不识别 COMMITTED/SKIPPED_PROFILE/zero window | off schema FAIL | profile-aware diagnostics 和 v6 identity checks；69/69 PASS |
| compact invalid candidate reason 为空 | mature attempt 284 schema FAIL | invalid candidate无条件保留 fail-closed reason；300-epoch复现和最终全部 schema PASS |

早期结果与最终二进制 hash 不同，只用于上述缺陷链，不用于最终 PASS 表。

## 8. 未通过、未执行与外部阻塞

| 优先级 | 项目 | 状态/证据 | 根因与下一步 | 必须保留的合同 |
|---|---|---|---|---|
| P0 | 主线姿态/长程 pose | FAIL：随机姿态 4/20；600 s 姿态全失败 | 检查姿态/bias 可观性、噪声/随机游走、轨迹激励和线性化；增加各轴初值与 RPE | 不改真值、不做自由对齐、不放宽 0.10 rad |
| P0 | active 实时性 | FAIL：p99 110–410 ms | 增量 boundary/history、共享/缓存 hypothesis numerics、批量 pair candidate PL | 保留完整 scope、分母和 deadline |
| P0 | 正确恢复 action | FAIL：恢复多为 discard 或 KEEP_ALL | 校准 fault evidence、健康状态和 candidate gate；逐族建立预定义正向动作 | detector/post-FDE/PL 同合同；无有效动作仍 discard |
| P1 | operational availability | FAIL：0–7.83% nominal | PL 偏保守；定位 history/bridge/model margins 与风险分配，不改 alert limit | 2 m/3 m 和完整故障族不变 |
| P1 | 历史跨窗正向恢复 | FAIL：post 0 commit | 检查历史 replacement/bridge 可恢复性和 maturity | scope/历史/cache/证书同身份 |
| P2 | active full benchmark / MC | NOT_RUN | pilot 明显超时且热点未消除；先完成等价优化再跑 12000×3 与 20×60 s | 不缩样本，不只统计快速帧 |
| P2 | queue/RSS 曲线、线上 arrival-to-publish | NOT_RUN | 当前 runner 只保留 peak RSS 和 offline outer epoch | 后续加紧凑逐 epoch resource sampler |
| External | 真机/真实标定/正式风险 | BLOCKED | 缺硬件、真实数据、overbound/bridge calibration、Gate J、独立评审 | `formal_eligible=false` |

当前可以得出的严格结论：统一五模式软件和合同已经实现且回归通过；off 能实时、fresh 地输出实际优化状态并正确不计算 PL；active 模式能生成有限且在本合成单轨迹覆盖真实误差的模型内 PL，但无法满足实时、告警限可用性或预定义 FDE 恢复，因此不能称为完整验收通过或正式保护系统。

## 9. 复现命令

```bash
cd /home/mint/ws_fusion_uwb
source /opt/ros/noetic/setup.bash
catkin build uwb_imu_pl --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
catkin run_tests uwb_imu_pl --no-status
catkin_test_results --verbose build/uwb_imu_pl/test_results

cd src/uwb-imu-fusion-pl
python3 tools/generate_fde_profile_configs.py
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate fgo \
  --output results/fde_profiles_repro/fgo --epochs 200
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate performance \
  --profiles off --output results/fde_profiles_repro/off_mc20 \
  --epochs 1200 --repeats 20
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate performance \
  --profiles off --output results/fde_profiles_repro/off_full --scale full
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate performance \
  --profiles active --output results/fde_profiles_repro/active --scale pilot
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate profiles \
  --profiles all --output results/fde_profiles_repro/profiles --epochs 80
```

ROS 启动：

```bash
roslaunch uwb_imu_pl realtime.launch \
  config_path:=$(rospack find uwb_imu_pl)/config/fde_joint_order1.yaml \
  fde_profile:=joint_order1
```

最终保留/删除项和磁盘占用见实施进度文件及任务产物根目录的 `ARTIFACT_MANIFEST.md`。
