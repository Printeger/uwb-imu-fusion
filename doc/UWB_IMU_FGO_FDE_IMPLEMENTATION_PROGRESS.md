# UWB–IMU FGO/FDE 五模式实施进度

更新时间：2026-09-23（Asia/Shanghai）
任务状态：`IMPLEMENTED_WITH_VALIDATION_FAILURES`；无运行中任务

## 可恢复 Git 基线固化

- 在不改动核心 FGO/FDE/PL 数学和既有 `FINAL_STATUS.json` 结论的前提下，
  已修复 README、完整 execution command 和 ROS 默认日志策略。
- 五种 profile 只编译一次并复用同一个
  `devel/lib/uwb_imu_pl/uwb_imu_pl_realtime_node`；运行时用对应 v6 配置切换，
  无进程内热切换。
- 三个日常/仿真 launch 的 `enable_run_logging` 默认均为 `false`；集中式
  optional logging session 在关闭时不创建 logger 或文件。显式开启时支持
  唯一自动目录，并拒绝任何已存在的显式运行目录。
- 四个 launch 的 execution identity 已覆盖 config/profile/lag/seed/logging/
  output 参数；仿真身份还覆盖 trajectory、fault、packet loss、NLOS 和 RViz。
  节点在 manifest 写入前以实际 resolved 值替换空默认值并拒绝未展开占位符。
- Release 构建 PASS；当前完整回归 442/442 PASS；launch XML/参数解析、Python
  py_compile、`git diff --check` PASS。
- 五模式同二进制 smoke PASS：off 的 PL 为 `NOT_COMPUTED`；四个 active profile
  生成 24/6/27/111 个 hypothesis 并进入 PL/FDE 路径；RViz 接线 PASS。
- 默认日志关闭 smoke 为零运行文件；显式开启 smoke 生成 208 KiB v6 合法
  证据，execution command/config hash/scope digest 一致；已有目录覆盖被拒绝。
- 本次仅执行接口最小回归，没有重跑长期性能或 Monte Carlo campaign；原有
  姿态、active 实时性、PL availability 和 recovery FAIL 结论全部保留。
- 新基线提交固化的是原 dirty workspace；历史实验仍由原 SHA
  `5a808ebd...`、dirty diff hash、旧二进制 hash 和证据 summary hash 标识。

## 基线与产物纪律

- 工作区：`/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl`
- 基线 SHA / 分支：`5a808ebda2d25fd13ee6af824c5230fcee73ffcc` / `feature/realtime-uwb-imu-pl`
- 最终实验绑定的 dirty diff hash 为 `62948a51f0eea7b28367bf41778a097ab0047f48f57bc93d46679ffe80a617f0`。
- 用户原有未跟踪文档均保留，包括设计、旧审计报告和本报告路径；未清理用户既有 `results/`、`logs/`、bag、数据集、build/devel/install。
- 开始时仓库约 1.2 GiB，workspace 既有 `build/` 约 3.2 GiB、`devel/` 约 1.4 GiB、`logs/` 约 14 MiB；磁盘约 743 GiB 可用。
- 本任务实验仅写入 `results/fde_profiles_p0_p7_20260923/`，峰值约 1.8 GiB，未超过 2 GiB 预算；始终远高于 5 GiB 空闲下限。
- 当前仓库及父路径未发现适用 `AGENTS.md`；兄弟仓库的 `AGENTS.md` 不适用。

## P0–P7 最终状态

| 阶段 | 实施状态 | 验证状态与证据 | 未通过/未执行 |
|---|---|---|---|
| P0 基线冻结 | PASS | Release 基线构建；本包基线测试通过；环境、输入 hash、worktree 在 `p0/` | workspace 聚合结果中的兄弟包历史失败不计入本包 |
| P1 scope/config | PASS | v6 五 profile、v5 迁移、唯一模式源、稳定 scope digest、完整配置与生成器；配置和 mutation 测试通过 | double-only v5 按设计拒绝迁移 |
| P2 主线 FGO | PASS（实现） | off 复用唯一 estimator/transaction/fixed-lag；noiseless、batch、fixed-lag、freshness 通过；20×60 s 位置/速度通过；`p7/fgo_final_v4`、`off_mc20_v3`、`off_full_v3` | 验收 FAIL：姿态仅 4/20 seed；3×600 s 长程姿态全失败，位置 1/3 repeat 失败；各轴/符号初值网格和 RPE NOT_RUN |
| P3 五模式 FDE | PASS（实现） | provider gate、完整 census、history/cache/action scope、38-run 故障矩阵；joint2 包含 15,840 个支持的 UWB×IMU pair；`p7/profile_matrix_v3` | 验收 FAIL：恢复段未选预定义 exclusion/bridge/二阶 action；历史正向恢复失败 |
| P4 PL/发布合同 | PASS（实现） | production/post-FDE/PL 双通道同合同；risk/certificate/deadline/scope 绑定；原子 ROS 消息与 fail-closed mutation 回归通过 | PL finite 比例通过，但 2 m/3 m 可用性仅 0–7.83%；正式资格保持 false |
| P5 全流验证 | PASS（工具与已执行矩阵） | 独立真值、原始 IMU/UWB、固定 seed/config、69/69 schema PASS；摘要见 `FINAL_STATUS.json` | active 20×60 s PL MC 和 3×12000 full 因 pilot 明显实时失败而 NOT_RUN；未以缩样本冒充完整实验 |
| P6 性能与优化 | PARTIAL | off 的 O(N²) ledger/全量 provenance 热点已修复；3×600 s core p99 2.18–2.58 ms，RSS <30 MiB；active 31-stage profiling完成 | active p99 110–410 ms，全部 mature epoch >40 ms；queue/RSS 时序和线上 arrival-to-publish NOT_RUN |
| P7 交付 | PASS | 五配置、统一 runner/analyzer、单元/集成/ROS/E2E 测试、README、实现报告、进度与紧凑证据齐备；当前工程回归 442/442 + Gate-D 14/14 PASS | 无真实硬件、真实数据、正式 calibration、Gate J 和独立评审，构成外部正式资格阻塞 |

## 最终证据与结论

- `p7/fgo_final_v4`：4 个短段场景；摘要 SHA-256 `e02d41dfbe7dad15e9644dcbb2dde2a6693434feda56ef000d73eb5e8ee2a8e8`。
- `p7/off_mc20_v3`：20 seed × 60 s；摘要 SHA-256 `d2032efab20e4edd98dec84928cea794118000327ac3224f8afea8c32e510ab6`。
- `p7/off_full_v3`：3 repeat × 600 s；摘要 SHA-256 `7862e165c056e4e84f99d024c7831965142d263f5dec531469197e51b7780bb8`。
- `p7/active_pilot_v3`：4 active profile × 600 epoch；摘要 SHA-256 `4ff72e43af2457c525ad4dff5781495a7c9bc5a021a60bc3eb1c1a4359f716ce`。
- `p7/profile_matrix_v3`：38 个 profile/fault run；摘要 SHA-256 `b26dbc3f52fae02fd486f7bc247418552248c6c9649e56d60f87ee85722386eb`。
- 完整机器可读索引：`FINAL_STATUS.json`；详细数值、热点和失败分母：`doc/UWB_IMU_FGO_FDE_IMPLEMENTATION_REPORT.md`。
- 原大型证据生成时回归为 436 tests；基线固化新增日志/launch 合同测试后，`catkin run_tests uwb_imu_pl --no-status` 为 442 tests、0 errors、0 failures、0 skipped；`PYTHONPATH=tools python3 test/test_gate_d_tools.py` 为 14/14 PASS。
- 最终 69 个 run 逐目录执行 `tools/validate_run_schema.py`，69/69 PASS。

严格结论：五模式和统一软件合同已实现并通过回归；off 正常输出实际优化状态且 PL 正确为 NOT_COMPUTED。完整系统验收未通过，原因是有真实测量证据的算法精度、active 实时性、可用性和恢复动作失败，而不是接口缺失。不得将该状态用作正式保护声明。

## 主要诊断与修复

1. UWB-only 历史 nominal IMU provenance 缺失：改为 nominal provenance 永久保留，仅故障列按 provider gate。
2. fixed-lag 双通道常数重复扣除：先给 pooled statistic 加 history constant，再拆 current/history；新增直接单测。
3. deadline 覆盖上游失败原因：改为追加 reason，并绑定末端证书身份。
4. off 长程 ledger O(N²) 与 RSS 增长：只扫描 frozen graph、提交后 prune ledger、benchmark 默认关闭全量 ledger dump。
5. v6 schema 对 off/COMMITTED/SKIPPED_PROFILE 误判：改为 profile-aware 验证，并检查 scope/contract/off PL。
6. invalid candidate 的 compact reason 丢失：无条件保留 fail-closed reason；300 epoch 最小复现与最终集验证通过。

## 外部阻塞与后续工作

- 正式保护资格需要真实硬件/传感器数据、lever-arm/时间同步/noise overbound/bridge calibration、罕见事件 campaign、Gate J 和独立评审；当前无法从仓库内合成证据替代，`formal_eligible=false`。
- 仓库内仍可继续的算法工作不是外部阻塞，但本轮验收结果明确为 FAIL：姿态/bias 可观性、增量 history/boundary、共享 hypothesis numerics、批量 pair candidate PL、恢复证据与 action gate 均需后续重构后重跑完整规模。
- active full benchmark 和 MC 不能在当前 pilot 性能下产生合格证据；若执行只会确认 deadline 分母近 100% 失败并额外消耗资源，因此按协议保留为 NOT_RUN，而不是缩样本或隐藏分母。

## 磁盘与清理

- 任务目录峰值约 1.8 GiB；最终仅保留五个最终证据集、`p0/p1/p2/p4` 的小型定位日志、状态 JSON 和 manifest。
- `p5/`、`p6/` 和 `p7/` 中被最终 hash 证据替代的早期 probe、partial、v2 和 aborted full 目录已删除；它们只用于缺陷链，不再作为可审查原始证据。
- 删除清单、删除前大小、最终占用和逐项重建命令见 `results/fde_profiles_p0_p7_20260923/ARTIFACT_MANIFEST.md`。

## 当前运行任务

无。所有启动的 benchmark、构建和测试进程均已结束；没有后台 ROS、runner 或清理任务。
