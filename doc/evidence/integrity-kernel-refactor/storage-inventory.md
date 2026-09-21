# storage-inventory — P3/B1（2026-09-21）

本文件只做**测量与可再生性评估**，不执行删除。`results/` 属历史 campaign 数据，
**本轮未删除任何内容**（另行决策）。

## 1. 仓库内体积（测量值）

| 路径 | 体积 | 内容 | 可再生性 |
|---|---|---|---|
| `results/`（合计） | **1.2 GB** | 历史 campaign（见 §2 明细） | 部分可再生（需重跑对应 benchmark/报告脚本与数据） |
| `doc/evidence/`（合计） | 11 MB | A/B 轮证据 | 见 §3 |
| `doc/evidence/integrity-kernel-refactor/` | 8.5 MB | 本目录（raw/ 已按 prune-log 压到 7.5MB） | 是（prune-log 给出再生命令） |
| `doc/evidence/` 其余（P2 之前的历史证据目录） | 2.5 MB | 早期 gate/closure 报告 | 是（脚本 + 配置） |
| `build/`（catkin 工作区外部） | 3.2 GB | 构建产物 | 是（`catkin build`） |
| `devel/`（catkin 工作区外部） | 1.3 GB | 开发安装空间 | 是（`catkin build`） |

本轮任务约定：`doc/evidence` 单文件 ≤1MB、整目录 ≤20MB；大产物写 `/tmp/uwb_imu_pl_b1_<date>/`。

## 2. `results/` 明细（**不得删除**）

| 目录 | 体积 | 备注 |
|---|---|---|
| `results/r0-r1` | 360 MB | R0/R1 开发 campaign |
| `results/integrity-closure-final` | 238 MB | 闭合验证 campaign |
| `results/advisor_report_focused_pl_20260903` | 194 MB | 顾问报告（聚焦 PL） |
| `results/gate_d_p0_p2` | 164 MB | Gate D P0–P2 |
| `results/gate_d_p3_p6` | 78 MB | Gate D P3–P6 |
| `results/advisor_report_0ed90a0d32c2_36e89b244984` | 65 MB | 顾问报告（早期） |
| `results/week4_0ed90a0d32c2_41f1449abe28` | 48 MB | week4 基线 |
| `results/week4_development` | 17 MB | week4 开发 |
| `results/r0-closure-r1-numerics` | 11 MB | R1 数值证据 |
| `results/p2_monte_carlo` | 912 KB | P2 蒙特卡洛 |
| `results/stage1_baseline_a4ab8ec0ba87` | 996 KB | Stage1 基线 |
| `results/week4_c9bbcb75a543_41f1449abe28` | 60 KB | week4 变体 |

评估：这些都是**历史一次性 campaign 输出**，其中 `r0-r1`、`integrity-closure-final`、
两个 `advisor_report_*` 与 `gate_d_*` 合计占 1.1 GB。绝大多数是逐帧 CSV 与中间产物，
按当前工具链**可以重跑**，但重跑需要原始配置/场景与相当机时；其中部分早期 campaign
的配置已不在仓库（例如具体 CLI 参数），因此**默认视为不可完整再生**，本轮只记录、不删除。

## 3. `raw/` 清理结果（本轮执行，详见 `prune-log.md`）

| 项 | 清理前 | 清理后 |
|---|---|---|
| `raw/` 合计 | 98 MB（P2 结束） | **7.5 MB** |
| 其中 fixture bin | 8 个 v5 bin，13.9 MB | 5 个 v5 bin，6.93 MB |
| 逐帧 CSV（runs/runs_p2/replay/replay_p2） | 约 84 MB | 0（摘要保留于 `raw/summaries/`） |
| census（P1/P2） | 6.6 MB | 0（再生命令在 prune-log） |
| 构建/测试日志 | 0.1 MB | 保留 |

被删文件的 sha256 与再生命令见 `prune-log.md`（692 条）。

## 4. 本轮之后

- 任务结束时的 `du` 数值会在最终报告中给出（`doc/evidence` 整目录与 `raw/`）。
- 若后续需要进一步压缩：`results/gate_d_*` 与 `results/advisor_report_*` 可先归档到仓库外
  （它们已不再被任何测试或工具引用）；`build/`、`devel/` 可用 `catkin clean` 释放 4.5 GB，
  但会拖慢下一轮构建，建议保留。
