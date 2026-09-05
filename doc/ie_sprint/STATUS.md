# UWB-IMU-IE sprint 状态

本文件是工程协作与任务交接的统一入口。它记录事实和任务状态，不代替方法合同或实验合同。

## 冻结材料

按以下顺序阅读：

1. 冻结论文结构：[`../v2/paper_structure.tex`](../v2/paper_structure.tex)
   SHA-256：`8ac373919823d755d9c1b4ceb807527432d16b69d368042e19aa56d93dc67144`
2. 执行路线图：[`../v2/v2_roadmap.md`](../v2/v2_roadmap.md)
   SHA-256：`b9bb63b65ebdb8a505bf99b26181318c6c64da1a72817b08cfa61315e2333bdb`

roadmap 所称的 `UWB_IMU_IE_System_Centered_Structure_v3.tex` / `Structure_v3`，在本仓库对应
`doc/v2/paper_structure.tex`。除非 amendment 表明确登记并获确认，上述文件视为冻结输入。

## 项目定位

本项目是可复现的 UWB–IMU 全轨迹后处理系统：复用现有 C++/GTSAM 后端，实现非负 L1/TV 支撑发现、去正则分段 refit、在排除全部候选的共同参考图上进行 `eta/s/gamma` 评分，以及组级 Use/Suppress 和最终联合推断。实验分别用于验证系统能力、多链路 NLOS 效果、门控价值和未来观测作用。

## 仓库快照

以下是启动准备开始前的只读快照，不代表 T00 验收：

| 字段 | 值 |
|---|---|
| 日期（UTC） | `2026-09-05` |
| 分支 | `feature/uwb-imu-fusion-ie-postprocessing` |
| 提交 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014` |
| T00 开始时工作区 | dirty：已有未跟踪 `AGENTS.md`、`doc/ie_sprint/STATUS.md`、`doc/ie_sprint/REPO_AUDIT.md`；无 tracked/staged 改动 |
| roadmap 参考提交 | `cfe6d29` |
| `cfe6d29..HEAD` 差异范围 | 仅文档（`doc/`） |

上述已有未跟踪文件和旧实验结果均已保护；T00 新证据继续保持未提交。

## 当前进度

状态词：`NOT_STARTED`、`IN_PROGRESS`、`BLOCKED`、`DONE`。`DONE` 只在对应验收证据齐全时使用。

| 项目 | 状态 | 产物/说明 |
|---|---|---|
| T00 前最小启动准备 | `DONE` | `AGENTS.md`、本文件、[`REPO_AUDIT.md`](REPO_AUDIT.md) |
| T00 本地审计与旧基线复现 | `BLOCKED` | 已完成环境、数据和源码审计；当前构建缺 `uwb_driver`，本机 GTSAM 仅 4.0.3；预存二进制 ABI 载入失败，轨迹/ATE 未生成。见 [`REPO_AUDIT.md`](REPO_AUDIT.md) |
| T01 唯一合同与论文工程 | `NOT_STARTED` | T00 的代码/数据事实已形成，可以开始；固定 `beta`、数据权限和构建环境须作为未决项保留 |
| T02–T13 | `NOT_STARTED` | 按 roadmap 顺序与前置推进 |

## 下一任务

下一任务可以是 **T01：从冻结框架生成唯一实现合同，并建立论文工程**。roadmap 对 T01 的前置是
T00 已取得的代码/数据事实，并不要求 T00 已标 `DONE`。T01 必须把独立 LOS 固定 `beta`、数据发表
权限和当前构建环境登记为未决项，不得编造数值；这些缺口补齐前 T00 仍为 `BLOCKED`，依赖当前
工程运行的 T02 及后续验收也不能据此宣称通过。

T01 预登记产物如下，本次不创建空文件：

- `doc/ie_sprint/METHOD_CONTRACT.md`
- `doc/ie_sprint/EXPERIMENT_CONTRACT.md`
- `paper/main.tex`（可编译论文入口）
- `paper/CLAIM_EVIDENCE.md`

## 任务交接

每次任务结束追加一行；“验证”必须区分实际运行与 `NOT_RUN`。

| UTC 日期 | 任务 | 提交/工作区 | 改动与产物 | 实际验证（命令、退出码、证据） | `NOT_RUN`/风险 | 下一步 |
|---|---|---|---|---|---|---|
| 2026-09-05 | T00 前最小启动准备 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014`；新增文档未提交 | 建立协作约束、统一入口和预审计模板 | 链接目标检查、`sha256sum -c`、`git diff --check` 均 exit 0；另以 `git diff --no-index --check` 检查 3 个未跟踪新文件，无空白错误 | build/test/baseline/trajectory/ATE/runtime/memory 全部 `NOT_RUN` | 执行 T00 |
| 2026-09-05 | T00 本地审计、数据清点与旧基线尝试 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014`；开始时已有 3 个未跟踪文档，结束时仍未提交 | 更新 [`REPO_AUDIT.md`](REPO_AUDIT.md)；新增隔离证据目录 [`evidence/t00_20260905T102745Z/`](evidence/t00_20260905T102745Z/)；未修改源码、配置、CMake 或冻结材料 | 当前 workspace `catkin build uwb_imu_fgo --no-status` exit 1（缺 `uwb_driver`）；GTSAM 4.2 CMake probe exit 1（仅 4.0.3）；数据 bag 健康检查 exit 0；预存节点启动 exit 127（GTSAM ABI 缺符号） | tests、trajectory、ATE/RPE、estimator 规模/耗时/峰值内存、重跑差异均 `NOT_RUN`；独立 LOS 标定与发表权限未建立；T00 `BLOCKED` | 可基于审计事实开始 T01；并行恢复依赖后重建、测试并重跑 baseline |

## Scope amendments

任何会改变冻结数学定义、证据边界、数据角色、指标或任务范围的决定都必须先登记。没有记录即表示无 amendment。

| ID | UTC 日期 | 提议/决定 | 原因与证据 | 影响的合同/代码/claim | 确认者 | 状态 |
|---|---|---|---|---|---|---|
| — | — | 当前无 amendment | — | — | — | — |
