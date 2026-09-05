# UWB-IMU-IE：七天代码、实验与论文交付 Roadmap v2

> **交付目标：第 7 天结束前，得到可运行的方法实现、可追溯的仿真与数据集结果、完整可编译论文和最小复现入口。不是第 7 天才决定是否开始正式实验。**
>
> 基准论文：`UWB_IMU_IE_System_Centered_Structure_v3.tex`。本计划不重新设计论文主题。  
> 编制日期：2026-09-05。D1–D7 是你实际执行的七个工作日；若当天开始，对应 9 月 5–11 日。这里没有假定或核实具体会议截止日。  
> 资源假设：你已有可工作的 IE 开发机和至少一种可读取的数据。你的本地代码、数据和 ROS/GTSAM 环境由 T00 核验；本计划不把远程源码中出现的接口当成已成功运行的实验。

## 阅读导航

- [0. 先回答：Python test 还是在现有系统里开发？](#s0)
- [1. 七天执行总表](#s1)
- [2. 仓库审查：先解决这些接入风险](#s2)
- [3. GPT、Codex、你分别负责什么](#s3)
- [4. 本周新增模块与数据合同](#s4)
- [5. T00–T13：逐步任务卡与可复制提示词](#s5)
- [6. 实验预算、指标及证据判定](#s6)
- [7. 每天收工前的检查与失败处理](#s7)
- [8. 工具调用模板、命令约定与 AGENTS.md](#s8)
- [9. 第七天交付清单](#s9)
- [10. 依据、源码与核验边界](#s10)

<a id="s0"></a>
## 0. 先回答：Python test 还是在现有系统里开发？

**两者都需要，但不是两套完整算法。**

本周只维护一个正式估计器：**现有 C++ / GTSAM IE 后端，加上新的 NLOS 模块。** Python 只负责小矩阵参考解、实验编排和结果分析，不另写一套完整 UWB–IMU smoother。

| 层级 | 实际运行什么 | 能证明什么 | 不能冒充什么 | 完成时间 |
|---|---|---|---|---|
| L0：数值参考 | Python/NumPy 小矩阵，解析反例 | `R_c / eta / s` 的公式与秩处理基本正确 | 不等于方法已接入 IE | D1，限一个短任务 |
| L1：后端最小闭环 | **仓库内 C++/GTSAM**；小图或短记录；提供已知候选段，联合估计轨迹与 `c_s` | 新因子、白化、变量排序、评分、决策、最终图能协同工作 | Oracle 支撑结果不是端到端性能 | D2 必须完成 |
| L2：正式端到端 | **同一个 IE 后端**；自动 discovery → refit → score → final | 从正常输入得到自动处理结果 | 不可暗中读取真实 bias 或遮挡标签 | D3 必须完成 |
| L3：论文实验 | 冻结的 IE 版本；仿真、真实/公开数据；统一评估 | 支撑论文贡献的实际证据 | 运行成功不等于优于基线或可投稿 | D4–D5 |

这里的“核心算法先跑通”，明确指 **L1，而不是停在 L0**。

L1 可以暂时给定“哪个 link 的哪个时间段是候选”，但**不给真实偏置幅值、真实轨迹初始化或真值轨迹先验**。`c_s`、轨迹以及其他不确定状态仍由后端估计。到 D3 再把 oracle provider 换成正式 discovery provider，后面的 Stage 2–4 不重写。

**最省返工的顺序是：**

```text
已有 IE 基线复现 ────────────────→ 真实数据/公开数据健康检查
        │
        ├→ 小矩阵 golden tests
        │         ↓
        └→ 测量保留与固定 β → C++ 分段因子 + oracle refit
                              ↓
                     GTSAM Jacobian → 稀疏评分 → 最终联合图
                              ↓
                     替换为自动 support discovery
                              ↓
                    冻结参数 → 仿真/数据集 → 图表 → 完稿

论文和复现记录：从第一步开始，每天同步更新。
```

### 与 Claude v1 相比，本计划改变了什么

保留其正确的数学边界和四阶段定义，但做五个执行层面的调整。[U1][U2]

1. **从“按论文阶段串行开发”改成“先集成风险最大的短闭环”。** 评分不能等全部 discovery 写完才接入 GTSAM。
2. **真实数据基线在 D1 就跑。** 不等仿真方法成功后才发现数据读错、锚点不可信或坐标系不一致。
3. **先补测量、标定、输出语义，再加公式。** 这些是当前源码中可见的具体接入问题，不是泛泛的工程洁癖。
4. **把实验压成缓存复用的最小矩阵。** 不做全参数笛卡尔积，不把所有 gate 阈值都变成一次完整后端重跑。
5. **每天都有论文交付物；D6 是完整稿，D7 不再开发功能。** 不保留“下一周再正式集成/采集/撰写”的隐含安排。

<a id="s1"></a>
## 1. 七天执行总表

“上午/下午”只表示先后顺序，不要求你从上午开始。D1 若只剩半天，优先保证基线、输入合同和数值测试；不要把整天花在写规格书上。

| 天 | 主执行任务 | GPT 用在哪里 | Codex 用在哪里 | 当天必须看见的结果 | 论文当天产物 |
|---|---|---|---|---|---|
| **D1** | T00 审计；T01 冻结；T02 开始；T03 数值参考 | 审核数学合同、锁定证据边界 | 本地构建与基线复现、输入修补、小测试、LaTeX 工程 | 旧系统基线报告；数据清单；golden tests；新功能关闭时不改变旧路径 | 全文可编译骨架；System 初稿；Method 公式与证明草稿 |
| **D2** | T02 完成；T04 分段 refit；T05 评分；T08 oracle 路径；T07 最小数据注入 | 只审查数学或实验语义的阻塞问题 | 在 IE 内完成 oracle 短闭环 | 同一短输入得到 `c_s、R_c、eta、s、gamma、Use/Suppress` 和最终轨迹 | Method 主体与算法伪代码，不等自动 discovery 完成 |
| **D3** | T06 discovery；T08 最终审计；T09 runner/基线；T10 开始 | 审核预声明实验合同 | 自动闭环、基线模式、日志与缓存、验证集小实验 | 非 oracle 自动运行；无前端误删；一组真实数据新方法 smoke test | Method 与代码逐项对齐；Experiments 设置完整 |
| **D4** | T10 gate 验证；T11 future context；T12 主实验 | 审阅验证集证据；冻结贡献措辞 | 冻结阈值、跑测试、生成首版表图 | gate 锁定；RQ3 初步结果；至少一组 prefix/full 结果；主表初版 | Results 初稿；Related Work 引文核实完成 |
| **D5** | T12 补齐最小矩阵；T13 结果落文 | 根据真实结果收窄论断，不再扩方法 | 真实/公开数据正式跑；统计、图表、结果导出 | RQ1–RQ4 各有实测证据或明确不足；结果版本冻结 | 完整结果段、Discussion；所有必要图表进入正文 |
| **D6** | T13 完稿与证据审计 | 论文全文逻辑与科学性审查 | LaTeX 写作、图表排版、符号与引文检查、复现演练 | **无空结果表的完整 PDF v1**；代码版本与表图能互相定位 | 完整论文，不再是 outline |
| **D7** | 只修 correctness / 排版 / 复现问题 | 审查剩余高风险 claim | 最终构建、最小复跑、匿名/制品核对 | 最终 PDF、源码提交、配置、测试/实验日志、复现入口 | 定稿；没有未完成承诺伪装成贡献 |

### 四个不可后移的时间点

**D1 收工前：** 数据和编译环境是否可用，必须有实际答案。  
**D2 收工前：** C++/GTSAM oracle 短闭环必须存在；只有 Python 图不算。  
**D3 收工前：** 自动端到端必须存在；否则 D4 不启动大规模 sweep。  
**D5 收工前：** 冻结论文主结果。D6–D7 的默认工作是写作与检查，不是挽救新功能。

本周不默认新增物理采集。已有受控 UGV 数据优先复用；确实缺一条必要对照时，只在数据健康检查后安排最小补采。**新硬件、物理重布和第二个新数据集不能进入关键路径。** 若冻结结构中的两种 UGV 运动轨迹实际不存在，应登记为证据缺口，明确调整实验叙述，不能把计划写成已完成。

<a id="s2"></a>
## 2. 仓库审查：先解决这些接入风险

### 2.1 核验边界

本计划已通过公开源码审阅 `types/config/graph_builder/uwb_factor/optimizer/data_loader/initializer/trajectory_io/logger/run_offline/CMakeLists/simulator` 等相关文件。GitHub 提交页显示的本次参考提交为：

```text
cfe6d29cc2a0077d9a0b958461d1615aa5e9079e
```

**这是静态审阅，不是编译或运行验收。** 当前会话中的容器无法完成 Git 网络克隆，也没有执行你的 ROS/GTSAM 工程；本地未提交改动、数据可用性、运行成功率和耗时都必须由 T00 实测。源码依据见 [R0]–[R13]。如果你本地版本不同，以本地审计报告为准，记录差异。

### 2.2 与本周任务直接相关的差距

| 位置／证据 | 已核验的源码行为 | 对冻结方法的影响 | 本周动作 |
|---|---|---|---|
| `src/outlier_filter.cpp::PreFilter` [R3] | RSSI 差值超过阈值直接删除测距 | 后端可能根本收不到要恢复的 NLOS | 论文路径把“格式/量程无效”与“疑似 NLOS”分开；疑似 NLOS 保留并打标 |
| `tools/run_offline.cpp` [R1] | 过滤后才按帧下采样；已有 GNC 多趟流程和持续可视化循环 | 各策略可能使用不同关键帧；批处理不自动退出 | 新增薄的 paper runner，统一关键帧计划、无可视化退出、独立输出目录 |
| `src/uwb_factor.cpp::MakeUwbFactor` [R4] | `calib_bias=false` 时预测直接使用几何距离 | “关闭在线 β 标定”并不等于“使用非零独立标定 β” | 增加固定 β 常量路径；不要用极强先验假装固定标定 |
| `src/graph_builder.cpp` [R5] | `Z(m)` 已作静态 range bias，`B(k)` 作 IMU bias；动态 segment bias 未在此接口出现 | 新 bias key 可能冲突；`b_km` 与 IMU bias 易混 | 单独分配 segment key；记录状态维度和 measurement→factor 映射 |
| `src/optimizer.cpp` [R6] | `Optimize()` 包含预热 LM、GNC/TLS、硬剔除和再优化 | 直接复用整个函数会改变四阶段方法的因子集合 | 复用 LM 参数/基础组件；Stage 2/4 走显式 refit，不再隐式 GNC/reject |
| `src/logger.cpp` [R7] | 现有残差按位置到 nominal anchor 计算；协方差 CSV 写 `-1` 占位 | 不能直接拿旧 residuals 当 `gamma`，不能把占位 CSV 写成不确定度结果 | 新结果从实际 factor/最终状态导出；缺失值带状态，不伪装为数值 |
| `src/trajectory_io.cpp` [R8] | 主 ATE 路径先做 SE(3) 对齐；RPE 实现需核对时间间隔含义 | 不等于冻结框架要求的独立坐标系 raw-frame 评估 | 新评估器区分固定变换误差与 aligned ATE，检查时间配对 |
| `src/data_loader.cpp`、`config.h` [R2] | 存在 original/MCD/VIRAL/VIUNet/MILUV/SFUISE 分支 | 不能只按 README/MCD 推断可用数据；多 tag 杆臂语义仍需核对 | 只挑本机已有且标定关系可信的一种外部数据，先 smoke test |
| `README.md` MCD 段 [R9] | 示例锚点由该序列 GT 与测距拟合，注明仅作接口/集成测试 | 使用该配置不能声称独立锚点的公平定位 benchmark | 限定为 portability，或换有独立锚点标定的数据；不直接采纳其 ATE 为主证据 |
| `CMakeLists.txt` [R10] | 要求 GTSAM 4.2，并列出 `uwb_driver`、`isas_msgs` 等依赖 | 不能照 README 的较宽版本描述准备新环境 | 锁定本机已工作的依赖，不在本周迁移 ROS/GTSAM |
| `simulator/src/uwb_twr_sim.cpp` [R11] | 随机 NLOS、噪声/时钟项；`random_seed=0` 不覆盖随机初始化 | 未必得到可重复的持续 bias；随机项可能污染 latent truth | 用固定非零 seed；新增确定性 step/ramp 注入并导出逐观测真值 |

**补充检查，不扩大重构：** 老入口的 covariance 段会再次调用优化器，输出又有 `best_values` 路径，必须避免“轨迹来自一个解、协方差来自另一个解”。Pose3 的位置不确定度也不能直接读前三个切空间坐标；GTSAM 4.2 的旋转/平移顺序为前 3／后 3，世界位置协方差应通过位置 Jacobian 传播。[R1][R12] 本周由新 paper 路径保证一致，不需要顺手重构所有旧可视化。

<a id="s3"></a>
## 3. GPT、Codex、你分别负责什么

这里的“GPT”指你用来审阅材料、检索和做数学/论文判断的对话；“Codex”指**能访问你本地仓库与开发环境**的编码会话。本文给出调用顺序和任务提示词，不假定当前对话能够替你启动另一个 Codex 会话。

| 角色 | 本周的主要任务 | 不该交给它独立决定的事 |
|---|---|---|
| **GPT：研究与审查** | 从冻结结构提炼合同；检查公式/假设；有针对性核实文献和数据协议；审查结果是否支持 claim；给论文修改意见 | 不根据 README 宣称运行通过；不随意添加新 gate/新定理；不编造 benchmark 数值 |
| **Codex：实现与证据生产** | 审计实际代码；按合同编程；执行测试；运行实验；产出 CSV/图/LaTeX；依据审稿意见修改文件并编译 | 不自行改变科学问题；不以降低测试标准解决失败；不把 TODO 或 stdout 当真实实验完成 |
| **你：负责人** | 提供本机数据路径/标定来源；选择可用记录；审查 diff 和验收证据；批准 scope 变更；控制提交与数据权限 | 不靠肉眼看一条漂亮轨迹就批准方法；不在测试集上反复调阈值 |

### 推荐的会话组织

默认只维护 **一个 Codex 工程主会话 + 一个 GPT 审查会话**。每次完成一个小任务，再进入下一个；这比同时让多个 agent 改 `graph_builder/config/CMake` 更可控。

已有并行工作经验时，最多增加一个 Codex 评估/写作会话，限制其改动为 `tools/paper/`、`paper/`、实验文档；共享接口由主会话先落地。worktree 可以隔离改动，但不会消除合并和接口冲突。[O1][O2] 不要把相同 ROS package 的多个 worktree 一起放入同一个 catkin 工作空间的 `src/`。

### 每次交接只传这一组材料

```text
冻结论文 + METHOD_CONTRACT.md
当前任务卡编号
相关源码或 commit/diff
对应测试/实验日志与 CSV
STATUS.md 中的已完成项、失败项、下一步
```

GPT 看不到你本地新代码时，必须上传/粘贴对应 diff 与日志；不要假定聊天上下文自动同步到 Codex。Codex 的完成回复必须包含：**改了什么、运行了什么、真实结果、没运行什么、剩余风险**。

### 最小文档集合

```text
AGENTS.md                              # 短的长期约束，已有则合并，不能盲目覆盖
 doc/ie_sprint/REPO_AUDIT.md             # 本地事实、数据路径/标定状态、旧基线
 doc/ie_sprint/METHOD_CONTRACT.md        # 唯一算法合同，含公式、接口、验收测试
 doc/ie_sprint/EXPERIMENT_CONTRACT.md    # 划分、阈值、基线、指标、实验预算
 doc/ie_sprint/STATUS.md                 # 当前进度与 scope 变更
 paper/CLAIM_EVIDENCE.md                 # 每个 claim → 结果文件/图表/源码
```

不要再先写十份设计报告。本 roadmap 的任务卡已经是执行计划。每个文档从小而完整的版本开始，后续补实际证据。

<a id="s4"></a>
## 4. 本周新增模块与数据合同

### 4.1 推荐接法：在同一工程内新增薄入口，不重写 IE

**以下路径是建议新增的接口，不是声称仓库已有这些文件。** 本地 Codex 可按实际组织调整名称，但应在 T01 记录对应关系。

```text
现有 DataLoader / Initializer / IMU preintegration / UWB geometry
                         │ 复用
                         ▼
tools/run_ie_paper.cpp              # 新薄入口，新增 target，运行结束退出
                         │
include/uifgo/nlos_types.h          # ID、segment/group、score、status
src/nlos_discovery.cpp              # Stage 1：交替优化中的 fused nonnegative 子问题
src/nlos_refit.cpp                  # Stage 2/4：无 L1/TV 的受约束联合 refit
src/nlos_recoverability.cpp         # Stage 3：分组、参考图、F/G、eta/s/gamma
src/nlos_pipeline.cpp               # 编排、冻结决策、最终审计、fallback
                         │
tools/paper/                       # 数值参考、runner、评估、图表/表格生成
 test/test_nlos_*.cpp               # GTest，接入现有 catkin 测试
 config/paper/                     # 验证后的配置，不能只复制默认值
 paper/                            # 实际正文与生成的图表
```

可将小文件合并，**不要为了模块图而增加抽象层**。必要改动仍进入共享 `uwb_factor/graph_builder/config`；旧接口用 overload/default 保持兼容。新 runner 必须调用这些共享组件，不复制一套预积分或几何实现。

### 4.2 从第一天就固定这些字段

| 对象 | 最少包含 | 关键约束 |
|---|---|---|
| `ObservationRecord` | 稳定 `obs_id`、传感器时间、tag/anchor ID、raw range、nominal sigma、keyframe ID、有效性/怀疑标记 | `obs_id` 在过滤、分段、重建图和所有策略间保持稳定 |
| `Segment` | `segment_id`、link、观测 ID 列表、起止时间、数量、`c_key`、boundary/short 状态 | link 至少区分 tag–anchor；主实验可限制单 tag，但要显式声明 |
| `Group` | `group_id`、segment IDs | 重叠区间连通分量；整组接受或抑制，不偷偷改成逐段 |
| `FactorMeta` | factor index → obs_id / factor type / keys | 重建图后重新映射；禁止把旧 factor index 当永久测量 ID |
| `ScoreRecord` | `eta,s,lambda_min,gamma_max,rank,status,linearization_id` | decision 与 final audit 分开；`inf`/无效不序列化成非法 JSON 数字 |
| `InferenceResult` | 最终 `Values`、最终 graph/mask、status、score sets、耗时 | 轨迹、bias、残差、协方差从同一个结果对象读取 |

**名义噪声与关键帧计划必须先于策略掩码冻结。** 当前 `AdaptiveSigma()` 依赖观测时间间隔；如果每种策略删完数据后再计算，会同时改变权重。主实验应预先确定所有输入测量的 `sigma_i`，各模式复用；`gamma` 不允许使用为当前残差自适应膨胀的噪声。[R5][U1]

### 4.3 非负约束的实现必须讲清

冻结框架要求 `c_s >= 0`。**普通无约束 LM 后把负数截到零，不等于完成了受约束联合优化。**

本周建议采用容易验证的交替 refit：

1. 固定当前 `c`，对导航状态做条件 LM；
2. 固定导航状态，对各不相交常值段做闭式更新：
   \[
   c_s\leftarrow\max\!\left(0,\frac{\sum_{i\in S_s}w_i[z_i-h_i(X)-\beta_i]}{\sum_{i\in S_s}w_i}\right);
   \]
3. 检查联合目标、状态更新及边界 KKT/投影梯度，达到预声明停止条件。

这是**本 roadmap 建议的求解器实现选择**，不是冻结论文已证明的全局收敛算法。若已有正确的 bound-constrained/active-set 实现，优先复用，不同时实现两套。

条件 LM 子步骤暂时固定 `c` 是允许的；**最终必须保存含 `c` 的完整联合图并以该图计算信息**。不能将最后一个条件导航子图冒充联合后验。非负边界由真实约束产生，不向信息矩阵添加人为巨大精度。

<a id="s5"></a>
## 5. T00–T13：逐步任务卡与可复制提示词

每张任务卡都按 **输入 → 动作 → 产物 → 验收 → 下一步** 执行。下文的时间限制是你分配本周工作量的上限建议，不是工具执行时间保证。

### T00｜审计本地工程，复现旧基线，清点数据

**时间：D1 第一项；负责人：Codex + 你。前置：无。**

**输入：** 本地仓库、冻结 `.tex`、本 roadmap、现有可运行配置与数据。

**做什么：**

- 检查 `git status/rev-parse`、构建命令、GTSAM/ROS/编译器版本、现有测试；不升级依赖，不丢弃本地改动。
- 用一条你实际拥有的短记录复现旧系统。原入口持续可视化时，基线采集可以人工正常结束，但不能把后续 paper runner 设计成一直等待 Ctrl+C。
- 清点自有 UGV、LOS 标定、仿真记录、已有外部数据。每项记录路径、时间长度、传感器、GT 所代表的刚体/点、锚点/杆臂/时钟来源、是否可发表。
- 阅读真正使用的 loader，检查单位、坐标轴、tag、时间戳。优先选本机已经下载且可通过检查的数据，而不是临时下载五个数据集。
- 记录旧系统耗时和内存量级；把 baseline 输入与结果归档。README 的测试数量不作为验收值。

**产物：** `REPO_AUDIT.md`、baseline 日志与结果路径、`STATUS.md`，以及一份事实化的数据角色表。

**完成标准：** 有实际 build/test 命令及退出码；至少一条记录有轨迹和评估结果；数据缺口逐项可见。未运行的测试明确写 `NOT_RUN`。

**卡住怎么做：** 环境问题优先修现有依赖与路径，不新建第二套平台。外部数据缺独立锚点时先标 portability-only，仍推进自有数据与仿真，不花整天“修出一个漂亮 ATE”。

**发给 Codex：**

```text
执行 roadmap T00。先读冻结结构和仓库，不开发 NLOS 模块。
审计本地提交/未提交改动、实际构建依赖、测试入口、现有数据配置。
尝试构建并复现一条现有短记录；记录真实命令、退出码、结果和耗时。
清点自有UGV/标定/仿真/外部数据，区分接口存在、跑通、可作公平benchmark。
重点核对RSSI前置删除、固定beta缺口、旧日志占位、最终graph/values一致性。
只修阻止旧基线运行的小问题，不重构；输出REPO_AUDIT.md和STATUS.md。
结束时列出已运行与未运行项目，不把源码存在写成实验通过。
```

**下一步：** 把审计报告交给 GPT 执行 T01。数据健康检查可继续，不阻塞小矩阵测试。

---

### T01｜从冻结框架生成唯一实现合同，并建立论文工程

**时间：D1；负责人：GPT 审核 → Codex 落地。前置：T00 的代码/数据事实。**

**做什么：** 把 [U1] 的定义转成 `METHOD_CONTRACT.md`，只补实现必需空白，不再改论文重心。

合同必须固定：`beta` 来源与固定值、非负求解方式、候选支撑/分段/跨 gap 规则、完整 nuisance 列表、所有候选排除的参考图、分组方式、名义权重、秩容差/列尺度、`gamma`、组级决策、最终联合图、一次 fallback，以及每项对应测试。

同时建立 `EXPERIMENT_CONTRACT.md` 的第一版：哪些记录用于标定/验证/测试；何种指标能由何种数据支持；最大实验预算。**数值门限此时可以标“待验证集锁定”，但来源和锁定流程必须明确，不能填一个未经核验的 magic number。**

Codex 将冻结 `.tex` 复制进 `paper/` 作为工作稿，保留原文件；保留已有 proposition 的证明，不新增独立理论大章。每天的实际实现与实验结果写入正文，内部计划文字逐步移出主文。

**产物：** 两份短合同、可编译的 `paper/main.tex`、`paper/CLAIM_EVIDENCE.md` 初版。

**完成标准：** 每个符号有单位、输入和对应代码对象；一个候选段从发现到最终图的状态流无歧义；论文已能编译。不能把“冻结”理解成禁止修 correctness：必要变更进入 `STATUS.md` 的 amendment 表。

**发给 GPT：**

```text
请仅基于冻结的Structure_v3、REPO_AUDIT和roadmap T01生成实现合同。
保持系统为主、四阶段、共同候选排除参考、eta+s+经验gamma、组级Use/Suppress、
接受c仍在最终联合图的边界。区分论文原有规定与本周建议的工程选择。
补全接口/单位/秩处理/非负求解/最终审计/测试，不新增科学贡献。
列出必须由验证集确定的参数与不能从现有数据支持的claim。
给出可直接存入METHOD_CONTRACT.md和EXPERIMENT_CONTRACT.md的内容。
```

**然后发给 Codex：**

```text
落地T01合同与paper工程。不要重新解释合同或增加模块。
保留原冻结tex；创建工作稿并编译，补System和Method中已有事实，结果保留显式待填。
建立claim→预期证据映射，并在STATUS记录尚未实现/验证的部分。
```

---

### T02｜打通论文专用输入路径：不丢候选、正确固定 β、可批处理

**时间：D1 开始、D2 先完成；负责人：Codex。前置：T01。**

**做什么：**

1. 在现有工程新增薄 paper runner，复用 loader、初始化、图构建和预积分。保留旧入口，避免破坏既有系统。
2. 新增明确策略开关，不靠把 RSSI 阈值设成极大数来表达“关闭”。有效性过滤只去掉非有限值、损坏数据、未知锚点及预声明的无效量程；疑似 NLOS 保留。
3. 为每条输入测量分配稳定 ID，**先确定关键帧/测量计划和 `sigma_i`，再应用不同方法掩码**。记录原始数、有效数、进入估计的数，别把采样丢失计为 NLOS suppression。
4. 给共享 factor 增加 `fixed_beta_by_link` 或等价显式常量路径。初始化和 residual helper 也必须遵守同一 β 约定；原始 `z` 保留，不重复减 β。
5. 入口运行结束正常退出；输出使用唯一 run 目录，不复用 `data/trajectory.txt` 或 `logs/latest` 作为批量结果索引。
6. 先只支持本周主实验需要的单 tag 及固定外参配置。多 tag 参数若未真正进入 factor，就限制 tag，而不是假装支持。

**产物：** 新 target/入口、配置字段、测量元数据、固定 β 支持；旧系统回归测试。

**完成标准：** 人造高 RSSI 差值但量程合法的样本仍到达候选池；`fixed_beta=0.3m` 在因子残差中正确出现且未创建在线 β 变量；两种策略的 keyframe/input-plan hash 一致；程序完成后退出并写完文件。

**发给 Codex：**

```text
执行T02，在同一IE工程增加薄paper runner，不另写估计器。
实现稳定obs_id、策略无关关键帧计划、名义sigma快照、有效性与NLOS标记分离、
显式固定beta常量路径、唯一输出目录和正常退出。复用原loader/initializer/IMU/factor。
不调用会隐式GNC剔除的旧多趟流程作为Stage2/4。
写测试验证疑似NLOS没有提前丢失、beta不被漏用/重复减、旧接口兼容。
给出构建与运行新target的已验证命令；尚未支持的配置应报错而非静默忽略。
```

---

### T03｜建立短小的 Python golden reference

**时间：D1；负责人：Codex 编程，GPT 只查数学分歧。前置：T01；不依赖 T02 完成。**

**范围上限：一个短任务，建议不超过约 60–90 分钟。** 不安装 Python GTSAM，不用 Python 重写 IMU smoother。

实现 `R_c=E^T E`，其中 `E=(I-P_F)G` 由小矩阵 SVD 投影得到；输出 `N=G^T G`、`eta`、`s`、数值秩和状态。

**必测：** 已知导航；标量混淆但 nuisance 块可逆；与 bias 无关的 nuisance 零列；近退化 multi-bias；`N=R=1e-4` 的高 eta/大 s；静态 β 平移歧义；满秩 projector/Schur 等价；固定模型下未来信息增加。这里不是计算论文性能。

**产物：** `tools/paper/reference_recoverability.py`、测试、保存的小矩阵 fixtures。必要时同时保存 `F/G/N/R` 以供 C++ 对照。

**完成标准：** 解析例子通过；秩亏给 `s=inf`；病态情形不偷偷通过加对角正则“修好”。比较用预声明绝对/相对容差，不要求不同库 bitwise 相同。

**发给 Codex：**

```text
执行T03，仅写NumPy小矩阵golden reference和测试，不实现完整定位器。
输入F/G，按SVD列空间投影得到E和R=E.T@E，再输出N、eta、s、rank、status。
覆盖roadmap列出的解析反例，尤其可逆A但R=0、高eta但s=100m、无关nuisance零空间。
秩亏报告inf/状态，不用LM或jitter创造信息；保存可供C++读取的fixture。
测试通过后立即停止，进入仓库C++集成，不继续优化Python版。
```

---

### T04｜在 IE 中实现分段 bias 因子和 oracle 联合 refit

**时间：D2；负责人：Codex。前置：T02。**

**做什么：**

- 扩展实际测距模型 `h(X)+beta+c_s-z`；同段所有测量共享一个 `c_s`，不得给每包独立的无约束偏置。
- 实现 `oracle_debug` provider，只提供候选时间区间/观测集合；估计器不能得到 `b*` 幅值或 GT pose。
- 按 §4.3 约束求解或已有正确实现，执行 Stage 2 无 L1/TV 联合 refit。导航扰动包括姿态、速度、IMU bias，不把它们固定在 GT。
- 导出未正则化的完整 graph/Values 和 factor metadata，留给评分；短段、负值边界、求解失败有状态。
- 跑一个解析 factor 测试、一个 GTSAM 小图和一个现有短输入。

**产物：** segment factor、refit、GTest、oracle run 目录。

**完成标准：** factor 数值/自动 Jacobian 与流形有限差分吻合；β/c key 不冲突；在已知导航的解析测试中 amplitude 符合非负加权均值；联合图上目标与约束收敛检查通过。**不要求任意退化场景都准确恢复。**

**发给 Codex：**

```text
执行T04：在现有GTSAM后端添加常值segment amplitude及oracle_debug support provider。
只给support，不给真值幅值/轨迹。复用测距几何，保留独立固定beta。
实现无L1/TV、c>=0的联合refit；可用条件导航LM+分段闭式非负更新，检查联合停止条件。
保存完整含c的graph/Values，不把条件导航子图当最终联合图。
提供factor有限差分测试、解析amplitude测试、小图与短记录运行结果。
oracle结果标DEBUG_ONLY，不能进入正式端到端主表。
```

---

### T05｜最优先攻克的算法模块：从 GTSAM 抽取真实信息并评分

**时间：D2，D3 只允许补性能/边界；负责人：Codex。前置：T03、T04。**

**做什么：**

1. 按冻结段的区间重叠连通分量分组。构造 `G0 = priors + IMU + noncandidate ranges`，排除全部候选，不止当前组。
2. 对组 A，只加回该组测距，保留组内全部 amplitude；其余不确定状态进入 `y`。首帧 prior 的来源和强度要记录；初始化值不自动等于独立高精度观测。
3. 在同一 debiased 线性化点得到真实白化 Jacobian。保存 `key→column offset/dimension`；不可把增广 Jacobian 最后一列 RHS 当状态，也不可重复乘噪声白化。[R13]
4. **生产实现优先计算最小二乘残差，不构造大投影矩阵：**
   \[
   Y=\arg\min_Y\|FY-G\|_F^2,\quad E=G-FY,\quad R_c=E^TE.
   \]
   可用本机 Eigen 的稀疏 QR/等价平方根消元。小矩阵用 SVD 做对照，不为正式全轨迹构造稠密 `F F†` 或 `A^{-1}`。
5. 求 `N=G^T G`、广义最小特征值 `eta` 和最弱方向 `s`。检查最小二乘残差正交性、对称性、PSD 与 `R<=N` 的数值容差。稀疏 QR 的秩判定有数值局限，必须测试并记录阈值，不能只调用 API 就宣称可靠。[R14]
6. 用 **实际残差和原始 nominal sigma** 算 `gamma_s`。不读取旧 residual CSV 反推它，不使用 GNC 权重把 gamma 压小。

**LM 相关测试的正确表述：** 在**相同 graph/线性化 Values** 下，改变优化器 damping 设置不得改变报告信息。不同 LM 设置收敛到不同状态时，信息可以变化；不能误写成对所有最终解都应完全相同。

**产物：** 评分模块、decision-time CSV、小图 `F/G/N/R` dumps、数值对照报告。

**完成标准：** 同一小图的 C++ 与 Python 输出在容差内一致；主函数没有用 `Marginals` 的单个 pose covariance 冒充候选排除评分；rank failure 和 numerical failure 可区分；短记录能得到可审计的整组分数。

**卡住怎么做：** 稀疏实现不稳时，先在**预声明的短完整记录**上完成正确性和正文证据，报告规模限制；不暗中把全轨迹改成局部窗口，也不加信息正则伪造可恢复性。超资源组记失败，不任意拆组。

**发给 Codex：**

```text
执行T05，优先审计graph/变量列/白化/掩码，再写分数。
建立排除全部候选的G0；每个重叠组只加本组ranges；y包含全部未固定nav/IMU bias/可选beta。
在debiased Values上线性化，显式保存key列映射和factor→obs映射。
稀疏解min||FY-G||，用E=G-FY及R=E.TE实现评分；N=G.TG。
禁止全局dense projector/inverse、LM damping、discarded L1/TV Hessian进入信息。
对比T03小矩阵，验证噪声没有白化两次，RHS没有变成状态列；gamma使用真实完整残差。
输出rank/numerical状态、两种容差、列缩放、最弱特征值及运行开销。
```

---

### T06｜把 oracle provider 换成自动 discovery，不重写后半段

**时间：D3 主任务；负责人：Codex。前置：T04；T05 应已在小图通过。**

**做什么：** 在同一 IE pipeline 实现 Stage 1。固定导航状态时，逐 link 解：

\[
\min_{b\ge0}\frac12\sum_iw_i(b_i-e_i)^2+\lambda_1\mathbf1^Tb+\lambda_{TV}\|Db\|_1,
\quad e_i=z_i-h_i(X)-\beta_i.
\]

本周建议是**交替导航更新 + 非负 fused-sparse 子问题**，而不是把每个 `b_i` 生硬塞进旧 GNC 回路。已有可信 solver 优先；否则可用 ADMM，利用差分链的稀疏结构。固定导航的小子问题应与独立参考求解/解析小例子对照。不要把 `L1/TV` 换成平方 L2 后仍沿用论文表述。

`D` 只连接同 link 的相邻有效观测，超过 gap 阈值断链；分段规则固定 `b_min、change_threshold、min_count、min_duration、merge_rule`。segment 数量、长度、gap 与过滤统计都输出。

提取支撑后调用 T04 的无 L1/TV refit，再接 T05。不复制两套 Stage 2–4。

**产物：** 自动 provider、子问题测试、`segments.csv`、非 oracle 端到端 run。

**完成标准：** 无 bias、常值段、渐变 ramp 均能运行；评估实际检测与幅值误差，不预设 ramp 必须被判拒；达到目标/可行性/迭代停止标准；正式模式不读取 oracle support。去正则只去掉 shrinkage，不能声称消除了支撑选择偏差。

**若 D3 仍不通过：** 先修一个具体阻塞，不再加 solver 变体。若必须临时换成更简单的 residual proposal，必须登记方法变更并修改论文；不能留下 L1+TV 的原 claim。只有 oracle 的结果不能作为主方法最终交付。

**发给 Codex：**

```text
执行T06，复用T04/T05，只新增自动support provider。
按冻结目标实现交替导航更新与非负L1+TV链子问题；D按tag-anchor和时间gap断开。
冻结并记录支撑/change-point/最小段长规则；提取后去掉L1和TV进入无收缩refit。
小子问题验证目标、约束、停止残差；不要用平方L2、softplus隐式先验替换原目标。
正式运行禁用oracle输入，导出分段数/长度和迭代诊断。
交付一条从正常IMU/UWB输入到最终轨迹的自动运行及完整日志。
```

---

### T07｜增强仿真：先做确定性注入，不重造动力学仿真器

**时间：D2 提供最小样本，D3 完成实验版本；负责人：Codex。前置：T02 数据 ID 合同。**

**优先复用已生成的干净仿真轨迹/IMU与测距时序。** 在 loader 后、任何策略过滤前，对同一观测序列生成可缓存的实验输入：指定 link、起止时间、step/ramp、幅值/斜率和 seed。可以实现为 typed C++ `ScenarioInjector`，由 Python runner 生成配置；它属于数据生成层，不属于 estimator。

先不改四旋翼控制器、Gazebo、ROS 消息生态。现有 simulator 已有随机噪声/NLOS，但没有本周所需的完整脚本化真值合同。[R11]

**数据分层必须严格：**

```text
生成器：base input + recipe + seed → estimator observations
                                 └→ evaluation-only truth sidecar

估计器只拿 observations 和独立 calibration。
评估器单独读取 truth sidecar；oracle_debug 是独立的诊断开关。
```

若使用真实测距加 bias，名称必须是 **semi-synthetic injected bias**。其真值只覆盖你加的分量，不能把原本未知 NLOS 当成已知总 bias。

对于 controlled latent-bias 仿真，先关闭或显式记录原 simulator 的随机 NLOS、时钟漂移、静态偏置等额外分量；噪声不等于 bias。对 noise sweep 要真正改变测量噪声实现，不能只改 estimator 的 sigma；对 sample-count sweep 要改变真实进入估计的样本集合。

**产物：** 场景配置、生成/注入模块、input cache、evaluation-only truth、确定性测试。

**完成标准：** 同一基础输入/recipe/seed 得到相同观测；某个测量的真值能与 `obs_id` 一一对应；注入后的 `z` 变化与记录的新增 bias 一致；prefix 对同一观测使用相同扰动。仿真器的 fixed nonzero seed 只能保证它控制的随机过程，不应声称所有实时 ROS 调度都 bitwise 可重复。

**发给 Codex：**

```text
执行T07，复用已有仿真/输入，不重写动力学。
新增确定性step/ramp多链路注入、seed和eval-only truth sidecar；依据obs_id缓存输入。
同一数据提供给所有方法，估计器不能读b_star/support truth。
记录base随机NLOS/clock/static bias是否关闭；真实数据叠加只能标semi-synthetic。
增加注入守恒、输入复现、prefix一致性测试；生成常值双链路和缓ramp两个最小样本。
noise/sample-count变化必须发生在输入层，不只是修改评分参数。
```

---

### T08｜实现冻结决策、最终联合推断与一次 fallback

**时间：D2 oracle 先跑，D3 自动模式验收；负责人：Codex。前置：T04–T05。**

组级主决策严格为：

\[
Use(A)\iff \eta_A\ge\tau_\eta\ \land\ s_A\le\tau_s\ \land\max_{s\in A}\gamma_s\le\tau_\gamma.
\]

**做什么：**

- 不足最小支撑、边界、无效分数和不通过门控的组有明确原因码。`F` 的无关零空间不自动代表 bias 不可辨识；组的判定基于合同规定的 `R_c` 和诊断状态。
- 冻结 support、group 和 Use/Suppress。最终图保留 accepted `c_s`，原始 range 恰好出现一次；suppressed candidate 不出现，noncandidate 按冻结参考政策进入。
- 运行受约束最终 joint refit。保存实际返回的 `final_graph/final_values`，不再让旧 `Optimize()` 添加第二套隐式剔除策略。
- 在最终线性化点对同一参考加组构造再审计，记录 decision/final 两套分数。失败则全候选抑制重跑一次；若 suppression 图也无法给出有效轨迹，记录 `ESTIMATION_FAILED`，不能把未优化的轨迹当有效回退。
- 正常路径、回退路径、非收敛路径均输出 status 和耗时。最终全图 covariance 与 reference-group score 分开命名。

**产物：** `decisions.csv`、`scores_decision.csv`、`scores_final.csv`、实际最终图/状态摘要、fallback 日志。

**完成标准：** 因子唯一性和 accepted key 存在性测试通过；强制触发 fallback 的测试通过；没有反复重接纳；所有报告读取同一个最终结果对象。

**发给 Codex：**

```text
执行T08，把评分接成冻结的组级Use/Suppress。
用含accepted c变量的最终联合图refit，原始测距只出现一次；不再跑旧GNC/reject。
记录decision/final两套参考组分数，终点不通过时全candidate suppression fallback一次。
fallback仍失败必须返回ESTIMATION_FAILED，并保留尝试与失败日志。
加入因子唯一性、mask不变、accepted c保留、故意触发fallback及no-re-admission测试。
轨迹/bias/residual/covariance必须来自同一最终graph/Values对象。
```

---

### T09｜统一基线、评估和批量 runner，避免“每次跑的是不同实验”

**时间：D3；负责人：Codex。前置：T02；T08 后接完整方法。**

**本周主模式：** `all_range`（至少 LOS 对照）、`gnc_rejection`（复用已有策略）、`structured_refit`（无 gate）、`fit_only`、`s_fit`、`full_gate`。名称为本计划建议，落地后写进合同。

所有模式使用相同观测池、初始化规则、keyframe plan、标定、nominal sigma 和 solver 精度。主消融可以缓存并复用 discovery/refit 的同一结果，以单独比较 gate；独立端到端实验再报告整体管道表现。

**不要把旧完整 pipeline 与新 pipeline 直接当成只差一个 gate。** 如果预滤、初始化、标定、时间匹配或采样不同，必须先匹配，或将旧结果标为额外系统参考。

评估器必须：

- 区分独立固定 map-to-GT 变换下的误差和每条估计轨迹重新对齐的 ATE；独立变换没有来源时不捏造 raw-frame 指标。
- 配对同一刚体/点，检查 IMU 原点、UWB 天线、Vicon body、Leica prism 等差异；使用冻结的时间关联窗口或插值策略，并报告匹配数量。
- 导出 bias/segment/group 指标、三种 coverage、求解/fallback 失败率、runtime/memory；缺失结果不能按零误差进入均值。
- 原始配置文件原样保存，另存有效配置，避免旧 `LogConfig()` 部分字段写出导致无法复现。[R7]

**产物：** 实验 manifest、运行脚本、统一评估器、结果 CSV/JSON。

**完成标准：** 同一场景的方法输入 hash 一致；失败运行仍占分母；一条命令能跑完一个小实验集合并退出。每个 run 有唯一 ID、commit、输入/标定/配置 hash 和状态。

**发给 Codex：**

```text
执行T09，按EXPERIMENT_CONTRACT统一六种method模式及runner。
保证共同input/keyframe/init/noise/calibration，复用discovery/refit缓存做gate-only对比。
修正/新增评估路径：固定变换误差与aligned ATE分开，时间配对/刚体点一致，失败不可丢。
实现每run独立目录、配置原件+effective配置、hash/commit/seed/status、资源记录。
不以logs/latest或共享data/trajectory.txt汇总，不让程序等待可视化Ctrl+C。
先运行2个场景验证全链路，再启动正式实验。
```

---

### T10｜验证集锁定门控，评估 RQ3，不为保住 η 改测试标准

**时间：D3 小样本，D4 锁定；负责人：Codex 运行 → GPT 审查。前置：T06–T09。**

**做什么：** 按整条记录/基础轨迹与 seed 分组划分 calibration/validation/test，避免相邻包泄漏。若只有一条仿真运动轨迹，明确结论是对该轨迹下扰动实现的泛化，而不是跨运动泛化。

比较 `fit_only`、`s_fit`、`full_gate` 的 accepted-bias risk/coverage；generic curvature 至少从缓存中导出一个**明确定义、量纲一致**的简单 comparator，而非声称复现某篇文献的方法。建议用 bias nominal curvature `lambda_min(N)` 作为低成本非消元对照；不得把混合单位的全导航 Hessian trace 当无条件合理的评分。[U1]

在验证集上给各策略相同的有限调参预算、预声明的 operating points；锁定规则后测试。测试曲线显示实际达到的 coverage，不用测试误差挑最优阈值。阈值和排名函数冻结，不等于每个测试集必须恰好相同 coverage。

**重要修正：不要强求 η 在“固定 s”时一定有独立预测力。** 对单 amplitude，`R=1/s²`，所以 `eta=1/(N s²)`；当 `N` 也固定时，η 与 s 是确定关系，无法提供新的信息。要检验的是：**在你真实的多组、噪声、样本数和模型失配条件中，加入 η 的政策是否有实用增量**，而不是给一个数学上不可能的“独立性证明”。

D4 的判断分两层：先看 pipeline 是否正确，再看政策是否有收益。不能将代码/标定错误归咎于方法，也不能凭一个噪声 seed 的好坏就改论文定位。

**产物：** `locked_gate.yaml`、验证选择记录、测试 risk-coverage 图、匹配条件的策略对比表、GPT 的 claim 决定。

**完成标准：** test 标签未参与门限选择；零接受不是零风险胜利；报告不利结果、未覆盖条件和不确定性。`eta` 无明显增量时，将其降为诊断/解释量并登记方法简化；系统价值仍须由 RQ1/RQ2 单独证明，不保证“系统兜底就足够发表”。

**发给 Codex：**

```text
执行T10，使用缓存候选/refit结果比较fit_only、s_fit、full_gate及nominal-curvature对照。
按整run/基础轨迹/seed切分，锁定验证集阈值与调参预算，再看测试标签。
报告实际coverage、accepted bias error、bad-correction rate、空接受和失败；
stage2相同候选的诊断风险与最终轨迹收益分开。
不要假定eta必须优于s，尤其单段固定N时两者有确定关系。
输出可追溯CSV、锁定参数和可供GPT判断的证据摘要，不自行改科学claim。
```

**发给 GPT：**

```text
审查T10结果，只使用提供的配置/CSV/图/失败记录。
先判断是否有实现或评估错误，再判断full gate相对s+fit的实用增量和coverage代价。
区分证据不足、无增量、负增量；不要把少量seed写成统计显著结论。
给出保持/收窄/删除的具体claim以及最小论文改动，不新加方法分支。
```

---

### T11｜prefix/full-batch：同一历史区间、无未来泄漏

**时间：D4，D5 定图；负责人：Codex。前置：T09，门限沿用 T10。**

定义历史 burst `B=[t_a,t_b]`，比较观测终点 `t_b+H`，例如 `H={0,2,5,full}` 秒。**H 是 burst 结束后可用的额外上下文，不是含混的“数据集前 50%”。** 具体数值由记录时长和采样率决定，执行前固定。

两条路径分开：

**端到端路径：** 在初始化前裁剪 UWB/IMU；每个 prefix 重做初始化、discovery、refit、评分和决策。使用同一份独立标定与锁定阈值；不使用 full-run warm start、support、full-run 拟合噪声/锚点或未来 IMU 插值样本。未来 GT 只可用于离线评估，不能进入估计。

**固定模型诊断：** 同一历史 amplitude/support 与共同线性化，仅增加有效未来因子，检查高斯信息结论；清楚标 `diagnostic_fixed_model`，不把它画成实时或端到端能力。

端到端各 prefix 分段可能不同；主要比较**固定历史观测 ID 集上的估计 bias 场/轨迹误差**。不要直接把不同含义的 segment ID 的 `c_s` 连成一条曲线。

**产物：** prefix manifest、访问时间范围检查、固定历史区间表、context 图。

**完成标准：** 保存输入最大时间并验证裁剪；删除/打乱 cutoff 之后的未来数据不会改变 prefix 结果；各方法评估同一区间；允许非线性实际误差非单调，并如实解释。

**发给 Codex：**

```text
执行T11，对固定历史burst做t_b+H的prefix/full比较。
先裁剪UWB和IMU再初始化；独立标定和锁定阈值可共享，full-run状态/support不可共享。
不在cutoff之后取IMU插值端点；检查loader自动锚点/时钟/初始化是否偷看全数据。
另写固定support/共同线性化诊断，和端到端结果分表。
跨prefix以相同历史obs_id比较bias场和轨迹，不能误配不同segment ID。
加“修改未来不影响prefix”的metamorphic test，并导出context图表。
```

---

### T12｜完成最小仿真矩阵与数据集正式测试

**时间：D4–D5；负责人：你选择可用记录，Codex 执行。前置：T09–T11；测试门限已冻结。**

按照 §6 的预算执行，不展开全部参数组合。仿真承担 latent bias 与门控诊断；受控记录承担真实 NLOS 端到端对比；公开数据承担接口/轨迹泛化，其没有 bias 标签时不承担 bias recovery ground truth。

优先使用 T00 已跑通的输入。自有数据与外部数据可共享后端，但**不要求错误共享同一数值噪声参数**：传感器特定噪声来自各自独立标定，方法之间匹配，同一门控协议是否跨域泛化须明确。

测 RQ1 时输出实际 build/run 条件、运行规模、分阶段耗时、评分开销、峰值内存及至少一次重跑差异。一次数据接入通过只说明该输入通过，不代表所有数据集都验证过。

**产物：** 主结果表、系统能力表、完整 metrics、失败清单、生成图表的命令、被排除场景及原因。

**完成标准：** RQ1–RQ4 每项有具体文件可定位，或在 `CLAIM_EVIDENCE.md` 标不足；LOS 不劣的容忍标准预先确定，不在结果出来后反设；失败与低 coverage 不隐去；没有把 MCD 同序列拟合锚点的结果当独立 benchmark。

**发给 Codex：**

```text
执行T12，只跑EXPERIMENT_CONTRACT中预声明的最小矩阵与已有可用数据。
所有输入/标定/参数/提交可追溯，所有失败和fallback计入统计。
仿真latent truth、受控真实参考、公开数据portability三种角色分开。
报告ATE定义、bias指标可用性、candidate/eligible/overall coverage、分阶段runtime和内存。
依据实测单run耗时控制预算，先完成每个RQ的最小证据，再增加重复；不挑结果好的seed。
生成主表/系统表/主图/context图与CLAIM_EVIDENCE映射，不写虚构数字。
```

---

### T13｜每日写作、D6 完整稿、D7 交付检查

**时间：D1 开始贯穿；D5–D7 为重点。负责人：Codex 写文件/编译，GPT 审查论证。**

**每天写什么：**

| 日 | 写作任务 | 写作依据 |
|---|---|---|
| D1 | System 和问题定义；Method 公式/已有证明；实验计划 | 冻结结构 + 代码审计，避免已实现/待实现混用 |
| D2 | 新 factor、refit、信息评分与算法流程 | 实际 C++ 接口、golden tests、oracle 模块测试 |
| D3 | discovery 与最终策略实现细节；baseline/数据协议 | 实际自动 pipeline 与冻结实验合同 |
| D4 | RQ3 和 future context 初稿；Related Work 来源确认 | 实际验证结果与官方论文/代码来源 |
| D5 | 主实验、限制和失败分析；重写贡献句 | 冻结结果，不反向挑选有利实验 |
| D6 | 完整 Abstract/Introduction/Results/Conclusion；压页数 | claim–evidence 对照表、可复现图表 |
| D7 | 术语、符号、匿名、引用、图表、制品和最终编译 | 最终提交版本 |

**数字进入 LaTeX 的方式：** Codex 从 `metrics.csv` 生成 `paper/generated/*.tex` 表格和必要数值宏；正文的 headline 数字引用相同导出来源。图也由脚本从固定 CSV 生成，禁止手工改柱子/删失败点。

**版面约束：** 沿用冻结结构的两幅全宽图（架构、主证据）和单栏 context 图；表格优先紧凑单栏。8 页是 [U1] 内部写作预算，不是已经核实的会议规则。目标 venue 明确后，用 GPT 核实官方页数/补充/匿名/制品规则；未明确前不臆测。

**产物：** 完整 `paper/`、PDF、引用清单、claim–evidence 表、复现 README、实验 manifest。

**完成标准：** D6 已是连续正文且必要图表都在；D7 无未完成结果声明。缺证据的 claim 要删改，不用更强形容词填补。开源状态区分 implemented/tested/released；发布仓库本身并不证明此次新模块已发布。

**发给 GPT（D5/D6）：**

```text
请审查完整论文与CLAIM_EVIDENCE/冻结metrics/代码实现摘要。
仅检查：贡献是否有实测证据，方法与实现是否一致，gamma/eta/不确定度解释是否过强，
prefix是否泄漏，基线是否公平，真实数据bias reference是否被错写成真值。
按必须修改/可保留给出可直接执行的段落级意见，不扩大scope或提出新实验战线。
文献缺证据只查最接近的原始论文/官方代码；不补虚构引用。
```

**发给 Codex（D6/D7）：**

```text
执行T13，根据审查意见改paper文件。所有结果数字只能来自锁定metrics及generated表。
将System/Method与最终源码逐项对齐；去掉planning占位和未验证能力的完成式表述。
编译LaTeX，检查引用/符号/图表溢出/页数/匿名和实际artifact状态。
跑最小复现流程，输出最终commit、命令、测试结果、图表输入hash与未解决事项。
不新增功能、不改测试集阈值、不手工调整结果。
```

<a id="s6"></a>
## 6. 实验预算、指标及证据判定

### 6.1 不做全组合：先主证据，再少量单因素压力测试

下面是**执行建议**，不是冻结论文已经拥有的数据，也不是保证统计充分性的样本数。D1 用现有单次耗时给出预算，D3 用完整新管道的耗时修正；正式测试前将删减规则写进实验合同。

| 层级 | 建议场景 | 重复与运行方式 | 支撑什么 |
|---|---|---|---|
| 必做主场景 | LOS；1/2/3-link 持续 step；双链路缓 ramp；双链路陡 ramp，共 6 类 | 初始建议每类 2 个开发/验证 seed + 3 个未见测试 seed；已有更多独立轨迹时按轨迹分组 | 多链路处理、LOS 控制、明显与温和失配 |
| 低成本诊断 | 等锚数弱几何；较高真实 range noise；较少真实观测数；另一 burst 时长 | 以双链路主场景为基准，每次只改一个因素；优先复用前段计算 | eta/s 的尺度解释、边界和局限 |
| gate 对照 | fit-only、s+fit、full、nominal-curvature comparator | **从同一候选/refit缓存算分与策略**；不为每个阈值重跑后端 | RQ3 的接受风险/coverage |
| 下游轨迹 | GNC/rejection、structured-refit、full；s+fit 在固定代表子集上补跑 | 所有测试主场景先完成前三种；LOS 上增加 all-range；子集在看结果前固定 | RQ2 及 gate 的实际导航收益 |
| future context | 选 2 个预声明历史 burst，H=0/2/5/full 或等价可用长度 | full 结果可复用；prefix 必须实际独立重跑 | RQ4，历史区间的变化 |
| 自有真实数据 | 已有 LOS + 长多链路；尽量包含冻结结构的两种运动 | 同记录配对跑；协议不完备的记录标限制 | 真实端到端证据 |
| 外部数据 | **一个已跑通且标定可信的数据集** | 先小段，再一条完整可承受序列；同后端 | RQ1 portability / 轨迹效果 |

**不要把 6×5 个仿真输入误解成必须给六种 gate、所有阈值和所有 context 各重跑一遍。**

建议缓存键：

```text
input_hash + calibration_hash + keyframe_plan_hash
+ initialization_rule + solver_version + discovery/refit_config_hash
```

缓存 discovery/refit/decision-time scores 后，risk–coverage 的大部分点只需分析 CSV。最终 joint refit 只用于**已经冻结的工作点**与少量必要下游对照。改变参考图政策、基础权重或 solver 语义时必须使缓存失效。

### 6.2 运行预算用实测时间决定

让 runner 实测代表场景的中位与较慢运行时间，记录 stage breakdown。用下式安排本地计算，不把预计值写成实测：

\[
\text{预计总计算量}=\sum_{\text{未缓存实验}}\text{该类单次耗时估计}+\text{汇总与重跑预算}.
\]

至少保留约四分之一计算预算用于失败诊断/复跑。先覆盖所有必要问题，再增加重复；减少额外参数水平先于删掉某类失败场景。不要按“哪个 seed 表现差”删样本。并行只开到本机实测内存允许的数量，输出目录绝不共享。

若只有少量独立 run，展示各 run 配对结果与区间，不把 packet 数当独立样本数。固定基础轨迹下的多个 noise seed 只代表该运动条件下的扰动重复。Bootstrap 以基础记录/run为重采样单位；样本很少时不写“统计显著优于”。

### 6.3 必须写清的指标

#### 三种 coverage，不能只给一个最好看的比例

- `candidate_use_coverage`：被接受的候选测量数 / 全部候选测量数，**包含不满足 eligibility 的候选在分母**。
- `eligible_use_coverage`：被接受的候选测量数 / eligible 候选测量数，供诊断门控本身。
- `overall_retained_fraction`：最终实际使用的原始测量数 / 固定有效输入测量数。

同时报告 group/segment 数量和长度，解释保守分组是否把大组整体抑制。零候选、零 eligible、零接受分别输出状态；**零接受的 conditional risk 是未定义，不是 0。**

#### correction error 不自动等于“对导航有害”

仿真可按固定观测集合计算 `hat_b_i - b_i*`。在 ramp 下，同一常值段的单一幅值不等于完整时变真值，因此给段内 bias-field RMSE，不只比较段均值。

预声明 `epsilon_bad_m`，例如按应用可接受 correction error 定义 bad correction；该数值与 `tau_s` 的含义不同：后者是局部不确定度阈值，不是误差上界保证。

`bad_correction_rate` 是 accepted correction 的误差判定；只有实际比较“使用 vs 抑制”的下游轨迹误差，才可以称 `trajectory_harm`。不要因为某段 bias error 较大就直接宣称 suppression 一定更好。

#### decision-time、final-time 与 fallback 不能混在一列

RQ3 的主诊断对比使用相同 Stage 2 候选/refit结果，便于隔离 gate。最终图 refit 后的 bias/ATE另报；fallback 结果与 recovery attempt failure 同时保留，不能只留下回退成功的轨迹掩盖模块失败。

### 6.4 最低数值测试矩阵

| ID | 条件 | 必须满足的验收 | 何时完成 |
|---|---|---|---|
| U01 | 导航已知 | `R=N`、`eta=1` | T03 |
| U02 | `z=x+c` 无外部导航约束 | `A` 可逆也可能 `R=0`；`s=inf` | T03/T05 |
| U03 | 与 bias 无关的 nuisance 零列 | 不因无关零列产生虚假有限/无限信息 | T03/T05 |
| U04 | 全部信息很弱 | 高 eta 与大 s 可以共存 | T03 |
| U05 | multi-amplitude 近退化 | 最弱方向正确暴露；记录秩阈值敏感性 | T03/T05 |
| U06 | 固定 β / 在线 β 两种声明 | 固定值不漏用；在线值进入 nuisance 与其真实 prior | T02/T05 |
| U07 | 满秩小图 | projector / Schur / C++ 稀疏结果在容差内一致 | T05 |
| U08 | 流形有限差分 | factor Jacobian 与 GTSAM 实际局部参数化一致 | T04 |
| U09 | 同图、同 Values、不同 LM damping 参数 | 信息不掺优化步正则 | T05 |
| U10 | 删除全部 candidate 的参考掩码 | 无跨组互相担保，因子只加入一次 | T05/T08 |
| U11 | gap/短段/边界 | 不跨 gap 偷连，不因一包零残差获得自动接受 | T06 |
| U12 | 最终图与 fallback | accepted c在图中；审计/失败/一次回退可追踪 | T08 |
| U13 | 改写 cutoff 之后的数据 | prefix 结果不变（在数值容差内） | T11 |
| U14 | 所有候选都拒绝/无候选 | coverage与risk分母正确；不产生“零风险获胜” | T09/T10 |

对良态小矩阵，可在合同中预声明例如 `relative 1e-7` 级的比较容差；这是**测试配置建议，不是跨尺度保证**。近奇异情形另报谱、残差和不同阈值下判定，不以放宽容差抹掉失败。

### 6.5 D4/D5 如何决定论文措辞

| 看到的证据 | 应做的决定 | 不应该做的事 |
|---|---|---|
| full gate 在相近 coverage 下有稳定改进，下游也受益 | 保留政策贡献，并给范围与失败例子 | 写成 guaranteed safe recovery |
| eta 相对 s+fit 没有清晰增量 | eta 降为诊断；保留被证据支持的边际信息/不确定度处理；登记简化 | 将无增量藏起来，或反复调测试集救 eta |
| 几乎全 suppress | 分析是分组、秩、边界还是门限；如实报告低覆盖 | 给零接受写零风险并称优越 |
| prefix/full 只有局部信息改善，没有实际误差改善 | 区分分析结论和实测结果，缩小 non-causal claim | 强求每条记录出现 Suppress→Use |
| 真实数据缺 bias labels | 报轨迹、集成和带噪参考，不报未知真实 bias RMSE | 把 post-fit residual 当 bias truth |
| 自动 discovery 未完成 | C2 未实现完整，不能保留自动四阶段的完成式描述 | 用 oracle 图冒充端到端方法 |
| 数学测试失败/图语义错误 | 修 correctness，受影响实验作废重跑 | 先“冻结结果”再改论文掩盖 |

<a id="s7"></a>
## 7. 每天收工前的检查与失败处理

### 7.1 每天固定做一次交接，不开新一轮大讨论

Codex 更新 `STATUS.md`，你只看下面六项，然后调用下一张任务卡：

```text
今天完成的任务 ID：
实际通过的测试/实验命令与结果路径：
未通过/未执行事项及原因：
当前 commit / 未提交 diff：
论文新增内容与支撑证据：
明天第一张任务卡与唯一主阻塞：
```

GPT 只在三类时候介入：**合同存在数学歧义、实验无法判断是实现错还是假设错、结果需要决定论文 claim**。编译错误、路径问题、API 适配先让 Codex 在本机解决，不把所有日志都变成新一轮方案讨论。

### 7.2 必须保留的止损规则

| 风险 | 最迟发现时间 | 处理，不延长到第二周 |
|---|---|---|
| 基础环境/数据不可运行 | D1 | 修现有环境；选已可用输入；直接记录缺证据，不虚构完成度 |
| 外部数据只有污染的标定 | D1 | portability-only 或换已有可信数据；不拿同序列 GT 拟合结果撑主 benchmark |
| 稀疏信息提取不可靠 | D2 | 用短完整记录保证正确性、明确规模边界；不能用强 prior/jitter伪造信息 |
| 自动 discovery 集成失败 | D3 | 修唯一阻塞；必须变更方法时写 amendment 并同步 `.tex`；不提交假自动流程 |
| 过度保守导致 coverage 很低 | D3 小样本/D4 正式 | 报原因与代价；只用验证集调锁定参数，不增加新的反复接纳算法 |
| gamma 抓不住缓 ramp | D4 | 作为失效范围报告；本周不再新增 whiteness 主门控 |
| eta 没有增量 | D4 | 降为解释性指标，不误称失败被“系统贡献自动补足” |
| 所需 UGV 运动或重复次数不存在 | D1确认、D4前处理 | 有条件只补最小必要记录；否则明确减小适用范围和实验主张 |
| 计算量超预算 | D3 | 缓存、中止非必要全组合、先每个RQ的最小证据；固定删减规则 |
| 结果修复发生在 D6/D7 | 发现即处理 | correctness bug 必须修；使相关缓存失效并重跑受影响项，不混用旧数字 |

**功能冻结不是允许保留错误。** D6/D7 可修测量语义、输出错位、评估错误等 correctness bug，但必须标记受影响的实验和图表，不能只改源码不改结果。

### 7.3 本周不做清单

不重新证明全 IMU observability；不加随机游走 bias 来追漂亮引理；不做 universal duration threshold；不开发 fixed-lag 边缘化；不加 anchor/lever/time 联合自标定新理论；不加第三态 downweight 主方法；不开发新传感器/第二套平台；不为了文档漂亮重构全部代码；不追求全数据集适配；不在最后一天重做 novelty survey。

这些是 [U1] 的 scope 控制与本周的交付限制，不是声称这些方向没有价值。

<a id="s8"></a>
## 8. 工具调用模板、命令约定与 AGENTS.md

### 8.1 你每次只需要按这个顺序调用

```text
1. 从STATUS确认当前任务卡与前置是否通过。
2. 将对应任务提示词发给Codex；附上合同和必要源码/日志。
3. Codex改代码/文档并在本机运行验收。
4. 你核对实际产物，不只读“已完成”总结。
5. 需要科学判断时，把证据包交给GPT审查。
6. Codex落实审查后的最小修改，更新STATUS与claim–evidence。
7. 进入下一张任务卡。
```

**不要这样调用：** “请根据论文一次实现全部方法并做全部实验”。  
**应这样调用：** “执行 T05；T03 fixtures 与 T04 oracle 图已通过；只允许改评分模块和对应测试；输出 F/G 对照与 test log”。

### 8.2 Codex 通用任务前缀

把它与每张任务卡的专用提示词合用；不需要每次粘贴整段讨论历史。

```text
你正在已有UWB-IMU-IE工程内执行一个七天交付任务，不是重写项目。
先读取AGENTS.md、METHOD_CONTRACT、EXPERIMENT_CONTRACT、STATUS及本任务卡。
只完成当前任务及其直接阻塞，不新增研究方向，不擅自改变冻结数学定义。
修改前指出将复用的文件和需要改动的文件；保护本地未提交改动。
实现后实际运行能运行的测试；记录命令、退出码和产物路径。
无法运行就标NOT_RUN并说明缺什么，不把静态检查写成通过。
所有科学结果只能来自本次可追溯实验；不得伪造数据、指标或引用。
结束时更新STATUS和相关claim–evidence，报告diff、验证、限制、下一步。
```

### 8.3 GPT 的定向检索提示词

**只在 T01/D4 的文献或数据协议空白处使用，不做无边界研究。**

```text
我在七天冲刺，不需要扩大related work。
请仅核实下面这些具体事实，优先原论文、官方代码/数据说明：
[填入当前缺口，例如某数据集锚点是否独立提供、GT刚体定义、某近邻方法是否full-trajectory]
每条给：结论、原始来源、证据位置、可写入论文的克制表述；无法确认写unknown。
不要由“offline experiment”推断full-batch，不要由仓库存在推断相关功能已发布。
输出短的source ledger和BibTeX修正；不要改变我们冻结的方法。
```

具体 GTSAM/Eigen API 遇到版本差异时，先由 Codex 查看**本机安装头文件与 CMake 检测版本**；必要时 GPT 再查相同版本原始文档。不要直接照抄最新版网上代码替换可工作的依赖。

### 8.4 可直接使用与必须先实现的命令要分开

**现有工程命令：** 来源为仓库构建/运行入口；你的工作空间路径需要替换，环境与依赖由 T00 核验。[R1][R9][R10]

```bash
# 仅设置你本机真实路径；不要新克隆覆盖已有工作区。
export IE_WS=/path/to/your/catkin_workspace
export IE_REPO="$IE_WS/src/uwb-imu-fusion"

cd "$IE_REPO"
git status --short
git rev-parse HEAD

cd "$IE_WS"
catkin build uwb_imu_fgo
source devel/setup.bash
catkin test uwb_imu_fgo

# 旧入口：用于T00复现，不作为后续无人值守runner的完成规范。
roslaunch uwb_imu_fgo offline.launch \
  config_path:="$IE_REPO/config/your_existing_config.yaml"
```

**下面是 T02/T09/T13 要由 Codex 创建并验证的接口约定，现在不声称存在：**

```bash
# T03完成后应支持的小矩阵测试（测试框架可由合同确定）。
python3 tools/paper/reference_recoverability.py --self-test

# T02/T09完成后：manifest决定输入、方法、输出目录；smoke不读正式test标签。
python3 tools/paper/run_experiments.py --manifest config/paper/smoke.yaml

# T10–T12完成后：使用已经锁定的测试矩阵，禁止自动改阈值。
python3 tools/paper/run_experiments.py --manifest config/paper/test_locked.yaml

# T13完成后：只从锁定结果生成图表，不重新估计或调参。
python3 tools/paper/build_paper_results.py --manifest config/paper/test_locked.yaml
```

本机确定不同文件名也可以，但 Codex 必须在 README 更新**真正执行通过的命令**。论文复现命令不是一段未经运行的 shell 示例。

### 8.5 建议合并入仓库 AGENTS.md 的短约束

OpenAI 的 Codex 文档描述了仓库级 `AGENTS.md` 的指令加载；本文用它固定项目约束，不把放一个文件当成成功率保证。[O1]

```markdown
# UWB-IMU-IE sprint rules

- Read doc/ie_sprint/METHOD_CONTRACT.md and EXPERIMENT_CONTRACT.md before changing the method.
- Reuse the existing IE/GTSAM backend; do not build a second full estimator in Python.
- Keep legacy behavior compatible unless an explicit correctness fix is documented.
- Do not discard suspected NLOS before the candidate stage in the paper path.
- Keep raw observations and stable IDs; log all selection masks.
- Distinguish fixed static beta, dynamic segment c, and IMU bias states.
- Do not add LM damping, discarded L1/TV terms, or artificial priors to reported information.
- Preserve accepted c states in the final joint graph; no duplicate corrected pseudo-ranges.
- No oracle/GT access in official end-to-end estimation. Prefix inputs must be cropped before initialization.
- Do not tune using test labels or omit failed/zero-coverage runs.
- Write run-isolated outputs; no shared latest/trajectory file as the results database.
- Report actual commands, exit codes, test results, and NOT_RUN items honestly.
- Paper numbers must be generated from locked metrics; do not invent results or citations.
- Update STATUS.md and paper/CLAIM_EVIDENCE.md after each task.
```

### 8.6 最小 evidence schema

不要为了 logging 建数据库。每次 run 一个目录，至少有：

```text
runs/<run_id>/
  input_manifest.json          # 输入/标定/配置hash、seed、cutoff、数据角色
  config_original.yaml
  config_effective.yaml
  run_status.json              # OK/FALLBACK/FAILED/TIMEOUT及原因、commit、耗时
  observations.csv             # obs_id、time、tag/anchor、z、sigma、keyframe、mask
  segments.csv
  scores_decision.csv
  scores_final.csv
  decisions.csv
  trajectory.tum
  metrics.json
  stdout.log
  stderr.log
```

Evaluation-only truth 另放数据生成/评估目录，不作为 estimator 的必需输入。没有计算的 covariance 不生成伪数值；确实计算时记录使用的图/Values/坐标系与求解状态。

`CLAIM_EVIDENCE.md` 最少五列：

```text
claim | implementation/source | experiment/run_ids | figure/table | status/limitations
```

<a id="s9"></a>
## 9. 第七天交付清单

### 代码与方法

- [ ] 旧 IE 路径保留可复现的基线，新 paper 路径运行结束退出。
- [ ] 自动 discovery → 无 L1/TV refit → 完整 nuisance 信息评分 → 组级冻结决策 → 最终 joint refit 全链路存在。
- [ ] 固定 β、nominal sigma、分段和 factor masks 与论文一致。
- [ ] accepted `c_s` 保留，原始 range 只进入一次；fallback 有实际测试。
- [ ] 小矩阵、GTSAM 小图、输入管道和 prefix 泄漏测试有真实日志。
- [ ] 执行版本/依赖/输入/配置可定位，未实现功能不写入贡献。

### 仿真与数据

- [ ] 有多链路持续 step、LOS 控制、缓/陡 ramp，以及最低限噪声/样本数/几何诊断。
- [ ] 验证与测试分离；门限已锁定；失败和空接受未被删除。
- [ ] 有受控真实证据，或明确声明其不足并同步调整适用范围。
- [ ] 至少一个外部数据接入经过实际验证；标定不独立时只作适当的 portability 证据。
- [ ] 同历史区间 prefix/full 与固定模型诊断分别命名、分别解释。
- [ ] bias truth / measured reference / post-fit residual 没有混用。

### 论文与制品

- [ ] 完整正文、图表、引用、Abstract、Conclusion 已编译。
- [ ] 主结果每个数字都有来源，图表能从冻结 CSV 重建。
- [ ] `eta` 不是 accuracy、`gamma` 不是独立安全证书、局部线性结论不是全局保证。
- [ ] RQ1 的系统能力有实际测量，而不只是一张架构图。
- [ ] 论文标题/贡献/开源措辞与实际实现、测试和发布状态一致。
- [ ] 页数、匿名、补充材料、代码链接是否允许按目标 venue 官方规则确认；公开旧仓库的身份风险单独检查。
- [ ] 一次从指定输入到至少一个主结果的复现演练完成；完整复现成本如实说明。

**最终验收不是“上面每项都打勾才能假装完成”，而是：已完成的有证据，未完成的在正文和贡献中不再被承诺。** 某一必要核心未完成可能意味着稿件仍不具备预期投稿成熟度；不能靠扩大系统描述或漂亮图掩盖。

<a id="s10"></a>
## 10. 依据、源码与核验边界

### 10.1 用户提供的冻结材料

**[U1]** `UWB_IMU_IE_System_Centered_Structure_v3.tex`。方法定义主要对应 `eq:discovery / eq:fg / eq:projection / eq:eta / eq:sigma / eq:gamma / eq:policy / eq:final / eq:future`，以及内部实施与实验 checklist。本文遵循这些定义；新增的文件布局、交替求解器、实验预算与提示词是执行建议，不伪装成原文规定。

**[U2]** `roadmap_v1.md`。采用其正确的 candidate-excluded reference、full nuisance、无正则 refit、整组政策、非 oracle 验证、coverage 与制品要求；改变开发先后顺序、七天时间约束和仓库风险处理。

### 10.2 仓库一手来源（2026-09-05 静态核验）

以下 URL 作为可复制的原始来源地址；正式源码定位应以 T00 记录的本地 commit 和行号为准。`main` 是浮动分支，后续可能变化。

**[R0] 参考提交与仓库**

```text
https://github.com/Printeger/uwb-imu-fusion
https://github.com/Printeger/uwb-imu-fusion/commit/cfe6d29cc2a0077d9a0b958461d1615aa5e9079e
```

**[R1] 主入口：数据分支、预滤/关键帧、旧多趟优化、covariance 再调用、可视化循环**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/tools/run_offline.cpp
```

**[R2] 数据类型与配置/加载：接口存在不等于已测能力**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/include/uifgo/types.h
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/include/uifgo/config.h
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/data_loader.cpp
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/initializer.cpp
```

**[R3] RSSI 预滤**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/outlier_filter.cpp
```

**[R4] 当前 UWB factor 与 beta 开关**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/uwb_factor.cpp
```

**[R5] 图构建、状态 key、初始 prior 与 AdaptiveSigma**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/graph_builder.cpp
```

**[R6] GNC/TLS、chi-square rejection 与 Marginals 路径**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/optimizer.cpp
```

**[R7] 日志残差、协方差占位和配置快照**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/logger.cpp
```

**[R8] 现有轨迹评估**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/src/trajectory_io.cpp
```

**[R9] README：运行命令、MCD 示例锚点限制与系统说明**

```text
https://github.com/Printeger/uwb-imu-fusion#readme
```

**[R10] 实际构建文件**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/CMakeLists.txt
```

**[R11] 当前 UWB simulator**

```text
https://raw.githubusercontent.com/Printeger/uwb-imu-fusion/main/simulator/src/uwb_twr_sim.cpp
```

### 10.3 数值接口与工具使用的一手文档

**[R12] GTSAM 4.2 Pose3 局部参数顺序与位置 Jacobian**

```text
https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/geometry/Pose3.h
```

**[R13] GTSAM GaussianFactorGraph 的 Jacobian/ordering 接口**

```text
https://raw.githubusercontent.com/borglab/gtsam/4.2/gtsam/linear/GaussianFactorGraph.h
https://gtsam.org/doxygen/4.0.0/a03135.html
```

旧版 Doxygen 用于解释白化/增广矩阵语义，不作为本机 4.2 API 签名的唯一依据；实际编译以本机头文件为准。

**[R14] Eigen SparseQR 文档：列置换、数值秩、求解准确性检查**

```text
https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1SparseQR.html
```

该地址为开发版文档；实际阈值行为与 API 须由 T00/T05 针对本机 Eigen 验证，不要求为此升级依赖。

**[O1] OpenAI：AGENTS.md 使用文档**

```text
https://developers.openai.com/codex/guides/agents-md
```

**[O2] OpenAI：worktree 工作方式**

```text
https://developers.openai.com/codex/app/worktrees
```

---

## 现在的第一步

**先把本文件与冻结 `.tex` 放进本地仓库文档目录，打开能访问现有 ROS/GTSAM 环境的 Codex 会话，发送 T00。**

T00 的目标不是写新算法，而是拿到可靠的起点：什么确实能跑、什么数据确实能用、哪些基础语义必须先补。随后按 T01 → T02/T03 → T04/T05/T08 → T06 → T09–T12 → T13 推进；论文从 D1 同步写，不延后到代码和实验全部结束。