# 0912-PL-PERSISTENT-CUSUM-PREFLIGHT 当前任务

`DONE / CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY / DYNAMIC_NOT_RUN`。用户授权执行
[`PL_PERSISTENT_CUSUM_PREFLIGHT_PROTOCOL.md`](../ie_0911/PL_PERSISTENT_CUSUM_PREFLIGHT_PROTOCOL.md)：
只用 PL per-anchor `conditional_z`，固定 `G=max(0,G+x-0.5)`、clean 60% temporal calibration / 1s guard /
held-out validation、`h=max(5,G_cal_max+1)`、last-zero backfill 与 1s gap/invalid reset。group chi-square
完全退出 candidate decision。先完成 frozen F0--F4；全部 PASS 后才执行 always-commit CONTROL/SHADOW
dynamic D0--D4。禁止 production provider/mode/commit feedback/cache、Stage2/Rc/final/ATE/RMSE、任何
adaptive rescue。实际 F0--F3 PASS：held-out clean 0 alarm/0 segment；target 在 affected #15 报警，
30/30 覆盖且 healthy 0 alarm/0 segment。但 locked last-zero segment 在 injection 后继续 10.242171s，
TP/FP/FN=30/37/0、precision=0.447761<0.80、recall=1，故 F4 唯一停止裁决为
`CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`。依协议 dynamic D0--D4、完整 ctest、production、Stage2/Rc/final/
ATE/RMSE 全部 `NOT_RUN_DUE_TO_EARLIER_FAILURE`；没有 rescue/sweep。直接 CUSUM 12/12 与旧 conditional
13/13 通过，truth-blind successful forbidden opens=0。结果见
[`PL_PERSISTENT_CUSUM_PREFLIGHT.md`](../ie_0911/PL_PERSISTENT_CUSUM_PREFLIGHT.md)。完成后普通非 force push。

# 0912-PL-THRESHOLD-SIGNAL-AUDIT 当前任务

`DONE / GROUP_STATISTIC_TASK_MISMATCH_PERSISTENT_PER_ANCHOR_SIGNAL_PRESENT`。用户授权执行
[`PL_THRESHOLD_SIGNAL_AUDIT_PROTOCOL.md`](PL_THRESHOLD_SIGNAL_AUDIT_PROTOCOL.md)：复用 f2ee3f0d sealed
Walk1 clean/injected 与锁定 PL `ae54fb8c` conditional replay，只做 group threshold sweep、逐 anchor
marginal/conditional innovation、clean/injected identity pairing、variance/contribution decomposition 和
persistent-signal 描述性诊断。[结果](../ie_0911/PL_THRESHOLD_SIGNAL_AUDIT.md)：原数字逐项复现；
`P_FA<=0.10` 的六档 sweep 均为 clean 0/224、affected 0/30，最大 T 的 DoF=5 upper-tail probability
为 0.433138，故 threshold 不是可合理小调即可解决。target conditional z injected mean 1.0926、30/30
正向、`Z_sum=5.9846`，paired `Z_delta=6.6220`；healthy 没有同等级正向 paired shift。统计 truth-blind
生成并封存，detector forbidden open=0；production detector/config/cache 未改，Stage2/Rc/LCB/recovery
与下一 detector 全部 `NOT_RUN/NOT_IMPLEMENTED`。按用户规则完成后停止。

# 0912-PL-CONDITIONAL-RAIM-FDE 当前任务

`DONE / PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM / PRODUCTION_AND_E2E_NOT_RUN`。用户授权实施
[`PL_CONDITIONAL_RAIM_PROTOCOL.md`](PL_CONDITIONAL_RAIM_PROTOCOL.md)，替代上一个未执行的
Projected GLRT v5 计划。当前只允许提取锁定 PL commit 的 conditional detector 合同，新增
source-neutral core、shadow replay、独立 evaluator 和工程测试，并在 locked Walk1 clean/injected
上执行 truth-blind preflight。实际 clean retained support 为 0，第一门通过；injected 的 30/30 planned
affected IDs 全部进入检测组，但 affected alarm 为 0，最大 `T=4.86052305024`、对应门限
`30.8561899404`，第二门失败。第三、四门、provider 注册、production、Stage2/final 和 E2E 依协议
全部 NOT_RUN；没有调参或扩大输入。完整结果见
[`../ie_0911/PL_CONDITIONAL_RAIM_PREFLIGHT.md`](../ie_0911/PL_CONDITIONAL_RAIM_PREFLIGHT.md)。
IE 基线 `d82d794e79f2d2550d6815ed687dff170af0e593`，
PL 基线 `ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`，recovery backend 基线
`cf287b4fec9bc5689317f302ccf4cdd927bfba81`。不调 detector/temporal/Stage2/kappa，不扩展 Walk2/3/
低冗余，保护旧结果、默认和 `doc/v2/ie_0911/`。仅按协议的普通、非 force 提交/推送流程交付。

# 0912-WINDOWED-FDE-E2E 当前任务

`DONE / ENGINEERING_PASS / DETECTOR_GATE_FAIL_MISSED_INJECTION / LOCALIZATION_NOT_EVALUABLE`。
用户授权实施
[Windowed FDE v4 与锁定 Walk1 E2E 协议](WINDOWED_FDE_E2E_PROTOCOL.md)，替代 grouped FDE
Walk1 gate failed 后的停止边界。实施基线为实际 HEAD `cf287b4fec9bc5689317f302ccf4cdd927bfba81`，
工作树仅有受保护的未跟踪 `doc/v2/ie_0911/`。v2/v3 与 legacy 默认保留；v4 仅由显式
`nlos.fde_windowed_test=true` 启用，固定 dyadic 50%-overlap window bank、chain-local Bonferroni、
full `P_gg` covariance test、负 GLS 与 observation-union merge。Stage2、Rc/local sigma、kappa、
LCB、final/fallback、evaluator 与四个 backend 文件冻结。工程门未通过前 Walk1 `NOT_RUN`；
detector screen 未通过则不运行 E2E，不调 policy、不扩大 Walk2/3/低冗余矩阵。T10=C2-C、T11=C、
C1--C3 限制不变；本任务获授权按限定路径提交并正常 push，禁止 force/rebase。

[完整结果](../ie_0911/WINDOWED_FDE_E2E_RESULT.md)：工程门权威汇总 384 tests、0 errors/failures/skipped；
锁定 Walk1 clean 1074/1074 完整测试、1010 windows、0 support，clean gate 通过。injected 同样完整测试，
但 30 个 planned injected IDs 上 TP=0、FP=0、FN=30，目标窗口最大统计量 `49.205998` 仍低于 Bonferroni
门限 `72.950243`；因此 injected gate 失败，六方法、Stage2/Rc/sigma/LCB/final 与扩大矩阵按预登记规则
全部 NOT_RUN。未调 detector policy；四个冻结 backend、论文结构和 roadmap 哈希不变。

# 0911-STAGE2-STATIONARITY-REPAIR 当前任务

`DONE / RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX / LOCALIZATION_BENEFIT_OBSERVED`。
按 [任务卡与 amendment](STAGE2_STATIONARITY_REPAIR_PROTOCOL.md) 和
[完整结果](../ie_0911/STAGE2_STATIONARITY_REPAIR_RESULT.md)，在实际 HEAD `3db4f317` 上封存 A--C
诊断后只实施 D2。raw-reference 的 numerical gradient floor 同样远高于 `1e-6`；Stage2 dominant
`x186[4]` gradient `0.0750243859` 经有限差分确认。原 50/100/200 outer 的 C-update 前后均保持该值，
故根因是 generic conditional LM 对 fixed-C 子问题过早数值停止，而不是 C-update coupling；D1/D3
均未实施。最小修复只在 objective/step/C-KKT 已通过而 navigation 未通过时，复用既有 V2 和同
graph/Values fixed-checkpoint recovery；模型、预算、全部阈值、FDE/Rc/sigma/LCB/final/evaluator均冻结。
完整 CTest/GTest 370/370；锁定 oracle Stage2 在 outer42 达到 navigation `7.10213e-7`，估计
`c_hat=0.471346m`、`sigma=0.075074m`、LCB=`0.321198m`。四个 final 均有效且无 fallback；LCB
RMSE `0.170086m` 比 suppress `0.181637m` 低 `0.011551m`（`6.3592%`）。仅支持单一 oracle-backend
development sanity check，不是 detector E2E；T10=C2-C、T11=C、C1--C3不升级。用户随后明确授权
将本轮交付提交并推送到当前 Git 分支；受保护的未跟踪 `doc/v2/ie_0911/` 不纳入提交。

# 0911-ORACLE-SUPPORT-BACKEND 当前任务

`DONE / RECOVERY_BACKEND_NOT_OPERATIONAL`。按 [任务卡与 amendment](ORACLE_SUPPORT_BACKEND_PROTOCOL.md) 在实际 HEAD `15b32f5a` 上完成 Walk1 normal oracle-support backend scientific sanity check；[完整结果](../ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md)。30/30 planned support IDs 与锁定目标 link 精确对齐，estimator 进程树未读取 injection amplitude 或 GT。非空 Stage2 实际执行 50 次 conditional optimizer call、累计 110 LM iterations/359 inner trials；objective、step、非负 KKT 最终通过，但 navigation stationarity `0.0750243859` 未达到冻结 `1e-6` 容差，返回 `MAX_REFIT_ITERATIONS`。未发布 cache，Rc/sigma/LCB 与四个 final 均不可用；没有放宽容差、导出未收敛 `c_hat`、算法重试或 correctness 修改。按 Case B 唯一裁决 `RECOVERY_BACKEND_NOT_OPERATIONAL`，low redundancy 为 `NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL`，停止 detector 开发。FDE/Stage2/Rc/LCB/optimizer/evaluator源码与 HEAD hash 一致。用户随后明确授权提交并推送本轮交付；受保护的未跟踪 `doc/v2/ie_0911/` 不纳入提交。

# 0911-FDE-FORENSICS 当前任务

`DONE / ENGINEERING_PASS / WALK1_GATE_FAILED`。按 [任务卡与 amendment](FDE_FORENSIC_PROTOCOL.md) 完成原算法取证、封存裁决、唯一 C 分支 grouped FDE 和固定 Walk1 六方法验收。见 [完整结果](../ie_0911/FDE_FORENSIC_AND_FIX_RESULT.md)。完整 CTest 31/31；physical +0.5m / prefit −0.5m 精确核验，原 truth-window aggregate 显著，hold-out 求解失败故 masking INCONCLUSIVE。新最大连续组两侧均零 support；clean RMSE约0.164m满足门，injected无TP，扩大矩阵 `NOT_RUN_WALK1_GATE_FAILED`。Stage2/recovery数值逻辑、默认、旧结果与受保护材料保留；T10=C2-C、T11=C、C1–C3不升级。任务结束，不调参；用户随后明确授权提交并推送本轮实现，受保护的未跟踪 `doc/v2/ie_0911/` 不纳入提交。

# 0911-NLOS-INJECTION 当前任务

`DONE / EXECUTED_ZERO_RECOVERY_SUPPORT / BENEFIT_EVIDENCE_INSUFFICIENT`。用户授权恢复后的剩余实验和独立评价均完成；[机器生成结果](../ie_0911/NLOS_INJECTION_EXPERIMENT_RESULT.md)、[协议](NLOS_INJECTION_PROTOCOL.md)。12个scenario条目：6 clean、4 injected执行，另2 injected因clean Cauchy失败跳过。六方法长表72行：57成功、3个Cauchy算法失败、12跳过；10个共享producer完成，另1次用户暂停中断保留。

四个injected均无新增偏置真阳性，10个执行场景temporal support均为空；LCB/suppression相同，不能证明检测后恢复优势。唯一完整normal/low配对Walk3收益均0，未观察到低冗余收益放大。四对实际计划/噪声/原始初值一致；六clean门精确planned时间核验未改变准入；算法/科学参数/冻结材料hash一致。完整CTest30/30、新指标工程测试7/7。Table A/B、全部失败/命令/封存身份见证据目录 `/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01`。

历史[暂停交接](NLOS_INJECTION_PAUSE_HANDOFF.md)已由RESUME.json COMPLETE解除；已完成项和算法失败项未重试。T10=C2-C、T11=C、C1–C3不升级。完成本轮后停止，sensitivity/新算法/新phase/提交/push均未执行。

用户随后明确授权将本轮实现与报告提交并推送到当前Git分支。原未跟踪 `doc/v2/ie_0911/` 保留；完整运行数据、二进制和大体积证据仍留本地，Git报告中的绝对证据路径指向原工作空间。

# UWB-IMU-IE sprint 状态

## 0911-RECOVERY-FDE-V2 当前任务

`DONE / LOCALIZATION_AND_ENGINEERING_GATES_PASS / WALK1_ZERO_CANDIDATE`。实施前授权见 [当前任务卡与 amendment](RECOVERY_FDE_V2_PROTOCOL.md)，[完整结果与命令](../ie_0911/RECOVERY_FDE_V2_RESULT.md)。fresh legacy/all-range/Cauchy full Walk1 aligned RMSE=0.169642149/0.163853268/0.163847231m，定位门通过；修复paper-only robust loss与IRLS不一致。完整Gaussian post-fit FDE、SUCCESS_EMPTY和同graph/Values零优化复用已实现。最终CTest29/29；固定smoke与full Walk1五方法均完成，分别55/1074条planned全检、零候选，Stage2及四final optimizer=0，协方差AVAILABLE。六输入五方法30项prepare通过，其它输入精度矩阵NOT_RUN。历史边界以下保留；T10=C2-C、T11=C、C1–C3不升级。用户于任务完成后明确授权提交并推送本轮实现；受保护的未跟踪 `doc/v2/ie_0911/` 不纳入提交。

## 0911-FDE-STEP1 用户授权 amendment（2026-09-11）

当前任务：`DONE / IMPLEMENTED_AND_ENGINEERING_VERIFIED / SIX_INPUT_NEGATIVE_INCOMPLETE`。本轮用户实施计划替代此前 STEP2 结束后的停止边界，授权论文主代码路径
新增 `imu_aided_fde` Stage1，同时完整保留 `automatic_discovery` 为 legacy/development。
FDE 先用与 fixed-rejection 共享的 preliminary tightly-coupled LM，在实际 planned raw UWB factor 上按
`r=h+beta-z`、factor `sigma`、1-DoF `Chi2inv(0.99)=6.6349` 做双边 fault detection；只将
`r<0` 的 fault 作为正 excess-range candidate。按 `(tag,anchor,time,obs_id)` 稳定排序，健康观测与严格
`gap>T_gap` 切段，`count>=n_min && duration>=T_min` 才保留。空候选仍为 Stage1 success 并运行既有
raw Stage2；无 GT/oracle 输入。该前端属于 RAIM/FDE family，不是完整 ARAIM 或 certified integrity。

Stage2、共同参考 `R_c`、local sigma、LCB、suppress、final graph/Values 与一次 fallback 定义不变；
FDE final 不使用 live-C rescore 的规则不因本 amendment 改写。新增 provider-aware identity/artifacts，
legacy/FDE cache 不互用；FDE 不与 `structured_bias_only` 组合。范围仅为实现、工程测试、Walk1 起始后
`[8,11]s` smoke、六输入五方法串行运行与冻结 GT 的独立评价；失败、fallback、zero-candidate/
zero-coverage 原样保留，不调概率、temporal 参数、kappa、区间或阈值，不提交或 push。
工程门 29/29 CTest 和 Walk1 `[8,11]s` 最终 smoke 均通过；smoke 是合法 zero-candidate，raw Stage2 与
四个 final 共用同一 cache。六输入中 3 个 FDE reference 成功但 zero-candidate raw Stage2 失败，3 个
reference LM 达到最大迭代；6/6 producer 无 cache，24 个 candidate-dependent final 均 parent unavailable。
锁定 GT 评价和 30 行聚合已生成，paired 四指标与 coverage 变化全部 unavailable；未调参或重跑算法。
T10=C2-C、T11=C、C1–C3 与冻结论文结构/roadmap 不变。最终结果见
[`../ie_0911/FDE_STAGE1_RESULT.md`](../ie_0911/FDE_STAGE1_RESULT.md)。

## 0911-STEP2 真实精度实验（2026-09-11）

用户授权的第二个且最后一个开发工作包已执行并停止：
`EXECUTED_WITH_STAGE1_BLOCKED_METHOD_COMPARISONS / STRICT_SYSTEMIC_MAX_OUTER_RULE_NOT_TRIGGERED`。
[完整结果](../ie_0911/STEP2_RESULTS.md)。六条 full 输入和两个预冻结低冗余条件均实际运行；独立 Cauchy
得到 5 条轨迹（其余 3 条 final LM 失败），但六个主 producer 与两个低冗余 producer 全部在 Stage1 失败，
所有 suppress/structured/LCB/full final 因 cache unavailable 未运行。严格“至少四个主 producer 同为
MAX_OUTER_ITERATIONS”规则未触发（实际 2/6），但新方法真实精度比较仍被 Stage1 两类失败完全阻塞。
LCB逐序列收益、低冗余收益、主全额消融差异均 UNAVAILABLE；range reference provenance 不闭合，统一 N/A。
最终 CTest 27/27；首次陈旧测试 ABI 导致的2个SIGSEGV及修复后重跑均保留。T10=C2-C、T11=C、C1–C3
不升级；不进入第三步、不调参数、不自动 push。

## 0911-STEP1 用户授权 amendment（2026-09-11）

本轮用户实施计划授权第一步 LCB 固定部分补偿及同集合全额消融，替代旧下一任务边界。
仅 `lcb_partial` / `lcb_fixed_full` 允许按段冻结动态 offset；旧方法 accepted live C 规则保留。
复用 Stage2 共同参考 R 与幅值列映射，独立有限性、满秩、正定检查后由单位向量求解得到
`sigma_c_local=sqrt(diag(R^-1))` 米，沿用原容差，不加 jitter/damping/prior。
按段 `delta=max(0,c_hat_stage2-2*sigma_c_local)`，结构/数值不合法或 delta=0 suppress；
不串联 eta/s/gamma gate。全额 variant 严格复用上述集合，仅 delta 改为 Stage2 幅值。
最终 raw factor 残差 h+beta+delta-z，每个恢复观测一次，无 live C；sigma 为局部诊断，
不是校准置信保证或最终 bias 后验。导航、残差、协方差来自同一 final graph/Values；
保留求解判据与一次 suppress fallback，固定模式无 live-C final rescore。
仅实现、工程测试、SFUISE Walk1 起始后 [8,11]s 固定 smoke、六输入四方法加载/启动检查；
第二步精度矩阵 NOT_RUN，不按结果调 kappa/区间/阈值，不自动 push。
T10=C2-C、T11=C 与 C1–C3 限制不变，旧结果/默认/用户材料保护。

当前 0911-STEP1：`LOCAL_ENGINEERING_DELIVERY_COMPLETE / REAL_SMOKE_FAILED_STAGE1`。
[交付结果](../ie_0911/STEP1_RESULT.md)：catkin build exit0，CTest 26/26（GTest XML 161 项、0 failures/errors）；
六输入四方法 24/24 prepare-only 通过。partial/full 非零固定补偿经既有 certified production final 优化，
独立 16 条残差与同恢复集合核验通过。真实 Walk1 [8,11]s Cauchy 成功，但自动 Stage1 原 50 outer 上限失败，
Stage2/候选 final NOT_RUN；无调参/重试算法。历史普通 solver fallback 与首次路径/build/启动失败均保留。
最终源码/库/命令/轨迹/局部 sigma/冻结 offset 见该交付；T10=C2-C、T11=C、C1–C3 不升级。
第一步已结束，第二步六输入精度矩阵 NOT_RUN，不自动继续。下方下一任务文本为此前历史边界。

本文件是工程协作与任务交接的统一入口。它记录事实和任务状态，不代替方法合同或实验合同。

## Git 交付与本地存储维护（2026-09-10）

按用户授权提交 T10 收口代码、文档和精简结果摘要；完整本地运行目录由 `.gitignore` 排除，保留必要回归输入。参见 [Git 证据范围](evidence/README.md)。清理中断 pack 和旧本地自动检查点，不改写正式分支历史；仍保留当前会话的活动 capture。此维护不增加科学运行、不改变 C2-C，T11/T12 仍为 `NOT_RUN`。本次提交前 Python 回归实际 4/4 通过（冻结阶段的历史记录仍保留原计数）。

## 冻结材料

按以下顺序阅读：

1. 冻结论文结构：[`../v2/paper_structure.tex`](../v2/paper_structure.tex)
   SHA-256：`8ac373919823d755d9c1b4ceb807527432d16b69d368042e19aa56d93dc67144`
2. 执行路线图：[`../v2/v2_roadmap.md`](../v2/v2_roadmap.md)
   SHA-256：`b9bb63b65ebdb8a505bf99b26181318c6c64da1a72817b08cfa61315e2333bdb`

roadmap 所称的 `UWB_IMU_IE_System_Centered_Structure_v3.tex` / `Structure_v3`，在本仓库对应
`doc/v2/paper_structure.tex`。除非 amendment 表明确登记并获确认，上述文件视为冻结输入。

## 项目定位

本项目是可复现的 UWB–IMU 全轨迹后处理系统：复用现有 C++/GTSAM 后端；当前论文路径以 IMU-aided
residual FDE 生成 temporal support，再执行去正则分段 refit、candidate-excluded `eta/s/gamma` 评分和
冻结 final；原非负 L1/TV 自动发现保留为 legacy/development。实验分别用于验证系统能力、多链路 NLOS
效果、门控价值和未来观测作用，当前六输入结果不支持运行收益。

## 仓库快照

以下是本次新环境 T00 开始前的只读快照：

| 字段 | 值 |
|---|---|
| 日期（UTC） | `2026-09-05T15:43:44Z` |
| 分支 | `feature/uwb-imu-fusion-ie-postprocessing` |
| 提交 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6` |
| T00 开始时工作区 | clean；当前未提交项只包含本次新环境证据和文档更新 |
| roadmap 参考提交 | `cfe6d29` |
| `cfe6d29..HEAD` 差异范围 | 仅 `doc/` 与 `AGENTS.md`；方法源码未改变 |

上一台 `/home/dev` 机器的 T00 证据已保留；当前结论只以新的 `/home/mint` 隔离证据为准。

## 当前进度

状态词：`NOT_STARTED`、`IN_PROGRESS`、`BLOCKED`、`DONE`。`DONE` 只在对应验收证据齐全时使用。

| 项目 | 状态 | 产物/说明 |
|---|---|---|
| T00 前最小启动准备 | `DONE` | `AGENTS.md`、本文件、[`REPO_AUDIT.md`](REPO_AUDIT.md) |
| T00 新环境审计与旧基线复现 | `DONE` | 当前源码 build exit 0；tests 实际执行为 38 项、2 failures；SFUISE Walk1 两次隔离运行均 exit 0，轨迹逐字节一致，aligned ATE RMSE `0.169642 m`。见 [`REPO_AUDIT.md`](REPO_AUDIT.md) |
| T01 唯一合同与论文工程 | `DONE` | oracle reference 边界、三类 debug 标签和 A01 技术接受状态已收口；冻结 hash、链接及文本检查通过。实现与 U11 不因规则接受而视为通过 |
| T02 paper 输入路径与最小运行入口 | `DONE` | 本轮指挥/审查会话最终验收通过，T02-R01–R07 均通过复审；独立复核包见 [`evidence/t02_final_review_20260906T083514Z/`](evidence/t02_final_review_20260906T083514Z/)。该状态只支持基础工程能力，不使 C1–C3 成为 `SUPPORTED` |
| T03 NumPy recoverability golden reference | `DONE` | 本轮指挥/审查会话接受 T03-R01–R04；默认/Sandybridge 各 22/22、跨 BLAS 容差、同环境字节复现及 100 个独立 Schur 检查均通过。见 [最终独立复核收口](evidence/t03_final_review_20260906T110914Z/VERIFICATION.md) |
| T04 Oracle 分段联合 refit | `DONE` | 独立复核已接受 T04-R01–R03；范围仅为 fixed-calibration、oracle-support debug、live-`C` 最终联合图。见 [T04 最终复核收口](evidence/t04_final_review_20260906T143023Z/VERIFICATION.md) |
| T05 sparse recoverability score | `DONE` | T05-R01–R05 已由独立复审接受；仅关闭当前已测试 sparse 支持域内的 oracle-debug 工程范围。完整复核包、15 个 unavailable case、被审查源码和哈希见 [最终收口](evidence/t05_final_review_20260907T022011Z/VERIFICATION.md)；不代表正式 gate、全规模性能或通用误差证明 |
| T06 automatic support discovery | `DONE` | T06-R01–R05 已由独立复审接受；范围仅为 reviewed fixed-calibration development engineering scope。真实短输入仍为 17 段（short 13、boundary 0）、Stage 2 收敛、3 个 group 均因含 short 而评分不适用；有效评分闭环只由独立 synthetic engineering GTSAM fixture 支持。见 [最终独立复核收口](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md) |
| T07 deterministic scenario/cache | `DONE` | `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；T07-R01--R03 均已独立复审接受。见 [`T07_IMPLEMENTATION.md`](T07_IMPLEMENTATION.md)、[最终收口证据](evidence/t07_final_review_20260907T070102Z/VERIFICATION.md) 与 [自包含归档修复](evidence/t07_closeout_fix_20260907T072733Z/VERIFICATION.md) |
| T08 冻结决策、最终联合推断与一次 fallback | `DONE` | `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；R01/R03 保留各自限定 scope，R02 为 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`。见 [最终独立复审收口](evidence/t08_final_review_20260907T121510Z/VERIFICATION.md) |
| T09 统一基线、缓存、批量执行与评估 | `DONE` | `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE / AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`；最终独立复审接受 R03-B，并与此前已接受的 R01–R08 合并关闭 T09 development-engineering 范围。真实 final 保持 `41/627`，parent canonical-v2 内容身份、ledger/mask/audit 与 failure accounting 反例通过。见 [最终独立复审收口](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md)。这不是 automatic accepted-candidate 科学证据、正式 RQ、C1–C3 支持或 T10 gate lock |
| T10 validation/gate | `DONE` | `FROZEN_C2_C_USER_SCOPED_CLOSEOUT`：仅本轮授权收缩范围完成。两base step2×A/B/C，N=6400I/1600I/3200I；30/30 final有效，s_fit/full_gate决定全同，structured RMSE1好5差/P95全差。LOS20102独立对照通过、20101对照数值失败保留。原完整T10/held-out验收未完成，T11/T12未运行。[收口](T10_CLOSEOUT.md) |
| T11 prefix | `DONE` | `FROZEN_T11_C_LIMITED_SCOPE`：6 fresh + 2 U13 已执行；科学 4/6 全链、8/8 final 有效；20101 H0/H1 Stage1失败，完整U13仅20102 PASS；truth未读、指标NOT_RUN。[收口](T11_CLOSEOUT.md) |
| T12 正式指标 | `DONE_0911_LIMITED_DEVELOPMENT_BLOCKED` | 0911 STEP2 限定矩阵已执行；5条 Cauchy conditional aligned 轨迹，候选方法8/8条件均被 Stage1/cache 阻塞；正式 locked/held-out RQ 与 U14 仍 `NOT_RUN`。见 [`STEP2_RESULTS.md`](../ie_0911/STEP2_RESULTS.md) |
| T13 论文结果与发布 | `NOT_STARTED` | 正式论文数字、图表与 release `NOT_RUN` |
| 0911 residual-FDE amendment | `DONE_NEGATIVE_INCOMPLETE` | FDE 工程门与 short smoke 通过；六输入 6/6 producer 无 cache、24 个 candidate-dependent final unavailable；见 [`FDE_STAGE1_RESULT.md`](../ie_0911/FDE_STAGE1_RESULT.md) |

## A14 指挥接受与实施前登记

`REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE` 来自本指挥会话，不是外部REVIEW.md。
A13唯一方案已技术接受，仅本轮显式opt-in development实施；执行[A14 amendment与冻结判据](T10_A14_AMENDMENT_PROTOCOL.md)。
T10 IN_PROGRESS；工程门通过前pilot NOT_RUN；A12负结果、A08历史15/18及全部claim限制保留。

## A13 指挥接受与运行前登记

本轮登记指挥接受 `REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE`。
A12限定诊断接受，不是estimator成功或validation准入；A12负结果与A08历史15/18及全部历史限制保留。
执行[A13协议](T10_A13_PROTOCOL.md)：只恢复P1 step seed10101 call50并通过原身份门，零optimizer iterate，
只审计实际40个PIM、协方差传播/白化和同一冻结最弱方向。生产协方差不改，后续修复仅草案，完成即停止。
T10保持IN_PROGRESS；truth/GT/chain/Stage2/validation/test/T11均NOT_RUN。

## A12 指挥接受与运行前登记

本轮登记指挥接受 `REVIEW_ACCEPTED_A11_BOUNDED_FIRST_BLOCK_DIAGNOSTIC_SCOPE`。
这是 A11 限定诊断接受，不是 estimator 成功、validation 准入或外部 REVIEW.md。
执行 [A12 单 checkpoint 协议](T10_A12_PROTOCOL.md)，仅 P1 step seed10101 call50；先恢复身份和静态尺度审计，
支持特定阻尼失衡假设后才可登记单次 A/B（各150次/30秒，A不复现则B不运行）。
T10 保持 IN_PROGRESS，A08 历史15/18限制保留；chain/Stage2/gate/held-out/scheduler NOT_RUN。

## A11 指挥接受与运行前登记

本轮指挥接受 `REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE`。
这是 A10 限定交付验收，不是 estimator 成功、validation 准入或外部 REVIEW.md。
T10 保持 IN_PROGRESS；A08 图级 FD 15/18 限制保留。
本轮执行 [A11 有限协议](T10_A11_PROTOCOL.md)：只用冻结 P1 三场景 seed10101，最多3进程/每次120s，
只观察 outer1 首 conditional block；第49/50实际接受完整delta通过原FD判据才可在同一optimizer续至总200次。
不进入chain/Stage2/gate，不运行held-out，不改solver/Jacobian或/usr/local。

## 当前任务结论与停止边界（2026-09-11）

[T11_CLOSEOUT.md](T11_CLOSEOUT.md) 保持唯一裁决 **T11-C**。其后的用户授权 0911 STEP2 已作为最后一个
开发工作包执行并停止；完整结果见 [`STEP2_RESULTS.md`](../ie_0911/STEP2_RESULTS.md)。本轮没有进入第三步，
没有正式 held-out 准入或 claim 升级。下方旧 T10/T11/T12“下一任务”文本均为历史记录。

## T11 当前授权（2026-09-11）

用户给定实施计划替代下方 T10 历史停止边界。仅 condition A/step2/20101–20102，H=0/1/2，历史 [3,6]s；两种 final、两条 U13 和 fresh H2 固定模型诊断。单一 [T11_MANIFEST.json](T11_MANIFEST.json) 在实现/运行前登记。保留已见 validation 身份，不锁 gate，不升级 C1/C2/C3；T10 C2-C 不变。下一任务只为 T12 输入语义核实及限定评估。

## 当前任务与停止边界

本轮单工作包已完成并冻结：[T10_CLOSEOUT.md](T10_CLOSEOUT.md)，唯一裁决 **C2-C**。
普通 correctness、六输入诊断、重启恢复和磁盘精简均在本包闭环；原完整T10实验未冒称验收。
T10不扩展；下一优先T11，随后T12，本轮均 NOT_RUN。旧记录下文保留为历史，不覆盖当前边界。
用户追加授权删除可再生成/编译中间证据，精简34.15 GiB；原完整冻结包保留范围见 retention_20260910。

2026-09-10 用户已授权实施 T10 单工作包收口，替代旧完整计划及逐轮停止边界。
先集中 correctness，再两条既有 validation step2 × A/B/C 六输入（A 可审计复用），不调参、不新增 solver/seed/test。
范围收缩 amendment 已由用户本轮明确确认；C2 按 C 优先、再 A/B 的冻结证据规则裁决，T11/T12 仅交接。
运行前登记：[单一 manifest](T10_CLOSEOUT_MANIFEST.json)。当前为实施中，尚未冻结或验收。

2026-09-10 用户要求停止逐轮最小修复，改为T10完整交付方案。
已制定 [T10完整交付计划与总任务prompt](T10_COMPLETE_DELIVERY_PLAN.md)，替代尚未执行的R09最小建议：
集中工程修复/真实全链验收、有限validation选择、锁定与准入审计、held-out RQ3和最终claim裁决，
常规correctness在同一工作包内闭环，设总预算与有限返工边界。本次仅制定计划，未实施或运行新实验；
实际启动前须按总任务登记master protocol/manifest，旧R08冻结包、失败和数据角色保持。
T10仍IN_PROGRESS，计划不等于验收完成或gate/test已执行。

2026-09-10 指挥独立复审R08：1320项冻结哈希一致，36份有效final的轨迹指标与可用配对差值、
decision/final bias分别独立复算一致。接受有限validation执行及负结果，不接受完整工程门或gate准入。
新发现8组N均为6400I，eta=1/(6400*s²)，当前矩阵不能分离eta相对s的增量。LOS两次接口失败证明
完整零候选生产链fixture漏测；另有turn02陡ramp metadata失败、turn01缓ramp无C lambda exhaustion，
不能概括为“只剩LOS”。中断行缺失计数误写0、CSV bias未标decision阶段需在新修正版表中澄清。
下一建议仅闭合LOS全链并各一次correctness复核、只读分类ramp失败；之后才设计不同N的有限诊断与cache选择。
本复审未修改estimator或新跑优化，test/gate/C1–C3不升级。
见 [R08复审与R09任务](T10_A19_R08_REVIEW_NEXT.md) 及
[独立AUDIT](evidence/t10_a19_r08_independent_review_20260910/AUDIT.json)。

T10-A19-R08 已按运行前协议执行固定 validation characterization 矩阵。validation role/split/ancestry/content
身份贯穿实际C++ producer、Stage2 request、冻结decision与final content identity；工程门完成CMake ABI、
2/2 discovery身份、27/27 refit、20/20 inference、真实无C recovery/fallback、三类拒绝和两个完整raw
零优化prepare。首个LOS暴露空partition误用非空C handoff request，失败ticket原样保留；仅修共享dispatch并
通过真实门后继续尚未运行的固定行。

12条输入均至多运行一次：8条完成“找段—重估—评分”及五策略final，3条保留首次算法失败，1条在系统
崩溃时已进入算法而封存不重试。8条score均1 group/1 eligible/0 unavailable；三gate在每条上决定相同：
turn01的step1/2/3使用16/32/48候选，其余已评分输入全部抑制。7条有效structured对比的历史RMSE为1好6差，
P95为0好7差；最大负收益是turn01/ramp_steep的`+0.028183407/+0.043758707 m`。turn02/step1仅RMSE改善
`0.000219404 m`但P95恶化`0.000256794 m`，不构成重复收益。turn01/ramp_gentle的四个抑制final均在唯一
fallback后失败，配对UNAVAILABLE。两条LOS都没有有效final，all_range代价及容忍判据UNAVAILABLE。

评价前冻结1320项源码/配置/身份/score/decision/final payload并复核零mismatch；12条estimator trace的truth
打开均为0，随后8个独立evaluator全部exit0。test30101--30103未生成/读取/运行，seed10101未返回。
Stage2 export中保守legacy `UNBLINDED_DEVELOPMENT`字段与实际validation身份并存且已披露，所有产物仍
`consumable=false`。T10保持IN_PROGRESS，gate不锁、C1--C3不升级。唯一主阻塞是把零候选LOS的
support/refit/score/all_range路径在validation身份下闭合，再进入有限cache选择与held-out test；本轮不重跑
已消费validation ticket。[完整交付](evidence/t10_a19_r08_validation_compare_20260910T115109Z/VERIFICATION.md)。

2026-09-10 指挥会话独立复审R07：387项冻结payload哈希一致，五final终行原四项AND通过；
冻结后独立复算五策略全记录/历史ATE及配对差值、structured bias RMSE，与交付一致。
接受限定 `UNBLINDED_DEVELOPMENT` 配对里程碑：历史RMSE/P95使用修正比全部抑制高
0.762812/1.855704 mm；三gate同决定，没有eta增量比较。结束seed10101求解器支线，T10仍IN_PROGRESS。
下一任务建议为绑定新validation基础轨迹的固定P1/固定工作点12输入有限比较，需先补实际role握手、
零候选/零eligible final和scenario-aware evaluator；本复审未执行该批次或修改estimator。
见 [R07独立复审及R08任务prompt](T10_A19_R07_REVIEW_NEXT.md) 与
[独立复算记录](evidence/t10_a19_r07_independent_review_20260910/AUDIT.json)。C1–C3不升级。

T10-A19-R07 已按预登记完成。工程门实际调用同一 production runner 的 certified callback：正常无C
recovery成功，受控失败后的唯一fallback也走相同policy且成功；两者日志隔离、handoff count 0，错误身份
在callback前拒绝。受影响refit/export 8项、inference/policy 15项及完整raw prepare全部通过；R06静态图
只读核对339 factors/123 X-V-B keys/0 C，未被当作checkpoint消费。

唯一fresh ticket从原raw初始化，exit0/external `83.586104785 s`，estimator truth open 0。Stage1为
90 outer/327 calls/468 trials，Stage2为24 outer/96 calls/189 trials，保存2段/32候选及完整score：eta
`0.58397847413582771`、s `0.016357299043480211 m`、gamma `0.1431107780213654/1.4164987078762303`。
五个final各执行一次并全部有效、fallback 0。suppress_all及fit_only/s_fit/full_gate都抑制32/32，终态
identity/trajectory相同；structured_debias使用32/32。独立evaluator在冻结387项payload后读取truth：
共同参考`[3,6]s` ATE RMSE/P95为`0.0151169771/0.0210468736 m`，structured为
`0.0158797893/0.0229025775 m`，差值`+0.0007628122/+0.0018557039 m`，即该已见seed上使用修正略差。
structured final accepted bias RMSE `0.005767514545 m`、bad correction 0/32，但bias-good不等于trajectory
beneficial。三个gate决定相同，无法评价eta增量；零接受risk为UNDEFINED。R05 truth事件永久保留，仍为
`UNBLINDED_DEVELOPMENT`。R07成功后结束本seed工程支线；唯一主阻塞转为预登记有限validation与held-out
未见数据证据，C1--C3及正式gate均不升级。[完整交付](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/VERIFICATION.md)。

2026-09-10T10:41:17Z 已在任何 R07 源码修改、工程执行、新 ticket 前登记
[`T10_A19_R07_PROTOCOL.md`](T10_A19_R07_PROTOCOL.md)。唯一 scope 扩展是把现有333-bit
`PAPER_CERTIFIED_PAIR_REDUCTION_V1` 接入 development final 的全部抑制/无C recovery与唯一fallback；
无C禁止inexact handoff，原四项AND、lambda、容差、200 outer、Stage1/有C Stage2、score、gate和
legacy/default路径不变。工程门通过后只允许一次fresh P1 step seed10101完整raw运行及原五策略比较，
各预算和总5400秒硬限保持；本seed继续`UNBLINDED_DEVELOPMENT`，不升级C1--C3。

2026-09-10 指挥会话独立复审R06：核验119项冻结payload，独立复算eta/s及structured_debias轨迹/bias
指标一致；完整自动评分成立，配对收益仍因参考失败UNAVAILABLE。四个Suppress策略的recovery/fallback
数值轨迹一致，第2--200轮梯度停在1.1159186055e-6；源码确认空accepted recovery、空C LM及fallback
未使用development certified policy。下一步仅显式扩展同一certified policy到该无C分支，禁止handoff及
放宽标准，不调gamma。按固定bias标签三个gate拒绝32/32 good候选是当前单输入观察，不等同轨迹损害。
见[R06复审与R07任务](T10_A19_R06_REVIEW_NEXT.md)。本复审无新优化/实现修改，保持UNBLINDED_DEVELOPMENT及claims边界。

T10-A19-R06 已按下述预登记完成限定实施与运行。生产导出从同一 Stage2 graph/Values 写出 41 X、41 V、
41 B、2 C（125 keys/1027 scalars），production reader 逐位回读一致；成功 manifest 在导出及评分完成后
原子发布。24 份 handoff 审计严格 JSON 有效，inner/guard/qualified 与 block_status 一致。14/14 受影响
测试、真实含非零/零边界 C 的 production export 反例门、完整 raw prepare 及小型
writer→reader→score→decision→final→evaluator 链均通过，solver/工作点未改。

科学 attempt1 因调用者漏建 `pilot/` 父目录在 raw/初始化前 exit2，日志明确支持一次启动修复；新 ticket
attempt2 exit0、external 103.136841s、truth/GT open 0。Stage1 90 outer、327 calls/468 trials 后冻结
2 段/32候选；Stage2 24 outer、96 calls/189 trials及1次限定 handoff 后收敛；同一完整终态成功导出并完成
1/1 eligible group 评分：eta `0.58397847413582771`、s `0.016357299043480211 m`，两段 gamma
`0.1431107780213654/1.4164987078762303`。

五个 final 各执行一次。structured_debias Use 两段/32候选并成功，无fallback；完整raw-frame ATE
RMSE/P95为`0.0240652206/0.0394080403 m`（41点），`[3,6]s`为
`0.0158797893/0.0229025775 m`（16点），final bias RMSE `0.005767514545 m`，bad correction 0/32。
suppress_all及fit_only/s_fit/full_gate均Suppress且共享等价决定，四者recovery与一次fallback都在冻结
200 outer失败；零接受risk为UNDEFINED，参考trajectory不存在，故所有相对suppress_all ATE差值
UNAVAILABLE。三个gate未区分本输入。评价前冻结119个payload hash；首次evaluator在hash阶段因前缀解析
exit1，修复只涉及hash前缀并登记freeze amendment，第二次exit0。R05 truth提前暴露永久保留，结果仅为
`UNBLINDED_DEVELOPMENT`。完整证据见
[R06 VERIFICATION](evidence/t10_a19_r06_live_c_score_compare_20260910T092530Z/VERIFICATION.md)。

T10继续`IN_PROGRESS`，C1--C3、正式validation/test/gate lock与论文claim不升级。唯一主阻塞是冻结
suppress-all recovery及其fallback均未在200 outer内收敛，导致共同参考轨迹和配对收益结论缺失；本轮不改
solver、阈值、精度或预算。

2026-09-10T09:25:30Z 已在任何 R06 修改、工程运行、新 estimator ticket 与新 truth 读取前登记
[`T10_A19_R06_PROTOCOL.md`](T10_A19_R06_PROTOCOL.md)。R06 保持共享 development schema、solver、333bit、
handoff guard、工作点和预算冻结，只修生产 Stage2 X/V/B/C 完整原子导出/逐位回读与严格真实 handoff JSON。
工程门必须调用 runner 同一 production writer/reader 并沿 score→decision→final→fixture evaluator 成功路径；
通过后直接签发一次 fresh P1 step seed10101 raw ticket。R05 truth 提前暴露事件永久保留，本 seed 后续结果均标
`UNBLINDED_DEVELOPMENT`，不恢复盲测身份、不升级 C1--C3。

2026-09-10 指挥会话独立只读复审R05：逐block计数、终行阈值与实际runtime身份一致，接受完整输入
Stage2在outer24收敛的日志级里程碑；终态缺失，未独立重算同一最终graph/Values。当前需修生产输出：
残缺final_values实际仅246行B/41 keys，无X/V/C；23份handoff审计既含非法nan，又将已收敛inner误写false。
下一轮限定完整live-C输出/严格JSON及真实生产导出路径回归，随后一次非盲development评分与比较，不扩solver。
R05提前truth读取事件保留；新ticket不会恢复同seed的盲测独立性。见[R05复审与R06任务](T10_A19_R05_REVIEW_NEXT.md)。
本复审未修改估计器或新跑优化，完整score/final收益仍NOT_RUN，C1--C3不升级。

T10-A19-R05 已按[预登记协议](T10_A19_R05_PROTOCOL.md)执行。共享 factory/validator 保留精确
`A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY` schema；完整 raw prepare 在 371-factor/123-value 建图后
实际接受生产请求且0 optimizer call，小型真实 C++/GTSAM 链完成直接 producer/consumer/evaluator 握手。
唯一 fresh ticket 的 Stage1 在90 outer、327 calls/468 trials收敛并冻结2段/32候选；Stage2 在24 outer、
96 calls/189 trials（1次限定handoff）满足原四项AND，首次越过R02 outer15。随后旧
`a17_graph_io::values` 只支持X/V/B，导出同一Stage2 Values遇live C即抛
`NUMERIC_REFERENCE_UNSUPPORTED:unsupported value`；进程exit2/82.672844s。score/decision/五final均
`NOT_RUN`，不存在实际Use/Suppress决定或轨迹收益数字，按停止边界不修复重跑。

边界事件同时封存：agent在科学decision冻结前为设计evaluator查看了generation manifest及step bias truth
表的表头/首末行；这违反评价truth读取时序，虽冻结可执行文件/阈值未随后改变且estimator strace确认0次
truth打开，也不得用本run作独立收益评价。见[R05完整记录](evidence/t10_a19_r05_identity_score_compare_20260910T083223Z/VERIFICATION.md)。
T10继续`IN_PROGRESS`，C1--C3不升级；唯一主阻塞是Stage2 live-C Values导出器不支持scalar C。

2026-09-10T08:32:23Z 已在任何 R05 修改、工程运行、新 science ticket 与 truth 读取前登记
[`T10_A19_R05_PROTOCOL.md`](T10_A19_R05_PROTOCOL.md)。R05 只修 R04 已证实的 Stage1 development
身份握手缺陷：payload 语义不变，继续使用精确 schema
`A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`，由 runner 与 producer 共享构造和校验；不引入
R05 schema，不改 solver、333bit、handoff、阈值、IMU、初始化或科学预算。必须先以完整 raw 的真实
请求 contract-check 和小型真实 C++/GTSAM 直接上下游链关闭工程门，随后才签发一次 fresh 完整输入
ticket；满足 Stage2/score 前置则直接完成五策略 final 和独立 truth 评价，首次预登记算法失败即封存。
当前在工程门前，R04 的 0 outer/0 call 身份失败仍是最新科学事实，C1--C3 不升级。

2026-09-10 指挥会话只读复核R04：封存栈/ABI配置与prepare结果支持原崩溃修复；当前失败是Stage1
明确的schema拒绝，0 outer/call/trial，不是数值不收敛。runner R04 schema与producer旧白名单不匹配，
prepare-only又在构造请求前返回，故未覆盖该握手。下一轮只修共享精确契约及真实请求预检，检查直接
上下游身份传递后再运行完整比较；不改solver或随任务编号反复改schema。见
[R04复审与R05完整任务prompt](T10_A19_R04_REVIEW_NEXT.md)。本复核未实施修复或新跑估计，科学状态不升级。

2026-09-10T07:28:15Z 已在任何 R04 诊断、重建、新 science ticket 与 truth 读取前登记
[`T10_A19_R04_PROTOCOL.md`](T10_A19_R04_PROTOCOL.md)。本轮只允许先保存 R03 实际二进制/库/构建身份，
取得最多60秒原栈，必要时再做一次120秒定向诊断，并仅修栈证据支持的根因；求解器阈值、lambda、333-bit
精度、IMU/初始化数学、partition/score及科学预算均不变。修复后必须以同一runner完成显式 prepare-only
正路径、非零测试计数、每final独立900秒控制和真实 mixed final->独立 evaluator 消费，才能签发一次 fresh
P1 step seed10101 ticket。科学分支仍在首次算法失败停止，所有输出 development/consumable=false，C1--C3
不升级。

R04 已按登记边界封存。原 R03 runner/库的唯一 GDB 运行在 8.16 秒内取得真实栈：
`free(0x41) -> NoiseModelFactor::error -> GraphLinearizationContentHash(factor0) -> A17Graph`；runner 的
Eigen 16-byte/SSE 与 core 的 32-byte/AVX 对齐 ABI 不一致。改由工程 CMake target 同构编译后，唯一
prepare-only 在 2.3502 秒内完整通过 raw→plan→materialize→initialize→371-factor/123-value graph 与内容身份，
明确 0 Stage1/conditional call、truth 打开 0。聚焦 refit 3/3、mixed/policy/fallback 4/4 实际执行；独立
evaluator 消费真实 C++ mixed final 导出的 score/trajectory/bias/mask/audit，确认一组 Use、一组 Suppress。

随后 11/11 startup gate 通过并消费唯一 fresh ticket。科学进程完成原 raw 初始化与 371-factor 建图，
但 Stage1 在 outer1 前以 `A18_INVALID_DEVELOPMENT_POLICY_OR_IDENTITY` 返回：runner 请求 schema
`A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`，producer allowlist 仍只接受 A18 或旧 A19 schema。
外部 2.31691 秒、exit1、无 timeout，0 outer/0 call/0 trial。此时已经进入 Stage1 算法入口，依预登记首次
算法失败停止；没有改身份重试，也未改阈值、精度或预算。Stage2、评分、五策略 final 与 truth 评价均
`NOT_RUN`，ATE/bias/risk/coverage/配对收益为 unavailable 而非零。唯一主阻塞是统一 Stage1 producer 与
runner 的 R04 development schema 身份并以该精确握手封工程门；新的 fresh ticket 需要另行授权。
[完整交付](evidence/t10_a19_r04_crash_score_compare_20260910T072815Z/VERIFICATION.md)。

2026-09-10 指挥会话对R03只读复核：崩溃根因尚未定位，优先取得调用栈并核查runner/core构建兼容性；
不据此改变handoff或最终收敛标准。发现启动预检不覆盖真实prepare正路径、final实际共用900s预算与协议
不符、既有evaluator回归不等于消费R03真实final的连贯证据；零测试过滤器也不能计入有效通过数。
正确过滤器下另有3项refit和2项final/policy测试真实通过的记录，予以保留。见
[R03复审与下一轮完整prompt](T10_A19_R03_REVIEW_NEXT.md)。本复核未新跑估计或实施修复；T10及claims不升级。

R03 已按预登记边界停止。封存 outer15 guard、12 类反例、真实 mixed graph 的 `c_s` 更新/四项 AND、五策略
development adapter 与启动预检均通过。attempt1 在任何输出/初始化前因 `char*` 地址比较 exit2，修正身份并
重新通过 8/8 startup gate；attempt2 已打开 config 与完整 raw 输入，但只写 diagnostic manifest 后在
Stage1 trace 前 SIGSEGV exit139（2.096557408 s）。无法证明算法尚未开始，因此不再重试。Stage1 为
`UNKNOWN_NOT_EXPORTED`，Stage2/score/五策略 final/evaluation truth 全部 `NOT_RUN`；不存在 eta/s/gamma、
eligible、ATE 或收益数值。唯一主阻塞为 fresh runner 在 raw initialization/graph construction 区间的
SIGSEGV 定位。[完整交付](evidence/t10_a19_r03_inexact_handoff_20260910T061142Z/VERIFICATION.md)。

2026-09-10T06:11:42Z 已在任何 R03 实现、estimator 运行和 truth 读取前登记
[`T10_A19_R03_PROTOCOL.md`](T10_A19_R03_PROTOCOL.md)。本轮唯一 amendment 为默认关闭的
`PAPER_STAGE2_INEXACT_HANDOFF_V1`：只在 Stage2 非空 live-`c_s` conditional block 的严格证书/步长/
身份 guard 下，把最后 certificate-accepted Values 原样交给既有闭式 `c_s` 更新；inner 仍未收敛，最终
joint 四项 AND 不变。新组合身份/schema 使旧 R02 cache/结果不可消费。工程门未通过前 fresh ticket
`NOT_ISSUED`；通过后仅一次 P1 step seed10101 raw fresh 自动运行（900秒），成功后五策略各至多900秒、
总科学预算5400秒。首次算法失败停止，无 retry/阈值、容差、精度或预算调整。角色仍为development，正式
validation/test/gate lock 与 C1--C3 升级均未授权。

2026-09-10 当前指挥会话已完成 **R02 outer15 静态定向复审**，未实施新的 solver 修改、未新跑完整估计。
原生重建12/12端点/常数/分支逐位匹配，J/r/delta文件逐字节匹配；独立区间重算12/12实际端点均非下降。
诊断支持微小下降被binary64位姿端点误差盖过，同时inner失败阻断了后续c更新。唯一下一步提案为Stage2非空
分段的受限inexact handoff，最终联合四项AND不变；状态为`PROPOSED_NOT_IMPLEMENTED_NOT_RUN`。
见[复审与数值证据](evidence/t10_a19_r02_outer15_review_20260910/REVIEW.md)及
[下一轮可执行任务prompt](evidence/t10_a19_r02_outer15_review_20260910/NEXT_CODEX_PROMPT.md)。
T10仍IN_PROGRESS，score/final/truth评价仍NOT_RUN，C1--C3不升级。下述R02执行记录为保留的历史事实。

本轮执行 T10-A19-R02；实施和读取 evaluation-only truth 前已登记
[完整自动评分与有限 Use/Suppress 比较协议](T10_A19_R02_PROTOCOL.md)。R02 复用身份匹配的 R01 core 与
Stage1/Stage2/scoring 工程证据，只重验 launcher/preflight 及后续实际受影响 final 接线。自动流程唯一
900 s；成功后五种预登记处理各至多一次/900 s、总计 4500 s，科学硬预算总计 5400 s。truth 只允许在所有
decisions/final outputs 冻结后由独立 evaluator 读取；本轮仍是 development，不锁 gate、不升级 C1--C3。
基础设施失败只有明确在初始化/优化前才可最多两次修复重试；算法开始后首次失败停止，不搜索参数。

R02 实际结果：第一轮 estimator-free preflight 仅因动态库符号链接比较器错误 exit1；保留失败并改为
realpath 比较后第二轮 9/9 exit0。随后唯一 `attempt1` 从原 raw 初始化，外部 `79.63632955400004 s`、
RSS `35828 KiB`、无 timeout/GT/oracle/checkpoint。Stage1 90 outer、327 calls/468 trials 收敛并立即冻结
2 段/32 候选（两段各16条、tag0-anchor1/2、`[3,6] s`、short=false）。Stage2 已完成14条 outer trace，
在 outer15 第3 conditional call 的原 lambda 上界内耗尽搜索；累计83 calls/170 trials、82接受/88拒绝，
返回 `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`。这已是算法执行后的首次失败，故没有 `attempt2`；Stage2
最终 graph/Values、score、group/eligible、五策略 decisions/final/fallback 和 evaluation truth 全部
`NOT_RUN/UNAVAILABLE`，不能写成零 eligible、零风险或轨迹无差异。唯一当前阻塞即该冻结设置下的 Stage2
conditional lambda exhaustion；本轮不提出新的泛化 solver 研究。[完整证据](evidence/t10_a19_r02_auto_score_compare_20260910T041247Z/VERIFICATION.md)。

A19 独立定向复审登记双重结论：`REVIEW_ACCEPTED_A19_FAILED_RUN_DIAGNOSIS_SCOPE` 与
`CHANGES_REQUESTED_A19_STAGE2_INTEGRATION`。来源为本轮任务 prompt
`/tmp/t10-a19-progress-review-FVSycd/NEXT_PROMPT.txt` 和真实审查依据
`/tmp/t10-a19-progress-review-FVSycd/REVIEW.md`；不是伪造的外部接受。旧 A19 ticket、失败输出和
事后摘要保持只读，T10 继续 `IN_PROGRESS`，C1--C3 不升级。

本轮执行 T10-A19-R01，实施前已登记[限定修复、身份失效、冻结工程门、唯一 fresh 运行预算与停止边界](T10_A19_R01_PROTOCOL.md)。
仅修 common-reference/candidate 全量 range metadata、真实 certified mixed-graph 工程门与阶段异常持久化。
相关 Stage2 implementation/request/diagnostic/producer identity 和缓存身份失效；新实现继续默认关闭、
development only、`consumable=false`，正式 reader 必须拒绝。全部工程门通过后只允许一个新 ticket、一个
P1 step seed10101 fresh estimator 进程树，Stage1 outer500/conditional50、Stage2 outer200、整树900秒；
首次失败或 scoring 完成即停止，无 retry/warm start/搜索。正式 cache/final/validation/test/T11/gate
和 C1--C3 仍不在本轮授权范围。

R01 实际完成 R1--R3 工程修复：Stage2 conditional graph 的 candidate/noncandidate 四类常数均全覆盖，
真实 2 candidate + 2 reference mixed fixture 经 CertifiedLm/native retract 7 outer、14 calls/22 trials、
14 accepted，并以同一最终 joint graph/Values 得到 `eta=0.9611501232376941`、`s=0.03606279586380549 m`
和逐段 `gamma=4.438131452773412e-25`；这些只属 engineering fixture。constructor-before-iterate
反例保留 Stage1 snapshot/partition/identity 与 Stage2 精确零计数，partial 反例保留 1 call/1 trial 下界并标
当前操作未知；25 项定向测试、受影响完整回归、正式 reader 拒绝和 A18 43-pair 相容性均通过。

17/17 工程门通过后签发并消耗新的 `A19_R01_PILOT_ONCE_TICKET_V1`。唯一 fresh P1 step 进程树在任何
fixture、Stage1 callback 或优化前因 launcher 未建立 `pilot/` 父目录而 exit2；external `0.10837029000686016 s`，
truth/GT 打开 0。Stage1、Stage2、scoring 及 segment/group/eligible/gamma 全部 `NOT_RUN`/`null`，不能继承
A18/A19 历史数值，也不能解释为零候选或数值失败；本轮严格无 retry。失败后只修正 launcher 的父目录
preflight 并以新 hash 记录，未再启动 estimator。T10 保持 `IN_PROGRESS`，C1--C3、正式 RQ、locked metrics、
cache/final/validation/test/T11/scheduler 均不升级或保持 `NOT_RUN`。完整证据见
[R01验证记录](evidence/t10_a19_r01_stage2_integration_20260910T024847Z/VERIFICATION.md)。

A19限定交付完成待review：登记 `REVIEW_ACCEPTED_A18_DEVELOPMENT_STAGE1_SCOPE`，来源本指挥会话及独立复审 `/tmp/t10-a18-commander-review-zl1ec9no/REVIEW.md`。预登记默认关闭A19身份与唯一900秒fresh P1 step预算；43-pair批次5.015875s、A18 C++104项、discovery39项、refit/scoring24项和相关121项回归通过，工程失败保留。
唯一运行Stage1于90outer/327calls/468trials满足四项AND；Stage2 outer1在零optimizer call前以 `NUMERIC_REFERENCE_UNSUPPORTED:UNKNOWN_FACTOR` 失败。原因是A19只为候选转换range传递证书metadata，漏掉同一条件图的共同参考raw range。Stage2 amplitude/boundary/group及eta/s/gamma均未产生；candidate/segment/eligible/unavailable因异常前未序列化写NA，score NOT_RUN，不能解释为零结果。无retry，truth/GT打开0，external62.0414s。
本轮到首次失败停止，不实施修复。唯一后续动作是补齐所有common-reference range的实际fixed-beta metadata，并增加真实certified mixed-reference/candidate Stage2 fixture；需另行授权后才可新跑。T10 IN_PROGRESS，正式validation/test/cache/final/gate/T11/scheduler NOT_RUN，A14/A15/A12负结果、A08历史15/18及C1-C3限制保留。

[完整A19交付](evidence/t10_a19_stage2_score_20260910T014316Z/VERIFICATION.md)。

### A17历史限定交付（本轮指挥接受）

登记指挥接受 `REVIEW_ACCEPTED_A16_FIXED_ENDPOINT_PRECISION_AUDIT_SCOPE`；本指挥会话与独立复审 `/tmp/t10-a16-commander-review-7d4bmy57/REVIEW.md` 为来源，仅接受A16限定审计。
[A17限定amendment/冻结工程门与预算](T10_A17_AMENDMENT_PROTOCOL.md)先于实现/测试；默认关闭的独立C++ PAPER_CERTIFIED_PAIR_REDUCTION_V1原型复用原GTSAM solve/native retract，333bit P/D与fidelity比较/转换均保守舍入。模型/初始化/priors/原generic AND stationarity、lambda预算保持，core/GTSAM/legacy未改。
工程门通过：A16固定端点/615维方向严格一致，D证书复现，独立精确有理P包络通过；23个区间/失败/转换反例、15个原策略相关GTest、原型小图2calls计数/原驻点通过；实际旧Stage2 reader拒绝原型schema。首次Python nextafter缺失、空分支CSV工程失败及实施前lambda勘误全部保留。
唯一fresh P1 step seed10101原raw初始化pilot exit0、external89.2440s/RSS35880KiB：20calls/43solves/43trials，20accepted/23rejected/0unresolved；最终尺度梯度2.2196421412e-7，原阈值1e-6/roundoff5.0389874292e-9，lambda0.010000000000000005，满足原generic AND stationarity。
这只是首conditional block合格，不是Stage1收敛；程序立即停止，无chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler，未评candidate/segment/group/eligible/unavailable为NA。证书累计求值81.2431s，尚无全链路成本/收益证据。
[完整交付](evidence/t10_a17_certified_prototype_20260910T002211Z/VERIFICATION.md)含全部P/D区间、逐factor/原始端点/实际命令/失败/库身份。原型独立策略身份与consumable=false，禁止旧缓存消费；不铺开Stage2/cache/final集成，不升级claim/准入。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。A17本地限定交付完成待review，完成后停止等待review，不自行扩展下一轮。

### A16 历史限定交付（已由本轮指挥复审接受）

登记指挥接受 `REVIEW_ACCEPTED_A15_TERMINAL_NUMERIC_DIAGNOSTIC_SCOPE`，来源为本指挥会话，非外部REVIEW.md、非estimator成功或validation准入。
[A16预登记协议](T10_A16_PROTOCOL.md)先于实现/计算；唯一call17 trial1恢复初始/末态graph、Values、371factor/123keys/615delta及两端1886native残差严格一致，原生retract一次后封存binary64端点。
零optimizer iterate/新estimator进程；固定50/100位参考与有向区间表明该端点对下降 `+2.59734275148281994e-13`；两档差`3.20911e-48`，独立区间半宽`9.16821e-46 / 7.22877e-96`满足预登记门，分支核对通过。
原生残差求值/白化将下降高估`4.00953e-13`，解释原白化恒等式与A15稳定GN差异的约79.26%；仍剩`D_ref-GN=+1.04901e-13`，未进一步区分端点舍入/非正交、J/r误差与GN余项。不宣称全域导数正确或solver会收敛。
[完整证据](evidence/t10_a16_endpoint_precision_20260909T160252Z/VERIFICATION.md)含公式/独立误差依据、逐factor分解、原始端点、命令/失败及实际库身份；core/GTSAM/模型/生产策略/合同不改，无truth/GT读取。
唯一[solver amendment草案](T10_A16_SOLVER_AMENDMENT_DRAFT.md)为paired reduction与误差证书/resolution判据，明确接受/失败/身份失效/回归；PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED，本轮不实施或追加实验。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评计数NA。
A16本地限定交付完成待审，完成本次审计后停止。

### A15 历史交付（限定范围已由指挥接受）

指挥接受 `REVIEW_ACCEPTED_A14_OPT_IN_IMU_MODEL_AND_BOUNDED_PILOT_SCOPE`，来源为本指挥会话，非外部 REVIEW.md，非 estimator 成功或 validation 准入。
[A15预登记协议](T10_A15_PROTOCOL.md)先于实现/捕获。默认关闭被动TRYDELTA模式无额外solve/InspectFirstLinkedLmTry/extension；只捕获首block并阻止chain。
唯一P1 step seed10101新模型捕获exit1，external1.75785s/RSS31160KiB；初始graph/Values/common严格复现A14，末态17calls/16accepted/42trials及E/五类梯度/lambda复现，仍不驻点。
全部42个trial 615维完整；静态零optimizer恢复371factor、16accepted native retract及内容身份通过。call17的19trials为18次线性差负/1次零，均未计算model fidelity。
指定首/末拒绝方向的稳定线性下降为+1.54833e-13/+5.80613e-15，而linked总目标相减为-1.36424e-12/-3.18323e-12；两正值仍低于原5.16324e-13 resolution门。
三条实际完整方向及末态最大梯度坐标在预登记连续步长判据下支持导数一致性；13356个factor残差FD中124失败全部保留，不能写全步长通过，A08历史15/18限制不消除。
结论支持线性化差值数值分辨力与linked分支限制；非线性真实下降/残差求值误差与非线性余项分离仍证据不足，不宣称改算术即可收敛。
唯一[最小后续提案](T10_A15_NEXT_ACTION_PROPOSAL.md)为同call17 trial1零iterate非线性残差精度审计，本轮NOT_RUN，无solver修复或数值语义amendment实施。
[A15完整证据](evidence/t10_a15_terminal_numeric_20260909T152633Z/VERIFICATION.md)保留编译失败、中止重叠构建、唯一pilot失败、完整静态结果和source/raw/config/actual runtime身份；truth/GT未读，GTSAM不变。
T10 IN_PROGRESS；A14失败、A12负结果、A08历史限制及C1–C3原状态保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评candidate/segment/group/eligible/unavailable计数NA。
A15限定诊断本地完成待审；完成后停止，不自行执行下一轮。

### A14 历史交付（限定范围已由本轮指挥接受）


指挥接受 `REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE`；A13唯一方案已技术接受，仅显式opt-in development。
[A14 amendment/冻结判据](T10_A14_AMENDMENT_PROTOCOL.md)先于实施。新PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1显式K0，live bias、其他噪声/RW/Qi、交叉项、初始化/priors/solver保持；缺省legacy I6。
实际模型身份接入common、discovery/support、Stage2 producer/cache/request、final及直接core消费门，跨模型拒绝。
A12四SHA精确复现；1680静态检查、127 C++工程回归及Python mock-runner合同通过，实际库身份闭合。
唯一P1 step seed10101 pilot为exit1：outer1第17调用lambda搜索耗尽；16接受/26拒绝trial，Gmax=1.20549e-5未达原驻点。
chain/Stage2/score/cache均未到；candidate/segment/group/eligible/unavailable计数NA，不能把失败空partition解释成零候选或零eligible。
external wall0.688s/RSS27396KiB；无retry，truth/GT未读，raw/config/GTSAM不变。
[A14完整交付](evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md)保留源码/配置/输入/库/命令/负结果；T10 IN_PROGRESS、A14本地限定交付完成待审。
A12负结果、A08历史15/18/UNKNOWN及C1–C3限制保留；validation/test/T11/scheduler/gate/final NOT_RUN；不默认迁移或升级claim。
本轮停止，不自行执行后续末态诊断。

## T02 review 问题登记

| ID | 复审事实 | 验收影响 | 当前状态 |
|---|---|---|---|
| T02-R01 | loader 在裁剪后的 view 内从零分配 `source_obs_index`，使同一源观测跨 `bag.start` 裁剪改变 ID，并可能复用其他观测的 ID | 改为 `content-fnv1a64 recording_id + 完整 UWB stream message ordinal + message 内 range ordinal`；original 627 条、SFUISE 820 条裁剪观测逐条回查 bag 且跨窗口 ID 一致 | `REVIEW_ACCEPTED` |
| T02-R02 | 非空 `fixed_beta_by_link` 未校验键语法或实际使用 link 的完整覆盖，未知键和部分覆盖均可成功 | 规范键在配置加载时校验，paper plan 后校验所有实际使用 link；malformed 与 partial 实跑均非零退出，空 map 明示 development-only 缺标定 | `REVIEW_ACCEPTED` |
| T02-R03 | SFUISE 分组将多条源消息的 range 统一写成组首时间/tag | ledger 从每条源消息写入 range 级原始时间/tag；paper runner 对非零 SFUISE group window 显式失败，legacy 分组语义未改 | `REVIEW_ACCEPTED` |
| T02-R04 | `lm_max_iter=0` 仍返回 `OK`，且 runner 未审计目标/Values 有限性或记录迭代与终止信息 | 非法参数前置拒绝；每次 LM 后检查目标/导航 Values 有限性并记录迭代、内迭代、lambda、目标变化和终止原因；1 次上限 run 为 FAILED 且无有效估计导出 | `REVIEW_ACCEPTED` |
| T02-R05 | `calib_td=false` 时非零 `td_init` 被 paper runner 忽略 | T02 paper runner 只接受精确 `td_init=0 s`，非零实跑非零退出；支持边界写入 capability/config effective | `REVIEW_ACCEPTED` |
| T02-R06 | 非有限目标失败时浮点诊断直接流式写入 `inf`，导致 `run_status.json` 不能被 strict JSON 解析 | 所有浮点诊断经统一有限性编码，NaN/±Inf 写 `null`；复现仍 exit 1、保留原失败原因且无有效估计输出 | `REVIEW_ACCEPTED` |
| T02-R07 | loader checker 的 `math.isclose` 使用默认相对容差，在 epoch 数值尺度上会接受约秒级时间改动 | 显式 `rel_tol=0`，绝对容差按 double 表示精度计算；真实 SFUISE ledger `+0.5 s` 篡改检查 exit 1，original/SFUISE 正常跨窗口检查仍通过 | `REVIEW_ACCEPTED` |

首轮复审材料：`/tmp/t02_review_ako2r7kz`；第二轮复审材料：
`/tmp/t02_rereview_ov0laerb`；最终独立复核来源：`/tmp/t02_final_review_95j7sbjy`，必要证据已归档至
[`evidence/t02_final_review_20260906T083514Z/`](evidence/t02_final_review_20260906T083514Z/)。以上均是
T02 接口/验收缺陷修复或复核，不改变冻结方法、指标或证据边界，因此没有新增 amendment。

## T03 review 问题登记

| ID | 复审事实 | 验收要求 | 当前状态 |
|---|---|---|---|
| T03-R01 | `np.linalg.norm` 对极小非零 nuisance 列下溢为零、对极大列上溢，改变 `col(F)` 或破坏 strict JSON | 以原始元素判断精确零列，稳定归一化 `1e-200`/`1e200` 列；所有非有限中间量显式 `NUMERICAL_FAILURE` | `REVIEW_ACCEPTED` |
| T03-R02 | `N-R` PSD 审计以 `N-R` 自身谱作尺度并带固定绝对地板，不能正确解释保存的正交矩阵舍入反例 | 使用由浮点运算误差和 `N/R` 尺度推导的界，接受舍入内反例且拒绝超过该界的错误 | `REVIEW_ACCEPTED` |
| T03-R03 | `F=[1,1,1]^T,G=1e10F` 的投影舍入残差被当作真实信息，返回 `OK` 和有限 `s` | 将投影误差可分辨界并入 `R` 数值秩判据；不可辨识/不可判定方向必须为无限 `s`，同时保留真实 `N=R=1e-4` 弱信息 | `REVIEW_ACCEPTED` |
| T03-R04 | fixture 单测逐对象精确比较，默认 BLAS 通过但 Sandybridge 失败 | 状态、维度、案例身份和结构严格比较，浮点字段使用预声明 `atol/rtol`；同环境字节复现与跨环境数值正确性分开验证 | `REVIEW_ACCEPTED` |

复核材料：`/tmp/t03_review_5co105jp`；最终独立复核来源：`/tmp/t03_final_review_va5vu4tl`，精简包归档至
[`evidence/t03_final_review_20260906T110914Z/`](evidence/t03_final_review_20260906T110914Z/)。以上为冻结 SVD
projector 定义的稳定实现、舍入审计和测试语义修复，不改变 `E=G-P_FG`、`R=E^T E`、`eta` 或 `s`
的数学定义，无新增 amendment。U01–U05 NumPy 侧为 `PASS`；在 T03 收口时 U07 仅 Python
projector/Schur 侧为 `PASS`、C++ sparse 尚未运行；其后续状态见下方 T05 Gate A/Gate B 记录。

## T04 review 问题登记

独立复核来源 `/tmp/t04_review_ljqmxpqd` 的精简输入已归档至
[`evidence/t04_20260906T121005Z/review_input/`](evidence/t04_20260906T121005Z/review_input/)。以下修复不改变
`h+beta+c-z` 残差、`c>=0` 或最终无 L1/TV 联合图语义，因此没有新增 amendment。

| ID | 复审事实 | 修复与验收 | 当前状态 |
|---|---|---|---|
| T04-R01 | 旧停止条件只检查目标变化、外层步长和 `c` KKT；条件 LM 停滞后，闭式 `c` 更新可能重新破坏 navigation 驻点 | 在闭式更新后的同一最终 graph/Values 线性化，对全部自由 `X/V/B` 局部坐标检查物理尺度化 `J^T r`；第 13 轮旧三项通过但 gradient `1.4538511614592409e-6 > 1e-6` 被拒绝，第 14 轮 `8.1725042155866845e-7` 才收敛；另有一轮上限回归 | `REVIEW_ACCEPTED` |
| T04-R02 | segment ID 含逗号/双引号时破坏 segments/factor metadata/residual CSV 列结构 | 三个 CSV 的所有 segment ID 均按 RFC 4180 风格转义；标准库 CSV parser 验证逗号、双引号、换行的列数、ID round-trip 与三表 obs 关联 | `REVIEW_ACCEPTED` |
| T04-R03 | manifest 未知字段含换行时，错误原因直接写入 JSON 控制字符导致状态不可解析 | JSON encoder 覆盖 `\b/\f/\n/\r/\t` 和全部 U+0000–001F；runner 反例 exit 1，reason 保真、debug 标签和 `INVALID_ORACLE_MANIFEST` 正确，无有效估计导出 | `REVIEW_ACCEPTED` |

## T04 实现与复审边界

- `MakeSegmentUwbFactor` 的残差为 `h(X)+beta+c_s-z`，动态段使用 `C(segment_ordinal)`，不与 `B(k)`
  或 `Z(m)` 混用。oracle manifest 严格限制为 schema/debug 标签、segment ID、规范 link，以及互斥的
  obs IDs/闭区间；幅值、GT、pose、initialization、未知字段、重复/跨 link/无效归属均拒绝。
- `SegmentRefitResult` 拥有最终无 L1/TV 联合 graph、含 `C(s)` 的 Values、完整 factor metadata、segment
  estimate 和迭代 trace。条件导航图只把 `beta+c` 作为临时常量，成功 trajectory/bias/amplitude/residual
  全部从联合结果对象导出；没有 corrected pseudo-range 或 amplitude prior，也未调用旧 GNC/rejection。
- 数值参数在 T04 debug 路径锁定为：boundary `1e-9 m`、relative objective `1e-8`、scaled step `1e-6`、
  projected gradient `1e-8 objective/m`、最终联合 navigation stationarity `1e-6 objective/normalized-coordinate`、
  gradient roundoff safety factor `8`、默认最多 20 个外层迭代；条件 LM 复用 `lm_max_iter/rel/abs`。
  驻点在闭式 `c` 更新后的同一最终 graph/Values 上对全部自由 `X/V/B` 检查：pose rotation/translation、
  velocity、accelerometer bias、gyroscope bias 分别按 `1 rad`、`1 m`、`1 m/s`、`1 m/s^2`、`1 rad/s`
  归一；舍入余量使用 factor-wise 绝对梯度和 binary64 `gamma_n`。只有相对目标、尺度步长、`c` KKT
  和 navigation stationarity 同时通过才收敛，目标仅允许 binary64 舍入尺度上升。
- 实际证据：定向 10/10、package 88 tests/0 failures；4 秒 `sim_circle` debug run 在 14 次外层后收敛，
  第 13 轮虽 objective/step/KKT 均通过但 navigation gradient `1.4538511614592409e-6` 未通过，第 14 轮
  降至 `8.1725042155866845e-7` 后才成功；这验证条件 LM 停滞或 `c` KKT 不能单独触发联合成功。
  8 条同 link 观测共享 `c0`；一轮上限 run 为 `MAX_REFIT_ITERATIONS` 且不导出有效 trajectory/amplitude/
  residual。带逗号/双引号/换行的 segment ID 经标准 CSV parser 三表 round-trip；未知 manifest 字段带
  换行时 exit 1、strict JSON 保真且不导出估计。T02 all-range 与 legacy SFUISE trajectory 均同旧证据逐字节一致。
- 该输入没有独立 fixed `beta`，状态保持 `MISSING_CALIBRATION_DEVELOPMENT_ONLY`。短段阈值只记录 debug
  count/duration，不构成 T05 gate。独立复核同时原样保留 `-0.4` 幅值在 20 轮后的
  `MAX_REFIT_ITERATIONS`、`c=0`、navigation gradient `7.928690804792637e-06`；`-0.2/-0.1`
  收敛到约束边界；嵌入 NUL 的非法 YAML reason 经 `what()` 截断，但失败状态、strict JSON 与无估计
  导出仍正确。没有调整 solver、阈值或测试制造成功。T04 最终复核见
  [收口证据](evidence/t04_final_review_20260906T143023Z/VERIFICATION.md)。

## T05 Gate A/Gate B 实现边界

- Gate A 新增独立 sparse 平方根最小二乘核心及 10 个 `sparse_rank_cases.json`，原 T03
  `golden_cases.json` 未修改。冻结 rank 仍按 scaled-`F` SVD 阈值定义；Eigen QR rank 只作诊断。
  指挥反例的 frozen rank 1/`R≈1` 与 naive QR rank 2/`R=0` 分歧时，production 返回
  `SPARSE_RANK_UNCERTAIN` 且不导出有限 `s`。条件数/roundoff 公式保持
  `PENDING_NUMERICAL_PROPOSAL`，没有 amendment。
- Gate B 在 T04 debiased 同一 graph/Values 上使用 GTSAM 4.2 实际已白化 sparse augmented
  Jacobian；RHS 独立，`G0` 排除全部 candidate，每组 factor 恰好加回一次，nuisance 为图中全部非本组
  amplitude 自由 key。closed-interval group 按传递重叠且端点相等即重叠，不加 epsilon。
- 最终真实短输入 Stage 2 收敛；1 group 的 `F=184x120, nnz=2027`，
  `eta=0.81570732433217819`、`s=0.088141539221235826 m`、
  `gamma=0.042596852206918272`，但仅是 `T05_ORACLE_SUPPORT_SCORE_DEBUG_ONLY`，且缺独立 fixed
  `beta`。short/boundary 只使 score 不可导出，不倒写 Stage 2。
- T05-R01–R05 已由 `/tmp/t05_rereview_xu53rhk8` 独立复审接受。独立定向测试为 6/6 与 19/19；
  100 个 seeded case 中 85 个支持域结果匹配、15 个明确 unavailable。该复核未独立重跑 package 120、
  T03 双 BLAS 或 T04/T02/legacy 全回归，这些未运行项按原样保留。T05 只在 oracle-debug/已测试 sparse
  支持域内关闭，不覆盖 T06、gate/fallback/final covariance 或正式 RQ。

## T05 review 问题登记

独立复核来源 `/tmp/t05_review_8woic5bf` 已归档到
[`evidence/t05_review_fix_20260906T160000Z/review_input/`](evidence/t05_review_fix_20260906T160000Z/review_input/)。
以下为 correctness/reproducibility 修复，不改变冻结 SVD rank、`E/R/eta/s` 定义，因此没有新增 amendment。

| ID | 复审事实 | 修复与验收 | 当前状态 |
|---|---|---|---|
| T05-R01 | QR pivot ambiguity band 不能证明冻结 SVD rank；triangular/重复列反例会导出错误有效分数 | 移除经验 band；以 active `R11` triangular solves 给出 inverse-norm 上界，同时用完整 scaled-F 非零列数量约束 `sigma_max`，重复列数量进入冻结阈值；不能证明 full-rank 支持域即 `SPARSE_RANK_UNCERTAIN`。指挥、triangular、多重复列均不导出分数，良态与少重复列支持域通过 | `REVIEW_ACCEPTED` |
| T05-R02 | `max(1,spectrum)` 形成单位信息地板，N-R 含固定绝对地板，归一化正交残差与未归一化容差混用 | 去除 N/R unit floor；N rank/PD、R rank/PD/最终状态分别共用实际谱尺度阈值；N-R 只用随 N/R 和运算次数缩放的界；正交容差同样归一化。`N=R=5e-11` 与 weak-R 均为有限有效分数，跨尺度/非有限参数回归通过 | `REVIEW_ACCEPTED` |
| T05-R03 | 错误 factor index/keys、重复 obs 和未知 factor type 可被评分 | 评分前完整校验 factor index、实际 keys、类型白名单、graph/Values keys、plan/support/refit 身份、obs 唯一归属、candidate 完整覆盖与逐组 factor mask；全部独立破坏案例明确抛错且无分数 | `REVIEW_ACCEPTED` |
| T05-R04 | 旧 ID 仅含 graph/Values 数量与 scalar error，实际不同 Jacobian 可碰撞 | 规范序列化实际 factor mask、ordering/dimensions、factor-row/key-column mapping、白化 F/G/RHS 后计算 SHA-256；同输入稳定，F/RHS 改变时 ID 改变，被排除的其他组变化不影响当前组 ID | `REVIEW_ACCEPTED` |
| T05-R05 | short/boundary 仍把有限 eta/s 写入主 decision 字段 | `score_availability/s_semantics` 区分 finite、rank-deficient `+inf`、not-applicable 和 numerical unavailable；short/boundary 主 eta/s/inf 字段为空，原始数学诊断仅在 `debug_*`/audit；Stage 2 仍为 `CONVERGED` 且 estimate exported | `REVIEW_ACCEPTED` |

最终独立复核来源 `/tmp/t05_rereview_xu53rhk8` 已完整归档至
[`evidence/t05_final_review_20260907T022011Z/`](evidence/t05_final_review_20260907T022011Z/)。复核前 12 个
源码/文档和 4 个 build product 哈希全部匹配；93 个非 ELF 复核文件、全部 unavailable case、命令、日志、
probe 源码及 12 个被审查文件均有 SHA-256 manifest。可重建 ELF 仅记录来源、大小、哈希，没有入库。
T05 状态为 `DONE/REVIEW_ACCEPTED`，但仅限上述 oracle-debug 已测试支持域。

## T06 Stage 1 与自动 partition 验收边界

- `AutomaticSupportProvider` 复用 C++/GTSAM IE 后端，交替执行固定 `b` 的 navigation LM 与逐 link/gap
  chain 非负 L1/TV 子问题；没有 Python estimator、平方 L2、softplus 或旧 GNC 替代。chain ADMM 在声明
  primal/dual/KKT 容差内求解，scaled dual `q` 对应物理 TV 次梯度 `p=rho*q`，最终审计原始可行 `u` 和
  原始 L1/TV 目标。目标超过 binary64 舍入容许上升、LM/分解/非有限/ADMM/外层不收敛均显式失败。
- Stage 1 外层步长用 GTSAM `Values::localCoordinates` 审计全部导航 `X/V/B`，再与逐观测
  `max_i |u_i-u_i^prev|/observation_bias_scale_m` 取最大；只有组合尺度步长、原始目标、chain 容差和
  最终导航驻点全部通过才收敛。配置与 trace 分别记录阈值、导航/bias/组合步长。
- 自动 provider 与 oracle provider 分离输入，但共同产出 `SupportPartition`，随后复用 T04 refit 和 T05
  scoring。链按 `(link,time,obs_id)` 排序，`Delta t > T_gap` 断开且等号相连；inactive、change point、short、
  boundary 均保留。A01 用不可变 discovery snapshot 做单遍 merge，保存加权代表值、哈希父子 ID、obs 唯一
  归属与 partition hash；加权代表幅值用按最大权重缩放的有限累加，累计量、分母和结果任一非有限即失败。
  snapshot/partition/父子 ID 使用 SHA-256，并纳入 source/config/input plan/calibration/solver 上下文；Stage 2
  后不再改变 partition。
- 空 partition 仍在无 `C(s)`/L1/TV 的共享 raw graph 上运行 Stage 2 LM；每轮 trace 审计原始目标、组合尺度
  步长和同一最终 raw graph/Values 的导航驻点，全部适用条件通过后才返回 `NO_CANDIDATES` 并导出
  同一 graph/Values 的 trajectory/bias/residual/factor metadata，score 为空且不适用；Stage 2 失败则失败。
  未实现 T08 fallback。
- 本地测试覆盖解析/独立 SciPy epigraph reference、无 bias/常值/ramp/gap/inactive/非均匀权重与溢出反例、单点、零
  正则、多个 `rho_scale`、A01 三段链/等号/端点/归属、非收敛及无候选。真实 4 秒自动 run 不读 oracle/GT，
  发现 17 段（short 13、boundary 0），Stage 2 收敛；3 个 overlap group 均含 short，故主评分 unavailable/
  not applicable，run 按设计 exit 1 并保留 Stage 2。缺独立 fixed beta，明确为 development-only。
- 自动工程小图从无 oracle/GT 的逐观测输入产生非空 eligible candidate，经共享 T04 refit 进入 T05 有效
  score。Stage 1 ADMM 失败路径仍保留 manifest、effective config、capability、诊断和所有 discovery trace。
- 本地兼容验证覆盖 T04 oracle、T05 normal/short、T02 all-range、T03 双 BLAS、158-test package 与
  legacy/default-off。独立复审实际重跑 discovery 14/14、refit/scoring 23/23、recoverability 6/6、config
  8/8、5 cases x 3 rho chain reference 及 14/14 配置拒绝探针；没有独立重跑 full package/build、T03 双
  BLAS 或 T04/T05/T02/legacy，只复核开发者证据。T06 仅在 reviewed fixed-calibration development
  engineering scope 内为 `DONE/REVIEW_ACCEPTED`；科学参数全部保持 `PENDING_VALIDATION`。T06 证据自身
  不含 T08 gate/fallback/final covariance 或正式 RQ，C1–C3 不标 `SUPPORTED`；后续 T08 本地状态另见下文。
- 独立复审保留旧默认空 fixture 的 20 轮 `MAX_REFIT_ITERATIONS`，不得改写成成功；宽松 conditional-LM
  fixture 在 4 轮达到最终驻点。真实短输入的 3 个 group 仍全部 score unavailable，有效 score 只来自
  无 oracle/GT 的 synthetic engineering GTSAM fixture，不能冒充真实评分成功。

## T06 review 问题登记

独立复核来源 `/tmp/t05_closeout_t06_review_qh5h6dhz` 的非 ELF 内容已归档至
[`evidence/t06_review_fix_20260907T035905Z/review_input/`](evidence/t06_review_fix_20260907T035905Z/review_input/)，
可重建 probe ELF 只记录来源、大小和 SHA-256。以下均为冻结目标的 correctness/reproducibility 修复，
没有修改数学定义或登记 amendment。

最终独立复核来源 `/tmp/t06_rereview_3d02_tsu` 已完整归档至
[`evidence/t06_final_review_20260907T043727Z/`](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md)。
复核决定为 `REVIEW_ACCEPTED_T06_AUTOMATIC_DEVELOPMENT_ENGINEERING_SCOPE`；193 个非 ELF 文件逐文件
SHA-256 一致，可重建 ELF 未归档。接受范围不包含正式 RQ、性能或通用收敛证明。

| ID | 独立复核事实 | 修复与本地验收 | 当前状态 |
|---|---|---|---|
| T06-R01 | 空 partition 的 raw Stage 2 可无 trace、无最终驻点检查即返回 `CONVERGED`；反例梯度 `7.450197073932685e-6 > 1e-6` | 空路径逐轮记录目标/步长/驻点及适用标志，只有全部适用条件通过才导出；一轮上限反例为 `MAX_REFIT_ITERATIONS`、exit 1 且无有效估计 | `REVIEW_ACCEPTED` |
| T06-R02 | automatic 可关闭 scoring 后返回 OK；科学参数缺失走默认；空 `oracle_support` 字段仍被接受 | automatic 强制 `score_recoverability=true` 且显式给出八个 discovery 科学参数和两个步长数值参数；任何 oracle 字段均拒绝；完整 config preflight 在创建 run 目录前完成 | `REVIEW_ACCEPTED` |
| T06-R03 | Stage 1 外层停止遗漏已批准的导航与逐观测 bias 组合尺度步长 | 以 GTSAM local coordinates 审计 `X/V/B`，与逐观测 bias 尺度步长取最大；effective config 与 outer trace 导出三项步长及阈值，停止条件纳入 `step_ok` | `REVIEW_ACCEPTED` |
| T06-R04 | A01 直接累计可溢出；两个 `1e308` 权重、幅值 `.4` 得到代表值 `0` | 按最大权重稳定缩放并逐项检查有限性；溢出反例返回 `.4`，另有非均匀权重解析值测试 | `REVIEW_ACCEPTED` |
| T06-R05 | snapshot/partition 缺计划中的 SHA-256/context；Stage 1 失败缺 manifest/effective config/capability | snapshot/partition/父子 ID 与 source/config/plan/calibration/solver context 使用 SHA-256；Stage 1 失败实跑保留 manifest、effective config、capability、诊断、snapshot/partition 和两类 trace | `REVIEW_ACCEPTED` |

## T07 review 问题登记

以下只修复本次 T07 复审指出的隔离、顺序验证和 checker 证据缺口；未改变 T06 科学参数、方法合同、
实验合同或冻结材料。本地修复证据位于
[`evidence/t07_review_fix_20260907T062126Z/`](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md)，
最终独立复审材料与收口检查归档于
[`evidence/t07_final_review_20260907T070102Z/`](evidence/t07_final_review_20260907T070102Z/VERIFICATION.md)。
其中旧 `final_checks.py` 只保留为 point-in-time checker；后续 archive-integrity 权威入口为不依赖
`/tmp`、使用固定历史目录列表和相对路径 manifest 的
[`t07_closeout_fix_20260907T072733Z/archive_integrity_check.py`](evidence/t07_closeout_fix_20260907T072733Z/archive_integrity_check.py)。

| ID | 复审事实 | 修复与本地验收 | 当前状态 |
|---|---|---|---|
| T07-R01 | cache/truth 仅按 lexical absolute path 比较，symlink alias 可使二者落到同一物理目录，失败还可能留下半成品 | 对最近存在祖先与创建后的 root/final dir 做 canonical 物理关系检查，拒绝同目录、alias 和双向包含；双 root staging、精确文件集、rename commit 与异常 cleanup 保留 no-overwrite。exact-same、symlink、overlap、sibling、写失败清理均通过 | `REVIEW_ACCEPTED` |
| T07-R02 | reader 只拒绝被拆开的 group；交换完整 group 并重算 payload hash/cache ID 可通过 | writer/reader 统一强制 UWB sensor time 非递减、source-message ordinal 严增 tie-break、group 唯一连续、range ordinal 顺序、source-observation ordinal 严增及 inherited obs ID；IMU 强制 row ordinal 与时间顺序。完整交换 group/IMU 并重算 hash/cache ID 的反例均拒绝，合法 cache/prefix/identity/no-conversion 仍通过 | `REVIEW_ACCEPTED` |
| T07-R03 | isolation checker 仅凭 Stage 1 前写出的 input manifest 判断进入 Stage 1 | checker 核对实际 cache/recording/interface/no-conversion，执行期轮询 base/recipe/truth 三路径不可达，finally 按 identity 恢复；以 50 行 discovery trace 和匹配 diagnostics 证明 Stage 1，保留 runner exit 1、`MAX_OUTER_ITERATIONS`、Stage 2/score `NOT_RUN` 以及原始 stdout/stderr/argv/cwd/exit | `REVIEW_ACCEPTED` |

## T08 冻结决策与最终联合推断本地验收边界

- 新增来源中立的 `FinalInferenceEngine`/`InferenceResult`。结果对象只持有一套实际
  `final_graph/final_values`、冻结 masks、decision/final score sets、状态、timing、factor audit 与实际全图
  covariance；trajectory、IMU/static/segment bias、post-fit factor residual 和 covariance exporter 只接收该对象。
  T08 路径不调用 legacy `Optimize`、GNC 或 rejection。
- gate 严格执行 `eta>=tau_eta && s<=tau_s && max(gamma)<=tau_gamma`，等号通过；group 原子
  Use/Suppress。short、boundary/KKT、支持不足、rank、numerical、invalid score 和三个阈值失败均有稳定原因码；
  zero candidate/eligible/accepted 与 suppressed/inapplicable final score 均保留显式状态行。
- final graph 重建同时接收 frozen full candidate set 与 accepted ordinal set，明确区分 accepted、suppressed
  和 noncandidate，避免 accepted-only partition 把 suppressed candidate 重新加入。`obs_id` 审计要求 accepted
  raw factor 一次且绑定 live `C`、suppressed 零次、noncandidate 一次；suppressed `C` 不存在且 corrected
  pseudo-range 为零。
- final refit 后按同一 reference-plus-group 定义重评分，decision/final linearization ID 分离；冻结 Suppress
  不会被 final score 重新接纳。rank/acceptance/求解/graph-values 审计失败只会从 frozen raw/reference graph
  触发一次 `ALL_CANDIDATES_SUPPRESSED` fallback；成功为 `FALLBACK_OK` 并保留 recovery 原因，再失败为
  `ESTIMATION_FAILED` 且不保留可导出的初值/半成品图。
- covariance 由实际 `final_graph/final_values` 的 GTSAM CHOLESKY marginals 计算并逐 key 记录 role/coordinates；
  与 reference-group `R_c` 明确分名。求解失败只返回 `UNAVAILABLE` 原因与空 block，不写 `-1/0` 占位。
- gate 配置在 T08 enabled 时要求三个有限显式值和精确
  `T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION` provenance。工程 runner 的 `0/1000000 m/1000000` 只为
  管线验收，不由 smoke 调参、不写成 T10 lock；正式 gate 仍 `PENDING_VALIDATION`。
- 本地实际验证：最终 build exit 0；全部 test targets 重建后 CTest 17/17；T08 定向 12/12；真实
  oracle-development runner exit 0/`OK`，74 factors/25 Values、8 accepted candidate/56 noncandidate、0 pseudo、
  25 covariance blocks；6 JSON/13 CSV strict parse。forced recovery→fallback success、forced fallback failure、
  covariance unavailable 与无 oracle/GT automatic engineering fixture 均在定向测试实际通过。证据见
  [`evidence/t08_20260907T083437Z/`](evidence/t08_20260907T083437Z/VERIFICATION.md)。
- 初次实现状态为 `LOCAL_IMPLEMENTATION_PASS_AWAITING_INDEPENDENT_REVIEW`；本轮 review-fix 后状态见下。
  缺独立 fixed beta；未做性能/RQ/
  locked gate/held-out test。T07 step/ramp 的既有 Stage 1 failures 原样保留且 T08 不可达；无 amendment。

### T08 review-fix R01–R03

以下均为已冻结 Stage 4 语义的实现/证据修复，没有修改 gate 公式、reference-plus-group 定义或 fallback 政策，
无需 amendment：

| ID | 最小反例结论 | 修复与本地验收 | 当前状态 |
|---|---|---|---|
| T08-R01 | 原 `SetInapplicableFinalScores` 在 fallback 后覆盖已经计算的 recovery final re-score；旧 `refit_iterations.csv` 实为 Stage 2，未保存 Stage 4 recovery/fallback/final trace | 独立保存 Stage 2、recovery attempt、fallback 和 selected final trace/status；保留 recovery group/segment score、失败原因和原 linearization ID，fallback-final 无 `C` 时另写显式 `NOT_APPLICABLE`，不导出失败尝试为有效估计。实际 final re-score 失败分别得到一次 fallback `FALLBACK_OK` 和一次 fallback 后 `ESTIMATION_FAILED` | `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| T08-R02 | 前轮正常/Stage-2 失败计时已修复，但独立复审实跑非法 oracle manifest：exit 1、外部 `9.480520605 s`、`ORACLE_SUPPORT_LOAD`，其已创建目录的 `run_status` 仍缺 elapsed semantics/value 和 Stage-4 字段 | 通用 `std::exception` 与 `LmFailure` 出口统一补写 runner wall 语义和 Stage 1–4 字段；已完成 stage 保留数值，未执行项为 `null`。最终独立复审重放同类非法 manifest：预期 exit 1，原 reason/stage/status 不变，外部/runner `9.453941426/9.441450969 s`，差 `0.012490457 s`，无有效估计产物；runner contract 覆盖 normal/repeat/Stage-2 failure/invalid manifest/LmFailure | `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE` |
| T08-R03 | 原 identity 仅绑定结构计数、总 error 和 masks；不同 Values 或不同 factor 数值可在相同结构/总 error 下碰撞 | 新 identity 用规范 binary64 序列绑定实际 Values、按序 factor type/keys/error/最终白化线性化 Jacobian，并绑定 input/config/plan/support 等必要上下文；run ID、score linearization ID 与 inference content ID 明确分离。相同输入重复稳定，`+1/-1` Values、等误差不同 prior 均可区分，seal 后篡改被 exporter 拒绝 | `REVIEW_ACCEPTED_TESTED_CONTENT_IDENTITY_SCOPE` |

修复后定向 18/18、完整 CTest 17/17、runner contract 和 `git diff --check` 均 exit 0；8 个 JSON、21 个
CSV strict parse。两个独立 group 的完整 engine case 保持一组 Use/一组 Suppress，accepted live `C`、suppressed
零 factor/key、noncandidate 一次且无 re-admission；无候选、零 eligible、零 accepted、covariance unavailable
语义保持显式。完整命令/退出码/stdout/stderr、反例、源码/二进制哈希和自包含归档见
[`evidence/t08_review_fix_20260907T092037Z/`](evidence/t08_review_fix_20260907T092037Z/VERIFICATION.md)。
T08 最终为 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；R01/R03 仅在上述限定 scope 内
`REVIEW_ACCEPTED`，R02 为 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`，仍不是正式 RQ 证据。

本轮独立复审输入 `/tmp/t08-independent-rereview-pl3x59g4/` 已在修改前完整归档；其 checker 首次
schema 误判 exit 1 明确属于复审工具调用错误，没有产品测试执行。R02 最终定向修复重新构建相关目标，
runner contract 覆盖 normal/repeat/Stage-2 failure/invalid manifest/`LmFailure`，并保存 normal、Stage-2
failure、invalid manifest 和 `LmFailure` 的独立 run。证据见
[`evidence/t08_r02_review_fix_20260907T104154Z/`](evidence/t08_r02_review_fix_20260907T104154Z/VERIFICATION.md)。
最终独立复审实际重跑 focused GTest 18/18、runner contract 和非法 manifest 反例；迁移归档 checker、
7/7 当前源码/二进制、4/4 R01/R03 源码不变、2/2 冻结材料 hash 与 `git diff --check` 均通过。
package build、完整 CTest 17/17、T07 step/ramp、正式 RQ、held-out validation 和性能/RSS 未由本轮重跑。
最终收口证据见
[`evidence/t08_final_review_20260907T121510Z/`](evidence/t08_final_review_20260907T121510Z/VERIFICATION.md)。

## T09 统一基线、缓存、批量执行与评估本地验收边界

- canonical registry 固定 12 个合同模式，并区分 baseline trajectory、Stage 1 trajectory、automatic Stage 2
  trajectory、cache producer/diagnostic、final trajectory 和 evaluation reference。独立 `run_unit_id/cell_id/run_id`
  不把方法 cell、threshold point、segment 或重试计作独立重复。
- all-range、Huber、Cauchy 和 `fixed_rejection_v1` 从相同 common preparation 独立执行，不依赖 Stage 1。
  rejection 使用一次 all-range LM 的冻结标准化残差、`q_i <= tau_reject` 等号保留、单次 mask 和从
  preliminary Values 开始的最终 checked LM；development 数值均带
  `T09_DEVELOPMENT_ENGINEERING_TEST_ONLY` provenance。
- `Stage1RegularizedResult` 仅在全部 Stage 1 stop audit 通过后产生，保留所有 fixed-valid/in-plan 观测的
  `b_i` snapshot、同驻点 navigation Values、只含物理因子的重建图，以及 `J_physical/J_L1/J_TV/J_stage1`
  和内容 identity。其 covariance 固定为
  `NOT_APPLICABLE_REGULARIZED_STAGE1_NON_GAUSSIAN`；失败时不导出 trajectory/bias/result identity。
- common preparation、automatic/fixed Stage 2 cache 与 Stage 4 final request 是不同身份域；
  `final_request_id` 不替代 T08 `inference_id`。T08 exporter 在写后重新核验 live graph/Values/context identity、
  graph/Values 数量、每个 inference-ID CSV 行和全部 artifact SHA-256，且 full-gate 行为由既有回归保护。
- batch 用 runner 实际导出的 common preparation 内容身份做同一 run-unit 跨方法复核，而不是把预登记请求
  hash 当成实际 graph/Values 身份；最终 replay 中所有执行 cell 一致，且无 `COMPARABILITY_INVALID`。
- Stage 2 cache 发布要求完整 obs/group/score 表、收敛 Stage 2、graph/Values/factor/trace 内容 identity 与逐
  payload SHA-256。partial score 保留为 `COMPLETE_WITH_SCORE_UNAVAILABLE`，不删除 group，也不计为轨迹失败。
  automatic 与 `RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY` 使用不同 namespace/ID，读取时禁止互换；Python
  publisher 与 C++ reader 使用同一长度前缀 canonical cache-ID 编码，并由跨语言固定向量测试保护。
- batch runner 在 estimator effective config 中剥离 GT/truth/label/oracle 字段，fixed debug 只接收无幅值/GT
  的 support manifest；所有进程使用唯一目录并记录 stdout/stderr、外部 wall/RSS 和 terminal state。
  estimator failure 不使 batch 进程非零，schema/scheduling/identity/artifact corruption 才非零。
- evaluator 使用 run-unit 配对、明确的 numerator/denominator/status/reason，区分 trajectory、automatic
  producer、cache diagnostic、score group、recovery/fallback/final failure；`overall_retained_fraction` 分母为
  keyframe 下采样前全部 `valid=true` ledger 观测。轨迹匹配支持有容差 nearest 或禁止外推且限制 gap 的
  interpolation；缺 GT/frame/point provenance、零分母、不完整 bias truth 与无最终 graph 均保留明确 NA。
- 独立 GT exporter 只在 evaluation 侧加载 truth 并写归一化 TUM/manifest；batch runner 接口不接受
  evaluation manifest。
- 本地实际验证：package build exit 0，focused CTest suites 通过，最终完整 CTest 21/21，Python contract tests 与
  `git diff --check` 均 exit 0。development batch 19/19 cell 终态。T07 baseline 全部成功，但 step/ramp
  automatic Stage 1 按预期 `MAX_OUTER_ITERATIONS`，下游无 cache/recovery/final 产物；T06 automatic cache
  保留 partial score；fixed debug cache namespace 隔离成功。完整证据见
  [`evidence/t09_20260907T143701Z/`](evidence/t09_20260907T143701Z/VERIFICATION.md)。
- T09 最终状态由后续独立复审收口为
  `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE / AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`。没有调整
  T07/T06 停止条件或科学参数，没有合同 amendment；`fixed_rejection_v1` 与 nominal-curvature 等号规则仅是
  `SELECTED_T09_ENGINEERING_DEFINITION`。T10–T13、held-out、正式 RQ 和论文数字保持 `NOT_RUN`。

### T09-R01–R08 review-fix 本地边界

- canonical v2 manifest 现在以闭合的 `mode/execution/path` 矩阵控制真实阶段；当前 scope 拒绝 validation/test，
  policy 阈值按精确字段和数值域预检。Stage-2 producer 强制不进 final；FINAL 必须绑定 cache，并由有效 T08
  identity、final summary、factor audit、masks、trajectory 和 `VERIFIED` export 共同判定成功。
- Stage-2 cache 增加完整 `X/V/B/C` typed Values。final replay 只做 common preparation、冻结 Stage-2 graph 重建和
  graph/Values 内容 identity 复核，不执行 Stage 1/2 optimizer；显式 producer/operating-point identity 允许多个
  diagnostic/final 消费同一 cache，并保留 external cache 的 payload/producer ABI 失效检查；归档 cache 也已由
  第二个独立 scheduler 进程成功复用到新的 diagnostic cell。
- evaluator 按 parent cache、inference ID、final masks、segment bias 和 final summary 连接 artifact；decision/final
  score 分列，coverage 使用 candidate/eligible/fixed-valid 合同分母，recovery solve 与 final-audit failure 分列，
  fallback 使用真实 Stage-4 分母；RPE 改为带 `T_G_E/T_Q_I` 的 SE(3) relative-pose translation/rotation。
- comparability mismatch 现在保留证据但使 batch/evaluation boundary exit 2，并从 estimator 聚合隔离；ABI hash
  不再包含 ASLR 加载地址。真实 fixed-partition DEBUG 链路为 1 producer、2 diagnostic operating point、2 final，
  同一 cache 且两个 final 均记录 Stage 1/2 optimizer 未重跑与 graph/Values `VERIFIED`。这不是 automatic 证据。
- review-fix 的 catkin build exit 0、focused 9/9、完整 CTest 24/24（252.70 s）、19/19 development terminal 和
  evaluator 均本地通过；逐项证据见
  [`evidence/t09_review_fix_20260907T164311Z/`](evidence/t09_review_fix_20260907T164311Z/VERIFICATION.md)。状态仅为
  `REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；T09 不标 `DONE`，T10–T13 不启动。

### T09 第二轮独立复审修复边界

- 独立复审仍为 `CHANGES_REQUESTED`；本轮仅修复 R01-A/B、R02-A、R03-A/B/C、R04-A，保持已接受的
  R05/R06/R07/R08 行为。阈值 identity 改为 lossless binary64，final request 独立绑定真实 final
  refit/score/solver 配置，并由 scheduler 复算且核对 parent cache、mode、operating point 和 final summary 阈值。
- Stage-2 cache schema v2 新增 common/Stage1/Stage2-refit/Stage3-score 配置身份和 producer context；gate-only
  operating point 可共享 producer，discovery/refit/score 变化在 Python external consumer 与 C++ replay 两层拒绝，
  不静默重跑 Stage 1/2。真实 T06 AUTO 链为 1 producer、2 diagnostic、2 个相邻精度 final，5/5 terminal；
  两个 final 共用一个 cache 但 request ID 不同，均明确 Stage1/Stage2 optimizer `NOT_RUN_CACHE_REPLAY`。
- evaluator 先验证 final sealed SHA-256 与 mask/factor audit，再用实际 factor count 计算 retained fraction；父 cache
  缺失、普通 estimator failure、recovery/fallback failure 保留 NA row 和 run-unit 分母，完整性损坏才 exit 2。
  trajectory/final/recovery/fallback/diagnostic 以 mode/point/path 分层并在层内按 `run_unit_id` 去重；decision-time
  bias 固定使用 Stage-2 map，final-time accepted risk 只使用完整 final C map，缺失时明确 unavailable。
- replay 成功和失败出口恢复统一 runner 总墙钟与 Stage-4 分项语义。最终被审二进制
  `d790cf3b...dba3f4e` 的本地成功两次 runner/external wall 为
  `95.658483925/95.783073867 s`、`98.129435205/98.259939363 s`；lambda 不兼容直接 runner 失败为
  `95.145223455/95.23 s`，未执行 Stage-4 分项为 null。旧的 `90.821909929/90.94 s` 属于前一运行版本，
  原始日志保留但不再作为本摘要的最终版本数字。
- 本轮 catkin build、tests target、focused 10/10、最终完整 CTest 25/25（1034.62 s）、真实链、evaluator、tamper
  rejection 和 diff check 均完成；逐项证据见
  [`evidence/t09_rereview_fix_20260907T180000Z/`](evidence/t09_rereview_fix_20260907T180000Z/VERIFICATION.md)。
  状态仍仅为 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；不生成 locked gate，不升级数据
  角色或 claim，不进入 T10。

### T09 最终独立复审结论

- 被审源码、runner `d790cf3b...dba3f4e`、实际加载 core library `9e1cfa09...3ebe6ce` 与第二轮本地修复清单
  一致；相关 focused CTest 9/9 通过。复用既有有效 AUTO cache 的两个独立 final consumer 均 exit 0，两个
  threshold/request identity 不同，parent cache 相同，Stage 1/2 optimizer 均为 `NOT_RUN_CACHE_REPLAY`，
  graph/Values 与最终 export 均为 `VERIFIED`。本轮热缓存 runner/external wall 分别为
  `11.326891761/11.341724468 s` 与 `9.712354287/9.725080793 s`。
- discovery-only 不兼容在 scheduler/C++ 两层分别 exit 2/1；direct runner 为
  `CACHE_STAGE1_CONFIG_INCOMPATIBLE`，runner/external wall `9.605174458/9.61 s`，Stage 4 全部 null。普通
  `final_masks.csv` 篡改保持旧 seal 时 evaluator exit 2；missing-parent、普通 estimator failure、recovery/fallback
  failure 的 NA 与 run-unit 分母，以及 decision/final bias map 分离均通过独立反例。
- **剩余 R03-B 缺陷：** [`tools/paper/evaluate_runs.py`](../../tools/paper/evaluate_runs.py) 只保证每个适用 mask
  能找到 audit，却不拒绝 audit 中额外的 `obs_id`，随后把全部 audit 行的 `final_factor_count` 求和。向真实成功
  final 添加一条合法形状的 `NONCANDIDATE_REFERENCE` audit 行并更新该文件的 manifest SHA-256 后，evaluator
  仍 exit 0，`overall_retained_fraction` 从 `41/627` 变为错误的 `42/627`。T09 工程验收因此未通过。
- 最小验收要求：在计算任何指标前证明 audit `obs_id` 集合与所有非
  `NOT_IN_FROZEN_VALID_PLAN` mask 行完全相等，逐行核对 candidate/noncandidate classification、expected/count/
  `ok` 与 `final_use`，coverage 只消费这份已验证映射；加入额外 audit、错误 noncandidate classification 及正常
  `41/627` 回归。修复后只需重跑该定向反例、R03/R04 evaluator 回归、两个 fresh AUTO final consumer 和相关
  focused tests；不需重跑整个 sprint。完整自包含校验包见
  [`evidence/t09_final_review_20260908T050201Z/`](evidence/t09_final_review_20260908T050201Z/VERIFICATION.md)。

### T09 R03-B 最小修复本地边界

- 修改前完整归档 `/tmp/t09-review-d0t4h1ne/` 的 324 个文件；源/归档相对路径哈希树同为
  `16858007c2fdfdf0bfe031bac71d92aac8d25a990a345bc2240c1aec1071b6e5`。历史 evidence 未覆盖。
- evaluator 对有效 final 强制验证 parent cache payload 与 `observation_mapping_sha256`，本地
  `observations.csv` 若存在必须与 parent ledger 同 hash；分母只取已验证 parent ledger 中 keyframe 下采样前
  全部 `valid=true` 观测。全部 mask obs domain 必须等于 parent ledger，适用 mask domain 必须等于
  `valid && planned` ledger domain，factor audit domain 再与适用 mask 精确双向相等。
- accepted candidate、suppressed candidate、noncandidate reference 分别严格核对 mask flags、classification、
  `final_factor_count`、`expected_count` 与 `ok`；额外、缺失、重复 audit 行均拒绝。retained numerator 由该
  已验证映射产生，coverage 不再重新读取未验证 audit 或优先使用本地 ledger。
- 真实归档 final 重放保持 exit 0、`41/627`；orphan/missing/duplicate audit、错误 noncandidate
  classification、矛盾 expected count/flags 和本地 ledger `valid` 篡改均 exit 2 且不写 evaluation。
  missing-parent、普通 baseline failure、failed recovery/fallback 仍保留 NA 与 run-unit 分母；focused CTest
  9/9。完整命令、产物和被修改源码快照见
  [`evidence/t09_r03b_fix_20260908T053929Z/`](evidence/t09_r03b_fix_20260908T053929Z/VERIFICATION.md)。
- 本轮只修改 `tools/paper/evaluate_runs.py` 与必要的 `test/test_t09_evaluator_artifact_graph.py`；未修改 C++、
  科学配置、停止条件、T08 identity 或合同。状态仅为
  `IN_PROGRESS/R03B_PARENT_CACHE_IDENTITY_LOCAL_FIX_PASS_AWAITING_INDEPENDENT_REREVIEW`，不表示 T09 工程验收通过。
- 后续独立复审指出 parent payload/ledger 可一致重封，但 evaluator 未重算 canonical v2 cache ID，从而曾接受
  过期 ID 并产生错误的 `41/626`。修改前已完整归档 `/tmp/t09-r03b-rereview-r9j21a6q/` 的 349 个文件；
  evaluator 现直接加载 `run_experiments.py::stage2_cache_identity`，先核对 v2 schema、重算 ID，再要求其与
  parent manifest 及 cell 相等；final artifact 校验只消费这份已验证 ID，并要求旧 final 不能绑定重算后的
  新 parent ID。没有引入另一套身份定义。
- 使用完整 v2 cache fixture 的新增回归和真实归档反例均通过：payload/ledger hash 一致但保留过期 ID 时
  exit 2、无 evaluation；parent 重算新 ID 且 cell 随之更新时，旧 final binding 仍 exit 2、无 evaluation。
  同一最终版本真实 final 保持 `41/627`，七个既有负例、failure/NA 与 decision/final bias accounting 均保持，
  focused CTest 9/9。完整证据见
  [`evidence/t09_r03b_parent_identity_fix_20260908T062741Z/`](evidence/t09_r03b_parent_identity_fix_20260908T062741Z/VERIFICATION.md)。

### T09 最终独立复审收口

- `/tmp/t09-final-rereview-bojvs6no/` 已原样归档到
  [`evidence/t09_final_rereview_closeout_20260908T065646Z/review_input/`](evidence/t09_final_rereview_closeout_20260908T065646Z/review_input/REVIEW.md)。
  源与归档的 743 个相对路径及内容逐项一致；归档内原 `MANIFEST.sha256` 覆盖 742 个文件并全部 `OK`，
  manifest 自身 SHA-256 为 `bf6734b5108730e32c4bad8e4aa6864270f240ce322340184e8a2ade7208e371`。
- 最终独立复审接受 R03-B 的 audit/ledger/mask 双向完整性、validated numerator/denominator 与 parent canonical-v2
  内容身份绑定；结合此前接受的 R01–R08，T09 在 development engineering scope 内关闭为
  `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`。正常真实 final 保持 `41/627`；完整身份链反例、七个既有
  负例、failure/NA/run-unit 分母均未回退；focused CTest 9/9。两个 fresh C++ AUTO final consumer 均 exit 0、
  request 不同、parent cache 相同、Stage 1/2 optimizer 为 `NOT_RUN_CACHE_REPLAY` 且 export `VERIFIED`。
- 本轮独立复审没有重跑 package build、完整 CTest 25、19-cell batch、T07 step/ramp 或新的 discovery；正式
  RQ、validation/test、locked gate、T10–T13 均 `NOT_RUN`。T07 step/ramp automatic producer 仍为 Stage 1
  `MAX_OUTER_ITERATIONS`，因此状态同时保留 `AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`；fixed DEBUG 与 T06
  partial-score replay 不替代 automatic accepted-candidate 科学证据。独立 fixed beta、标定/数据许可等外部
  provenance 仍未闭合，C1–C3 不升级为 `SUPPORTED`。
- 归档、原始复审结论、命令/退出码、源码/二进制身份及文档收口一致性见
  [`evidence/t09_final_rereview_closeout_20260908T065646Z/`](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md)。

## T10 前置检查与 Stage 1 诊断边界

- 当前状态为 `IN_PROGRESS/A04_REVIEW_ACCEPTED/A05_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE/A06_REVIEW_ACCEPTED_BOUNDED_SCOPE/A07_REVIEW_ACCEPTED_BOUNDED_SCOPE/A08_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE/A09_REVIEW_ACCEPTED_BOUNDED_SCOPE/A10_LOCAL_DELIVERY_COMPLETE_AWAITING_REVIEW`。准入清单、AUTO/fixture/fixed DEBUG 的证据等级、
  truth/split/calibration 缺口、scheduler development-only 限制、待审数值与预算见
  [`T10_READINESS.md`](T10_READINESS.md)。前置检查已经接受；本阶段仍不构成 T10 validation 或 gate lock。
- T06 真实 AUTO development 短输入可完成 Stage 1/2，但 3 个 group 全部不 eligible；T09 证明同一 AUTO
  partial cache 可供两个 final consumer replay，结果为 `NO_ELIGIBLE_CANDIDATES`。T07 step/ramp 真实 AUTO
  均停在 Stage 1，engineering fixture 及 `FIXED_PARTITION_DEBUG` 不能替代 accepted-candidate AUTO 科学闭环。
- 新 step/ramp 原参数 trace 各有 50 个 outer iteration。两者 relative objective、scaled step、navigation
  stationarity 都是 0/50 通过；chain ADMM/KKT 都是 50/50 通过。末轮相对各自阈值，step 为
  `105.07× / 303.04× / 6406.50×`，ramp 为 `245.86× / 397.90× / 8732.41×`；末轮 chain KKT
  分别为阈值的 `0.9946×` 与 `0.9795×`。`MAX_OUTER_ITERATIONS` 只描述终态。
- 新诊断表明 100/100 次 conditional LM 都只由 actual linked-GTSAM relative-decrease 谓词停止；LM 停止时
  旧 bias 条件图的 navigation stationarity 仍为 step/ramp 各 0/50。更新 bias 后同 Values 的梯度在两条 run
  各 50/50 次变大，末轮 post/pre 为 `23.86×/46.38×`。这支持 LM 停止不足与 block coupling 混合，不证明
  GTSAM 调用错误；数据/模型归因仍为 `UNKNOWN`。
- step/ramp 各使用一次预算（`2/2`），未改 iteration cap、容差、注入参数、科学配置、停止条件或冻结合同。
  两次均保持历史终态，18 个旧 trace 字段逐行最大差 `0`。完整命令、hash、诊断 JSON、未运行清单见
  [`evidence/t10_stage1_diagnostics_20260908T081328Z/`](evidence/t10_stage1_diagnostics_20260908T081328Z/VERIFICATION.md)。
- 下一步建议先审查 default-off、版本化的 stationarity-qualified conditional solve 属于 correctness fix 还是
  method amendment；本阶段不实施行为改变。失败与零 coverage 仍须完整记账，不要求失败场景变成功。
- A02 随后已按用户裁定登记并实现为 development-only 数值策略。focused config/discovery/refit/runner 验收均
  通过；step/ramp 各一次仍为 50 轮 `MAX_OUTER_ITERATIONS`。A02 使全部 conditional checkpoint 驻点通过，
  但 chain 后末轮梯度仍为 `6.317840e-3/8.691964e-3`，Stage 1 耗时约为诊断基线的 `3.87/3.93` 倍，未进入
  Stage 2。见 [A02 对照证据](evidence/t10_conditional_stationarity_policy_20260908T091614Z/VERIFICATION.md)。
- T10-A02-R01 独立复审反例证明 A02 内外层 stationarity tolerance、roundoff 与五类尺度曾可冲突但共用
  support identity。本轮在 `AutomaticSupportProvider` 和独立 `BuildAutomaticSupportPartition` 入口统一
  检查；7 类冲突逐项在双入口拒绝，一致 A02 输入、默认策略身份/行为和 standalone conditional LM 均保持。
  focused build exit 0，config 11/11、discovery 26/26、refit/scoring 23/23、runner contract exit 0。
- A03 已在运行前登记为仅有的 `DEVELOPMENT_BUDGET_DIAGNOSTIC`。两份新配置相对 A02 cap=50 只把
  `discovery_max_outer_iterations` 改为 500；step/ramp 各唯一执行一次，均未 timeout。两条前 50 轮所有公共
  非计时 trace 精确一致（最大差 0），config/solver/support-context/snapshot 及潜在 Stage-2 cache producer
  config identity 分离；raw T07 source cache identity 按设计相同。
- step 完成 120 个 outer 后在第 121 次 conditional solve 失败，ramp 完成 126 个后在第 127 次失败；原因均为
  原 50-check inner budget 下 `CONDITIONAL_LM_STATIONARITY_NOT_REACHED`，不是 outer=500 或 120 秒 timeout。
  最后完成轮 objective/chain 已通过，但 combined step 与 post-chain stationarity 均未通过；四项 AND 首次
  同时满足轮数均不存在。Stage 1/Stage 2 分别为 `FAILED/NOT_RUN`，eligible candidate `NOT_EVALUATED`。
  truth 仍为 `INJECTED_COMPONENT_ONLY`、base total latent bias `UNKNOWN` 且 fixed beta 缺失，不能科学评价。
  不继续增加预算；见 [R01/A03 证据](evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/VERIFICATION.md)。
- A02-R01 与 A03 已由独立复审接受。本轮按接受意见只执行一次原 A03 step 配置 replay（整进程
  `2.506007 s`，未 timeout），未运行 ramp。outer 121 的 50 次 wrapper check 中只有第 1 次 LM 候选被接受：
  optimizer/inner iteration `0→1`、lambda `1e-5→1e-6`、error 减少 `1.3233e-10`。其后 49 次 linked
  GTSAM 都生成同一 tentative step，但 nonlinear cost 增加 `9.76996e-15`、model fidelity
  `-0.0180 < 0.001`；又因 `abs(costChange) < relativeErrorTol*error`，`tryLambda` 直接返回 true 而不写回
  state、不增加 lambda/inner/optimizer iteration。A02 驻点仍为 `2.29030094e-6 > 1e-6 + roundoff`，wrapper
  遂重复调用至 50-check cap。linked stdout 恰有 50 条 try、49 条 small-reduction return、0 次 lambda-search
  rejection；这解释了计数，不是从末尾汇总推断。
- 最大梯度坐标 `x1` translation local coordinate 4 的解析值为 `-2.29030094e-6 objective/m`；预声明
  13 点中央差分网格中 `1e-4..3e-8` 连续 8 点满足
  `|fd-analytic| <= 5e-9+5e-3|analytic|`。这不支持该坐标存在 Jacobian/objective graph 不一致；tentative
  目标差确处于相减敏感尺度，但 authoritative Values 不变的直接原因是候选未接受和 linked 返回分支。
  前 120 轮 79 个非计时字段与 A03 baseline 精确一致。Stage 1 失败、Stage 2 未运行、eligible 未评价，
  truth/provenance 缺口不变。见
  [停滞定位证据](evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md)。
- A04 使用唯一一次原 A03 step replay 重建同一 live conditional graph；A/B 从 authoritative stall 后同一
  objective `4.8408701715814653`、Values、lambda `1e-6` 启动。A 为 50 calls/50 small-change returns、0
  accepted、gradient `2.29030094e-6`，不驻点。B 只把内部 `relativeErrorTol` 改为 0，先在 `1e-6` 拒绝，
  增加到 `1e-5` 接受，再于下一 call 的 `1e-6` 接受；2/2 accepted update 均严格下降，总下降
  `3.86357613e-13`，最终 gradient `3.45174471e-7` 并通过原 external generic + stationarity。A/B 分别
  `0.054561/0.002497 s`，lambda upper `1e5`、50-call cap 与接受门槛不变。见
  [A04 证据](evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/VERIFICATION.md)。这是明确的内部数值
  语义变化和 shadow 可行性证据，不是 A02/default 集成、Stage-1 成功或科学结果。
- A05 已集成默认关闭的版本化 V2：optimizer 内部 `relativeErrorTol=0`，外部 generic rel/abs、原
  stationarity/roundoff/五类尺度、GTSAM 接受规则、lambda upper、50-call cap 和外层四项 AND 不变。
  step/ramp 各一次 outer=500 定向运行恢复旧 outer 121/127 checkpoint，但分别在 outer 123/168 因
  `lambda=1e5` 搜索耗尽而失败；完成 122/167 轮，均无 timeout、Stage 1 failed、Stage 2 `NOT_RUN`、eligible
  `NOT_EVALUATED`。见 [A05 证据](evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/VERIFICATION.md)。
  V2 与 V1 的 config/solver/support/snapshot/scheduler producer identity 分离；因 Stage 1 失败未产生实际 cache。
  公开 diagnostic struct 有新增字段，旧 overload 源码调用保留但受影响二进制已重链接，不声称 binary ABI 兼容。
- A05 独立复审以 `CHANGES_REQUESTED` 返回两项：`T10-A05-RF1` 对应原报告 F1，确认 linearize 阶段异常被
  虚增为 1 次 trial/rejection；`T10-A05-RF2` 对应原报告 F2，确认 A05 历史包未加密绑定运行时加载的
  `libuwb_imu_fgo.so`/GTSAM。review-fix 不改变 solver 数值策略：catch 仅累计 GTSAM state 已提交的
  transition；异常可能发生在 trial 内但 counter 尚未提交时，计数作为 confirmed lower bound 并标
  `INCOMPLETE_EXCEPTION_DURING_ITERATE`，不伪造总数。pre-trial fixture 独立证明实际 trial/rejection 为
  0；trial 内 fixture 则明确证明总数不可由现有 linked state 完整确定。`NOT_EXECUTED`、`COMPLETE` 与
  `INCOMPLETE_EXCEPTION_DURING_ITERATE` 已进入 CSV/JSON，failure diagnostic schema 升为 v2。
- 本轮重新链接公开 struct 的受影响调用者；discovery 32/32、config 11/11、refit 23/23、inference 18/18、
  methods 5/5、cache 7/7、scheduler identity、runner contract 和带断言独立 probe 均 exit 0。测试前后
  runner/core/GTSAM/discovery-test 身份一致；最终证据另绑定全部实际测试 executable 的 realpath、SHA-256、
  build ID、原始及去 ASLR `ldd`。这些是本轮运行身份，不是 A05 历史 step/ramp 的事后加载证明；历史缺口保留。
- V2 在“初值已达最优、非零 residual 且 `J^T r=0`”时仍按已登记语义在 lambda 上界耗尽后失败，external
  generic/stationarity 不执行且不导出 Values；本轮回归明确锁住该已知边界，不将其改作 correctness fix，
  也不推断为历史 outer 123/168 的根因。完整 review-fix 证据见
  [`evidence/t10_a05_review_fix_20260909T040832Z/`](evidence/t10_a05_review_fix_20260909T040832Z/VERIFICATION.md)。
- 指挥追加 `T10-A05-RF1-N01`：对 `r(x)=1+0.003x+0.5*0.000218x^2`，默认与 A02 V1 的单次 wrapper
  调用均由 fixture 独立观察到 2 trials（`1e-5` rejection 后 `1e-4` small-change）、1 rejection、0 accepted、
  1 normal no-update；修复前 wrapper 只报 1 trial 且错误标 `COMPLETE`。本地最小修复仅在正常返回时按 linked
  state 区分 terminal small-change 与 lambda exhaustion，异常仍为 confirmed lower bound/`INCOMPLETE`，
  `NOT_EXECUTED` 不变。discovery 34/34、其余 focused tests、runner contract、独立前后 probe 与本轮
  core/GTSAM 身份稳定性均通过；step/ramp 为 `NOT_RUN (0/2)`。证据见
  [`evidence/t10_a05_counter_review_fix_20260909T060028Z/`](evidence/t10_a05_counter_review_fix_20260909T060028Z/VERIFICATION.md)。
- 本轮指挥会话实际复核 counter-review-fix/A05 review-fix/A05 原包 manifest `233/233`、`196/196`、
  `160/160`，三个当前源码快照、最终运行身份 `9/9`、discovery `34/34`、双策略 post probe 与 final audit，
  随后接受 `REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE`。该会话接受限定关闭 RF1、RF2、
  RF1-N01 工程范围；指挥未重建、未重跑 step/ramp、未修改仓库，也不存在本轮外部 `REVIEW.md`。
- A06 按运行前登记只增强 observer 的最终驻点序列化和 linked stdout 精度/调用边界，不增加 solve 或改变决策。
  step outer 123/ramp outer 168 各唯一运行一次、child exit 1、无 timeout；两点均 `NOT_STATIONARY`。终止搜索
  trial 都是 predicted decrease 正、actual decrease 负，model fidelity 为负或最后一个 ramp trial 不可用；
  方向随 lambda 缩小。主导坐标有限差分支持解析梯度，浮点相减敏感只覆盖部分 trial，完整方向/逐 factor
  根因仍 `UNKNOWN`。历史 86 个公共非计时字段和终态精确一致，无有效估计/Stage-2 cache。见
  [A06 证据](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。
- 本轮指挥会话接受 `REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE`；指挥实际核验 manifest
  `247/247`、源码 `4/4`、当前运行身份 `9/9`、discovery `34/34`、历史 `122/167 x 86` 公共字段和终止
  `11/10` 个拒绝 trial，未重建、重跑 estimator 或修改仓库，且不存在外部 `REVIEW.md`。`64*epsilon`
  只是一项已登记敏感性判据，不是 factor/retract/累加的完整浮点误差上界；A06 没有新增 solve，但既有
  `InspectFirstLinkedLmTry` 本身包含 diagnostic solve。
- A07 只把实际 linked `TRYDELTA` stdout 以 max-digits 捕获并在 authoritative `iterate()` 返回后求值；没有
  修改 `/usr/local` GTSAM、新增 solve、复制 optimizer 或让诊断参与决策。小图断言全 3 key/15 维、文本解析
  与 actual delta/`retract` round-trip 一致，方向 FD 3/3 通过；真实两点 22/22 trial 全 24 key/120 维完整。
  预登记 6 个方向的 18 个 FD 均与解析 `g^T u` 不一致，高 lambda 方向还出现解析下降而实际一阶上升；逐
  factor long-double 差值和 direct graph change 同为目标上升，因此不是仅由 graph 总量直接相减改变符号。
  factor 变化存在 `5.28e4--6.85e6` 倍对消且 UWB expression 绝对变化最大；因未记录逐 factor 方向导数，
  尚不能在 UWB/pose prior/IMU 中指定错误 factor。历史 `122/167 x 86` 与终态仍精确一致，无有效估计或
  Stage 2。见 [A07 决策证据](evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)。
- 本轮指挥会话接受 `REVIEW_ACCEPTED_A07_BOUNDED_EXACT_DIRECTION_DIAGNOSTIC_SCOPE`；这是会话验收，不是
  外部 `REVIEW.md`，指挥未重建、重跑或修改仓库。A08 在冻结六方向/三步长保留 1314 个逐 factor 点，证实
  linked `PriorFactor<Pose3>` 对 `-Local(x,prior)` 的 identity Jacobian 与 Pose3 retract 不一致，并以
  paper-only factor 提供 `-D_x Local(x,prior)`；残差、目标、noise、prior mean/weight 和 legacy builder 不变。
  独立反例由 pre-fix exit 1 变为 post-fix exit 0，focused tests 全通过。修复后两条 Stage 1 分别在 outer
  178/173 满足原四项 AND，但 Stage 2 均在原 50 轮上限失败，无 cache/有效估计。冻结 factor 点
  `1314/1314` 通过，图级 `15/18`；三个 `h=1e-6` 残差保留，来源仍 `UNKNOWN`，未放宽判据。见
  [A08 决策证据](evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/DECISION.md)。

T01 产物：

- [`METHOD_CONTRACT.md`](METHOD_CONTRACT.md)
- [`EXPERIMENT_CONTRACT.md`](EXPERIMENT_CONTRACT.md)
- [`paper/main.tex`](../../paper/main.tex) 与 [`paper/main.pdf`](../../paper/main.pdf)
- [`paper/CLAIM_EVIDENCE.md`](../../paper/CLAIM_EVIDENCE.md)
- [T01 编译与检查证据](evidence/t01_20260906T052319Z/VERIFICATION.md)
- [T01 review 检查证据](evidence/t01_review_20260906T061639Z/VERIFICATION.md)
- [T01 收尾检查证据](evidence/t01_closeout_20260906T065100Z/VERIFICATION.md)
- [T02 构建、测试、运行与回归证据](evidence/t02_20260906T065545Z/VERIFICATION.md)
- [T02 review 修复与真实 loader 裁剪证据](evidence/t02_review_20260906T090000Z/VERIFICATION.md)
- [T02 第二轮复审收尾证据](evidence/t02_rereview_20260906T081734Z/VERIFICATION.md)
- [T02 最终独立复核收口证据](evidence/t02_final_review_20260906T083514Z/VERIFICATION.md)
- [T03 NumPy recoverability golden reference 证据](evidence/t03_20260906T084306Z/VERIFICATION.md)
- [T03 review 修复与跨 BLAS 证据](evidence/t03_review_20260906T093843Z/VERIFICATION.md)
- [T03 最终独立复核收口](evidence/t03_final_review_20260906T110914Z/VERIFICATION.md)
- [T04 oracle 分段联合 refit 证据](evidence/t04_20260906T111500Z/VERIFICATION.md)
- [T04 review R01–R03 修复证据](evidence/t04_20260906T121005Z/VERIFICATION.md)
- [T04 最终独立复核收口](evidence/t04_final_review_20260906T143023Z/VERIFICATION.md)
- [T05 Gate A sparse 原型证据](evidence/t05_sparse_prototype_20260906T143023Z/VERIFICATION.md)
- [T05 Gate B 真实图与 runner 证据](evidence/t05_20260906T144556Z/VERIFICATION.md)
- [T05 review R01–R05 修复证据](evidence/t05_review_fix_20260906T160000Z/VERIFICATION.md)
- [T05 最终独立复核收口](evidence/t05_final_review_20260907T022011Z/VERIFICATION.md)
- [T06 自动支撑发现本地证据](evidence/t06_20260907T025122Z/VERIFICATION.md)
- [T06 review R01–R05 修复证据](evidence/t06_review_fix_20260907T035905Z/VERIFICATION.md)
- [T06 最终独立复核收口](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md)
- [T07 初次本地实现证据](evidence/t07_20260907T053301Z/VERIFICATION.md)
- [T07-R01--R03 review-fix 本地证据](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md)
- [T07 最终独立复审收口证据](evidence/t07_final_review_20260907T070102Z/VERIFICATION.md)
- [T07 final-closeout 自包含归档修复](evidence/t07_closeout_fix_20260907T072733Z/VERIFICATION.md)
- [T08 冻结决策/最终推断本地证据](evidence/t08_20260907T083437Z/VERIFICATION.md)
- [T08-R01–R03 review-fix 本地证据](evidence/t08_review_fix_20260907T092037Z/VERIFICATION.md)
- [T08-R02 最终定向修复证据](evidence/t08_r02_review_fix_20260907T104154Z/VERIFICATION.md)
- [T08 最终独立复审收口](evidence/t08_final_review_20260907T121510Z/VERIFICATION.md)
- [T09 初次本地证据](evidence/t09_20260907T143701Z/VERIFICATION.md)
- [T09 R01–R08 review-fix 本地证据](evidence/t09_review_fix_20260907T164311Z/VERIFICATION.md)
- [T09 第二轮 rereview-fix 本地证据](evidence/t09_rereview_fix_20260907T180000Z/VERIFICATION.md)
- [T09 最终独立复审与证据收口](evidence/t09_final_review_20260908T050201Z/VERIFICATION.md)
- [T09 R03-B 最小本地修复证据](evidence/t09_r03b_fix_20260908T053929Z/VERIFICATION.md)
- [T09 R03-B parent-cache identity 本地修复证据](evidence/t09_r03b_parent_identity_fix_20260908T062741Z/VERIFICATION.md)
- [T09 最终独立复审收口证据](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md)
- [T10 前置检查与最小诊断](T10_READINESS.md)
- [T10 precheck 独立证据](evidence/t10_precheck_20260908T073803Z/VERIFICATION.md)
- [T10 A02-R01 修复与 A03 限定预算诊断](evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/VERIFICATION.md)
- [T10 conditional LM 停滞定位](evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md)
- [T10 A04 fixed-checkpoint LM 恢复对照](evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/VERIFICATION.md)
- [T10 A05 默认关闭 V2 集成与定向运行](evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/VERIFICATION.md)
- [T10 A06 新失败 checkpoint 诊断](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)
- [T10 A07 actual LM 方向诊断](evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)
- [T10 A08 冻结方向逐 factor 定位与 Jacobian correctness fix](evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/VERIFICATION.md)
- [T10 A09 Stage 2 有界预算诊断](evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/VERIFICATION.md)

## 已知未决项与阻塞范围

| 未决项 | 当前状态 | 阻塞范围 |
|---|---|---|
| 独立 LOS 固定 `beta` 标定 | `MISSING`；没有填 0 | fixed-beta 正式配置及 RQ2–RQ4 主实验 |
| 自有/外部数据许可与发表权限 | `UNKNOWN` | 可发表数据证据、公开 artifact、C1/C3 release 表述 |
| GT 刚体点、杆臂、时钟及部分标定来源 | `UNKNOWN` | raw-frame ATE、真实 measured bias reference、公平跨数据比较 |
| 本机自有多链路 NLOS、两种运动及重复记录 | `MISSING_LOCAL_DATA` | controlled RQ2 与 C2/C3 真实主证据 |
| IMU preintegration 构造参数/测试语义冲突 | `RESOLVED_T02`：T00 的 38 tests/2 failures 保留为历史；构造函数现使用调用方 `gravity_world`，T02 review 当前 68 tests/0 failures | 不再阻塞后续实现；正式实验仍受数据、标定和许可项约束 |
| linked-devel 同名仿真库碰撞与 overlay 风险 | `OBSERVED_RISK` | T07/runner 的二进制 provenance 和环境复现 |
| T07 cache/truth 双目录发布 crash atomicity | `NON_BLOCKING_ENGINEERING_LIMITATION`：可捕获异常会清理 staging/partial final；不声明 `SIGKILL`、进程死亡或断电下原子发布 | 不影响已接受的开发工程范围；发布系统强化若需要须另立任务 |
| T08 gate 数值 | `PENDING_VALIDATION`；当前只允许显式 development-only engineering fixture 值，不是 T10 lock | T10 validation、所有正式 gate/RQ/claim |
| T07 step/ramp automatic 闭环可用性 | `PARTIAL_DIAGNOSTIC`：A08 paper prior fix 后 Stage 1 已在 outer 178/173 满足四项 AND；A09 仅增 Stage-2 refit cap 后又在 outer 66/124 收敛并形成两个 cache。其 1/3 个 group 均因 short/boundary 不 eligible，score unavailable；输入仍是已查看的 development 数据，truth/calibration/split 不闭合 | T10 AUTO eligible-candidate validation 准入；不影响 T07/T09 既有 development engineering 验收 |
| T10 split/预算/数值准入 | `PROPOSED_NOT_ADMITTED`：A10 已交付独立 base/recording/seed split、四策略各12点、公平缓存预算及 B_total=14400 s/25%预留；scheduler 改造仅为具体待审方案 | 当前合成 AUTO pilot 首次 conditional LM 失败；正式 validation 仍需数值/实现/role-provenance 准入，不是缺一份计划 |

这些未决项是外部事实、correctness 或证据缺口，不是 scope amendment。合同中的交替非负
refit 已经 T04 独立复核接受；T05 sparse/列 equilibration 与端点重叠在当前测试支持域已独立复审接受。
一次 merge 的执行语义会影响 partition 定义与 cache hash，A01 已由本轮用户指挥/审查会话技术接受，
T06 实现/U11 fixture 已在 development engineering scope 内独立复审接受。所有科学数值门限均按来源标为
`PENDING_VALIDATION` 或 `PENDING_NUMERICAL`。

## 任务交接

每次任务结束追加一行；“验证”必须区分实际运行与 `NOT_RUN`。

| UTC 日期 | 任务 | 提交/工作区 | 改动与产物 | 实际验证（命令、退出码、证据） | `NOT_RUN`/风险 | 下一步 |
|---|---|---|---|---|---|---|
| 2026-09-05 | T00 前最小启动准备 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014`；新增文档未提交 | 建立协作约束、统一入口和预审计模板 | 链接目标检查、`sha256sum -c`、`git diff --check` 均 exit 0；另以 `git diff --no-index --check` 检查 3 个未跟踪新文件，无空白错误 | build/test/baseline/trajectory/ATE/runtime/memory 全部 `NOT_RUN` | 执行 T00 |
| 2026-09-05 | T00 本地审计、数据清点与旧基线尝试 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014`；开始时已有 3 个未跟踪文档，结束时仍未提交 | 更新 [`REPO_AUDIT.md`](REPO_AUDIT.md)；新增隔离证据目录 [`evidence/t00_20260905T102745Z/`](evidence/t00_20260905T102745Z/)；未修改源码、配置、CMake 或冻结材料 | 当前 workspace `catkin build uwb_imu_fgo --no-status` exit 1（缺 `uwb_driver`）；GTSAM 4.2 CMake probe exit 1（仅 4.0.3）；数据 bag 健康检查 exit 0；预存节点启动 exit 127（GTSAM ABI 缺符号） | tests、trajectory、ATE/RPE、estimator 规模/耗时/峰值内存、重跑差异均 `NOT_RUN`；独立 LOS 标定与发表权限未建立；T00 `BLOCKED` | 可基于审计事实开始 T01；并行恢复依赖后重建、测试并重跑 baseline |
| 2026-09-05 | T00 新环境复核、数据清点与旧基线复现 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；开始时 clean，结束时本次证据/文档未提交 | 更新 [`REPO_AUDIT.md`](REPO_AUDIT.md)；新增隔离证据目录 [`evidence/t00_newenv_20260905T154344Z/`](evidence/t00_newenv_20260905T154344Z/)；仅定向清理并重建可再生的 `uwb_imu_fgo` build/devel 产物；未修改源码、仓库配置、CMake 或冻结材料 | 初次 build exit 1（旧路径 cache）；定向 `catkin clean --yes uwb_imu_fgo` exit 0；重建 exit 0；tests exit 1（38 tests、2 failures）；9 个 bag 健康检查均 exit 0；Walk1 两次节点 exit 0，aligned ATE `0.169642 m`，轨迹/GT/calibration 哈希一致 | 自有 UGV、仿真、MILUV、VIRAL estimator `NOT_RUN`；独立 LOS `beta`、数据许可和部分标定来源 `UNKNOWN`；IMU preintegration 测试语义冲突未修 | 执行 T01，并登记所有未决项；T02 前处理 correctness 与输入/输出合同 |
| 2026-09-06 | T01 方法/实验合同与论文工程 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保留 T00 未提交文档/证据，本轮产物未提交 | 新增两份合同、`paper/main.tex`、`paper/CLAIM_EVIDENCE.md`、T01 隔离验证日志/PDF；只在工作稿加入缺包 fallback 与 T00 事实；未修改源码、测试、配置、CMake 或冻结材料 | 初次 compile exit 1 并保留日志；修正 fallback 后 `pdflatex -interaction=nonstopmode -halt-on-error -file-line-error main.tex` 连续两遍 exit 0，生成 10 页 PDF；冻结 hash、Markdown links、tracked/new-file whitespace checks 通过。见 [验证记录](evidence/t01_20260906T052319Z/VERIFICATION.md) | C++/tests/estimator/paper runner/U01–U14/RQ1–RQ4 全部 `NOT_RUN`；IEEE submission layout 因本机缺 `IEEEtran/algorithm/algpseudocode` 为 `NOT_RUN`；六类未决项见上 | 审查 T01 的 `SELECTED_NOT_YET_REVIEWED` 工程选择；随后执行 T02，不并行启动 T03 |
| 2026-09-06 | T01 review：RQ3、baseline/ablation、segment merge | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保留上一行 T01/T00 未提交产物和证据 | 补齐 RQ3 fixed-partition DEBUG 与 automatic-discovery E2E 双路径及隔离 cache；逐项映射冻结 baseline/ablation 和 nominal-curvature；提出确定性单遍 merge `A01`；同步 claim ledger；未修改论文、源码、测试、配置、CMake 或冻结材料 | 冻结 hash、40 个文档链接和当前 T01 新文本 whitespace 检查 exit 0；全仓额外扫描发现 5 个受保护 T00 生成证据的既有 EOF 空行，本轮未改；见 [review 验证记录](evidence/t01_review_20260906T061639Z/VERIFICATION.md)。论文未改，沿用上一行双遍编译证据，未重新编译 | estimator/C++ tests/U01–U14/RQ1–RQ4/全部 baseline 与 merge fixture 均 `NOT_RUN`；A01 待裁决；六类未决项见上 | 保持 T01 `IN_PROGRESS` 并交回审查；裁决 A01，复核通过前不进入 T02/T03 |
| 2026-09-06 | T01 最小收尾 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保留全部既有未提交修改和证据 | oracle reference 改为仿真参考且不保证上下界；METHOD §7 分开 T04/RQ3/RQ4 debug；A01 记录本轮用户指挥/审查会话技术接受但未实现/测试 | 冻结 hash、37 个链接、tracked/new T01 文本 whitespace、11 项内容断言均 exit 0；见 [收尾验证](evidence/t01_closeout_20260906T065100Z/VERIFICATION.md) | C++/runner/U01–U14/RQ1–RQ4 `NOT_RUN`；A01 U11 `NOT_RUN` | T01 `DONE`；开始 T02，不进入 T03–T13 |
| 2026-09-06 | T02 paper 输入路径与最小运行入口 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护并保留 T00/T01 全部未提交产物 | 修复 preintegration 调用方重力向量语义；新增 paper raw ledger/stable `obs_id`、valid/suspected 分离、冻结 keyframe/sigma plan/hash、fixed-beta 常量接口、同后端薄 runner、统一 trajectory/bias/residual 导出和显式能力状态 | `catkin build` exit 0；`catkin test` exit 0（60 tests、0 failures）；SFUISE development smoke exit 0，4850 raw/4266 valid/541 planned/76 suspected，plan hash `d161bee54e19c79a`；隔离 artifact 检查 exit 0；legacy trajectory 与 T00 byte-identical。见 [T02 验证](evidence/t02_20260906T065545Z/VERIFICATION.md) | discovery/refit/score/gate/fallback/covariance、正式 RQ1–RQ4、U01–U14 非 T02 项 `NOT_RUN`；真实 beta、许可、GT/标定来源、多链路数据和 overlay 风险仍在 | T02 `DONE`；停在验收边界，不启动 T03–T13 |
| 2026-09-06 | T02 review 修复 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保留上行及全部 T00/T01 未提交文件和证据 | 修复 recording 级稳定 `obs_id`、range 级原始时间/tag、fixed-beta 键与覆盖准入、零 time-offset/SFUISE grouping 支持边界，以及 LM 有限性/迭代/终止审计；新增真实 bag 裁剪检查脚本和隔离 review 证据 | build exit 0；package tests 68/68；original 627 条与 SFUISE 820 条裁剪观测逐条回查真实 bag 并跨窗口保持 ID；正常 smoke exit 0（126 iterations），1-iteration nonconvergence 与五类非法配置均 FAILED；legacy trajectory 与 review 前 byte-identical。见 [review 验证](evidence/t02_review_20260906T090000Z/VERIFICATION.md) | discovery/refit/score/gate/fallback/covariance、正式 RQ1–RQ4、U01–U14 非 T02 项继续 `NOT_RUN`；外部 beta/许可/GT/数据与 linked-devel 风险未消除 | T02 `IN_PROGRESS`，修复已验证并交回复审；T03–T13 不启动 |
| 2026-09-06 | T02 第二轮复审收尾 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部历史未提交文件及证据 | 修复失败状态中 NaN/±Inf 的 strict-JSON 序列化；loader 时间比较显式禁用相对容差并新增 `+0.5 s` 篡改回归；未改合同或 solver | build exit 0；package tests 68/68；溢出复现 exit 1 且 strict JSON/无有效估计检查通过；original 627 条、SFUISE 820 条真实裁剪身份复核通过；正常 smoke exit 0（126 iterations）。见 [第二轮复审证据](evidence/t02_rereview_20260906T081734Z/VERIFICATION.md) | discovery/refit/score/gate/fallback/covariance、T03–T13、正式 RQ1–RQ4 均 `NOT_RUN`；外部阻塞与 overlay 风险不变 | T02 保持 `IN_PROGRESS`，停在最终复审边界 |
| 2026-09-06 | T02 最终复审收口 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部历史未提交文件及证据 | 本轮指挥/审查会话接受 T02-R01–R07；归档独立复核的汇总、日志、状态与配置，不重复归档大 ledger/轨迹 | 独立复核来源 `/tmp/t02_final_review_95j7sbjy`：normal smoke exit 0；overflow exit 1 且 strict JSON `null`；时间单测 exit 0；original/SFUISE identity exit 0；`+0.5 s` 时间篡改 exit 1。见 [最终复核收口](evidence/t02_final_review_20260906T083514Z/VERIFICATION.md) | 该验收只支持 T02 基础工程；C1–C3 不标 `SUPPORTED`；T03–T13、正式 RQ 均未由此运行 | T02 `DONE`；切换到 T03 |
| 2026-09-06 | T03 NumPy recoverability golden reference | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护并保留 T00–T02 全部未提交文件和历史证据 | 新增纯 NumPy SVD column-space reference、显式列 equilibration/rank/PSD/PD/status 审计、15 项测试及 strict-JSON C++ 对照 fixtures；未修改 C++ estimator | `py_compile` exit 0；`--self-test` exit 0；定向 unittest 15/15；fixture 重生成 byte-identical 且 strict JSON；弱信息 CLI 得到 `eta=1,s=100 m`；冻结 hash 检查 exit 0。见 [T03 证据](evidence/t03_20260906T084306Z/VERIFICATION.md) | 初次 module-style unittest 因 `test/` 非 package exit 1，改用 discover 后通过；Python GTSAM/C++ sparse 对照/T04–T13/正式 RQ `NOT_RUN`；U07 仅 Python 侧完成 | T03 `DONE`；停在验收边界，不进入 T04/T05 |
| 2026-09-06 | T03 review 数值修复 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护并保留全部既有未提交修改和历史证据 | 修复极端 nuisance 列稳定归一化、基于 binary64 `gamma_k` 与 `N/R` 谱尺度的 `N-R` PSD 审计、projector 舍入信息地板和 fixture 结构/数值分层比较；fixtures 扩充为 13 个矩阵；未改 C++ estimator | `py_compile`/self-test exit 0；默认与 Sandybridge unittest 各 22/22；跨 BLAS fixture 容差比较 exit 0；同环境 byte-identical exit 0；极端尺度/完全混淆/弱信息/非有限中间量 CLI 均得到预期状态。见 [review 修复证据](evidence/t03_review_20260906T093843Z/VERIFICATION.md) | 非有限中间量负例按设计 exit 1 且 strict JSON；Python GTSAM/C++ sparse/T04–T13/正式 RQ 均 `NOT_RUN`；无数学定义 amendment | T03 保持 `IN_PROGRESS` 并交回复审；不进入 T04/T05 |
| 2026-09-06 | T03 最终复审收口 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部 T00–T03 未提交产物 | 接受 T03-R01–R04；从 `/tmp/t03_final_review_va5vu4tl` 归档 summary、命令、review cases、独立数学检查与源文件 hash，不重复大型 fixtures | 默认/Sandybridge 各 22/22；跨 BLAS `atol=1e-10,rtol=1e-7`；同环境 byte-identical；100 个独立 Schur case 最大绝对差 `7.105427357601002e-15`。见 [收口证据](evidence/t03_final_review_20260906T110914Z/VERIFICATION.md) | U07 仅 Python projector/Schur `PASS`；Python GTSAM、T05 C++ sparse、C1–C3 正式支持均 `NOT_RUN` | T03 `DONE`；开始 T04，不进入 T05 |
| 2026-09-06 | T04 oracle 分段联合 refit | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保留全部历史未提交文件/证据，新增 T04 源码、配置与隔离证据 | 新增严格 oracle support、`C(s)` Expression factor、无 L1/TV 交替非负联合 refit、完整 result/metadata/trace；runner 增加 debug 分支和三类 CSV，disabled 分支保持 T02 all-range | build exit 0；定向 9/9；package 86/86；4 秒 oracle run exit 0/13 outer；max1 exit 1 且无有效估计导出；T02 all-range 与 legacy trajectory byte-identical；strict JSON/冻结 hash/diff check 通过。见 [T04 证据](evidence/t04_20260906T111500Z/VERIFICATION.md) | `MISSING_CALIBRATION_DEVELOPMENT_ONLY`；T05 sparse score、automatic discovery、gate/fallback/covariance、T05–T13、正式 RQ 全部 `NOT_RUN`；linked-devel warning 保留 | T04 `IN_PROGRESS`；实现已交回复审，独立复审前不进入 T05 |
| 2026-09-06 | T04 review R01–R03 修复 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部 T00–T04 未提交修改和历史证据，新建隔离 review 证据 | 最终联合 `X/V/B` 尺度化梯度与 binary64 舍入审计纳入必要停止条件；segment ID 三类 CSV 完整转义；JSON 全控制字符转义；新增 solver 与 runner 反例回归，同步 METHOD/claim/status | build exit 0；定向 10/10；package 88/88；正常 debug 第 13 轮旧三项通过但驻点失败、第 14 轮全条件通过；max1/非法 manifest 均 exit 1 且无有效估计；CSV 标准解析三表关联通过；T03 默认/Sandybridge 各 22/22；T02/legacy trajectory byte-identical。见 [review 修复证据](evidence/t04_20260906T121005Z/VERIFICATION.md) | 独立复验仍未执行；T05 sparse score、automatic discovery、gate/fallback/covariance、T05–T13、正式 RQ 均 `NOT_RUN`；缺独立 beta 与 linked-devel 风险不变 | T04 仅 `IN_PROGRESS`，修复已准备交回复审；不进入 T05 |
| 2026-09-06 | T04 最终独立复核收口 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；完整保护历史未提交文件与证据 | 归档 `/tmp/t04_rereview_81i5mycw` 全部非 ELF 复核输入、源码、配置、日志、状态、CSV、trace 与 hash；可重建 ELF 只记文件名/大小/hash/编译命令；T04-R01–R03 全部接受 | 独立执行定向 10/10、fixture probe、normal/CSV/JSON/max13/max1 runner checks；历史 package/T03 只检查证据不冒充重跑；T02/legacy 保存轨迹独立 byte compare。见 [最终复核](evidence/t04_final_review_20260906T143023Z/VERIFICATION.md) | `-0.4` 20 轮非收敛与嵌入 NUL reason 截断原样保留；T05 及后续不由本次复核覆盖 | T04 `DONE`；进入 T05 Gate A |
| 2026-09-06 | T05 Gate A sparse 数值原型 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护历史 dirty worktree，新增独立核心/fixture/test 与隔离证据 | 冻结 SVD rank 不变；SparseQR 只做平方根 LS；结构零/重复列保留映射；临界分歧显式 `SPARSE_RANK_UNCERTAIN`；新增 10 个 sparse rank 案例并归档指挥反例 probe | final build exit 0；`test_nlos_recoverability` 4/4；13 个 T03 fixture 在支持域对照通过；初始 binary/target/orthogonality 失败日志均保留。见 [Gate A 证据](evidence/t05_sparse_prototype_20260906T143023Z/VERIFICATION.md) | condition/roundoff 仍 `PENDING_NUMERICAL_PROPOSAL`；不支持域不输出有限 `s`；原 golden 未改 | Gate A 通过后进入 Gate B |
| 2026-09-06 | T05 Gate B 真实图与 runner 接入 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部历史未提交结果，新增唯一 T05 证据目录 | 默认关闭的 score 配置；真实 GTSAM 白化 sparse Jacobian、candidate-excluded 分组图、完整 factor/key/RHS/audit artifacts；U06/U09/U10、group/RHS/short/boundary 定向回归 | final build/runner exit 0；定向 17/17；package 112/112；T03 两环境各 22/22；T04/T02/legacy 回归；strict JSON/CSV/MatrixMarket/hash/diff 审计。真实短输入 `eta=0.81570732433217819,s=0.088141539221235826 m`。见 [Gate B 证据](evidence/t05_20260906T144556Z/VERIFICATION.md) | 仅 oracle-score debug、缺独立 fixed beta；sparse condition/roundoff 待复审；T06、gate/fallback/final covariance、正式 RQ `NOT_RUN` | T05 保持 `IN_PROGRESS`，等待独立复审；不进入 T06 |
| 2026-09-06 | T05 review R01–R05 修复 | `aa6f76a285ca42ae00825b8d8ec3969060f61dd6`；保护全部历史 dirty worktree/证据，新增隔离 review 目录 | 收紧完整 scaled-F rank 支持域；修复弱信息/跨尺度误差审计；加入 graph/metadata/support/plan 完整校验；重建规范 SHA-256 linearization ID；统一 unavailable/`+inf` 输出语义 | final build exit 0；定向 6/6 + 19/19；package 120/120；T03 两环境各 22/22；复核三类 probe 通过；normal exit 0、short 预期 exit 1 且 Stage 2 保留；T04/T02/legacy 轨迹 byte-identical。见 [review 修复证据](evidence/t05_review_fix_20260906T160000Z/VERIFICATION.md) | sparse rank/condition/roundoff 仍 `PENDING_NUMERICAL_PROPOSAL`；仅 oracle debug 且缺独立 beta；T06、gate/fallback/final covariance、正式 RQ 均 `NOT_RUN` | T05 保持 `IN_PROGRESS`，交回独立复审；不进入 T06 |
| 2026-09-07 | T05 最终独立复核收口 | 同一未提交 sprint worktree；先归档复核包及被审查源码，后开始 T06 修改 | 归档 `/tmp/t05_rereview_xu53rhk8` 全部 93 个非 ELF 文件和 12 个审查时源码；可重建 ELF 仅记录来源/大小/hash；R01–R05 全部接受 | 独立 6/6 + 19/19、三类 probe、85/100 支持域对照通过，15/100 明确 unavailable；归档两份 SHA-256 manifest 校验 exit 0。见 [最终收口](evidence/t05_final_review_20260907T022011Z/VERIFICATION.md) | 独立复核未重跑 package 120、T03 双 BLAS、T04/T02/legacy；仅 oracle-debug 支持域，不是性能/通用证明/正式 RQ | T05 `DONE`；进入 T06 |
| 2026-09-07 | T06 automatic support discovery 本地实现 | 同一未提交 sprint worktree；保护全部历史产物，新增独立 T06 证据目录 | 实现冻结非负 L1/TV 交替 Stage 1、scaled-dual ADMM/KKT 审计、link/gap chain、A01 immutable partition、来源中立接口；共享 T04 refit/T05 score；空 partition 仍跑 raw Stage 2 | build exit 0；discovery 10/10、refit 21/21、config 8/8、独立 reference 与双路径 runner contract 通过；package 146/146；T03 双 BLAS各22/22；T04/T05/T02/legacy 兼容。真实自动 run 17 段/13 short/0 boundary、Stage 2 收敛、score unavailable，预期 exit 1。见 [T06 证据](evidence/t06_20260907T025122Z/VERIFICATION.md) | 初次旧测试 ELF/新库 ABI 不一致导致 segfault，强制重建后通过；三次调试失败及 legacy no-master/visualization timeout 均保留；缺 fixed beta，参数待验证，无 T08/正式 RQ | T06 `IN_PROGRESS`，交回独立复审；不进入 T08 |
| 2026-09-07 | T06 review R01–R05 修复 | 同一未提交 sprint worktree；不覆盖 T05 收口归档或首轮 T06 证据，新建隔离 review-fix 目录 | 补齐空 partition raw Stage 2 停止/trace；强制 automatic scoring 与显式科学参数并前置拒绝 oracle；加入 Stage 1 组合尺度步长；稳定 A01 加权均值；完成 SHA-256/context 与 Stage 1 失败留证 | final build exit 0；定向 discovery 14/14、refit 23/23、config 8/8、recoverability 6/6；chain reference 与 runner contract 通过；package 158/158；T03 双 BLAS各22/22；T04/T05/T02 与 legacy 兼容。真实 run 仍 17/13/0、Stage 2 收敛、score unavailable；工程小图产生非空 eligible candidate 和有效 score。见 [review 修复证据](evidence/t06_review_fix_20260907T035905Z/VERIFICATION.md) | 一次 runner-contract 命令漏参数 exit 2、sandbox 内 build 首试 exit 1、legacy timeout 124 均保留；缺 fixed beta、科学参数待 validation；无 T08/正式 RQ | T06 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；不进入后续任务 |
| 2026-09-07 | T06 最终独立复核收口 | 同一未提交 sprint worktree；完整保护 T05 归档和历史 T06 证据，只新增最终复核收口目录与状态文档 | 归档 `/tmp/t06_rereview_3d02_tsu` 全部 193 个非 ELF 文件，包括反例源码、命令、日志、运行结果和 27 个 reviewed_sources；可重建 ELF 只记录来源/大小/SHA-256；T06-R01–R05 全部接受 | 独立 discovery 14/14、refit/scoring 23/23、recoverability 6/6、config 8/8、5x3 chain reference、14/14 preflight probes；535 evidence、27 source/binary、112 T05 archive hashes 匹配；三类完成 discovery 的 snapshot/partition SHA-256 可重建；20 个复审 JSON strict-valid。见 [最终收口](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md) | full package/build、T03 双 BLAS、T04/T05/T02/legacy 未由独立复审重跑；真实 17/13/0 run 仍 3 组 unavailable；旧默认空 fixture 仍 20 轮不收敛；缺 beta/验证参数，无 T08/正式 RQ | T06 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；T07 仍 `NOT_STARTED`，本轮不实现 |
| 2026-09-07 | T07 deterministic multi-link scenario/cache 本地实现 | 同一未提交 sprint worktree；保护 T00–T06 归档，仅新增隔离 T07 证据 | 复用 full original loader 与 paper backend；generator-only step/ramp、完整 base hash/origin/count验证；稳定 obs_id/source ordinals；锁定单位/时间/分组的自包含 cache；分离 injected-component-only truth；runner cache reader 不读 base/recipe/truth/support | targeted build exit 0；config 9/9、paper input 9/9、T07 4/4，package 168/168；12498 行守恒、逐 link 79/77、step byte-identical 重生成；同时移走 base/recipe/truth 后真实 runner 到达 Stage 1；见 [T07 本地证据](evidence/t07_20260907T053301Z/VERIFICATION.md) | 两个真实 smoke 均 exit 1 / Stage 1 `MAX_OUTER_ITERATIONS`，refit/score `NOT_RUN`；runtime seed/parameter snapshot 缺失，总 latent bias `UNKNOWN`；T08–T13/正式 RQ `NOT_RUN` | T07 `IN_PROGRESS/LOCAL_IMPLEMENTATION_PASS_AWAITING_INDEPENDENT_REVIEW`；只交独立 review，不进入 T08 |
| 2026-09-07 | T07 review-fix R01--R03 | 同一未提交 sprint worktree；保护 T00–T07 初次归档并以 3226 files/66633915 bytes/tree SHA-256 基线复核，只新增隔离 review-fix 目录 | 修复 symlink/包含关系物理隔离与 staging cleanup；锁定 UWB group/source ordinal 和 IMU canonical order；checker 核对 cache/recording/interface/unit 状态并在运行期证明 base/recipe/truth 不可达，以非空 discovery trace 证明 Stage 1 | targeted build exit 0；config 9/9、paper input 9/9、T07 7/7；package 174/174；12498 行守恒、79/77、跨 scenario stable IDs、step cache/truth byte-identical；两种真实 cache 均执行 paper backend。见 [review-fix 证据](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md) | `/usr/local/bin/catkin` 首次调用按实际 launch exit 127 保留，正确 `/usr/bin/catkin` 后通过；两个 runner 均 exit 1 / `MAX_OUTER_ITERATIONS`，Stage 2/score `NOT_RUN`；linked-devel 风险、未知总 latent bias 和外部阻塞不变 | T07 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；交回独立复审，不进入 T08 |
| 2026-09-07 | T07 最终独立复审收口 | 同一未提交 sprint worktree；不覆盖 24 个既有证据根，只新增 final-review 归档和更新三份状态文档；未修改源码或冻结文件 | T07-R01--R03 均 `REVIEW_ACCEPTED`；完整归档 `/tmp/t07-independent-review.JKkZk0` 的 15 个 isolation/run 文件、原始 runner 流、命令/退出码、源码/二进制 hash 与 ldd/overlay provenance | 独立结论确认 config 9/9、paper input 9/9、T07 7/7、package 174/174、12498 行守恒、79/77 和 stable IDs；isolation 24 次均不可达、恢复 identity、Stage 1 trace 50 行；runner 原样 exit 1/`MAX_OUTER_ITERATIONS`。见 [最终收口](evidence/t07_final_review_20260907T070102Z/VERIFICATION.md) | base 总 latent bias `UNKNOWN`；runtime seed/parameter provenance 不完整；两个 smoke 均失败，Stage 2/refit/score 与 formal RQ、T08 gate/fallback/final covariance 均 `NOT_RUN`；双目录发布不声明 SIGKILL/power-loss crash atomicity | T07 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；T08--T13 保持 `NOT_STARTED`，本次停止 |
| 2026-09-07 | T07 final-closeout 文档/证据修复 | 同一未提交 sprint worktree；旧 final-review 及全部既有 evidence root 只读；仅修改 STATUS/claim ledger 并新增隔离 closeout-fix evidence | 修正 C3 未勾选 checklist 的过期“待独立 review”；为旧 independent-review 15 文件新增相对路径 SHA-256/size manifest 与仓库内自包含 checker；旧 checker 明确保留为 point-in-time | checker 验证精确 15 文件/hash/size、全部归档 JSON strict-valid、50 行 Stage-1 trace、24 次 isolation、runner exit 1/`MAX_OUTER_ITERATIONS`、Stage 2/refit/score `NOT_RUN`；固定 24-root 历史基线、旧 final-review 树和产品树一致。见 [closeout-fix 证据](evidence/t07_closeout_fix_20260907T072733Z/VERIFICATION.md) | 未重跑 generator/tests/paper backend；T07 功能验收结论不变；C1--C3 不为 `SUPPORTED`；正式 RQ 与 T08 gate/fallback/final covariance `NOT_RUN` | T07 保持 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；T08--T13 保持 `NOT_STARTED`，本次停止 |
| 2026-09-07 | T08 冻结决策、最终联合推断与一次 fallback 本地实现 | 同一未提交 sprint worktree；完整保护 T00–T07 源码/产物/证据，仅新增隔离 T08 evidence 与 T08 源码、测试、development config | 新增来源中立 `InferenceResult`/Stage 4 engine；完整候选集感知的 final graph 重建；原子 gate/稳定原因码；decision/final 分离复审计；一次全抑制 fallback；同图 trajectory/bias/residual/covariance exporter；严格 CSV/JSON 与 runner contract | 最终 build、tests target、diff check 均 exit 0；CTest 17/17；T08 12/12；runner exit 0/`OK`，74 factors/25 Values、8 accepted/56 reference、0 pseudo、25 covariance blocks；6 JSON/13 CSV strict。见 [T08 本地证据](evidence/t08_20260907T083437Z/VERIFICATION.md) | 初次缺测试源 build、错误 binary path、writer compile 和 stale test-ELF failures 均登记后修复；T07 step/ramp 不重跑且仍为既有 Stage 1 failure；gate 仍 `PENDING_VALIDATION`；T09–T13/正式 RQ/性能/独立复审 `NOT_RUN`；C1–C3 非 `SUPPORTED` | T08 `IN_PROGRESS/LOCAL_IMPLEMENTATION_PASS_AWAITING_INDEPENDENT_REVIEW`；停止并交回复审，不进入 T09 |
| 2026-09-07 | T08 review-fix R01–R03 | 同一 dirty sprint worktree；先归档 reviewed source/binary hash，保护全部 T00–T08 历史 evidence；只修改 T08 inference/IO/runner/tests 与状态说明 | 分离 Stage 2/recovery/fallback/final 诊断且保留失败 recovery score；修正 runner wall/Stage-4 分项计时；建立绑定实际 graph/Values 数值和输入配置上下文的 canonical content identity；加入实际 final re-score 触发 fallback、双 group 和 identity collision 反例 | build/tests/diff exit 0；T08 18/18；CTest 17/17；runner contract exit 0（含一轮 Stage-2 failure 外部计时）；normal runner exit 0/`OK`；外部/runner/Stage-4 为 `9.474669116/9.463173210/0.007065469 s`；8 JSON/21 CSV strict。见 [review-fix 证据](evidence/t08_review_fix_20260907T092037Z/VERIFICATION.md) | T07 两 smoke 原样 Stage 1 `MAX_OUTER_ITERATIONS`，T08 `NOT_RUN`；gate `PENDING_VALIDATION`；正式 RQ、T09–T13、性能/RSS、held-out 与独立复审 `NOT_RUN`；C1–C3 非 `SUPPORTED` | T08 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；停止并交回独立复审，不进入 T09 |
| 2026-09-07 | T08-R02 最后一轮异常出口计时修复 | 同一 dirty sprint worktree；源码修改前完整归档独立复审 881 文件及 reviewed source/binary hash；历史 evidence 只读 | 仅把 Stage timing 提升到异常出口可见范围，并为通用 exception/LmFailure 写入统一 elapsed semantics/value 与 Stage 1–4 nullable 字段；runner contract 新增非法 manifest 和 LmFailure，未改 R01/R03/solver/gate/fallback | build/tests exit 0；runner contract exit 0；非法 manifest 预期 exit 1，外部/runner 差 `0.013141745 s`、原因/阶段不变、Stage 1–4 null、无估计；normal/Stage-2 failure/LmFailure 独立 runs 语义一致；CTest 17/17、diff check exit 0。见 [R02 证据](evidence/t08_r02_review_fix_20260907T104154Z/VERIFICATION.md) | 独立复审 R01/R03 scoped accepted；R02 修复尚待独立复审。T07 两 smoke、gate validation、正式 RQ、T09–T13、性能/RSS/held-out 均未运行；C1–C3 非 `SUPPORTED` | T08 保持 `IN_PROGRESS`；R02 `LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`，停止，不进入 T09 |
| 2026-09-07 | T08 最终独立复审收口 | 同一 dirty sprint worktree；方法源码只读；归档本轮 128 个新增复核文件和 6 个被审查源码快照；`archive_copy` 仅记录与既有 R02 evidence 的 976 文件迁移同一性 | 接受 T08 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；R01/R03 保留限定接受，R02 为 `REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`；只更新 STATUS、METHOD 实现状态与 claim ledger | 独立 focused GTest 18/18、runner contract、迁移 archive checker、7/7 current source/binary、4/4 R01/R03 source、2/2 frozen hash、diff check 均 exit 0；非法 manifest 预期 exit 1，外部/runner 差 `0.012490457 s`，未执行阶段 null、无有效估计。见 [最终收口](evidence/t08_final_review_20260907T121510Z/VERIFICATION.md) | 本轮未重跑 package build、完整 CTest 17/17、T07 step/ramp、正式 RQ、held-out validation、性能/RSS；gate 仍 `PENDING_VALIDATION`，标定/数据 provenance 未闭合，C1–C3 非 `SUPPORTED` | T08 `DONE`；下一任务为 T09 实施规划，T09 尚未实现 |
| 2026-09-07 | T09 统一基线、缓存、批量执行与评估本地实现 | 同一 dirty sprint worktree；保护 T00–T08 源码、结果与 evidence，只新增 T09 工程、development config 和隔离证据 | 实现 canonical registry/独立 baseline、Stage1RegularizedResult、live common/final/cache identities、automatic/fixed cache 隔离、policy diagnostics、batch DAG、GT exporter、统一 evaluator 与 failure/NA 分母；不改 T08 inference ID 语义 | catkin build exit 0；最终完整 CTest 21/21（242.99 s）；batch 19/19 terminal，实际 common identity 跨方法一致、cache ID 跨 C++/Python 一致且无 comparability failure。T07 baselines 成功，step/ramp automatic 仍 `MAX_OUTER_ITERATIONS` 且无下游产物；T06 AUTO partial cache 与 fixed DEBUG cache 隔离。见 [T09 本地证据](evidence/t09_20260907T143701Z/VERIFICATION.md) | GT/frame/fixed-beta 与正式数据 provenance 未闭合；T07 automatic E2E 仍阻塞；fixed debug 不算自动闭环；T10 lock、held-out、正式 RQ/论文指标均 `NOT_RUN`；尚待独立复审 | T09 `IN_PROGRESS/IMPLEMENTED_DEVELOPMENT_ENGINEERING_SCOPE / AUTOMATIC_E2E_ACCEPTANCE_BLOCKED_BY_T07_STAGE1`；不进入 T10 |
| 2026-09-08 | T09 independent-review R01–R08 修复 | 同一 dirty sprint worktree；先加入可在旧实现失败的回归，保护全部原 evidence/用户修改，只新增 review-fix 证据 | 严格 mode/execution/path 与 development admission；完整 typed Stage-2 cache、无 Stage1/2 optimizer 的 final replay、shared producer/operating point；修复 evaluator artifact graph、SE(3) RPE、comparability quarantine 和稳定 ABI hash；不改 T08 identity/科学参数 | catkin build exit 0；focused 9/9；完整 CTest 24/24（252.70 s）；真实 1 producer→同一 cache→2 diagnostic+2 final 全部 COMPLETE 且 T08 export VERIFIED；归档 cache 又由第二个 scheduler 进程外部复用成功；19/19 development terminal，T07 automatic failure/downstream absence 保持。见 [review-fix 证据](evidence/t09_review_fix_20260907T164311Z/VERIFICATION.md) | 修复尚待独立复审；T07 automatic E2E 仍 Stage1 阻塞；fixed DEBUG 不是 automatic；validation/test、locked gate、正式 RQ、数据角色/claim 升级、T10–T13 均未执行 | T09 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；停止并等待独立复审，不进入 T10 |
| 2026-09-08 | T09 第二轮 rereview R01–R04 定向修复 | 同一 dirty sprint worktree；完整保留原 evidence 和 R05–R08 已接受行为；先落地可在复审版本失败的回归 | lossless threshold/final config request identity；cache producer semantic config v2 与双层不兼容拒绝；failure-safe evaluator、sealed hash/factor audit、mode-point-path/run-unit 分层；decision/final bias map 分离；replay 统一成功/失败 timing | catkin build、tests target exit 0；focused 10/10；最终完整 CTest 25/25（1034.62 s）；真实 AUTO 1 producer→同一 cache→2 diagnostic+2 adjacent final 5/5 terminal，两个 request 不同；lambda stale cache scheduler/direct runner 分别预期 exit 2/1；真实 final tamper evaluator 预期 exit 2。见 [rereview-fix 证据](evidence/t09_rereview_fix_20260907T180000Z/VERIFICATION.md) | T07 automatic E2E 仍 Stage1 阻塞；未修改科学参数/T08 identity；validation/test、locked gate、正式 RQ、数据角色/claim 升级、T10–T13 均未执行；尚待再次独立复审 | T09 保持 `IN_PROGRESS/REVIEW_FIX_LOCAL_PASS_AWAITING_INDEPENDENT_REREVIEW`；停止并交回独立复审，不进入 T10 |
| 2026-09-08 | T09 最终独立复审与证据收口 | 同一 dirty sprint worktree；产品源码/科学配置/测试及历史 evidence 只读；只新增独立复审包并修正事实摘要 | 接受 R01-A/B、R02-A、R03-A/C、R04-A、R05–R08 的 development-engineering 范围；复核并修复历史 `/tmp` 依赖的归档对应关系；发现 R03-B audit-domain 非双射缺陷并保留最小反例 | 被审 source/runner/core identity 匹配；focused 9/9；既有 AUTO cache→两个独立 final 均 exit 0/不同 request/同 parent/Stage1–2 NOT_RUN/VERIFIED；scheduler/direct stale-config exit 2/1；普通 tamper exit 2；failure/bias probes exit 0；重封存额外 audit 反例 evaluator **意外 exit 0**、retained `42/627`。见 [最终独立复审](evidence/t09_final_review_20260908T050201Z/VERIFICATION.md) | 完整 CTest 25、package build、19-cell batch、T07 step/ramp、formal RQ、validation/test、locked gate、T10–T13 均 `NOT_RUN`；automatic accepted-candidate 科学证据仍缺 | T09 `IN_PROGRESS/INDEPENDENT_REREVIEW_CHANGES_REQUESTED_R03B_AUDIT_DOMAIN`；只修 evaluator audit-domain 双射并定向复审，不进入 T10 |
| 2026-09-08 | T09 R03-B 最小本地修复 | 同一 dirty sprint worktree；修改前归档 `/tmp/t09-review-d0t4h1ne` 324 文件；保护全部历史 evidence 与无关修改 | 仅修改 evaluator 与 artifact-graph 回归：parent ledger identity/local-copy 检查、ledger/mask/audit 三层域一致性、三类 classification/flags/count 严格语义、validated numerator/denominator 单一路径 | 真实 final exit 0、`41/627`；orphan/missing/duplicate audit、错误 classification、矛盾 expected count/flags、本地 ledger 篡改均 exit 2 且无 evaluation；failure/bias probe exit 0；focused CTest 9/9；归档校验见 [R03-B 证据](evidence/t09_r03b_fix_20260908T053929Z/VERIFICATION.md) | 未构建/修改 C++；完整 CTest 25、19-cell batch、T07 step/ramp、formal RQ、validation/test、locked gate、T10–T13 `NOT_RUN`；automatic accepted-candidate 科学证据仍缺 | T09 保持 `IN_PROGRESS/R03B_LOCAL_FIX_PASS_AWAITING_INDEPENDENT_REREVIEW`；停止并交回独立复审，不进入 T10 |
| 2026-09-08 | T09 R03-B parent-cache 内容身份最小修复 | 同一 dirty sprint worktree；修改前归档 `/tmp/t09-r03b-rereview-r9j21a6q` 349 文件；保护历史 evidence 和无关修改 | evaluator 直接复用 scheduler canonical v2 Stage-2 cache ID，重算并核对 manifest/cell，final 只绑定已验证 parent ID；artifact-graph fixture 升级为完整 v2 并新增两个身份反例 | 真实 final exit 0、`41/627`；过期 cache ID 和重算新 ID 替换旧 final 均 exit 2、无 evaluation；七个旧负例及 failure/bias probe 保持；focused CTest 9/9。见 [identity 修复证据](evidence/t09_r03b_parent_identity_fix_20260908T062741Z/VERIFICATION.md) | 未构建/修改 C++；当前 runner 不存在且 `NOT_RUN`；完整 CTest 25、19-cell batch、T07 step/ramp、formal RQ、validation/test、locked gate、T10–T13 `NOT_RUN`；automatic accepted-candidate 科学证据仍缺 | T09 保持 `IN_PROGRESS/R03B_PARENT_CACHE_IDENTITY_LOCAL_FIX_PASS_AWAITING_INDEPENDENT_REREVIEW`；停止并交回独立复审，不进入 T10 |
| 2026-09-08 | T09 最终独立复审文档收口 | 产品源码、科学配置、测试与历史 evidence 只读；原样归档 `/tmp/t09-final-rereview-bojvs6no` 743 文件，仅更新 STATUS/CLAIM_EVIDENCE 与新增收口证据 | 最终复审接受 R03-B；与此前已接受 R01–R08 合并关闭 T09 development-engineering 范围，不再启动 T09 修复轮次 | 原包/归档 743 路径内容一致；原 `MANIFEST.sha256` 覆盖 742 文件且全部 OK；真实 final `41/627`、两 fresh C++ consumer、focused 9/9、身份/完整性与 failure probes 由归档复审接受。见 [最终收口](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md) | package build、完整 CTest 25、19-cell、T07 step/ramp、新 discovery、正式 RQ、validation/test、locked gate、T10–T13 本轮 `NOT_RUN`；外部 provenance 未闭合，automatic accepted-candidate 科学证据仍缺，C1–C3 非 `SUPPORTED` | T09 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE / AUTOMATIC_E2E_BLOCKED_BY_T07_STAGE1`；下一任务仅为 T07 step/ramp Stage 1 `MAX_OUTER_ITERATIONS` 限定诊断与实施规划，不自动进入 T10 |
| 2026-09-08 | T10 前置检查与最小诊断 | 同一 dirty sprint worktree；保护全部既有修改和历史 evidence；只新增 [`T10_READINESS.md`](T10_READINESS.md)、独立 precheck evidence，并更新当前 STATUS/claim ledger | 分离真实 AUTO、engineering fixture 与 fixed DEBUG；审计 truth/split/calibration/scheduler/budget 准入；逐项解析 step/ramp 50 轮 objective/step/chain/navigation 与 inner LM 边界；提交最小诊断增强和 validation admission 方案 | read-only trace/source/hash checker与文档检查实际 exit 0，见 [T10 precheck 证据](evidence/t10_precheck_20260908T073803Z/VERIFICATION.md)。两 run objective/step/navigation 0/50、chain 50/50；source-compatible，故限定重跑 `NOT_RUN`（`0/2`） | 新 discovery、Stage2/score/final、package build、CTest、19-cell、validation/sweep/gate lock/test、正式 RQ/U14、T11–T13、ATE/RPE/runtime/RSS 全部 `NOT_RUN`；inner LM trigger、实现 correctness 与数据/模型归因仍有 `UNKNOWN` | T10 保持 `IN_PROGRESS/PRECHECK_ONLY`，提交指挥官 review；先审查诊断增强、split/provenance、预算和待定数值，不直接进入 validation/test |
| 2026-09-08 | T10 Stage 1 诊断增强与原参数定向复现 | 同一 dirty sprint worktree；保护已有修改与历史归档；只改只读诊断/序列化/定向测试并新增隔离 evidence | 记录 actual linked-GTSAM LM errors/tolerances/predicates 与既有 inner/lambda；同 `lm.values` 记录 old-bias pre-chain 和 updated-bias post-chain stationarity；追加兼容 trace/failure 字段与诊断耗时 | affected build exit 0；discovery 17/17、refit 23/23、runner contract、GTSAM probe 均通过；step/ramp 原参数各一次均预期 exit 1/50 轮；历史 18 列最大差 0；100/100 次 LM 只由 relative predicate 停止，pre-chain 0/50、post-chain 0/50 | Stage2/refit/score、validation admission/执行、新数据、sweep/gate lock/test、19-cell、正式 RQ、T11–T13 均 `NOT_RUN`；更多 outer budget 与数据/模型归因 `UNKNOWN`；C1–C3 不升级 | T10 保持 `IN_PROGRESS/DIAGNOSTIC_IMPLEMENTATION`；提交 [证据](evidence/t10_stage1_diagnostics_20260908T081328Z/VERIFICATION.md) 与 [下一方案](evidence/t10_stage1_diagnostics_20260908T081328Z/NEXT_IMPLEMENTATION.md)，等待指挥官 review |
| 2026-09-08 | T10 A02 条件导航驻点策略实现与限定开发对照 | 同一 dirty sprint worktree；保护全部既有改动与封存证据；A02 在编码前登记，只改 solver/config/discovery/runner 诊断与定向测试，新增独立 evidence | 实现默认关闭的 `GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1`；同一 optimizer/原 inner budget 内以 actual GTSAM check 加既有 stationarity audit 资格；policy 绑定 solver/support/cache identity、effective config、trace/failure；默认与 refit 兼容 | 最终 build exit 0；config 11/11、discovery 23/23、refit 23/23、runner contract 含 cap strict JSON 均通过；step/ramp 各一次 child exit 1、50 轮，conditional qualification 全轮通过但 Stage 2 未运行；inner 总数 115→538、118→513，Stage 1 wall 约 3.87/3.93 倍。见 [A02 证据](evidence/t10_conditional_stationarity_policy_20260908T091614Z/VERIFICATION.md) | 首轮测试编译及一次 evidence-only identity 引号命令失败均保留；validation admission/新数据/sweep/gate lock/test/19-cell/正式 RQ/T11–T13 `NOT_RUN`；更多 outer budget 与数据/模型归因 `UNKNOWN`；C1–C3 不升级 | T10 保持 `IN_PROGRESS/CONDITIONAL_STATIONARITY_POLICY_LOCAL_PASS_AWAITING_REVIEW`；等待指挥官 review，下一最小动作仅建议 block-coupling consistency attribution |
| 2026-09-08 | T10 A02-R01 最小修复与 A03 限定 outer-budget 诊断 | 同一 dirty sprint worktree；先完整归档 `/tmp/t10-a02-independent-review-vcus5lg1` 的 84 个非 ELF 文件并复现反例；保护历史 evidence/默认配置/T07/T09 | 两入口统一拒绝 A02 内外层 tolerance、roundoff、五类尺度冲突；默认策略与 standalone LM 兼容。A03 在运行前登记，只复制两份 A02 配置并把 outer cap `50→500`，每输入一次、整进程 120 秒 | build exit 0；config 11/11、discovery 26/26、refit/scoring 23/23、runner contract exit 0；两条前 50 轮非计时 trace 最大差 0，身份分离。step/ramp 分别完成 120/126 轮，在 outer 121/127 的原 inner cap 返回 `CONDITIONAL_LM_STATIONARITY_NOT_REACHED`；无 timeout，Stage 2 `NOT_RUN`。见 [完整证据](evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/VERIFICATION.md) | 没有重复、加 cap、改 inner/tolerance/block/gate/default；eligible 与科学 truth/provenance 均不可评价；validation/sweep/gate lock/test/RQ/T11–T13/正式指标 `NOT_RUN`，C1–C3 不升级 | T10 保持 `IN_PROGRESS/A02_R01_LOCAL_FIX_PASS_AWAITING_INDEPENDENT_REVIEW / A03_DEVELOPMENT_BUDGET_DIAGNOSTIC_COMPLETE`；下一步仅独立复审修复与只读分析 inner stationarity floor/coupling，不继续预算扩张 |
| 2026-09-08 | T10 step outer 121 conditional LM 停滞定位 | 同一 dirty sprint worktree；先归档 `/tmp/t10-a03-independent-review-gd5ed4b1`；保护历史 evidence/默认配置/T07/T09；observer 默认关闭并保留旧 ABI overload | 增加 development-only branch/Values/factor/FD observer，不改 solver 决策；核对实际 linked GTSAM 4.2a5 `tryLambda/iterate`，捕获 outer 121 每次调用；提出但未实施 A02 no-progress fail-fast | build/rebuild exit 0；config 11/11、discovery 27/27、refit/scoring 23/23、T06 runner contract 通过；唯一 A03 step replay child exit 1、2.506007 s、无 timeout。50 calls=1 accepted+49 small-cost no-update，0 lambda-search rejection；最大梯度 FD 13 点中连续 8 点一致；前 120 轮 79 个非计时字段精确相同。见 [完整证据](evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md) | ramp/retry、参数/cap/tolerance/solver 改动、Stage 2/score、validation/sweep/gate lock/test/RQ/T11–T13 均 `NOT_RUN`；完整 Hessian conditioning/全坐标 FD 与数据/模型归因仍未知；C1–C3 不升级 | `REVIEW_ACCEPTED_CONDITIONAL_LM_STALL_DIAGNOSTIC_SCOPE`；独立复审接受实际分支证据、FD 一致性和未知项边界，不接受 solver 修复、默认迁移或科学 claim |
| 2026-09-08 | T10 A04 fixed-checkpoint LM 恢复可行性对照 | 同一 dirty sprint worktree；停滞 review 原样归档；A04 运行前登记和预声明；历史 evidence/默认配置/T07/T09 保留 | observer 显式旁路在 authoritative stall 后复制同一 graph/Values/lambda；A 原行为，B 仅内部 rel tol 置零继续 bounded lambda search；external generic/stationarity/acceptance/upper/cap 不变，shadow 不回写 | build exit 0；discovery 28/28、config 11/11、重链接后 refit/scoring 23/23、T06 runner contract 通过；唯一 step replay child exit 1、无 timeout。A 50 calls 0 accepted；B 2 calls/3 trials、2 strict descents、最终 gradient `3.4517e-7` 通过。前 120 轮 79 个非计时字段及 authoritative calls 与上轮精确相同。见 [完整证据](evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/VERIFICATION.md) | B 内部 `relativeErrorTol 1e-6→0` 是明确 solver 数值语义变化；原 estimator 仍 Stage 1 failed，Stage 2/eligible `NOT_RUN/NOT_EVALUATED`；ramp/validation/gate/test/RQ/T11–T13 `NOT_RUN`，C1–C3 不升级 | T10 `IN_PROGRESS/A04_FIXED_CHECKPOINT_LM_RECOVERY_DIAGNOSTIC_COMPLETE`；下一步只复审并另行裁定版本化 V2，不默认迁移或追加运行 |
| 2026-09-08--09 | T10 A05 默认关闭 V2 集成与两条定向运行 | 同一 dirty sprint worktree；先原样归档 A04 独立复审；A05 在实现/运行前登记；保护历史 evidence、默认配置及 T07/T09 状态 | 增加版本化 V2：optimizer 内部 rel=0、external generic rel/abs 不变；保留接受/upper/cap/stationarity/outer AND；两入口七字段一致性；identity/effective config/trace/failure 版本化；lambda 上界耗尽显式失败 | affected build/relink exit 0；最终 discovery 30/30、config 11/11、refit 23/23、inference 18/18、methods 5/5、cache 7/7、runner/scheduler checks 通过。step/ramp 配置仅 policy 改变且各运行一次：旧 outer 121/127 均恢复，随后 outer 123/168 在 lambda `1e5` 耗尽；child exit 1，0.820071/0.868272 s，无 timeout。见 [A05 证据](evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/VERIFICATION.md) | 两条均 Stage 1 failed；Stage 2、eligible、validation/gate/test/RQ/T11--T13 均未进入；没有 actual cache，truth/provenance 不足；不迁移默认、不扩预算，C1--C3 不升级 | T10 保持 `IN_PROGRESS/A05_V2_LOCAL_INTEGRATION_AND_DIRECTED_RUNS_COMPLETE_AWAITING_INDEPENDENT_REVIEW`；下一步只独立复审 A05 |
| 2026-09-09 | T10 A05 review-fix（`T10-A05-RF1`、`T10-A05-RF2`） | 同一 dirty sprint worktree；校验并原样归档独立复审包 `149ac1f3...d816`；修改/构建前保存源码、runner/core/GTSAM 当前身份；历史 A05 evidence/manifest 均未改 | RF1 删除 catch 的虚构 1 次 trial/rejection，仅累计 linked state 已确认 transition，并增加 `NOT_EXECUTED/COMPLETE/INCOMPLETE_EXCEPTION_DURING_ITERATE` 三态与 CSV/JSON/schema v2；RF2 以 evidence-local 工具绑定本轮 runner、focused tests、core、GTSAM realpath/SHA-256/build ID/raw+normalized ldd | affected build/relink exit 0；discovery 32/32、config 11/11、refit 23/23、inference 18/18、methods 5/5、cache 7/7、scheduler/runner contract、前后带断言 probe 均 exit 0；测试前后关键库身份一致。见 [review-fix 证据](evidence/t10_a05_review_fix_20260909T040832Z/VERIFICATION.md) | pre-trial 实际 0 已由 fixture 调用断言；trial 内异常总数保持 `INCOMPLETE`；V2 最优点 exhaustion 边界不变；A05 历史 step/ramp 共享库身份缺口不能事后补证。step/ramp、validation/sweep/gate/test/RQ/T11--T13 `NOT_RUN`，C1--C3 不升级；无 amendment | 本轮指挥会话已接受 `REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE`，限定关闭 RF1/RF2/RF1-N01；不是外部复审报告 |
| 2026-09-09 | T10 A06 新失败 checkpoint 定向诊断 | 同一 dirty sprint worktree；A06 在实现/运行前登记；复用 A05 配置且逐字节一致；仅允许 step outer 123/ramp outer 168 各一次、120 秒、无 retry | observer-only 增加 authoritative final Values 的原驻点审计、binary64 max-digits linked TRYLAMBDA 和 call markers；没有新增此类 solve、不参与决策，不改 policy/参数/上限/规则；既有 `InspectFirstLinkedLmTry` 含 diagnostic solve | 必要目标 build/relink exit 0，discovery 34/34；两 estimator child 均预期 exit 1、无 timeout；运行身份前后稳定。两点不驻点，终止 trials predicted 正/actual 负，fidelity 负或不可用；FD 主导坐标 7/13、8/13；历史 86 公共非计时字段及终态精确一致。见 [A06 证据](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md) | Stage 1 仍 0/2，Stage 2/cache/estimate `NOT_RUN`；`64*epsilon` 不是完整浮点误差上界；历史 A05 库身份缺口与最优点边界保留；C1-C3 不升级 | 本轮指挥会话 `REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE`；实际核验 manifest 247/247、源码 4/4、身份 9/9、discovery 34/34、历史 122/167x86 和终止 11/10；未重建、重跑或修改，不虚构外部 REVIEW |
| 2026-09-09 | T10 A07 actual LM 方向有限诊断 | 同一 dirty sprint worktree；A07 在实现/真实运行前登记；A06 配置逐字节复用；step outer 123/ramp outer 168 各一次、120 秒、无 retry | 捕获 authoritative linked `TRYDELTA`，事后在同一 live graph/base Values 重构 tentative Values、线性/非线性目标、逐 factor error 和预登记方向 FD；不改 `/usr/local`、不新增 solve、不改 optimizer 决策 | 首次独立 probe 因 standalone 链接环境 SIGSEGV、一次链接含不兼容 unstable 库失败，均保留；工程实际链接 probe exit 0。最终 build/relink exit 0，discovery 35/35、CLI contract exit 0；真实 step/ramp 预期 exit 1、无 timeout；22/22 trial 的 24 key/120 维完整，6 个方向的 18 个 FD 全部显示 `g^T u` mismatch；逐 factor fsum/long-double 与 direct objective 同为上升；历史 122/167x86 与终态不变 | UWB expression 贡献最大绝对 factor 变化，但与 pose prior/IMU 强抵消，尚缺逐 factor analytic-direction vs FD，不能指定错误 factor。Stage 2/validation/sweep/gate/test/RQ/T11--T13 `NOT_RUN`，C1--C3 不升级 | 本轮指挥会话 `REVIEW_ACCEPTED_A07_BOUNDED_EXACT_DIRECTION_DIAGNOSTIC_SCOPE`；未重跑、重建或改仓库，不虚构外部 REVIEW |
| 2026-09-09 | T10 A08 冻结方向逐 factor 定位与条件 Jacobian correctness fix | 同一 dirty sprint worktree；A08 在源码修改/新运行前登记；复用 A07 两 checkpoint、六 actual delta、三个 FD 步长；旧 evidence 与用户结果只读 | observer 保留 1314 个逐 factor analytic/FD 点；证实 linked `PriorFactor<Pose3>` identity Jacobian 与其 `-Local(x,prior)` residual/retract 不一致；新增 paper-only factor使用 `-D_x Local`，显式替换唯一 prior并版本化 producer identity；legacy、objective 和 solver 策略不变 | required build/relink exit 0；prior probe pre 预期 exit 1/post exit 0；focused discovery 37/37、UWB 6/6、refit 23/23、inference 18/18、methods 5/5、cache 7/7、config 11/11、CLI 通过。修复后 factor `1314/1314`、图级 `15/18`；step/ramp Stage 1 在 outer 178/173 收敛，Stage 2 均 `MAX_REFIT_ITERATIONS=50`、exit 1、无有效估计 | 三个 `h=1e-6` 图级残差来源 `UNKNOWN`，不以 `64*epsilon` 排除浮点误差；无效 raw-graph 恢复尝试原样保留。正式 validation/sweep/gate lock/test/RQ/T11--T13 `NOT_RUN`，C1--C3 不升级 | 本轮指挥会话 `REVIEW_ACCEPTED_A08_PAPER_POSE_PRIOR_JACOBIAN_FIX_SCOPE`；不是外部 REVIEW，未虚构指挥重建、重跑或修改仓库 |
| 2026-09-09 | T10 A09 Stage 2 有界 refit-budget 诊断 | 同一 dirty sprint worktree；A08 指挥会话验收与 A09 在运行前登记；只复制 A08 两份隔离 development 配置并把 `max_refit_iterations 50→200`；历史 evidence/用户结果只读 | 未改源码、默认配置、solver/停止/容差/LM/dependency；step/ramp 各唯一进程、120 秒上限、无 retry/observer；运行前后绑定 runner/core/GTSAM，离线用既有 T09 publisher 封存已生成 payload | preflight exit 0；两进程实际 exit 1、无 timeout。Stage 1 `178/173 x 87`、Stage 2 前 50 轮 `50 x 22` 公共非计时字段与 A08 精确一致；Stage 2 outer 66/124 首次四项 AND 全过并导出 estimate。两个 cache 均 `CONVERGED/COMPLETE_WITH_SCORE_UNAVAILABLE`，身份前后与 A08 3/3 一致 | step/ramp 为 24/27 段、17/21 short、0/1 boundary、1/3 groups，eligible group 0/0、valid score 0/0；正式 validation/sweep/gate/test/RQ/T11--T13 `NOT_RUN`，C1--C3 不升级 | 本轮指挥会话接受 `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`；不是外部 REVIEW；随后按授权执行 A10 |

| 2026-09-09 | T10-A10 自包含生成链路与有限 development pilot | 保护原 dirty sprint worktree 和历史输入/结果；无 C++/默认/solver/依赖修改 | 新增生成器、数值/随机源/腐败反例 checker、9-cell 编排和审计、独立 raw/truth、具体 validation 准入与 split 提案 | 生成/一致性最终 exit0；首轮 join checker exit1 保留；反例5/5；9/9 estimator exit1、无 timeout，共9.750853s；实际 raw/cache/init/maps/动态库身份与 truth 隔离审计通过。见 [证据](evidence/t10_a10_synthetic_pilot_20260909T115705Z/VERIFICATION.md) | 9/9 首次 conditional LM 未达驻点；Stage2/score/final NOT_RUN；support不可比较；生成成功不等于估计通过；A08图级15/18限制、正式validation/test/gate/claim NOT_RUN | T10继续IN_PROGRESS，A10 LOCAL_DELIVERY_COMPLETE_AWAITING_REVIEW；预算关闭，仅交审交付/方案，不追加运行 |

| 2026-09-10 | T10-A19-R02 完整自动评分与有限比较 | 同一 dirty sprint worktree；保护 A19/A19-R01 ticket、失败、结果和全部用户改动；R02 新协议/身份/attempt 隔离 | 新增 launcher/preflight/authorization/final audit 工具；修复预检对动态库 symlink 的身份比较；复用身份匹配的 R01 runner/core 与 17/17 工程门；唯一 fresh 自动运行按首次算法失败停止 | estimator-free preflight 首次 exit1、修正后 9/9 exit0；唯一 estimator exit1/79.63632955400004s/RSS35828KiB：Stage1 90 outer、2段/32候选收敛并冻结，Stage2 outer15 `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`，83 calls/170 trials；审计16/16 exit0。见 [R02证据](evidence/t10_a19_r02_auto_score_compare_20260910T041247Z/VERIFICATION.md) | score、Stage2 final graph/Values、五策略 decision/final/fallback、truth evaluator、ATE/bias/risk/coverage、validation/test/gate lock 均 `NOT_RUN/UNAVAILABLE`；无 attempt2；C1--C3不升级 | T10保持`IN_PROGRESS/A19_R02_STOPPED_AT_FIRST_ALGORITHMIC_FAILURE`；唯一主阻塞为冻结设置下 Stage2 conditional lambda exhaustion；本轮停止，不扩参数/预算或新 solver 研究 |

| 2026-09-10 | T10-A19-R08 有限 synthetic validation 固定矩阵 | 先登记R08协议、两条保留base/seed、6场景顺序、固定P1/工作点/预算/truth边界；保护历史dirty worktree、失败与ticket | 新增validation-only生成/role身份/runner/evaluator/串行ledger；复用同一C++/GTSAM estimator及R07 certified策略；共享空partition dispatch缺陷修复后只继续未运行行 | CMake ABI exit0；discovery 2/2、refit27/27、inference20/20；12条各至多一次：8 SCORED、3 FAILED、1 INTERRUPTED；冻结1320项hash复核0 mismatch；8 evaluator exit0、estimator truth open0。见[R08证据](evidence/t10_a19_r08_validation_compare_20260910T115109Z/VERIFICATION.md) | 三gate在8条均同决定；7条structured有效配对RMSE 1好6差、P95 0好7差；LOS无有效final所以代价UNAVAILABLE；test/held-out/gate lock/C1--C3升级均未发生 | T10保持IN_PROGRESS；下一唯一阻塞为validation零候选LOS support/refit/score/all_range一致性，再进入有限cache选择与held-out |

## Scope amendments

任何会改变冻结数学定义、证据边界、数据角色、指标或任务范围的决定都必须先登记。没有记录即表示无 amendment。

| ID | UTC 日期 | 提议/决定 | 原因与证据 | 影响的合同/代码/claim | 确认者 | 状态 |
|---|---|---|---|---|---|---|
| A01 | 2026-09-06 | 将“一次 segment merge”明确为：Stage 1 immutable support snapshot 上以 nominal `sigma^-2` 加权代表幅值，按 link/time/ID 左到右单遍；accumulator 每次合并后重算、next 不重算；差值等于门限时合并；保存父子 ID 和 obs 唯一归属，Stage 2 后 partition 不变 | 原合同允许一次相邻 merge，但未唯一规定三段链的快照、顺序、重算和等号语义；review 要求在实施前消除 partition/hash 歧义。详见 [`METHOD_CONTRACT.md`](METHOD_CONTRACT.md) 4.1 | METHOD partition 定义、T06/U11 fixture、RQ3 automatic cache invalidation、C2 partition correctness；不改变不跨 gap/inactive 和 Stage 2 后冻结的原边界 | 本轮用户指挥/审查会话（未记录个人姓名） | `TECHNICALLY_ACCEPTED_IMPLEMENTED_REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；U11 `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| A02 | 2026-09-08 | 增加默认关闭、显式版本化的 Stage 1 `conditional-navigation stationarity qualification`：开启时，actual linked `gtsam::checkConvergence` 为 true 且同 conditional graph/current Values 的既有 navigation stationarity audit 有效并通过，条件求解才成功；generic convergence 已通过但驻点未通过时，在原总 inner budget 内继续同一 optimizer，不重建、不重置 lambda | T10 诊断复审确认原外层正确拒绝失败 run，未证明 correctness 违反；本策略仅用于检验 conditional accuracy/block coupling 假设 | SELECTED Stage 1 数值求解实现说明、solver/support/cache identity、effective config、trace/failure diagnostics；不改变 frozen objective、外层四项 AND、参数、预算、block 顺序或默认行为 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_DEVELOPMENT_IMPLEMENTATION_AND_DIAGNOSTIC_SCOPE`；R01 双入口同参数修复已由 `/tmp/t10-a03-independent-review-gd5ed4b1/REVIEW.md` 接受，不是默认迁移、正式 validation 或 claim 支持 |
| A03 | 2026-09-08 | `DEVELOPMENT_BUDGET_DIAGNOSTIC`：保留 A02，只在两份隔离 step/ramp 实验配置中把 `discovery_max_outer_iterations` 从 `50` 改为 `500`；每个原输入只运行一次，整个进程硬限 `120 s`，原四项 AND 可自然提前停止 | A02 独立复审接受现有 cap=50 对照的诊断结论，并在 T10-A02-R01 最小修复及 focused checks 通过后批准一次限定 outer-budget 充分性诊断；不是逐档调参或成功承诺 | 仅影响本次两份隔离 development 配置及其 solver/support/cache identity；输入、初始化、科学参数、inner cap、容差、block 顺序、Stage 2、gate、默认配置均不变；若前 50 轮公共数值 trace 超出预声明 `1e-12 + 1e-12*scale` 容差则停止扩大实验 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_DEVELOPMENT_BUDGET_DIAGNOSTIC_SCOPE`：独立复审重算并 fresh 重跑接受 2/2 限定运行；两条仍为 Stage 1 failure，Stage 2/eligible/scientific evidence 未接受，且不授权更多预算 |
| A04 | 2026-09-08 | `DEVELOPMENT_FIXED_CHECKPOINT_LM_RECOVERY_DIAGNOSTIC`：只在 step outer 121 已验证失败的同一 conditional graph、固定 Values 与 captured lambda 上比较 A=当前 linked 行为、B=未接受且目标变化很小的候选不立即结束 lambda 搜索而继续有界搜索；不强制接受增加目标的候选 | 独立复审接受 1 accepted + 49 small-change/no-update 的停滞机制，并批准直接检验继续原上界内 lambda 搜索是否产生真实接受下降及达到原驻点资格 | 仅限隔离诊断旁路；两臂同 graph/Values/lambda、原 `lambdaUpperBound=1e5`、最多 50 次调用且各 `10 s`。B 将内部 `relativeErrorTol 1e-6→0` 以隔离该规则，明确改变 solver 数值语义；外部 generic check `rel=1e-6/abs=1e-8`、stationarity `1e-6`、roundoff/scales、model-fidelity 接受门槛、其余参数及默认路径不变。只执行一次 A03 step 活图重捕获、整进程 `<120 s`，ramp 未运行 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A04_FIXED_CHECKPOINT_LM_RECOVERY_DIAGNOSTIC_SCOPE`：A 50 calls 无更新/不驻点；B 2 calls/3 trials、2 个真实下降更新并驻点。只证明 fixed checkpoint 恢复可行，不授权默认迁移、validation/gate/test/RQ 或更多预算 |
| A05 | 2026-09-08 | `DEVELOPMENT_CONDITIONAL_LM_RECOVERY_V2_INTEGRATION`：新增默认关闭、版本化 V2；optimizer 内部 lambda search 使用 `relativeErrorTol=0`，外部 generic check 保留原 relative/absolute tolerance，成功仍为 generic AND stationarity | A04 独立复审接受同一 fixed checkpoint 上继续有界搜索的恢复可行性；本轮获准把唯一候选接入 development 路径并验证端到端行为 | 保留 GTSAM 接受规则、lambda 上界、同一 optimizer、50-call inner cap、stationarity/roundoff/五类尺度及外层四项 AND；V2 进入 effective config 与 solver/support/cache/scheduler producer identity，两入口执行七字段一致性检查。focused checks 通过后只运行现有 A03 step/ramp 各一次，outer=500、inner=50、每进程最多 120 秒；其余参数/输入/Stage 2/gate 不变，失败不重试、不搜索、不扩预算 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE`：本轮指挥会话实际复核三个 manifest、当前源码快照、最终运行身份、34/34 discovery、双策略反例和 final audit 后接受；限定关闭 RF1/RF2/RF1-N01，不虚构外部 REVIEW。历史动态库身份缺口、最优点 exhaustion 边界、Stage 1 0/2 与正式 `NOT_RUN` 保留；不迁移默认，C1-C3 不升级 |
| A06 | 2026-09-09 | `DEVELOPMENT_V2_FAILED_CHECKPOINT_DIAGNOSTIC`：只在 A05 原配置重捕获 step outer 123 与 ramp outer 168 的 conditional LM 失败 checkpoint，检查原驻点、解析梯度/有限差分、浮点目标消减、各 lambda trial 的 predicted/actual decrease、model fidelity 与分支 | A05 恢复旧 outer 121/127 后在新位置于原 lambda 上界内无 accepted step；现有证据不足以区分已经驻点、Jacobian/objective 不一致、浮点消减或方向/model fidelity 异常 | 复用默认关闭 observer；基线配置固定为 A05 `step_v2_outer500.yaml`/`ramp_v2_outer500.yaml`，分别指定 outer 123/168。每条 estimator 至多一次、整进程硬限 120 秒、无 retry；不启用 A04 shadow recovery，不改输入、初始化、policy、outer=500、inner=50、lambda upper、任何容差/尺度/接受/停止规则或科学 identity。诊断只写隔离 evidence；若 observer 精度不足，只允许不参与决策的最小输出增强 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE`：本轮指挥会话实际核验 manifest 247/247、源码 4/4、当前身份 9/9、discovery 34/34、历史 122/167 行的 86 公共非计时字段及 11/10 个拒绝 trial 后接受；指挥未重跑、重建或改仓库，不虚构外部 REVIEW。`64*epsilon` 不是完整浮点误差上界；A06 没有新增 diagnostic solve，但原 `InspectFirstLinkedLmTry` 本身包含 diagnostic solve |
| A07 | 2026-09-09 | `DEVELOPMENT_EXACT_LM_DIRECTION_DIAGNOSTIC`：优先使用实际 linked GTSAM `TRYDELTA`，捕获 step outer 123/ramp outer 168 每个 authoritative trial 的完整 tangent delta 与基准 Values；只对预登记有限子集做同一 live graph 的方向/逐 factor 求值审计 | A06 已定位为 linear prediction 与 nonlinear objective/fidelity 不符，但没有实际完整方向，不能区分 full-direction gradient、factor/retract/累加误差 | 不修改/安装/替换 `/usr/local` GTSAM，不新增 diagnostic solve、不复制 optimizer；先以工程小图验证 TRYDELTA 的 max-digits 精度、key/dimension 完整性、round-trip 与 actual accepted delta 一致。真实配置/目标固定同 A06；每条 estimator 至多一次、120 秒、无 retry。有限拒绝子集固定为 step lambda `1e-6,1,1e4`，ramp `1e-5,1,1e4`；方向 `u=delta/||delta||2`，中心 FD 步长 `{1e-4,1e-5,1e-6}`，误差界 `5e-9+5e-3|g^T u|`，不要求通过；逐 factor old/tentative 和汇总只在捕获后诊断求值，不参与决策 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A07_BOUNDED_EXACT_DIRECTION_DIAGNOSTIC_SCOPE`：本轮指挥会话接受，不是外部 REVIEW；小图及两次有限捕获事实保持，不授权 validation/sweep/gate/test/RQ/T11--T13 |
| A08 | 2026-09-09 | `DEVELOPMENT_FROZEN_DIRECTION_FACTOR_LOCALIZATION_AND_CONDITIONAL_JACOBIAN_CORRECTNESS_FIX`：在 A07 冻结 step outer 123/ramp outer 168、六个 actual delta 和 `{1e-4,1e-5,1e-6}` 上计算逐 factor analytic `g_f^T u` 与中心 FD；若证据证明某 factor Jacobian 与既有残差/retract 不一致，则在 paper 路径实施最小 correctness fix | A07 证明图级解析方向导数与 nonlinear FD 不一致，但只保存逐 factor full-step error，不能在 UWB/Pose prior/IMU 中确定实现根因；linked GTSAM `PriorFactor<Pose3>` 对 `-Local(x,prior)` 返回单位 Jacobian 是优先待检假设，不预设结论 | A07 不足以恢复 live factor，故按预算 pre-fix 各唯一捕获一次；1314 点与独立反例证实 Pose3 prior Jacobian 不一致。paper-only factor 保持原残差/objective/noise/prior，用 `-D_x Local`；legacy、LM 规则/参数/cap、`/usr/local` 不变。focused 回归后 post-fix 各唯一运行一次：Stage 1 通过，Stage 2 原 50 轮失败；无有效输出 | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A08_PAPER_POSE_PRIOR_JACOBIAN_FIX_SCOPE`；本轮指挥会话接受，不是外部 REVIEW，指挥未重建、重跑或修改仓库。冻结 factor `1314/1314`、图级 `15/18`，三个最小步长残差限制保留；T10 仍 `IN_PROGRESS`，不构成正式 validation |
| A09 | 2026-09-09 | `DEVELOPMENT_STAGE2_BOUNDED_OUTER_BUDGET_DIAGNOSTIC`：只在 A08 两份隔离 development 配置中把 `max_refit_iterations` 从 `50` 改为 `200`，检验 Stage 2 是否因原 outer budget 不足而失败 | A08 step/ramp 已完成 Stage 1，但 Stage 2 均恰在原 50 轮硬上限返回 `MAX_REFIT_ITERATIONS`；现有证据不能区分仅预算不足与其他停止条件阻塞 | 运行前必须证明唯一有效数值配置变化为 refit outer cap；不改 solver 源码、停止/接受规则、容差、LM/default policy、其他 cap 或依赖。step/ramp 各一次、每进程硬限 120 秒、无 retry/搜索；核对 Stage 1 与 A08、一致的 Stage 2 前 50 轮及四项 AND，绑定 runner/core/GTSAM 身份。即使收敛也只产生 development 证据，不等于正式评分/validation | 本轮用户指挥/审查会话（未记录个人姓名） | `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`（本轮指挥会话，不是外部 REVIEW）：唯一配置变化预检通过；Stage 1 与 A08、Stage 2 前 50 轮精确一致；step/ramp 在 outer 66/124 收敛并封存 cache，但全部 group 不 eligible、score unavailable。无源码/solver 改动，不授权继续增 cap或进入 validation |

## T10-A10 本轮预登记

本轮用户/指挥会话明确接受 `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`；
这是指挥会话复审，不是外部 `REVIEW.md`，不虚构新的重建/重跑。A09 两个 cache 全不 eligible 的事实保留。
T10 继续 `IN_PROGRESS`，A08 图级 FD `15/18` 及三个最小步长失败来源 UNKNOWN 保留。
A10 协议、数据角色和 3 套物理尺度 support 候选在生成器实施/运行前登记于
[T10_A10_PROTOCOL.md](T10_A10_PROTOCOL.md)。新数据仅 development，9 次 estimator、每次 120 s，
无 retry/结果驱动扩展；不运行正式 validation/test、不锁 gate、不升级 C1–C3。

| ID | UTC 日期 | 提议/决定 | 原因与证据 | 影响 | 确认者 | 状态 |
|---|---|---|---|---|---|---|
| A10 | 2026-09-09 | 自包含同运动 UWB/IMU + total latent truth；有限 development pilot；交付 validation 准入方案 | 旧 bag 基础误差 UNKNOWN、A09 全部组不可评分，结束旧输入反复诊断 | 生成/输入 provenance 与 development 候选范围；复用既有 cache/C++ estimator，方法/solver/默认/容差不变 | 本轮用户/指挥会话 | AUTHORIZED_DEVELOPMENT_ONLY，协议先于实施；held-out NOT_RUN |

A10 完成事实：生成器与实际有限 pilot 已交付，运行预算关闭。validation 提案不等于准入通过；
Stage1 failure、首次 checker failure 和不可评计数均保留，C1–C3 不升级。

## T10-A11 有限定位交付登记

| ID | UTC日期 | 指挥接受/授权 | 实际结果 | 限制与状态 |
|---|---|---|---|---|
| A10复审 | 2026-09-09 | REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE；会话接受，不是外部REVIEW | 完整生成器和有限pilot交付限定验收 | 不表示estimator成功或validation准入；A08历史15/18不变 |
| A11 | 2026-09-09 | [运行前协议](T10_A11_PROTOCOL.md)：P1三场景各1进程/120s，实际delta通过才同optimizer续至200 | 50次复现；图级18/18、factor6678/6678、坐标9/9；3条均200次后未达原驻点，持续振荡下降；92/92 focused tests；首轮语法build失败保留 | LOCAL_DIAGNOSTIC_COMPLETE_AWAITING_REVIEW；T10 IN_PROGRESS；不进chain/Stage2/held-out/gate/scheduler |

## T10-A13 定向审计与待审 amendment

| ID | UTC日期 | 决定/证据 | 影响与状态 |
|---|---|---|---|
| A12复审 | 2026-09-09 | REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE；本轮指挥会话接受，不是外部REVIEW | 仅接受A12限定诊断；原两臂未驻点/B未达改善判据，非validation准入 |
| A13静态审计 | 2026-09-09 | 原身份门通过，40PIM/1600步、814静态检查；零optimizer iterate；[全部证据](evidence/t10_a13_imu_covariance_20260909T135328Z/VERIFICATION.md) | LOCAL_DIRECTED_AUDIT_COMPLETE_AWAITING_REVIEW；实际K=I6未被A10声明覆盖，保留全部历史限制 |
| A13-PROPOSED-IMU-CONDITIONAL-COVARIANCE | 2026-09-09 | [唯一amendment草案](T10_A13_AMENDMENT_DRAFT.md)：显式paper conditional-on-live-bias模型，以无额外独立积分bias噪声的定义推导K0；非经验赋值 | PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED；改变白化目标及cache身份，legacy默认保留；生产修改、回归和新运行全部NOT_RUN |

## T10-A14 交付登记

指挥接受 `REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE`；A13唯一方案已技术接受，仅显式opt-in development。
[A14 amendment/冻结判据](T10_A14_AMENDMENT_PROTOCOL.md)先于实施。新PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1显式K0，live bias、其他噪声/RW/Qi、交叉项、初始化/priors/solver保持；缺省legacy I6。
实际模型身份接入common、discovery/support、Stage2 producer/cache/request、final及直接core消费门，跨模型拒绝。
A12四SHA精确复现；1680静态检查、127 C++工程回归及Python mock-runner合同通过，实际库身份闭合。
唯一P1 step seed10101 pilot为exit1：outer1第17调用lambda搜索耗尽；16接受/26拒绝trial，Gmax=1.20549e-5未达原驻点。
chain/Stage2/score/cache均未到；candidate/segment/group/eligible/unavailable计数NA，不能把失败空partition解释成零候选或零eligible。
external wall0.688s/RSS27396KiB；无retry，truth/GT未读，raw/config/GTSAM不变。
[A14完整交付](evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md)保留源码/配置/输入/库/命令/负结果；T10 IN_PROGRESS、A14本地限定交付完成待审。
A12负结果、A08历史15/18/UNKNOWN及C1–C3限制保留；validation/test/T11/scheduler/gate/final NOT_RUN；不默认迁移或升级claim。
本轮停止，不自行执行后续末态诊断。
