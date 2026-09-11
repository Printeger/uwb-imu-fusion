# T11 Closeout — T11-C，限定工作包冻结

本次完成用户授权的最小匹配前缀工作包；**完整 U13 未通过，正式 RQ4 科学验收未通过**。唯一裁决 T11-C；T10 保持 C2-C，C1/C2/C3 不整体升级。

## 1. 范围与预登记

仅 condition A、step2、已见 validation 20101/20102；历史固定 [3,6] s、128 个原始历史 obs_id。H=0/1/2 对应 cutoff=6/7/8 s。六个 fresh 科学前缀（包括 H=2）与两条独立 H=0 U13 重放均实际执行，无旧估计复用、无科学重试。
[T11_MANIFEST.json](T11_MANIFEST.json) 在实现/运行前登记；[执行身份](evidence/t11_20260910T160942Z/EXECUTION_BINDING.json) 在首个科学 ticket 前锁定。串行，每次自动链 900 s、每个 final 各 900 s；仅 full_gate 与 structured_debias。

## 2. 最小实现及兼容性

新增 `tools/paper/t11_prefix.py`、`t11_diagnostics.h` 和独立核验/评价入口，复用现有 C++/GTSAM 后端与 ComputeSparseRecoverability。先逐字节保留 time<=cutoff 原始行，再启动估计进程；recording identity、obs_id、source ordinals 不变，prefix payload/cache 身份重算，full-source lineage 独立保存。
显式 T11 context 在初始化前检查两种输入计数和最大时间，导出初始 Values、关键帧与实际 IMU 因子两端/采样依赖；标定内容 hash 与输入路径分离。compact 只关闭逐试步大矩阵/重复细节，证书与调用/停止/失败摘要保留；旧入口默认输出行为保持。solver、gate、噪声、priors、容差与依赖未改。

## 3. 工程验证

两次构建 exit 0；三个 cutoff 的实际 prepare exit 0；严格未来样本反例 exit 2 且在初始化前拒绝。compact/full 小图科学输出精确一致。实际通过：T11 Python 5/5、既有 NumPy golden 22/22、sparse C++ 6/6、IMU 6/6、paper input 9/9；六份固定模型生产矩阵另与既有 SVD golden 对照全部通过。
首次 unittest 模块调用导入失败，直接文件调用后通过；固定诊断汇总首次因 NumPy bool JSON 序列化失败，修为 Python bool 后通过。U13 比较器补齐预登记已排除的两项阶段计时字段；没有排除任何新科学数值或放宽容差。原失败日志与结果保留。
[边界/配置证据](evidence/t11_20260910T160942Z/BOUNDARY_ENGINEERING.json)、[compact 对照](evidence/t11_20260910T160942Z/COMPACT_COMPARISON.json)、[SVD 对照](evidence/t11_20260910T160942Z/FIXED_GOLDEN_AUDIT.json)。

## 4. 六个科学前缀实测

| seed | H (s) | UWB / IMU | Stage1 / Stage2 | final 有效数 |
|---|---:|---:|---|---:|
| 20101 | 0 | 248 / 1201 | FAILED / NOT_RUN | 0 / 2 |
| 20101 | 1 | 288 / 1401 | FAILED / NOT_RUN | 0 / 2 |
| 20101 | 2 | 328 / 1601 | CONVERGED / CONVERGED | 2 / 2 |
| 20102 | 0 | 248 / 1201 | CONVERGED / CONVERGED | 2 / 2 |
| 20102 | 1 | 288 / 1401 | CONVERGED / CONVERGED | 2 / 2 |
| 20102 | 2 | 328 / 1601 | CONVERGED / CONVERGED | 2 / 2 |

20101 H=0/H=1 均在 Stage1 `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`；分别完成 57/60 outer，Stage2/score/final 未运行。另四条科学链均完成评分和两种 final，共 8/8 有效，fallback 0。失败行的未知候选数和误差为 unavailable，不能记零。
八次运行（含 U13）实测外部进程墙时合计 298.602075 s；无 timeout。每次命令、退出码、阶段耗时见 [RUN_LEDGER.json](evidence/t11_20260910T160942Z/RUN_LEDGER.json)。

## 5. U13 结果与访问边界

两条重放均从独立 full-source 副本删除严格晚于 6 s 的行后再构造 prefix；两种 prefix payload 均与原 H=0 字节一致。8/8 estimator trace 未发现 full raw、旧结果、truth 或诊断数据读取。
| seed | 初始/已到达阶段精确比较 | 完整链 U13 |
|---|---|---|
| 20101 | 555 项一致；同一 Stage1 失败 | NOT_RUN_INCOMPLETE_CHAIN |
| 20102 | 1206 项一致；Stage1、Stage2、Rc/η/s、两种 final/物理 graph 与 Values 一致 | PASS |

整体 **NOT_PASSED**。共同失败不能冒充完整链通过；没有由该失败证明未来泄漏，也没有依据触发前缀 correctness 重跑。路径、计时、provenance 身份的排除与对应关系均留档，实际科学数值要求精确。见 [U13_RESULT.json](evidence/t11_20260910T160942Z/U13_RESULT.json)。

## 6. 固定模型诊断（只报告机械核验）

基准仅来自各自本次 fresh H=2 的 Stage2，均有一个完全位于历史区间的两幅值组。固定 support、priors、名义权重、公共 Values/白化 Jacobian 和全部 615 nuisance 列；H 只筛实际有效因子行。保留行数 713/828/943，frozen nuisance rank 465/540/615。N 均为 6400 I₂ 且逐项不变；嵌套行、公共数值、Rc 增量 PSD 与正定时协方差次序的数值检查通过。无 damping、L1/TV 或新增 prior。
| seed | H | λmin(Rc) (m⁻²) | η | s (mm) |
|---|---:|---:|---:|---:|
| 20101 | 0 | 3317.426958 | 0.518347962 | 17.361983 |
| 20101 | 1 | 3464.584268 | 0.541341292 | 16.989259 |
| 20101 | 2 | 3750.014122 | 0.585939707 | 16.329901 |
| 20102 | 0 | 3296.326514 | 0.515051018 | 17.417463 |
| 20102 | 1 | 3441.284020 | 0.537700628 | 17.046677 |
| 20102 | 2 | 3729.864858 | 0.582791384 | 16.373950 |

这些是 `diagnostic_fixed_model` 的数值核验事实；由于整体 U13 未通过，不据此解释端到端科学收益或升级裁决。N 相同意味着 η 与同一 Rc 定义一致；本例 η=1/(6400·s²)，**不意味着 H 间 η 必须数值不变**。谱、秩、行/列映射和公共线性化身份可定位于 [固定模型审计](evidence/t11_20260910T160942Z/FIXED_MODEL_AUDIT.json) 和各 H=2 的 `output/diagnostic_fixed_model/`。

## 7. 匹配、指标与两张表

20102 三个 H 的历史候选观测/幅值 partition 可配对，历史候选无增删；20101 因 H=0/H=1 未完成 Stage1，三前缀比较不可用。四个已评分科学输入均为 32 候选、2 段、1 eligible group。逐组 Rc/η/s/γ、完整 partition 和增删集合全部保留，不按 segment ID 或交集强行配对。
历史实际 correction 覆盖率（历史 128 个原始 obs_id 为分母）：20101 H=2 两种 final 都是 32/128；20102 三个 H 的 full_gate 都是 0/128、structured_debias 都是 32/128。零接受的 conditional error 为 undefined；未运行 final 的 coverage 为 unavailable。
全部结果先冻结，独立 evaluator 随后检查 U13 准入并保持 **truth_read=false**。本轮历史 bias RMSE、accepted-only error、raw-frame 轨迹 RMSE/P95 均 **NOT_RUN_U13_NOT_PASSED**，未以 residual 或旧 T10 指标替代；也未运行全记录收益或策略胜负评价。
[Table A：PREFIX_RESULTS.csv](evidence/t11_20260910T160942Z/PREFIX_RESULTS.csv)、[Table B：FIXED_MODEL_RESULTS.csv](evidence/t11_20260910T160942Z/FIXED_MODEL_RESULTS.csv) 各固定六行；[组明细](evidence/t11_20260910T160942Z/GROUP_DETAILS.json)、[context 图数据](evidence/t11_20260910T160942Z/CONTEXT_FIGURE_DATA.json) 为关联产物。

## 8. 唯一裁决

**T11-C。** 直接触发预登记的优先条件：整体 U13 未通过（20101 完整链未到达）。固定模型机械核验通过不能覆盖此条件。不是“已证伪 future information 定理”，也不是“发现未来泄漏”；不修 solver、不放宽数值标准、不追加 seed/H 或重试。见 [DECISION.json](evidence/t11_20260910T160942Z/DECISION.json)。

## 9. 论文后果与 claim 边界

保留 future-context 数学分析和限定工程诊断；当前不能把 RQ4 写成两条轨迹重复端到端收益证据，context 图不得填入未评价的轨迹/bias 数字。T10 的 C2-C、η/gate 探索性定位和所有历史限制保持。C1/C2/C3 不整体升级，更新 `paper/CLAIM_EVIDENCE.md`；未改冻结论文结构。

## 10. 冻结、精简与 T12 交接

运行输入、初始/Stage2/最终状态、support/masks、score、失败与访问日志、命令/退出码/hash 均在隔离目录保留。冻结清单含 7459 项产物；报告前的只读核验与导出失败不被写成科学失败或成功。没有归档重复二进制或逐试步大矩阵，也没有改动旧冻结包。
核验后仅将本次 U13 中 1047 个字节相同的大于等于 1 KiB 的重复产物替换为指向原 H=0 产物的相对引用，去重 4,147,883 bytes；原 frozen hash 仍可逐项复核。独立运行发生在去重之前；重建路径、hash 与时间边界见 [RETENTION.json](evidence/t11_20260910T160942Z/RETENTION.json)。
**下一任务仅为 T12：先核实既有真实/公共输入的标定、参考与时间语义，再按冻结实现和指标执行限定评估。** T12 本轮 NOT_RUN，不自动恢复 T10/T11 扩展研究。
