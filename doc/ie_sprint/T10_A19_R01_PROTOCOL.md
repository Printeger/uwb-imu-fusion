# T10-A19-R01 Stage2 接线修复、工程门与唯一 fresh development 运行协议

时间：2026-09-10

授权与审查来源：本轮用户任务
`/tmp/t10-a19-progress-review-FVSycd/NEXT_PROMPT.txt`，审查依据为真实文件
`/tmp/t10-a19-progress-review-FVSycd/REVIEW.md`，独立核验为同目录 `AUDIT.json`。

登记双重结论：

- `REVIEW_ACCEPTED_A19_FAILED_RUN_DIAGNOSIS_SCOPE`：只接受 A19 已消费的一次有界失败运行及
  `Stage2 common-reference range metadata` 缺失这一直接归因。
- `CHANGES_REQUESTED_A19_STAGE2_INTEGRATION`：A19 的 Stage2/scoring 集成尚未接受；T10 保持
  `IN_PROGRESS`，C1--C3 不升级。

本协议只授权 T10-A19-R01。旧 A19 ticket、输出目录和失败证据只读保留，不补写 A18 partition，
也不把旧失败改标为本轮结果。

## 1. 限定修复与不变量

1. R1：复用 `SegmentRefitter` 和既有 `CertifiedLm`。每次构造 Stage2 conditional graph 时，
   同步为图中每个实际 range 提供 certificate metadata。共同参考/noncandidate 使用该观测的实际
   fixed beta；candidate 使用同轮实际 fixed beta 加当前 live `c_s`。在调用 certified consumer 前检查
   factor index、pose key、唯一 obs_id、raw measurement、nominal sigma、anchor、lever、条件常数及实际
   residual 与图一一对应、全覆盖、无重复。未知或不一致因子必须失败，不能跳过，也不能把 reference
   重标为 candidate。
2. R2：保留既有 stub 为局部接口测试，新增真实 mixed-reference/candidate fixture。fixture 必须包含
   非零 fixed beta、至少一次真实非零 `c_s` 更新，并实际调用 certified solve 和 native retract；验证每轮
   `beta+c_s` 逐位来自 live 状态、固定 partition、精确 `c_s>=0` 更新、无 L1/TV、原四项 AND、最终 joint
   graph 保留 live `c_s`，并用既有 scoring 验证共同 reference 排除全部 candidate 以及 eta/s/gamma 语义。
   不改变数值阈值或停止判据制造通过。
3. R3：Stage1 完成后、进入 Stage2 前原子保存本轮 support snapshot/partition、obs_id/parent/hash 和阶段
   状态。所有异常路径写合法 JSON 的 pipeline/stage 状态和准确原因；已确认计数保留，无法确认的字段为
   `null`/`NA`，未到达阶段为 `NOT_RUN`。constructor-before-iterate 反例必须为零 call/trial；部分执行后
   异常必须保留已知计数并将未知状态明确为未知。

冻结方法定义不变：不修改 GTSAM、系统依赖、IMU 模型、Jacobian、333-bit precision、paired reduction、
damping、objective、lambda、阈值、初始化、采样、分组、非负更新或评分定义。Stage2 最终结果仍是含 live
`c_s` 的同一无 L1/TV joint graph/Values；不增加 corrected pseudo-range。信息量不含 damping、L1/TV 或
人工 prior。实现默认关闭、只允许 development role，schema `consumable=false`，正式 reader 继续拒绝。

## 2. 身份与失效范围

R1--R3 改动使旧 A19 的 Stage2 implementation/request/diagnostic manifest/producer identity 全部失效。
新请求必须绑定新的实现版本与 policy identity，并记录实际 runner、core、GTSAM、MPFR、GMP 及运行时
映射库身份。受实现影响的 Stage2 cache/final cache 均不可复用；本轮不生产正式 cache。A10 frozen raw、
配置语义、PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1、A18 Stage1 数值政策和原输入身份保持，不把旧 checkpoint
作为 warm start。

## 3. 冻结工程门

工程门在唯一运行前全部满足，任何失败只可在 R1--R3 范围内修复并保留失败记录；仍有失败则 pilot
`NOT_RUN`：

1. R1 正例覆盖真实 mixed graph 的全部 range metadata；反例覆盖遗漏、重复、错位 factor/obs/key/raw/
   sigma/anchor/lever/constant/residual，并明确失败。
2. R2 fixture 实际经过 CertifiedLm direction/native retract、非零 live `c_s` 更新、原四项 AND 和既有
   ScoreRefitRecoverability；stub 结果不计入该门。
3. R3 验证 Stage1 完成即持久化、constructor-before-iterate 零计数、iterate 内部分执行异常的已知/未知
   计数、strict JSON、后续阶段 `NOT_RUN`。
4. A18 封存 43-pair 的 P/D、decision、fidelity 与精确有理相容性保持；受影响 A18/A19 工程测试以及
   discovery/refit/scoring/config/cache/default 回归全部通过，旧正式 reader 拒绝新 diagnostic manifest。
5. 保存每条实际命令、退出码、stdout/stderr、当前源码 SHA、binary/core 及实际动态库身份；工程语义由
   上述定向断言证明，不以测试总数、文件存在或 stub exit 0 代替。

## 4. 唯一 fresh development 运行预算与停止边界

工程门全部通过后，无需再次授权，直接创建一个新隔离输出目录和新一次性 ticket，启动恰好一个新的
estimator 进程树：

- 输入只用 A10 frozen P1 step seed10101 的原 raw/config，从 raw 重新初始化；estimator 不读取 truth/GT。
- Stage1 outer `<=500`，每个 conditional block `<=50`；Stage2 outer `<=200`。保持
  PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1、PAPER_CERTIFIED_PAIR_REDUCTION_V1、333-bit 策略、lambda 和全部
  原容差。
- 外部硬超时 `900 s` 覆盖整个进程树。这是修复后的新登记运行，不是旧 A19 retry。
- 首次 Stage1、Stage2 或 scoring 失败立即停止；仅 Stage1 与 Stage2 都成功才运行现有 eligibility 和
  eta/s/gamma scoring，评分完成立即停止。失败后不追加科学进程、warm start、预算、参数或精度搜索。
- cache production、final inference、正式 validation/test、LOS/ramp/P2/P3、T11、scheduler、gate decision
  与 threshold sweep 全部 `NOT_RUN`。

## 5. 交付判据

保存原始 Stage1/Stage2 traces、calls/trials/accepted/rejected/unresolved、停止条件、Stage1 snapshot/
partition、Stage2 amplitude/short/boundary/KKT、group/eligible/unavailable 原因及逐段 gamma。若达到 scoring，
只从同一 Stage2 graph/Values 无优化地导出 factor/obs mask、ordering、F/G/N/R、谱/秩/数值审计与
linearization identity；全部标 diagnostic-only。

分别报告 Stage1、Stage2、scoring 和总成本。零 eligible 或 rank/numerical unavailable 是允许的实际结果，
不能强制接受 group，也不能从 solver 成功推断 bias 正确。若仍失败，保存完整失败和直接阻塞后停止。
若成功，仅关闭本轮接口修复；下一工作包只提出“同策略 final/cache 贯通和 validation 准入”的最小缺口、
复用点与基于本轮实测的有限预算，不在本轮执行。

本轮 P1 单输入不支持 eta 增量或正式政策比较。正式 RQ3 仍须使用冻结的数据变化、公平 cache、held-out
规则，并允许无增量或负增量。正式 RQ、锁定指标、validation/test 和 C1--C3 均保持 `NOT_RUN`/不升级。
