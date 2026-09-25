P0-06 第五轮独立验收：**FAILED（可修复，非 BLOCKED）**。

阻断项：

1. Production 异常路径仍丢失 action census/identity。
- Production 仅在搜索本身 non-exhaustive 时于 `integrity_monitor.cpp:2465-2482` 预写 omission。
- exhaustive 搜索的 occurrence/duplicate audit 要到 candidate、post、PL、FDE 之后才在 `:3164-3177` 写入。
- candidate router 调用位于 `:2758`；任一 candidate/post/PL 异常会直接进入 `:3564-3582`，此时 `last_attempt_output_` 尚无 generated occurrence、operation/semantic identity、duplicate-of。
- `ProductionExceptionsPreserveCensusThroughPacketAndCsv` 未运行 `RealtimeIntegrityPipeline`。它在 test:1576-1603 手工构造 `IntegrityOutput`并填audit，在:1606-1647局部lambda抛/捕获，再于:1690-1727手工finalize/logger，证明的不是production exception chain。

2. Router未实现rank异常fail-closed。
`CandidateEvaluationRouterV1::evaluate()`在rank_update_kernel.cpp:1590裸调用`rank_.evaluate(...)`，无catch/异常时dense terminal/reason/counter。异常会越过exact postcondition。

3. 130/index128 winner不是完整真实recoverable oracle，并引入越界风险合同入口。
- 测试在test:1304,1322直用RankUpdateEvaluator，绕过production router。
- PL在:1319,1327-1328用空hypothesis集合none，FDE却在:1342-1343用真实target，PL未覆盖FDE声称覆盖hypothesis。
- 测试通过`context.complete_risk_inputs=&qualified`人工令风险闭合。为此release public FdeDecisionContextV2新增未经proof/validator绑定的complete_risk_inputs，并由fde_manager.cpp thread-local替换正常ledger输入。这不是D09修复，外部调用者可用。

通过项：normal clean/full/tests、directed23/23、54-cell统计与50m/s无崩溃、P0-04/05/selected/round2、CTest29/29(91.85s)、ABI、五hash、12-path patch、无config/threshold/risk文件变更及无旧env hook均PASS。

必须修复：production异常前完整真实census持久化及真实pipeline测试；router catch可捕获rank异常返回dense/failclosed；完全删除risk override/thread_local/public入口；130用router、PL/FDE同一非空完整hypotheses并通过正常production risk ledger获得真实winner/commit，不得shortcut。
