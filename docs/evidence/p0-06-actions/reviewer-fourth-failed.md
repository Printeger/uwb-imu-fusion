P0-06 独立复核结论：**FAILED（可修复，非 BLOCKED）**。

阻断项：

1. **50 m/s² 分支掩盖 production `RankUpdateEvaluator` 崩溃。**

我在 `/tmp` 复制测试，以 `-O3 -DNDEBUG` 编译，仅将 50 m/s² 的 `DenseCandidateOracle` 替换回 `evaluator.evaluate(base, action)`；限定 50 m/s² 后运行同一 54-cell 测试，结果：

```text
[ RUN      ] P006O07.ProductionBackedAmplitudeOnsetAxisJointGrid
Segmentation fault
EXIT=139
```

正式测试在 `test_p0_06_action_search.cpp:877-890` 手工为 50 m/s² 选择 dense oracle。生产 monitor 在 `integrity_monitor.cpp:2678-2753` 始终直接调用 `RankUpdateEvaluator`，没有安全前置条件、自动 dense exact fallback，也没有能将 SIGSEGV 转为 terminal fail-closed 的边界。因此 evidence 所称“production-backed 54-cell”不成立。

2. **54-cell 的 plausible/profile evidence 是测试手工指定，不是真实 production evidence 链。**

`test_p0_06_action_search.cpp:820-826` 直接写入：

```cpp
target_evidence.plausible = true;
target_evidence.profile_j = amplitude * amplitude;
```

随后仅把 `{target_evidence}` 交给 action/FDE；PL 甚至使用空 hypothesis 集合（`:894-897`）。这没有运行生产 hypothesis evidence/plausibility 计算，selection 由手工证据决定。

joint coverage 也不足：测试只检查这个手工挑出的 `target.affected_groups`，最终恢复断言仅查 `target.affected_groups.front()`（`:950-954`），未核验所有真实 plausible hypotheses 的全部 affected groups。故 evidence 中“complete plausible census/full production chain”声明过强。

通过项：
- trusted raw snapshot 修复仍有效：validator 从外部 snapshot 重算完整 census、cap、duplicate 与 evaluated bytes；定向回归 `SelfConsistentSidecarAndEmbeddedRawCannotReplaceTrustedSnapshot` PASS。
- release 环境 hook 修复仍有效：生产源码无 `UWB_IMU_PL_P006_THROW_STAGE` 读取，`ReleaseRuntimeIgnoresLegacyP006EnvironmentVariable` 在 NDEBUG binary PASS。
- 现有 evidence 记录 normal build PASS、directed 22/22、完整 CTest 29/29、ABI PASS；这些不能覆盖上述两个阻断项。
- reviewer 未修改共享工作树；实验源码和 ELF 仅位于 `/tmp`。

最小修复：
- production 在调用 rank evaluator 前依据可证明的安全 precondition 自动选择 dense exact fallback，或修复 rank-update 内核；不得靠测试手工分流，且必须保留全部 actions/census。
- 54-cell 每格运行真实 production evidence/profile/plausibility 计算，并以其完整 plausible set 驱动 action generation、PL、FDE；逐一验证所有 hypothesis 的全部 affected_groups 和 joint coverage。
