# P0-05 seventh independent review — FAILED

P0-05 第七轮独立复核：**FAILED**，不是 BLOCKED。全程只读，未修改工作树。

失败原因均在 O08 强制门：

1. 15D raw covariance 未验证对称性  
`test_p0_05_publication.cpp:406` 仅检查有限值；随后先计算 `0.5*(C+Cᵀ)`，再检查特征值为正。没有断言原始 Marginals covariance 满足 `C≈Cᵀ`，因此未完成要求的 finite/symmetric/strict-positive 三项验证。

2. `DuringBackendUpdate` 不能证明精确到达注入点  
`incremental_estimator.cpp:545` 先执行真实 fixed-lag update，只有成功返回后才触发注入。update 内部任意自然异常也会被包装成相同 `boundary="backend_update"`。测试在 `test_p0_05_publication.cpp:589` 只比较 boundary 字符串，随后无条件计入 `reached`，未核验注入原因或不可伪造的 hit token。因此历史 near-x4 自然异常仍可能冒充该注入点，证据中“exact eight-boundary reached”陈述过强。

其余独立验收均通过：
- O08：1200 个真实冷进程、8 路并发，1200/1200 PASS；有效 `--gtest_repeat=500` 为 500/500 PASS。
- clean → full all-target → tests：PASS。
- 完整 CTest：28/28 PASS，91.43 秒。
- P0-05 directed：13/13 PASS。
- P0-01～P0-04/round2 selected：6/6 PASS。
- 五类 SHA-256、`ldd`、`git diff --check`：PASS。
- complete-diff：临时 golden archive 实际应用成功，22 个文件与当前实现逐字节一致，reverse check PASS。
- Golden-header 跨 DSO ABI：424/416、4128/3864、V1 88/80；按值、数组、vector 和旧客户端运行 PASS。
- 归档仅有 `.launch.txt`，与生产 launch 逐字节一致；包名 simulator 无 duplicate-file。
- 独立 certified 和 simulator ROS capture 均成功：共同 digest，`authoritative=false`、`protected=false`、`formal=false`；certified reference 为 `map/body_origin`。
- O04、proof、publication slow/partial/logger、yaw 等未发现回归。
- 六轮历史 FAILED 均保留。

最小修复范围：增加原始 15×15 covariance 对称性断言；为每个 fault point 增加不可被自然异常冒充的注入命中身份，并在 receipt/test 中核验，尤其是 `DuringBackendUpdate`。修复后需保留本轮 FAILED 并交新的 reviewer。
