# P0-05 sixth independent review — FAILED

P0-05 第六轮独立复核：**FAILED**，不是 BLOCKED。共享工作树保持只读。

通过项：
- `complete-diff.patch` 精确覆盖当前相对 `golden-p0-04-risk` 的 22 个实现/消息/构建/测试文件。
- 在临时 golden archive 上 `git apply --check`、实际 apply 均成功；22 个目标文件与当前树逐字节一致；当前树反向检查成功。
- 技术文件时间均早于第五轮 review，确认本轮修复仅涉及 evidence。
- `source/artifact/main/binary/loaded-library` 五份 SHA-256 manifest 全部通过；`ldd` 指向已哈希的同一 private DSO。
- Golden-header ABI 独立重新编译、链接和运行通过：424/416、4128/3864、V1 88/80，按值、数组、vector 均成功。
- ROS simulator artifacts 与 `/tmp/p005_ros_repro` 原件逐字节一致；run manifest、capture、CSV 的 attempt/digest/authority/formal/frame/reference 对齐，truth 时间覆盖 capture。N/A/NOT_RUN 理由与实际执行相符。
- README、environment、summary、journal 与证据一致，五份历史 FAILED 均保留。
- `git diff --check` 通过。

失败原因：
独立首次执行当前已哈希的 `test_p0_05_publication` 得到 **12/13 PASS，exit 1**。失败项：
```
P005Transaction.EveryPostMutationBoundaryHasExplicitTerminalReceipt
commit terminal at backend_update:
gtsam::IndeterminantLinearSystemException near x4
```
该异常发生在 O08 mature fixture 建立期间，导致当次没有完成八个注入边界的验收。已确认当时没有 agent 修改、重建或替换 DSO。

后续有效复跑均通过：同一测试新进程300次；同一进程--gtest_repeat=500；8路并发200次；完整13项连续5次。这说明低频/环境敏感的不确定失败，但后续PASS不能抹掉真实非零退出。按照§2.3停止规则本轮不能PASS。

建议最小修复：仅稳定O08 mature fixture，使预备图在所有运行中严格充分约束并可靠到达八个故障注入点；保留本轮FAILED，随后正常clean/full/tests/CTest，刷新complete diff、source/binary/library hashes，再交新的reviewer。不得更新总表、commit或golden tag。
