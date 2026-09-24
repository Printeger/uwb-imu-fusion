# P0-05 fourth independent review — FAILED

P0-05 第四轮独立复核结论：**FAILED**，不是 BLOCKED。未修改工作树。

逐 gate 结果：
- 正常 `clean → full all-target build → tests`：PASS
- Full CTest：28/28 PASS，92.06 s
- Round2 runner：存在，Python 7/7 PASS
- P0-01～P0-04 selected regressions：4/4 PASS
- P0-05 directed：12/12 PASS
- O08 成熟多 epoch、8 注入点、完整 raw snapshot：PASS
- O04 独立 GTSAM LM/Marginals、真实 certified commit、triangle transfer：PASS
- genuine/single-use proof 与 fake/tamper/replay/stale/Inf 等 fail-closed：PASS
- yaw 90° 非对角协方差、back-clock、overflow：PASS
- ROS 字段与 capture artifact 一致性：PASS
- 实际 publication deadline/atomic authority：**FAILED**
- 公共 ABI：**FAILED**
- loaded-library hash 真实性：**FAILED**

关键失败证据：

1. 实际 ROS publish 仍在 terminal freeze 后执行。
- `invokeFinalOutputPacket()` 在调用 `commitReceipt()` 前已经采集最后一个 clock、冻结 terminal packet：src/uwb_imu_pl/publication/final_output_packet.cpp:175
- 生产 `publishCandidate()` 仅计算 serialization length；真正的 compound/odometry/status publish 位于 post-freeze `commitReceipt()`：tools/run_realtime_integrity.cpp:891
- CSV logger 又在 `publish()` 返回后运行：tools/run_realtime_integrity.cpp:482
独立 probe：
```
slow actual_return=5000 sampled_return=140 missed=0
protected=1 authoritative=1 outcome=1

partial first_visible=1 second_failed=1
authoritative=1 outcome=1 protected=0
```
因此 slow actual authoritative return 可以不进入 deadline；`commitReceipt noexcept` 中第二 sink 失败也能被吞掉，而返回 packet 仍声称 `Success/authoritative`。

2. 公共 ABI 并未完整保持。
```
golden IntegrityOutput: 4128
current IntegrityOutput: 4256
current PublicationDiagnostics: 544
V1: 88 / 80
V2: 256 / 248
```
V1/V2 单独布局正确，但 `PublicationDiagnostics` 被原地扩张，连带改变 `IntegrityOutput`。证据中声称 `git diff golden-p0-04-risk -- include/uwb_imu_pl/common/types.hpp is empty`，实际非空。旧 client 偶然运行成功不能证明按值传递、数组或容器场景 ABI 安全。

3. Hash evidence 不闭合。
```
sha256sum -c docs/evidence/p0-05-publication/loaded-library-hashes.sha256
/home/mint/ws_fusion_uwb/devel/lib/libuwb_imu_pl.so: FAILED
```
文件记录期望 `9e7a95...`，正常重建后的实际值为 `3bda58...`；后者与 `binary-hashes.sha256` 一致。

最小修复范围：
- 恢复 `PublicationDiagnostics`/`IntegrityOutput` 的 golden 布局，把新增 terminal 字段移入独立 V2 sidecar。
- 生产真实 ROS authoritative publish 调用与返回，以及所有可失败 journal/serialization，必须进入 terminal deadline/freeze；或者将 ROS 全部明确降为 non-authoritative mirror，并使用真正的单一原子 receipt sink。
- post-freeze `commitReceipt()` 不得执行多 topic fanout、吞掉 sink 失败后仍返回 `Success/authoritative`。
- 增加针对生产 adapter 的 slow actual return、第二 sink 失败 probe。
- 保存本轮 FAILED，并重做诚实 ABI/hash evidence。

不得更新总表、commit 或创建 `golden-p0-05-publication`。
