# P0-05 first independent review — FAILED

P0-05 独立 review **FAILED**。

- Mutation guard 部分通过，但 O08 staging 断言不足。
- Receipt/reference `FAILED`：仅检查 `proof_id != 0`，测试中的伪造 `1234`
  仍然 `reference_bound=true`；registry 验证发生在 commit 后，并且 caller 提供的
  mean 只由 caller 自哈希。
- Final packet/deadline `FAILED`：`publish_call` 在真实 publish 前采样，之后仍有
  finalize/patch；publisher 异常没有写入同一 packet/CSV。
- 90-degree-yaw 部分通过，但只测了对角 covariance，且缺少 ROS raw 证据。
- O04/O08/O10 不是独立 oracle。
- 正常 clean/full build 后 CTest 为 `27/28`、exit `8`：
  `test_round2_tools` expected `FAIL`，实际 runner 报 `PASS`，即使
  `risk_closed=false`、`protected=false`。实现把 ELF 移到 `/tmp` 后得到
  `28/28` 不可接受；`/tmp` 中仍有重复副本。
- ABI：`IntegrityOutput`、`CommitReceipt` 等 public layout 改变，旧 binary crash，
  且没有 ABI/SONAME bump 或兼容层。
- Section 2.5 evidence 不诚实或缺失 raw ROS、tamper replays。

Required repair is limited to these P0-05 findings: restore honest normal
clean/full/test behavior; consume and verify a real P0-03 candidate/reference
sidecar before mutation; independently snapshot every O08 mutation boundary;
place deadline observations on the actual publish invocation/return boundary
and persist the same terminal packet on all publisher exceptions; strengthen
O04 covariance/reference oracles; retain reproducible compound/legacy/CSV raw
evidence; preserve the golden public ABI (prefer a versioned sidecar); and
record the failed review plus every repair attempt honestly.
