# P0-05 fifth independent review — FAILED

P0-05 第五轮独立复核：**FAILED**，不是 BLOCKED。工作树保持只读。

技术验收全部通过：

- 正常 clean → full all-target → tests 构建通过。
- Full CTest：28/28 PASS，93.64 秒。
- P0-05 directed：13/13 PASS。
- P0-01～P0-04/round2 selected：6/6 PASS；round2 Python：7/7 PASS。
- Golden-header 跨 DSO ABI：按值、数组、vector 全通过；布局为
  424/416、4128/3864、V1 88/80、V2 256/248；`common/types.hpp` 相对
  golden 无差异。
- 三份现有 SHA-256 manifest 全通过，运行时动态库解析到已哈希的
  `devel/lib/libuwb_imu_pl.so`。
- 独立真实 certified ROS capture 通过：finite reference transfer，
  compound/legacy/CSV digest 一致，且均为 `authoritative=false`、
  `protected=false`、`formal=false`。
- O04、O08、single-use proof 对抗、slow/partial/logger failure、yaw、
  back-clock、overflow 未发现新技术失败。

失败原因是 §2.5 强制证据不完整：

1. 缺少 `docs/evidence/p0-05-publication/complete-diff.patch`。
2. `environment.txt` 声称 resolved config、manifest、simulator input hashes
   已保存在 hash manifests，实际两个 manifest 中没有任何
   config/manifest/input/truth 条目。
3. `ros-smoke-results.txt` 只记录 `/tmp` 中 `run_manifest.json` 和
   `resolved_config.yaml` 的摘要，未保留对应 artifact 供独立
   `sha256sum -c`；input/truth 也没有 hash 或明确的 `NOT_RUN/N/A`。

最小修复：补齐相对 `golden-p0-04-risk` 的完整实现 diff、实际 resolved
config/run manifest 及 fault-manifest/simulator input/truth hashes；不适用项
明确记录 `NOT_RUN/N/A`；修正证据陈述并保留本轮 FAILED，然后交由新的
reviewer 复核。
