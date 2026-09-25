# P0-06 independent review — first result

P0-06 独立验收结论：**FAILED（可修复，非 BLOCKED）**。

通过项：
- HEAD 与 `golden-p0-05-publication^{}` 均为 `02ae9f6...`，总表未修改。
- clean、full all-target、tests target 全部通过。
- 完整 CTest：29/29 PASS，91.79 s。
- P0-06 directed：11/11 PASS。
- P0-01～P0-05 selected regressions、round2、P0-05 13/13 均通过。
- Golden-header cross-DSO ABI PASS：旧枚举值和冻结布局保持。
- 无 config、threshold、AL、risk、prior、fault/hypothesis contract 改动；未提前实现 P1 streaming。
- complete diff 可应用到临时 golden checkout并逐字节匹配。
- source/artifact/input/loaded-library hashes通过。
- 130/cap128基本拒绝路径返回SEARCH_INCOMPLETE，最终packet降级unavailable/unprotected/formal-false。

阻断PASS的缺陷：
1. **生产 census 漏记生成阶段的 exact duplicate。** `actionsForPlausibleSetV1()`在调用`censusAndCapActionsV1()`前已去重，重复操作既不进入generated，也无omission identity/reason。独立probe：
```
actual_generated_keep_plus_requests=3
census_generated=2 evaluated=2 omitted=0 omission_records=0 exhaustive=1
```
2. **所谓 exact dedup 不是 byte-exact。** `equivalentBlock()`使用浮点`==`。`+0.0`与`-0.0`字节不同、operation identity不同，却被当重复：
```
identity_equal=0 retained=1 omitted=1
reason=EXACT_OPERATION_DUPLICATE
```
3. **FDE未验证census证书。** FdeManager只信`exhaustive`布尔，不校验protocol_version、generated/evaluated/omitted等式及溢出、identity数量唯一性、records与计数一致、proven_safe/exhaustive派生、unknown/resource omission。构造exhaustive=true但带未证明omission的sidecar可绕过。
4. **异常传播不完整。** production直到全部候选求值后才把SEARCH_INCOMPLETE写output；若candidate kernel/post/PL抛异常，outer catch保存的output不含reason/omitted identities。现有测试只是手工构造已带reason packet，未复现生产路径。
5. **O07不是生产恢复oracle。** grid仅synthetic `abs(signature-target)` nearest-action helper，未运行production detector/post-detector/PL/FDE，未验证wrong exclusion、完整plausible coverage；index128非真实唯一post/PL通过动作。
6. **边界测试/evidence不足。** 未测max=1；“oversize/overflow”实际仅2动作+cap0；未验证重复identity/action ID、排序/collision对最终离散decision不变；独立clean后binary-hashes中P0-06 test ELF mismatch。

最小修复：所有raw generated先进入唯一census/dedup并记录每个duplicate proof；真正byte-exact deterministic multiset比较；严格census validator由FDE/commit/final消费；在可抛操作前持久化incomplete或提前拒绝并用actual evaluated计数；增加production-backed uncapped O07真实post/PL/plausible/FDE/reorder；补cap0/1/128、count/ID overflow、tamper、duplicate identity、异常传播，重生成全部evidence/binary hashes。Live ROS NOT_RUN非本步blocker。
