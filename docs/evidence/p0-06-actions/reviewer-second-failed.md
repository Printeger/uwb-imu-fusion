# P0-06 independent review — second result

P0-06 第二轮独立验收结论：**FAILED（可修复，非 BLOCKED）**。

通过项：
- HEAD 与 `golden-p0-05-publication^{}` 均为 `02ae9f6...`；总表、commit、tag 未改。
- normal clean、full all-target、tests target 全部通过。
- 完整 CTest：29/29 PASS，92.61 s；round2 PASS。
- P0-06 directed：17/17 PASS。
- selected P0-02/FDE/selection：15/15 PASS。
- P0-04：13/13 PASS；P0-05：13/13 PASS。
- golden-header cross-DSO ABI：PASS，旧枚举值和冻结布局保持。
- source/binary/loaded/input/artifact hashes通过；`git diff --check` PASS。
- raw occurrence census、cap 0/1/128、`+0/-0`、NaN、compact collision、validator fail-closed、130/cap128 refusal等基础修复有效。
- 未改 config、threshold、AL、risk、prior 或 hypothesis coverage；未提前实现 P1 streaming。

阻断 PASS 的缺陷：
1. **Exact duplicate 的代表选择仍依赖枚举顺序，离散结果不稳定。** `exactActionOperationIdentityV1()`不包含`action.id`和`action_model_id`，而`censusAndCapActionsV1()`保留首个occurrence。独立probe：
```
identity_equal=1 forward_id=1 forward_model=UWB_EXCLUSION
reverse_id=99 reverse_model=IMU_EXCLUSION
```
`action_model_id`被`FdeManager::statusFor()`用于FDE status；action ID也是selection tie-breaker和最终输出。因此duplicate reorder可改变selected identity/type/status。
2. **O07 54-cell grid仍非所声明真onset×axis×joint production oracle。** 只把labels折进标量`raw_fault`，固定H和两个手工action；未构造对应hypothesis/action/census，也未在54 cells运行FDE。注释承认no injection label passed。
3. **130-action O07没证明最终winner。** 129个非目标action用不存在group ID；只断言index128唯一reference disposition，reorder只比status/commit boolean，没有断言selected_action/winner ID/plausible coverage/risk disposition全量一致。
4. **异常传播probe仍是手工final-packet input。** 未在production monitor candidate kernel/post/PL抛异常，也未证明outer catch保留generated/evaluated/omitted、每occurrenceidentity/reason/duplicate-of。
5. **Evidence不可复现完整candidate。** complete-diff.patch漏整个untracked `test/test_p0_06_action_search.cpp`；acceptance-summary directed仍11而实际17，selected_inherited写21与记录selected15不一致。

最小修复：对exact duplicates选择order-independent canonical representative，或把所有影响排序/status/commit/publication字段纳入等价；增加正反顺序并精确比winner/action ID/type/status/disposition。用真实mode/hypothesis/action生成路径做54-cell并逐cell走census/rank/post/PL/FDE。index128断言真正uncapped selected winner及reorder完整离散状态/coverage/disposition。增加production exception injection检查persisted census/final packet/CSV。重生成含新测试文件的complete diff并统一evidence计数。
