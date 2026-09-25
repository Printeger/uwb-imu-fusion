# P0-06 independent review — third result

P0-06 独立验收结论：**FAILED（可修复，非 BLOCKED）**。

阻断项：

1. **测试异常开关污染 Release 生产行为**

   `integrity_monitor.cpp:45` 直接读取 `UWB_IMU_PL_P006_THROW_STAGE`；
   `integrity_monitor.cpp:2456` 会据此向真实 KEEP_ALL raw census 插入
   duplicate，`integrity_monitor.cpp:2485` 改写输出，随后对应 stage 会抛异常。
   当前库以 `-O3 -DNDEBUG` 构建，未见 test-only 编译隔离。任意运行环境只需
   设置该变量，即可把正常生产结果变为异常、改变 generated/omitted census，
   违反普通软件合同。

2. **strict census validator 未绑定可信 raw/generated action bytes**

   `hypothesis_generator.cpp:897` 只验证 sidecar 字符串彼此自洽；
   `hypothesis_generator.cpp:969` 仅把 evaluated records 绑定到实际 evaluated
   actions，没有任何实际 raw/generated vector 用于核验 omitted duplicate。
   独立临时 C++ probe 构造了真实 raw omitted action 与 retained action不同，但
   伪造一套自洽 duplicate sidecar：

   ```text
   actual_raw_equal=0
   validator_valid=1
   validator_exhaustive=1
   reason=VALID
   ```

   因而虚假的 omitted exact-duplicate 仍能被验证成 exhaustive/safe。

3. **54-cell O07 未经过真实 mode/hypothesis generation**

   `test_p0_06_action_search.cpp:152` 的 `productionActionFixture()` 手工填充
   `FaultModeBasis`、`FaultUnit` 和 `FaultHypothesisV2`；joint hypothesis 也在
   `test_p0_06_action_search.cpp:676` 手工构造。54-cell 测试仅调用
   `actionsForPlausibleSetV1()`，全文件没有 `HypothesisGenerator::generate()`
   调用。因此后半段虽然经过真实 census/rank/post/PL/FDE，也不能支持 evidence
   中“production mode/hypothesis generation”的声明。

通过但不足以扭转结论的项目：normal clean/full all-target build、tests target、
完整 CTest 29/29（92.55 s）、P0-06 directed 19/19、selected P0-02/FDE 17/17、
FdePostSelection 3/3、P0-04 13/13、P0-05 13/13、round2、golden-header
cross-DSO ABI、hash 与 complete-diff byte comparison 均 PASS。未发现 config、
阈值、AL、风险预算、prior、fault/hypothesis coverage 变更，reviewer 未修改工作树。

最小修复：从生产库移除环境变量驱动 fault hook，改为仅测试构建的显式注入；
validator 从完整 raw/generated snapshot 重算并绑定 omitted duplicate；54-cell 每个
cell 从相应真实 window/transaction/IMU subspaces 调用 production generator 后再
进入 plausible evidence、action census、rank/post/PL/FDE。
