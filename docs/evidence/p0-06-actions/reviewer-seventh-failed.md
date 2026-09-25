P0-06 第七轮独立验收：**FAILED（可修复，非 BLOCKED）**。

阻断项：
1. **130-action核心O07仍绕过production generator。** completeSemanticActionFixture在test:233-377手工构造window/mode/unit/hypothesis/evidence和130 actions；主用例:1638-1658直调censusAndCapActionsV1，无HypothesisGenerator::generate/actionsForPlausibleSetV1。129 recovery action额外删除人工KinematicBridge shard，group虽存在但非production generator从同一语义集合产生。
2. **独立raw-factor oracle未从raw H/z/cov重建。** independentDenseActionOracle只拼jacobian_whitened/residual_whitened，不读raw/cov/whitener；fixture covariance全单位阵，无法发现whitening错误；README raw声明不实。后续SVD/cov/parity/Boost PL本身独立。
3. **golden ABI遗漏并实际破坏公开RealtimeIntegrityPipeline布局。** 新test_dependency_seams_成员令sizeof golden5056→current5072，align16。现有ABI client未覆盖pipeline。

通过：normal clean/full/tests、directed24/24、inherited、CTest29/29/round2、54-cell真实链/router fallback、pipeline duplicate三异常、既有ABI clients、13-path patch、五hash均PASS；无合同/配置变化。

最小修复：130从真实production generator/action seam产生全部130并同一universe贯穿；独立oracle从correlated raw covariance/raw H/raw z自己whitening；test seam移到ABI-stable间接存储/外部registry并新增实例化pipeline的golden-header canary。Reviewer未修改树/总表/commit/tag。
