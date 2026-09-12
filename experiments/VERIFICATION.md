# 2026-09-12 基础设施验证记录

## 五数据集 inventory v2

用户恢复 `0912-ICRA-FIVE-DATASET-VALIDATION` 后，实际完成 39 条 recording 的本地内容盘点与 manifest v2。完整范围、异常和 PASS 边界见 [DATA_VALIDATION.md](DATA_VALIDATION.md)。

| 实际命令 | 退出码 | 产物/结论 |
|---|---:|---|
| `python3 experiments/scripts/inspect_dataset_metadata.py --recover-unindexed > experiments/datasets/five_dataset_metadata_observed.json` | 0 | 五 family 共 39 条 recording；只读 bag/CSV 结构、身份、选定字段有限性和时间检查；原始数据未修改 |
| `python3 experiments/scripts/build_dataset_manifest.py` | 0 | 生成 `icra_dataset_manifest_v2`，39 条实际 recording、无模板 |
| `python3 experiments/scripts/validate_manifest.py --check-hashes` | 0 | 0 errors、42 warnings；所有主/支持文件 SHA-256 一致；`FIVE_FAMILY_MANIFEST_VALIDATION=PASS` |
| `python3 experiments/scripts/test_validate_manifest.py` | 0 | 15/15 validator 工程反例通过 |
| `python3 -m py_compile experiments/scripts/{inspect_dataset_metadata,build_dataset_manifest,validate_manifest,test_validate_manifest}.py` | 0 | 四个基础设施脚本语法通过 |

权威日志：[validation_20260912_v2](datasets/validation_20260912_v2/)。36 条内容验证无本地异常；3 条带已登记问题：一条 HUEC 辅助 RSSI 为 `-inf`，两个 own_vicon 原 bag 无 index、只在临时 reindex 副本恢复扫描。所有 39 条仍 `NOT_ADMITTED`；6 DEV、33 UNASSIGNED、0 VAL/TEST。

成功前 inspector 两次实际 exit 1（SFUISE 实际字段名、无 header anchor list），validator/py_compile/tests 一次实际 exit 1（Python 条件缺右括号）；修复基础设施脚本后完整重跑。未覆盖或修改这些失败事实。

本轮 estimator、CUSUM、NLOS recovery、核心 build/CTest、batch、accuracy 全部 `NOT_RUN`。没有修改 `src/`、`include/`、`config/`、核心 `tools/` 或参数；没有提交/push。

## 初始 manifest v1（历史记录）

开始 HEAD：`439282c2790ab1414ca9b276dd61a18b5e1643a8`；分支 `feature/uwb-imu-fusion-ie-postprocessing`。
开始时 STATUS.md、CLAIM_EVIDENCE.md 已有未提交改动，并有用户未跟踪文档；原内容保留，
本轮只在两个索引增补任务记录。受保护 doc/v2/ie_0911/ 未读取或修改。

以下命令从仓库根执行，未安装/升级依赖：

| 实际命令 | 退出码 | 产物/结论 |
|---|---:|---|
| `python3 experiments/scripts/inspect_sfuise_metadata.py > experiments/datasets/isas_metadata_observed.json` | 0 | 三个 ISAS bag 只读元数据/完整文件 SHA-256；无 estimator 或指标 |
| `python3 experiments/scripts/validate_manifest.py --check-hashes > experiments/datasets/validation_20260912/static.log 2>&1` | 0 | [static.log](datasets/validation_20260912/static.log)：0 errors、7 明确未准入/模板警告 |
| `python3 experiments/scripts/test_validate_manifest.py > experiments/datasets/validation_20260912/tests.log 2>&1` | 0 | [tests.log](datasets/validation_20260912/tests.log)：12/12 工程测试通过 |
| `git diff --check` | 0 | tracked patch whitespace 检查通过 |

首次无重定向的相同 validator 与测试也各 exit 0；随后记录日志，没有失败后参数调整。
工具测试仅创建临时假文件/内存 manifest，不修改真实数据。SHA 核对不解析 GT 或运行算法。
测试涵盖绝对测量枚举、TDoA/unknown 拒绝、每个必填字段、重复 identity、split 泄漏、
开发记录误标 TEST、模板伪装 verified、UNKNOWN 填非空、错误单位、非有限值、JSON 重复键、
丢失文件/损坏软链接和 hash 不符。

本轮 estimator / CUSUM / NLOS recovery / build / CTest / batch experiments / accuracy evaluation：
全部 `NOT_RUN`。本轮没有修改 src/、include/、config/、tools/ 或核心测试；没有提交/push。
本地元数据盘点不是适配器测试或实验通过，3 条 recording 全是 DEV/NOT_ADMITTED；
4 个 family 模板不等于新增已验证数据集。C1–C3、T10=C2-C、T11=C 保持原证据边界。
