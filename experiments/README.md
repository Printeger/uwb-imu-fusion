# ICRA 实验基础设施使用说明

数据合同和准入边界见 [DATASET_MANIFEST.md](DATASET_MANIFEST.md)。当前 v2 inventory 有 39 条实际 recording，覆盖 HUEC、MILUV、own_vicon、SFUISE 和 starloc；没有正式 VAL/TEST，所有 recording 仍为 `NOT_ADMITTED`。

目录用途：

- `datasets/`：manifest、schema 和只读内容盘点证据；
- `configs/`：未来锁定的实验配置；
- `scripts/`：metadata inspector、manifest builder 和静态 validator；
- `results/`：按 run ID 隔离的运行结果；
- `figures/`：仅由锁定结果生成的图。

从仓库根执行日常静态验证：

```bash
# schema、必填字段、TDoA 拒绝、split/开发暴露和身份语义
python3 experiments/scripts/validate_manifest.py

# 加上所有主/支持文件和软链接存在性检查
python3 experiments/scripts/validate_manifest.py --check-paths

# 加上 metadata artifact 与所有记录文件的只读 SHA-256 核对
python3 experiments/scripts/validate_manifest.py --check-hashes

# validator 工程反例；不运行 estimator
python3 experiments/scripts/test_validate_manifest.py
```

返回码：0 表示 manifest 静态一致，可含已登记 warning；1 表示 schema/语义/路径/hash 失败；2 表示 JSON、文件或依赖读取错误。成功输出同时包含：

```text
FIVE_FAMILY_MANIFEST_VALIDATION=PASS
EXPERIMENT_ADMISSION=NOT_GRANTED
ESTIMATOR=NOT_RUN
```

重新生成本地内容证据和 manifest 时按以下顺序执行。metadata inspector 需要当前 ROS Python `rosbag` 环境；对两个无索引 own_vicon bag，`--recover-unindexed` 只复制到系统临时目录并 reindex 临时副本，退出后删除临时副本，原始 `data/` 不变。

```bash
python3 experiments/scripts/inspect_dataset_metadata.py \
  --recover-unindexed \
  > experiments/datasets/five_dataset_metadata_observed.json

python3 experiments/scripts/build_dataset_manifest.py
python3 experiments/scripts/validate_manifest.py --check-hashes
```

`build_dataset_manifest.py` 是显式维护命令，只重写 `dataset_manifest.json`；validator 从不修改 manifest 或数据。不要把 inspector、builder 或 validator 当作 adapter/estimator 测试。不要将 UNKNOWN 自动替换为 0、false 或 identity，也不要因静态 PASS 分配 VAL/TEST。

当前 manifest 直接引用仓库 `data/`，不复制数据。若原始文件以后迁移到外部盘，可在原 `data/<family>/...` 路径放置软链接，但目标内容 SHA-256 必须一致。正式运行须另立准入协议并使用唯一 `experiments/results/<run_id>/`；不得使用共享 `latest` 作为结果数据库。

实际执行记录见 [DATA_VALIDATION.md](DATA_VALIDATION.md) 和 [VERIFICATION.md](VERIFICATION.md)。本任务没有修改核心算法或参数，也没有启动批量实验。

## Evaluator-only per-range error

[几何/标定合同、物理隔离与结果](RANGE_EVALUATOR.md)。
新增 evaluate_range_gt.py；真实 estimator 调用已强制 bwrap 白名单隔离。
GT/派生 range CSV 只保存在工作空间 evaluator_private/icra 下，不进入 estimator cache/runs。
官方 SFUISE 已确认 lever 和 beta 的符号/ID 映射；GT→anchor、marker→IMU 与时钟关系仍缺失，
真实 per-range 结果当前是带完整 observation 行的 UNAVAILABLE_CALIBRATION，不是误差实验通过。

Canonical controlled positive-bias smoke：见 [CONTROLLED_INJECTION.md](CONTROLLED_INJECTION.md)，
固定Walk1/anchor20276/+1m/10s，clean与corrupted独立运行；无sweep。

SFUISE absolute-ToA baseline adapter、Walk1/2/3复现命令、统一主表与失败记录见
[SFUISE_BASELINE.md](SFUISE_BASELINE.md)。该路径拒绝TDoA，SFUISE运行时不播放GT，也不接收本方法
detector/recovery信息。
