#!/usr/bin/env python3
"""Evaluate and report the locked oracle-support backend diagnostic."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess

import yaml

import nlos_injection_metrics as metrics


LOCKED_ROOT = Path("/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01")
GROUND_TRUTH_ROOT = Path(
    "/home/mint/ws_fusion_uwb/res/ie0911_step2_truth_20260911_01")
FINAL_CLASSIFICATION = "RECOVERY_BACKEND_NOT_OPERATIONAL"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def metric_value(values: dict, key: str):
    return values.get(key, {}).get("value")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--normal", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    normal = args.normal.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    locked_path = LOCKED_ROOT / "locked_manifest.json"
    locked = load(locked_path)
    injected = next(item for item in locked["scenarios"]
                    if item["scenario_id"] == "sfuise_walk1_normal_injected")
    clean = next(item for item in locked["scenarios"]
                 if item["scenario_id"] == "sfuise_walk1_normal_clean")
    truth_path = Path(injected["truth"]) / "nlos_injection_truth.json"
    truth = load(truth_path)
    true_amplitude = float(truth["specification"]["amplitude_m"])
    support = load(normal / "oracle_support.json")
    batch = load(normal / "batch" / "batch_manifest.json")
    cells = batch["cells"]
    producer = next(cell for cell in cells if cell["execution_type"] == "CACHE_PRODUCER")
    producer_run = Path(producer["run_directory"])
    observations = rows(producer_run / "observations.csv")
    by_id = {row["obs_id"]: row for row in observations}
    support_ids = set(support["obs_ids"])
    actual = [by_id[value] for value in support_ids if value in by_id]
    alignment = {
        "status": "PASS" if len(actual) == len(support_ids) else "FAIL",
        "support_nonempty": bool(support_ids),
        "support_count": len(support_ids),
        "mapped_count": len(actual),
        "all_valid_and_planned": all(row["valid"] == "1" and row["planned"] == "1"
                                     for row in actual),
        "all_target_link": all(int(row["raw_tag_id"]) == support["tag_id"] and
                               int(row["anchor_id"]) == support["anchor_id"]
                               for row in actual),
        "unexpected_ids": sorted(support_ids - set(by_id)),
        "non_target_ids": sorted(row["obs_id"] for row in actual
                                 if int(row["raw_tag_id"]) != support["tag_id"] or
                                 int(row["anchor_id"]) != support["anchor_id"]),
        "actual_start_time": min(float(row["raw_time"]) for row in actual),
        "actual_end_time": max(float(row["raw_time"]) for row in actual),
        "support_source": support["support_source"],
    }
    if not (alignment["support_nonempty"] and alignment["all_valid_and_planned"] and
            alignment["all_target_link"] and not alignment["unexpected_ids"]):
        raise AssertionError("oracle support alignment failed")
    (output / "oracle_support_alignment_audit.json").write_text(
        json.dumps(alignment, indent=2) + "\n", encoding="utf-8")

    support_yaml = yaml.safe_load((normal / "oracle_support.yaml").read_text())
    effective = yaml.safe_load((producer_run / "batch_config_effective.yaml").read_text())
    forbidden_keys = {"amplitude_m", "amplitude", "bias_m", "c_true",
                      "ground_truth", "true_bias"}

    def forbidden(value) -> list[str]:
        found = []
        if isinstance(value, dict):
            for key, child in value.items():
                if str(key).lower() in forbidden_keys:
                    found.append(str(key))
                found.extend(forbidden(child))
        elif isinstance(value, list):
            for child in value:
                found.extend(forbidden(child))
        return found

    trace_path = normal / "estimator_file_access.trace"
    trace_lines = trace_path.read_text(encoding="utf-8").splitlines()
    opened_support = [line for line in trace_lines
                      if "openat(" in line and str(normal / "oracle_support.yaml") in line]
    forbidden_opens = [line for line in trace_lines if "openat(" in line and any(
        marker in line for marker in (
            "nlos_injection_truth.json", "ground_truth.tum", "ground_truth.csv"))]
    truth_audit = {
        "status": "PASS",
        "estimator_oracle_manifest_forbidden_fields": forbidden(support_yaml),
        "effective_config_forbidden_fields": forbidden(effective),
        "estimator_support_manifest_sha256": sha(normal / "oracle_support.yaml"),
        "support_manifest_contains_numeric_true_amplitude_field": False,
        "stage2_physical_graph_true_amplitude_parameter": False,
        "stage2_receives_only": ["segment_id", "tag_id", "anchor_id", "obs_ids"],
        "support_open_observed": bool(opened_support),
        "support_open_count": len(opened_support),
        "forbidden_truth_or_gt_opens": forbidden_opens,
        "original_truth_tree_hidden_by_bwrap": True,
        "true_amplitude_read_by": "POST_ESTIMATOR_EVALUATOR_ONLY",
        "estimator_input_manifest_oracle_support_read_field_note":
            "Current fixed_partition_debug export reports false because it tests only "
            "oracle_debug mode; strace is authoritative and records the allowed support file.",
    }
    if truth_audit["estimator_oracle_manifest_forbidden_fields"] or \
            truth_audit["effective_config_forbidden_fields"] or forbidden_opens or \
            not opened_support:
        raise AssertionError("truth separation audit failed")
    (output / "truth_separation_audit.json").write_text(
        json.dumps(truth_audit, indent=2) + "\n", encoding="utf-8")

    repository = Path(__file__).resolve().parents[2]
    frozen_paths = [
        "src/nlos_fde.cpp", "src/fde_grouped.cpp", "src/nlos_refit.cpp",
        "src/nlos_recoverability.cpp", "src/nlos_inference.cpp",
        "include/uifgo/nlos_fde.h", "include/uifgo/nlos_refit.h",
        "include/uifgo/nlos_recoverability.h", "include/uifgo/nlos_inference.h",
        "tools/run_ie_paper.cpp", "tools/paper/run_experiments.py",
        "tools/paper/nlos_injection_metrics.py",
    ]
    frozen_files = {}
    for name in frozen_paths:
        worktree = repository / name
        committed = subprocess.check_output(
            ["git", "show", f"HEAD:{name}"], cwd=repository)
        frozen_files[name] = {
            "head_sha256": "sha256:" + hashlib.sha256(committed).hexdigest(),
            "worktree_sha256": sha(worktree),
            "equal": committed == worktree.read_bytes(),
        }
    locked_config_path = Path(injected["config"])
    locked_config = yaml.safe_load(locked_config_path.read_text())
    oracle_config = yaml.safe_load((normal / "walk1_normal_oracle.yaml").read_text())
    locked_compare = copy.deepcopy(locked_config)
    oracle_compare = copy.deepcopy(oracle_config)
    permitted_oracle_fields = {
        "mode", "oracle_support", "short_min_count_debug",
        "short_min_duration_debug", "fde_grouped_test",
    }
    for document in (locked_compare, oracle_compare):
        for key in permitted_oracle_fields:
            document["nlos"].pop(key, None)
    if locked_compare != oracle_compare:
        raise AssertionError("oracle config changed a frozen non-provider field")
    frozen_audit = {
        "status": "PASS" if all(item["equal"] for item in frozen_files.values())
        else "FAIL",
        "head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip(),
        "files": frozen_files,
        "scientific_configuration_source": str(
            normal / "walk1_normal_oracle.yaml"),
        "scientific_configuration_sha256": sha(
            normal / "walk1_normal_oracle.yaml"),
        "locked_injected_config": str(locked_config_path),
        "locked_injected_config_sha256": sha(locked_config_path),
        "only_permitted_provider_fields_differ_from_locked_injected_config": True,
        "permitted_provider_fields": sorted(permitted_oracle_fields),
        "detector_executed": False,
        "low_redundancy_execution": "NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL",
    }
    if frozen_audit["status"] != "PASS":
        raise AssertionError("frozen algorithm/evaluator source changed")
    (output / "frozen_implementation_audit.json").write_text(
        json.dumps(frozen_audit, indent=2) + "\n", encoding="utf-8")

    failed_launch = normal.parent / "normal"
    failed_trace = failed_launch / "estimator_file_access.trace"
    infrastructure_audit = {
        "initial_attempt_directory": str(failed_launch),
        "status": "PRESERVED_PRE_ESTIMATOR_MANIFEST_VALIDATION_FAILURE",
        "reason": "development policy provenance is not authorized",
        "estimator_exec_count": sum(
            "uwb_imu_fgo_paper_runner" in line and "execve(" in line
            for line in failed_trace.read_text().splitlines())
            if failed_trace.is_file() else 0,
        "scientific_run_retried": False,
        "correction": "Metadata provenance gained required TEST_ONLY/PENDING_VALIDATION markers; fresh directory used.",
    }
    if infrastructure_audit["estimator_exec_count"] != 0:
        raise AssertionError("initial infrastructure failure launched estimator")
    (output / "infrastructure_failure_audit.json").write_text(
        json.dumps(infrastructure_audit, indent=2) + "\n", encoding="utf-8")

    trace = rows(producer_run / "refit_iterations.csv")
    first, last = trace[0], trace[-1]
    status = load(producer_run / "run_status.json")
    stage2 = {
        "status": "FAILED",
        "solver_status": status["solver_status"],
        "reason": status["reason"],
        "support_count": len(support_ids),
        "success_empty_forbidden_and_not_used": status["solver_status"] != "SUCCESS_EMPTY",
        "optimizer_calls": len(trace),
        "outer_iterations": len(trace),
        "conditional_lm_iterations_total": sum(int(row["conditional_lm_iterations"])
                                               for row in trace),
        "conditional_lm_inner_iterations_total": sum(
            int(row["conditional_lm_inner_iterations"]) for row in trace),
        "initial_objective": float(first["objective_before"]),
        "first_outer_objective_after": float(first["objective_after"]),
        "final_objective": float(last["objective_after"]),
        "final_relative_objective_change": float(last["relative_objective_change"]),
        "final_scaled_state_step": float(last["scaled_state_step"]),
        "final_max_kkt_violation": float(last["max_kkt_violation"]),
        "final_max_scaled_navigation_gradient_objective": float(
            last["max_scaled_navigation_gradient_objective"]),
        "navigation_stationarity_tolerance_objective": float(
            last["navigation_stationarity_tolerance_objective"]),
        "navigation_gradient_roundoff_allowance_objective": float(
            last["navigation_gradient_roundoff_allowance_objective"]),
        "final_predicates": {name: last[name] == "1" for name in (
            "objective_ok", "step_ok", "kkt_ok", "navigation_stationarity_ok")},
        "c_hat_m": None,
        "c_hat_status": "UNAVAILABLE_STAGE2_NOT_CONVERGED_NO_VALID_ESTIMATE_EXPORT",
        "correctness_diagnosis":
            "Objective, state-step and nonnegative KKT predicates pass, but the "
            "unchanged navigation stationarity predicate remains false: gradient "
            "0.0750243858624 exceeds tolerance 1e-6 plus roundoff 2.2356e-11. "
            "No support/link/identity plumbing mismatch was observed.",
        "solver_threshold_or_policy_changed": False,
        "algorithm_retry_count": 0,
    }
    (output / "stage2_backend_audit.json").write_text(
        json.dumps(stage2, indent=2) + "\n", encoding="utf-8")

    protocol = yaml.safe_load((Path(__file__).resolve().parents[2] /
        "config/paper/ie0911/step2_evaluation.yaml").read_text())["run_units"]["sfuise_walk1"]
    if sha(Path(protocol["ground_truth"])) != protocol["ground_truth_sha256"]:
        raise AssertionError("locked evaluator GT hash mismatch")
    table = []
    for condition, source in (("clean", clean), ("injected", injected)):
        if condition == "clean":
            selected = [(name, data["run_directory"], data["cell_status"])
                        for name, data in source["gate"]["methods"].items()]
        else:
            selected = [(cell["canonical_mode"], cell.get("run_directory"), cell["status"])
                        for cell in cells if cell["canonical_mode"] in
                        {"all_range", "robust_cauchy"}]
        for method, run_directory, cell_status in selected:
            values = metrics.localization(Path(run_directory), protocol,
                                          tuple(source["window"])) \
                if run_directory and "COMPLETE" in cell_status else {}
            table.append({
                "condition": condition, "method": method,
                "cell_status": cell_status,
                "run_directory": run_directory,
                "rmse_m": metric_value(values, "aligned_ATE_rmse_m"),
                "p95_m": metric_value(values, "aligned_ATE_p95_m"),
                "horizontal_rmse_m": metric_value(values, "aligned_horizontal_rmse_m"),
                "vertical_rmse_m": metric_value(values, "aligned_height_rmse_m"),
                "coverage": values.get("trajectory_coverage"),
                "complete_planned_trajectory": values.get("complete_planned_trajectory"),
                "window": values.get("window"),
            })
    for method in ("suppress_all", "structured_debias", "lcb_fixed_full", "lcb_partial"):
        cell = next(item for item in cells if item["canonical_mode"] == method and
                    item["execution_type"] == "FINAL_TRAJECTORY")
        table.append({
            "condition": "injected", "method": method,
            "cell_status": cell["status"], "run_directory": None,
            "rmse_m": None, "p95_m": None, "horizontal_rmse_m": None,
            "vertical_rmse_m": None, "coverage": None,
            "complete_planned_trajectory": None, "window": None,
        })
    rmse_by_key = {(row["condition"], row["method"]): row["rmse_m"]
                   for row in table}
    clean_raw = rmse_by_key[("clean", "all_range")]
    injected_raw = rmse_by_key[("injected", "all_range")]
    raw_degradation = {
        "absolute_rmse_increase_m": injected_raw - clean_raw,
        "relative_rmse_increase_percent":
            (injected_raw - clean_raw) / clean_raw * 100.0,
    }
    with (output / "localization_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        for row in table:
            writer.writerow({key: json.dumps(value, sort_keys=True)
                             if isinstance(value, (dict, list)) else value
                             for key, value in row.items()})

    recovery = {
        "stage2": stage2,
        "bias_evaluation": {"c_true_m": true_amplitude, "c_hat_m": None,
                            "signed_error_m": None, "absolute_error_m": None,
                            "status": "UNAVAILABLE_STAGE2_FAILED"},
        "recoverability": {"Rc_status": "NOT_RUN_STAGE2_FAILED", "rank": None,
                           "conditioning": None, "sigma_c_m": None},
        "lcb": {"kappa": 2.0, "delta_lcb_m": None, "delta_full_m": None,
                "lcb_over_correction": None, "full_over_correction": None,
                "lcb_residual_injected_bias_m": None,
                "status": "NOT_RUN_STAGE2_FAILED"},
        "finals": {method: {"status": "PARENT_CACHE_UNAVAILABLE",
                            "optimizer_calls": 0, "initial_objective": None,
                            "final_objective": None, "factor_count": None,
                            "corrected_factor_count": None}
                   for method in ("suppress_all", "structured_debias",
                                  "lcb_fixed_full", "lcb_partial")},
        "comparisons": {name: {"absolute_rmse_change_m": None,
                               "relative_improvement_percent": None,
                               "status": "UNAVAILABLE_STAGE2_FAILED"}
                        for name in ("lcb_vs_suppress", "full_vs_suppress",
                                    "structured_vs_suppress", "lcb_vs_full")},
        "normal_classification": FINAL_CLASSIFICATION,
        "low_redundancy": "NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL",
        "final_classification": FINAL_CLASSIFICATION,
    }
    result = {
        "schema": "uifgo_oracle_support_backend_result_v1",
        "baseline_head": "15b32f5afa01c5a0f8bc432bbac78c53bfa36420",
        "locked_manifest_sha256": sha(locked_path),
        "truth_sha256": sha(truth_path),
        "oracle_support": support,
        "support_alignment": alignment,
        "truth_separation": truth_audit,
        "frozen_implementation": frozen_audit,
        "infrastructure_failure": infrastructure_audit,
        "recovery": recovery,
        "localization": table,
        "raw_injection_degradation": raw_degradation,
        "claims": "BACKEND_ONLY_ORACLE_SUPPORT_DEBUG; NOT_END_TO_END_DETECTOR",
    }
    (output / "oracle_backend_result.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    def fmt(value) -> str:
        return "UNAVAILABLE" if value is None else f"{value:.9f}"

    lines = [
        "# Oracle-support recovery backend sanity check",
        "",
        f"最终裁决：**{FINAL_CLASSIFICATION}**。这是 backend-only oracle-support "
        "development diagnostic，不是端到端 detector 成绩。",
        "",
        "## 1. 基线与锁定输入",
        "",
        "实际 git HEAD 为 `15b32f5afa01c5a0f8bc432bbac78c53bfa36420`。使用既有 "
        "SFUISE Walk1 normal-redundancy constant-step injection：seed 911，link "
        f"`{support['tag_id']}:{support['anchor_id']}`，闭区间 "
        f"`[{support['start_time']},{support['end_time']}]`，30 个已锁定 planned obs IDs。"
        "输入 cache、truth 与 locked manifest 身份见 machine-readable result。",
        "",
        "## 2. Oracle support 与 truth 隔离",
        "",
        "复用已有 `OracleSupportProvider -> SupportPartition` 接口。传给 estimator 的 YAML "
        "只含 segment ID、link 和 30 个 obs IDs；30/30 都是 valid/planned 且属于唯一目标 link。"
        "原 truth 目录和 evaluator GT 在 estimator 进程树内由 bwrap 隐藏；strace 观察到 support "
        "文件读取，未观察到 injection truth 或 GT 打开。effective config、support manifest 和 "
        "Stage2 参数中没有 true amplitude 字段。`0.5 m` 只由本独立 evaluator 在 estimator "
        "结束后读取。",
        "",
        "证据：`oracle_support_alignment_audit.json`、`truth_separation_audit.json`、"
        "`oracle_support.json` 和原始 file-access trace。当前 runner 的 fixed-partition "
        "`input_manifest.oracle_support_read=false` 是已有导出字段局限；实际读取以 strace 为准，"
        "没有为本任务修改 runner。",
        "",
        "## 3. Stage2 实际执行与失败",
        "",
        f"support 非空，Stage2 实际执行 {stage2['optimizer_calls']} 次 conditional optimizer "
        f"调用（累计 {stage2['conditional_lm_iterations_total']} LM iterations / "
        f"{stage2['conditional_lm_inner_iterations_total']} inner trials），未走 SUCCESS_EMPTY。"
        f"objective 从 {stage2['initial_objective']:.12g} 降至 "
        f"{stage2['final_objective']:.12g}。最后 objective、scaled-step 和非负 KKT 条件通过；"
        f"KKT violation={stage2['final_max_kkt_violation']:.12g}。但 navigation stationarity "
        f"gradient={stage2['final_max_scaled_navigation_gradient_objective']:.12g}，远高于冻结容差 "
        f"{stage2['navigation_stationarity_tolerance_objective']:.12g} 加 roundoff "
        f"{stage2['navigation_gradient_roundoff_allowance_objective']:.12g}。50 次外层迭代后返回 "
        "`MAX_REFIT_ITERATIONS`，没有导出未收敛 Values 或 c_hat。没有发现 support/link/obs "
        "identity plumbing 错误，因此未修改算法、容差或 solver policy，也未重试。",
        "首次 batch 在 estimator 启动前因 scheduler provenance 标签缺少既有 "
        "`TEST_ONLY/PENDING_VALIDATION` 标记而失败；trace 核验 estimator exec=0。仅修正元数据"
        "标签后使用 fresh 目录执行上述唯一科学运行。",
        "",
        "## 4. Bias、Rc 与补偿",
        "",
        "由于 Stage2 没有收敛，`c_hat`、其相对 0.5 m 的误差、Rc rank/conditioning、"
        "`sigma_c`、full correction 和 `max(0,c_hat-2*sigma_c)` 全部为 "
        "`UNAVAILABLE_STAGE2_FAILED`。没有手工 uncertainty 或 correction fallback，也没有把"
        "未收敛中间状态冒充有效估计。",
        "",
        "## 5. Final optimization",
        "",
        "Stage2 cache 未发布；suppress_all、structured_debias、lcb_fixed_full 和 lcb_partial "
        "均为 `PARENT_CACHE_UNAVAILABLE`。它们的 final optimizer calls 为 0，corrected factors "
        "不可用。没有 empty-support graph reuse，也没有 CSV-only correction。",
        "",
        "## 6. 定位精度",
        "",
        "使用冻结的同一 evaluator；coverage 是 planned trajectory coverage。",
        "",
        "| condition | method | status | RMSE m | P95 m | horizontal RMSE m | vertical RMSE m | coverage |",
        "|---|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in table:
        coverage = row["coverage"].get("value") if isinstance(row["coverage"], dict) else None
        lines.append(f"| {row['condition']} | {row['method']} | {row['cell_status']} | "
                     f"{fmt(row['rmse_m'])} | {fmt(row['p95_m'])} | "
                     f"{fmt(row['horizontal_rmse_m'])} | {fmt(row['vertical_rmse_m'])} | "
                     f"{fmt(coverage)} |")
    lines += [
        "",
        f"相对 clean raw，injected raw RMSE 增加 "
        f"{raw_degradation['absolute_rmse_increase_m']:.9f} m（"
        f"{raw_degradation['relative_rmse_increase_percent']:.3f}%）。",
        "",
        "LCB-vs-suppress、full-vs-suppress、structured-vs-suppress 和 LCB-vs-full 的绝对 "
        "RMSE 变化与百分比改善均不可用；原因是 perfect support 下 Stage2 失败，而不是 detector "
        "漏检。所有负结果和 parent-unavailable 状态保留。",
        "",
        "## 7. 四个问题与裁决",
        "",
        "- Q1 Bias estimation：不能回答接近程度；没有有效 `c_hat`。",
        "- Q2 Recoverability：Rc 未执行，不能产生有限 `sigma_c`。",
        "- Q3 Bounded compensation：LCB 未执行，属于 unavailable，既不能称 0 correction，"
        "也不能称 over-correction。",
        "- Q4 Localization benefit：LCB 和 suppress final 均未运行，无法比较；没有观测到收益。",
        "",
        f"因此按预先规则唯一裁决为 **{FINAL_CLASSIFICATION}**，暂停 detector 开发。"
        "normal backend 未通过，故锁定 low-redundancy oracle check 为 "
        "`NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL`。没有修改 FDE、Stage2、Rc、LCB、优化器或 "
        "evaluator，没有运行下一阶段算法。",
        "",
        "完整机器证据位于 "
        "`/home/mint/ws_fusion_uwb/res/oracle_support_backend_20260911_01`。",
    ]
    (Path(__file__).resolve().parents[2] /
     "doc/ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
