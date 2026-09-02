#!/usr/bin/env python3
"""Generate dependency-free Chinese Markdown/HTML reports from raw v1/v2 runs."""

import argparse
import csv
import html
import json
import math
import pathlib
import statistics
from collections import Counter, defaultdict


def rows(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(row, *names):
    for name in names:
        if name in row:
            try:
                return float(row[name])
            except ValueError:
                return math.nan
    return math.nan


def percentile(values, fraction):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    position = fraction * (len(values) - 1)
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (position - lower) * (values[upper] - values[lower])


def fmt(value):
    return "—" if not math.isfinite(value) else f"{value:.6g}"


def polyline_svg(series, title, width=760, height=220):
    clean = [(x, y) for x, y in series if math.isfinite(x) and math.isfinite(y)]
    if len(clean) < 2:
        return (f'<svg viewBox="0 0 {width} {height}" role="img" '
                f'aria-label="{html.escape(title)}"><text x="20" y="40">'
                '无足够原始数据</text></svg>')
    xs, ys = zip(*clean)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if xmax == xmin:
        xmax += 1
    if ymax == ymin:
        ymax += 1
    points = " ".join(
        f"{30 + (x-xmin)/(xmax-xmin)*(width-50):.2f},"
        f"{10 + (ymax-y)/(ymax-ymin)*(height-35):.2f}" for x, y in clean)
    return (f'<svg viewBox="0 0 {width} {height}" role="img" '
            f'aria-label="{html.escape(title)}"><rect width="100%" height="100%" '
            'fill="#fff"/><path d="M30 10V195H740" fill="none" stroke="#777"/>'
            f'<polyline points="{points}" fill="none" stroke="#1769aa" '
            'stroke-width="2"/><text x="35" y="215" font-size="12">'
            f'{html.escape(title)}</text></svg>')


def analyze(run):
    manifest = json.loads((run / "run_manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8")) \
        if (run / "summary.json").exists() else {}
    integrity = rows(run / "integrity.csv")
    timing = rows(run / "timing.csv")
    states = rows(run / "states.csv")
    availability = Counter(row.get("availability", "UNKNOWN") for row in integrity)
    committed = sum(row.get("batch_committed", "0") in ("1", "true", "True")
                    for row in integrity)
    stage_values = defaultdict(list)
    for row in timing:
        stage_values[row.get("stage", "unknown")].append(number(row, "wall_ms"))
    timing_stats = []
    for stage in sorted(stage_values):
        values = [value for value in stage_values[stage] if math.isfinite(value)]
        if values:
            timing_stats.append((stage, len(values), statistics.fmean(values),
                                 percentile(values, .50), percentile(values, .95),
                                 percentile(values, .99), max(values)))
    pl_series = []
    for row in integrity:
        pl_series.append((number(row, "timestamp_ns") * 1e-9,
                          number(row, "hpl_m")))
    state_series = [(number(row, "timestamp_ns") * 1e-9, number(row, "px"))
                    for row in states]
    return manifest, summary, integrity, availability, committed, timing_stats, \
        pl_series, state_series


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=pathlib.Path)
    parser.add_argument("--markdown", type=pathlib.Path, required=True)
    parser.add_argument("--html", dest="html_path", type=pathlib.Path, required=True)
    args = parser.parse_args()
    (manifest, summary, integrity, availability, committed, timing_stats,
     pl_series, state_series) = analyze(args.run_directory)
    timing_md = "\n".join(
        f"| {stage} | {count} | {fmt(mean)} | {fmt(p50)} | {fmt(p95)} | "
        f"{fmt(p99)} | {fmt(maximum)} |"
        for stage, count, mean, p50, p95, p99, maximum in timing_stats)
    if not timing_md:
        timing_md = "| 无数据 | 0 | — | — | — | — | — |"
    availability_md = ", ".join(f"{key}={value}" for key, value in availability.items()) or "无数据"
    missing = []
    for artifact in ("h0.csv", "noncentral.csv", "slope_oracle.csv",
                     "method_ab.csv", "roc.csv"):
        if not (args.run_directory / artifact).exists():
            missing.append(artifact)
    md = f"""# UWB/IMU 完整性研究报告

> 非认证声明：本报告及其保护级输出仅用于研究验证，不构成任何安全认证或运行授权。

## 可追溯信息

| 字段 | 值 |
|---|---|
| schema | {manifest.get('schema_version', 'unknown')} |
| git SHA | {manifest.get('git_sha', 'unknown')} |
| dirty | {manifest.get('git_dirty', 'unknown')} |
| config hash | {manifest.get('config_hash', 'unknown')} |
| seed | {manifest.get('seed', 'unknown')} |
| 创建时间 | {manifest.get('created_utc', 'unknown')} |

## 在线结果

共记录 {len(integrity)} 个完整性历元，提交 {committed} 个；availability：{availability_md}。
结构化 summary：processed={summary.get('processed', '—')}，committed={summary.get('committed', '—')}，rejected={summary.get('rejected', '—')}，errors={summary.get('errors', '—')}。

## Timing

| stage | count | mean ms | P50 | P95 | P99 | max |
|---|---:|---:|---:|---:|---:|---:|
{timing_md}

## 图表

HTML 版本内嵌 HPL 与轨迹 x 分量 SVG。图表直接读取 raw CSV 生成。

## 未执行或缺失的门禁原始数据

{', '.join(missing) if missing else '无'}。缺失项不会被提升为 PASS；需运行对应 Monte Carlo、ROC、oracle 和 Method A/B 工具后再生成最终研究结论。
"""
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(md, encoding="utf-8")
    timing_rows = "".join(
        "<tr>" + "".join(f"<td>{html.escape(str(value))}</td>" for value in
        (stage, count, fmt(mean), fmt(p50), fmt(p95), fmt(p99), fmt(maximum))) + "</tr>"
        for stage, count, mean, p50, p95, p99, maximum in timing_stats)
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>UWB/IMU 完整性研究报告</title>
<style>body{{font:15px system-ui;max-width:900px;margin:auto;padding:24px;color:#222}}table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #ccc;padding:6px}}.warn{{background:#fff3cd;padding:12px}}svg{{width:100%;height:auto;border:1px solid #ddd;margin:8px 0}}</style>
<h1>UWB/IMU 完整性研究报告</h1><p class="warn">非认证声明：仅用于研究验证，不构成安全认证或运行授权。</p>
<h2>可追溯信息</h2><table><tr><th>schema</th><td>{html.escape(str(manifest.get('schema_version')))}</td></tr><tr><th>git SHA</th><td>{html.escape(str(manifest.get('git_sha')))}</td></tr><tr><th>config hash / seed</th><td>{html.escape(str(manifest.get('config_hash')))} / {manifest.get('seed')}</td></tr></table>
<h2>结果</h2><p>完整性历元 {len(integrity)}；提交 {committed}；{html.escape(availability_md)}</p>
<h2>HPL timeline</h2>{polyline_svg(pl_series, 'HPL (m) vs time (s)')}
<h2>轨迹 x timeline</h2>{polyline_svg(state_series, 'x (m) vs time (s)')}
<h2>Timing</h2><table><tr><th>stage</th><th>count</th><th>mean</th><th>P50</th><th>P95</th><th>P99</th><th>max</th></tr>{timing_rows}</table>
<h2>缺失门禁</h2><p>{html.escape(', '.join(missing) if missing else '无')}</p></html>"""
    args.html_path.parent.mkdir(parents=True, exist_ok=True)
    args.html_path.write_text(page, encoding="utf-8")


if __name__ == "__main__":
    main()
