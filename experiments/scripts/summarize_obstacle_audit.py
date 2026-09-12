#!/usr/bin/env python3
"""Summarize a completed evaluator run, without rerunning any measurement adapter."""
import argparse
from pathlib import Path
import json
import uuid
import numpy as np
import pandas as pd
from audit_obstacle_ranges import ROOT, PRIVATE, MILUV_COMMIT, sha, dump


def table(frame, fields):
    def fmt(x):
        if x is None or (isinstance(x, (float, np.floating)) and not np.isfinite(x)): return 'NA'
        if isinstance(x, (float, np.floating)): return '%.4f' % x
        return str(x)
    return '\n'.join(['| '+' | '.join(fields.values())+' |', '| '+' | '.join(['---']*len(fields))+' |'] +
                     ['| '+' | '.join(fmt(row.get(k)) for k in fields)+' |' for row in frame.to_dict('records')])


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('run', type=Path); args = parser.parse_args()
    run = args.run.resolve()
    if PRIVATE.resolve() not in run.parents: raise ValueError('only private evaluator runs allowed')
    execution = json.loads((run/'execution.json').read_text())
    if execution['exit_code'] or execution['input_changes']: raise ValueError('completed zero-failure run required')
    out = run/('report-'+uuid.uuid4().hex[:8]); out.mkdir(exist_ok=False)
    records = pd.DataFrame(json.loads((run/'recordings.json').read_text()))
    links = pd.read_csv(run/'link_summary.csv', dtype={'anchor_id':str, 'tag_id':str})
    anchors = pd.read_csv(run/'anchor_summary.csv', dtype={'anchor_id':str})
    geometry = json.loads((run/'geometry_audit.json').read_text())
    hashes = json.loads((run/'source_hashes.json').read_text())
    # Lock recorded input identities before interpreting metrics.
    assert all(sha(p) == v['sha256'] for p,v in hashes.items())
    extra = []; event_rows = []; association = []; checks = []
    for row in records.to_dict('records'):
        d = pd.read_csv(row['observation_path'], dtype={'anchor_id':str, 'tag_id':str})
        for col in ['error_m', 'time_s', 'raw_m']:
            d[col] = pd.to_numeric(d[col], errors='raise').astype(float)
        e = d.error_m.to_numpy(); finite = e[np.isfinite(e)]
        assert len(d) == row['sample_count']
        assert d.obs_id.is_unique
        assert len(finite) == row['error_sample_count']
        if len(finite):
            assert np.isclose(np.sqrt(np.mean(finite**2)), row['raw_range_error_rmse_m'], rtol=1e-12)
        extra.append({'family':row['family'], 'recording':row['recording'],
                      'error_min_m': np.min(finite) if len(finite) else None,
                      'error_max_m': np.max(finite) if len(finite) else None,
                      'negative_lt_minus1_count': int(sum(finite < -1)),
                      'nonfinite_timestamp_count': int(sum(~np.isfinite(d.time_s))),
                      'raw_invalid_nonpositive_count': int(sum(~np.isfinite(d.raw_m) | (d.raw_m <= 0)))})
        for status, count in d.association_status.value_counts().items():
            association.append({'family':row['family'], 'recording':row['recording'], 'status':status, 'count':int(count)})
        for link in links[(links.family==row['family']) & (links.recording==row['recording'])].to_dict('records'):
            if link['positive_gt_0.5_longest_duration_s'] < 2 or link['positive_gt_0.5_longest_count'] < 5: continue
            g = d[(d.tag_id==link['tag_id']) & (d.anchor_id==link['anchor_id'])]
            start, end = link['positive_gt_0.5_longest_start_s'], link['positive_gt_0.5_longest_end_s']
            event = g[(g.time_s>=start) & (g.time_s<=end)]
            assert len(event) == link['positive_gt_0.5_longest_count'] and (event.error_m > .5).all()
            assert event.time_s.diff().dropna().max() <= 1
            outside = g[(g.time_s<start) | (g.time_s>end)].error_m.dropna()
            event_rows.append({'family':row['family'], 'recording':row['recording'], 'tag_id':link['tag_id'], 'anchor_id':link['anchor_id'],
                               'start_relative_s':start-d.time_s.min(), 'end_relative_s':end-d.time_s.min(),
                               'duration_s':end-start, 'count':len(event), 'episode_mean_m':event.error_m.mean(),
                               'outside_episode_median_m':outside.median() if len(outside) else None,
                               'left_censored':start==g.time_s.min(), 'right_censored':end==g.time_s.max(),
                               'reference_status':row['reference_status']})
        checks.append({'family':row['family'], 'recording':row['recording'], 'counts_identity_rmse': 'PASS'})
    events = pd.DataFrame(event_rows).sort_values('duration_s', ascending=False)
    events.to_csv(out/'persistent_links.csv', index=False)
    pd.DataFrame(extra).to_csv(out/'extreme_and_invalid.csv', index=False)
    pd.DataFrame(association).to_csv(out/'association_status.csv', index=False)
    # Main deliverable NA semantics: no reference is not a zero-duration successful observation.
    for frame in [records, links, anchors]:
        mask = frame.error_sample_count == 0
        for key in frame.columns:
            if 'longest_' in key: frame.loc[mask, key] = np.nan
    records.to_csv(out/'recording_summary.csv', index=False)
    links.to_csv(out/'link_summary.csv', index=False); anchors.to_csv(out/'anchor_summary.csv', index=False)
    # Missing static files are not fabricated zero-error recordings.
    static_root = ROOT/'data/HUEC/Static_measurements'; missing = []
    for condition in ['LOS','NLOS']:
        for height in np.arange(50., 200.01, 12.5):
            folder = static_root/condition/('height_%.1fcm' % height)
            for distance in range(2,61,2):
                path = folder/('%dm.csv' % distance)
                if not path.exists(): missing.append(str(path.relative_to(ROOT)))
    dump(out/'missing_static_grid.json', {'expected_grid_size':780, 'absent_files':missing,
                                         'interpretation':'ABSENT_IN_LOCAL_RELEASE; not proof of RF loss'})
    static_geometry = [x for x in geometry if x['family']=='HUEC_static']
    counters = []
    for g in static_geometry:
        c = g['counters']; tx=c.get('Transmission #'); rx=c.get('Reception #')
        if not tx or not rx: continue
        delta_tx=tx['last']-tx['first'] if tx['last'] is not None and tx['first'] is not None else np.nan
        delta_rx=rx['last']-rx['first'] if rx['last'] is not None and rx['first'] is not None else np.nan
        valid = not tx['resets'] and not rx['resets'] and np.isfinite(delta_tx) and np.isfinite(delta_rx) and delta_tx>0 and 0<=delta_rx<=delta_tx
        counters.append({'recording':g['recording'], 'tx_delta':delta_tx, 'rx_delta':delta_rx,
                         'counter_increment_loss_fraction':1-delta_rx/delta_tx if valid else None,
                         'status':'COUNTER_INTERVAL_ONLY' if valid else 'UNAVAILABLE_OR_INCONSISTENT_COUNTERS',
                         'footer_rows_not_measurements':g['footer_rows_not_measurements']})
    pd.DataFrame(counters).to_csv(out/'static_counter_audit.csv', index=False)
    # Fixed same-family nominal / obstacle comparisons; no baseline subtraction from measurements.
    comparison = records[records.family.isin(['MILUV','own_vicon','HUEC_dynamic'])].copy()
    comparison['positive_gt_0.5_percent'] = 100*comparison['positive_gt_0.5_fraction']
    comparison.to_csv(out/'nominal_obstacle_comparison.csv', index=False)
    overview = records.groupby('family').agg(recordings=('recording','size'), raw_rows=('sample_count','sum'),
                valid_ranges=('valid_range_count','sum'), evaluated=('error_sample_count','sum')).reset_index()
    top = events.head(12)
    gtchecks = pd.DataFrame([g for g in geometry if g['family']=='MILUV'])
    huecchecks = pd.DataFrame([g for g in geometry if g['family']=='HUEC_dynamic'])
    own = [g for g in geometry if g['family']=='own_vicon']
    import yaml
    configured = {str(x['id']):x['pos'] for x in yaml.safe_load((ROOT/'config/vicon_test.yaml').read_text())['anchors']}
    own_deltas = [{'recording':g['recording'],'anchor_id':a,
                   'legacy_config_minus_vicon_norm_m':np.linalg.norm(np.asarray(configured[a])-s['median_m']),
                   'vicon_max_component_span_m':s['max_component_span_m']}
                  for g in own for a,s in g['anchors_vicon'].items()]
    pd.DataFrame(own_deltas).to_csv(out/'own_anchor_config_discrepancy.csv', index=False)
    dump(out/'validation.json', {'recordings_checked':len(checks), 'checks':checks,
                               'sources_rehashed':len(hashes), 'source_changes':0, 'exit_code':0,
                               'summary_script_sha256':sha(__file__),
                               'legacy_config_sha256':sha(ROOT/'config/vicon_test.yaml')})
    # Inspection plots are scatter-only (no lines bridging packet gaps) with unclipped error axes.
    figures = '\n'.join('- ['+Path(p).parent.parent.name+'/'+Path(p).parent.name+']('+p+')' for p in execution['figures'])
    fam_table = table(overview, {'family':'数据类','recordings':'recordings','raw_rows':'源观测行','valid_ranges':'有效正测距','evaluated':'可评残差'})
    comp_table = table(comparison, {'family':'数据类','recording':'recording','sample_count':'N raw', 'error_sample_count':'N error',
                       'raw_range_error_median_m':'median m','raw_range_error_mean_m':'mean m','raw_range_error_rmse_m':'RMSE m',
                       'raw_range_error_absolute_p95_m':'absolute P95 m','positive_gt_0.5_percent':'>0.5m %',
                       'positive_gt_0.5_longest_duration_s':'最长>0.5m s'})
    top_table = table(top, {'family':'数据类','recording':'recording','tag_id':'tag','anchor_id':'anchor',
                       'start_relative_s':'起点 s','end_relative_s':'终点 s','duration_s':'持续 s','count':'N',
                       'episode_mean_m':'段均值 m','outside_episode_median_m':'段外中位数 m'})
    persistent_names = '; '.join(fam+': '+', '.join(sorted(events[events.family==fam].recording.unique())) for fam in events.family.unique())
    miluv_drop = links[(links.family=='MILUV') & (links.recording=='cirObstacles_1_random3_0')]
    gmax=max(g['recomputed_minus_published_gt_max_abs_m'] for g in geometry if g['family']=='starloc')
    text = f'''# 五数据集 obstacle / persistent positive range-error audit

状态：`AUDIT_COMPLETE_WITH_REFERENCE_LIMITATIONS`。2026-09-12；本轮只运行独立 evaluator，
核心 estimator、recovery、定位 benchmark、参数调整、提交/push 均 **NOT_RUN**。
任务边界和全部描述性阈值见 [预登记协议]({ROOT}/experiments/OBSTACLE_RANGE_AUDIT_PROTOCOL.md)。

## 结论

1. **最值得优先用于真实持续偏置实验的是 STAR-loc，尤其 zigzag_s4 的 tag1→anchor7/6。**
   原始误差存在长平台，段外误差明显更低，几何重算与作者 Vicon reference 一致。
   这支持“真实 persistent positive range-error”，不等于已获得逐包 NLOS label 或证明 recovery 有益。
   MILUV nominal random/circular 也有持续正误差段；不能由 nominal 名称推断全部 LOS。
2. **cirObstacles_1_random3_0 没显示比默认 random 更强的总误差分布，并且采样太稀疏。**
   每 link median interarrival {miluv_drop.interarrival_median_s.min():.3f}–{miluv_drop.interarrival_median_s.max():.3f}s，
   每 link仅 {int(miluv_drop.sample_count.min())}–{int(miluv_drop.sample_count.max())} 包。
   所有相邻包都超过固定1s continuity gap，最长严格连续段为0s；这是**连续性不可确认**，不是没有 NLOS。
   用户两次提到的同名 recording 只统计一次。CIR 没有读取。
3. **两个 own_vicon obstacle 及 HUEC 动态 NLOS 更适合候选 no-harm/selectivity 场景，而非当前强持续 NLOS 主证据。**
   自有数据约0.3–0.4m的持续正基线确实存在，nominal也有；未有独立 beta，不能把全部基线归为动态 NLOS。
   15:46:27 有短促异常，15:48:55更稳定；两条均没有 >0.5m、持续≥2s且≥5包的连续段。
   HUEC四条 NLOS 的 raw median 较小，但少量大负误差会抬高 RMSE；不应称为“全体测距准确”。
   本轮没有定位运行，因此 no-harm 只是后续实验用途建议，**不是已证明无伤害**。
4. **存在明确的数据解释/几何风险。** STAR-loc v1 radio/marker ID 不同；自有旧配置与bag anchor明显不符；
   HUEC GNSS高度基准需+1m，个别参考高度异常；自有两bag缺index，静态CSV含footer/损坏行。
   这些问题和参考类型不符项均保留，不能解释成 positive NLOS。

## 范围和指标

{fam_table}

共 {len(records)} 条本地 recording/静态配置文件，{len(links)} 条 tag–anchor 汇总、{len(anchors)} 条 anchor 汇总。
“源观测行”已排除明确标记的统计footer，但保留无效/空白记录；valid_ranges另列。
HUEC静态缺 {len(missing)} 个预期height/distance组合文件，见 [缺项清单]({out}/missing_static_grid.json)。
缺文件不能直接计为无线丢包。SFUISE 是 Vive，静态HUEC是激光参考，均不符合本轮限定的 Vicon/GNSS；
它们已接入count/dropout审计，error/persistence输出NA。没有使用别的传感器补GT。

统一 `e_raw=z_raw-||p_tag_GT-a_GT||`，无 beta subtraction、无demean、无range/GT拟合外参或时钟；
它包含固定偏置、动态excess、测量噪声及参考误差，**不是 latent bias truth**，也不是post-fit residual。
主表P95为绝对误差P95；CSV另有signed P95及positive-tail条件均值/P95、e>0/0.2/0.5/1m计数和比例。
最长段逐tag–anchor计算；e≤门槛、无效range/GT或gap>1s中断，anchor/recording汇总取link最长，不跨tag串段。
“明显持续”仅为预登记描述：e>0.5m且持续≥2s、N≥5；不是检测器、置信保证或NLOS真值标签。
所有图为全原始有效误差散点，未用折线跨掉包连接、未剪掉大误差；时间从各recording首UWB起算。

## 哪些 recording / link 有明显持续正误差

以下按持续时间展示12条代表link；完整 qualifying 集合与起止位置见
[persistent_links.csv]({out}/persistent_links.csv)，同时保留left/right-censored标记。
“段外中位数”只作离线描述对照，不作为减偏置、校准或 estimator preprocessing。

{top_table}

满足描述条件的全部 recording：{persistent_names}。

zigzag_s4 anchor7最长段从记录开头出现，属于left-censored，不能声称观测到了真实onset；
anchor6有记录内部的平台，更适合包含前后正常参考的实验。s3的loop-3d/zigzag和MILUV默认random/circular
可作为第二组真实候选。建议保留完整 recording 与其他anchor，不只导出漂亮片段。
这些序列已经因本轮GT审计产生development曝光，不能随后当未见held-out test。

## nominal/default 与 obstacle 对照

{comp_table}

这是不同recording的原始分布比较，不是控制住姿态、距离、速度和采样率后的障碍因果效应。
MILUV obstacle与default random的median仅相差约厘米量级，default circular的尾部反而更重。
自有nominal的anchor1也存在约2秒正偏段及离散大异常，不能把no_obstacle名字当作零偏标定。
HUEC没有大持续正段，但negative mismatch需单独诊断；完整极值与无效计数见
[extreme_and_invalid.csv]({out}/extreme_and_invalid.csv)。

STAR-loc无统一“obstacle recording”标签。作者区分手持高度和Jackal安装：s1/s2/s3/s4/s5是setup，
不是每包传播类别；s2提高rig位置以增加LOS比例。相同轨迹跨setup可描述比较，不能混同为同运动配对。
详见 [作者setup说明](https://arxiv.org/html/2309.05518v1#S3.SS1)。

## 几何、时间与数据质量核验

- MILUV：锁定作者commit `{MILUV_COMMIT}`，三条都用constellation0。GT为ifo001 Vicon body；
  天线位置为p+R·lever，tag10/11分别使用作者非零杆臂，不把body当tag或混用两个tag。
  `range_raw`在校正前保存，`range`经antenna-delay/power校正；本轮只统计前者。
  timeshift.yaml是共同epoch origin，CSV已经减过一次，不能再次给GT单独平移。
  线性位置/SLERP、不外推、GT bracket≤0.05s；与发布GT range的差异有少数局部异常，不能把两种插值声称精确相同。
  [作者预处理](https://github.com/decargroup/miluv/blob/{MILUV_COMMIT}/preprocess/process_uwb.py)、
  [时间处理](https://github.com/decargroup/miluv/blob/{MILUV_COMMIT}/preprocess/cleanup_csv.py)、
  [anchor](https://github.com/decargroup/miluv/blob/{MILUV_COMMIT}/config/uwb/anchors.yaml)、
  [tag](https://github.com/decargroup/miluv/blob/{MILUV_COMMIT}/config/uwb/tags.yaml)。
- STAR-loc：从作者测量时刻Vicon tag_pos重算距离；v1的8项radio→marker映射经过逐包GT几何恒等验证，
  v2/v3同号。全部重算与作者gt_range最大绝对差 {gmax:.3g}m。v1映射来源是发布GT的身份解析，
  不是独立外参标定，严格只在evaluator使用，不能交给estimator作GT-derived adapter。
  由rig/tag点反算的body lever仅作一致性诊断；没有用于估计器。camera tf不用于本轮测距几何。
  [作者README](https://github.com/utiasASRL/starloc/blob/d3ad541/README.md)、
  [v1 marker文件](https://github.com/utiasASRL/starloc/blob/d3ad541/mocap/uwb_markers_v1.csv)。
- own_vicon：按用户声明近似共点，tas_uwb_0为移动参考、1..4为anchor；两端在各包时刻插值，
  不从旧配置搬anchor、不将测距拟合到GT。旧config与同bag Vicon anchor的距离差范围
  {min(x['legacy_config_minus_vicon_norm_m'] for x in own_deltas):.3f}–{max(x['legacy_config_minus_vicon_norm_m'] for x in own_deltas):.3f}m，
  见[逐anchor差异]({out}/own_anchor_config_discrepancy.csv)。未修改旧config。
  最大anchor坐标分量跨度 {max(x['vicon_max_component_span_m'] for x in own_deltas):.4f}m；这含mocap抖动/异常，
  不是测距拟合证据。header同一epoch且无倒序，bag−header延迟已有记录，但这不能证明独立时钟偏移为零。
  两条obstacle原bag无索引，只对临时副本reindex；可读UWB窗口仅约11秒，不假设未封存尾部也已恢复。
- HUEC动态：作者trajectory.csv提供处理后的GNSS位置，作者评估对z加1m；若把z直接当tag高度会引入几何错误。
  /odometry/local_gps、pose.csv、LS/ESKF结果不能不加区分当成同一GT；本轮采用声明的trajectory.csv，未读取LS/ESKF输出。
  时间从ns转s，约8Hz GT采用≤0.25s bracket，未外推/搜索延迟。
  GNSS→UWB坐标转换和NTP由作者声明，独立杆臂/heading/同步不确定度未完整给出。
  LOS Case2存在z相对高度达到1.63m；原作者评估会按abs(z)<0.5筛掉一些GT，本轮保留并报告，
  不靠删不利参考降低RMSE。各条都有少量大负range/reference误差，其原因未定位，不能作为positive NLOS证据。
  [作者论文](https://www.nature.com/articles/s41597-025-05887-9)、
  [作者评估源码](https://github.com/cucudasluv/UWB-dataset/blob/main/Technical_validation/localization_error_analysis.py)。
- HUEC静态：{sum(x['footer_rows_not_measurements'] for x in static_geometry)}行明确统计footer不算measurement；
  完全空白文件和截断行保留为invalid，原文件未修写。激光GT超出本轮限制，不能回答它们是否有持续正偏。
  Transmission/Reception counter仅报告首末采样之间的增量差，遇到reset/不一致则NA，见
  [static_counter_audit.csv]({out}/static_counter_audit.csv)；不把累计历史计数当本recording包数。
- SFUISE：仅absolute ToA测距count/dropout；Vive非Vicon/GNSS，且历史单位GT→anchor假设未闭合。
  因此不给新range误差或NLOS结论，保留三条UNAVAILABLE。不是数据读取失败。

## dropout 和统计文件

主交付：

- [逐recording总表]({out}/recording_summary.csv)
- [逐recording/anchor总表]({out}/anchor_summary.csv)
- [逐recording/tag/anchor完整表]({out}/link_summary.csv)
- [nominal/obstacle对照]({out}/nominal_obstacle_comparison.csv)
- [GT关联状态]({out}/association_status.csv)
- [全部几何、GT/IMU时序与header延迟]({run}/geometry_audit.json)

link表包含N、median/mean/RMSE/absolute和signed P95、positive tails、四阈值最长段、
interarrival median/P95/max、>1s gap数量/总时间、>3×median gap、cadence缺样本估计、首尾覆盖缺口。
无真实发包计划时cadence估计不是packet-loss rate；MILUV CIR记录的慢轮询也不能写成大量无线丢包。
未匹配GT和无效range分别计数；e不可用时最终summary的最长段为NA。

## 诊断图

{figures}

## 执行证据与限制

最终run：`{run.name}`；evaluator exit0，{len(records)}条处理完成，0个执行失败，源SHA前后0变化。
9/9工程测试通过，包含杆臂/符号、gap/缺失GT断段、坏geometry拒绝、footer/损坏行、全无效recording保留。
独立summary再核验所有recording行数/obs_id唯一性/RMSE及每个最长事件成员，exit0。

实际命令：

```bash
/usr/bin/python3 experiments/scripts/test_obstacle_range_audit.py
/usr/bin/python3 experiments/scripts/audit_obstacle_ranges.py
/usr/bin/python3 experiments/scripts/summarize_obstacle_audit.py {run}
```

[执行manifest]({run}/execution.json)、[source SHA]({run}/source_hashes.json)、
[固定官方来源下载ledger]({run}/sources/ledger.json)、[逐recording验证]({out}/validation.json)。
原始逐观测CSV和13幅图均在evaluator_private，未导出至estimator cache。
两次工程失败历史保留：首轮751个静态解析失败且v1同号几何结果作废；第二轮2个无时间行处理失败。
报告生成首轮因全NA列的读取类型检查失败（exit1），原日志保留；显式数值类型处理后复核通过，
未修改观测或统计值。执行日志位于仓库 experiments/evidence/obstacle_range_audit_20260912/。
最终版本没有更改阈值、选取新时窗或删掉失败数据来优化结论。

本报告是数据资格和原始误差审计，不是全大规模定位实验、恢复收益报告、独立beta标定或正式held-out验证。
T10=C2-C、T11=C、C1–C3不升级。
'''
    (out/'AUDIT_REPORT.md').write_text(text)
    dump(out/'artifact_hashes.json', {str(p):sha(p) for p in out.iterdir() if p.is_file()})
    print(out/'AUDIT_REPORT.md')
    print(overview.to_string(index=False)); print('FINAL_VALIDATION=PASS')


if __name__ == '__main__': main()
