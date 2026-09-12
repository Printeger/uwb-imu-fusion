#!/usr/bin/env python3
"""Render MILUV frozen metrics; preserve prior experiments."""
import argparse,shutil
from pathlib import Path
import run_recover_vs_reject as r
from report_rr_admitted import table

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);out=p.parse_args().run.resolve()
    data=r.read(out/'evaluation.json');ledger=r.read(out/'execution.json');lock=r.read(out/'lock.json')
    dest=r.ROOT/'experiments/evidence'/out.name;dest.mkdir(exist_ok=False)
    for path in out.iterdir():
        if path.is_file():shutil.copy2(path,dest/path.name)
    for rec in lock['rows']:
        name=rec['recording'];d=dest/name;d.mkdir()
        for fn in ['preparation.json','ros_roundtrip.json','config.yaml','cauchy.yaml','sfuise.yaml']:
            shutil.copy2(out/name/fn,d/fn)
        for fn in ['trajectory_errors.png','all_positive_error_events.csv','window_union.json']:
            shutil.copy2(Path(data['private_directory'])/name/fn,d/fn)
        for task in [t for t in ledger['tasks'] if t['recording']==name]:
            dd=d/task['task'];dd.mkdir();src=Path(task.get('run_directory',out/'ABSENT'))
            for fn in ['run_status.json','production_detector_status.json','stage2_refit_status.json','refit_iterations.csv','common_preparation.json','final_inference_summary.json','cache_replay_status.json','fixed_compensations.csv','adapter_runtime.txt']:
                if (src/fn).exists():shutil.copy2(src/fn,dd/fn)
            proc=task.get('process',{})
            for key,value in proc.items():
                if isinstance(value,str) and value.endswith('.log') and Path(value).is_file():shutil.copy2(value,dd/Path(value).name)
    rel='evidence/'+out.name
    lines=['# MILUV Recover vs Reject development 预实验','',
        '仅已审计合格的 default_1_random3_0、default_1_circular3D_0；ifo001/tag10、全部6 anchors、完整录制。',
        '原始 range_raw 与 PX4 IMU，作者公开杆臂 [0.13189,-0.17245,-0.05249] m，沿用作者 body/IMU 共点近似。未进行GT初始化、IMU去偏、时间拟合或静态测距校正；总正误差不是已分离的动态NLOS。',
        '冻结PL双向CUSUM与LCB参数，不按结果调整；每条一个producer，RR读取同一Stage2 cache。Cauchy scale=2.3849；SFUISE为原生独立系统参考，ToA offset明确置零。',
        '10 Hz完整UWB区间；估计最近邻≤0.02s、GT bracket≤0.05s、不外推。天线参考点、scale=1 SE3。RR在共同有效时刻各做一次全区间对齐，窗口不重新对齐；四方法共同覆盖另列。',
        '窗口使用每link全部 e>0.5m、持续≥2s、至少5包、gap≤1s 事件的去重并集，仅evaluator可见。','',
        'SUCCESS表示有效格式导出，不等于精度通过。全部距离指标单位m。','']
    lines+=['本轮两条producer均未发布Stage2 cache，RR四格NOT_RUN；两条Cauchy均达到LM迭代上限。SFUISE两条虽导出，但轨迹明显发散；独立scipy SE3复核与主指标一致，原始轨迹本身已远超GT运动范围。不能据此判断Recover胜负，也不能认定仅是外参问题。','']
    lines+=table(data['metrics'],[('recording','recording'),('method','method'),('ATE RMSE','ATE_RMSE_m'),('正误差窗口RMSE','Window_RMSE_m'),('ATE P95','ATE_P95_m'),('覆盖率','coverage'),('status','status')])
    lines+=['','range before=z_raw−h_GT；after=z_raw−delta_final−h_GT，仅应用最终实际恢复offset。suppressed/fallback保留原始range；Cauchy/SF列是相同planned观测的测量诊断，不是其post-fit残差。','']
    lines+=table(data['metrics'],[('recording','recording'),('method','method'),('range n','range_count'),('before','range_before_RMSE_m'),('after','range_after_RMSE_m'),('候选观测','candidate_observations'),('候选段','candidate_segments'),('decision接受段','decision_accepted_segments'),('final接受段','final_accepted_segments'),('fallback','fallback')])
    lines+=['']+table(data['metrics'],[('recording','recording'),('method','method'),('恢复子集n','recovered_range_count'),('子集before','recovered_before_RMSE_m'),('子集after','recovered_after_RMSE_m')])
    support=[]
    for rec in lock['rows']:
        name=rec['recording'];path=out/name/'backend/producer/production_detector_status.json';d=r.read(path) if path.exists() else {}
        support.append(dict(recording=name,valid=d.get('valid',False),candidate_observations=d.get('candidate_observation_count') if d.get('valid') else None,candidate_segments=d.get('final_segment_count') if d.get('valid') else None,status=d.get('status'),support_hash=d.get('support_hash')))
    from evaluate_recover_vs_reject import save_csv
    save_csv(dest/'detector_support.csv',support)
    lines+=['','上表依赖方法未运行时，Stage2/recovery计数为NA。下面单列Stage1已有证据：有效空support可为0，但不代表已成功发布Stage2或运行LCB。','']
    lines+=table(support,[('recording','recording'),('detector有效','valid'),('候选观测','candidate_observations'),('候选段','candidate_segments')])
    lines+=['','差值为 Recover−Reject，正值表示Recover更差；缺失NA，fallback不计为成功恢复收益。','']
    lines+=table(data['pairs'],[('recording','recording'),('delta_RR','delta_RR_m'),('delta_RR_NLOS','delta_RR_NLOS_m'),('共同样本','common_samples'),('解释','interpretation')])
    lines+=['',f"科学进程启动 {ledger['scientific_processes_started']}/10，逐条串行，每树1800秒；无算法重试。producer失败时两个依赖方法NOT_RUN，独立方法继续。",'']
    lines+=table(ledger['tasks'],[('recording','recording'),('task','task'),('status','status'),('exit','exit_code'),('原因','reason')])
    lines+=['','26项工程测试通过（20项共同输入/评价/记账、3项MILUV、3项准入及final-mask/fallback）；两条C++ prepare-only与Cauchy共同初值、完整CSV→ROS往返、SFUISE无测量启动检查见preflight。科学成功与否以上表为准。',
        'random3 producer在x244（约27.80s）报告线性系统不定，局部IMU bracket约4ms；circular3D detector完成有效空support，但PL_RAW_REFERENCE达到LM上限。没有因此放宽求解判据或重跑参数；根因尚未唯一识别。',
        'GT变化不改变测量白名单cache；原始GT、窗口及其他方法产物未挂载给估计器。冻结实现、核心、源输入、准备产物均有hash；实际RR配对额外核验input plan、nominal sigma、原始Values、共同准备身份、support和Stage2 Values。','',
        f'[逐方法CSV]({rel}/metrics.csv)、[配对CSV]({rel}/paired_differences.csv)、[四方法共同覆盖]({rel}/four_method_common.csv)、[配对身份]({rel}/pairing_checks.json)、[全事件统计]({rel}/evaluator_audit.csv)。',
        f'[锁定manifest]({rel}/lock.json)、[实际命令、退出码与产物hash]({rel}/execution.json)、[工程准入]({rel}/preflight.json)。',
        f'完整独立输出：`{out}`；GT私有评价：`{data["private_directory"]}`。','']
    for rec in lock['rows']:lines += [f'![{rec["recording"]}]({rel}/{rec["recording"]}/trajectory_errors.png)','']
    lines+=['SFUISE频率配置40/125仅为冻结元数据；原生sample_coeff=1分支保留每条消息，不按该频率重采样。','', '复现入口（prepare要求新目录；execute拒绝已有ledger，不能自动重跑科学矩阵）：','', '```bash']
    for stage in ['prepare','preflight','execute','evaluate','verify']:lines.append(f'python3 experiments/scripts/run_miluv_recover_vs_reject.py {stage} --run {out}')
    lines+=['```','','[冻结协议](MILUV_RECOVER_VS_REJECT_PROTOCOL.md)。保留STAR-loc全部历史结果；不升级held-out、C1–C3、T10/T11或Recover优越性claim。未提交或push。']
    (r.ROOT/'experiments/MILUV_RECOVER_VS_REJECT.md').write_text('\n'.join(lines)+'\n')
    r.write(dest/'evidence_sha256.json',{str(p.relative_to(dest)):r.sha(p) for p in dest.rglob('*') if p.is_file()})
    print(dest)
if __name__=='__main__':main()
