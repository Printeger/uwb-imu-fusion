#!/usr/bin/env python3
"""Render only locked metrics and retained task failures; no estimator or tuning."""
import argparse,csv,json,shutil
from pathlib import Path
import run_recover_vs_reject as r


def table(rows,columns):
    def fmt(v):
        if v is None or v=='':return 'NA'
        if isinstance(v,float):return format(v,'.6f')
        return ' '.join(str(v).split()).replace('|','/')
    return ['| '+' | '.join(c[0] for c in columns)+' |','|'+'|'.join(['---']*len(columns))+'|']+[
        '| '+' | '.join(fmt(row.get(key)) for _,key in columns)+' |' for row in rows]


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',required=True,type=Path);a=p.parse_args();out=a.run.resolve()
    data=r.read(out/'evaluation.json');lock=r.read(out/'lock.json');ledger=r.read(out/'execution.json');pre=r.read(out/'preflight.json')
    evidence=r.ROOT/'experiments/evidence'/out.name;evidence.mkdir(exist_ok=False)
    for fn in ['lock.json','preflight.json','execution.json','metrics.csv','paired_differences.csv','four_method_common.csv','evaluator_audit.csv','pairing_checks.json','evaluation.json','engineering_validation.json','engineering_base.log','engineering_admitted.log','clock_reproduction.log','input_plan_diagnostics.json','failure_diagnosis.json','independent_metric_verification.json','verify.log']:
        shutil.copy2(out/fn,evidence/fn)
    diagnostic=Path(lock['geometry']['source_diagnosis'])
    shutil.copy2(diagnostic,evidence/'imu_diagnosis.json')
    shutil.copy2(diagnostic.parents[1]/'clock_diagnosis.json',evidence/'clock_diagnosis.json')
    for name in r.RECORDINGS:
        d=evidence/name;d.mkdir()
        for fn in ['audit_diagnostic.png','trajectory_errors.png','all_positive_error_events.csv','window_union.json']:
            shutil.copy2(Path(data['private_directory'])/name/fn,d/fn)
        for fn in ['ros_roundtrip.json','preparation.json']:
            shutil.copy2(out/name/fn,d/fn)
        for task in [x for x in ledger['tasks'] if x['recording']==name]:
            src=Path(task.get('run_directory',out/'ABSENT'));dest=d/task['task'];dest.mkdir()
            for fn in ['run_status.json','production_detector_status.json','final_inference_summary.json','cache_replay_status.json','common_preparation.json','adapter_runtime.txt','stage2_refit_status.json','refit_iterations.csv']:
                if (src/fn).exists():shutil.copy2(src/fn,dest/fn)
    rel='evidence/'+out.name
    counts={k:sum(x['status']==k for x in data['metrics']) for k in ['SUCCESS','FAILURE','NOT_RUN']}
    lines=['# Recover vs Reject：GT辅助准入 development 预实验','',
        f"12个方法单元：{counts['SUCCESS']}正常导出、{counts['FAILURE']}失败、{counts['NOT_RUN']}未运行；科学进程{ledger['scientific_processes_started']}/15。RR差值可用{sum(x['delta_RR_m'] is not None for x in data['pairs'])}/3，不能从NA判断Recover胜负。", '',
        '本轮按冻结三条录制执行并保留全部失败。输入符号/轴变换与固定时钟偏移由现有数据诊断后冻结；估计器只读测量白名单输入。',
        '这是使用本批GT辅助输入解释的development实验；不作为独立标定、held-out或论文主claim。','',
        '## 为什么上一轮没能运行，以及本轮解决了什么','',
        '上一轮在独立资料不能闭合CSV的IMU语义时按合同阻塞，科学任务为0，不能据此判断Recover失败。',
        '本轮用户授权用现有GT诊断。IMU CSV与UWB CSV携带的同一rig GT位姿证明两套时间轴不一致；只用GT-to-GT位姿拟合一个常量偏移，未用估计轨迹或测距残差优化。',
        '加速度和角速度均采用固定坐标变换 `diag(1,-1,-1)`：翻转y/z，是det=+1的旋转；加速度不整体取负，符合 `f=Rᵀ(a−g)`。IMU时间加下表偏移，UWB源行和时间完全不动。','']
    dg=[]
    for name,g in lock['geometry']['recordings'].items():
        dg.append(dict(recording=name,clock=g['imu']['time_offset_s'],acc=g['diagnostics']['acc']['rmse'],gyro=g['diagnostics']['gyro']['rmse'],
            rig=g['diagnostics']['origin_rig']['rmse'],imu=g['diagnostics']['origin_imu']['rmse'],signal=g['diagnostics']['origin_signal']['rmse']))
    lines+=table(dg,[('recording','recording'),('IMU→UWB偏移s','clock'),('加速度RMS m/s²','acc'),('角速度RMS rad/s','gyro'),('rig原点假设RMS','rig'),('IMU原点假设RMS','imu'),('两原点预测差RMS','signal')])
    lines+=['','rig原点假设的加速度RMS略低；仅凭平滑GT二阶导数、传感器噪声和这些运动，不能唯一确定作者是否作过原点转换。',
        '本轮明确采用物理IMU原点近似、rig方向作为body坐标；未从较小残差宣称辨认出实际处理流程。厂商camera–IMU与三条发布camera–rig外参的共同均值只形成近似杆臂，不逐录制优化测距几何。',
        '`lever_body_m='+str(lock['geometry']['lever_body_m'])+'`；GT参考为作者Vicon tag1天线位置。不能从最终CSV唯一逆推出原始IMU到CSV的完整处理代码。',
        '该原点近似尤其限制快速转动段的解释；原点信号与诊断误差量级均保留。没有拟合传感器bias/噪声/scale，原始range的固定beta未校正。','',
        '## 12个方法单元','',
        'SUCCESS只表示进程/后端完成有效格式导出，不表示定位精度达标。5条导出轨迹中4条严重发散：Cauchy在两条zigzag上ATE为376.269/20640.156m，SFUISE在loop-3d_s3/zigzag_s3上为3512.804/14494.768m。',
        '已用独立scipy Rotation.align_vectors复核全部5个SE3 RMSE，数值一致；原始导出位置本身已跨越千米至数万米，GT运动范围约6m，因此这些大误差不是对齐公式造成的。它们是保留的负结果，不升级为成功定位。',
        '所有距离指标单位m。RR主指标使用两方法共同有效时刻各自进行一次全区间SE3、scale=1对齐；窗口内不重对齐。',
        '固定10Hz网格覆盖完整UWB首末时刻，估计最近邻≤0.02s、GT bracket≤0.05s，不外推。SFUISE缺失不缩短主RR对照。','']
    lines+=table(data['metrics'],[('recording','recording'),('method','method'),('ATE RMSE','ATE_RMSE_m'),('正误差窗口RMSE','Window_RMSE_m'),('ATE P95','ATE_P95_m'),('覆盖率','coverage'),('status','status')])
    audits=list(csv.DictReader((out/'evaluator_audit.csv').open()))
    lines+=['']+table(audits,[('recording','recording'),('全部正误差事件','event_count'),('去重窗口','window_union_count'),('并集时长s','window_union_duration_s')])
    lines+=['','窗口是审计 `e>0.5m、持续≥2s、至少5包、gap≤1s` 的逐link全部事件并集，不能等同真实动态NLOS标签。','',
        '## 同planned obs_id的range诊断与恢复记账','',
        '失败producer的候选计数仅为失败状态快照；特别是valid=false时的0不代表有效空support。未运行的decision/final接受数保持NA，不能据此声称LCB拒绝了全部候选。',
        '`before=z_raw−h_GT`；`after=z_raw−delta_final−h_GT`。只对最终实际恢复观测施加offset；suppressed/fallback观测保持原始range，未伪装成校正。SFUISE/Cauchy没有本方法offset，before/after相同仅表示这一测量诊断。','']
    lines+=table(data['metrics'],[('recording','recording'),('method','method'),('range n','range_count'),('before','range_before_RMSE_m'),('after','range_after_RMSE_m'),('候选观测','candidate_observations'),('候选段','candidate_segments'),('decision接受段','decision_accepted_segments'),('final接受段','final_accepted_segments'),('fallback','fallback')])
    lines+=['']+table(data['metrics'],[('recording','recording'),('method','method'),('恢复子集n','recovered_range_count'),('子集before','recovered_before_RMSE_m'),('子集after','recovered_after_RMSE_m')])
    lines+=['','## 三条独立配对差值','',
        '如果producer未成功发布Stage2 cache，两个依赖方法均不运行，实际共同support/Stage2 Values配对核验不可用；工程拒配测试不能代替实际成功配对。', '差值严格为Recover−Reject；正值表示Recover更差。缺失为NA，fallback差值不计作成功恢复收益。','']
    lines+=table(data['pairs'],[('recording','recording'),('delta_RR','delta_RR_m'),('delta_RR_window','delta_RR_NLOS_m'),('RR共同样本','common_samples'),('四方法共同样本','four_method_common_samples'),('解释','interpretation')])
    lines+=['',f'[四方法共同覆盖比较]({rel}/four_method_common.csv) 单列，不替代RR主表。',
        f'[共同input/初值/support/Stage2 Values身份核验]({rel}/pairing_checks.json)。不可配对者不产生数值差。','',
        '## 运行预算、失败与工程验证','',
        f"科学进程实际启动 {ledger['scientific_processes_started']}/15；每个进程树限时1800秒，名单顺序串行，不因失败更换录制或调参重试。",'']
    lines+=['zigzag_s4的PL bootstrap使用前5个关键帧，截止0.06850529s；校正后的首个IMU为0.09731442s，因此明确报BOOTSTRAP_IMU_EMPTY。未裁掉UWB开头或伪造IMU样本以重试。', '', '固定步长4下，zigzag_s4/zigzag_s3启动5帧分别只含anchor(10,5)/(9,4)，后续完整planned集合仍覆盖全部8anchor；该初始化几何条件保留为诊断，不调整步长或初始化。', '']
    lines+=table(ledger['tasks'],[('recording','recording'),('task','task'),('status','status'),('exit','exit_code'),('原因','reason')])
    failed_stage2=[x for x in ledger['tasks'] if 'RESTART_LIMIT_EXHAUSTED' in x.get('reason','')]
    if failed_stage2:lines+=['', 'Stage2的RESTART_LIMIT_EXHAUSTED是冻结求解判据拒绝数值停滞，并非进程超时：应同时检查步长、KKT和导航梯度，不能仅因目标函数不变/步长很小就宣称收敛。完整refit_iterations.csv与stage2_refit_status.json随证据保留。']
    lines+=['', 'zigzag_s3的x3062对应52.378824s，其IMU bracket约0.017290s，并非0.262s全局最大间断；不能用最大gap直接断言该处线性系统不定的根因。见failure_diagnosis.json。']
    lines+=['','23/23工程fixture通过：GT/窗口不可见与冻结准入参数后GT变化不改变白名单cache、源行ID/单位/杆臂、配对身份拒绝、空候选/零接受/恢复/fallback/producer失败/超时/负收益、全部事件并集与缺失GT断段。',
        '三条完整C++ prepare-only及Cauchy prepare-only均exit0、optimizer_calls=0，共同初值完全相同；全部CSV→ROS往返通过；三个SFUISE无测量启动探针通过。',
        '工程A保留LAPACK系统链接目标未挂载导致的SFUISE启动失败，科学任务0。修复隔离运行时后B通过，C为最终冻结运行；未修改estimator核心或参数。',
        'SFUISE为锁定75bf5a32原生ToA系统参考，保留其初始化、优化和rejection；仅输入topic、单位、频率、共同杆臂及显式零ToA offset改变。ROS float32 range舍入误差与纳秒时间舍入见逐录制ros_roundtrip.json。','',
        '## 诊断图','']
    for name in r.RECORDINGS:lines += [f'![{name}天线轨迹误差]({rel}/{name}/trajectory_errors.png)','']
    lines+=['## 产物与复现','',f'隔离完整输出：`{out}`；独立评价私有输出：`{data["private_directory"]}`。',
        f'[锁定manifest及hash]({rel}/lock.json)、[全部命令/退出码/产物hash]({rel}/execution.json)、[工程检查命令]({rel}/preflight.json)、[指标CSV]({rel}/metrics.csv)、[配对CSV]({rel}/paired_differences.csv)。',
        '[GT辅助准入amendment](RECOVER_VS_REJECT_GT_ADMISSION.md)；[原冻结协议](RECOVER_VS_REJECT_PROTOCOL.md)；[几何/时钟锁定值](recover_vs_reject_gt_geometry.json)。','',
        '```bash',f'/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py prepare --gt-assisted --run {out}',
        f'/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py preflight --gt-assisted --run {out}',
        f'/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py execute --gt-assisted --run {out}',
        f'/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py evaluate --gt-assisted --run {out}',
        f'/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py verify --gt-assisted --run {out}','```','',
        'prepare要求新目录；已有science ledger拒绝重跑，复现须用新目录且受原科学预算授权限制。历史阻塞结果与既有审计保留；未提交或push。']
    report=r.ROOT/'experiments/RECOVER_VS_REJECT.md';archive=evidence/'previous_blocked_report.md'
    shutil.copy2(report,archive);report.write_text('\n'.join(lines)+'\n')
    r.write(evidence/'evidence_sha256.json',{str(p.relative_to(evidence)):r.sha(p) for p in evidence.rglob('*') if p.is_file()})
    print(evidence)

if __name__=='__main__':main()
