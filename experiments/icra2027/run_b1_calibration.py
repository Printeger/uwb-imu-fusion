#!/usr/bin/env python3
"""R2.5: three fresh constant-beta runs, frozen common-GT evaluation, no tuning."""
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml
from fingerprint import HERE, ROOT, WS, sha, digest
from evaluate_clean import read, rows, save, associate, score, density, sources, ev

OFFSETS = [-0.0700, 0.1539, -0.0751, 0.1409, -0.0247]
ANCHORS = [7475, 9524, 10548, 15155, 20276]
BETA = {f'27956:{aid}': -offset for aid, offset in zip(ANCHORS, OFFSETS)}


def preflight():
    version = yaml.safe_load((HERE / 'VERSION.yaml').read_text())
    fingerprint = version.pop('fingerprint')
    assert digest(version) == fingerprint
    protected = {str(ROOT / p): h for p, h in version['source_files'].items()}
    protected.update(version['linked_libraries'])
    protected[version['runner']] = version['runner_sha256']
    for item in version['inputs']:
        protected.update(item['files'])
    for directory in ('metrics', 'audits', 'configs/backbone'):
        for p in (HERE / directory).glob('*'):
            if p.is_file() and not p.name.startswith('B1_'):
                protected[str(p)] = sha(p)
    base = yaml.safe_load((HERE / 'configs/detector/starting_pipeline.yaml').read_text())
    for n in (1, 2, 3):
        official = HERE / f'configs/sfuse/config_test_isas-walk{n}.yaml'
        protected[str(official)] = sha(official)
        assert yaml.safe_load(official.read_text())['toa_offset'] == OFFSETS
        cfg = yaml.safe_load((HERE / f'configs/backbone/Walk{n}_B0_CURRENT.yaml').read_text())
        assert sorted(a['id'] for a in cfg['anchors']) == ANCHORS
        assert not cfg['calibration']['calib_range_bias']
        assert cfg['calibration']['fixed_beta_by_link'] == {}
        expected = json.loads(json.dumps(base))
        expected['dataset']['cache_manifest'] = version['inputs'][n-1]['manifest']
        expected['nlos'].update(mode='disabled', score_recoverability=False, final_inference_enabled=False)
        assert expected == cfg, 'B1 builder differs from frozen B0 before beta'
    for item in rows(HERE / 'metrics/run_status.csv'):
        if item['variant'] in ('B0_CURRENT', 'SF_NATIVE'):
            protected[item['trajectory_path']] = item['trajectory_sha256']
            n = int(item['sequence'][-1])
            protected[str(WS / f'res/ie0911_step2_truth_20260911_01/sfuise_walk{n}/ground_truth.tum')] = item['gt_sha256']
    protected[str(HERE / 'evaluate_clean.py')] = sha(HERE / 'evaluate_clean.py')
    protected[str(HERE / 'run_backbones.py')] = sha(HERE / 'run_backbones.py')
    for p, h in protected.items():
        assert sha(p) == h, p
    return {'schema': 'R2_5_B1_INPUT_PARITY_V1', 'fingerprint': fingerprint,
            'offsets': OFFSETS, 'fixed_beta_by_link': BETA, 'protected': protected,
            'common_gt_sets': read(HERE / 'audits/frame_fits_evaluator_only.json'),
            'script_sha256': sha(__file__), 'status': 'PREFLIGHT_PASS',
            'calibration_provenance': 'AUTHOR_DATASET_SPECIFIC_VALUES_INDEPENDENCE_UNVERIFIED'}


def evaluate(batch, lock):
    native = sources(batch)
    previous = rows(HERE / 'metrics/backbone_parity.csv')
    provenance = rows(HERE / 'metrics/run_status.csv')
    output, checks = [], []
    for n in (1, 2, 3):
        sequence = f'Walk{n}'
        baseline = next(r for r in previous if r['sequence'] == sequence and r['variant'] == 'B0_CURRENT')
        common = lock['common_gt_sets'][sequence]['common_gt_times']
        assert digest(common) == baseline['common_gt_sha256']
        for variant in ('B0_CURRENT', 'SF_NATIVE'):
            r = next(r for r in previous if r['sequence'] == sequence and r['variant'] == variant)
            output.append({'sequence': sequence, 'variant': variant,
                           **{k: r[k] for k in ('used_range_count', 'state_count', 'ATE_RMSE_aligned',
                                               'ATE_P95', 'RPE_1s', 'status', 'n_gt_samples', 'common_gt_sha256')},
                           'runtime': r['runtime_s'], 'runtime_scope': r['runtime_scope'], 'source_reused': True,
                           'delta_B1_minus_B0_ATE': None, 'relative_change_percent': None})
        run = native[n]
        status = read(run / 'run_status.json')
        record = {'sequence': sequence, 'variant': 'B1_CAL', 'used_range_count': None, 'state_count': None,
                  'ATE_RMSE_aligned': None, 'ATE_P95': None, 'RPE_1s': None,
                  'runtime': status.get('elapsed_seconds'), 'runtime_scope': 'NATIVE_ESTIMATOR',
                  'status': 'failed', 'source_reused': False, 'n_gt_samples': 0,
                  'common_gt_sha256': digest(common), 'delta_B1_minus_B0_ATE': None,
                  'relative_change_percent': None, 'failure_reason': status.get('reason', ''),
                  'run_directory': str(run), 'trajectory_sha256': None}
        b0_path = Path(next(r for r in provenance if r['sequence'] == sequence and r['variant'] == 'B0_CURRENT')['trajectory_path']).parent
        cfg0 = yaml.safe_load((HERE / f'configs/backbone/Walk{n}_B0_CURRENT.yaml').read_text())
        cfg1 = yaml.safe_load((HERE / f'configs/backbone/Walk{n}_B1_CAL.yaml').read_text())
        assert cfg1['calibration']['fixed_beta_by_link'] == BETA
        cfg1['calibration']['fixed_beta_by_link'] = {}
        assert cfg0 == cfg1, 'configuration changed beyond fixed beta'
        check = {'sequence': sequence, 'only_fixed_beta_config_changed': True, 'backend_status': status}
        assert sha(run / 'observations.csv') == sha(b0_path / 'observations.csv'), 'observation plan/noise changed'
        assert sha(run / 'imu_covariance_model.txt') == sha(b0_path / 'imu_covariance_model.txt'), 'IMU model changed'
        graph_line = lambda p: next(line for line in (p / 'stdout.log').read_text().splitlines() if line.startswith('GraphBuilder:'))
        assert graph_line(run) == graph_line(b0_path), 'graph cardinalities changed'
        obs = rows(run / 'observations.csv')
        planned = [r for r in obs if r['planned'] == '1']
        record.update(state_count=len({r['keyframe_id'] for r in planned}), planned_range_count=len(planned),
                      used_range_count_scope='VALID_FINAL_GRAPH_ONLY',
                      planned_common_gt_samples=len(common))
        check.update(observations_byte_identical=True, imu_model_byte_identical=True,
                     graph_cardinalities_identical=True, graph_cardinalities=graph_line(run),
                     initial_values_same=read(run / 'common_preparation.json')['values_sha256'] ==
                                         read(b0_path / 'common_preparation.json')['values_sha256'])
        check['raw_reference'] = read(run / 'raw_reference.json')
        if status.get('valid_estimate_exported') and status['exit_code'] == 0:
            assert sha(run / 'observations.csv') == sha(b0_path / 'observations.csv'), 'observation plan/noise changed'
            assert sha(run / 'baseline_factor_audit.csv') == sha(b0_path / 'baseline_factor_audit.csv'), 'final mask changed'
            estimates = ev.load_tum(run / 'trajectory.tum')
            assert [r[0] for r in estimates] == [r[0] for r in ev.load_tum(b0_path / 'trajectory.tum')], 'state times changed'
            counts, _ = density(run, sequence, 'B1_CAL')
            record.update(used_range_count=counts[0]['used_range_count'], state_count=counts[0]['state_count'])
            gt = ev.load_tum(WS / f'res/ie0911_step2_truth_20260911_01/sfuise_walk{n}/ground_truth.tum')
            mapping = associate(estimates, gt, float(baseline['input_evaluation_start']), float(baseline['input_evaluation_end']))
            assert all(t in mapping for t in common), 'frozen common GT set must not shrink'
            metrics, points, fit = score(mapping, common)
            assert metrics['common_gt_sha256'] == baseline['common_gt_sha256']
            for k in ('ATE_RMSE_aligned', 'ATE_P95', 'RPE_1s', 'n_gt_samples'):
                record[k] = metrics[k]
            delta = record['ATE_RMSE_aligned'] - float(baseline['ATE_RMSE_aligned'])
            record.update(status='completed', failure_reason='', delta_B1_minus_B0_ATE=delta,
                          relative_change_percent=100*delta/float(baseline['ATE_RMSE_aligned']),
                          trajectory_sha256=sha(run / 'trajectory.tum'))
            check.update(observations_byte_identical=True, final_factor_mask_byte_identical=True,
                         state_times_identical=True, common_gt_identical=True,
                         initial_values_same=read(run / 'common_preparation.json')['values_sha256'] ==
                                             read(b0_path / 'common_preparation.json')['values_sha256'])
            save(HERE / f'figures/data/B1_calibration_walk{n}.csv', points)
            check['evaluator_only_fit'] = fit
        output.append(record)
        checks.append(check)
    for p, h in lock['protected'].items():
        assert sha(p) == h, 'protected file changed: ' + p
    save(HERE / 'metrics/B1_calibration.csv', output)
    result = {'schema': lock['schema'], 'checks': checks, 'protected_hashes_pass': True,
              'protected_file_count': len(lock['protected']), 'batch': str(batch),
              'evaluation_script_sha256': sha(__file__),
              'fresh_run_count': 3, 'B2': 'NOT_RUN', 'B3': 'NOT_RUN', 'injected': 'NOT_RUN'}
    (HERE / 'audits/B1_calibration_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    report = ['# R2.5 B1 static ToA input-parity result', '',
              '结果：Walk1/Walk2在原100次preliminary LM上限失败，未导出有效轨迹；Walk3成功。'
              'Walk3 aligned ATE差值与百分比见下表。本轮未观察到static offset追回clean ATE的证据；'
              '两条失败不能作为零收益、也不能唯一归因于offset数值或某个solver问题。', '',
              '这些 offsets 是 SFUISE 作者配置提供的 dataset-specific values；repository/paper 没有证明它们来自独立 '
              'calibration recording，因此本实验用于 input-parity analysis，而不宣称独立 calibration provenance。', '',
              '固定 anchor 顺序为7475、9524、10548、15155、20276。offset(m)为 '+str(OFFSETS)+'；native beta=-offset，'
              '残差 h+beta-z 与 SFUISE h-z-offset 等价。既有 fixed_beta_by_link 常量路径恰好应用一次，raw不改。', '',
              '| Sequence | Variant | Status | Used ranges | States | Aligned ATE (m) | P95 (m) | RPE 1s (m) | Runtime (s) | Δ B1−B0 (m) | Change % |',
              '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in output:
        def fmt(k):
            v = r.get(k)
            return 'NA' if v is None or v == '' else f'{float(v):.6f}'
        report.append('| '+' | '.join([r['sequence'],r['variant'],r['status'], str(r['used_range_count'] or 'NA'),
                                      str(r['state_count'] or 'NA')]+[fmt(k) for k in ('ATE_RMSE_aligned','ATE_P95','RPE_1s',
                                      'runtime','delta_B1_minus_B0_ATE','relative_change_percent')])+' |')
    report += ['', 'Δ=B1−B0；relative change=100×Δ/B0，负值表示改善。失败保持NA，不改变共同集合。', '',
               'B0与SF_NATIVE复用R1/R2冻结结果；B1三条各fresh一次。共同GT样本精确固定为227/292/313；'
               '直接调用未修改的 R1/R2 associate/score，scale=1 SE3；raw-frame和tracker/body proxy限制保持。', '',
               '配置仅fixed_beta_by_link变化；三条完整observations（含ID、plan、nominal sigma）、IMU模型转储及graph '
               'cardinalities与B0相同；成功Walk3的final factor mask及state timestamps也相同。失败两条没有有效final，'
               'used_range_count与accuracy保持NA，state_count/计划range仍按实际构图记录。没有live beta变量、没有新增factor或state。输入、源码、二进制、'
               '依赖、旧指标和evaluator hash核验通过。SF原产物未提供实际factor/state计数，保留NA；SF runtime含1x ROS playback，'
               'native runtime为estimator elapsed，不能据此排名速度。', '',
               '既有Initializer::Run会在局部geometry副本上使用z-beta作trilateration（src/initializer.cpp:462）；'
               '因此B1的初值随static calibration自然变化，三条initial Values hash均不同。初始化代码未修改，'
               'raw ledger未修改；本对照衡量现有constant机制的整体影响，不声称隔离了固定初值下的factor-only效应。', '',
               '未运行B2、B3或injected-NLOS；未修改detector/recovery/IMU/求解参数，不按结果调参，未提交或push。', '',
               '复现命令：`python3 experiments/icra2027/run_b1_calibration.py`（新复现需单独锁目录，当前脚本拒绝覆盖已有交付）。',
               '实际batch：`'+str(batch.relative_to(ROOT))+'`。命令、退出码、日志、fingerprint见batch的run_metadata.json；'
               '执行前锁见B1_calibration_lock.json，逐序列核验见B1_calibration_verification.json。']
    (HERE / 'audits/B1_calibration_result.md').write_text('\n'.join(report)+'\n')
    print(json.dumps(output, indent=2))


def main():
    lock_path = HERE / 'audits/B1_calibration_lock.json'
    if lock_path.exists():
        raise RuntimeError('R2.5 already locked; refuse scientific rerun or overwrite')
    lock = preflight()
    lock_path.write_text(json.dumps(lock, indent=2)+'\n')
    before = set((HERE / 'runs').glob('B1_CAL-*'))
    cmd = [sys.executable, str(HERE / 'run_backbones.py'), '--author-offsets-dev']
    start = time.monotonic()
    with (HERE / 'logs/B1_calibration_run.log').open('w') as log:
        code = subprocess.call(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    fresh = set((HERE / 'runs').glob('B1_CAL-*')) - before
    lock.update(argv=cmd, exit_code=code, wall_time_s=time.monotonic()-start,
                batch_directories=[str(p) for p in fresh])
    lock_path.write_text(json.dumps(lock, indent=2)+'\n')
    assert len(fresh) == 1
    evaluate(fresh.pop(), lock)


if __name__ == '__main__':
    main()
