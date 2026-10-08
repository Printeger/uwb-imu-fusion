#!/usr/bin/env python3
"""Sensor-only startup noise inventory; no GT or trajectory files are read."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml


def audit(cache, configs):
    records = []
    for manifest_path in sorted(cache.glob('*/*/manifest.json')):
        manifest = json.loads(manifest_path.read_text())
        path = manifest_path.parent/'imu.csv'
        samples = np.genfromtxt(path, delimiter=',', names=True)
        window = samples[samples['t'] <= samples['t'][0]+2.0]
        dt = float(np.median(np.diff(window['t'])))
        acc = np.stack([window[k] for k in ('ax','ay','az')], axis=1)
        gyro = np.stack([window[k] for k in ('gx','gy','gz')], axis=1)
        config = yaml.safe_load((configs/manifest['dataset']/manifest['sequence']/'effective_nominal_config.yaml').read_text())
        rms = float(np.sqrt(np.mean(np.sum(gyro*gyro, axis=1))))
        std = float(np.sqrt(np.sum(np.var(acc, axis=0))))
        norm_error = float(abs(np.linalg.norm(np.mean(acc, axis=0))-config['imu']['gravity_mps2']))
        records.append(dict(dataset=manifest['dataset'], sequence=manifest['sequence'],
            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), samples=len(window),
            window_start_s=float(window['t'][0]), window_end_s=float(window['t'][-1]),
            median_dt_s=dt, gyro_rms_radps=rms, accel_std_mps2=std,
            accel_norm_error_mps2=norm_error,
            stationary_entire_window=(rms<=.05 and std<=.20 and norm_error<=.30),
            accel_sample_std_mps2=np.std(acc,axis=0).tolist(),
            gyro_sample_std_radps=np.std(gyro,axis=0).tolist(),
            provisional_white_accel_density=float(np.sqrt(np.mean(np.var(acc,axis=0))*dt)),
            provisional_white_gyro_density=float(np.sqrt(np.mean(np.var(gyro,axis=0))*dt)),
            existing_parameters=config['imu'],
            qualification='PROVISIONAL_NO_ALLAN_NO_AUTOCORRELATION_OVERBOUND_NO_BIAS_RW_CALIBRATION'))
    return dict(schema='nominal-imu-sensor-statistics/v1', ground_truth_used=False,
                density_conversion_assumption='independent_white_samples; variance_density = sample_variance * dt',
                bias_integration_fallback='diagonal initialization bias prior covariance; never implicit I6',
                records=records)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache',type=Path,default=Path('results/benchmark/cache'))
    p.add_argument('--configs',type=Path,default=Path('results/benchmark/runs/current'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(audit(a.cache,a.configs),indent=2)+'\n')

if __name__=='__main__':main()
