#!/usr/bin/env python3
"""CAL-only falsification of tracker=IMU; never applies a fitted transform."""
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import run_dataset_benchmark as dataset
from benchmark_common import ROOT,sha256

def main():
    cache=ROOT/'.cache/sc8/data/CAL-W'
    measured,target=dataset._own_gyro_pairs([cache],0.)
    fitted,error=dataset._wahba(measured-measured.mean(0),target-target.mean(0))
    identity=float(np.sqrt(np.mean(np.sum((measured-target)**2,axis=1))))
    angle=float(np.degrees(Rotation.from_matrix(fitted).magnitude()))
    result=dict(status='OPEN_REFERENCE_CONTRACT',scope='CAL-W only; read-only diagnostic, no transform applied to input, GT or estimator',
        pairs=len(measured),identity_rmse_radps=identity,centered_rotation_fit_rmse_radps=error,diagnostic_rotation_deg=angle,
        diagnostic_R_tracker_from_imu=fitted.tolist(),excitation_singular_values=np.linalg.svd(measured-measured.mean(0),compute_uv=False).tolist(),
        source_files={name:sha256(cache/name) for name in ['imu.csv','gt.csv','manifest.json']},
        reason='Frozen Walk adapter equated VIVE tracker/body and added the IMU-tag lever. CAL gyro derivatives contradict identity. Full tracker-body-tag translation/rotation and gravity frame remain unverified; numerical scores are proxies, not physical APE/ARE.',
        primary_source='https://isas.iar.kit.edu/pdf/RAL23_Li.pdf',original_parameter_changes=False,new_FGO_solves=0)
    star=ROOT/'.cache/sc8/data/CAL-S';manifest=json.loads((star/'manifest.json').read_text());original=Path(manifest['source'])
    calib=json.loads((original/'calib.json').read_text())
    def rotation(key):
        x=calib[key];return Rotation.from_quat([x['rot_x'],x['rot_y'],x['rot_z'],x['w']]).as_matrix()
    a=rotation('tf_cam_rig');b=rotation('tf_cam_imu');frozen=a.T@b;candidate=a@b.T
    measured,target=dataset._own_gyro_pairs([star],0.);raw=measured@frozen
    uwb=pd.read_csv(original/'uwb.csv');uwb=uwb[uwb.from_id==manifest['primary_tag']].iloc[::8]
    lever=pd.read_csv(star/'uwb.csv').iloc[0][['lever_x','lever_y','lever_z']].to_numpy(float);q=Rotation.from_quat(uwb[['rot_x','rot_y','rot_z','w']].to_numpy())
    residual=uwb[['tag_pos_x','tag_pos_y','tag_pos_z']].to_numpy()-uwb[['x','y','z']].to_numpy()-q.apply(lever)
    result['STAR_CAL_only']=dict(status='INPUT_GYRO_FRAME_VALIDATION_OPEN',pairs=len(measured),
        frozen_gyro_derivative_rmse_radps=float(np.sqrt(np.mean(np.sum((measured-target)**2,axis=1)))),
        published_direction_candidate_rmse_radps=float(np.sqrt(np.mean(np.sum((raw@candidate.T-target)**2,axis=1)))),
        frozen_R=frozen.tolist(),published_direction_candidate_R=candidate.tolist(),candidate_applied=False,
        official_tag_lever_consistency_rmse_m=float(np.sqrt(np.mean(np.sum(residual*residual,axis=1)))),
        source_files={name:sha256(star/name) for name in ['imu.csv','gt.csv','manifest.json']},
        published_metadata_sha256=sha256(original.parents[1]/'README.md'),
        reason='Published transform direction labels and frozen composition differ. CAL angular consistency does not close either interpretation; no TEST fit or silent conversion change. Official tag positions/rig quaternion satisfy the CAD lever exactly, so position errors remain observed pipeline errors, not evidence against an algorithm under validated inputs.')
    assert len(measured)>100 and angle>150 and error<identity/2,'Diagnostic changed; review reference scope rather than automatically fitting TEST'
    out=ROOT/'.cache/sc8/reference-check/result.json';out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('Walk physical reference OPEN; proxy preservation and suppression required:',identity,error,angle)

if __name__=='__main__':main()
