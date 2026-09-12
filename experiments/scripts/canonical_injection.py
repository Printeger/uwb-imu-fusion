"""One preregistered measurement transform; truth stays outside estimator mounts."""
import csv
import io
import json
from pathlib import Path
import sys
import run_walk1_smoke as h
sys.path.insert(0,str(h.ROOT/'tools/paper'))
from nlos_injection import cache_id

PARENT=h.ROOT.parents[1]/'res/nlos_injection_20260911_01/inputs/sfuise_walk1_normal_clean/input_manifest.json'
PARENT_SHA='7af404c8580ecf0b9f9d2742bbc56d03ed9a592172603d3df0e8a0d569e22c0e'
SPEC={'schema':'ICRA_CANONICAL_POSITIVE_RANGE_V1','tag_id':'27956','anchor_id':'20276',
      'onset_s':1664959684.9745398,'offset_s':1664959694.9745398,'interval':'[onset,offset)',
      'bias_m':1.0,'duration_s':10.0,'measurement_type':'ABSOLUTE_TOA'}


def targeted(row):
    return row['tag_id']==SPEC['tag_id'] and row['anchor_id']==SPEC['anchor_id'] and SPEC['onset_s']<=float(row['sensor_time_s'])<SPEC['offset_s']


def transform(payload):
    # Preserve bytes outside the range token, including every ID/time/validity and newline.
    lines=payload.decode().splitlines(keepends=True);fields=lines[0].strip().split(',')
    idx=fields.index('observed_range_m');out=[lines[0]];affected=[]
    for line in lines[1:]:
        end='\r\n' if line.endswith('\r\n') else ('\n' if line.endswith('\n') else '')
        values=line.rstrip('\r\n').split(',')
        if len(values)!=len(fields):raise ValueError('noncanonical measurement CSV')
        row=dict(zip(fields,values))
        if targeted(row):
            old=float(values[idx]);values[idx]=format(old+SPEC['bias_m'],'.17g')
            affected.append({'obs_id':row['obs_id'],'timestamp':float(row['sensor_time_s']),
                             'anchor_id':row['anchor_id'],'source_valid':row['source_valid'],
                             'clean_range':old,'corrupted_range':float(values[idx]),'injected_bias':SPEC['bias_m']})
        out.append(','.join(values)+end)
    return ''.join(out).encode(),affected


def expected_child():
    if h.sha(PARENT)!=PARENT_SHA:raise ValueError('clean parent identity mismatch')
    parent=h.read(PARENT)
    for kind in ('uwb','imu'):
        if 'sha256:'+h.sha(PARENT.parent/parent[kind+'_file'])!=parent[kind+'_sha256']:raise ValueError('parent payload changed')
    data,affected=transform((PARENT.parent/parent['uwb_file']).read_bytes())
    import hashlib
    child=dict(parent,uwb_sha256='sha256:'+hashlib.sha256(data).hexdigest(),
               transform_sha256='sha256:'+h.digest({'parent_sha256':PARENT_SHA,'specification':SPEC}))
    child['cache_id']=cache_id(child)
    return child,data,affected


def validate_child(manifest):
    expected,data,_=expected_child();manifest=Path(manifest)
    if h.read(manifest)!=expected:raise ValueError('not the canonical sealed child manifest')
    if (manifest.parent/expected['uwb_file']).read_bytes()!=data:raise ValueError('child differs from exact preregistered range-only transform')
    if 'sha256:'+h.sha(manifest.parent/expected['imu_file'])!=expected['imu_sha256']:raise ValueError('IMU changed')


def generate(directory,private):
    directory=Path(directory);private=Path(private)
    if directory.resolve()==private.resolve() or directory.resolve() in private.resolve().parents or private.resolve() in directory.resolve().parents:
        raise ValueError('truth must be disjoint from estimator input')
    child,data,affected=expected_child()
    directory.mkdir(parents=True,exist_ok=False);private.mkdir(parents=True,exist_ok=False)
    (directory/child['imu_file']).symlink_to(PARENT.parent/child['imu_file'])
    (directory/child['uwb_file']).write_bytes(data);h.write(directory/'input_manifest.json',child)
    truth={'schema':'ICRA_INJECTION_TRUTH_V1','semantics':'INJECTED_COMPONENT_ONLY_NOT_TOTAL_NLOS',
           'specification':SPEC,'affected_obs_ids':[r['obs_id'] for r in affected],
           'parent_manifest':str(PARENT),'parent_manifest_sha256':h.sha(PARENT),
           'child_manifest':str(directory/'input_manifest.json'),'child_manifest_sha256':h.sha(directory/'input_manifest.json'),
           'parent_recording_id':'ISAS-Walk1','child_recording_id':'ISAS-Walk1-canonical-positive-1m-10s',
           'parent_cache_id':h.read(PARENT)['cache_id'],'child_cache_id':child['cache_id'],
           'imu_sha256':child['imu_sha256'],'gt_path':str(h.GT),'gt_sha256':h.sha(h.GT)}
    h.write(private/'injection_truth.json',truth);h.save_csv(private/'affected_observations.csv',affected)
    validate_child(directory/'input_manifest.json')
    return directory/'input_manifest.json'
