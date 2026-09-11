#!/usr/bin/env python3
import csv,json,sys
import yaml
from pathlib import Path
root=Path(sys.argv[1]);output=Path(sys.argv[2])
ledger=json.loads((root/'ledger.json').read_text());assert len(ledger)==24
by_input={}
for row in ledger:
 assert row['exit_code']==0 and row['status']['optimizer_calls']==0
 unit=row['input'];run=root/'runs'/(unit+'-'+row['method'])
 common=json.loads((run/'common_preparation.json').read_text())['common_preparation_id']
 if unit in by_input:assert by_input[unit]['common_preparation_id']==common
 else:by_input[unit]=dict(common_preparation_id=common,counts=row['status'])
 if unit.startswith('miluv'):
  rows=list(csv.DictReader((run/'observations.csv').open()))
  assert all(r['raw_tag_id']=='10' for r in rows)
  assert len({r['source_message'] for r in rows})==len(rows)
  assert all(r['source_message']==r['source_observation'] and r['source_range']=='0' for r in rows)
extra=root.parent/'extra_prepares'
subset=list(csv.DictReader((extra/'anchor_subset/observations.csv').open()))
assert {r['anchor_id'] for r in subset if r['valid']=='1'} <= {'7475','9524','10548','15155'}
full=list(csv.DictReader((root/'runs/miluv_random-lcb_partial/observations.csv').open()))
crop=list(csv.DictReader((extra/'miluv_crop/observations.csv').open()))
by_id={r['obs_id']:r for r in full}
assert crop and len(crop)<len(full)
for row in crop:
 assert row['obs_id'] in by_id
 for field in ['source_message','source_observation','raw_time','raw_z_m','raw_tag_id']:
  assert row[field]==by_id[row['obs_id']][field]
cfg=yaml.safe_load((extra.parent/'miluv_crop_config.yaml').read_text())
source=Path(cfg['miluv']['data_dir'])/cfg['miluv'].get('robot_dir','ifo001')
with (source/cfg['miluv'].get('imu_csv','imu_px4.csv')).open() as f:
 imu_start=min(float(r['timestamp']) for r in csv.DictReader(f))
with (source/cfg['miluv'].get('uwb_csv','uwb_range.csv')).open() as f:
 raw=list(csv.reader(f))[1:]
 uwb_start=min(float(r[-1]) for r in raw if float(r[-1])>=0.001)
origin=min(imu_start,uwb_start)
begin=origin+cfg['bag']['start'];end=begin+cfg['bag']['durr']
assert {r['obs_id'] for r in crop}=={r['obs_id'] for r in full if begin<=float(r['raw_time'])<=end}
output.write_text(json.dumps(dict(status='PASS',prepared=24,inputs=by_input,
 miluv_crop_source_rows_preserved=len(crop),real_anchor_subset_before_initialization=True),indent=2))
print('PASS 24 shared preparations; MILUV source rows/tag 10; crop stable IDs; real anchor subset')
