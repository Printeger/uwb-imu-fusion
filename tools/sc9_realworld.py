#!/usr/bin/env python3
"""SC9 native integration checks. Intermediate evidence is owned by .cache/sc9."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

from sc4_system_research import ROOT, BIN, environment

CACHE = ROOT / '.cache/sc9'
# Fixed before observing these runs. Both fault rows extend past injection and x0 exit.
LEGACY = [('S0', 35), ('F0', 76), ('F3', 76), ('S3', 35)]
SEMANTIC = '''epoch mutations updates qualified status recovery version generation
parent boundary x0 source_prefix risk q shared_q alerts uwb_alarm bridge rejoined
excluded repair removed_priors removed_groups hpl vpl bias_width slots live ledger
replay capsules width_x width_y width_z vwidth_x vwidth_y vwidth_z rwidth_x rwidth_y
rwidth_z bwidth_0 bwidth_1 bwidth_2 bwidth_3 bwidth_4 bwidth_5 tag_x tag_y tag_z px py
pz vx vy vz qw qx qy qz bax bay baz bgx bgy bgz stamp_ns fd_status imu_alarm
alert_generation first_imu_alarm_ns imu_active imu_ended imu_excluded uwb_active
imu_no_active imu_q temporal_alerts active_states proof_explicit support_boxes
support_frontier bound_method joint_branches joint_empty joint_frontier joint_fallback
formal_eligible publication_protected application_available'''.split()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity():
    env = environment()
    binary = BIN / 'epoch_raim_runner'
    loaded = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    dso = next(line.split('=>')[1].split()[0] for line in loaded.splitlines()
               if 'libuwb_imu_pl.so =>' in line)
    return dict(head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                                             text=True).strip(),
                elf=sha(binary), dso=sha(dso), loaded_dso=dso,
                requirements=sha(ROOT / 'docs/requirements/sc9/SC9_REALWORLD_INTEGRATION_REQUIREMENTS.md'))


def legacy(freeze):
    CACHE.mkdir(parents=True, exist_ok=True)
    baseline = CACHE / 'legacy-baseline.json'
    if freeze and baseline.exists():
        raise RuntimeError('baseline exists; refusing to overwrite pre-change evidence')
    result = dict(identity=identity(), profiles={})
    for scene, count in LEGACY:
        path = CACHE / f'legacy-{scene}-{"before" if freeze else "after"}.csv'
        command = [BIN / 'epoch_raim_runner', ROOT / 'config/sc7.yaml', path,
                   scene, 'sc6', 'on', str(count), 'tight-optimized', 'offline', '20261010']
        with path.with_suffix('.log').open('w') as log:
            subprocess.run(list(map(str, command)), cwd=ROOT, env=environment(),
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == count
        assert all(r['updates'] == r['mutations'] == r['qualified'] == '1' for r in rows)
        assert all(r['publication_protected'] == '0' for r in rows)
        assert any(r['x0'] == '0' for r in rows)
        result['profiles'][scene] = [{k: r[k] for k in SEMANTIC} for r in rows]
        print(scene, 'commits/raw', len(rows), 'repairs', sum(r['repair'] == '1' for r in rows),
              'rejoins', sum(r['rejoined'] == '1' for r in rows), flush=True)
    if not freeze:
        old = json.loads(baseline.read_text())
        for scene, _ in LEGACY:
            for before, after in zip(old['profiles'][scene], result['profiles'][scene]):
                for key in SEMANTIC:
                    assert before[key] == after[key], (scene, before['epoch'], key, before[key], after[key])
    target = baseline if freeze else CACHE / 'legacy-parity.json'
    target.write_text(json.dumps(result, indent=2) + '\n')
    print('FROZEN' if freeze else 'EXACT_LEGACY_PARITY', result['identity'])


def check():
    CACHE.mkdir(parents=True, exist_ok=True)
    for name in ['sc9_input_checks', 'sc9_event_checks']:
        with (CACHE / (name + '.log')).open('w') as log:
            subprocess.run([str(BIN / name)], cwd=ROOT, env=environment(),
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        print((CACHE / (name + '.log')).read_text(), end='')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('command', choices=['legacy-freeze', 'legacy-parity', 'check'])
    args = parser.parse_args()
    if args.command == 'check':
        check()
    else:
        legacy(args.command == 'legacy-freeze')
