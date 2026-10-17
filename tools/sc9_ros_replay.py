#!/usr/bin/env python3
"""Freeze, run and compare one canonical direct/1x ROS recording."""
import argparse
import csv
from collections import Counter
import platform
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path
from sc4_system_research import ROOT, BIN, environment

ACTIVE_RUN = None


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--wire', required=True)
    p.add_argument('--model', default='.cache/sc9/empirical-model.json')
    p.add_argument('--run-id', required=True)
    p.add_argument('--mode', choices=['on', 'off'], default='on')
    args = p.parse_args()
    if not args.run_id.replace('-', '').replace('_', '').isalnum():
        raise ValueError('invalid run identity')
    destination = ROOT / '.cache/sc9/ros' / args.run_id
    destination.mkdir(parents=True, exist_ok=False)
    global ACTIVE_RUN
    ACTIVE_RUN = destination
    wire = Path(args.wire).resolve()
    model = Path(args.model).resolve()
    env = environment()
    env['PYTHONPATH'] = ':'.join([str(BIN.parent/'python3/dist-packages'),
                                '/opt/ros/noetic/lib/python3/dist-packages',
                                env.get('PYTHONPATH', '')])
    env['LD_LIBRARY_PATH'] += ':/opt/ros/noetic/lib'
    env['ROS_PACKAGE_PATH'] = str(ROOT)+':/opt/ros/noetic/share'
    env['CMAKE_PREFIX_PATH'] = str(BIN.parent.parent)+':/opt/ros/noetic'
    env['ROS_LOG_DIR'] = str(destination/'roslogs')
    env['ROS_HOME'] = str(destination/'roshome')
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env.pop('ROS_NAMESPACE',None)
    env['ROS_HOSTNAME'] = '127.0.0.1'
    env['ROS_IP'] = '127.0.0.1'
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    env['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    sources = {}
    for folder in ('research', 'include', 'src', 'msg', 'config'):
        for path in sorted((ROOT/folder).rglob('*')):
            if path.is_file() and path.suffix in ('.cpp', '.hpp', '.h', '.msg', '.yaml'):
                sources[str(path.relative_to(ROOT))] = sha(path)
    for path in [ROOT/'CMakeLists.txt', Path(__file__).resolve(), ROOT/'tools/sc9_ros_wire.py']:
        sources[str(path.relative_to(ROOT))] = sha(path)
    loaded = {name:subprocess.check_output(['ldd',str(BIN/name)],env=env,text=True) for name in ('sc9_wire_runner','sc9_ros_node')}
    for listing in loaded.values():
        actual=next(line.split('=>')[1].split()[0] for line in listing.splitlines() if 'libuwb_imu_pl.so =>' in line)
        if Path(actual).resolve() != (BIN.parent/'libuwb_imu_pl.so').resolve():
            raise RuntimeError('wrong runtime DSO')
    freeze = dict(host=platform.uname()._asdict(),cpu_affinity=sorted(os.sched_getaffinity(0)),loaded_libraries=loaded,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                  input_manifest=json.loads((wire/'manifest.json').read_text()),model=sha(model), inputs={n:sha(wire/n) for n in ('events.jsonl','anchors.json','manifest.json')},
                  sources=sources, binaries={n:sha(BIN/n) for n in ('sc9_wire_runner','sc9_ros_node')},
                  dso=sha(BIN.parent/'libuwb_imu_pl.so'), mode=args.mode,
                  primary_latency='original UWB batch header to actual sink, mapped only by original recording receive schedule; no CAL shift subtracted',
                  clock='same-host steady clock; release offsets from original bag receive_ns', speed=1)
    (destination/'freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    running = subprocess.check_output(['ps','-eo','comm'],text=True).splitlines()
    if any(name.strip() in ('cc1plus','cc1','clang','clang++') for name in running):
        raise RuntimeError('compiler active; refusing performance run')
    with (destination/'direct.log').open('w') as log:
        subprocess.run([str(BIN/'sc9_wire_runner'),str(model),str(wire/'anchors.json'),
                        str(wire/'events.jsonl'),str(destination/'direct.csv'),args.mode],
                       cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    processes = []
    logs = []
    def launch(name, command):
        log = (destination/f'{name}.log').open('w');logs.append(log)
        proc = subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc)
        return proc
    try:
        master = launch('master',['/opt/ros/noetic/bin/roscore','-p',str(port)])
        deadline = time.monotonic()+30
        while True:
            try:
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/sc9_probe')[0] == 1:
                    break
            except OSError:
                pass
            if master.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('isolated ROS master failed')
            time.sleep(.05)
        sink = launch('sink',['/usr/bin/python3',str(ROOT/'tools/sc9_ros_wire.py'),'sink',
                             '--header',str(destination/'direct.csv'),'--events',str(wire/'events.jsonl'),'--output',str(destination/'ros.csv'),
                             '--timing',str(destination/'sink.jsonl'),'--status',str(destination/'sink-status.json')])
        node = launch('node',[str(BIN/'sc9_ros_node'),'_integrity_backend:=unified_rolling',
                             '_model:='+str(model),'_anchors:='+str(wire/'anchors.json'),'_rolling:='+args.mode,
                             '_input_audit:='+str(destination/'arrivals.csv')])
        producer = launch('producer',['/usr/bin/python3',str(ROOT/'tools/sc9_ros_wire.py'),'publish',
                                     '--events',str(wire/'events.jsonl'),'--output',str(destination/'releases.csv')])
        for name, proc in [('producer',producer),('node',node),('sink',sink)]:
            if proc.wait(timeout=180) != 0:
                raise RuntimeError(name+' failed; retain all partial artifacts')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
    read = lambda path:list(csv.DictReader(path.open()))
    direct, ros = read(destination/'direct.csv'), read(destination/'ros.csv')
    timings = [json.loads(line) for line in (destination/'sink.jsonl').open()]
    ignored = {'core_ms','prepare_ms','candidate_ms','proof_ms','commit_ms','cleanup_ms','rss_kb'}
    mismatches = []
    for a,b in zip(direct,ros):
        for key in a:
            if key not in ignored and a[key]!=b[key]:
                mismatches.append([a['epoch'],key,a[key],b[key]])
    def distribution(values):
        ordered=sorted(values)
        return dict(n=len(ordered),median=ordered[math.ceil(.5*len(ordered))-1],
                    p95=ordered[math.ceil(.95*len(ordered))-1],p99=ordered[math.ceil(.99*len(ordered))-1],max=max(ordered))
    misses = [dict(epoch=r['epoch'],core_ms=r['core_ms'],measurement_to_sink_ms=r['measurement_to_sink_ms'])
              for r in timings if r['core_ms']>40 or r['measurement_to_sink_ms']>50]
    status=json.loads((destination/'sink-status.json').read_text())
    startup=next((line.split('startup_digest=')[1].split()[0] for line in (destination/'direct.log').read_text().splitlines() if 'startup_digest=' in line),None)
    startup_equal=bool(startup and ('startup_digest='+startup) in status['finished'][0]['summary'])
    summary=dict(planned=len(direct),received=len(ros),timing_rows=len(timings),mismatches=mismatches,
                 startup_equal=startup_equal,core_ms=distribution([r['core_ms'] for r in timings]),
                 measurement_to_sink_ms=distribution([r['measurement_to_sink_ms'] for r in timings]),
                 worker_queue_ms=distribution([r['worker_queue_ms'] for r in timings]),misses=misses,
                 timely_ratio=(len(timings)-len(misses))/len(direct),queue_high_water=max(r['queue_high_water'] for r in timings),
                 protected_publication=False,hardware_raw_qualified=False)
    releases=read(destination/'releases.csv')
    arrivals=read(destination/'arrivals.csv')
    summary['input_events_received']=len(arrivals)-1
    summary['input_sequence_complete']=([int(r['event_sequence']) for r in arrivals]==list(range(1,len(releases)+2)) and all(r['accepted']=='1' for r in arrivals))
    summary['transport_arrival_ms']=distribution([(int(r['callback_ns'])-int(r['release_ns']))/1e6 for r in arrivals])
    summary['release_lateness_ms']=distribution([int(r['lateness_ns'])/1e6 for r in releases])
    summary['status_counts']=dict(Counter(r['status'] for r in ros))
    summary['committed_ratio']=len(ros)/len(direct)
    summary['clock_latency_diagnostics']={key:distribution([r[key] for r in timings if key in r])
        for key in ('earliest_range_to_sink_ms','latest_range_to_sink_ms','model_epoch_to_sink_ms','latest_used_sensor_to_sink_ms')}
    summary['same_wire_pass']=len(direct)==len(ros)==len(timings) and not mismatches and startup_equal and summary['input_sequence_complete']
    summary['frozen_inputs_unchanged']=all(sha(wire/name)==digest for name,digest in freeze['inputs'].items()) and sha(model)==freeze['model'] and sha(BIN.parent/'libuwb_imu_pl.so')==freeze['dso']
    summary['same_wire_pass'] &= summary['frozen_inputs_unchanged']
    summary['realtime_pass']=summary['same_wire_pass'] and summary['core_ms']['p95']<=40 and summary['measurement_to_sink_ms']['p95']<=50 and summary['timely_ratio']>=.95
    (destination/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('mismatches','misses')},indent=2))
    if not summary['same_wire_pass']:
        raise RuntimeError('direct/ROS canonical parity failed')


if __name__=='__main__':
    try:
        main()
    except Exception as error:
        if ACTIVE_RUN is not None:
            rows = {}
            for name in ('direct.csv','ros.csv','releases.csv','arrivals.csv'):
                path=ACTIVE_RUN/name
                rows[name]=max(0,sum(1 for _ in path.open())-1) if path.exists() else 0
            (ACTIVE_RUN/'failure.json').write_text(json.dumps(dict(error=str(error),retained_prefix_rows=rows),indent=2)+'\n')
        raise
