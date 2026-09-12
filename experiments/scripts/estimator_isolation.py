"""Fail-closed mount allowlist for measurement-only estimator processes."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def sandbox_command(command, readonly_files, writable_directory):
    """Empty filesystem root plus system runtime and explicit data files only."""
    output=Path(writable_directory).resolve()
    if not output.is_dir(): raise ValueError('sandbox output directory missing')
    # A fresh batch/runs directory is owned only by the estimator scheduler.
    prefix=['bwrap','--unshare-all','--die-with-parent','--new-session']
    for p in ['/usr','/bin','/sbin','/lib','/lib64']:
        if Path(p).exists(): prefix+=['--ro-bind',p,p]
    prefix+=['--ro-bind','/etc/ld.so.cache','/etc/ld.so.cache',
             '--proc','/proc','--dev','/dev','--tmpfs','/tmp','--chdir','/']
    for p in sorted(set(map(lambda x:str(Path(x).absolute()),readonly_files))):
        source=Path(p)
        if not source.is_file(): raise ValueError('allowlist requires a file: '+p)
        prefix+=['--ro-bind',str(source.resolve()),p]
    prefix+=['--bind',str(output),str(output),'--']
    return prefix+list(map(str,command))


def measurement_files(config):
    cfg=yaml.safe_load(Path(config).read_text())
    dataset=cfg.get('dataset',{})
    if dataset.get('interface')!='t07_cache': raise ValueError('measurement-only t07 cache required')
    if cfg.get('sfuise') or cfg.get('topics',{}).get('vicon'):
        raise ValueError('GT-capable bag configuration forbidden in estimator sandbox')
    manifest=Path(dataset['cache_manifest']).resolve()
    d=json.loads(manifest.read_text())
    if d.get('schema')!='nlos_measurement_cache_v2': raise ValueError('unsupported input schema')
    if any(any(word in k.lower() for word in ['ground_truth','oracle','gt_range','range_error']) for k in d):
        raise ValueError('GT-derived input manifest forbidden')
    files=[Path(config).resolve(),manifest]
    for kind in ['uwb','imu']:
        name=d[kind+'_file']
        if Path(name).name!=name: raise ValueError('input payload path must be a basename')
        p=manifest.parent/name
        if sha(p)!=d[kind+'_sha256'].split(':')[-1]: raise ValueError('measurement hash mismatch')
        # Allow only the locked clean Walk1 cache, not a relabeled GT-enriched payload.
        files.append(p)
    if sha(manifest)!='7af404c8580ecf0b9f9d2742bbc56d03ed9a592172603d3df0e8a0d569e22c0e':
        from canonical_injection import validate_child
        validate_child(manifest)
    return files


def runner_call(binary,config,runroot,rid,method,execution,cache,point,anchors,overrides):
    command=[str(binary.resolve()),'--config',str(config),'--output-root',str(runroot),
             '--run-id',rid,'--execution-type',execution]
    if method: command+=['--method',method]
    if cache: command+=['--stage2-cache-manifest',str(cache)]
    if point: command+=['--operating-point-id',point]
    if anchors: command+=['--anchor-ids',','.join(map(str,anchors))]
    files=measurement_files(config)+[binary.resolve()]
    # ldd is read by the host orchestrator; mount each non-system library at its required pathname.
    ldd=subprocess.check_output(['ldd',str(binary)],text=True)
    for line in ldd.splitlines():
        if '=> /' in line:
            p=Path(line.split('=>',1)[1].strip().split()[0])
            if not str(p).startswith(('/usr/','/lib/','/lib64/')): files.append(p)
    if cache:
        cache=Path(cache);files.append(cache)
        data=json.loads(cache.read_text())
        if data.get('stage1_provider')!='PL_BIDIRECTIONAL_CUSUM_V1':
            raise ValueError('wrong parent provider')
        for payload in data['payloads']:
            name=payload['name']
            if Path(name).name!=name or any(w in name.lower() for w in ['ground_truth','range_metrics','oracle']):
                raise ValueError('invalid estimator cache payload')
            p=cache.parent/name
            if sha(p)!=payload['sha256'].split(':')[-1]: raise ValueError('parent payload hash mismatch')
            files.append(p)
    wrapped=sandbox_command(command,files,runroot)
    record={'schema':'estimator_mount_allowlist_v1','command':wrapped,
            'read_only_files':{str(p):sha(p) for p in files},'writable_directory':str(runroot),
            'gt_mounts':[],'network':'UNSHARED','host_home':'ABSENT'}
    (runroot.parent/(rid+'.isolation.json')).write_text(json.dumps(record,indent=2)+'\n')
    env={'PATH':'/usr/bin:/bin','LANG':'C','HOME':'/tmp',
         'LD_LIBRARY_PATH':':'.join(sorted({str(p.parent) for p in files if '.so' in p.name})),**overrides}
    start=time.monotonic()
    process=subprocess.Popen(wrapped,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                             env=env,start_new_session=True)
    try:
        stdout,stderr=process.communicate(timeout=1800)
        code=process.returncode
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGKILL);stdout,stderr=process.communicate();code=124
    (runroot.parent/(rid+'.sandbox.stdout.log')).write_text(stdout)
    (runroot.parent/(rid+'.sandbox.stderr.log')).write_text(stderr)
    if code and not (runroot/rid).exists():
        failed=runroot/rid;failed.mkdir()
        (failed/'run_status.json').write_text(json.dumps({'status':'FAILED','exit_code':code,
            'reason':'SANDBOX_BACKEND_FAILED: '+stderr[-1000:],'valid_estimate_exported':False})+'\n')
    return code,time.monotonic()-start,stdout,stderr,None
