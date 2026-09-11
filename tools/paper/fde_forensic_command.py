#!/usr/bin/env python3
"""Run one recorded command with a hard process-group deadline, no retry."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args()
a.directory.mkdir(parents=True,exist_ok=False);start=time.time()
record={'command':a.command,'cwd':os.getcwd(),'started_unix':start,'timeout_seconds':1800}
(a.directory/'command.json').write_text(json.dumps(record,indent=2)+'\n')
with (a.directory/'stdout.log').open('w') as out,(a.directory/'stderr.log').open('w') as err:
 proc=subprocess.Popen(a.command,stdout=out,stderr=err,start_new_session=True)
 try:code=proc.wait(timeout=1800);record['timeout']=False
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait();code=124;record['timeout']=True
record.update(exit_code=code,elapsed_seconds=time.time()-start)
(a.directory/'command.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record));raise SystemExit(code)
