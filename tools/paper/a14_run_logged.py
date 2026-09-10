#!/usr/bin/env python3
"""One bounded command with argv, exit, file access and actual library evidence."""
import argparse,datetime,hashlib,json,os,re,subprocess,time
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--record',required=True);p.add_argument('--seconds',type=int,default=120);p.add_argument('command',nargs=argparse.REMAINDER);a=p.parse_args();cmd=a.command
 if cmd and cmd[0]=='--':cmd=cmd[1:]
 r=Path(a.record).resolve();r.mkdir(parents=True,exist_ok=False)
 argv=['/usr/bin/time','-v','-o',str(r/'resources.txt'),'/usr/bin/timeout','--signal=KILL',str(a.seconds)+'s','/usr/bin/strace','-f','-e','trace=openat,open,execve','-o',str(r/'file_access.trace')]+cmd
 meta={'argv':argv,'cwd':str(Path.cwd()),'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'timeout_s':a.seconds}
 (r/'command.json').write_text(json.dumps(meta,indent=2)+'\n')
 start=time.monotonic()
 with (r/'stdout.log').open('wb') as out,(r/'stderr.log').open('wb') as err:
  proc=subprocess.Popen(argv,stdout=out,stderr=err)
  seen={}
  while proc.poll() is None:
   # Follow only descendants; saves actual maps for even short GTests.
   pending=[proc.pid]
   while pending:
    pid=pending.pop()
    try:
     pending += [int(v) for v in Path(f'/proc/{pid}/task/{pid}/children').read_text().split()]
     exe=os.readlink(f'/proc/{pid}/exe')
     if Path(exe).name in {'strace','time','timeout'}:continue
     maps=Path(f'/proc/{pid}/maps').read_text()
     if len(maps)>len(seen.get(pid,'')):
      seen[pid]=maps;(r/f'process_{pid}_maps.txt').write_text(maps)
    except (OSError,ProcessLookupError):pass
   time.sleep(.01)
  code=proc.wait()
 meta.update(exit_code=code,external_wall_s=time.monotonic()-start,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
 trace=(r/'file_access.trace').read_text(errors='replace')
 forbidden=[l for l in trace.splitlines() if re.search(r'/(evaluation|truth|ground_truth)/|generation_manifest|bias_truth|ground_truth\.csv',l,re.I)]
 meta['forbidden_input_open_lines']=forbidden
 # Successful open/mmap candidates supplemented by actual maps. Every ELF path
 # has full SHA/build ID; an open alone is identified separately from mapping.
 paths=set();mapped=set()
 for text in seen.values():
  for l in text.splitlines():
   fields=l.split(None,5)
   if len(fields)==6 and fields[5].startswith('/') and Path(fields[5]).is_file():mapped.add(fields[5])
 for l in trace.splitlines():
  if ' = -1 ' in l:continue
  for path in re.findall(r'"(/[^"\n]+)"',l):
   if '.so' in Path(path).name and Path(path).is_file():paths.add(path)
 ids=[]
 for path in sorted(paths|mapped):
  f=Path(path).resolve()
  try:
   if f.open('rb').read(4)!=b'\x7fELF':continue
   data=f.read_bytes();notes=subprocess.check_output(['readelf','-n',str(f)],text=True,stderr=subprocess.DEVNULL)
   build=re.search(r'Build ID: (\S+)',notes)
   ids.append({'path':path,'realpath':str(f),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'build_id':build.group(1) if build else None,'mapped':path in mapped})
  except OSError:pass
 meta['elf_identity']=ids;meta['maps_captured']=bool(seen)
 (r/'command.json').write_text(json.dumps(meta,indent=2)+'\n')
 print(json.dumps({'command':cmd,'exit_code':code,'wall_s':meta['external_wall_s'],'forbidden_count':len(forbidden),'mapped_elf_count':sum(i['mapped'] for i in ids)}))
 return 1 if forbidden else code
if __name__=='__main__':raise SystemExit(main())
