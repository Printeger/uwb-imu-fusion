#!/usr/bin/env python3
"""Runs inside an empty filesystem/network namespace; receives one sensor bag only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client


def wait_connections(env):
    master=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
    expected={'/SplineFusion':{'/EstimationInterface/imu_ds','/EstimationInterface/toa_ds'},
              '/sfuise_trajectory_adapter':{'/EstimationInterface/toa_ds','/SplineFusion/est_window','/SplineFusion/sys_calib'}}
    proof={}
    for _ in range(200):
        try:
            for node,topics in expected.items():
                code,_,uri=master.lookupNode('/formal_readiness',node)
                if code!=1:raise RuntimeError('NODE_NOT_REGISTERED')
                code,_,connections=xmlrpc.client.ServerProxy(uri).getBusInfo('/formal_readiness')
                connected={r[4] for r in connections if r[2]=='i' and r[5]}
                if not topics<=connected:raise RuntimeError('SUBSCRIBER_NOT_CONNECTED')
                proof[node]=connections
            return proof
        except Exception:time.sleep(.1)
    raise RuntimeError('SUBSCRIBER_READINESS_TIMEOUT')


def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:p.kill();p.wait()


def main():
    out=Path(sys.argv[1]);build=Path(sys.argv[2]);bag=Path(sys.argv[3]);config=Path(sys.argv[4])
    ready='--ready' in sys.argv
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11450',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',
             ROS_HOME=str(out/'.ros'),ROS_PACKAGE_PATH='/opt/ros/noetic/share',ROS_ROOT='/opt/ros/noetic/share/ros',
             PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages',PATH='/opt/ros/noetic/bin:/usr/bin:/bin',
             LD_LIBRARY_PATH='/opt/ros/noetic/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu')
    procs=[];logs=[];commands=[];start=time.monotonic()
    def launch(name,cmd):
        log=(out/(name+'.log')).open('w');logs.append(log);commands.append(cmd)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env);procs.append(p);return p
    status={'status':'failed','gt_available':False,'injection_truth_available':False}
    try:
        launch('master',['roscore','-p','11450'])
        for _ in range(100):
            if subprocess.call(['rosnode','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)==0:break
            time.sleep(.1)
        else:raise RuntimeError('ROS_MASTER_TIMEOUT')
        for ns in ('/EstimationInterface','/SplineFusion'):
            subprocess.run(['rosparam','load',str(config),ns],env=env,check=True)
        interface=launch('interface',[str(build/'devel/lib/sfuise/EstimationInterface'),'__name:=EstimationInterface',
                                     '/SplineFusion/sys_calib:=/sfuise_adapter/interface_calib_blocked'])
        fusion=launch('fusion',[str(build/'devel/lib/sfuise/SplineFusion'),'__name:=SplineFusion'])
        adapter=launch('adapter',[str(build/'devel/lib/sfuise_baseline_adapter/sfuise_trajectory_adapter'),
                                 '__name:=sfuise_trajectory_adapter','_output_path:='+str(out/'trajectory.tum'),
                                 '_metadata_path:='+str(out/'adapter_runtime.txt')])
        time.sleep(2)
        if any(p.poll() is not None for p in (interface,fusion,adapter)):raise RuntimeError('STARTUP_PROCESS_DIED')
        if ready:
            status['readiness_connections']=wait_connections(env)
            recorder=launch('sensor_transport',['rosbag','record','-O',str(out/'received.bag'),
                                                '/EstimationInterface/imu_ds','/EstimationInterface/toa_ds','/SplineFusion/start_time'])
            time.sleep(2)
        play=launch('play',['rosbag','play','--quiet']+(['--wait-for-subscribers','--delay=1'] if ready else [])+
                    [str(bag),'--topics','/waveshare_sense_hat_b','/rtls_flares','/anchor_list'])
        if play.wait(timeout=1000):raise RuntimeError('PLAY_FAILED')
        time.sleep(5);stop(adapter)
        if ready:stop(recorder)
        if any(p.poll() not in (None,0) for p in (interface,fusion)):raise RuntimeError('SF_PROCESS_DIED')
        if not (out/'trajectory.tum').is_file() or not (out/'trajectory.tum').stat().st_size:raise RuntimeError('EMPTY_TRAJECTORY')
        status.update(status='completed',failure_reason='',exit_code=0)
    except Exception as e:status.update(status='failed',failure_reason=type(e).__name__+': '+str(e),exit_code=1)
    finally:
        for p in reversed(procs):stop(p)
        for log in logs:log.close()
        status.update(runtime=time.monotonic()-start,commands=commands)
        (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    return status['exit_code']


if __name__=='__main__':raise SystemExit(main())
