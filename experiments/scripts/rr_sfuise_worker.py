#!/usr/bin/env python3
"""Inside a private mount/network namespace: no GT/support/recovery inputs."""
import json,os,signal,subprocess,sys,time
from pathlib import Path


def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:p.kill();p.wait()


def main():
    d=Path(sys.argv[1]);build=Path(sys.argv[2]);probe='--probe' in sys.argv;out=d/('sfuise_probe' if probe else 'sfuise')
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11450',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',
        ROS_HOME=str(out/'.ros'),ROS_PACKAGE_PATH='/opt/ros/noetic/share',ROS_ROOT='/opt/ros/noetic/share/ros',
        PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages',PATH='/opt/ros/noetic/bin:/usr/bin:/bin',
        LD_LIBRARY_PATH='/opt/ros/noetic/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu')
    processes=[];logs=[];commands=[]
    def launch(name,cmd):
        f=(out/(name+'.log')).open('w');logs.append(f);commands.append(cmd)
        p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT);processes.append(p);return p
    status=dict(status='RUNNING',commands=commands,gt_available=False,estimate_origin='IMU_ORIGIN_RIG_AXES')
    start=time.monotonic()
    try:
        master=launch('master',['roscore','-p','11450'])
        for _ in range(100):
            try:
                code=subprocess.run(['rosnode','list'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=2).returncode
                if code==0:break
            except subprocess.TimeoutExpired:pass
            time.sleep(.1)
        else:raise RuntimeError('ROS_MASTER_TIMEOUT')
        for ns in ['/EstimationInterface','/SplineFusion']:
            subprocess.run(['rosparam','load',str(d/'sfuise.yaml'),ns],env=env,check=True)
        interface=launch('interface',[str(build/'devel/lib/sfuise/EstimationInterface'),'__name:=EstimationInterface',
                 '/SplineFusion/sys_calib:=/sfuise_adapter/interface_calib_blocked'])
        fusion=launch('fusion',[str(build/'devel/lib/sfuise/SplineFusion'),'__name:=SplineFusion'])
        adapter=launch('adapter',[str(build/'devel/lib/sfuise_baseline_adapter/sfuise_trajectory_adapter'),
                 '__name:=sfuise_trajectory_adapter','_output_path:='+str(out/'trajectory.tum'),'_metadata_path:='+str(out/'adapter_runtime.txt')])
        time.sleep(2)
        if any(p.poll() is not None for p in [interface,fusion,adapter]):raise RuntimeError('STARTUP_PROCESS_DIED')
        if probe:
            status.update(status='SUCCESS',exit_code=0,probe_only=True,sensor_messages=0)
            return 0
        # Bag contains exactly measurement topics and static anchors; playback naturally
        # preserves asynchronous messages. No artificial grouping or raw-range correction.
        play=launch('play',['rosbag','play','--quiet',str(d/'measurements.bag'),'--topics','/rr/imu','/rtls_flares','/anchor_list'])
        code=play.wait(timeout=1000)
        if code:raise RuntimeError('BAG_PLAY_EXIT_'+str(code))
        time.sleep(5);stop(adapter)
        if any(p.poll() not in (None,0) for p in [interface,fusion]):raise RuntimeError('SFUISE_PROCESS_DIED')
        if not (out/'trajectory.tum').is_file() or (out/'trajectory.tum').stat().st_size==0:raise RuntimeError('EMPTY_TRAJECTORY')
        status.update(status='SUCCESS',exit_code=0)
    except Exception as exc:status.update(status='FAILURE',reason=type(exc).__name__+': '+str(exc),exit_code=1)
    finally:
        for p in reversed(processes):stop(p)
        for f in logs:f.close()
        status['wall_time_s']=time.monotonic()-start
        (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    return status['exit_code']

if __name__=='__main__':raise SystemExit(main())
