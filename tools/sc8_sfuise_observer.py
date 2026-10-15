#!/usr/bin/env python3
"""Generate a sealed read-only SFUISE observer, never modify its pinned checkout.

Every edit is an insertion. Reversing the insertions must reproduce the exact
pinned source. No residual, optimization, rejection or update statement changes.
The numeric cutoff tracks all inputs actually consumed by startup gravity,
propagation and residual windows, including inherited past dependence. A separate
callback cutoff records buffered input used only by availability decisions.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import time
import os
import sys
import run_sfuise_benchmark as sfuise

ROOT=sfuise.ROOT
CACHE=ROOT/'.cache/sc8/sfuise_observer'


def generate():
    sfuise.checked_pin()
    original=(sfuise.SFUISE_ROOT/'sfuise/src/SplineFusion.cpp').read_text()
    source=original
    edits=[('#include <ros/ros.h>',
  '\n'
  '#include <fstream>\n'
  '#include <iomanip>\n'
  '#include <chrono>\n'
  '#include <ctime>\n'
  '#include <cstdlib>\n'
  '#include <sys/resource.h>\n'
  '\n'
  '#include <std_msgs/Float64MultiArray.h>\n'),
 ('        window_count = 0;',
  '\n'
  '        if(const char* path=std::getenv("SC8_SFUISE_OBSERVER")){\n'
  '          sc8_out.open(path);sc8_out<<std::setprecision(17);\n'
  '          '
  'sc8_out<<"estimate_index,state_ns,max_sensor_stamp_used,published_wall_time,core_ms,cpu_ms,imu_received,toa_received,revision_index,observed_sensor_cutoff_ns,process_cpu_ms,peak_rss_kib\\n";\n'
  '        }'),
 ('        TicToc t_window;',
  '\n'
  '        const auto sc8_start=std::chrono::steady_clock::now();\n'
  '        const auto sc8_cpu=std::clock();\n'
  '        bool sc8_emitted=false;double sc8_published_wall=0;'),
 ('            pub_est.publish(est_msg);',
  '\n            sc8_emitted=true;sc8_published_wall=ros::WallTime::now().toSec();'),
 ('    ros::Subscriber sub_imu;',
  '\n'
  '    std::ofstream sc8_out;\n'
  '    int64_t sc8_max_sensor_ns=0,sc8_numeric_max_ns=0;\n'
  '    size_t sc8_imu_received=0,sc8_toa_received=0;\n'
  '    double '
  'sc8_pending_callbacks_ms=0,sc8_pending_callbacks_cpu_ms=0,sc8_pending_run_ms=0,sc8_pending_run_cpu_ms=0;\n'
  '    ros::Subscriber sc8_toa64_sub;\n'
  '    // Lossless wire adapter only: exactly the upstream TOAData ingestion order,\n'
  '    // source offset, validity rule and timestamp. No estimator arithmetic changes.\n'
  '    void getSc8Toa64Callback(const std_msgs::Float64MultiArray::ConstPtr& msg){\n'
  '      const auto started=std::chrono::steady_clock::now();const auto cpu=std::clock();\n'
  '      if(msg->data.size()<2 || !std::isfinite(msg->data[0]) || msg->data[1]<0 || '
  'msg->data[1]>4096)\n'
  '        throw std::runtime_error("SC8_TOA64_MALFORMED");\n'
  '      const auto n=static_cast<size_t>(msg->data[1]);\n'
  '      if(msg->data.size()!=2+3*n)throw std::runtime_error("SC8_TOA64_SIZE");\n'
  '      const auto stamp=static_cast<int64_t>(msg->data[0]);\n'
  '      sc8_max_sensor_ns=std::max(sc8_max_sensor_ns,stamp);++sc8_toa_received;\n'
  '      for(size_t i=0;i<n;++i){\n'
  '        if(msg->data[4+3*i]==0)continue;\n'
  '        const auto aid=static_cast<uint16_t>(msg->data[2+3*i]);\n'
  '        TOAData measurement(stamp,aid,msg->data[3+3*i]);\n'
  '        toa_buff.push_back(measurement);\n'
  '      }\n'
  '      '
  'sc8_pending_callbacks_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();\n'
  '      sc8_pending_callbacks_cpu_ms+=1000.*(std::clock()-cpu)/CLOCKS_PER_SEC;\n'
  '    }\n'
  '    size_t sc8_estimates=0;'),
 ('        int64_t t_ns = imu_msg->header.stamp.toNSec();',
  '\n'
  '        const auto sc8_cb_start=std::chrono::steady_clock::now();const auto '
  'sc8_cb_cpu=std::clock();\n'
  '        sc8_max_sensor_ns=std::max(sc8_max_sensor_ns,t_ns);++sc8_imu_received;'),
 ('        int64_t t_ns = uwb_msg->header.stamp.toNSec();',
  '\n'
  '        const auto sc8_cb_start=std::chrono::steady_clock::now();const auto '
  'sc8_cb_cpu=std::clock();\n'
  '        sc8_max_sensor_ns=std::max(sc8_max_sensor_ns,t_ns);++sc8_toa_received;'),
 ('                        gravity_sum += imu_buff.at(i).accel;',
  '\n'
  '                        '
  'sc8_numeric_max_ns=std::max(sc8_numeric_max_ns,imu_buff.at(i).time_ns);'),
 ('                integrateStep(last_imu_t_ns, dt, imu_vec[i], Rs0, Ps0, Vs0);',
  '\n                sc8_numeric_max_ns=std::max(sc8_numeric_max_ns,imu_vec[i].time_ns);'),
 ('        bool converged = false;',
  '\n'
  '        '
  'if(!imu_window.empty())sc8_numeric_max_ns=std::max(sc8_numeric_max_ns,imu_window.back().time_ns);\n'
  '        '
  'if(!toa_window.empty())sc8_numeric_max_ns=std::max(sc8_numeric_max_ns,toa_window.back().time_ns);'),
 ('        imu_buff.push_back(imu);',
  '\n'
  '        '
  'sc8_pending_callbacks_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-sc8_cb_start).count();\n'
  '        sc8_pending_callbacks_cpu_ms+=1000.*(std::clock()-sc8_cb_cpu)/CLOCKS_PER_SEC;'),
 ('            toa_buff.push_back(uwb);\n        }',
  '\n'
  '        '
  'sc8_pending_callbacks_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-sc8_cb_start).count();\n'
  '        sc8_pending_callbacks_cpu_ms+=1000.*(std::clock()-sc8_cb_cpu)/CLOCKS_PER_SEC;'),
 ('            if (solver_flag == FULLSIZE) spline_local.removeOneOldState();\n        }',
  '\n'
  '        const double '
  'sc8_run_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-sc8_start).count();\n'
  '        const auto sc8_cpu_done=std::clock();\n'
  '        if(sc8_emitted){\n'
  '          ++sc8_estimates;\n'
  '          if(sc8_out){\n'
  '            struct rusage sc8_usage{};getrusage(RUSAGE_SELF,&sc8_usage);\n'
  "            sc8_out<<num_window<<','<<spline_local.maxTimeNs()<<','<<sc8_numeric_max_ns<<','\n"
  '              '
  "<<sc8_published_wall<<','<<(sc8_run_ms+sc8_pending_run_ms+sc8_pending_callbacks_ms)<<','\n"
  '              '
  "<<(1000.*(sc8_cpu_done-sc8_cpu)/CLOCKS_PER_SEC+sc8_pending_run_cpu_ms+sc8_pending_callbacks_cpu_ms)<<','\n"
  '              <<sc8_imu_received<<\',\'<<sc8_toa_received<<",0,"<<sc8_max_sensor_ns<<\',\'\n'
  "              <<1000.*sc8_cpu_done/CLOCKS_PER_SEC<<','<<sc8_usage.ru_maxrss<<'\\n';\n"
  '          }\n'
  '          '
  'sc8_pending_run_ms=sc8_pending_run_cpu_ms=sc8_pending_callbacks_ms=sc8_pending_callbacks_cpu_ms=0;\n'
  '        }else{\n'
  '          sc8_pending_run_ms+=sc8_run_ms;\n'
  '          sc8_pending_run_cpu_ms+=1000.*(sc8_cpu_done-sc8_cpu)/CLOCKS_PER_SEC;\n'
  '        }'),
 ('        pub_start_time = nh.advertise<std_msgs::Int64>("start_time", 1000);',
  '\n'
  '        if(!if_tdoa && std::getenv("SC8_SFUISE_TOA64")){\n'
  '          sub_uwb.shutdown();\n'
  '          '
  'sc8_toa64_sub=nh.subscribe("/EstimationInterface/toa_ds64",1000,&SplineFusion::getSc8Toa64Callback,this);\n'
  '        }'),
 ('    EIGEN_MAKE_ALIGNED_OPERATOR_NEW',
  '\n'
  '    ~SplineFusion(){\n'
  '      if(const char* path=std::getenv("SC8_SFUISE_OBSERVER")){\n'
  '        struct rusage usage{};getrusage(RUSAGE_SELF,&usage);\n'
  '        std::ofstream out(std::string(path)+".final.json");\n'
  '        '
  'out<<std::setprecision(17)<<"{\\"imu_received\\":"<<sc8_imu_received<<",\\"toa_received\\":"<<sc8_toa_received\n'
  '          '
  '<<",\\"published_updates\\":"<<sc8_estimates<<",\\"max_numeric_input_ns\\":"<<sc8_numeric_max_ns\n'
  '          '
  '<<",\\"max_callback_input_ns\\":"<<sc8_max_sensor_ns<<",\\"process_cpu_ms\\":"<<1000.*std::clock()/CLOCKS_PER_SEC\n'
  '          <<",\\"peak_rss_kib\\":"<<usage.ru_maxrss<<"}\\n";\n'
  '      }\n'
  '    }')]
    for needle,addition in edits:
        assert source.count(needle)==1,needle
        source=source.replace(needle,needle+addition)
    restored=source
    for needle,addition in reversed(edits):restored=restored.replace(needle+addition,needle)
    assert restored==original,'observer changed numerical source'
    CACHE.mkdir(parents=True,exist_ok=True)
    (CACHE/'src').mkdir(exist_ok=True)
    inc=CACHE/'include'
    if not inc.exists():inc.symlink_to(sfuise.SFUISE_ROOT/'sfuise/include',target_is_directory=True)
    path=CACHE/'src/SplineFusion.cpp'
    if not path.exists() or path.read_text()!=source:path.write_text(source)
    template=(ROOT/'benchmark/sfuise_adapter_ros/src/sc8_observer_checks.cpp.in').read_text()
    template=template.replace('@UPSTREAM_INCLUDES@',original[:original.index('class SplineFusion')])
    template=template.replace('@ORIGINAL@',str(sfuise.SFUISE_ROOT/'sfuise/src/SplineFusion.cpp'))
    template=template.replace('@OBSERVED@',str(path))
    (CACHE/'src/checks.cpp').write_text(template)

    identity={'pinned_sha':sfuise.PIN,'original_sha256':hashlib.sha256(original.encode()).hexdigest(),
              'observed_sha256':hashlib.sha256(source.encode()).hexdigest(),
              'reversed_insertions_exact_original':True,'insertion_count':len(edits),
              'cutoff':'MAX_NUMERIC_INPUT_INCLUDING_INITIAL_GRAVITY_INTEGRATION_AND_INHERITED_PAST; CALLBACK_BOUND_SEPARATE',
              'core_boundary':'all callback/run active work since preceding output, including maintenance after publication; excludes idle sleeps',
              'wire_adapter':'optional Float64 ToA -> unchanged upstream TOAData; original Float32 subscription shut down in this mode',
              'upstream_worktree_clean':True}
    (CACHE/'patch_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    return path


def build():
    source=generate();ws=sfuise.workspace(ROOT/'.cache/sc8')
    # Existing workspace links remain untouched; only an additive observer target.
    log=ROOT/'.cache/sc8/build-sfuise-observer.log'
    with log.open('w') as f:
        code=subprocess.run(['catkin_make','-C',str(ws),'-DCMAKE_BUILD_TYPE=Release',
            '-DCMAKE_CXX_FLAGS_DEBUG=-O3 -DNDEBUG',f'-DSFUISE_INCLUDE_DIR={sfuise.SFUISE_ROOT}/sfuise/include',
            f'-DSFUISE_SC8_OBSERVER_SOURCE={source}','-j2'],cwd=ws,stdout=f,stderr=subprocess.STDOUT).returncode
    if code:raise RuntimeError(str(log))
    return ws/'devel/setup.bash'

def check():
    """Same callbacks/order through upstream and observed classes; no GT."""
    candidates=list((ROOT/'.cache/sc8/runs').glob('E00-R1-SFUISE-*/execution.json'))
    candidates=[p for p in candidates if json.loads(p.read_text())['status']=='SUCCESS']
    if not candidates:raise RuntimeError('run E00 R1 SFUISE prefix first')
    run=max(candidates,key=lambda p:int(json.loads(p.read_text()).get('attempt',1))).parent
    if not (run/'input.bag').exists():raise RuntimeError('run E00 R1 SFUISE prefix first')
    setup=sfuise.workspace(ROOT/'.cache/sc8')/'devel/setup.bash'
    env=sfuise.source_environment(setup)
    env.update(ROS_MASTER_URI='http://127.0.0.1:11343',ROS_HOSTNAME='127.0.0.1',
               OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',
               SC8_SFUISE_OBSERVER=str(CACHE/'observer-check-readonly.csv'))
    with (CACHE/'check-master.log').open('w') as log:
        master=subprocess.Popen(['roscore','-p','11343'],env=env,stdout=log,stderr=subprocess.STDOUT)
        try:
            time.sleep(3)
            subprocess.run(['rosparam','load',str(run/'sfuise.yaml'),'/SplineFusion'],env=env,check=True)
            binary=sfuise.workspace(ROOT/'.cache/sc8')/'devel/lib/uwb_benchmark_sfuise_adapter/sfuise_sc8_observer_checks'
            start=time.monotonic()
            with (CACHE/'check.log').open('w') as output:
                result=subprocess.run([str(binary),str(run/'input.bag')],env=env,stdout=output,stderr=subprocess.STDOUT,timeout=180)
            (CACHE/'check.json').write_text(json.dumps(dict(returncode=result.returncode,elapsed_s=time.monotonic()-start,
                input_bag_sha256=sfuise.common.sha256(run/'input.bag'),numerical_equality=(CACHE/'check.log').read_text().strip())))
            if result.returncode:raise RuntimeError((CACHE/'check.log').read_text())
            print((CACHE/'check.log').read_text()[-1000:])
        finally:sfuise.terminate([master])

if __name__=='__main__':
    if sys.argv[1:]==['check']:check()
    else:print(build())
