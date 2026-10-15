#!/usr/bin/env python3
"""Generate a sealed read-only SFUISE observer, never modify its pinned checkout.

Every edit is an insertion. Reversing the insertions must reproduce the exact
pinned source. No residual, optimization, rejection or update statement changes.
The cutoff is the conservative maximum input seen in callbacks, including input
used for startup/gravity and availability decisions, not just residual windows.
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
    edits=[
      ('#include <ros/ros.h>','\n#include <fstream>\n#include <iomanip>\n#include <chrono>\n#include <ctime>\n#include <cstdlib>\n'),
      ('        window_count = 0;','\n        if(const char* path=std::getenv("SC8_SFUISE_OBSERVER")){\n          sc8_out.open(path);sc8_out<<std::setprecision(17);\n          sc8_out<<"estimate_index,state_ns,max_sensor_stamp_used,published_wall_time,source_clock_at_publish_ns,core_ms,cpu_ms,imu_received,toa_received,revision_index\\n";\n        }'),
      ('        TicToc t_window;','\n        const auto sc8_start=std::chrono::steady_clock::now();\n        const auto sc8_cpu=std::clock();'),
      ('            pub_est.publish(est_msg);','\n            if(sc8_out){\n              sc8_out<<num_window<<\',\'<<spline_local.maxTimeNs()<<\',\'<<sc8_max_sensor_ns<<\',\'\n                <<ros::WallTime::now().toSec()<<\',\'<<ros::Time::now().toNSec()<<\',\'\n                <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-sc8_start).count()<<\',\'\n                <<1000.*(std::clock()-sc8_cpu)/CLOCKS_PER_SEC<<\',\'<<sc8_imu_received<<\',\'<<sc8_toa_received<<",0\\n";\n            }'),
      ('    ros::Subscriber sub_imu;','\n    std::ofstream sc8_out;\n    int64_t sc8_max_sensor_ns=0;\n    size_t sc8_imu_received=0,sc8_toa_received=0;'),
      ('        int64_t t_ns = imu_msg->header.stamp.toNSec();','\n        sc8_max_sensor_ns=std::max(sc8_max_sensor_ns,t_ns);++sc8_imu_received;'),
      ('        int64_t t_ns = uwb_msg->header.stamp.toNSec();','\n        sc8_max_sensor_ns=std::max(sc8_max_sensor_ns,t_ns);++sc8_toa_received;'),
    ]
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
              'cutoff':'CONSERVATIVE_MAX_CALLBACK_INPUT_INCLUDING_INITIALIZATION',
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
    run=ROOT/'.cache/sc8/runs/E00-R1-SFUISE-20261020-30x30'
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
