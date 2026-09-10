#!/usr/bin/env python3
import sys,subprocess
from pathlib import Path
E=Path(sys.argv[1]);kind=sys.argv[2];record=sys.argv[3]
base=['g++','-std=gnu++17','-O3','-g','-march=native','-pthread','-DBOOST_ALL_NO_LIB','-Iinclude','-I/usr/local/include','-I/usr/include/eigen3']
cmd=base+['-c','tools/paper/a18_'+kind+'.cpp','-o',str(E/('a18_'+kind+'.o'))]
subprocess.run([sys.executable,'tools/paper/t07_run_logged_command.py','--record-dir',str(E/(record+'_compile')),'--cwd','.','--expected-exit','0','--']+cmd,check=True)
libs=['-Wl,-rpath,/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib:/opt/ros/noetic/lib:/usr/local/lib','-Wl,--no-as-needed','/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so','/usr/local/lib/libgtsam.so.4.2.0','/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2','-lboost_serialization','-lboost_system','-lboost_filesystem','-lboost_timer','-lboost_chrono','-lyaml-cpp','-ltbb','-ltbbmalloc','/usr/local/lib/libmetis-gtsam.so','-lm']
cmd=['g++','-pthread','-rdynamic',str(E/('a18_'+kind+'.o'))]+libs+['-o',str(E/('a18_'+kind))]
subprocess.run([sys.executable,'tools/paper/t07_run_logged_command.py','--record-dir',str(E/(record+'_link')),'--cwd','.','--expected-exit','0','--']+cmd,check=True)
