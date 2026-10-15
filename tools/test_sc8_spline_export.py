#!/usr/bin/env python3
"""Deterministic spline codec/export boundary check. Constructs no estimator."""
import json,shlex,subprocess
from pathlib import Path
import run_sfuise_benchmark as sf
import benchmark_common as common

CPP=r'''
#include <SplineState.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
SplineState decode(const sfuise_msgs::Spline& m){
 SplineState s;s.init(m.dt,0,m.start_t,static_cast<int>(m.start_idx));
 for(const auto& k:m.knots){Eigen::Matrix<double,6,1>b;b<<k.bias_acc.x,k.bias_acc.y,k.bias_acc.z,k.bias_gyro.x,k.bias_gyro.y,k.bias_gyro.z;
  s.addOneStateKnot(Eigen::Quaterniond(k.orientation.w,k.orientation.x,k.orientation.y,k.orientation.z),Eigen::Vector3d(k.position.x,k.position.y,k.position.z),b);}
 for(int i=0;i<3;++i){const auto& k=m.idles[i];Eigen::Matrix<double,6,1>b;b<<k.bias_acc.x,k.bias_acc.y,k.bias_acc.z,k.bias_gyro.x,k.bias_gyro.y,k.bias_gyro.z;
  s.setIdles(i,Eigen::Vector3d(k.position.x,k.position.y,k.position.z),Eigen::Quaterniond(k.orientation.w,k.orientation.x,k.orientation.y,k.orientation.z),b);}
 return s;
}
int main(){try{
 constexpr int64_t dt=100000000,origin=1000000000000;
 SplineState root,history,correct;root.init(dt,0,origin);history.init(dt,0,origin);correct.init(dt,0,origin);
 double online=0,late=0;size_t windows=0;
 for(int i=0;i<80;++i){
  Eigen::Matrix<double,6,1>b=Eigen::Matrix<double,6,1>::Constant(i*.001);
  root.addOneStateKnot(Eigen::Quaterniond(Eigen::AngleAxisd(.03*i,Eigen::Vector3d(1,2,3).normalized())),Eigen::Vector3d(i*.3,i*i*.02,-i*i*.01),b);
  if(root.numKnots()<2)continue;
  sfuise_msgs::Spline msg;root.getSplineMsg(msg);auto copied=decode(msg);const auto t=root.maxTimeNs();
  Eigen::Quaterniond a,q;root.itpQuaternion(t,&a);copied.itpQuaternion(t,&q);
  online=std::max(online,(root.itpPosition(t)-copied.itpPosition(t)).norm());
  online=std::max(online,(root.itpBias(t)-copied.itpBias(t)).norm());
  online=std::max(online,(root.itpPosition<1>(t)-copied.itpPosition<1>(t)).norm());
  online=std::max(online,(a.toRotationMatrix()-q.toRotationMatrix()).norm());
  history.updateKnots(&copied);correct.updateKnots(&copied);
  // Actual initial idles in this fixture are the init() zero/identity states.
  for(int j=0;j<3;++j)correct.setIdles(j,Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),Eigen::Matrix<double,6,1>::Zero());
  ++windows;if(root.numKnots()>=12)root.removeOneOldState();
 }
 const auto first=origin+dt-1;
 const double boundary=(history.itpPosition(first)-correct.itpPosition(first)).norm();
 for(int64_t t=origin+2*dt;t<=history.maxTimeNs();t+=dt/5){late=std::max(late,(history.itpPosition(t)-correct.itpPosition(t)).norm());}
 if(online>1e-12 || boundary<1 || late>1e-12)throw std::runtime_error("SPLINE_CODEC_OR_BOUNDARY_ASSERTION");
 std::cout<<std::setprecision(17)<<"{\"status\":\"PASS\",\"windows\":"<<windows<<",\"online_codec_max_error\":"<<online
  <<",\"corrupted_initial_history_position_difference_m\":"<<boundary<<",\"history_error_after_origin_plus_2dt\":"<<late
  <<",\"safe_history_offset_ns\":"<<2*dt<<",\"estimator_constructed\":false,\"FGO_solves\":0}"<<std::endl;
 return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
'''

def check():
    root=common.ROOT;cache=root/'.cache/sc8';dest=cache/'spline-export-check';dest.mkdir(parents=True,exist_ok=True)
    workspace=sf.workspace(cache);build=workspace/'build/uwb_benchmark_sfuise_adapter'
    flags=build/'CMakeFiles/sfuise_trajectory_adapter.dir/flags.make';link=build/'CMakeFiles/sfuise_trajectory_adapter.dir/link.txt'
    values={line.split(' = ',1)[0]:line.split(' = ',1)[1] for line in flags.read_text().splitlines() if ' = ' in line}
    source=dest/'check.cpp';source.write_text(CPP);binary=dest/'check';tokens=shlex.split(link.read_text())
    args=[tokens[0],*shlex.split(values['CXX_FLAGS']),*shlex.split(values['CXX_DEFINES']),*shlex.split(values['CXX_INCLUDES'])]
    i=1
    while i<len(tokens):
        token=tokens[i]
        if token=='-o':args.extend(['-o',str(binary)]);i+=2;continue
        if token.endswith('.o'):args.append(str(source))
        else:args.append(token)
        i+=1
    with (dest/'build.log').open('w') as log:subprocess.run(args,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
    output=subprocess.check_output([str(binary)],text=True);result=json.loads(output)
    result.update(spline_header_sha256=common.sha256(sf.SFUISE_ROOT/'sfuise/include/SplineState.h'),test_source_sha256=common.sha256(source),
        interpretation='Online codec is exact. Aggregating moving windows with updateKnots overwrites global initial idles; only t < global_origin+2*dt depends on them. This is an exporter boundary, not estimator divergence.')
    (dest/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)

if __name__=='__main__':check()
