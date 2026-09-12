// Diagnostic only: existing linked GTSAM/backend preintegration, GT starts, no graph solve.
#include "uifgo/imu_preint.h"
#include <fstream>
#include <iostream>
#include <iomanip>
int main(int argc,char**argv){
 if(argc!=3)return 2;std::ifstream f(argv[1]);std::ofstream o(argv[2]);o<<std::setprecision(17);int count;f>>count;
 uifgo::Config cfg;gtsam::imuBias::ConstantBias bias;
 for(int k=0;k<count;++k){int n;double t0,t1;f>>n>>t0>>t1;gtsam::Vector3 p,v,pt,vt;gtsam::Matrix3 R,Rt;
 for(int j=0;j<3;++j)f>>p(j);for(int j=0;j<3;++j)f>>v(j);for(int i=0;i<3;++i)for(int j=0;j<3;++j)f>>R(i,j);
 for(int j=0;j<3;++j)f>>pt(j);for(int j=0;j<3;++j)f>>vt(j);for(int i=0;i<3;++i)for(int j=0;j<3;++j)f>>Rt(i,j);
 std::vector<uifgo::ImuSample> samples(n);for(auto &s:samples){f>>s.t;for(int j=0;j<3;++j)f>>s.acc(j);for(int j=0;j<3;++j)f>>s.gyro(j);}
 if(!f)return 3;uifgo::ImuPreintegrator pim(cfg,gtsam::Vector3(0,0,-9.81));pim.Reset(bias);uifgo::IntegrateBetween(samples,0,t0,t1,&pim);
 auto pred=pim.Pim().predict(gtsam::NavState(gtsam::Rot3(R),p,v),bias);
 o<<k<<","<<gtsam::Rot3::Logmap(pred.attitude().between(gtsam::Rot3(Rt))).norm()*180/M_PI<<","<<(pred.position()-pt).norm()<<","<<(pred.velocity()-vt).norm()<<"\n";
 }return 0;
}
