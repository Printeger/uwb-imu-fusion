#include "epoch_raim.hpp"
#include "scenario_material.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
int main(int argc,char** argv){
 NoisyMotionScenario scene(argv[1],"U0",20261010,20);
 for(double correlation:{0.,.2}){
  auto b=scene.uwb[13];int n=b.measurements.size();b.covariance_m2=Eigen::MatrixXd::Constant(n,n,.01*correlation);b.covariance_m2.diagonal().setConstant(.01);
  auto a=uwbEpochRaim(b);require(a.status=="FROZEN_MODEL_ONLY","A healthy");require(a.gaussian.dof==n-3,"A dof");
  auto retained=retainRanges(b,-1);UwbPositionBatchFactor f(1,retained);gtsam::Matrix h;Eigen::VectorXd residual=-f.evaluateError(a.tag,h);
  auto r=validatedUwbCovariance(b);Eigen::MatrixXd info=h.transpose()*r.ldlt().solve(h);Eigen::Vector3d delta=info.ldlt().solve(h.transpose()*r.ldlt().solve(residual));
  require(delta.norm()<1e-6,"WLS stationary oracle");require((a.gaussian.covariance-info.inverse()).norm()<1e-9,"WLS covariance oracle");
  double eps=1e-5;auto plus=b,minus=b;plus.measurements[0].range_m+=eps;minus.measurements[0].range_m-=eps;
  auto ap=uwbEpochRaim(plus,false),am=uwbEpochRaim(minus,false);
  // Nonlinear re-fit includes Hessian curvature; small nominal residual permits
  // approximate comparison, unlike exact frozen linear perturbation below.
  require(((ap.tag-am.tag)/(2*eps)-a.response.col(0)).norm()<.08,"A nonlinear response");
  b.measurements[0].range_m+=2.25;auto recovery=uwbEpochRaim(b);require(recovery.alarm && recovery.excluded==0 && recovery.status=="FROZEN_MODEL_ONLY","A actual FDE");require(recovery.candidates==n+1,"complete enumeration");
  for(auto& m:b.measurements)m.anchor_position_m.z()=1.1;
  require(uwbEpochRaim(b).status=="GEOMETRY_OR_MIRROR_UNQUALIFIED","mirror refusal");
 }
 std::cout<<"A correlated/independent WLS, dof, response, full FDE, mirror: PASS\n";
}
