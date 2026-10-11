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
  Eigen::MatrixXd independentGain=info.ldlt().solve(h.transpose()*r.inverse());
  require((a.response-independentGain).norm()<1e-8,"independent frozen WLS fault response");
  Eigen::LLT<Eigen::MatrixXd> rl(r);Eigen::MatrixXd w=rl.matrixL().solve(Eigen::MatrixXd::Identity(n,n));
  Eigen::HouseholderQR<Eigen::MatrixXd> parityQr(w*h);
  Eigen::MatrixXd qq=parityQr.householderQ()*Eigen::MatrixXd::Identity(n,n);
  Eigen::MatrixXd parity=qq.rightCols(n-3).transpose();
  require(std::abs((parity*w*residual).squaredNorm()-a.gaussian.statistic)<1e-8,"independent QR parity statistic");
  for(int i=0;i<n;++i)require((independentGain.col(i).cwiseAbs()/(parity*w.col(i)).norm()-a.slope.col(i)).norm()<1e-8,"independent QR slope");
  double eps=1e-5;auto plus=b,minus=b;plus.measurements[0].range_m+=eps;minus.measurements[0].range_m-=eps;
  auto ap=uwbEpochRaim(plus,false),am=uwbEpochRaim(minus,false);
  // Nonlinear re-fit includes Hessian curvature; small nominal residual permits
  // approximate comparison, unlike exact frozen linear perturbation below.
  require(((ap.tag-am.tag)/(2*eps)-a.response.col(0)).norm()<.08,"A nonlinear response");
  b.measurements[0].range_m+=2.25;auto recovery=uwbEpochRaim(b);require(recovery.alarm && recovery.excluded==0 && recovery.status=="FROZEN_MODEL_ONLY","A actual FDE");require(recovery.candidates==n+1,"complete enumeration");
  for(auto& m:b.measurements)m.anchor_position_m.z()=1.1;
  require(uwbEpochRaim(b).status=="GEOMETRY_OR_MIRROR_UNQUALIFIED","mirror refusal");
 }
 // Single range is an exact state alias: no residual response, dangerous.
 Eigen::MatrixXd h=Eigen::MatrixXd::Zero(5,3);h.topRows(3).setIdentity();
 Eigen::MatrixXd f=Eigen::MatrixXd::Zero(5,1);f(0)=1.;
 auto dangerous=gaussianReference(h,Eigen::VectorXd::Zero(5),Eigen::MatrixXd::Identity(5,5),Eigen::Matrix3d::Identity(),{f},1e-6,1e-9);
 require(dangerous.status=="DANGEROUS_NULLSPACE" && !dangerous.pl.allFinite(),"dangerous single-range nullspace");
 Eigen::Matrix3d safe=Eigen::Matrix3d::Identity();safe.col(0).setZero();
 require(gaussianReference(h,Eigen::VectorXd::Zero(5),Eigen::MatrixXd::Identity(5,5),safe,{f},1e-6,1e-9).status=="FROZEN_MODEL_ONLY","structural harmless nullspace");
 auto near=scene.uwb[13];for(auto& m:near.measurements)m.anchor_position_m.z()=1.1+1e-10*m.anchor_position_m.z();
 require(uwbEpochRaim(near).status=="GEOMETRY_OR_MIRROR_UNQUALIFIED","near rank uncertainty refusal");
 std::cout<<"A correlated/independent WLS, dof, response, full FDE, mirror: PASS\n";
}
