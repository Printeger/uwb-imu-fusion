#include "bounded_range_time.hpp"
#include "numerical_candidate.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){try{
 const TimestampNs end(917654321000LL);const Eigen::Vector3d lever(.1,-.025,0),p(.3,-.2,1.2),v(.7,-.4,.2),a(.1,.1,-.1);
 const double rate=.3;auto rotation=[&](double t){return Eigen::AngleAxisd(.4+rate*t,Eigen::Vector3d::UnitZ()).toRotationMatrix();};
 const gtsam::Pose3 pose(gtsam::Rot3(rotation(0)),p);
 std::vector<Eigen::Vector3d> anchors{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
 RollingRawState endpoint;for(int j=0;j<3;++j)endpoint.v[j]=I(v[j]-.01,v[j]+.01);
 for(int direction:{-1,1,0}){
 UwbBatch batch;batch.id=BatchId(99);batch.timestamp=end;RangeTimeRadii radii,zeros;
 for(size_t i=0;i<anchors.size();++i){const int sign=direction?direction:(i%2?1:-1);const double h=sign*double(i+1)*.002;
  UwbMeasurement m;m.id=MeasurementId(100+i);m.factor_id=FactorId(100+i);m.anchor_id=AnchorId(999999+i*73);m.anchor_position_m=anchors[i];m.timestamp=TimestampNs(end.value()+sign*int64_t((i+1)*2000000));
  m.range_m=(p+h*v+.5*h*h*a+rotation(h)*lever-anchors[i]).norm();m.sigma_m=.001;m.lever_arm_body_m=lever;batch.measurements.push_back(m);
  radii[m.id]=rangeTransportRadius(endpoint,m.timestamp,end,lever,.4,rate).upper();zeros[m.id]=0;
 }
 for(bool correlated:{false,true}){
  if(correlated)batch.covariance_m2=1e-6*(Eigen::MatrixXd::Identity(5,5)+.2*Eigen::MatrixXd::Ones(5,5));
  BoundedTimeRangeFactor factor(1,batch,lever,radii);require(factor.evaluateError(pose).norm()<1e-10,"actual asynchronous trajectory outside bounded factor");
  const gtsam::Pose3 moved(pose.rotation(),p+Eigen::Vector3d(1,.5,.3));gtsam::Matrix analytic;
  factor.evaluateError(moved,analytic);gtsam::Matrix numeric(5,6);
  for(int j=0;j<6;++j){gtsam::Vector6 delta=gtsam::Vector6::Zero();delta[j]=1e-6;
   numeric.col(j)=(factor.evaluateError(moved.retract(delta))-factor.evaluateError(moved.retract(-delta)))/(2e-6);}
  require((analytic-numeric).cwiseAbs().maxCoeff()<2e-5,"bounded factor tangent derivative mismatch");
  BoundedTimeRangeFactor zero(1,batch,lever,zeros);UwbPoseBatchFactor original(1,batch,lever);RangeTimeWhitening original_whitening(batch,nullptr);
  require((zero.evaluateError(moved)-original_whitening.w*original.evaluateError(moved)).norm()<1e-10,"zero-radius factor changed original whitened residual");
  const auto healthy=numericalUwbCandidate(batch,-1,1e-6,&radii);
  require(healthy.functional_post_test && !healthy.gaussian_reference_qualified,"bounded healthy functional post failed/overqualified");
  auto fault=batch;fault.measurements[0].range_m+=2.25;
  const auto all=numericalUwbCandidate(fault,-1,1e-6,&radii),excluded=numericalUwbCandidate(fault,0,1e-6,&radii);
  require(!all.functional_post_test && excluded.functional_post_test,"bounded-time fault exclusion did not use identical nuisance model");
 }
 }
 std::cout<<"SC9 time factor PASS forward/reverse/mixed-time  analytic moving/rotating trajectory, correlated original covariance, tangent finite difference, zero-span equivalence, functional strong-fault post; Gaussian qualification remains false\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
