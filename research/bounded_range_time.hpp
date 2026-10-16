#pragma once
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "physical_time.hpp"
#include <gtsam/linear/NoiseModel.h>
#include <map>

namespace uwb_imu_pl::research {
using RangeTimeRadii=std::map<MeasurementId,double>;
// Original covariance is unchanged. Deterministic asynchronous transport is
// an interval nuisance, not another independent Gaussian measurement. The
// whitened box |W*d| <= |W|*radius is an outer relaxation of that nuisance.
// Functional fitting profiles this box; it earns no Gaussian alpha guarantee.
struct RangeTimeWhitening {
 Eigen::MatrixXd w;
 Eigen::VectorXd radius;
 bool bounded=false;
 RangeTimeWhitening(const UwbBatch&,const RangeTimeRadii*);
 Eigen::VectorXd profile(Eigen::VectorXd residual,Eigen::VectorXd* active=nullptr)const{
  if(active)*active=Eigen::VectorXd::Ones(residual.size());
  if(!bounded)return residual;
  for(int i=0;i<residual.size();++i){
   if(radius[i]==0)continue;
   if(std::abs(residual[i])<=radius[i]){residual[i]=0;if(active)(*active)[i]=0;}
   else residual[i]-=std::copysign(radius[i],residual[i]);
  }
  return residual;
 }
};
class BoundedTimeRangeFactor final:public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 UwbPoseBatchFactor original_;
 RangeTimeWhitening model_;
 public:
 BoundedTimeRangeFactor(gtsam::Key key,const UwbBatch& batch,const Eigen::Vector3d& lever,const RangeTimeRadii& transport)
  :gtsam::NoiseModelFactor1<gtsam::Pose3>(gtsam::noiseModel::Isotropic::Sigma(batch.measurements.size(),1.),key),
   original_(key,batch,lever),model_(batch,&transport){}
 gtsam::Vector evaluateError(const gtsam::Pose3& pose,boost::optional<gtsam::Matrix&> jacobian=boost::none)const override{
  gtsam::Matrix raw_j;const auto raw=original_.evaluateError(pose,jacobian?boost::optional<gtsam::Matrix&>(raw_j):boost::none);
  Eigen::VectorXd active;auto result=model_.profile(model_.w*raw,jacobian?&active:nullptr);
  if(jacobian)*jacobian=active.asDiagonal()*model_.w*raw_j;
  return result;
 }
 const UwbBatch& batch()const{return original_.batch();}
 const Eigen::VectorXd& whitenedRadius()const{return model_.radius;}
};
}
