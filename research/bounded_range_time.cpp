#include "bounded_range_time.hpp"
namespace uwb_imu_pl::research {
RangeTimeWhitening::RangeTimeWhitening(const UwbBatch& batch,const RangeTimeRadii* transport){
  const size_t n=batch.measurements.size();
  Eigen::LLT<Eigen::MatrixXd> chol(validatedUwbCovariance(batch));
  if(chol.info()!=Eigen::Success)throw std::invalid_argument("TIME_MODEL_COVARIANCE_INVALID");
  w=chol.matrixL().solve(Eigen::MatrixXd::Identity(n,n));radius=Eigen::VectorXd::Zero(n);
  if(!transport)return;
  for(size_t i=0;i<n;++i){RollingInterval r(0);
   for(size_t j=0;j<n;++j){const double physical=transport->at(batch.measurements[j].id);
    if(!std::isfinite(physical) || physical<0)throw std::invalid_argument("TIME_MODEL_RADIUS_INVALID");
    r+=abs(RollingInterval(w(i,j)))*RollingInterval(physical);
   }
   radius[i]=r.upper();bounded|=radius[i]>0;
  }
 }
}
