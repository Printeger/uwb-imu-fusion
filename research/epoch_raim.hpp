#pragma once
// Isolated, fixed-linear-Gaussian research model; never a production PL.
#include "gaussian_reference.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include <Eigen/QR>
#include <algorithm>
namespace uwb_imu_pl::research {
struct EpochRaim {
  GaussianReference gaussian;
  Eigen::Vector3d tag=Eigen::Vector3d::Constant(NAN);
  Eigen::MatrixXd response, slope;
  std::vector<int> retained;
  int excluded=-1, candidates=0;
  double detection_statistic=INFINITY, detection_threshold=0;
  int detection_dof=0;
  bool alarm=false;
  std::string status="MODEL_UNQUALIFIED";
};
inline UwbBatch retainRanges(const UwbBatch& b,int excluded) {
  if(excluded < -1 || (excluded>=0 && size_t(excluded)>=b.measurements.size()))
    throw std::invalid_argument("EXCLUDED_RANGE_INDEX_INVALID");
  UwbBatch r=b;r.measurements.clear();std::vector<int> ids;
  for(int i=0;i<int(b.measurements.size());++i)if(excluded<0 || b.measurements[i].anchor_id!=b.measurements[excluded].anchor_id){r.measurements.push_back(b.measurements[i]);ids.push_back(i);}
  r.covariance_m2=principal(validatedUwbCovariance(b),ids);return r;
}
inline bool spatialAnchors(const UwbBatch& b) {
  if(b.measurements.size()<5)return false;
  Eigen::MatrixXd a(b.measurements.size()-1,3);
  for(int i=1;i<int(b.measurements.size());++i)a.row(i-1)=(b.measurements[i].anchor_position_m-b.measurements[0].anchor_position_m).transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> s(a);
  return s.singularValues().minCoeff()>1e-6*s.singularValues().maxCoeff();
}
inline void responses(EpochRaim& r,const Eigen::MatrixXd& c,int offset,int n) {
  r.response=(c*r.gaussian.gain*r.gaussian.whitener).rightCols(n);
  r.slope.resize(3,n);
  for(int i=0;i<n;++i){double d=(r.gaussian.parity*r.gaussian.whitener.col(offset+i)).norm();r.slope.col(i)=r.response.col(i).cwiseAbs()/d;}
}
inline EpochRaim uwbCandidate(const UwbBatch& batch,int excluded,double alpha,double tail) {
  EpochRaim r;r.excluded=excluded;auto b=retainRanges(batch,excluded);
  if(!spatialAnchors(b)){r.status="GEOMETRY_OR_MIRROR_UNQUALIFIED";return r;}
  // Squared-range differences provide a deterministic, UWB-only branch seed.
  int n=b.measurements.size();Eigen::MatrixXd d(n-1,3);Eigen::VectorXd q(n-1);
  const auto& m0=b.measurements[0];
  for(int i=1;i<n;++i){const auto& m=b.measurements[i];d.row(i-1)=2*(m.anchor_position_m-m0.anchor_position_m).transpose();q(i-1)=m.anchor_position_m.squaredNorm()-m0.anchor_position_m.squaredNorm()-m.range_m*m.range_m+m0.range_m*m0.range_m;}
  Eigen::Vector3d p=d.colPivHouseholderQr().solve(q);
  Eigen::LLT<Eigen::MatrixXd> chol(validatedUwbCovariance(b));
  Eigen::MatrixXd w=chol.matrixL().solve(Eigen::MatrixXd::Identity(n,n)),h(n,3);Eigen::VectorXd z(n);
  bool converged=false;
  for(int it=0;it<40;++it){
    for(int i=0;i<n;++i){Eigen::Vector3d v=p-b.measurements[i].anchor_position_m;double norm=v.norm();if(norm<1e-6){r.status="NONLINEAR_DOMAIN_FAILED";return r;}h.row(i)=v.transpose()/norm;z(i)=b.measurements[i].range_m-norm;}
    Eigen::Vector3d step=(w*h).colPivHouseholderQr().solve(w*z);
    if(!step.allFinite() || step.norm()>10){r.status="NONLINEAR_DOMAIN_FAILED";return r;}
    if(step.norm()<1e-9){converged=true;break;}
    // Monotone line search on the exact correlated WLS objective.
    double scale=1;bool moved=false;
    for(int k=0;k<20;++k){Eigen::Vector3d trial=p+scale*step;Eigen::VectorXd v(n);for(int i=0;i<n;++i)v(i)=b.measurements[i].range_m-(trial-b.measurements[i].anchor_position_m).norm();if((w*v).squaredNorm()<(w*z).squaredNorm()){p=trial;moved=true;break;}scale*=.5;}
    if(!moved){converged=step.norm()<1e-6;break;}
  }
  if(!converged){r.status="NONLINEAR_CONVERGENCE_FAILED";return r;}
  std::vector<Eigen::MatrixXd> faults;
  for(int i=0;i<n;++i)faults.push_back(Eigen::MatrixXd::Identity(n,n).col(i));
  r.gaussian=gaussianReference(h,z,validatedUwbCovariance(b),Eigen::Matrix3d::Identity(),faults,alpha,tail);
  r.tag=p+r.gaussian.mean;r.status=r.gaussian.status;
  if(r.gaussian.gain.size())responses(r,Eigen::Matrix3d::Identity(),0,n);
  for(int i=0;i<int(batch.measurements.size());++i)if(excluded<0 || batch.measurements[i].anchor_id!=batch.measurements[excluded].anchor_id)r.retained.push_back(i);
  return r;
}
// Union over ALL n+1 candidates pays 2*(n+1)*tail, including wrongly deleted
// healthy rows. Every retained single-fault direction remains in its PL.
template<class Candidate> inline EpochRaim epochFde(int n,bool enabled,Candidate evaluate,bool external_alarm=false) {
  auto all=evaluate(-1);all.candidates=1;
  all.detection_statistic=all.gaussian.statistic;all.detection_threshold=all.gaussian.threshold;all.detection_dof=all.gaussian.dof;
  all.alarm=all.gaussian.dof>0 && all.gaussian.statistic>all.gaussian.threshold;
  if((!all.alarm && !external_alarm) || !enabled)return all;
  EpochRaim best;double score=INFINITY;
  if(external_alarm && all.status=="FROZEN_MODEL_ONLY" && all.gaussian.statistic<=all.gaussian.threshold){best=all;score=all.gaussian.statistic;}
  for(int i=0;i<n;++i){auto r=evaluate(i);if(r.status=="FROZEN_MODEL_ONLY" && r.gaussian.statistic<score){score=r.gaussian.statistic;best=std::move(r);}}
  best.candidates=n+1;best.alarm=true;
  best.detection_statistic=all.detection_statistic;best.detection_threshold=all.detection_threshold;best.detection_dof=all.detection_dof;
  if(!std::isfinite(score))best.status="FDE_NO_QUALIFIED_CANDIDATE";
  return best;
}
inline EpochRaim uwbEpochRaim(const UwbBatch& b,bool fde=true,double alpha=1e-6,double tail=1e-9) {
 return epochFde(b.measurements.size(),fde,[&](int i){return uwbCandidate(b,i,alpha,tail);});
}
} // namespace
