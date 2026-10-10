#pragma once
#include "scenario_material.hpp"
#include <Eigen/QR>
#include <Eigen/SVD>
#include <limits>

namespace uwb_imu_pl::research {
struct RangePositionSet {
  bool valid=false;
  std::vector<Eigen::Vector3d> centres;
  std::vector<double> radius;
  std::string reason;
};
// simulation-only/deterministic-range/v1: at most ONE corrupted anchor;
// all other raw ranges have <= 1e-12 m deterministic generation error.
// Enumerate the union of physical event realizations, never their linear span.
// No truth positions, estimated fault labels or velocity values are inputs.
inline RangePositionSet rangePositionSet(const UwbBatch& batch) {
  RangePositionSet proof;
  constexpr double delta=1e-12;
  for(const auto& m:batch.measurements)
    if(!m.anchor_position_m.allFinite() || !std::isfinite(m.range_m) || m.range_m<0.) {
      proof.reason="invalid raw range";return proof;
    }
  if(batch.measurements.size()<5){proof.reason="insufficient independent ranges";return proof;}
  for(int omitted=-1;omitted<int(batch.measurements.size());++omitted) {
    std::vector<const UwbMeasurement*> m;
    for(std::size_t i=0;i<batch.measurements.size();++i)if(int(i)!=omitted)m.push_back(&batch.measurements[i]);
    const auto& first=*m.front();
    Eigen::MatrixXd a(m.size()-1,3);Eigen::VectorXd y(m.size()-1),uncertainty(m.size()-1);
    for(std::size_t i=1;i<m.size();++i) {
      a.row(i-1)=2.*(m[i]->anchor_position_m-first.anchor_position_m).transpose();
      y(i-1)=m[i]->anchor_position_m.squaredNorm()-first.anchor_position_m.squaredNorm()-
          m[i]->range_m*m[i]->range_m+first.range_m*first.range_m;
      uncertainty(i-1)=2*delta*(std::abs(m[i]->range_m)+std::abs(first.range_m))+2*delta*delta;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a,Eigen::ComputeThinU|Eigen::ComputeThinV);
    const double round=128*std::numeric_limits<double>::epsilon();
    const double sigma=svd.singularValues()(2)-round*a.norm();
    if(!(sigma>1e-10*a.norm()))continue; // unresolved geometry stays refused
    const Eigen::Vector3d centre=svd.solve(y);
    const double position_limit=first.anchor_position_m.norm()+std::abs(first.range_m)+delta;
    const double arithmetic=round*(a.norm()*position_limit+y.norm()+
        first.anchor_position_m.squaredNorm()+first.range_m*first.range_m+1.);
    // A feasible exact range realization must satisfy the difference equations.
    // The least-squares orthogonal residual is a necessary feasibility test;
    // it cannot be inflated into a radius for an inconsistent fault subset.
    const double residual_norm=(a*centre-y).norm();
    if(residual_norm>uncertainty.norm()+arithmetic)continue;
    const double radius=(residual_norm+uncertainty.norm()+arithmetic)/sigma;
    bool consistent=centre.allFinite() && std::isfinite(radius);
    for(const auto* measurement:m) {
      const double residual=std::abs((centre-measurement->anchor_position_m).norm()-measurement->range_m);
      if(residual>radius+delta+round*(measurement->anchor_position_m.norm()+centre.norm()+1.))consistent=false;
    }
    if(consistent){proof.centres.push_back(centre);proof.radius.push_back(radius);}
  }
  proof.valid=!proof.centres.empty();
  proof.reason=proof.valid?"CONDITIONAL exact range intersection union":"no consistent full-rank range subset";
  return proof;
}
inline bool validRangeSet(const RangePositionSet& proof) {
  if(!proof.valid || proof.centres.empty() || proof.radius.size()!=proof.centres.size())return false;
  for(std::size_t i=0;i<proof.radius.size();++i)
    if(!proof.centres[i].allFinite() || !std::isfinite(proof.radius[i]) || proof.radius[i]<0.)return false;
  return true;
}
inline Eigen::Vector3d transferBound(const RangePositionSet& proof,const Eigen::Vector3d& mean) {
  if(!validRangeSet(proof) || !mean.allFinite())return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d reference_bound=Eigen::Vector3d::Zero();
  for(std::size_t i=0;i<proof.centres.size();++i)
    reference_bound=reference_bound.cwiseMax((proof.centres.front()-proof.centres[i]).cwiseAbs()+Eigen::Vector3d::Constant(proof.radius[i]));
  // Explicit common-reference triangle, same source/time/output for EVERY
  // candidate. It is an independently computed bound, never a hash alias.
  return reference_bound+(mean-proof.centres.front()).cwiseAbs();
}
inline Eigen::Vector3d relativeTransferBound(const RangePositionSet& proof,
    const Eigen::Vector3d& origin,const Eigen::Vector3d& shift) {
  if(!validRangeSet(proof) || !origin.allFinite() || !shift.allFinite())return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  const Eigen::Vector3d reference_bound=transferBound(proof,proof.centres.front());
  return reference_bound+(shift-(proof.centres.front()-origin)).cwiseAbs();
}
} // namespace
