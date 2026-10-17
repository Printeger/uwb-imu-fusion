#pragma once
#include "generalized_source_ledger.hpp"

namespace uwb_imu_pl::research {
// Necessary initial set from a single original range batch under the same
// one-physical-source fault union used online. Numerical initialization may
// use these measurements too: there is no second independent prior draw.
// The caller supplies declared deterministic v/R/bias domains, not an estimated
// covariance. Real CAL domains remain empirical unless their bounds are proved.
inline InitialSetEvidence measurementInitial(GeneralizedSourceLedger& sources,
 const UwbBatch& batch,RollingRawState domain,const Eigen::Vector3d& lever,
 double acceleration,double rate,ModelEvidenceStatus status,const std::string& model){
 using I=RollingInterval;
 if(status==ModelEvidenceStatus::MissingOrInconsistent || model.empty() || domain.epoch!=0 ||
    rawRangeFaultMasks(batch).size()<3 || !lever.allFinite())throw std::invalid_argument("BOOTSTRAP_SOURCE_GEOMETRY_MISSING");
 domain.has_tag_relation=false;
 std::vector<RawRangeShell> shells;Eigen::Vector3d low=Eigen::Vector3d::Constant(INFINITY),high=-low;
 for(const auto& m:batch.measurements){
  sources.append(m);
  const double q=sources.event().birth({SourceKind::Range,m.anchor_id.value(),m.id.value(),0}).q;
  const I radius=I(q)*I(m.sigma_m)+rangeTransportRadius(domain,m.timestamp,batch.timestamp,lever,acceleration,rate,sources.law().global_speed);
  const I upper=I(m.range_m)+radius;
  if(!std::isfinite(upper.upper()) || !m.anchor_position_m.allFinite())throw std::invalid_argument("BOOTSTRAP_RANGE_NONFINITE");
  shells.push_back({std::max(0.,(I(m.range_m)-radius).lower()),upper.upper()});
  // At least one distinct physical source is healthy. The union of ALL
  // upper balls therefore contains the tag before any branch contraction.
  for(int j=0;j<3;++j){low[j]=std::min(low[j],(I(m.anchor_position_m[j])-upper).lower());high[j]=std::max(high[j],(I(m.anchor_position_m[j])+upper).upper());}
 }
 RawRangeBudget budget;budget.tight_shell=true;budget.optimized=true;budget.retain_support=true;
 budget.lower_shell=true;budget.split_depth=2;budget.max_boxes=256;
 const auto bound=contractRawRangeBox(low,high,batch,shells,Eigen::Vector3d::Zero(),budget);
 if(!bound.valid)throw std::invalid_argument("BOOTSTRAP_ALL_BRANCHES_EMPTY");
 const I length=sqrt(square(I(lever.x()))+square(I(lever.y()))+square(I(lever.z())));
 RollingVector tag;for(int j=0;j<3;++j){tag[j]=I(bound.low[j],bound.high[j]);domain.p[j]=tag[j]+I(-length.upper(),length.upper());}
 if(!rollingContractTagSphere(domain,tag,lever))throw std::invalid_argument("BOOTSTRAP_LEVER_RELATION_EMPTY");
 domain.has_tag_relation=true;domain.tag_relation=tag;domain.tag_lever=lever;
 InitialSetEvidence out;out.domain=rollingProject(domain);out.timestamp=batch.timestamp;out.status=status;
 out.event_id=sources.event().id();out.model_id=model;out.source_prefix=sources.event().coordinates();out.risk_upper=sources.event().prefixUpper();out.measurement_derived=true;
 return out;
}
}
