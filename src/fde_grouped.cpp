#include "uifgo/nlos_fde.h"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <map>
#include <set>
#include <tuple>
#include <stdexcept>

namespace uifgo {
std::vector<FdeGroupTest> ApplyGroupedFdeTests(
    std::vector<FdeObservationRecord>* observations,
    const Eigen::MatrixXd& covariance,
    const std::vector<std::uint64_t>& ids,
    const FdeOptions& options, std::string* status) {
  if (options.chi2_probability != .99 || !std::isfinite(options.gap_threshold_s) ||
      options.gap_threshold_s < 0 || !options.minimum_count ||
      !std::isfinite(options.minimum_duration_s) || options.minimum_duration_s < 0 ||
      !observations || !status || covariance.rows()!=static_cast<int>(ids.size()) ||
      covariance.cols()!=covariance.rows() || !covariance.allFinite())
    throw std::invalid_argument("GROUPED_FDE_COVARIANCE_IDENTITY_INVALID");
  std::map<uint64_t,size_t> by_id;
  for(size_t j=0;j<ids.size();++j)
    if(!by_id.emplace(ids[j],j).second)
      throw std::invalid_argument("GROUPED_FDE_DUPLICATE_COVARIANCE_ID");
  std::vector<size_t> order;
  std::set<uint64_t> unique;
  for(size_t j=0;j<observations->size();++j) {
    auto& row=observations->at(j);
    if(!row.valid || !row.planned) continue;
    if(!row.tested || !std::isfinite(row.residual_m) ||
       !std::isfinite(row.factor_sigma_m) || row.factor_sigma_m<=0 ||
       !std::isfinite(row.sensor_time) || !by_id.count(row.obs_id) ||
       !unique.insert(row.obs_id).second)
      throw std::invalid_argument("GROUPED_FDE_OBSERVATION_MAPPING_INVALID");
    order.push_back(j);
    row.nlos_candidate=false;
    row.candidate_filter_reason="GROUP_NOT_SIGNIFICANT_POSITIVE_EXCESS";
  }
  if(order.size()!=ids.size())
    throw std::invalid_argument("GROUPED_FDE_ROW_COVERAGE_INVALID");
  std::sort(order.begin(),order.end(),[&](size_t a,size_t b){
    const auto& x=observations->at(a);const auto& y=observations->at(b);
    return std::tie(x.tag_id,x.anchor_id,x.sensor_time,x.obs_id)<
           std::tie(y.tag_id,y.anchor_id,y.sensor_time,y.obs_id);
  });
  std::vector<std::vector<size_t>> groups;
  for(size_t j:order) {
    const auto& row=observations->at(j);
    if(groups.empty()) groups.emplace_back();
    if(!groups.back().empty()) {
      const auto& prev=observations->at(groups.back().back());
      if(prev.tag_id!=row.tag_id || prev.anchor_id!=row.anchor_id ||
         row.sensor_time-prev.sensor_time>options.gap_threshold_s)
        groups.emplace_back();
    }
    groups.back().push_back(j);
  }
  std::vector<FdeGroupTest> results;
  std::set<std::pair<int,int>> qualified_links;
  for(const auto& group:groups) {
    const auto& first=observations->at(group.front());
    const auto& last=observations->at(group.back());
    FdeGroupTest record;record.tag_id=first.tag_id;record.anchor_id=first.anchor_id;
    record.start_time=first.sensor_time;record.end_time=last.sensor_time;
    for(size_t j:group)record.obs_ids.push_back(observations->at(j).obs_id);
    if(group.size()<options.minimum_count || record.end_time-record.start_time<options.minimum_duration_s) {
      record.status="FILTERED_ORIGINAL_COUNT_DURATION";
      results.push_back(std::move(record));continue;
    }
    Eigen::VectorXd e(group.size()),direction(group.size());
    record.covariance.resize(group.size(),group.size());
    for(size_t j=0;j<group.size();++j) {
      const auto& row=observations->at(group[j]);
      direction[j]=1/row.factor_sigma_m;e[j]=row.residual_m/row.factor_sigma_m;
      for(size_t k=0;k<group.size();++k)
        record.covariance(j,k)=covariance(by_id.at(row.obs_id),by_id.at(observations->at(group[k]).obs_id));
    }
    record.test=FdeCovarianceTest(e,record.covariance);
    if(!record.test.valid)throw std::runtime_error("GROUPED_FDE_UNAVAILABLE:"+record.test.reason);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig((record.covariance+record.covariance.transpose())*.5);
    const Eigen::VectorXd de=eig.eigenvectors().transpose()*direction;
    const Eigen::VectorXd ee=eig.eigenvectors().transpose()*e;
    double numerator=0,denominator=0;
    for(int j=0;j<ee.size();++j)if(eig.eigenvalues()[j]>record.test.rank_threshold) {
      numerator+=de[j]*ee[j]/eig.eigenvalues()[j];
      denominator+=de[j]*de[j]/eig.eigenvalues()[j];
    }
    if(!std::isfinite(numerator) || !std::isfinite(denominator) || denominator<=0)
      throw std::runtime_error("GROUPED_FDE_GLS_DIRECTION_UNAVAILABLE");
    record.gls_signed_residual_m=numerator/denominator;
    record.positive_excess=record.gls_signed_residual_m<0;
    record.status=record.test.rejected && record.positive_excess ? "SIGNIFICANT_POSITIVE_EXCESS" : "CONSISTENT_OR_NON_POSITIVE_EXCESS";
    if(record.test.rejected && record.positive_excess)
      qualified_links.insert({record.tag_id,record.anchor_id});
    results.push_back(std::move(record));
  }
  *status=qualified_links.empty()?"GROUP_CONSISTENT_NO_SUPPORT":
          (qualified_links.size()==1?"GROUP_UNIQUE_POSITIVE_LINK":"FDE_ISOLATION_AMBIGUOUS");
  for(size_t k=0;k<groups.size();++k) {
    const auto& result=results[k];
    for(size_t j:groups[k]) {
      auto& row=observations->at(j);
      if(qualified_links.size()>1)row.candidate_filter_reason="FDE_ISOLATION_AMBIGUOUS";
      else if(qualified_links.size()==1 && result.test.rejected && result.positive_excess && row.residual_m<0) {
        row.nlos_candidate=true;row.candidate_filter_reason="PENDING_TEMPORAL_FILTER";
      } else if(result.status=="FILTERED_ORIGINAL_COUNT_DURATION")
        row.candidate_filter_reason=result.status;
    }
  }
  return results;
}

void ApplyFullGraphGroupedFde(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,const std::vector<FactorMeta>& metadata,
    const FdeOptions& options,FdeResult* result) {
  const auto residuals=ReadScalarUwbFactorResiduals(graph,values,metadata);
  auto linear=graph.linearize(values);gtsam::Ordering ordering;
  for(const auto& item:values)ordering.push_back(item.key);
  size_t rows=0,columns=0;
  auto entries=linear->sparseJacobian(ordering,rows,columns);
  if(columns!=values.dim()+1)throw std::runtime_error("GROUPED_FDE_DIMENSION_MISMATCH");
  Eigen::VectorXd e=Eigen::VectorXd::Zero(rows);
  std::vector<Eigen::Triplet<double>> triplets;
  for(const auto& entry:entries) {
    int r=std::get<0>(entry),c=std::get<1>(entry);double x=std::get<2>(entry);
    if(c==static_cast<int>(columns-1))e[r]=-x;else triplets.emplace_back(r,c,x);
  }
  Eigen::SparseMatrix<double>A(rows,columns-1);A.setFromTriplets(triplets.begin(),triplets.end());
  std::vector<size_t> offsets;size_t offset=0;
  for(size_t j=0;j<graph.size();++j) {
    offsets.push_back(offset);
    if(!linear->at(j) || linear->at(j)->augmentedJacobian().rows()!=graph.at(j)->dim())
      throw std::runtime_error("GROUPED_FDE_FACTOR_ROW_MISMATCH");
    offset+=graph.at(j)->dim();
  }
  if(offset!=rows)throw std::runtime_error("GROUPED_FDE_ROW_COVERAGE_MISMATCH");
  std::vector<size_t> selected;std::vector<uint64_t> ids;
  for(const auto& row:residuals){selected.push_back(offsets.at(row.factor_index));ids.push_back(row.obs_id);}
  auto block=FdeFullProjectionBlock(A,e,selected);
  if(!block.certificate.valid)throw std::runtime_error("GROUPED_FDE_PROJECTOR_UNAVAILABLE:"+block.certificate.reason);
  result->group_tests=ApplyGroupedFdeTests(&result->observations,block.Pww,ids,options,&result->grouped_status);
  result->positive_candidate_count=0;
  for(const auto& row:result->observations)result->positive_candidate_count+=row.nlos_candidate;
}
} // namespace uifgo
