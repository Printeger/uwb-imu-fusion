#pragma once
#include "gaussian_sufficient_summary.hpp"
#include <set>
namespace uwb_imu_pl::research {
// A single common primitive Gaussian ball covers every time, episode and
// adaptive retained subset. No multiplication of detector pass probabilities.
struct HybridFixedLaw {
 int horizon=24, ranges=8, episode_duration=3;
 double epsilon=1e-6, budget=2e-6, alpha=1e-6;
 Eigen::MatrixXd F=Eigen::MatrixXd::Identity(15,15);
 Eigen::MatrixXd L=Eigen::MatrixXd::Zero(15,6);
 Eigen::MatrixXd H=Eigen::MatrixXd::Zero(8,15);
 Eigen::MatrixXd C=Eigen::MatrixXd::Zero(3,15);
 double initial_sigma=.002, process_sigma=.0002, endpoint_sigma=.0001, range_sigma=.003;
 HybridFixedLaw(){
  F.block(3,6,3,3)=.05*Eigen::Matrix3d::Identity();
  F.block(0,12,3,3)=-.05*Eigen::Matrix3d::Identity();
  F.block(6,9,3,3)=-.05*Eigen::Matrix3d::Identity();
  L.topRows(6).setIdentity();C.block(0,3,3,3).setIdentity();
  // Registered before data: eight distinct fixed 3D range directions.
  for(int i=0;i<8;++i){Eigen::Vector3d d(i&1?1:-1,i&2?1:-1,i&4?1:-1);H.block(i,3,1,3)=d.normalized().transpose();}
 }
 bool same(const HybridFixedLaw& b)const{
  return horizon==b.horizon && ranges==b.ranges && episode_duration==b.episode_duration && epsilon==b.epsilon && budget==b.budget && alpha==b.alpha && initial_sigma==b.initial_sigma && process_sigma==b.process_sigma && endpoint_sigma==b.endpoint_sigma && range_sigma==b.range_sigma && F.isApprox(b.F,0) && L.isApprox(b.L,0) && H.isApprox(b.H,0) && C.isApprox(b.C,0);
 }
 double radius()const{return std::sqrt(boost::math::quantile(boost::math::complement(
  boost::math::chi_squared(21+horizon*(21+ranges)),epsilon)));}
};
struct HistorySegment {
 std::string episode;
 Eigen::VectorXd direction;
 double low=0,high=0;
};
struct HybridHistoryEvidence {
 std::string status="UNINITIALIZED",model="hybrid-fixed-21-v1";
 size_t epoch=0,generation=0;int horizon=0;double radius=0,risk=0;
 Eigen::VectorXd mean;
 Eigen::MatrixXd covariance;
 std::vector<HistorySegment> segments;
 std::vector<uint64_t> retained_sources;
 // Union of line segments, including no fault. Gaussian part is separate.
 double support(const Eigen::RowVectorXd& row)const{
  double bound=0;for(const auto& s:segments)bound=std::max(bound,std::abs(row.dot(s.direction))*std::max(std::abs(s.low),std::abs(s.high)));return bound;
 }
 double imageNorm(const Eigen::MatrixXd& a)const{
  double bound=0;for(const auto& s:segments)bound=std::max(bound,(a*s.direction).norm()*std::max(std::abs(s.low),std::abs(s.high)));return bound;
 }
};
// Research fixed-law producer. Reuses the existing square-root retirement,
// provenance and fault residual machinery, without calling C candidate search.
class HybridHistoryProducer {
 HybridFixedLaw law_;GaussianSufficientSummary summary_{21};
 size_t epoch_=0,generation_=0,max_bytes_=16*1024*1024;
 std::vector<uint64_t> sources_;std::string invalid_;
 struct Episode{std::string id;int onset,source,length;};
 std::vector<Episode> episodes_;
 HybridHistoryEvidence calculate()const{
  HybridHistoryEvidence e;e.epoch=epoch_;e.generation=generation_;e.horizon=law_.horizon;
  e.radius=law_.radius();e.risk=law_.epsilon;e.retained_sources=sources_;
  if(!invalid_.empty()){e.status=invalid_;return e;}
  if(summary_.bytes()>max_bytes_){e.status="EXHAUSTED";return e;}
  if(epoch_>=size_t(law_.horizon)){e.status="STALE";return e;}
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(summary_.h,Eigen::ComputeThinU|Eigen::ComputeFullV);
  auto singular=svd.singularValues();
  if(singular.size()!=21 || singular.minCoeff()<1e-9*singular.maxCoeff()){e.status="INDETERMINATE";return e;}
  Eigen::MatrixXd gain=svd.matrixV()*singular.cwiseInverse().asDiagonal()*svd.matrixU().transpose();
  e.mean=gain*summary_.d;e.covariance=gain*gain.transpose();
  if(epoch_==0){e.status="BOUNDED";return e;}
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,21);c.leftCols(15)=law_.C;
  auto evaluated=summary_.evaluate(c,law_.alpha,law_.epsilon,true);
  for(const auto& item:summary_.faults){const auto& f=item.second;const auto& result=evaluated.faults.at(item.first);
   Eigen::VectorXd direction=-gain*f.top.col(0);
   if(!f.ever_nonzero)continue;
   if(result.classification!="MONITORABLE"){
    // Only exactly zero full-state effect is harmless for future propagation.
    if((direction.array()==0.).all())continue;
    e.status=result.classification;return e;
   }
   double gram=result.residual_gram(0,0);
   double estimate=result.profile_rhs(0)/gram,half=e.radius/std::sqrt(gram);
   e.segments.push_back({item.first,direction,estimate-half,estimate+half});
  }
  e.status="BOUNDED";return e;
 }
 public:
 explicit HybridHistoryProducer(HybridFixedLaw law,size_t max_bytes=16*1024*1024):law_(std::move(law)),max_bytes_(max_bytes){
  if(law_.horizon<=0 || law_.horizon>50 || law_.ranges!=8 || law_.episode_duration!=3 || !(law_.epsilon>0 && law_.epsilon<law_.budget && law_.budget<1) || law_.initial_sigma<=0 || law_.process_sigma<=0 || law_.endpoint_sigma<=0 || law_.range_sigma<=0)throw std::runtime_error("H_MODEL_INVALID");
  Eigen::MatrixXd prior=Eigen::MatrixXd::Zero(21,21);
  prior.topLeftCorner(15,15).diagonal().setConstant(1/law_.initial_sigma);
  prior.bottomRightCorner(6,6).diagonal().setConstant(1/law_.endpoint_sigma);
  summary_.append(prior,Eigen::VectorXd::Zero(21),{});
 }
 HybridHistoryEvidence query()const{return calculate();}
 const HybridFixedLaw& law()const{return law_;}
 void invalidate(const std::string& reason){invalid_=reason;}
 size_t bytes()const{return summary_.bytes();}
 // Receipt is already verified by the coordinator. This method still requires
 // exact sequential identity and actual retained raw IDs, never a health flag.
 void committed(size_t epoch,size_t generation,const Eigen::VectorXd& ranges,
                const std::vector<int>& retained,const std::vector<uint64_t>& actual_ids){
  if(epoch!=epoch_+1 || generation!=generation_+1 || ranges.size()!=law_.ranges || actual_ids.size()!=retained.size())throw std::runtime_error("H_COMMIT_LINEAGE");
  if(epoch>size_t(law_.horizon))throw std::runtime_error("H_HORIZON");
  std::set<int> unique;std::set<uint64_t> unique_ids;
  for(size_t i=0;i<retained.size();++i)if(retained[i]<0 || retained[i]>=law_.ranges || !unique.insert(retained[i]).second || !unique_ids.insert(actual_ids[i]).second || actual_ids[i]!=epoch*100+retained[i]+1)throw std::runtime_error("H_SOURCE_LINEAGE");
  for(int source=0;source<law_.ranges;++source)for(int length:{1,law_.episode_duration}){
   std::string id="source:"+std::to_string(source)+":onset:"+std::to_string(epoch)+":length:"+std::to_string(length);
   summary_.declareFault(id,1,-1,true);episodes_.push_back({id,int(epoch),source,length});
  }
  summary_.addStateColumns(21);
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(21+retained.size(),42);
  // Whiten independent process and endpoint priors once. The previous shared
  // endpoint remains a state column across retirement; it is never redrawn.
  a.block(0,0,15,15)=-law_.F/law_.process_sigma;
  a.block(0,15,15,6)=-law_.L/law_.process_sigma;
  a.block(0,21,15,15)=Eigen::MatrixXd::Identity(15,15)/law_.process_sigma;
  a.block(0,36,15,6)=-law_.L/law_.process_sigma;
  a.block(15,36,6,6)=Eigen::MatrixXd::Identity(6,6)/law_.endpoint_sigma;
  Eigen::VectorXd z=Eigen::VectorXd::Zero(a.rows());
  std::map<std::string,Eigen::MatrixXd> faults;
  for(const auto& episode:episodes_){Eigen::MatrixXd f=Eigen::MatrixXd::Zero(a.rows(),1);
   for(size_t i=0;i<retained.size();++i)if(retained[i]==episode.source && int(epoch)>=episode.onset && int(epoch)<episode.onset+episode.length)f(21+i)=1/law_.range_sigma;
   faults.emplace(episode.id,std::move(f));
  }
  for(size_t i=0;i<retained.size();++i){a.block(21+i,21,1,15)=law_.H.row(retained[i])/law_.range_sigma;z(21+i)=ranges(retained[i])/law_.range_sigma;sources_.push_back(actual_ids[i]);}
  summary_.append(a,z,faults);summary_.retire(21);epoch_=epoch;generation_=generation;
 }
 // Legal rebuild reruns the actual recipe from initial law and committed
 // sources. No timestamp-only refresh and no consecutive-PASS recovery.
 void rebuiltFrom(const HybridHistoryProducer& replay){
  if(!law_.same(replay.law_) || replay.epoch_!=epoch_ || replay.generation_!=generation_ || replay.sources_!=sources_ || !replay.invalid_.empty() || replay.query().status!="BOUNDED")throw std::runtime_error("H_REBUILD_LINEAGE");
  summary_=replay.summary_;episodes_=replay.episodes_;invalid_.clear();
 }
};
} // namespace
