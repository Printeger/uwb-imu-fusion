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
 Eigen::Vector3d reference_tag=Eigen::Vector3d(.2,-.1,1.08);
 double initial_sigma=.002, process_sigma=.0002, endpoint_sigma=.0001, range_sigma=.003;
 HybridFixedLaw(){
  F.block(3,6,3,3)=.05*Eigen::Matrix3d::Identity();
  F.block(0,12,3,3)=-.05*Eigen::Matrix3d::Identity();
  F.block(6,9,3,3)=-.05*Eigen::Matrix3d::Identity();
  L.topRows(6).setIdentity();C.block(0,3,3,3).setIdentity();
  Eigen::Vector3d lever(.2,-.1,.08);Eigen::Matrix3d skew;skew<<0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;C.leftCols(3)=-skew;
  // Registered before data: eight distinct fixed 3D range directions.
  for(int i=0;i<8;++i){Eigen::Vector3d d(i&1?1:-1,i&2?1:-1,i&4?1:-1);H.row(i)=d.normalized().transpose()*C;}
 }
 bool same(const HybridFixedLaw& b)const{
  auto matrix=[](const Eigen::MatrixXd& x,const Eigen::MatrixXd& y){return x.rows()==y.rows() && x.cols()==y.cols() && x.allFinite() && y.allFinite() && x.isApprox(y,0);};
  return reference_tag.isApprox(b.reference_tag,0) && horizon==b.horizon && ranges==b.ranges && episode_duration==b.episode_duration && epsilon==b.epsilon && budget==b.budget && alpha==b.alpha && initial_sigma==b.initial_sigma && process_sigma==b.process_sigma && endpoint_sigma==b.endpoint_sigma && range_sigma==b.range_sigma && matrix(F,b.F) && matrix(L,b.L) && matrix(H,b.H) && matrix(C,b.C);
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
 std::vector<uint64_t> sources_;std::vector<Eigen::VectorXd> observations_;std::vector<std::vector<int>> subsets_;std::string invalid_;
 struct Episode{std::string id;int onset,source,length;};
 std::vector<Episode> episodes_;
 HybridHistoryEvidence calculate()const{
  HybridHistoryEvidence e;e.epoch=epoch_;e.generation=generation_;e.horizon=law_.horizon;
  e.radius=law_.radius();e.risk=law_.epsilon;e.retained_sources=sources_;
  if(!invalid_.empty()){e.status=invalid_;return e;}
  if(bytes()>max_bytes_){e.status="EXHAUSTED";return e;}
  if(epoch_>=size_t(law_.horizon)){e.status="STALE";return e;}
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(summary_.h,Eigen::ComputeFullU|Eigen::ComputeFullV);
  auto singular=svd.singularValues();
  if(singular.size()!=21 || singular.minCoeff()<1e-9*singular.maxCoeff()){e.status="INDETERMINATE";return e;}
  Eigen::MatrixXd gain=svd.matrixV()*singular.cwiseInverse().asDiagonal()*svd.matrixU().leftCols(21).transpose();
  e.mean=gain*summary_.d;e.covariance=gain*gain.transpose();
  if(epoch_==0){e.status="BOUNDED";return e;}
  // Read residual sufficient statistics directly. No historical C PL, tag
  // slope, or candidate search is computed as a side effect of H issuance.
  Eigen::MatrixXd parity=svd.matrixU().rightCols(summary_.h.rows()-21).transpose();
  Eigen::VectorXd residual=parity*summary_.d;
  for(const auto& item:summary_.faults){const auto& f=item.second;
   Eigen::VectorXd direction=-gain*f.top.col(0);
   if(!f.ever_nonzero || (direction.array()==0.).all())continue;
   Eigen::VectorXd z(parity.rows()+f.residual.rows()),r(residual.size()+f.residual.rows());
   z.head(parity.rows())=parity*f.top.col(0);z.tail(f.residual.rows())=f.residual.col(0);
   r.head(residual.size())=residual;r.tail(f.residual.rows())=f.residual.col(1);
   double norm=z.norm();if(norm<=1e-8){e.status="INDETERMINATE";return e;}
   double estimate=z.dot(r)/(norm*norm),half=e.radius/norm;
   e.segments.push_back({item.first,direction,estimate-half,estimate+half});
  }
  e.status="BOUNDED";return e;
 }
 public:
 explicit HybridHistoryProducer(HybridFixedLaw law,size_t max_bytes=16*1024*1024):law_(std::move(law)),max_bytes_(max_bytes){
  if(!law_.reference_tag.allFinite() || law_.F.rows()!=15 || law_.F.cols()!=15 || law_.L.rows()!=15 || law_.L.cols()!=6 || law_.H.rows()!=8 || law_.H.cols()!=15 || law_.C.rows()!=3 || law_.C.cols()!=15 || !law_.F.allFinite() || !law_.L.allFinite() || !law_.H.allFinite() || !law_.C.allFinite() || !std::isfinite(law_.initial_sigma+law_.process_sigma+law_.endpoint_sigma+law_.range_sigma) || law_.horizon<=0 || law_.horizon>50 || law_.ranges!=8 || law_.episode_duration!=3 || !(law_.epsilon>0 && law_.epsilon<law_.budget && law_.budget<1) || law_.initial_sigma<=0 || law_.process_sigma<=0 || law_.endpoint_sigma<=0 || law_.range_sigma<=0)throw std::runtime_error("H_MODEL_INVALID");
  Eigen::MatrixXd prior=Eigen::MatrixXd::Zero(21,21);
  prior.topLeftCorner(15,15).diagonal().setConstant(1/law_.initial_sigma);
  prior.bottomRightCorner(6,6).diagonal().setConstant(1/law_.endpoint_sigma);
  summary_.append(prior,Eigen::VectorXd::Zero(21),{});
 }
 HybridHistoryEvidence query()const{return calculate();}
 const HybridFixedLaw& law()const{return law_;}
 void invalidate(const std::string& reason){invalid_=reason;}
 size_t bytes()const{size_t result=summary_.bytes()+sources_.size()*sizeof(uint64_t);for(const auto& observation:observations_)result+=observation.size()*sizeof(double);return result;}
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
  summary_.append(a,z,faults);summary_.retire(21);observations_.push_back(ranges);subsets_.push_back(retained);epoch_=epoch;generation_=generation;
 }
 void rebuild(){
  HybridHistoryProducer replay(law_,max_bytes_);
  for(size_t i=0;i<observations_.size();++i){std::vector<uint64_t> ids;for(int row:subsets_[i])ids.push_back((i+1)*100+row+1);
   replay.committed(i+1,i+1,observations_[i],subsets_[i],ids);
  }
  rebuiltFrom(replay);
 }
 // Legal rebuild reruns the actual recipe from initial law and committed
 // sources. No timestamp-only refresh and no consecutive-PASS recovery.
 void rebuiltFrom(const HybridHistoryProducer& replay){
  if(!law_.same(replay.law_) || replay.epoch_!=epoch_ || replay.generation_!=generation_ || replay.sources_!=sources_ || !replay.invalid_.empty() || replay.query().status!="BOUNDED")throw std::runtime_error("H_REBUILD_LINEAGE");
  for(size_t i=0;i<observations_.size();++i)if(subsets_[i]!=replay.subsets_[i] || !observations_[i].isApprox(replay.observations_[i],0))throw std::runtime_error("H_REBUILD_RAW_VALUES");
  summary_=replay.summary_;episodes_=replay.episodes_;invalid_.clear();
 }
};
} // namespace
