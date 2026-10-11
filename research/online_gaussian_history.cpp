#include "online_gaussian_history.hpp"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Ordering.h>
#include <set>
namespace uwb_imu_pl::research {
struct OnlineGaussianHistory::Impl {
 struct Mode {std::string id;AnchorId anchor;size_t onset;int duration;};
 size_t lag,frontier=0,epoch=0;int duration;double sa,sg;Eigen::Vector3d lever;
 GaussianSufficientSummary history{21};
 std::map<uint64_t,PendingFactorGroup> groups;
 std::map<size_t,HistoricalEpochContext> episodes;
 std::map<uint64_t,std::pair<size_t,AnchorId>> lineage;
 std::set<uint64_t> retired_groups;
 std::vector<Mode> modes;
 std::vector<SummaryRetirement> old_navigation;
 gtsam::Values points;
 LinearizedIntegrityWindow local_window;
 GaussianWindow local_noise;
 EpochTransaction material;
 bool initialized=false;TransactionId prepared_id;
 LinearizationVersion prepared_version;
 size_t original_rows=0;
 Impl(size_t w,int d,double a,double g,Eigen::Vector3d l):lag(w),duration(d),sa(a),sg(g),lever(l){
  if(w<2 || d<1 || !(a>0 && g>0))throw std::runtime_error("invalid online history model");
  Eigen::MatrixXd prior=Eigen::MatrixXd::Zero(6,21);prior.rightCols(6).setIdentity();history.append(prior,Eigen::VectorXd::Zero(6),{});
 }
 static size_t endEpoch(const PendingFactorGroup& g){size_t e=0;for(auto key:g.keys)e=std::max(e,(size_t)gtsam::Symbol(key).index());return e;}
 LinearizedFactorBlock block(const PendingFactorGroup& g,size_t first,size_t last)const{
  if(g.kind==FactorKind::KinematicBridge || g.kind==FactorKind::BiasContinuity)throw std::runtime_error("UNAVAILABLE_IMU_BRIDGE_GAUSSIAN_HISTORY");
  gtsam::Ordering order;for(auto key:g.keys)order.push_back(key);
  auto dense=g.factors.linearize(points)->jacobian(order);LinearizedFactorBlock b;
  b.group_id=g.id;b.kind=g.kind;b.sensor=g.sensor;b.version=prepared_version;
  b.jacobian_whitened=Eigen::MatrixXd::Zero(dense.first.rows(),15*(last-first+1));b.residual_whitened=dense.second;
  int cursor=0;for(auto key:order){char ch=gtsam::Symbol(key).chr();size_t e=gtsam::Symbol(key).index();int dim=ch=='v'?3:6;
   if(e<first || e>last)throw std::runtime_error("UNAVAILABLE_FACTOR_OUTSIDE_SUFFIX");
   int off=15*(e-first)+(ch=='x'?0:(ch=='v'?6:9));b.jacobian_whitened.middleCols(off,dim)=dense.first.middleCols(cursor,dim);cursor+=dim;}
  if(g.kind==FactorKind::CombinedImu){auto f=dynamic_cast<const gtsam::CombinedImuFactor*>(g.factors.front().get());if(!f)throw std::runtime_error("IMU provenance mismatch");b.covariance=f->preintegratedMeasurements().preintMeasCov();}
  else if(g.kind==FactorKind::BoundaryPrior)b.covariance=Eigen::MatrixXd::Identity(dense.second.size(),dense.second.size());else b.covariance=g.raw_covariance;
  if(b.covariance.rows()!=dense.second.size())throw std::runtime_error("UNAVAILABLE_RAW_COVARIANCE_LINEAGE");
  auto r=gtsam::noiseModel::Gaussian::Covariance(b.covariance)->R();b.jacobian_raw=r.triangularView<Eigen::Upper>().solve(b.jacobian_whitened);b.residual_raw=r.triangularView<Eigen::Upper>().solve(b.residual_whitened);return b;
 }
 std::map<std::string,Eigen::MatrixXd> faultRows(const PendingFactorGroup& g)const{
  std::map<std::string,Eigen::MatrixXd> out;if(g.sensor!=SensorType::Uwb)return out;
  for(const auto& mode:modes){Eigen::MatrixXd b=Eigen::MatrixXd::Zero(g.source_measurements.size(),1);
   for(size_t i=0;i<g.source_measurements.size();++i){auto it=lineage.find(g.source_measurements[i].value());if(it==lineage.end())throw std::runtime_error("UNAVAILABLE_UWB_SOURCE_LINEAGE");
    if(it->second.second==mode.anchor && it->second.first>=mode.onset && it->second.first<mode.onset+mode.duration)b(i)=1.;}
   if(b.squaredNorm()>0)out.emplace(mode.id,std::move(b));}
  return out;
 }
 // Convert one raw block to independent conditional noise rows. Shared endpoint
 // ADC coordinates remain explicit columns, with a prior added ONCE per source.
 void appendBlock(GaussianSufficientSummary& target,const PendingFactorGroup& g,size_t first,size_t last,const std::vector<int>* keep=nullptr)const{
  auto it=std::find_if(local_window.blocks.begin(),local_window.blocks.end(),[&](const auto& b){return b.group_id==g.id;});
  if(it==local_window.blocks.end())throw std::runtime_error("UNAVAILABLE_REPLACEMENT_BLOCK");
  size_t k=it-local_window.blocks.begin();int offset=local_noise.offsets[k],rows=it->residual_raw.size();
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(rows,21*(last-first+1));
  for(size_t e=first;e<=last;++e)a.middleCols(21*(e-first),15)=it->jacobian_raw.middleCols(15*(e-frontier),15);
  auto z=it->residual_raw;Eigen::MatrixXd covariance=it->covariance;
  if(g.kind==FactorKind::CombinedImu){
   covariance=local_noise.auxiliary_sigma.block(offset,offset,rows,rows);
   for(int col=0;col<local_noise.noise_j.cols();++col){auto j=local_noise.noise_j.block(offset,col,rows,1).eval();if((j.array()==0.).all())continue;
    const auto& id=local_noise.noise_source_ids[col];size_t separator=id.find(':',4);long long time=std::stoll(id.substr(4,separator-4));int axis=std::stoi(id.substr(separator+1));
    if(time%50000000LL==0){size_t e=time/50000000LL;if(e<first || e>last)throw std::runtime_error("UNAVAILABLE_NOISE_FRONTIER");a.col(21*(e-first)+15+axis)=j;}
    else covariance.noalias()+=j*j.transpose();
   }
  }
  auto b=faultRows(g);
  if(keep){Eigen::MatrixXd selected_a(keep->size(),a.cols());Eigen::VectorXd selected_z(keep->size());
   for(size_t i=0;i<keep->size();++i){selected_a.row(i)=a.row(keep->at(i));selected_z(i)=z(keep->at(i));}
   for(auto& f:b){Eigen::MatrixXd selected_b(keep->size(),f.second.cols());for(size_t i=0;i<keep->size();++i)selected_b.row(i)=f.second.row(keep->at(i));f.second=std::move(selected_b);}
   covariance=principal(covariance,*keep);a=std::move(selected_a);z=std::move(selected_z);
  }
  if(a.rows()==0)return;
  Eigen::LLT<Eigen::MatrixXd> chol(covariance);if(chol.info()!=Eigen::Success)throw std::runtime_error("INDETERMINATE_CONDITIONAL_COVARIANCE");
  for(auto& f:b)f.second=chol.matrixL().solve(f.second).eval();
  target.append(chol.matrixL().solve(a),chol.matrixL().solve(z),b);
 }
 void noisePrior(GaussianSufficientSummary& target,size_t first,size_t e)const{
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(6,target.states());a.block(0,21*(e-first)+15,6,6).setIdentity();target.append(a,Eigen::VectorXd::Zero(6),{});
 }
};
OnlineGaussianHistory::OnlineGaussianHistory(size_t lag,int duration,double sa,double sg,Eigen::Vector3d lever):impl(new Impl(lag,duration,sa,sg,lever)){}
OnlineGaussianHistory::~OnlineGaussianHistory()=default;
void OnlineGaussianHistory::prepare(const EpochTransaction& tx){auto& s=*impl;
 if(tx.proposed_epoch!=s.epoch+1 || tx.proposed_epoch>50)throw std::runtime_error("UNAVAILABLE_HISTORY_SEQUENCE_OR_HORIZON");
 s.epoch=tx.proposed_epoch;s.prepared_id=tx.id;s.prepared_version=tx.base_version;
 size_t changing=s.epoch>s.lag?s.epoch-s.lag:0;
 for(const auto& item:*tx.frozen_values){if(gtsam::Symbol(item.key).index()<s.frontier)continue;
  if(!s.points.exists(item.key))s.points.insert(item.key,item.value);else if(gtsam::Symbol(item.key).index()>=changing)s.points.update(item.key,item.value);}
 if(!s.initialized){for(const auto& f:tx.frozen_slots)if(f.factor){auto id=f.group_id.value_or(FactorGroupId(f.slot));auto& g=s.groups[id.value()];g.id=id;g.kind=FactorKind::BoundaryPrior;g.sensor=SensorType::Prior;g.factors.push_back(f.factor);
   for(auto key:f.factor->keys())if(std::find(g.keys.begin(),g.keys.end(),key)==g.keys.end())g.keys.push_back(key);}
  s.initialized=true;}
 HistoricalEpochContext episode;episode.previous_epoch=tx.previous_epoch;episode.proposed_epoch=tx.proposed_epoch;episode.begin=tx.begin;episode.end=tx.end;episode.raw_imu_slice=tx.raw_imu_slice;episode.preintegration=tx.preintegration;episode.groups={tx.imu_group};episode.uwb_batch=tx.uwb_batch;s.episodes[s.epoch]=episode;
 std::set<AnchorId> anchors;for(const auto& m:tx.uwb_batch.measurements){s.lineage[m.id.value()]={s.epoch,m.anchor_id};anchors.insert(m.anchor_id);}
 for(auto anchor:anchors)for(int length:{1,s.duration}){std::string id="prefix-anchor:"+std::to_string(anchor.value())+":onset:"+std::to_string(s.epoch)+":duration:"+std::to_string(length);
  s.modes.push_back({id,anchor,s.epoch,length});s.history.declareFault(id,1,-1,true);}
 auto current=s.groups;current[tx.imu_group.id.value()]=tx.imu_group;for(const auto& g:tx.uwb_groups)if(g.nominal)current[g.id.value()]=g;
 s.local_window={};s.local_window.id=WindowId(tx.id.value());s.local_window.version=tx.base_version;
 for(const auto& g:current)s.local_window.blocks.push_back(s.block(g.second,s.frontier,s.epoch));
 s.local_window.H=Eigen::MatrixXd::Zero(0,15*(s.epoch-s.frontier+1));s.local_window.protected_state_map=Eigen::MatrixXd::Zero(3,s.local_window.H.cols());
 s.local_window.protected_state_map.rightCols(12).leftCols(3)=s.points.at<gtsam::Pose3>(gtsam::Symbol('x',s.epoch)).rotation().matrix();
 s.material=tx;s.material.frozen_values=std::make_shared<const gtsam::Values>(s.points);s.material.recoverable_history.clear();
 for(const auto& h:s.episodes)if(h.first<s.epoch)s.material.recoverable_history.push_back(h.second);
 s.local_noise=rawGaussianWindow(s.local_window,s.material,{},s.sa,s.sg,s.lever,true);
 size_t next=s.epoch>s.lag+1?s.epoch-s.lag-1:0;
 // local rows use the old frontier column origin until all retirement completes.
 const size_t old_frontier=s.frontier;
 if(next>s.frontier){s.history.addStateColumns(21);s.noisePrior(s.history,s.frontier,next);
  for(const auto& g:current)if(Impl::endEpoch(g.second)<=next){s.appendBlock(s.history,g.second,s.frontier,next);s.retired_groups.insert(g.first);s.groups.erase(g.first);
   auto block=std::find_if(s.local_window.blocks.begin(),s.local_window.blocks.end(),[&](const auto& b){return b.group_id==g.second.id;});s.original_rows+=block->residual_raw.size();}
  auto recovery=s.history.retire(21);
  for(auto& old:s.old_navigation){old.offset+=old.coefficients*recovery.offset;old.coefficients=(old.coefficients*recovery.coefficients).eval();}
  s.old_navigation.push_back({recovery.coefficients.topRows(15),recovery.offset.head(15)});
 }
 // Leave the local matrix origin immutable for candidate evaluation; the
 // effective history frontier is updated only through the bounded suffix map.
 s.frontier=old_frontier;
}
SummaryEvaluation OnlineGaussianHistory::evaluate(const EpochTransaction& tx,const ExclusionAction* action,double alpha,double tail,bool faults)const{
 const auto& s=*impl;if(tx.id!=s.prepared_id || !(tx.base_version==s.prepared_version))throw std::runtime_error("foreign online summary root");
 size_t first=s.epoch>s.lag+1?s.epoch-s.lag-1:0;
 GaussianSufficientSummary query=s.history;query.addStateColumns(21*(s.epoch-first));
 for(size_t e=first+1;e<=s.epoch;++e)s.noisePrior(query,first,e);
 auto groups=s.groups;groups[tx.imu_group.id.value()]=tx.imu_group;for(const auto& g:tx.uwb_groups)if(g.nominal)groups[g.id.value()]=g;
 if(action){if(action->bridge_mode!=BridgeMode::None)throw std::runtime_error("UNAVAILABLE_BRIDGE_SUMMARY_SCOPE");
  for(auto added:action->groups_to_add)if(std::none_of(tx.uwb_groups.begin(),tx.uwb_groups.end(),[&](const auto& g){return g.id==added;}))throw std::runtime_error("UNAVAILABLE_ADDED_GROUP_PROVENANCE");}
 if(action)for(auto id:action->groups_to_remove)if(s.retired_groups.count(id.value()) || !groups.count(id.value()))throw std::runtime_error("UNAVAILABLE_RETIRED_CANDIDATE_SOURCE");
 for(const auto& item:groups){const auto& g=item.second;if(Impl::endEpoch(g)<first || (first>0 && Impl::endEpoch(g)==first))continue;
  bool remove=action && std::find(action->groups_to_remove.begin(),action->groups_to_remove.end(),g.id)!=action->groups_to_remove.end();
  if(!remove){s.appendBlock(query,g,first,s.epoch);continue;}
  std::vector<int> retain;
  if(action)for(auto added:action->groups_to_add){auto replacement=std::find_if(tx.uwb_groups.begin(),tx.uwb_groups.end(),[&](const auto& candidate){return candidate.id==added;});
   if(replacement==tx.uwb_groups.end())continue;
   for(auto measurement:replacement->source_measurements){auto row=std::find(g.source_measurements.begin(),g.source_measurements.end(),measurement);if(row!=g.source_measurements.end())retain.push_back(row-g.source_measurements.begin());}}
  if(!retain.empty()){std::sort(retain.begin(),retain.end());if(std::adjacent_find(retain.begin(),retain.end())!=retain.end())throw std::runtime_error("duplicate replacement source");s.appendBlock(query,g,first,s.epoch,&retain);}
 }
 return query.evaluate(tagMap(),alpha,tail,faults);
}
Eigen::MatrixXd OnlineGaussianHistory::tagMap()const{const auto& s=*impl;size_t first=s.epoch>s.lag+1?s.epoch-s.lag-1:0;Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,21*(s.epoch-first+1));
 auto pose=s.points.at<gtsam::Pose3>(gtsam::Symbol('x',s.epoch));Eigen::Matrix3d skew;skew<<0,-s.lever.z(),s.lever.y(),s.lever.z(),0,-s.lever.x(),-s.lever.y(),s.lever.x(),0;
 c.block<3,3>(0,c.cols()-21)=-pose.rotation().matrix()*skew;c.block<3,3>(0,c.cols()-18)=pose.rotation().matrix();return c;}
Eigen::Matrix<double,5,1> OnlineGaussianHistory::referenceSteps(const GaussianReference& r)const{
 Eigen::Matrix<double,5,1> maximum=Eigen::Matrix<double,5,1>::Zero();
 if(r.mean.size()!=tagMap().cols()){maximum.setConstant(INFINITY);return maximum;}
 auto accumulate=[&](const Eigen::VectorXd& nav){for(int i=0;i<5;++i)maximum(i)=std::max(maximum(i),nav.segment(3*i,3).norm());};
 for(int q=0;q<r.mean.size();q+=21)accumulate(r.mean.segment(q,15));
 for(const auto& old:impl->old_navigation)accumulate(old.offset+old.coefficients*r.mean.head(21));
 return maximum;
}
OnlineHistoryCounts OnlineGaussianHistory::counts()const{const auto& s=*impl;OnlineHistoryCounts c;size_t first=s.epoch>s.lag+1?s.epoch-s.lag-1:0;c.frontier_epoch=first;c.active_groups=s.groups.size();c.active_states=21*(s.epoch-first+1);c.history_rows=s.history.h.rows();c.fault_modes=s.modes.size();c.source_columns=s.local_noise.noise_j.cols();c.integrations=s.local_noise.integrations;c.bytes=s.history.bytes()+8*(s.local_noise.a.size()+s.local_noise.sigma.size()+s.local_noise.noise_j.size());c.orthogonal_updates=s.history.orthogonal_updates;c.original_rows=s.original_rows;c.eliminated_navigation_states=15*first;
 for(const auto& b:s.local_window.blocks)if(!s.retired_groups.count(b.group_id.value()))c.original_rows+=b.residual_raw.size();
 return c;}
void OnlineGaussianHistory::committed(const EpochTransaction& tx,const EpochCommitPlan& plan){auto& s=*impl;if(tx.id!=s.prepared_id)throw std::runtime_error("foreign online commit");
 for(auto id:plan.groups_to_remove){if(s.retired_groups.count(id.value()))throw std::runtime_error("UNAVAILABLE_RETIRED_COMMIT");s.groups.erase(id.value());}
 std::vector<const PendingFactorGroup*> catalog{&tx.imu_group};for(const auto& g:tx.uwb_groups)catalog.push_back(&g);for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)catalog.push_back(&g);
 for(auto id:plan.groups_to_add){auto g=std::find_if(catalog.begin(),catalog.end(),[&](const auto* p){return p->id==id;});if(g==catalog.end())throw std::runtime_error("UNAVAILABLE_ONLINE_COMMIT_PROVENANCE");s.groups[id.value()]=**g;}
 s.frontier=s.epoch>s.lag+1?s.epoch-s.lag-1:0;
 for(auto it=s.episodes.begin();it!=s.episodes.end();)if(it->first<=s.frontier)it=s.episodes.erase(it);else ++it;
 std::vector<gtsam::Key> erase;for(const auto& item:s.points)if(gtsam::Symbol(item.key).index()<s.frontier)erase.push_back(item.key);for(auto key:erase)s.points.erase(key);
 std::set<uint64_t> live;for(const auto& g:s.groups)for(auto id:g.second.source_measurements)live.insert(id.value());for(auto it=s.lineage.begin();it!=s.lineage.end();)if(!live.count(it->first))it=s.lineage.erase(it);else ++it;
}
}
