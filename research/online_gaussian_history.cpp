#include "online_gaussian_history.hpp"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Ordering.h>
#include <set>
#include <tuple>
namespace uwb_imu_pl::research {
struct OnlineGaussianHistory::Impl {
 struct WhiteBlock {Eigen::MatrixXd a;Eigen::VectorXd z;std::map<std::string,Eigen::MatrixXd> b;};
 struct Mode {std::string id;AnchorId anchor;size_t onset;int duration;};
 bool optimized;size_t max_numeric_bytes;
 std::shared_ptr<const gtsam::Values> prepared_values;
 size_t lag,frontier=0,epoch=0;int duration;double sa,sg;Eigen::Vector3d lever;
 GaussianSufficientSummary history{21};
 std::map<uint64_t,PendingFactorGroup> groups;
 std::map<size_t,HistoricalEpochContext> episodes;
 std::map<uint64_t,std::pair<size_t,AnchorId>> lineage;
 std::set<uint64_t> retired_groups;
 std::map<uint64_t,std::pair<size_t,AnchorId>> retired_lineage;
 std::vector<Mode> modes;
 std::set<AnchorId> anchor_catalog;
 boost::shared_ptr<gtsam::PreintegrationCombinedParams> noise_law;
 std::vector<SummaryRetirement> old_navigation;
 gtsam::Values points;
 LinearizedIntegrityWindow local_window;
 GaussianWindow local_noise;
 EpochTransaction material;
 bool initialized=false;TransactionId prepared_id;
 bool prepared_active=false;
 std::shared_ptr<const GaussianSufficientSummary> prepared_query;
 mutable std::map<std::tuple<double,double,bool>,SummaryEvaluation> root_evaluations;
 mutable OnlineHistoryCounts work;
 mutable std::map<std::pair<double,double>,std::shared_ptr<const SummaryStateRoot>> state_roots;
 mutable std::map<std::tuple<std::vector<std::pair<uint64_t,int>>,double,double,bool>,SummaryEvaluation> candidate_results;
 mutable size_t state_decompositions=0,fault_products=0;
 LinearizationVersion prepared_version;
 size_t original_rows=0;
 Impl(size_t w,int d,double a,double g,Eigen::Vector3d l,bool opt,size_t budget):optimized(opt),max_numeric_bytes(budget),lag(w),duration(d),sa(a),sg(g),lever(l){
  if(w<2 || d<1 || !(a>0 && g>0))throw std::runtime_error("invalid online history model");
  history.optimized=optimized;
  Eigen::MatrixXd prior=Eigen::MatrixXd::Zero(6,21);prior.rightCols(6).setIdentity();history.append(prior,Eigen::VectorXd::Zero(6),{});
 }
 void requireResources(size_t n,size_t leaves,size_t roots,size_t results)const{
  // Conservative numeric workspace/cache allowance, not an RSS guarantee.
  // Every event stays declared; exceeding resources refuses the whole epoch.
  long double bytes=sizeof(double)*(8.L*n*n+4.L*n*leaves+2.L*21*21*(epoch+1)+roots*(7.L*n*n+8*n+8.L*leaves)+results*(2.L*n*n+8.L*leaves));
  if(leaves>4096 || n>1024 || bytes>max_numeric_bytes){work.resource_refused=true;throw std::runtime_error("UNAVAILABLE_SUMMARY_RESOURCES");}
 }
 void validateTransaction(const EpochTransaction& tx)const{
  if(tx.proposed_epoch!=epoch || tx.begin!=material.begin || tx.end!=material.end || tx.preintegration!=material.preintegration || tx.frozen_values!=prepared_values || tx.uwb_groups.size()!=material.uwb_groups.size())throw std::runtime_error("UNAVAILABLE_SUMMARY_TRANSACTION_BINDING");
  if(tx.base_graph_version!=material.base_graph_version || tx.ledger_version!=material.ledger_version || tx.previous_epoch!=material.previous_epoch || tx.raw_imu_slice.size()!=material.raw_imu_slice.size())throw std::runtime_error("UNAVAILABLE_SUMMARY_TRANSACTION_BINDING");
  if(!tx.preintegration->params()->equals(*noise_law,0.))throw std::runtime_error("UNAVAILABLE_NOISE_MODEL_CHANGE");
  for(size_t i=0;i<tx.raw_imu_slice.size();++i){const auto& a=tx.raw_imu_slice[i];const auto& b=material.raw_imu_slice[i];if(a.id!=b.id || a.timestamp!=b.timestamp || a.specific_force_mps2!=b.specific_force_mps2 || a.angular_velocity_radps!=b.angular_velocity_radps || a.quality_flags!=b.quality_flags)throw std::runtime_error("UNAVAILABLE_SUMMARY_RAW_BINDING");}
  auto group=[](const PendingFactorGroup& a,const PendingFactorGroup& b){
   if(a.id!=b.id || a.kind!=b.kind || a.sensor!=b.sensor || a.nominal!=b.nominal || a.keys!=b.keys || a.source_measurements!=b.source_measurements || a.raw_covariance.rows()!=b.raw_covariance.rows() || a.raw_covariance.cols()!=b.raw_covariance.cols() || !(a.raw_covariance.array()==b.raw_covariance.array()).all() || a.factors.size()!=b.factors.size())throw std::runtime_error("UNAVAILABLE_SUMMARY_SOURCE_BINDING");
   for(size_t j=0;j<a.factors.size();++j)if(a.factors[j]!=b.factors[j])throw std::runtime_error("UNAVAILABLE_SUMMARY_FACTOR_BINDING");
  };
  group(tx.imu_group,material.imu_group);for(size_t i=0;i<tx.uwb_groups.size();++i)group(tx.uwb_groups[i],material.uwb_groups[i]);
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
  if(optimized){
   // Temporal incidence is exact: scalar onset1/length3 leaves cover only
   // these raw UID rows. Missing entries are literal zero, not dropped leaves.
   for(size_t row=0;row<g.source_measurements.size();++row){auto source=lineage.find(g.source_measurements[row].value());if(source==lineage.end())throw std::runtime_error("UNAVAILABLE_UWB_SOURCE_LINEAGE");
    size_t e=source->second.first;for(int length:{1,duration})for(size_t onset=e>=size_t(length)?e-length+1:1;onset<=e;++onset){
     std::string id="prefix-anchor:"+std::to_string(source->second.second.value())+":onset:"+std::to_string(onset)+":duration:"+std::to_string(length);
     auto found=out.find(id);if(found==out.end())found=out.emplace(id,Eigen::MatrixXd::Zero(g.source_measurements.size(),1)).first;found->second(row)=1.;}
   }return out;
  }
  for(const auto& mode:modes){Eigen::MatrixXd b=Eigen::MatrixXd::Zero(g.source_measurements.size(),1);
   for(size_t i=0;i<g.source_measurements.size();++i){auto it=lineage.find(g.source_measurements[i].value());if(it==lineage.end())throw std::runtime_error("UNAVAILABLE_UWB_SOURCE_LINEAGE");
    if(it->second.second==mode.anchor && it->second.first>=mode.onset && it->second.first<mode.onset+mode.duration)b(i)=1.;}
   if(b.squaredNorm()>0)out.emplace(mode.id,std::move(b));}
  return out;
 }
 // Convert one raw block to independent conditional noise rows. Shared endpoint
 // ADC coordinates remain explicit columns, with a prior added ONCE per source.
 void appendBlock(GaussianSufficientSummary& target,const PendingFactorGroup& g,size_t first,size_t last,const std::vector<int>* keep=nullptr,std::vector<WhiteBlock>* batch=nullptr)const{
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
  if(!covariance.allFinite() || !covariance.isApprox(covariance.transpose(),1e-12))throw std::runtime_error("INDETERMINATE_CONDITIONAL_COVARIANCE");
  Eigen::LLT<Eigen::MatrixXd> chol(covariance);if(chol.info()!=Eigen::Success)throw std::runtime_error("INDETERMINATE_CONDITIONAL_COVARIANCE");
  for(auto& f:b)f.second=chol.matrixL().solve(f.second).eval();
  if(batch)batch->push_back({chol.matrixL().solve(a),chol.matrixL().solve(z),std::move(b)});
  else target.append(chol.matrixL().solve(a),chol.matrixL().solve(z),b);
 }
 void noisePrior(GaussianSufficientSummary& target,size_t first,size_t e,std::vector<WhiteBlock>* batch=nullptr)const{
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(6,target.states());a.block(0,21*(e-first)+15,6,6).setIdentity();
  if(batch)batch->push_back({std::move(a),Eigen::VectorXd::Zero(6),{}});else target.append(a,Eigen::VectorXd::Zero(6),{});
 }
 static void appendBatch(GaussianSufficientSummary& target,const std::vector<WhiteBlock>& batch){
  int rows=0;for(const auto& item:batch)rows+=item.a.rows();Eigen::MatrixXd a(rows,target.states());Eigen::VectorXd z(rows);
  std::map<std::string,Eigen::MatrixXd> b;
  for(const auto& item:batch)for(const auto& f:item.b)if(!b.count(f.first))b.emplace(f.first,Eigen::MatrixXd::Zero(rows,f.second.cols()));
  int off=0;for(const auto& item:batch){a.middleRows(off,item.a.rows())=item.a;z.segment(off,item.z.size())=item.z;
   for(const auto& f:item.b)b.at(f.first).middleRows(off,item.a.rows())=f.second;off+=item.a.rows();}
  target.append(a,z,b);
 }
 using RowKey=std::vector<std::pair<uint64_t,int>>;
 RowKey selectedRows(const EpochTransaction& tx,const ExclusionAction* action)const{
  size_t first=epoch>lag+1?epoch-lag-1:0;auto current=groups;current[tx.imu_group.id.value()]=tx.imu_group;for(const auto& g:tx.uwb_groups)if(g.nominal)current[g.id.value()]=g;
  if(action){if(action->bridge_mode!=BridgeMode::None)throw std::runtime_error("UNAVAILABLE_BRIDGE_SUMMARY_SCOPE");
   for(auto added:action->groups_to_add){auto replacement=std::find_if(tx.uwb_groups.begin(),tx.uwb_groups.end(),[&](const auto& g){return g.id==added;});
    if(replacement==tx.uwb_groups.end())throw std::runtime_error("UNAVAILABLE_ADDED_GROUP_PROVENANCE");
    std::vector<std::pair<const PendingFactorGroup*,int>> original;
    for(auto id:replacement->source_measurements){std::pair<const PendingFactorGroup*,int> match{nullptr,0};
     for(auto removed:action->groups_to_remove){auto group=current.find(removed.value());if(group==current.end())continue;auto row=std::find(group->second.source_measurements.begin(),group->second.source_measurements.end(),id);
      if(row!=group->second.source_measurements.end()){if(match.first)throw std::runtime_error("ambiguous replacement lineage");match={&group->second,int(row-group->second.source_measurements.begin())};}}
     if(!match.first)throw std::runtime_error("UNAVAILABLE_REPLACEMENT_LINEAGE");original.push_back(match);}
    if(replacement->raw_covariance.rows()!=int(original.size()))throw std::runtime_error("UNAVAILABLE_REPLACEMENT_NOISE_LAW");
    for(size_t i=0;i<original.size();++i)for(size_t j=0;j<original.size();++j){double expected=original[i].first==original[j].first?original[i].first->raw_covariance(original[i].second,original[j].second):0.;
     if(replacement->raw_covariance(i,j)!=expected)throw std::runtime_error("UNAVAILABLE_REPLACEMENT_NOISE_LAW");}
   }
   for(auto id:action->groups_to_remove)if(retired_groups.count(id.value()) || !current.count(id.value()))throw std::runtime_error("UNAVAILABLE_RETIRED_CANDIDATE_SOURCE");}
  RowKey key;
  for(const auto& item:current){const auto& g=item.second;if(endEpoch(g)<first || (first>0 && endEpoch(g)==first))continue;
   bool remove=action && std::find(action->groups_to_remove.begin(),action->groups_to_remove.end(),g.id)!=action->groups_to_remove.end();
   std::vector<int> retain;
   if(remove){for(auto added:action->groups_to_add){auto replacement=std::find_if(tx.uwb_groups.begin(),tx.uwb_groups.end(),[&](const auto& candidate){return candidate.id==added;});
    for(auto measurement:replacement->source_measurements){auto row=std::find(g.source_measurements.begin(),g.source_measurements.end(),measurement);if(row!=g.source_measurements.end())retain.push_back(row-g.source_measurements.begin());}}
    std::sort(retain.begin(),retain.end());if(std::adjacent_find(retain.begin(),retain.end())!=retain.end())throw std::runtime_error("duplicate replacement source");
   }else{auto block=std::find_if(local_window.blocks.begin(),local_window.blocks.end(),[&](const auto& value){return value.group_id==g.id;});if(block==local_window.blocks.end())throw std::runtime_error("UNAVAILABLE_REPLACEMENT_BLOCK");for(int row=0;row<block->residual_raw.size();++row)retain.push_back(row);}
   for(int row:retain)key.emplace_back(item.first,row);
  }return key;
 }
 GaussianSufficientSummary query(const EpochTransaction& tx,const ExclusionAction* action)const{
  size_t first=epoch>lag+1?epoch-lag-1:0;GaussianSufficientSummary result=history;result.append_state_ms=0.;result.append_fault_ms=0.;result.full_fault_transforms=0;result.frontier_fault_transforms=0;result.frontier_kernels=0;result.addStateColumns(21*(epoch-first));std::vector<WhiteBlock> batch;
  for(size_t e=first+1;e<=epoch;++e)noisePrior(result,first,e,&batch);
  auto current=groups;current[tx.imu_group.id.value()]=tx.imu_group;for(const auto& g:tx.uwb_groups)if(g.nominal)current[g.id.value()]=g;
  selectedRows(tx,action);
  for(const auto& item:current){const auto& g=item.second;if(endEpoch(g)<first || (first>0 && endEpoch(g)==first))continue;
   bool remove=action && std::find(action->groups_to_remove.begin(),action->groups_to_remove.end(),g.id)!=action->groups_to_remove.end();
   if(!remove){appendBlock(result,g,first,epoch,nullptr,&batch);continue;}
   std::vector<int> retain;
   for(auto added:action->groups_to_add){auto replacement=std::find_if(tx.uwb_groups.begin(),tx.uwb_groups.end(),[&](const auto& candidate){return candidate.id==added;});
    for(auto measurement:replacement->source_measurements){auto row=std::find(g.source_measurements.begin(),g.source_measurements.end(),measurement);if(row!=g.source_measurements.end())retain.push_back(row-g.source_measurements.begin());}}
   if(!retain.empty()){std::sort(retain.begin(),retain.end());if(std::adjacent_find(retain.begin(),retain.end())!=retain.end())throw std::runtime_error("duplicate replacement source");appendBlock(result,g,first,epoch,&retain,&batch);}
  }
  appendBatch(result,batch);++work.queries;work.query_columns+=modes.size();work.query_state_qr_ms+=result.append_state_ms;work.query_fault_qr_ms+=result.append_fault_ms;work.full_fault_transforms+=result.full_fault_transforms;work.frontier_fault_transforms+=result.frontier_fault_transforms;work.frontier_kernels+=result.frontier_kernels;return result;
 }
};
OnlineGaussianHistory::OnlineGaussianHistory(size_t lag,int duration,double sa,double sg,Eigen::Vector3d lever,bool optimized,size_t max_numeric_bytes):impl(new Impl(lag,duration,sa,sg,lever,optimized,max_numeric_bytes)){}
OnlineGaussianHistory::~OnlineGaussianHistory()=default;
void OnlineGaussianHistory::prepare(const EpochTransaction& tx,bool build_current_root){
 if(!tx.frozen_values)throw std::runtime_error("UNAVAILABLE_HISTORY_FROZEN_VALUES");
 auto& s=*impl;auto timing_begin=std::chrono::steady_clock::now();s.work={};s.prepared_active=false;
 size_t future_states=21*(std::min(tx.proposed_epoch,s.lag+1)+1);std::set<AnchorId> future_anchors;for(const auto& m:tx.uwb_batch.measurements)future_anchors.insert(m.anchor_id);s.requireResources(future_states,s.modes.size()+2*future_anchors.size(),0,0);
 if(tx.proposed_epoch!=s.epoch+1 || tx.proposed_epoch>50)throw std::runtime_error("UNAVAILABLE_HISTORY_SEQUENCE_OR_HORIZON");
 if(!tx.preintegration)throw std::runtime_error("UNAVAILABLE_PIM_NOISE_MODEL");
 auto params=boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(tx.preintegration->params());
 if(!params)throw std::runtime_error("UNAVAILABLE_PIM_NOISE_MODEL");
 if(!s.noise_law)s.noise_law=boost::make_shared<gtsam::PreintegrationCombinedParams>(*params);
 else if(!params->equals(*s.noise_law,0.))throw std::runtime_error("UNAVAILABLE_NOISE_MODEL_CHANGE");
 if(tx.begin.value()!=50000000LL*(tx.proposed_epoch-1) || tx.end.value()!=50000000LL*tx.proposed_epoch || tx.raw_imu_slice.size()!=11)throw std::runtime_error("UNAVAILABLE_NOISE_SUPPORT_MODEL");
 for(size_t i=0;i<tx.raw_imu_slice.size();++i)if(tx.raw_imu_slice[i].timestamp.value()!=tx.begin.value()+5000000LL*i)throw std::runtime_error("UNAVAILABLE_NOISE_SUPPORT_MODEL");
 if(s.epoch>0){const auto& previous=s.episodes.at(s.epoch).raw_imu_slice.back();const auto& current=tx.raw_imu_slice.front();
  if(previous.id!=current.id || previous.specific_force_mps2!=current.specific_force_mps2 || previous.angular_velocity_radps!=current.angular_velocity_radps)throw std::runtime_error("UNAVAILABLE_SHARED_SAMPLE_IDENTITY");}
 s.epoch=tx.proposed_epoch;s.prepared_id=tx.id;s.prepared_values=tx.frozen_values;s.prepared_version=tx.base_version;s.prepared_active=true;s.prepared_query.reset();s.root_evaluations.clear();s.state_roots.clear();s.candidate_results.clear();s.state_decompositions=0;s.fault_products=0;
 size_t changing=s.epoch>s.lag?s.epoch-s.lag:0;
 for(const auto& item:*tx.frozen_values){if(gtsam::Symbol(item.key).index()<s.frontier)continue;
  if(!s.points.exists(item.key))s.points.insert(item.key,item.value);else if(gtsam::Symbol(item.key).index()>=changing)s.points.update(item.key,item.value);}
 if(!s.initialized){for(const auto& f:tx.frozen_slots)if(f.factor){auto id=f.group_id.value_or(FactorGroupId(f.slot));auto& g=s.groups[id.value()];g.id=id;g.kind=FactorKind::BoundaryPrior;g.sensor=SensorType::Prior;g.factors.push_back(f.factor);
   for(auto key:f.factor->keys())if(std::find(g.keys.begin(),g.keys.end(),key)==g.keys.end())g.keys.push_back(key);}
  s.initialized=true;}
 HistoricalEpochContext episode;episode.previous_epoch=tx.previous_epoch;episode.proposed_epoch=tx.proposed_epoch;episode.begin=tx.begin;episode.end=tx.end;episode.raw_imu_slice=tx.raw_imu_slice;episode.preintegration=tx.preintegration;episode.groups={tx.imu_group};episode.uwb_batch=tx.uwb_batch;s.episodes[s.epoch]=episode;
 std::set<AnchorId> anchors;for(const auto& m:tx.uwb_batch.measurements){s.lineage[m.id.value()]={s.epoch,m.anchor_id};anchors.insert(m.anchor_id);}
 if(s.anchor_catalog.empty())s.anchor_catalog=anchors;else if(anchors!=s.anchor_catalog)throw std::runtime_error("UNAVAILABLE_FAULT_TOPOLOGY_CHANGE");
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
 if(s.epoch==2){Eigen::LLT<Eigen::MatrixXd> chol(s.local_noise.sigma);if(chol.info()!=Eigen::Success || s.local_noise.cross_norm<=1e-12)throw std::runtime_error("joint ADC covariance check failed");
  auto w=chol.matrixL().solve(Eigen::MatrixXd::Identity(s.local_noise.sigma.rows(),s.local_noise.sigma.rows())).eval();
  if((w*s.local_noise.sigma*w.transpose()-Eigen::MatrixXd::Identity(w.rows(),w.rows())).norm()>1e-7)throw std::runtime_error("joint whitening check failed");}
 auto retire_begin=std::chrono::steady_clock::now();s.work.source_ms=std::chrono::duration<double,std::milli>(retire_begin-timing_begin).count();
 size_t next=s.epoch>s.lag+1?s.epoch-s.lag-1:0;
 // local rows use the old frontier column origin until all retirement completes.
 const size_t old_frontier=s.frontier;
 if(next>s.frontier){s.history.addStateColumns(21);s.noisePrior(s.history,s.frontier,next);
  for(const auto& g:current)if(Impl::endEpoch(g.second)<=next){s.appendBlock(s.history,g.second,s.frontier,next);s.retired_groups.insert(g.first);if(g.second.sensor==SensorType::Uwb)for(auto id:g.second.source_measurements)s.retired_lineage.emplace(id.value(),s.lineage.at(id.value()));s.groups.erase(g.first);
   auto block=std::find_if(s.local_window.blocks.begin(),s.local_window.blocks.end(),[&](const auto& b){return b.group_id==g.second.id;});s.original_rows+=block->residual_raw.size();}
  auto recovery=s.history.retire(21);
  s.old_navigation.push_back(std::move(recovery));
 }
 // Leave the local matrix origin immutable for candidate evaluation; the
 // effective history frontier is updated only through the bounded suffix map.
 s.frontier=old_frontier;
 auto query_begin=std::chrono::steady_clock::now();s.work.retire_ms=std::chrono::duration<double,std::milli>(query_begin-retire_begin).count();
 if(build_current_root)s.prepared_query=std::make_shared<const GaussianSufficientSummary>(s.query(tx,nullptr));
 s.work.root_build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-query_begin).count();
 // Optional census of exact original scalar row supports, not Gram-based
 // equivalence and never a physical-event/risk merge. Excluded sources stay out.
 if(std::getenv("FDE_FAULT_COST")){
  std::set<uint64_t> sources;for(const auto& row:s.retired_lineage)sources.insert(row.first);for(const auto& item:current)if(item.second.sensor==SensorType::Uwb)for(auto id:item.second.source_measurements)sources.insert(id.value());
  std::map<std::pair<size_t,AnchorId>,std::vector<uint64_t>> rows;
  for(auto id:sources){auto source=s.retired_lineage.count(id)?s.retired_lineage.at(id):s.lineage.at(id);rows[source].push_back(id);}s.work.raw_rank=rows.size();
  std::set<std::vector<uint64_t>> signatures;
  for(const auto& mode:s.modes){std::vector<uint64_t> ids;for(size_t e=mode.onset;e<=s.epoch && e<mode.onset+mode.duration;++e){auto it=rows.find({e,mode.anchor});if(it!=rows.end())ids.insert(ids.end(),it->second.begin(),it->second.end());}
   std::sort(ids.begin(),ids.end());if(ids.empty())++s.work.zero_raw_columns;if(!signatures.insert(ids).second)++s.work.equal_raw_columns;}
 }

}
SummaryEvaluation OnlineGaussianHistory::evaluate(const EpochTransaction& tx,const ExclusionAction* action,double alpha,double tail,bool faults)const{
 const auto& s=*impl;if(!s.prepared_active || tx.id!=s.prepared_id || !(tx.base_version==s.prepared_version))throw std::runtime_error("foreign online summary root");
 s.validateTransaction(tx);if(!s.prepared_query)throw std::runtime_error("UNAVAILABLE_CURRENT_ROOT_NOT_REQUESTED");if(s.work.resource_refused)throw std::runtime_error("UNAVAILABLE_SUMMARY_RESOURCES");
 if(!(alpha>0 && alpha<1 && tail>0 && tail<1))throw std::runtime_error("INDETERMINATE_RISK_ARGUMENTS");
 bool keep=!action || (action->groups_to_remove.empty() && action->groups_to_add.empty() && action->bridge_mode==BridgeMode::None);
 auto record=[&](const SummaryEvaluation& result){s.work.state_ms+=result.state_ms;s.work.fault_ms+=result.fault_ms;s.work.leaf_svds+=result.leaf_svds;for(const auto& leaf:result.faults)if(leaf.second.classification=="MONITORABLE")++s.work.accepted_leaf_rank;};
 if(keep){auto key=std::make_tuple(alpha,tail,faults);auto it=s.root_evaluations.find(key);if(it!=s.root_evaluations.end())return it->second;
  const SummaryStateRoot* state=nullptr;
  if(s.optimized){auto risk=std::make_pair(alpha,tail);auto root=s.state_roots.find(risk);if(root==s.state_roots.end()){
    s.requireResources(s.prepared_query->states(),s.modes.size(),s.state_roots.size()+1,s.candidate_results.size());
    auto created=s.prepared_query->freezeState(tagMap(),alpha,tail);s.work.state_ms+=created->stateMilliseconds();++s.state_decompositions;root=s.state_roots.emplace(risk,std::move(created)).first;}state=root->second.get();
  }else ++s.state_decompositions;
  if(faults)s.fault_products+=s.modes.size();
  auto result=s.prepared_query->evaluate(tagMap(),alpha,tail,faults,state);record(result);s.root_evaluations.emplace(key,result);return result;
 }
 // Validate the complete source/covariance replacement EVERY time, including
 // cache hits. The key shares numbers, never action identity or proof leases.
 auto rows=s.selectedRows(tx,action);auto key=std::make_tuple(rows,alpha,tail,faults);
 if(s.optimized){auto found=s.candidate_results.find(key);if(found!=s.candidate_results.end()){++s.work.candidate_hits;return found->second;}}
 s.requireResources(s.prepared_query->states(),s.modes.size(),s.state_roots.size(),s.candidate_results.size()+1);
 auto query_begin=std::chrono::steady_clock::now();auto query=s.query(tx,action);s.work.candidate_build_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-query_begin).count();++s.state_decompositions;if(faults)s.fault_products+=s.modes.size();
 auto result=query.evaluate(tagMap(),alpha,tail,faults);record(result);if(s.optimized)s.candidate_results.emplace(std::move(key),result);return result;
}

Eigen::MatrixXd OnlineGaussianHistory::tagMap()const{const auto& s=*impl;size_t first=s.epoch>s.lag+1?s.epoch-s.lag-1:0;Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,21*(s.epoch-first+1));
 auto pose=s.points.at<gtsam::Pose3>(gtsam::Symbol('x',s.epoch));Eigen::Matrix3d skew;skew<<0,-s.lever.z(),s.lever.y(),s.lever.z(),0,-s.lever.x(),-s.lever.y(),s.lever.x(),0;
 c.block<3,3>(0,c.cols()-21)=-pose.rotation().matrix()*skew;c.block<3,3>(0,c.cols()-18)=pose.rotation().matrix();return c;}
Eigen::Matrix<double,5,1> OnlineGaussianHistory::referenceSteps(const GaussianReference& r)const{
 Eigen::Matrix<double,5,1> maximum=Eigen::Matrix<double,5,1>::Zero();
 if(r.mean.size()!=tagMap().cols()){maximum.setConstant(INFINITY);return maximum;}
 auto nav=navigationMean(r);for(int q=0;q<nav.size();q+=15)for(int i=0;i<5;++i)maximum(i)=std::max(maximum(i),nav.segment(q+3*i,3).norm());
 return maximum;
}
Eigen::VectorXd OnlineGaussianHistory::navigationMean(const GaussianReference& r)const{
 const auto& s=*impl;if(r.mean.size()!=tagMap().cols())throw std::runtime_error("foreign smoothing root shape");
 Eigen::VectorXd nav(15*(s.epoch+1)),next=r.mean.head(21);size_t first=s.old_navigation.size();
 for(size_t k=0;k<r.mean.size()/21;++k)nav.segment(15*(first+k),15)=r.mean.segment(21*k,15);
 for(size_t k=first;k-->0;){const auto& old=s.old_navigation[k];next=(old.offset+old.coefficients*next).eval();nav.segment(15*k,15)=next.head(15);}
 return nav;
}
Eigen::MatrixXd OnlineGaussianHistory::navigationCovariance(const GaussianReference& r)const{
 const auto& s=*impl;if(r.covariance.rows()!=tagMap().cols())throw std::runtime_error("foreign smoothing covariance shape");
 Eigen::MatrixXd covariance=r.covariance;
 for(size_t k=s.old_navigation.size();k-->0;){const auto& old=s.old_navigation[k];int n=covariance.rows();Eigen::MatrixXd extended(n+21,n+21);
  extended.topLeftCorner(21,21)=old.conditional_covariance+old.coefficients*covariance.topLeftCorner(21,21)*old.coefficients.transpose();
  extended.topRightCorner(21,n)=old.coefficients*covariance.topRows(21);extended.bottomLeftCorner(n,21)=extended.topRightCorner(21,n).transpose();extended.bottomRightCorner(n,n)=covariance;covariance=std::move(extended);}
 Eigen::MatrixXd nav(15*(s.epoch+1),15*(s.epoch+1));for(size_t i=0;i<=s.epoch;++i)for(size_t j=0;j<=s.epoch;++j)nav.block(15*i,15*j,15,15)=covariance.block(21*i,21*j,15,15);return nav;
}
OnlineHistoryCounts OnlineGaussianHistory::counts()const{const auto& s=*impl;OnlineHistoryCounts c=s.work;size_t first=s.epoch>s.lag+1?s.epoch-s.lag-1:0;c.frontier_epoch=first;c.active_groups=s.groups.size();c.active_states=21*(s.epoch-first+1);c.history_rows=s.history.h.rows();c.fault_modes=s.modes.size();for(const auto& mode:s.modes)if(std::min(s.epoch,mode.onset+mode.duration-1)>first)++c.active_modes;c.source_columns=s.local_noise.noise_j.cols();c.integrations=s.local_noise.integrations;c.bytes=s.history.bytes()+8*(s.local_noise.a.size()+s.local_noise.sigma.size()+s.local_noise.noise_j.size());if(s.prepared_query)c.bytes+=s.prepared_query->bytes();
 for(const auto& old:s.old_navigation)c.bytes+=8*(old.coefficients.size()+old.offset.size()+old.conditional_covariance.size());
 for(const auto& root:s.state_roots)c.bytes+=root.second->bytes();
 for(const auto& cached:s.candidate_results){const auto& r=cached.second.reference;c.bytes+=8*(r.mean.size()+r.covariance.size()+r.gain.size()+r.parity.size());}
 for(const auto& cached:s.root_evaluations){const auto& r=cached.second.reference;c.bytes+=8*(r.mean.size()+r.covariance.size()+r.gain.size()+r.parity.size());}
 c.state_decompositions=s.state_decompositions;c.fault_products=s.fault_products;
 c.orthogonal_updates=s.history.orthogonal_updates;c.original_rows=s.original_rows;c.eliminated_navigation_states=15*first;
 for(const auto& b:s.local_window.blocks)if(!s.retired_groups.count(b.group_id.value()))c.original_rows+=b.residual_raw.size();
 return c;}
OnlinePreUwbEvidence OnlineGaussianHistory::preUwbEvidence(const EpochTransaction& tx,double radius)const{
 const auto& s=*impl;OnlinePreUwbEvidence result;
 if(!s.prepared_active || tx.id!=s.prepared_id || !(radius>0))throw std::runtime_error("foreign history extraction");
 s.validateTransaction(tx);
 ExclusionAction removal;for(const auto& g:tx.uwb_groups)if(g.nominal)removal.groups_to_remove.push_back(g.id);
 auto summary=s.query(tx,&removal);
 auto& h=result.history;h.epoch=tx.previous_epoch;h.generation=tx.base_graph_version;
 h.horizon=50;h.radius=radius;h.risk=NAN;h.model="raw-frozen-prefix-with-shared-adc-v1";
 result.transaction=tx.id;result.excluded_batch=tx.uwb_batch.id;result.version=tx.base_version;
 auto pose=s.points.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
 result.chart.timestamp=tx.end;result.chart.q_world_body=Eigen::Quaterniond(pose.rotation().matrix());result.chart.position_world_m=pose.translation();
 result.chart.velocity_world_mps=s.points.at<gtsam::Vector3>(gtsam::Symbol('v',tx.proposed_epoch));
 auto bias=s.points.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.proposed_epoch));result.chart.accel_bias_mps2=bias.accelerometer();result.chart.gyro_bias_radps=bias.gyroscope();
 ++s.state_decompositions;Eigen::JacobiSVD<Eigen::MatrixXd> svd(summary.h,Eigen::ComputeFullU|Eigen::ComputeThinV);
 const auto singular=svd.singularValues();int n=summary.states();
 if(singular.size()!=n || singular.minCoeff()<1e-9*singular.maxCoeff()){h.status="INDETERMINATE";return result;}
 Eigen::MatrixXd gain=svd.matrixV()*singular.cwiseInverse().asDiagonal()*svd.matrixU().leftCols(n).transpose();
 Eigen::MatrixXd boundary=gain.bottomRows(21).topRows(15);
 result.correction=boundary*summary.d;h.mean=result.correction;h.covariance=boundary*boundary.transpose();
 Eigen::MatrixXd parity=svd.matrixU().rightCols(summary.h.rows()-n).transpose();
 Eigen::VectorXd residual=parity*summary.d;
 for(const auto& item:summary.faults){const auto& f=item.second;if(!f.ever_nonzero)continue;
  Eigen::VectorXd direction=-boundary*f.top.col(0);
  if((direction.array()==0.).all())continue;
  Eigen::VectorXd response(parity.rows()+f.residual.rows()),observed(residual.size()+f.residual.rows());
  response.head(parity.rows())=parity*f.top.col(0);response.tail(f.residual.rows())=f.residual.col(0);
  observed.head(residual.size())=residual;observed.tail(f.residual.rows())=f.residual.col(1);
  double norm=response.norm();if(norm<1e-8){h.status="INDETERMINATE";return result;}
  double estimate=response.dot(observed)/(norm*norm),half=radius/norm;
  h.segments.push_back({item.first,direction,estimate-half,estimate+half});
 }
 for(const auto& item:s.groups)if(item.second.sensor==SensorType::Uwb)for(auto id:item.second.source_measurements)h.retained_sources.push_back(id.value());
 for(const auto& item:s.retired_lineage)h.retained_sources.push_back(item.first);
 h.status="BOUNDED";return result;
}
void OnlineGaussianHistory::committed(const EpochTransaction& tx,const EpochCommitPlan& plan){auto& s=*impl;if(!s.prepared_active || tx.id!=s.prepared_id)throw std::runtime_error("foreign online commit");
 for(auto id:plan.groups_to_remove){if(s.retired_groups.count(id.value()))throw std::runtime_error("UNAVAILABLE_RETIRED_COMMIT");s.groups.erase(id.value());}
 std::vector<const PendingFactorGroup*> catalog{&tx.imu_group};for(const auto& g:tx.uwb_groups)catalog.push_back(&g);for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)catalog.push_back(&g);
 for(auto id:plan.groups_to_add){auto g=std::find_if(catalog.begin(),catalog.end(),[&](const auto* p){return p->id==id;});if(g==catalog.end())throw std::runtime_error("UNAVAILABLE_ONLINE_COMMIT_PROVENANCE");s.groups[id.value()]=**g;}
 s.prepared_active=false;
 s.frontier=s.epoch>s.lag+1?s.epoch-s.lag-1:0;
 for(auto it=s.episodes.begin();it!=s.episodes.end();)if(it->first<=s.frontier)it=s.episodes.erase(it);else ++it;
 std::vector<gtsam::Key> erase;for(const auto& item:s.points)if(gtsam::Symbol(item.key).index()<s.frontier)erase.push_back(item.key);for(auto key:erase)s.points.erase(key);
 std::set<uint64_t> live;for(const auto& g:s.groups)for(auto id:g.second.source_measurements)live.insert(id.value());for(auto it=s.lineage.begin();it!=s.lineage.end();)if(!live.count(it->first))it=s.lineage.erase(it);else ++it;
}
}
