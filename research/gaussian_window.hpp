#pragma once
#include "gaussian_reference.hpp"
#include "raw_imu_episode.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"
#include <gtsam/inference/Symbol.h>
namespace uwb_imu_pl::research {
struct GaussianWindow {
 Eigen::MatrixXd a,sigma,c,noise_j,auxiliary_sigma;
 std::vector<std::string> noise_source_ids;
 Eigen::VectorXd z;
 std::vector<Eigen::MatrixXd> faults;
 std::vector<int> offsets;
 double cross_norm=0.;
 bool history_qualified=false; // only the independent finite-prefix path sets this
 size_t integrations=0,source_products=0;
 bool covariance_fallback=false;
};
inline Eigen::MatrixXd worldTagMap(const LinearizedIntegrityWindow& w,
 const EpochTransaction& tx,const Eigen::Vector3d& lever){
 auto c=w.protected_state_map;
 auto pose=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
 Eigen::Matrix3d skew;skew<<0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
 c.block<3,3>(0,w.H.cols()-15)=-pose.rotation().matrix()*skew;
 return c;
}
inline GaussianWindow rawGaussianWindow(const LinearizedIntegrityWindow& w,const EpochTransaction& tx,
    const std::vector<FaultModeBasis>& modes,double sa,double sg,const Eigen::Vector3d& lever,bool shared_gram=false) {
 GaussianWindow o;int n=0;for(const auto& b:w.blocks){o.offsets.push_back(n);n+=b.residual_raw.size();}
 o.a.resize(n,w.H.cols());o.z.resize(n);o.sigma=Eigen::MatrixXd::Zero(n,n);o.c=worldTagMap(w,tx,lever);
 // A shared sample owns ONE noise column, across all affected factors.
 std::map<long long,int> ids;std::map<FactorGroupId,EpochTransaction> materials;
 materials.emplace(tx.imu_group.id,tx);
 for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)
  if(g.kind==FactorKind::CombinedImu){auto t=historicalEpisodeTransaction(h);t.frozen_values=tx.frozen_values;materials.emplace(g.id,std::move(t));}
 for(const auto& b:w.blocks)if(b.kind==FactorKind::CombinedImu){auto it=materials.find(b.group_id);
  if(it==materials.end())throw std::runtime_error("missing raw covariance material");
  for(const auto& s:it->second.raw_imu_slice)if(!ids.count(s.timestamp.value())){int k=ids.size();ids.emplace(s.timestamp.value(),k);}}
 Eigen::MatrixXd j=Eigen::MatrixXd::Zero(n,6*ids.size());
 std::vector<std::vector<int>> sources(w.blocks.size());
 for(size_t k=0;k<w.blocks.size();++k){const auto& b=w.blocks[k];int off=o.offsets[k],m=b.residual_raw.size();
  o.a.middleRows(off,m)=b.jacobian_raw;o.z.segment(off,m)=b.residual_raw;
  o.sigma.block(off,off,m,m)=b.covariance;
  if(b.kind!=FactorKind::CombinedImu)continue;
  const auto& t=materials.at(b.group_id);
  for(const auto& sample:t.raw_imu_slice)for(int axis=0;axis<6;++axis)sources[k].push_back(6*ids.at(sample.timestamp.value())+axis);
  std::sort(sources[k].begin(),sources[k].end());
  // Reintegrate with ADC covariances removed. Only bias/process auxiliary
  // covariance remains: ADC uncertainty is counted exactly once via JQJ'.
  auto params=boost::make_shared<gtsam::PreintegrationCombinedParams>(*boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(t.preintegration->params()));
  params->accelerometerCovariance.setZero();params->gyroscopeCovariance.setZero();
  gtsam::PreintegratedCombinedMeasurements aux(params,t.preintegration->biasHat());
  for(size_t i=1;i<t.raw_imu_slice.size();++i){const auto& l=t.raw_imu_slice[i-1];const auto& r=t.raw_imu_slice[i];
   aux.integrateMeasurement(.5*(l.specific_force_mps2+r.specific_force_mps2),.5*(l.angular_velocity_radps+r.angular_velocity_radps),r.timestamp.seconds()-l.timestamp.seconds());}
  o.sigma.block(off,off,m,m)=aux.preintMeasCov();++o.integrations;
  for(const auto& s:t.raw_imu_slice)for(int axis=0;axis<6;++axis){
   RawImuEpisode e{"adc-sample-v1",s.timestamp,TimestampNs(s.timestamp.value()+1),axis};double eps=1e-5;
   Eigen::VectorXd d=-(episodeFactorError(t,integrateEpisode(t,e,eps))-episodeFactorError(t,integrateEpisode(t,e,-eps)))/(2*eps);
   j.block(off,6*ids.at(s.timestamp.value())+axis,m,1)=d*(axis<3?sa:sg);o.integrations+=2;}
 }
 o.auxiliary_sigma=o.sigma;
 o.noise_source_ids.resize(j.cols());for(const auto& id:ids)for(int axis=0;axis<6;++axis)
  o.noise_source_ids[6*id.second+axis]="adc:"+std::to_string(id.first)+":"+std::to_string(axis);
 Eigen::MatrixXd adc=Eigen::MatrixXd::Zero(n,n),roundoff=Eigen::MatrixXd::Zero(n,n);
 if(shared_gram){
  for(size_t i=0;i<w.blocks.size();++i)if(!sources[i].empty())for(size_t k=0;k<=i;++k)if(!sources[k].empty()){
   std::vector<int> common;std::set_intersection(sources[i].begin(),sources[i].end(),sources[k].begin(),sources[k].end(),std::back_inserter(common));
   if(common.empty())continue;
   int mi=w.blocks[i].residual_raw.size(),mk=w.blocks[k].residual_raw.size();Eigen::MatrixXd left(mi,common.size()),right(mk,common.size());
   for(size_t q=0;q<common.size();++q){left.col(q)=j.block(o.offsets[i],common[q],mi,1);right.col(q)=j.block(o.offsets[k],common[q],mk,1);}
   auto block=(left*right.transpose()).eval();double e=2*j.cols()*std::numeric_limits<double>::epsilon();
   auto error=(e/(1-e)*left.cwiseAbs()*right.cwiseAbs().transpose()).eval();
   adc.block(o.offsets[i],o.offsets[k],mi,mk)=block;roundoff.block(o.offsets[i],o.offsets[k],mi,mk)=error;
   if(k!=i){adc.block(o.offsets[k],o.offsets[i],mk,mi)=block.transpose();roundoff.block(o.offsets[k],o.offsets[i],mk,mi)=error.transpose();}
   ++o.source_products;
  }
  // Units-normalized Gershgorin witness; unresolved/near-PSD systems use the
  // original dense source Gram. Never jitter a covariance to make it pass.
  roundoff+=std::numeric_limits<double>::epsilon()*(o.sigma.cwiseAbs()+adc.cwiseAbs());
  auto trial=(o.sigma+adc).eval();auto diagonal=trial.diagonal().eval();bool safe=(diagonal.array()>0).all();
  if(safe){Eigen::VectorXd inv=diagonal.cwiseSqrt().cwiseInverse();
   auto corr=(inv.asDiagonal()*trial*inv.asDiagonal()).eval();auto err=(inv.asDiagonal()*roundoff*inv.asDiagonal()).eval();
   auto lower=(2*corr.diagonal()-corr.cwiseAbs().rowwise().sum()).eval();
   safe=lower.minCoeff()>64*err.rowwise().sum().maxCoeff()+1e-10;
  }
  if(!safe){adc=j*j.transpose();o.covariance_fallback=true;}
 }else {adc=j*j.transpose();o.source_products=1;}
 o.sigma+=adc;o.noise_j=std::move(j);
 for(size_t i=0;i<w.blocks.size();++i)for(size_t k=0;k<i;++k)
  o.cross_norm+=adc.block(o.offsets[i],o.offsets[k],w.blocks[i].residual_raw.size(),w.blocks[k].residual_raw.size()).squaredNorm();
 o.cross_norm=std::sqrt(o.cross_norm);
 for(const auto& mode:modes){Eigen::MatrixXd b=Eigen::MatrixXd::Zero(n,mode.parameter_dimension);
  for(size_t k=0;k<w.blocks.size();++k){auto it=mode.raw_group_maps.find(w.blocks[k].group_id);
   if(it!=mode.raw_group_maps.end())b.middleRows(o.offsets[k],it->second.rows())=it->second;}
  o.faults.push_back(std::move(b));}
 return o;
}
struct GaussianCandidateCache {
 const GaussianWindow* root;
 WindowId window;LinearizationVersion version;
 double alpha,tail;
 std::map<std::vector<int>,GaussianReference> entries;
 GaussianCandidateCache(const GaussianWindow& r,const LinearizedIntegrityWindow& w,double a,double t):root(&r),window(w.id),version(w.version),alpha(a),tail(t){}
};
inline GaussianReference gaussianCandidate(const GaussianWindow& o,const LinearizedIntegrityWindow& w,
    const ExclusionAction& action,double alpha,double tail,bool batched=false,
    const GaussianReference* base=nullptr,GaussianCandidateCache* cache=nullptr,size_t* decompositions=nullptr) {
 if(cache && (cache->root!=&o || cache->window!=w.id || !(cache->version==w.version) || cache->alpha!=alpha || cache->tail!=tail))
  throw std::runtime_error("foreign Gaussian candidate cache root");
 std::vector<int> rows;
 for(size_t k=0;k<w.blocks.size();++k)if(std::find(action.groups_to_remove.begin(),action.groups_to_remove.end(),w.blocks[k].group_id)==action.groups_to_remove.end())
  for(int j=0;j<w.blocks[k].residual_raw.size();++j)rows.push_back(o.offsets[k]+j);
 // Range replacement must be a literal raw-row subset. Never delete mixed
 // white rows or assume a bridge shares the covariance of the removed factor.
 for(const auto& b:action.added_blocks)for(int j=0;j<b.residual_raw.size();++j){int match=-1;
  for(size_t k=0;k<w.blocks.size();++k)if(std::find(action.groups_to_remove.begin(),action.groups_to_remove.end(),w.blocks[k].group_id)!=action.groups_to_remove.end())
   for(int q=0;q<w.blocks[k].residual_raw.size();++q){int r=o.offsets[k]+q;
    if((o.a.row(r)-b.jacobian_raw.row(j)).norm()<1e-9 && std::abs(o.z(r)-b.residual_raw(j))<1e-9){if(match!=-1)throw std::runtime_error("ambiguous replacement raw lineage");match=r;}}
  if(match<0)return GaussianReference{};
  if(std::find(rows.begin(),rows.end(),match)!=rows.end())throw std::runtime_error("duplicate candidate raw row");
  rows.push_back(match);
 }
 if(cache){auto it=cache->entries.find(rows);if(it!=cache->entries.end())return it->second;}
 bool keep=rows.size()==size_t(o.a.rows());for(size_t i=0;i<rows.size();++i)keep=keep && rows[i]==int(i);
 if(base && keep){auto result=gaussianFaultBounds(*base,o.c,o.faults,tail,batched,&o.a);if(cache)cache->entries.emplace(rows,result);return result;}
 Eigen::MatrixXd a(rows.size(),o.a.cols());Eigen::VectorXd z(rows.size());std::vector<Eigen::MatrixXd> faults;
 for(size_t i=0;i<rows.size();++i){a.row(i)=o.a.row(rows[i]);z(i)=o.z(rows[i]);}
 for(const auto& b:o.faults){Eigen::MatrixXd selected(rows.size(),b.cols());for(size_t i=0;i<rows.size();++i)selected.row(i)=b.row(rows[i]);faults.push_back(std::move(selected));}
 if(decompositions)++*decompositions;
 auto result=gaussianReference(a,z,principal(o.sigma,rows),o.c,faults,alpha,tail,batched);
 if(cache)cache->entries.emplace(rows,result);
 return result;
}
}
