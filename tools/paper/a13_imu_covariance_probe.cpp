// A13 read-only PIM audit. A12 restoration helpers/graph path reused verbatim.
// No optimizer construction, navigation solve, or iterate entry point.
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <array>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include "uifgo/config.h"
#include "uifgo/hash_utils.h"
#include "uifgo/graph_builder.h"
#include "uifgo/initializer.h"
#include "uifgo/imu_preint.h"
#include "uifgo/paper_input.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/t07_scenario_cache.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/uwb_factor.h"
using namespace gtsam;
using Row = std::map<std::string,std::string>;
std::vector<std::string> split(std::string line) {
  if (!line.empty() && line.back()=='\r') line.pop_back();
  std::vector<std::string> out; std::stringstream s(line); std::string v;
  while(std::getline(s,v,',')) out.push_back(v);
  if(!line.empty() && line.back()==',') out.emplace_back(); return out;
}
std::vector<Row> read(const std::string& path) {
  std::ifstream f(path); if(!f) throw std::runtime_error("missing "+path);
  std::string line; std::getline(f,line); auto h=split(line); std::vector<Row> rows;
  while(std::getline(f,line)) {auto v=split(line); if(v.size()!=h.size())throw std::runtime_error("CSV columns "+path); Row r; for(size_t i=0;i<h.size();++i)r[h[i]]=v[i]; rows.push_back(r);} return rows;
}
std::ofstream output(const std::string& path) {std::ofstream f(path); if(!f)throw std::runtime_error("cannot write "+path);f<<std::setprecision(17);return f;}
Values loadValues(const std::string& path,const Values& initial) {
 struct Entry {std::string type; std::map<std::string,double> c;};std::map<Key,Entry> parsed;
 for(auto&r:read(path)){Key k=std::stoull(r.at("key_value"));auto&e=parsed[k];if(!e.type.empty()&&e.type!=r.at("value_type"))throw std::runtime_error("type collision");e.type=r.at("value_type");if(!e.c.emplace(r.at("coordinate"),std::stod(r.at("value"))).second)throw std::runtime_error("duplicate value coordinate");}
 Values out;
 for(auto&kv:parsed){auto k=kv.first;auto&e=kv.second;if(!initial.exists(k))throw std::runtime_error("unexpected key");
 if(e.type=="Pose3" && e.c.size()==16){Matrix4 m;for(int i=0;i<4;++i)for(int j=0;j<4;++j)m(i,j)=e.c.at("m"+std::to_string(i)+std::to_string(j));out.insert(k,Pose3(m));}
 else if(e.type=="Vector3" && e.c.size()==3){Vector3 v;for(int i=0;i<3;++i)v[i]=e.c.at("v"+std::to_string(i));out.insert(k,v);}
 else if(e.type=="ConstantBias" && e.c.size()==6){Vector3 a,g;for(int i=0;i<3;++i){a[i]=e.c.at("accel"+std::to_string(i));g[i]=e.c.at("gyro"+std::to_string(i));}out.insert(k,imuBias::ConstantBias(a,g));}
 else throw std::runtime_error("incomplete/unsupported Values");}
 if(out.keys()!=initial.keys())throw std::runtime_error("key mismatch");
 for(auto k:out.keys())if(typeid(out.at(k))!=typeid(initial.at(k))||out.at(k).dim()!=initial.at(k).dim())throw std::runtime_error("type/dim mismatch");
 return out;
}
VectorValues loadDelta(const std::string& path,const Values& initial) {
 auto d=initial.zeroVectors();std::map<std::pair<Key,int>,bool> seen;
 for(auto&r:read(path)){if(r.at("call_index")!="50"||r.at("trial_index_within_call")!="2")throw std::runtime_error("wrong delta selection");Key k=std::stoull(r.at("key_value"));int j=std::stoi(r.at("coordinate"));if(!seen.emplace(std::make_pair(k,j),true).second||j<0||j>=d.at(k).size())throw std::runtime_error("delta duplicate/dim");d.at(k)[j]=std::stod(r.at("value"));}
 if(seen.size()!=615)throw std::runtime_error("incomplete actual delta");return d;
}
VectorValues scale(VectorValues v,double s){for(auto&kv:v)kv.second*=s;return v;}
double dot(const VectorValues& a,const VectorValues& b){double out=0;for(auto&kv:a)out+=kv.second.dot(b.at(kv.first));return out;}
double maxabs(const VectorValues& a){double out=0;for(auto&kv:a)out=std::max(out,kv.second.cwiseAbs().maxCoeff());return out;}
std::string keys(const NonlinearFactor& f){std::string s;for(auto k:f.keys()){if(!s.empty())s+=';';s+=DefaultKeyFormatter(k)+":"+std::to_string(k);}return s;}
void matrix(const std::string& path,const Matrix& m){auto f=output(path);for(int i=0;i<m.rows();++i){for(int j=0;j<m.cols();++j){if(j)f<<',';f<<m(i,j);}f<<'\n';}}
void station(std::ostream& f,const uifgo::NavigationStationarityAudit& a){f<<a.valid<<','<<a.stationary<<','<<a.max_pose_rotation_gradient_objective_per_rad<<','<<a.max_pose_translation_gradient_objective_per_m<<','<<a.max_velocity_gradient_objective_per_mps<<','<<a.max_accel_bias_gradient_objective_per_mps2<<','<<a.max_gyro_bias_gradient_objective_per_radps<<','<<a.max_scaled_gradient_objective<<','<<a.roundoff_allowance_objective;}

using M15=Eigen::Matrix<double,15,15>;
void dump(std::ostream& f,size_t index,const std::string& name,const Matrix& m) {
  for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)
    f<<index<<','<<name<<','<<m.rows()<<','<<m.cols()<<','<<i<<','<<j<<','<<m(i,j)<<'\n';
}
int group(Key k,int j){char c=Symbol(k).chr();return c=='x'?(j<3?0:1):(c=='v'?2:(j<3?3:4));}
bool near(double a,double b,double at=2e-12,double rt=2e-12){return std::abs(a-b)<=at+rt*std::abs(b);}

int main(int argc,char**argv) {try {
 if(argc!=4)throw std::runtime_error("usage: CONFIG A13_REFERENCE OUTPUT");
 std::string config=argv[1],ref=argv[2],out=argv[3];
 auto started=std::chrono::steady_clock::now(); auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
 {std::ifstream maps("/proc/self/maps");auto f=output(out+"/process_maps.txt");f<<maps.rdbuf();}
 auto cfg=uifgo::ConfigLoader::Load(config);
 auto cache=uifgo::LoadT07ScenarioCache(cfg.t07_cache_manifest,cfg.t07_cache_start_s,cfg.t07_cache_duration_s);
 auto plan=uifgo::BuildPaperInputPlan(cache.uwb,cfg,cache.base_recording_id);
 auto frames=uifgo::MaterializePaperKeyframes(plan,uifgo::AllPlannedObservationMask(plan));
 auto init=uifgo::Initializer(cfg).Run(cache.imu,frames);if(!init.ok)throw std::runtime_error("raw initializer failed");
 NonlinearFactorGraph graph;Values initial;std::vector<size_t> uwb;
 uifgo::GraphBuilder(cfg).Build(frames,cache.imu,init,&graph,&initial,&uwb);
 if(uifgo::ReplacePosePriorsForPaperPath(&graph)!=1)throw std::runtime_error("prior count");
 // Exact first conditional replacement, c=0 before any chain update.
 size_t n=0;for(auto&r:plan.observations)if(r.planned){auto anchor=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto&a){return a.id==r.anchor_id;});if(anchor==cfg.anchors.end())throw std::runtime_error("anchor");graph.at(uwb.at(n++))=uifgo::MakeUwbFactor(Symbol('x',r.keyframe_id),0,0,0,anchor->pos,cfg.lever_arm_init,r.raw_range,r.nominal_sigma,false,false,false,uifgo::FixedBetaForLink(cfg,r.tag_id,r.anchor_id)+0.0);}
 uifgo::InferenceIdentityContext context; context.input_sha256=cache.cache_id; context.config_sha256="sha256:"+uifgo::Sha256FileHex(config); context.input_plan_sha256=plan.plan_sha256; context.support_partition_sha256="t09-common-no-support:"+uifgo::Sha256Hex(plan.plan_sha256);
 auto identity=uifgo::ComputeInferenceContentIdentity(graph,initial,context);
 auto ident=output(out+"/restoration_identity.csv");ident<<"name,value\ninitial_graph_sha,"<<identity.graph_linearization_sha256<<"\ninitial_values_sha,"<<identity.values_sha256<<"\nkeys,"<<initial.size()<<"\nfactors,"<<graph.size()<<"\nuwb_factors,"<<uwb.size()<<'\n';ident.flush();
 if(identity.graph_linearization_sha256!="t08graphlin-sha256:22fab638c30f51770c7d8d34b1b4d4d7bfe4d71ed1812dee75b6b83df2ffe789"||identity.values_sha256!="t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f"||initial.size()!=123||graph.size()!=371||uwb.size()!=328)throw std::runtime_error("RESTORATION_INITIAL_IDENTITY_FAILED");
 auto values=loadValues(ref+"/first_block_call50_values.csv",initial);
 auto checkpoint=uifgo::ComputeInferenceContentIdentity(graph,values,context);
 ident<<"checkpoint_graph_sha,"<<checkpoint.graph_linearization_sha256<<"\ncheckpoint_values_sha,"<<checkpoint.values_sha256<<"\ncheckpoint_error,"<<graph.error(values)<<'\n';
 // A12 original gate, before inspecting any PIM or decomposing curvature.
 if(checkpoint.graph_linearization_sha256!="t08graphlin-sha256:60f7180fb01eca83634129937cede207e8c5c276bfda7ccd96c5eccb679a1e00" || checkpoint.values_sha256!="t08values-sha256:48012c8dd18fb70afaa7abf7bf363edd3363e467a0a145bc05d94f4b157e92db" || !near(graph.error(values),1911.3132066226271))throw std::runtime_error("A12_CHECKPOINT_IDENTITY_FAILED");
 auto audit=uifgo::AuditNavigationStationarity(graph,values,{},1e-6,8);auto expected=read(ref+"/checkpoint_stationarity.csv").at(0);
 auto ast=output(out+"/checkpoint_stationarity.csv");ast<<"valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n";station(ast,audit);ast<<'\n';
 const double actualGrad[]={audit.max_pose_rotation_gradient_objective_per_rad,audit.max_pose_translation_gradient_objective_per_m,audit.max_velocity_gradient_objective_per_mps,audit.max_accel_bias_gradient_objective_per_mps2,audit.max_gyro_bias_gradient_objective_per_radps};
 const char* groups[]={"rotation","translation","velocity","accel_bias","gyro_bias"};
 if(!audit.valid)throw std::runtime_error("INVALID_STATIONARITY_AUDIT");
 for(int j=0;j<5;++j)if(!near(actualGrad[j],std::stod(expected.at(groups[j])),2e-10,2e-12))throw std::runtime_error("A12_GRADIENT_MISMATCH");
 auto oldFactors=read(ref+"/factor_identity.csv");
 for(size_t i=0;i<graph.size();++i)if(oldFactors.at(i).at("dynamic_type")!=typeid(*graph[i]).name()||oldFactors.at(i).at("keys")!=keys(*graph[i])||!near(std::stod(oldFactors.at(i).at("error_at_checkpoint")),graph[i]->error(values)))throw std::runtime_error("A12_FACTOR_IDENTITY_MISMATCH");
 ident<<"restoration_gate,PASS\n";ident.flush();
 auto columns=read(ref+"/columns.csv"),direction=read(ref+"/frozen_weak_direction.csv");
 auto u=values.zeroVectors();if(columns.size()!=615||direction.size()!=615)throw std::runtime_error("FROZEN_DIRECTION_SIZE");
 for(size_t j=0;j<615;++j){auto&c=columns[j];auto&d=direction[j];if(d.at("column")!=std::to_string(j)||d.at("key")!=c.at("key")||d.at("coordinate")!=c.at("coordinate"))throw std::runtime_error("FROZEN_DIRECTION_MAPPING");u.at(std::stoull(c.at("key_value")))[std::stoi(c.at("coordinate"))]=std::stod(d.at("dimensionless_component"));}
 if(std::abs(u.norm()-1)>1e-12)throw std::runtime_error("FROZEN_DIRECTION_NORM");
 auto matrices=output(out+"/matrices.csv");matrices<<"factor_index,name,nrows,ncols,row,col,value\n";
 auto factors=output(out+"/factor_curvature.csv");factors<<"factor_index,dynamic_type,keys,rows,error,kappa\n";
 auto cross=output(out+"/navigation_cross_curvature.csv");cross<<"factor_index,group_row,group_col,value\n";
 auto pims=output(out+"/pim_summary.csv");pims<<"factor_index,keyframe_i,keyframe_j,t0,t1,deltaT,steps,reset_cov_max,reset_biasAccOmegaInt_difference,final_covariance_replay_max_diff,component_sum_max_diff,mean_replay_max_diff,body_P_sensor,omegaCoriolis,use2ndOrderCoriolis,noise_type\n";
 auto steps=output(out+"/propagation_steps.csv");steps<<"factor_index,step,t_end,dt,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,native_covariance_max_diff";for(int i=0;i<15;++i)for(int j=0;j<15;++j)steps<<",F_"<<i<<'_'<<j;steps<<'\n';
 std::array<std::string,8> sources={"measurement_acc","measurement_gyro","integration","bias_RW_acc","bias_RW_gyro","biasAccOmegaInt_acc","biasAccOmegaInt_gyro","biasAccOmegaInt_cross"};
 double totalCurvature=0;size_t totalPims=0,totalSteps=0;
 for(size_t fi=0;fi<graph.size();++fi){const auto&factor=graph[fi];GaussianFactorGraph fg;fg.push_back(factor->linearize(values));auto jb=fg.jacobian(Ordering(factor->keys()));int dims=jb.first.cols();Vector local(dims);Matrix localGroups=Matrix::Zero(dims,5);int c=0;for(auto k:factor->keys())for(int j=0;j<u.at(k).size();++j){local[c]=u.at(k)[j];localGroups(c,group(k,j))=local[c];++c;}
  Vector whiteZ=jb.first*local;double kappa=whiteZ.squaredNorm();totalCurvature+=kappa;factors<<fi<<','<<typeid(*factor).name()<<','<<keys(*factor)<<','<<jb.first.rows()<<','<<factor->error(values)<<','<<kappa<<'\n';
  Matrix projections=jb.first*localGroups,composition=projections.transpose()*projections;for(int i=0;i<5;++i)for(int j=0;j<5;++j)cross<<fi<<','<<groups[i]<<','<<groups[j]<<','<<composition(i,j)<<'\n';
  auto imu=boost::dynamic_pointer_cast<CombinedImuFactor>(factor);if(!imu)continue;++totalPims;
  const auto& actual=imu->preintegratedMeasurements();const auto& p=actual.p();auto noise=boost::dynamic_pointer_cast<noiseModel::Gaussian>(imu->noiseModel());if(!noise)throw std::runtime_error("NOT_GAUSSIAN_IMU");
  dump(matrices,fi,"actual_PIM_covariance",actual.preintMeasCov());dump(matrices,fi,"actual_noise_R",noise->R());dump(matrices,fi,"actual_noise_covariance",noise->covariance());
  dump(matrices,fi,"accelerometerCovariance",p.accelerometerCovariance);dump(matrices,fi,"gyroscopeCovariance",p.gyroscopeCovariance);dump(matrices,fi,"integrationCovariance",p.integrationCovariance);dump(matrices,fi,"biasAccCovariance",p.biasAccCovariance);dump(matrices,fi,"biasOmegaCovariance",p.biasOmegaCovariance);dump(matrices,fi,"biasAccOmegaInt",p.biasAccOmegaInt);dump(matrices,fi,"gravity",p.n_gravity);dump(matrices,fi,"biasHat",actual.biasHat().vector());dump(matrices,fi,"bias_i_at_checkpoint",values.at<imuBias::ConstantBias>(imu->key5()).vector());
  Matrix h1,h2,h3,h4,h5,h6;Vector residual=imu->evaluateError(values.at<Pose3>(imu->key1()),values.at<Vector3>(imu->key2()),values.at<Pose3>(imu->key3()),values.at<Vector3>(imu->key4()),values.at<imuBias::ConstantBias>(imu->key5()),values.at<imuBias::ConstantBias>(imu->key6()),h1,h2,h3,h4,h5,h6);
  Matrix H(15,30);H<<h1,h2,h3,h4,h5,h6;dump(matrices,fi,"unwhitened_H",H);dump(matrices,fi,"actual_whitened_J",jb.first);dump(matrices,fi,"local_frozen_direction",local);dump(matrices,fi,"unwhitened_z",H*local);dump(matrices,fi,"actual_white_z",whiteZ);dump(matrices,fi,"unwhitened_residual",residual);
  dump(matrices,fi,"preintegrated",actual.preintegrated());dump(matrices,fi,"preintegrated_H_biasAcc",actual.preintegrated_H_biasAcc());dump(matrices,fi,"preintegrated_H_biasOmega",actual.preintegrated_H_biasOmega());
  // Read-only copies with identical parameters and the exact original biasHat.
  // No parameter is changed; covariance components are separate algebra arrays.
  PreintegrationType meanReplay=actual;meanReplay.resetIntegrationAndSetBias(actual.biasHat());
  PreintegratedCombinedMeasurements nativeReplay=actual;nativeReplay.resetIntegrationAndSetBias(actual.biasHat());
  double resetCov=nativeReplay.preintMeasCov().cwiseAbs().maxCoeff();double resetK=(nativeReplay.p().biasAccOmegaInt-p.biasAccOmegaInt).cwiseAbs().maxCoeff();
  std::array<M15,8> covs;for(auto&x:covs)x.setZero();M15 sum=M15::Zero();size_t count=0;
  auto integrate=[&](const uifgo::ImuSample&sample,double dt,double time){
   Matrix9 A;Matrix93 B,C;meanReplay.update(sample.acc,sample.gyro,dt,&A,&B,&C);Matrix3 T=-C.topRows<3>(),V=-B.bottomRows<3>();M15 F=M15::Zero();F.block<9,9>(0,0)=A;F.block<3,3>(0,12)=T;F.block<3,3>(6,9)=V;F.block<6,6>(9,9)=I_6x6;
   std::array<M15,8> Q;for(auto&x:Q)x.setZero();
   Q[0].block<3,3>(6,6)=(1/dt)*V*p.accelerometerCovariance*V.transpose();
   Q[1].block<3,3>(0,0)=(1/dt)*T*p.gyroscopeCovariance*T.transpose();
   Q[2].block<3,3>(3,3)=dt*p.integrationCovariance;
   Q[3].block<3,3>(9,9)=dt*p.biasAccCovariance;Q[4].block<3,3>(12,12)=dt*p.biasOmegaCovariance;
   Q[5].block<3,3>(6,6)=(1/dt)*V*p.biasAccOmegaInt.block<3,3>(0,0)*V.transpose();
   Q[6].block<3,3>(0,0)=(1/dt)*T*p.biasAccOmegaInt.block<3,3>(3,3)*T.transpose();
   Q[7].block<3,3>(6,0)=V*p.biasAccOmegaInt.block<3,3>(3,0)*T.transpose();Q[7].block<3,3>(0,6)=Q[7].block<3,3>(6,0).transpose();
   sum.setZero();for(size_t j=0;j<8;++j){covs[j]=F*covs[j]*F.transpose()+Q[j];sum+=covs[j];}
   nativeReplay.integrateMeasurement(sample.acc,sample.gyro,dt);++count;++totalSteps;double err=(sum-nativeReplay.preintMeasCov()).cwiseAbs().maxCoeff();
   steps<<fi<<','<<count<<','<<time<<','<<dt<<','<<sample.acc.x()<<','<<sample.acc.y()<<','<<sample.acc.z()<<','<<sample.gyro.x()<<','<<sample.gyro.y()<<','<<sample.gyro.z()<<','<<err;for(int i=0;i<15;++i)for(int j=0;j<15;++j)steps<<','<<F(i,j);steps<<'\n';
  };
  size_t ki=Symbol(imu->key1()).index(),kj=Symbol(imu->key3()).index();double t0=frames.at(ki).t,t1=frames.at(kj).t;size_t index=0;while(index<cache.imu.size()&&cache.imu[index].t<=t0)++index;double prev=t0;
  while(index<cache.imu.size()&&cache.imu[index].t<t1){double dt=cache.imu[index].t-prev;if(dt>1e-9)integrate(cache.imu[index],dt,cache.imu[index].t);prev=cache.imu[index].t;++index;}
  auto end=uifgo::InterpolateImu(cache.imu,t1);double dt=t1-prev;if(dt>1e-9)integrate(end,dt,t1);
  for(size_t j=0;j<8;++j)dump(matrices,fi,"component_"+sources[j],covs[j]);dump(matrices,fi,"component_sum",sum);dump(matrices,fi,"native_replay_covariance",nativeReplay.preintMeasCov());
  double covError=(nativeReplay.preintMeasCov()-actual.preintMeasCov()).cwiseAbs().maxCoeff(),componentError=(sum-actual.preintMeasCov()).cwiseAbs().maxCoeff(),meanError=(meanReplay.preintegrated()-actual.preintegrated()).cwiseAbs().maxCoeff();
  pims<<fi<<','<<ki<<','<<kj<<','<<t0<<','<<t1<<','<<actual.deltaTij()<<','<<count<<','<<resetCov<<','<<resetK<<','<<covError<<','<<componentError<<','<<meanError<<','<<bool(p.body_P_sensor)<<','<<bool(p.omegaCoriolis)<<','<<p.use2ndOrderCoriolis<<','<<typeid(*noise).name()<<'\n';
  if(covError>1e-12+1e-10*actual.preintMeasCov().cwiseAbs().maxCoeff()||componentError>1e-12+1e-10*actual.preintMeasCov().cwiseAbs().maxCoeff()||meanError>1e-12+1e-10*actual.preintegrated().cwiseAbs().maxCoeff()||!near(nativeReplay.deltaTij(),actual.deltaTij(),1e-12,1e-10))throw std::runtime_error("PIM_REPLAY_OR_DECOMPOSITION_MISMATCH");
 }
 auto after=uifgo::ComputeInferenceContentIdentity(graph,values,context);if(after.graph_linearization_sha256!=checkpoint.graph_linearization_sha256||after.values_sha256!=checkpoint.values_sha256)throw std::runtime_error("LIVE_GRAPH_MUTATED");
 if(totalPims!=40||totalSteps!=1600)throw std::runtime_error("PIM_OR_STEP_COUNT");
 ident<<"frozen_direction_norm,"<<u.norm()<<"\ntotal_curvature,"<<totalCurvature<<"\npim_count,"<<totalPims<<"\nintegration_steps,"<<totalSteps<<"\nafter_graph_sha,"<<after.graph_linearization_sha256<<"\nafter_values_sha,"<<after.values_sha256<<"\noptimizer_constructions,0\niterate_calls,0\nwall_seconds,"<<elapsed()<<'\n';
 std::cout<<std::setprecision(17)<<"A13 static completed; PIM="<<totalPims<<" steps="<<totalSteps<<" curvature="<<totalCurvature<<" iterate=0\n";return 0;
 }catch(const std::exception&e){std::cerr<<"A13_FAILED: "<<e.what()<<'\n';return 1;}}
