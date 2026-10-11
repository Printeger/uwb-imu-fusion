#include "online_gaussian_history.hpp"
#include "raw_prefix_history.hpp"
#include "scenario_material.hpp"
#include <iostream>
#include <chrono>
#include <tbb/global_control.h>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool x,const char* name){if(!x)throw std::runtime_error(name);}
void close(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b,double tol,const char* name){check(a.rows()==b.rows() && a.cols()==b.cols() && (a-b).norm()<=tol*(1+a.norm()),name);}
int main(int argc,char** argv){
 tbb::global_control single(tbb::global_control::max_allowed_parallelism,1);
 if(argc!=2 && argc!=3)throw std::runtime_error("usage: fde_online_checks CONFIG");
 NoisyMotionScenario scene(argv[1],"U0",20261010,50);auto law=YAML::LoadFile(argv[1]);
 IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);
 size_t cursor=1;RawPrefixHistory oracle;OnlineGaussianHistory online(scene.cfg.integrity_window.epochs,law["persistent_epochs"].as<int>(),law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
 OnlineGaussianHistory original(scene.cfg.integrity_window.epochs,law["persistent_epochs"].as<int>(),law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever,false);
 for(int e=1;e<=50;++e){while(cursor<scene.imu.size() && !(scene.uwb[e-1].timestamp<scene.imu[cursor].timestamp))backend.ingestImu(scene.imu[cursor++]);
  auto tx=backend.prepareEpoch(scene.uwb[e-1]);oracle.observe(tx,scene.cfg.integrity_window.epochs);
  auto start=std::chrono::steady_clock::now();online.prepare(tx);auto root=online.evaluate(tx,nullptr,1e-6,1e-9,true);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  original.prepare(tx);
  if(e==1){
   OnlineGaussianHistory limited(scene.cfg.integrity_window.epochs,3,.01,.01,scene.lever,true,1);bool refused=false;try{limited.prepare(tx);}catch(const std::runtime_error&){refused=true;}check(refused && limited.counts().resource_refused,"preflight resource refusal");
   OnlineGaussianHistory bounded(scene.cfg.integrity_window.epochs,law["persistent_epochs"].as<int>(),law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever,true,295000);bounded.prepare(tx);bounded.evaluate(tx,nullptr,1e-6,1e-9,false);
   ExclusionAction one;for(const auto& g:tx.uwb_groups)if(g.nominal)one.groups_to_remove.push_back(g.id);bounded.evaluate(tx,&one,1e-6,1e-9);
   ExclusionAction two;two.groups_to_remove={tx.imu_group.id};refused=false;try{bounded.evaluate(tx,&two,1e-6,1e-9);}catch(const std::runtime_error& error){refused=std::string(error.what())=="UNAVAILABLE_SUMMARY_RESOURCES";}check(refused && bounded.counts().resource_refused,"candidate resource exhaustion");
   refused=false;try{bounded.evaluate(tx,&one,1e-6,1e-9);}catch(const std::runtime_error& error){refused=std::string(error.what())=="UNAVAILABLE_SUMMARY_RESOURCES";}check(refused,"resource failure must latch over earlier cached candidate");
   for(int mutation=0;mutation<5;++mutation){auto bad=tx;if(mutation==0)++bad.ledger_version;if(mutation==1)bad.uwb_groups.front().raw_covariance(0,0)=std::nextafter(bad.uwb_groups.front().raw_covariance(0,0),INFINITY);if(mutation==2)bad.raw_imu_slice.front().specific_force_mps2(0)+=1e-12;if(mutation==3)bad.imu_group.id=FactorGroupId(bad.imu_group.id.value()+1);if(mutation==4)bad.frozen_values=std::make_shared<const gtsam::Values>(*tx.frozen_values);
    bool rejected=false;try{online.evaluate(bad,nullptr,1e-6,1e-9);}catch(const std::runtime_error&){rejected=true;}check(rejected,("cached transaction/raw/noise binding mutation="+std::to_string(mutation)).c_str());}
   auto risk=online.evaluate(tx,nullptr,1e-5,1e-8);auto risk_ref=original.evaluate(tx,nullptr,1e-5,1e-8);close(risk.reference.pl,risk_ref.reference.pl,1e-10,"risk argument separate numerical root");
  }
  if(e==30 || e==31 || e==32){auto strict=original.evaluate(tx,nullptr,1e-6,1e-9,true);
   close(root.reference.mean,strict.reference.mean,1e-10,"original summary mean");close(root.reference.covariance,strict.reference.covariance,1e-10,"original summary covariance");close(root.reference.pl,strict.reference.pl,1e-10,"original summary PL");check(root.reference.status==strict.reference.status && root.reference.dof==strict.reference.dof,"original summary status/dof");
   for(const auto& item:root.faults){const auto& a=item.second;const auto& b=strict.faults.at(item.first);check(a.classification==b.classification,"original leaf class");close(a.residual_gram,b.residual_gram,1e-10,"original leaf Gram");close(a.profile_rhs,b.profile_rhs,1e-10,"original leaf profile");close(a.profile_estimate,b.profile_estimate,1e-10,"original leaf estimate");close(a.response,b.response,1e-10,"original leaf response");}
   auto w=oracle.window(tx);auto material=oracle.material(tx);auto modes=oracle.uwbModes(tx,law["persistent_epochs"].as<int>());
   auto raw=rawGaussianWindow(w,material,modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
   auto dense=gaussianReference(raw.a,raw.z,raw.sigma,raw.c,raw.faults,1e-6,1e-9);
   check(root.reference.status==dense.status,"online status");check(root.reference.dof==dense.dof,"online dof");
   check(root.total_rank-6*(e+1)==raw.a.cols(),"original physical state rank");
   check(std::abs(root.reference.statistic-dense.statistic)<1e-7,"online statistic");check(std::abs(root.reference.threshold-dense.threshold)<1e-9,"online threshold");
   int current=root.reference.mean.size()-21;close(root.reference.mean.segment(current,15),dense.mean.tail(15),1e-7,"online current mean");
   close(root.reference.covariance.block(current,current,15,15),dense.covariance.bottomRightCorner(15,15),1e-7,"online current covariance");
   close(online.tagMap()*root.reference.mean,raw.c*dense.mean,1e-7,"online protected mean");close(root.reference.pl,dense.pl,1e-6,"online PL");
   close(online.navigationMean(root.reference),dense.mean,1e-7,"all historical navigation means");
   close(online.navigationCovariance(root.reference),dense.covariance,1e-7,"all historical navigation covariance blocks");
   Eigen::Matrix<double,5,1> steps=Eigen::Matrix<double,5,1>::Zero();for(int q=0;q<dense.mean.size();q+=15)for(int i=0;i<5;++i)steps(i)=std::max(steps(i),dense.mean.segment(q+3*i,3).norm());
   close(online.referenceSteps(root.reference),steps,1e-7,"old-state reference domain gate");
   for(size_t i=0;i<modes.size();++i){auto& f=root.faults.at(modes[i].physical_source_id);auto pb=(dense.parity*dense.whitener*raw.faults[i]).eval();auto pz=(dense.parity*dense.whitener*raw.z).eval();
    close(f.residual_gram,pb.transpose()*pb,1e-7,"online historical fault Gram");close(f.profile_rhs,pb.transpose()*pz,1e-7,"online historical profile");close(f.profile_estimate,pb.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(pz),1e-7,"online estimated fault amplitude");check(f.classification=="MONITORABLE","historical fault monitorability");close(f.response,raw.c*dense.gain*dense.whitener*raw.faults[i],1e-7,"online fault protected response");}
   if(e==31){ExclusionAction old;old.groups_to_remove={modes.front().raw_group_maps.begin()->first};bool refused=false;
    try{online.evaluate(tx,&old,1e-6,1e-9);}catch(const std::runtime_error& error){refused=std::string(error.what())=="UNAVAILABLE_RETIRED_CANDIDATE_SOURCE";}check(refused,"retired-source candidate must refuse");}
   std::cout<<"ONLINE_ORACLE epoch="<<e<<" dof="<<dense.dof<<" statistic="<<dense.statistic<<" state_cols="<<root.reference.mean.size()<<" faults="<<modes.size()<<" all_fields=PASS\n";
  }
  if(e==12 || e==31 || e==32 || e==50){auto c=online.counts();std::cout<<"ONLINE_COUNTS epoch="<<e<<" active_states="<<c.active_states<<" history_rows="<<c.history_rows<<" fault_modes="<<c.fault_modes<<" sources="<<c.source_columns<<" integrations="<<c.integrations<<" bytes="<<c.bytes<<" ms="<<ms<<"\n";}
  auto plan=EpochCommitPlan::nominalPlan(tx);online.committed(tx,plan);original.committed(tx,plan);oracle.committed(tx,plan);
  bool stale_refused=false;try{online.evaluate(tx,nullptr,1e-6,1e-9);}catch(const std::runtime_error&){stale_refused=true;}check(stale_refused,"post-commit cached root must refuse");
  auto receipt=backend.commitEpoch(std::move(tx),plan);check(receipt.backend_updates==1,"single backend update");if(argc==3){std::cout<<"RESOURCE_BINDINGS all_negative_cases=PASS\n";break;}
 }
}
