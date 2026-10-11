#include "online_gaussian_history.hpp"
#include "raw_prefix_history.hpp"
#include "scenario_material.hpp"
#include <iostream>
#include <chrono>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool x,const char* name){if(!x)throw std::runtime_error(name);}
void close(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b,double tol,const char* name){check(a.rows()==b.rows() && a.cols()==b.cols() && (a-b).norm()<=tol*(1+a.norm()),name);}
int main(int argc,char** argv){
 if(argc!=2)throw std::runtime_error("usage: fde_online_checks CONFIG");
 NoisyMotionScenario scene(argv[1],"U0",20261010,32);auto law=YAML::LoadFile(argv[1]);
 IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);
 size_t cursor=1;RawPrefixHistory oracle;OnlineGaussianHistory online(scene.cfg.integrity_window.epochs,law["persistent_epochs"].as<int>(),law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
 for(int e=1;e<=32;++e){while(cursor<scene.imu.size() && !(scene.uwb[e-1].timestamp<scene.imu[cursor].timestamp))backend.ingestImu(scene.imu[cursor++]);
  auto tx=backend.prepareEpoch(scene.uwb[e-1]);oracle.observe(tx,scene.cfg.integrity_window.epochs);
  auto start=std::chrono::steady_clock::now();online.prepare(tx);auto root=online.evaluate(tx,nullptr,1e-6,1e-9,true);
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if(e==12 || e==31){auto w=oracle.window(tx);auto material=oracle.material(tx);auto modes=oracle.uwbModes(tx,law["persistent_epochs"].as<int>());
   auto raw=rawGaussianWindow(w,material,modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
   auto dense=gaussianReference(raw.a,raw.z,raw.sigma,raw.c,raw.faults,1e-6,1e-9);
   check(root.reference.status==dense.status,"online status");check(root.reference.dof==dense.dof,"online dof");
   check(std::abs(root.reference.statistic-dense.statistic)<1e-7,"online statistic");check(std::abs(root.reference.threshold-dense.threshold)<1e-9,"online threshold");
   int current=root.reference.mean.size()-21;close(root.reference.mean.segment(current,15),dense.mean.tail(15),1e-7,"online current mean");
   close(root.reference.covariance.block(current,current,15,15),dense.covariance.bottomRightCorner(15,15),1e-7,"online current covariance");
   close(online.tagMap()*root.reference.mean,raw.c*dense.mean,1e-7,"online protected mean");close(root.reference.pl,dense.pl,1e-6,"online PL");
   Eigen::Matrix<double,5,1> steps=Eigen::Matrix<double,5,1>::Zero();for(int q=0;q<dense.mean.size();q+=15)for(int i=0;i<5;++i)steps(i)=std::max(steps(i),dense.mean.segment(q+3*i,3).norm());
   close(online.referenceSteps(root.reference),steps,1e-7,"old-state reference domain gate");
   for(size_t i=0;i<modes.size();++i){auto& f=root.faults.at(modes[i].physical_source_id);auto pb=(dense.parity*dense.whitener*raw.faults[i]).eval();auto pz=(dense.parity*dense.whitener*raw.z).eval();
    close(f.residual_gram,pb.transpose()*pb,1e-7,"online historical fault Gram");close(f.profile_rhs,pb.transpose()*pz,1e-7,"online historical profile");close(f.response,raw.c*dense.gain*dense.whitener*raw.faults[i],1e-7,"online fault protected response");}
   std::cout<<"ONLINE_ORACLE epoch="<<e<<" dof="<<dense.dof<<" statistic="<<dense.statistic<<" state_cols="<<root.reference.mean.size()<<" faults="<<modes.size()<<" all_fields=PASS\n";
  }
  if(e==12 || e==31 || e==32){auto c=online.counts();std::cout<<"ONLINE_COUNTS epoch="<<e<<" active_states="<<c.active_states<<" history_rows="<<c.history_rows<<" fault_modes="<<c.fault_modes<<" sources="<<c.source_columns<<" integrations="<<c.integrations<<" bytes="<<c.bytes<<" ms="<<ms<<"\n";}
  auto plan=EpochCommitPlan::nominalPlan(tx);online.committed(tx,plan);oracle.committed(tx,plan);auto receipt=backend.commitEpoch(std::move(tx),plan);check(receipt.backend_updates==1,"single backend update");
 }
}
