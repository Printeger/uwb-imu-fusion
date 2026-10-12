#include "scenario_material.hpp"
#include "raw_source_law.hpp"
#include "raw_prefix_history.hpp"
#include "hybrid_coordinator.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(int argc,char** argv){try{
 std::string path=argc>1?argv[1]:"config/hybrid_raw.yaml";
 NoisyMotionScenario scene(path,"U0",20261010,5);
 check(scene.initial_law_valid && scene.physical_source_reference,"matched law");
 auto inventory=rawSourceInventory(50,8,scene.cfg.realtime.prior_sigmas,.02,.001,scene.cfg.imu.accelerometer_bias_rw_sigma,scene.cfg.imu.gyroscope_bias_rw_sigma,.1);
 std::set<std::string> ids;for(const auto& s:inventory)check(ids.insert(s.id).second,"unique primitive ID");check(ids.size()==6421,"full horizon unique dimension");
 // Shared point-sample trapezoid oracle for a parameterized scalar family.
 for(double dt:{.005,.02,.1})for(double variance:{.0004,.01,1.}){
  Eigen::Matrix<double,2,3> map;map<<dt/2,dt/2,0,0,dt/2,dt/2;
  Eigen::Matrix2d cov=variance*map*map.transpose();
  check(std::abs(cov(0,0)-variance*dt*dt/2)<1e-14 && std::abs(cov(0,1)-variance*dt*dt/4)<1e-14,"shared endpoint units");
  // Constant bias marginalization is t^2 Var(b), fresh increments t Var(b).
  check(std::abs(std::pow(10*dt,2)*variance-100*dt*dt*variance)<1e-13,"latent bias analytic oracle");
 }
 IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);
 OnlineGaussianHistory online(2,3,.02,.001,scene.lever,true,64u*1024u*1024u,true);RawPrefixHistory prefix;
 double maxCov=0,maxCross=0;
 for(int epoch=1;epoch<=5;++epoch){for(int i=(epoch-1)*10+1;i<=epoch*10;++i)backend.ingestImu(scene.imu[i]);
  auto tx=backend.prepareEpoch(scene.uwb[epoch-1],EpochPreparationOptions{true,true,false,false});prefix.observe(tx,2);online.prepare(tx,false);
  auto window=prefix.window(tx);auto material=prefix.material(tx);auto raw=rawGaussianWindow(window,material,{},.02,.001,scene.lever,false,true);
  auto evidence=online.preUwbEvidence(tx,100);
  std::vector<int> kept;for(size_t k=0;k<window.blocks.size();++k){bool current=false;for(const auto& g:tx.uwb_groups)current=current || (g.nominal && g.id==window.blocks[k].group_id);if(!current)for(int r=0;r<window.blocks[k].residual_raw.size();++r)kept.push_back(raw.offsets[k]+r);}
  Eigen::MatrixXd a(kept.size(),raw.a.cols());Eigen::VectorXd z(kept.size());for(size_t i=0;i<kept.size();++i){a.row(i)=raw.a.row(kept[i]);z(i)=raw.z(kept[i]);}
  Eigen::LLT<Eigen::MatrixXd> white(principal(raw.sigma,kept));check(white.info()==Eigen::Success,"physical covariance SPD without jitter");
  Eigen::MatrixXd aw=white.matrixL().solve(a);Eigen::VectorXd zw=white.matrixL().solve(z);
  auto qr=aw.colPivHouseholderQr();Eigen::MatrixXd gain=qr.solve(Eigen::MatrixXd::Identity(aw.rows(),aw.rows()));Eigen::MatrixXd cov=gain*gain.transpose();Eigen::VectorXd mean=gain*zw;
  double rel=(cov.bottomRightCorner(15,15)-evidence.history.covariance).norm()/evidence.history.covariance.norm();maxCov=std::max(maxCov,rel);maxCross=std::max(maxCross,raw.cross_norm);
  check(rel<1e-6 && (mean.tail(15)-evidence.correction).norm()<1e-6,"physical source prefix/retirement full15 oracle");
  // Independent residual oracle mutates literal input samples and terminal
  // bias, then uses the public reintegration entry, not its RW map builder.
  auto rw=physicalBiasIncrementMap(tx);
  for(size_t k:{size_t(1),size_t(10)})for(int axis=0;axis<6;++axis){auto plus=tx,minus=tx;double eps=2e-5;
   auto mutate=[&](EpochTransaction& t,double d){for(size_t i=k;i<t.raw_imu_slice.size();++i){if(axis<3)t.raw_imu_slice[i].specific_force_mps2(axis)+=d;else t.raw_imu_slice[i].angular_velocity_radps(axis-3)+=d;}auto values=std::make_shared<gtsam::Values>(*t.frozen_values);auto key=gtsam::Symbol('b',t.proposed_epoch);auto bias=values->at<gtsam::imuBias::ConstantBias>(key).vector();bias(axis)+=d;values->update(key,gtsam::imuBias::ConstantBias(bias.head<3>(),bias.tail<3>()));t.frozen_values=values;};
   mutate(plus,eps);mutate(minus,-eps);RawImuEpisode empty{"oracle",tx.begin,TimestampNs(tx.begin.value()+1),0};
   auto jp=episodeFactorError(plus,integrateEpisode(plus,empty,0));auto jm=episodeFactorError(minus,integrateEpisode(minus,empty,0));double sig=(axis<3?scene.cfg.imu.accelerometer_bias_rw_sigma:scene.cfg.imu.gyroscope_bias_rw_sigma)*std::sqrt(.005);
   check((-(jp-jm)/(2*eps)*sig-rw.col(6*(k-1)+axis)).norm()<1e-9,"independent full15 RW source residual");
  }
  auto plan=EpochCommitPlan::nominalPlan(tx);auto receipt=backend.commitEpoch(EpochTransaction(tx),plan);check(receipt.backend_updates==1,"actual source commit");online.committed(tx,plan);prefix.committed(tx,plan);
 }
 check(maxCross>0,"adjacent PIM shared source cross");
 std::cout<<"raw source inventory="<<ids.size()<<" full15 RW/ADC/cross/history oracle PASS max_cov_relative="<<maxCov<<" cross="<<maxCross<<" nonlinear_qualification=OPEN\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
