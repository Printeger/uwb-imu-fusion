#include "hybrid_coordinator.hpp"
#include "scenario_material.hpp"
#include "raw_prefix_history.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <tbb/global_control.h>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
void verify(bool b,const char* name){if(!b)throw std::runtime_error(name);}
void vec(std::ostream& s,const Eigen::Vector3d& v){s<<','<<v.x()<<','<<v.y()<<','<<v.z();}
}
void rawTighteningDiagnostics(const std::string& config,const std::string& destination){
 tbb::global_control single(tbb::global_control::max_allowed_parallelism,1);
 std::filesystem::create_directories(destination);
 std::ofstream out(destination+"/candidates.csv"),intervals(destination+"/intervals.csv"),audit(destination+"/audit.csv");out<<std::setprecision(17);intervals<<std::setprecision(17);audit<<std::setprecision(17);
 out<<"scene,epoch,excluded,selected,q,dimension,event_diagnostic_epsilon,raw_risk,threshold,statistic,eta,history_explanations,nominal_x,nominal_y,nominal_z,slope_x,slope_y,slope_z,threshold_x,threshold_y,threshold_z,noise_x,noise_y,noise_z,eta_x,eta_y,eta_z,history_x,history_y,history_z,joint_fault_x,joint_fault_y,joint_fault_z,center_x,center_y,center_z,old_hpl,old_vpl,joint_hpl,joint_vpl,old_status,joint_status,A_alarm,B_alarm,qualified\n";
 intervals<<"scene,epoch,excluded,episode,historical_low,historical_high,active_column,joint_feasible,joint_low,joint_high,output_support_x,output_support_y,output_support_z\n";
 audit<<"scene,epoch,initial_law_valid,initial_position_error,initial_normalized_error,P_native_vs_raw_relative,P_raw_vs_prefix_relative,mean_raw_vs_prefix,adc_cross_norm,pim_vs_raw_relative,rot_pim,rot_raw,position_pim,position_raw,velocity_pim,velocity_raw,bias_pim,bias_raw,auxiliary_trace,adc_trace,prior_bias_sigma,prior_gyro_bias_sigma,configured_accel_sigma,configured_gyro_sigma,actual_accel_sigma,actual_gyro_sigma,chart_vs_truth_rotation,chart_vs_truth_position,native_vs_chart_position,raw_noise_unique_samples,primitive_dimension_prefix,declared_horizon_dimension,aux_without_biasHat_trace,biasHat_covariance_trace\n";
 for(const std::string kind:{"U0","U1"}){
  NoisyMotionScenario scene(config,kind,20261010,31);IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);
  backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);
  OnlineGaussianHistory online(scene.cfg.integrity_window.epochs,3,scene.law["raw_accel_sigma_mps2"].as<double>(),scene.law["raw_gyro_sigma_radps"].as<double>(),scene.lever);RawPrefixHistory prefix;
  for(int epoch=1;epoch<=31;++epoch){for(int i=(epoch-1)*10+1;i<=epoch*10;++i)backend.ingestImu(scene.imu[i]);
   auto batch=scene.uwb[epoch-1];auto tx=backend.prepareEpoch(batch,EpochPreparationOptions{true,true,false,false});prefix.observe(tx,scene.cfg.integrity_window.epochs);online.prepare(tx,false);
   constexpr int dimension=15+50*(66+15+8);double q=std::sqrt(boost::math::quantile(boost::math::complement(boost::math::chi_squared(dimension),1e-6)));
   auto evidence=online.preUwbEvidence(tx,q);auto a=uwbEpochRaim(batch,true,1e-6,1e-9);
   auto baseline=hybridRawCurrent(batch,evidence,scene.lever,a,true,1e-6);
   auto alternative=hybridRawCurrent(batch,evidence,scene.lever,a,true,1e-6,nullptr,nullptr,true);
   verify(alternative.excluded==baseline.excluded,"same input selection under joint representation");
   auto plan=EpochCommitPlan::nominalPlan(tx);
   if(baseline.excluded>=0){const PendingFactorGroup* selected=nullptr;for(const auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()==baseline.retained.size()){
    bool match=true;for(int i:baseline.retained)match=match && std::find(g.source_measurements.begin(),g.source_measurements.end(),batch.measurements[i].id)!=g.source_measurements.end();if(match)selected=&g;}
    verify(selected,"native baseline subset exists");plan.groups_to_add={tx.imu_group.id,selected->id};plan.fde_status=FdeStatus::SuccessUwbExclusion;
   }
   if(epoch==15 || epoch==31){
    verify(evidence.history.status=="BOUNDED","raw prediction extracted");
    auto c=tagMap(evidence.chart,scene.lever);gtsam::Matrix hp;gtsam::Pose3 chart(gtsam::Rot3(evidence.chart.q_world_body.toRotationMatrix()),evidence.chart.position_world_m);
    Eigen::VectorXd innovation=-UwbPoseBatchFactor(1,batch,scene.lever).evaluateError(chart,hp);Eigen::MatrixXd h=Eigen::MatrixXd::Zero(8,15);h.leftCols(6)=hp;innovation-=h*evidence.correction;
    std::vector<int> continuing;verify(hybridRawEpisodeColumns(batch,evidence.history,continuing),"raw episode provenance");
    std::vector<HybridBound> oldBounds,jointBounds;std::vector<std::vector<JointEpisodeSupport>> supportDetails;
    for(int excluded=-1;excluded<8;++excluded){auto old=hybridBound(evidence.history.covariance,h,validatedUwbCovariance(batch),Eigen::MatrixXd::Zero(15,8),c,innovation,evidence.history,excluded,1e-6,q);
     std::vector<JointEpisodeSupport> details;auto joint=hybridJointEpisodeBound(old,h,c,innovation,evidence.history,continuing,q,&details);
     verify((joint.gaussian.pl.array()<=old.gaussian.pl.array()+1e-8).all(),"raw frozen joint bound no larger");oldBounds.push_back(old);jointBounds.push_back(joint);supportDetails.push_back(details);
    }
    if(epoch==31){ // Exactly two statistical audits: U0/31 and U1/31.
     auto native=backend.researchPreUwbPredictionV1(tx);auto window=prefix.window(tx);auto material=prefix.material(tx);auto raw=rawGaussianWindow(window,material,{},scene.law["raw_accel_sigma_mps2"].as<double>(),scene.law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
     std::vector<int> keep;int imuBlock=-1;
     for(size_t k=0;k<window.blocks.size();++k){bool current=false;for(const auto& g:tx.uwb_groups)current=current || (g.nominal && g.id==window.blocks[k].group_id);
      if(window.blocks[k].group_id==tx.imu_group.id)imuBlock=k;
      if(!current)for(int r=0;r<window.blocks[k].residual_raw.size();++r)keep.push_back(raw.offsets[k]+r);
     }
     Eigen::MatrixXd pa(keep.size(),raw.a.cols());Eigen::VectorXd pz(keep.size());for(size_t i=0;i<keep.size();++i){pa.row(i)=raw.a.row(keep[i]);pz(i)=raw.z(keep[i]);}
     Eigen::LLT<Eigen::MatrixXd> white(principal(raw.sigma,keep));Eigen::MatrixXd aw=white.matrixL().solve(pa);Eigen::VectorXd zw=white.matrixL().solve(pz);
     Eigen::MatrixXd information=aw.transpose()*aw;Eigen::LDLT<Eigen::MatrixXd> info(information);
     Eigen::VectorXd mean=info.solve(aw.transpose()*zw);Eigen::MatrixXd covariance=info.solve(Eigen::MatrixXd::Identity(information.rows(),information.cols()));
     double covarianceRelative=(covariance.bottomRightCorner(15,15)-evidence.history.covariance).norm()/evidence.history.covariance.norm();
     verify(covarianceRelative<1e-6 && (mean.tail(15)-evidence.correction).norm()<1e-6,"independent raw prefix P oracle");
     verify(imuBlock>=0,"current IMU block found");int off=raw.offsets[imuBlock];Eigen::MatrixXd rp=raw.sigma.block(off,off,15,15),pim=tx.preintegration->preintMeasCov(),aux=raw.auxiliary_sigma.block(off,off,15,15);
     auto params=boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(tx.preintegration->params());verify(bool(params),"actual combined params");
     auto withoutBias=boost::make_shared<gtsam::PreintegrationCombinedParams>(*params);
     withoutBias->accelerometerCovariance.setZero();withoutBias->gyroscopeCovariance.setZero();withoutBias->biasAccOmegaInt.setZero();
     gtsam::PreintegratedCombinedMeasurements noBias(withoutBias,tx.preintegration->biasHat());
     for(size_t i=1;i<tx.raw_imu_slice.size();++i){const auto& l=tx.raw_imu_slice[i-1];const auto& r=tx.raw_imu_slice[i];noBias.integrateMeasurement(.5*(l.specific_force_mps2+r.specific_force_mps2),.5*(l.angular_velocity_radps+r.angular_velocity_radps),r.timestamp.seconds()-l.timestamp.seconds());}
     auto truth=scene.truth(epoch*scene.dt);Eigen::Vector3d initialError=scene.initial_observation.position_world_m-scene.truth(0).position_world_m;
     double normalizedInitial=(initialError.array()/scene.cfg.realtime.prior_sigmas.segment<3>(3).array()).matrix().norm();
     audit<<kind<<','<<epoch<<','<<scene.initial_law_valid<<','<<initialError.norm()<<','<<normalizedInitial<<','<<(native.prior.covariance-evidence.history.covariance).norm()/evidence.history.covariance.norm()<<','<<covarianceRelative<<','<<(mean.tail(15)-evidence.correction).norm()<<','<<raw.cross_norm<<','<<(pim-rp).norm()/rp.norm();
     for(auto block:std::vector<std::pair<int,int>>{{0,3},{3,3},{6,3},{9,6}})audit<<','<<pim.block(block.first,block.first,block.second,block.second).trace()<<','<<rp.block(block.first,block.first,block.second,block.second).trace();
     audit<<','<<aux.trace()<<','<<(rp-aux).trace()<<','<<scene.cfg.realtime.prior_sigmas(9)<<','<<scene.cfg.realtime.prior_sigmas(12)<<','<<std::sqrt(params->accelerometerCovariance(0,0))<<','<<std::sqrt(params->gyroscopeCovariance(0,0))<<','<<scene.law["raw_accel_sigma_mps2"].as<double>()<<','<<scene.law["raw_gyro_sigma_radps"].as<double>()<<','<<Eigen::AngleAxisd(truth.q_world_body.conjugate()*evidence.chart.q_world_body).angle()<<','<<(truth.position_world_m-evidence.chart.position_world_m).norm()<<','<<(native.prior.mean.position_world_m-evidence.chart.position_world_m).norm()<<','<<raw.noise_j.cols()/6<<','<<15+6*(10*epoch+1)+15*epoch+8*epoch<<','<<dimension<<','<<noBias.preintMeasCov().trace()<<','<<(aux-noBias.preintMeasCov()).trace()<<'\n';
    }
    auto receipt=backend.commitEpoch(EpochTransaction(tx),plan);verify(receipt.backend_updates==1,"one diagnostic native commit");auto state=backend.currentState();Eigen::Vector3d committed=state.position_world_m+state.q_world_body*scene.lever;
    for(int index=0;index<9;++index){int excluded=index-1;auto& old=oldBounds[index];auto& joint=jointBounds[index];Eigen::Vector3d reference=evidence.chart.position_world_m+evidence.chart.q_world_body*scene.lever+c*(evidence.correction+old.gaussian.mean),center=(committed-reference).cwiseAbs();
     Eigen::Vector3d oldPL=old.gaussian.pl+center,jointPL=joint.gaussian.pl+center;
     out<<kind<<','<<epoch<<','<<excluded<<','<<(excluded==baseline.excluded)<<','<<q<<','<<dimension<<",1e-6,UNKNOWN,"<<old.gaussian.threshold<<','<<old.gaussian.statistic<<','<<old.eta<<','<<evidence.history.segments.size();
     for(const auto& v:{old.nominal,old.slope,old.threshold_part,old.noise_part,old.eta_part,old.history,joint.joint_fault,center})vec(out,v);
     out<<','<<oldPL.head<2>().norm()<<','<<oldPL.z()<<','<<jointPL.head<2>().norm()<<','<<jointPL.z()<<','<<old.status<<','<<joint.status<<','<<a.alarm<<','<<(old.gaussian.statistic>old.gaussian.threshold)<<",0\n";
     for(size_t j=0;j<evidence.history.segments.size();++j){const auto& segment=evidence.history.segments[j];const auto& details=supportDetails[index];auto it=std::find_if(details.begin(),details.end(),[&](const auto& d){return d.episode==segment.episode;});
      intervals<<kind<<','<<epoch<<','<<excluded<<','<<segment.episode<<','<<segment.low<<','<<segment.high<<','<<continuing[j]<<','<<(it!=details.end())<<','<<(it==details.end()?NAN:it->low)<<','<<(it==details.end()?NAN:it->high);vec(intervals,it==details.end()?Eigen::Vector3d::Constant(NAN):it->support);intervals<<'\n';
     }
     if(excluded==baseline.excluded)std::cout<<"RAW_PL "<<kind<<" epoch="<<epoch<<" selected="<<excluded<<" alternative_selected="<<alternative.excluded<<" q="<<q<<" D="<<dimension<<" H="<<evidence.history.segments.size()<<" eta="<<old.eta<<" nominal="<<old.nominal.transpose()<<" threshold="<<old.threshold_part.transpose()<<" noise="<<old.noise_part.transpose()<<" eta_part="<<old.eta_part.transpose()<<" history="<<old.history.transpose()<<" joint_fault="<<joint.joint_fault.transpose()<<" center="<<center.transpose()<<" served_old="<<oldPL.head<2>().norm()<<"/"<<oldPL.z()<<" served_joint="<<jointPL.head<2>().norm()<<"/"<<jointPL.z()<<" raw_risk=UNKNOWN qualified=0\n";
    }
   }else{auto receipt=backend.commitEpoch(EpochTransaction(tx),plan);verify(receipt.backend_updates==1,"one prefix native commit");}
   online.committed(tx,plan);prefix.committed(tx,plan);
  }
 }
}
