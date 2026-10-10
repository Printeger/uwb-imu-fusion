#include "scenario_material.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
int main(int argc,char** argv) {
 try {
  if(argc!=6)throw std::runtime_error("CONFIG OUTPUT normal|joint2 reference|optimized COUNT");
  const auto cfg=IntegrityConfigLoader::load(argv[1]);const bool joint=std::string(argv[3])=="joint2";
  const bool optimized=std::string(argv[4])=="optimized";const int count=std::stoi(argv[5]);
  if(count<1 || count>35)throw std::runtime_error("invalid attempt count");
  IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
  estimator.initialize(initialState(),cfg.realtime.prior_sigmas);estimator.ingestImu(sample(cfg,0));
  for(int e=1;e<7;++e){auto tx=prepare(estimator,cfg,e);auto p=EpochCommitPlan::nominalPlan(tx);estimator.commitEpoch(std::move(tx),p);}
  auto tx=prepare(estimator,cfg,7);IntegrityWindowRequest req;req.epochs=7;const auto window=estimator.buildIntegrityWindow(tx,req);
  HypothesisGeneratorConfig gc;gc.include_persistent_uwb=gc.include_ramp_uwb=false;gc.double_faults_enabled=joint;
  gc.total_hmi_allocation=conservativeRemainingHypothesisRisk(cfg.risk_v2);
  const auto block=estimator.buildPendingFactorBlock(tx,tx.imu_group.id);
  const auto original=HypothesisGenerator(gc).generate(window,tx,ImuFaultSubspaceBuilder{}.buildAnalytic(tx,block),{});
  std::map<FactorGroupId,EpochTransaction> materials;materials.emplace(tx.imu_group.id,tx);
  for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)if(g.kind==FactorKind::CombinedImu)materials.emplace(g.id,historicalEpisodeTransaction(h));
  CandidateWorkerPool workers(1);
  std::ofstream out(argv[2]);out<<std::setprecision(17)<<"attempt,modes,hypotheses,response_calls,empty_support,integrations,model_ms,evidence_ms,flat_ms,total_ms,mode_fingerprint,profile_sum,hpl,vpl,pl_status,formal_eligible\n";
  for(int attempt=1;attempt<=count;++attempt) {
    const auto start=std::chrono::steady_clock::now();double model_ms=0.,evidence_ms=0.,flat_ms=0.,hpl=0.,vpl=0.,profiles=0.;
    std::uint64_t fingerprint=0;int status=0;std::size_t calls=0,empty=0,integrations=0;
    { // Destruction of the seal, arena, maps and results is inside total_ms.
      auto models=original;
      for(auto& mode:models.modes)if(mode.sensor!=SensorType::Uwb) {
        mode.raw_group_maps.clear();mode.affected_groups.clear();
        RawImuEpisode e{"raw-batch-v1",TimestampNs(mode.onset_time.value()+5000000),TimestampNs(mode.onset_time.value()+55000000),mode.axis+(mode.sensor==SensorType::ImuGyroscope?3:0)};
        for(const auto& b:window.blocks)if(b.kind==FactorKind::CombinedImu) {
          const auto response=episodeResponse(materials.at(b.group_id),b,e,1e-5,optimized);
          ++calls;integrations+=response.integrations;empty+=response.affected_trapezoids==0;
          if(response.affected_trapezoids){mode.raw_group_maps.emplace(b.group_id,response.raw);mode.affected_groups.push_back(b.group_id);}
        }
      }
      std::ostringstream semantic;semantic<<std::setprecision(17);
      for(const auto& mode:models.modes)for(const auto& map:mode.raw_group_maps){if(count==1)semantic<<"mode "<<mode.id.value()<<" group "<<map.first.value()<<'\n'<<map.second<<'\n';}
      fingerprint=faultModeSetFingerprint(models.modes);
      auto handle=freezeIntegrityWindowCopy(window);auto admission=admitFrozenIntegrityWindow(handle);if(!admission)throw std::runtime_error(admission.reason);
      const auto model_end=std::chrono::steady_clock::now();model_ms=std::chrono::duration<double,std::milli>(model_end-start).count();
      DetectorRiskContext dr;dr.p_fa_per_test=cfg.detector.p_fa_per_test;dr.continuity_horizon_tests=cfg.detector.continuity_horizon_tests;
      const auto detector=JointWindowDetector{}.evaluate(admission.window(),dr);
      AttemptProofArena arena;std::shared_ptr<const FrozenHypothesisNumerics> frozen;std::shared_ptr<const FrozenHypothesisDualNumerics> dual;
      auto evidence=HypothesisEvidenceEvaluator{}.evaluateAll(admission,models.modes,&models.hypotheses,detector.squared_threshold,&frozen,&dual,nullptr,&arena);
      for(const auto& ev:evidence){profiles+=ev.profile_j;
        if(count==1)semantic<<"evidence "<<ev.hypothesis.value()<<' '<<ev.profile_j<<' '<<ev.conditioned_statistic<<' '<<ev.all_in_statistic<<' '<<ev.plausible<<' '<<ev.monitorability.rank<<' '<<ev.monitorability.monitorable<<' '<<ev.monitorability.sigma_min<<' '<<ev.monitorability.sigma_max<<' '<<ev.monitorability.condition_number<<'\n'<<ev.estimated_fault<<'\n'<<ev.monitorability.protected_slopes<<'\n'<<ev.monitorability.reason<<' '<<ev.explained_energy<<' '<<ev.log_evidence<<' '<<ev.profile_valid<<' '<<int(ev.unit_kind)<<' '<<ev.parameter_dimension<<'\n'<<ev.fault_gram<<'\n';}
      const auto evidence_end=std::chrono::steady_clock::now();evidence_ms=std::chrono::duration<double,std::milli>(evidence_end-model_end).count();
      ExclusionAction keep;keep.id=ExclusionActionId(1);RankUpdateEvaluator evaluator;
      auto candidate=evaluator.evaluate(admission,evaluator.factorizeOnce(admission,{keep}),keep);
      ProtectionLevelSharedContext shared;
      for(const auto& m:models.modes){auto& map=shared.mode_maps[m.id.value()];map=Eigen::MatrixXd::Zero(window.H.rows(),m.parameter_dimension);int offset=0;
       for(const auto& b:window.blocks){const auto it=m.raw_group_maps.find(b.group_id);if(it!=m.raw_group_maps.end())map.middleRows(offset,b.residual_whitened.size())=b.whitener*it->second;offset+=b.residual_whitened.size();}}
      ProtectionLevelV2Result result;ProtectionLevelV2ProofV1 proof;
      FlatProtectionCandidateV1 job;job.candidate=&candidate;const auto post=JointWindowDetector{}.evaluateCandidate(admission.window(),candidate,dr);job.detector=&post;job.hypotheses=&models.hypotheses;job.shared=&shared;job.result=&result;job.computation_audit=&proof;job.proof_arena=&arena;
      std::vector<FlatProtectionCandidateV1> jobs{job};
      ProtectionLevelV2{}.computeSharedFlatBatch(admission,&jobs,cfg.risk_v2,&workers,1,64u*1024u*1024u,false);
      hpl=result.hpl_m;vpl=result.vpl_m;status=int(result.availability);
      std::string reason;const bool proof_ok=validateProtectionLevelV2Proof(candidate,post,models.hypotheses,result,arena,&reason);
      if(!proof_ok && result.model_valid)throw std::runtime_error("finite final PL validation: "+reason);
      if(!proof_ok)std::cerr<<"attempt="<<attempt<<" terminal="<<result.reason<<" candidate="<<candidate.reason<<" validation="<<reason<<'\n';
      if(count==1)semantic<<"candidate "<<candidate.valid<<' '<<candidate.reason<<'\n'<<candidate.state_increment<<'\n'<<(window.protected_state_map*candidate.covarianceTimes(window.protected_state_map.transpose()))<<'\n';
      if(count==1)semantic<<"PL "<<result.pl_xyz_m.transpose()<<' '<<result.nominal_component_m.transpose()<<' '<<result.fault_component_m.transpose()<<' '<<result.bridge_component_m.transpose()<<' '<<result.hpl_m<<' '<<result.vpl_m<<' '<<result.allocated_outcome_risk<<' '<<result.risk_budget_valid<<' '<<result.model_valid<<' '<<result.formal_eligible<<' '<<int(result.availability)<<' '<<result.hypothesis_tail_used<<' '<<result.axis_tail_used<<' '<<result.fault_multiplier_used<<' '<<result.noncentrality_used<<' '<<result.reason<<'\n';
      if(count==1 && attempt==1){std::ofstream comparison(std::string(argv[2])+".semantics.txt");comparison<<semantic.str();}
      flat_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-evidence_end).count();
    }
    const double total=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    out<<attempt<<','<<original.modes.size()<<','<<original.hypotheses.size()<<','<<calls<<','<<empty<<','<<integrations<<','<<model_ms<<','<<evidence_ms<<','<<flat_ms<<','<<total<<','<<fingerprint<<','<<profiles<<','<<hpl<<','<<vpl<<','<<status<<",0\n";
  }
  std::cout<<"VERIFIED research timing attempts="<<count<<" optimized="<<optimized<<"; no production qualification\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
