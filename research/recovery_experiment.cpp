#include "scenario_material.hpp"
#include "prior_evidence.hpp"
#include "range_position_bound.hpp"
#include "common_reference_selection.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <chrono>

using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
namespace {
void velocityObservation(EpochTransaction& tx,const Eigen::Vector3d& measured) {
  // An independent simulated odometer input, NOT a frozen velocity column or
  // a truth fault label. Attached to every exclusive UWB group recipe so one
  // copy survives either KEEP or range-source removal. IMU removal never
  // removes this information. Its zero-noise realization is declared below.
  for(auto& g:tx.uwb_groups) {
    const int n=g.raw_covariance.rows();
    Eigen::MatrixXd covariance=Eigen::MatrixXd::Zero(n+3,n+3);
    covariance.topLeftCorner(n,n)=g.raw_covariance;
    covariance.bottomRightCorner<3,3>()=.0004*Eigen::Matrix3d::Identity();
    g.raw_covariance=covariance;
    g.factors.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v',tx.proposed_epoch),measured,
        gtsam::noiseModel::Isotropic::Sigma(3,.02)));
    g.keys.push_back(gtsam::Symbol('v',tx.proposed_epoch));
    g.model_id+="|simulation-only/independent-odometer/v1";
  }
}
void trimGeneratorCovariances(EpochTransaction& tx) {
  const auto trim=[](PendingFactorGroup& g) {
    if(g.kind==FactorKind::UwbBatch && g.raw_covariance.rows()>int(g.source_measurements.size()))
      g.raw_covariance=Eigen::MatrixXd(g.raw_covariance.topLeftCorner(g.source_measurements.size(),g.source_measurements.size()));
  };
  for(auto& g:tx.uwb_groups)trim(g);
  for(auto& h:tx.recoverable_history)for(auto& g:h.groups)trim(g);
}
void restoreIndependentRows(ExclusionAction& action,const LinearizedIntegrityWindow& w) {
  for(auto& b:action.added_blocks)if(b.kind==FactorKind::UwbBatch) {
    const int n=b.residual_whitened.size();
    // Replacement factors share the exact velocity row of the removed source
    // group, including frozen-point residual. Never substitute a known truth.
    const LinearizedFactorBlock* source=nullptr;
    for(const auto& original:w.blocks)if(original.kind==FactorKind::UwbBatch &&
        std::find(action.groups_to_remove.begin(),action.groups_to_remove.end(),original.group_id)!=action.groups_to_remove.end()) {
      // Recover epoch by the local pose column touched by the replacement.
      if((original.jacobian_whitened.cwiseAbs().colwise().sum().array()*
          b.jacobian_whitened.cwiseAbs().colwise().sum().array()).sum()>0.){source=&original;break;}
    }
    if(!source || source->residual_whitened.size()<n+3)throw std::runtime_error("independent row lineage unavailable");
    b.jacobian_whitened.conservativeResize(n+3,Eigen::NoChange);
    b.jacobian_whitened.bottomRows(3)=source->jacobian_whitened.bottomRows(3);
    b.residual_whitened.conservativeResize(n+3);b.residual_whitened.tail(3)=source->residual_whitened.tail(3);
    b.jacobian_raw=b.jacobian_whitened;b.residual_raw=b.residual_whitened;
    b.covariance=Eigen::MatrixXd::Identity(n+3,n+3);b.whitener=b.covariance;
    b.whitening_model_id+="|independent-velocity-retained/v1";
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=5)throw std::invalid_argument("usage: fde_research_recovery CONFIG OUTPUT_CSV normal|uwb|imu independent_velocity=0|1");
    const auto cfg=IntegrityConfigLoader::load(argv[1]);const std::string scenario=argv[3];
    if(scenario!="normal" && scenario!="uwb" && scenario!="imu")throw std::invalid_argument("invalid scenario");
    const bool odometer=std::stoi(argv[4])==1;
    constexpr int epochs=7,fault_epoch=3;
    // Complete categorical generation law: nominal or one UWB batch/anchor,
    // or one raw accel-x batch. Priors equal the old configured numbers, not
    // a retuned hardware rate. No persistent-source merging or combinations.
    std::vector<SimulationAtom> atoms;
    for(int e=1;e<=epochs;++e) {
      for(const auto& a:cfg.anchors)atoms.push_back({"uwb/"+std::to_string(a.id.value())+"/"+std::to_string(e),cfg.fault_models.uwb.prior_probability_bound,.05,e*.05});
      atoms.push_back({"imu/x/"+std::to_string(e),cfg.fault_models.imu.accel_prior_probability_bound,.05,(e-1)*.05+.005});
    }
    const auto prior=PriorEvidence::simulation("simulation-only/raw-range-episode/v1","stationary-input-v1","one-physical-episode-v1",epochs*.05,atoms);
    if(prior.nominalMass()<=0.)throw std::runtime_error("generation law incomplete");
    // These runs are specified conditional witnesses of that categorical law,
    // not frequency estimates. GT is used below ONLY to generate raw inputs
    // and evaluate output error; no truth event enters numerical consumers.
    IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
    estimator.initialize(initialState(),cfg.realtime.prior_sigmas);estimator.ingestImu(sample(cfg,0));
    std::ofstream out(argv[2]);if(!out)throw std::runtime_error("output unavailable");
    out<<std::setprecision(17)<<"epoch,alarm,statistic,threshold,hypotheses,actions,numerical_candidates,post_candidates,pl_candidates,risk_closes,selected,exclusion,bridge,commit,backend_updates,conditional_available,formal_eligible,publication_protected,position_error,hpl,vpl,core_ms,within_40ms,within_50ms,strict_status,strict_risk_closes,strict_reason,reason,retained_current_ranges,uwb_exclusion_correct,position_x,position_y,position_z\n";
    int exclusions=0,after=0;
    for(int epoch=1;epoch<=epochs;++epoch) {
      const auto start=std::chrono::steady_clock::now();
      for(int i=(epoch-1)*10+1;i<=epoch*10;++i) {
        auto imu=sample(cfg,i);
        if(scenario=="imu" && epoch==fault_epoch)imu.specific_force_mps2.x()+=20.;
        estimator.ingestImu(imu);
      }
      auto tx=estimator.prepareEpoch(ranges(cfg,epoch,false,scenario=="uwb" && epoch==fault_epoch?2.25:0.));
      if(odometer)velocityObservation(tx,Eigen::Vector3d::Zero()); // declared independent raw observation
      IntegrityWindowRequest req;req.epochs=cfg.integrity_window.epochs;
      const auto w=estimator.buildIntegrityWindow(tx,req);
      DetectorRiskContext dr;dr.p_fa_per_test=cfg.detector.p_fa_per_test;dr.continuity_horizon_tests=cfg.detector.continuity_horizon_tests;
      const auto detector=JointWindowDetector{}.evaluate(w,dr);
      auto generation_tx=tx;if(odometer)trimGeneratorCovariances(generation_tx);
      HypothesisGeneratorConfig gc;gc.include_persistent_uwb=false;gc.include_ramp_uwb=false;
      gc.max_exclusion_cardinality=cfg.fde.max_exclusion_cardinality;
      gc.uwb_prior_bound=cfg.fault_models.uwb.prior_probability_bound;
      gc.accel_prior_bound=cfg.fault_models.imu.accel_prior_probability_bound;
      gc.gyro_prior_bound=cfg.fault_models.imu.gyro_prior_probability_bound;
      gc.total_hmi_allocation=conservativeRemainingHypothesisRisk(cfg.risk_v2);
      const auto pending=estimator.buildPendingFactorBlock(tx,tx.imu_group.id);
      auto models=HypothesisGenerator(gc).generate(w,generation_tx,ImuFaultSubspaceBuilder{}.buildAnalytic(tx,pending),{});
      std::map<FactorGroupId,EpochTransaction> materials;materials.emplace(tx.imu_group.id,tx);
      for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)if(g.kind==FactorKind::CombinedImu)materials.emplace(g.id,historicalEpisodeTransaction(h));
      // A removed factor does not remove its final raw sample from the next
      // trapezoid. Enumerate latent physical episodes from retained RAW history,
      // independently of selected graph groups and without truth labels.
      for(const auto& history:tx.recoverable_history) {
        if(history.raw_imu_slice.empty() || history.raw_imu_slice.back().timestamp!=tx.raw_imu_slice.front().timestamp)continue;
        for(int axis=0;axis<6;++axis) {
          const auto sensor=axis<3?SensorType::ImuAccelerometer:SensorType::ImuGyroscope;
          bool present=false;for(const auto& m:models.modes)
            present=present || (m.sensor==sensor && m.axis==axis%3 && m.onset_time==history.begin);
          if(present)continue;
          auto it=std::find_if(models.modes.begin(),models.modes.end(),[&](const auto& m){return m.sensor==sensor && m.axis==axis%3;});
          if(it==models.modes.end())throw std::runtime_error("missing raw episode template");
          auto mode=*it;const auto id=models.modes.size()+1;
          mode.id=FaultModeId(id);mode.onset_epoch=history.previous_epoch;mode.onset_time=history.begin;
          mode.physical_source_id+="|raw-boundary-carry";
          auto unit=models.units.at(it->id.value()-1);unit.id=FaultUnitId(id);
          unit.epoch_begin=history.previous_epoch;unit.epoch_end=history.proposed_epoch+1;
          unit.time_begin=TimestampNs(history.begin.value()+5000000);unit.time_end=TimestampNs(history.end.value()+5000000);
          unit.physical_source_id=mode.physical_source_id;
          FaultHypothesisV2 hypothesis;hypothesis.id=HypothesisId(models.hypotheses.size()+1);
          hypothesis.units={unit.id};hypothesis.modes={mode.id};hypothesis.prior_probability_bound=mode.prior_probability_bound;
          hypothesis.p_md_allocation=gc.imu_p_md;
          models.modes.push_back(std::move(mode));models.units.push_back(std::move(unit));models.hypotheses.push_back(std::move(hypothesis));
        }
      }
      // Preserve the declared total allocation when extending raw support.
      const double allocation=conservativeEqualRiskAllocation(gc.total_hmi_allocation,models.hypotheses.size());
      for(auto& h:models.hypotheses)h.hmi_allocation=allocation;
      // Keep a conservative six-axis registry/priors even though the declared
      // generator only corrupts accel x. Never lower configured prior values.
      for(auto& mode:models.modes)if(mode.sensor!=SensorType::Uwb) {
        mode.raw_group_maps.clear();mode.affected_groups.clear();mode.affected_measurements.clear();
        // IMU generator onset_epoch is the PREVIOUS state epoch, unlike UWB
        // occurrence epoch; axis metadata is sensor-local xyz, not 0..5.
        RawImuEpisode e{"raw-batch-v1",TimestampNs(mode.onset_time.value()+5000000),TimestampNs(mode.onset_time.value()+55000000),
            mode.axis+(mode.sensor==SensorType::ImuGyroscope?3:0)};
        for(const auto& b:w.blocks) {
          const auto it=materials.find(b.group_id);
          if(b.kind!=FactorKind::CombinedImu || it==materials.end())continue;
          const auto r=episodeResponse(it->second,b,e);
          if(r.affected_trapezoids){mode.raw_group_maps.emplace(b.group_id,r.raw);mode.affected_groups.push_back(b.group_id);
            for(const auto& sample:it->second.raw_imu_slice)if(!(sample.timestamp<e.begin) && sample.timestamp<e.end &&
                std::find(mode.affected_measurements.begin(),mode.affected_measurements.end(),sample.id)==mode.affected_measurements.end())mode.affected_measurements.push_back(sample.id);
          }
        }
      }
      // Raw maps must include independent observation rows explicitly. Never
      // let a covariance/recipe adaptation silently change their dimensions.
      for(const auto& mode:models.modes)for(const auto& map:mode.raw_group_maps) {
        const auto block=std::find_if(w.blocks.begin(),w.blocks.end(),[&](const auto& b){return b.group_id==map.first;});
        if(block==w.blocks.end() || map.second.rows()!=block->residual_whitened.size() || map.second.cols()!=mode.parameter_dimension)
          throw std::runtime_error("research raw map row/dimension lineage mismatch");
      }
      for(auto& unit:models.units)for(const auto& mode:models.modes)if(unit.id.value()==mode.id.value()){unit.affected_groups=mode.affected_groups;unit.affected_measurements=mode.affected_measurements;}
      const auto evidence=HypothesisEvidenceEvaluator{}.evaluateAll(w,models.modes,&models.hypotheses,detector.squared_threshold);
      if(!detector.passed)for(const auto& ev:evidence) {
        const auto& h=models.hypotheses.at(ev.hypothesis.value()-1);
        const auto& m=models.modes.at(h.modes.front().value()-1);
        if(m.sensor==SensorType::ImuAccelerometer && m.axis==0 && m.onset_epoch+1==std::size_t(epoch))
          std::cerr<<"epoch="<<epoch<<" raw_accel_x_profile="<<ev.profile_j<<" conditioned="<<ev.conditioned_statistic<<" plausible="<<ev.plausible<<" rank="<<ev.monitorability.rank<<" reason="<<ev.monitorability.reason<<'\n';
      }
      std::vector<ExclusionAction> actions;
      if(detector.passed){ExclusionAction keep;keep.id=ExclusionActionId(1);keep.action_model_id="KEEP_ALL";actions.push_back(keep);}
      else {
        actions=HypothesisGenerator(gc).actionsForPlausibleSet(w,generation_tx,&models,evidence);
        for(auto& a:actions) {
          if(odometer)restoreIndependentRows(a,w);
          if(a.bridge_mode!=BridgeMode::None) {
            // Exact stationary generator satisfies the configured a/omega box;
            // this source is a physical-process proof, not a calibration name.
            a.model_error_validated=true;a.model_error_record="simulation-only/stationary-a=omega=0/v1";
          }
        }
      }
      RankUpdateConfig rc;rc.max_linearization_step_norm=cfg.integrity_window.max_linearization_step_norm;
      RankUpdateEvaluator evaluator(rc);const auto base=evaluator.factorizeOnce(w,actions);
      const auto position_set=rangePositionSet(tx.uwb_batch);
      std::vector<CandidateEvaluation> candidates;std::map<std::uint64_t,std::uint64_t> proof_ids;
      int numerical=0,post=0,pl_count=0;
      for(const auto& a:actions) {
        auto c=evaluator.evaluate(base,a);numerical+=c.valid;
        if(!c.valid)std::cerr<<"epoch="<<epoch<<" action="<<a.id.value()<<" candidate_reason="<<c.reason<<" step="<<c.state_increment.norm()<<'\n';
        if(c.valid) {
          const auto d=JointWindowDetector{}.evaluateCandidate(w,c,dr);c.post_detector_passed=d.passed;post+=d.passed;
          if(d.passed && position_set.valid) {
            const Eigen::Vector3d mean=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)).translation()+w.protected_state_map*c.state_increment;
            c.pl_xyz_m=relativeTransferBound(position_set,
                tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)).translation(),
                w.protected_state_map*c.state_increment);c.hpl_m=c.pl_xyz_m.head<2>().norm();c.vpl_m=c.pl_xyz_m.z();
            // Independent simulation range-set proof: never a production PL
            // registry token and never passed to retain/mint/publication.
            proof_ids.emplace(a.id.value(),tx.id.value()*1000000+a.id.value());++pl_count;
          }
        }
        candidates.push_back(std::move(c));
      }
      BoundRiskContextV1 bound;bound.window_id=w.id;bound.version=w.version;bound.valid_from=tx.end;bound.valid_until=tx.end;
      bound.scope_id="simulation-only/one-episode-v1";bound.manifest_id="simulation-only/raw-batch-v1";bound.model_id="simulation-only/deterministic-range-union-v1";
      bound.risk_contract_id=riskContractIdentityV1(cfg.risk_v2,models.hypotheses);
      auto& input=bound.inputs;input.omitted_scope_complete=prior.nominalMass()>0.;input.omitted_event_bound_known=input.omitted_scope_complete;input.omitted_event_bound_validated=input.omitted_scope_complete;
      input.envelope_event_bound_known=true;input.envelope_event_bound_validated=true;
      input.bridge_escape_validated=true;input.history_escape_validated=true;input.model_escape_validated=position_set.valid;
      input.model_formal_eligible=false;
      // Omitted: categorical law has at most one episode. Envelope: exact
      // exhaustive traversal. History/bridge/model: the range union proof
      // bounds current position independent of optimizer/process/history.
      // Thus these are actual structural/conditional proof premises, not a
      // blanket assertion that the original Gaussian model is calibrated.
      for(const auto* id:{"omitted","envelope","bridge","history","model"})bound.evidence[id]={RiskEvidenceDomainV1::SimulationConditional,bound.model_id+"|"+id};
      FdeRiskDecisionV1 risk_result;RiskLedger ledger;FdeDecisionContextV4 context;
      GeneratedActionSnapshotV1 trusted;
      const auto census=censusAndCapActionsV1(actions,actions.size(),&trusted);
      context.v2.action_search=&census.census;context.v2.trusted_generated_actions=&trusted;
      context.v2.max_evaluated_actions=census.max_evaluated_actions;
      context.v2.action_search_lifecycle=census.lifecycle;
      context.v2.v1.protection_proof_ids=&proof_ids;context.v2.v1.risk_result=&risk_result;context.risk_context=&bound;
      context.expected_scope_id=bound.scope_id;context.expected_manifest_id=bound.manifest_id;context.expected_model_id=bound.model_id;context.information_cutoff=tx.end;context.ledger_result=&ledger;
      if(!proof_ids.empty())verifyRangeOutputs(position_set,tx.uwb_batch,
          tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)).translation(),
          w,cfg.risk_v2,models.hypotheses,bound,candidates);
      auto decision=FdeManager{}.decide(detector,models.hypotheses,evidence,&candidates,{},cfg.risk_v2,&context);
      const auto strict_decision=decision;const bool strict_risk=risk_result.complete_bound_closes;
      decision=commonRangeReferenceSelection(decision,position_set,tx.uwb_batch,
          tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)).translation(),
          w,cfg.risk_v2,models.hypotheses,bound,candidates,ledger);
      if(decision.commit_allowed && ledger.closes)risk_result.complete_bound_closes=true;
      if(epoch==fault_epoch) {
        if(ledger.terms.empty())std::cerr<<"risk epoch="<<epoch<<" NOT_RUN (no eligible bounded candidate)\n";
        else std::cerr<<std::setprecision(17)<<"risk epoch="<<epoch<<" total="<<ledger.charged_total<<" budget="<<ledger.budget<<"\n";
        for(const auto& term:ledger.terms)std::cerr<<term.id<<"="<<term.value<<" status="<<toString(term.status)<<" source="<<term.source<<"\n";
      }
      EpochCommitPlan plan;
      bool selected=decision.commit_allowed && decision.integrity_available && decision.selected_action.has_value();
      double hpl=std::numeric_limits<double>::infinity(),vpl=hpl;
      if(selected) {
        const auto& a=*decision.selected_action;plan=EpochCommitPlan::nominalPlan(tx);
        if(!a.groups_to_remove.empty()){std::cerr<<"selected epoch="<<epoch<<" action="<<a.id.value()<<" groups_removed=";for(const auto g:a.groups_to_remove)std::cerr<<g.value()<<',';std::cerr<<" bridge="<<int(a.bridge_mode)<<'\n';}
        plan.action_id=a.id;plan.fde_status=decision.status;plan.bridge_mode=a.bridge_mode;
        // Pending exclusions choose a different add recipe; they have never
        // entered iSAM and therefore cannot appear in committed removal slots.
        for(const auto id:a.groups_to_remove) {
          bool pending=id==tx.imu_group.id;
          for(const auto& g:tx.uwb_groups)pending=pending || id==g.id;
          if(!pending)plan.groups_to_remove.push_back(id);
        }
        for(const auto id:a.groups_to_remove)plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),id),plan.groups_to_add.end());
        for(const auto id:a.groups_to_add)if(std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),id)==plan.groups_to_add.end())plan.groups_to_add.push_back(id);
        for(const auto& c:candidates)if(c.selected){hpl=c.hpl_m;vpl=c.vpl_m;}
      }else {plan=EpochCommitPlan::nominalPlan(tx);plan.best_effort_integrity_unavailable=true;}
      // Evaluation ONLY, after the selector and final add recipe are fixed.
      // Check actual source lineage rather than treating any exclusion as a
      // correct one. The injected label never enters a numerical consumer.
      std::vector<MeasurementId> retained_ranges;
      for(const auto& group:tx.uwb_groups)
        if(std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),group.id)!=plan.groups_to_add.end())
          retained_ranges.insert(retained_ranges.end(),group.source_measurements.begin(),group.source_measurements.end());
      int correct=-1;
      if(scenario=="uwb" && epoch==fault_epoch) {
        const auto bad=tx.uwb_batch.measurements.front().id;
        correct=selected && decision.selected_action && !decision.selected_action->groups_to_remove.empty() &&
          retained_ranges.size()==tx.uwb_batch.measurements.size()-1 &&
          std::find(retained_ranges.begin(),retained_ranges.end(),bad)==retained_ranges.end();
      }
      // Ordinary simulation transaction: no protected token or certification.
      const auto receipt=estimator.commitEpoch(std::move(tx),plan);
      const bool exclusion=selected && decision.selected_action && !decision.selected_action->groups_to_remove.empty();exclusions+=exclusion;
      // Rebind the independent position set to the actual nonlinear committed
      // mean. A finite pre-commit bound is not a final-output certificate.
      const auto final_pl=transferBound(position_set,estimator.currentState().position_world_m);
      if(selected){hpl=final_pl.head<2>().norm();vpl=final_pl.z();}
      const bool available=selected && hpl<=cfg.risk_v2.horizontal_alert_limit_m && vpl<=cfg.risk_v2.vertical_alert_limit_m;
      if(exclusions && epoch>fault_epoch && available)++after;
      const double error=(estimator.currentState().position_world_m-Eigen::Vector3d(0,0,1)).norm();
      const double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      const auto position=estimator.currentState().position_world_m;
      out<<epoch<<','<<!detector.passed<<','<<detector.squared_parity_statistic<<','<<detector.squared_threshold<<','<<models.hypotheses.size()<<','<<actions.size()<<','<<numerical<<','<<post<<','<<pl_count<<','<<risk_result.complete_bound_closes<<','<<selected<<','<<exclusion<<','<<int(plan.bridge_mode)<<",1,"<<receipt.backend_updates<<','<<available<<",0,0,"<<error<<','<<hpl<<','<<vpl<<','<<wall<<','<<(available && wall<=40.)<<','<<(available && wall<=50.)<<','<<int(strict_decision.status)<<','<<strict_risk<<','<<std::quoted(strict_decision.reason)<<','<<std::quoted(decision.reason)<<','<<retained_ranges.size()<<','<<correct<<','<<position.x()<<','<<position.y()<<','<<position.z()<<'\n';
    }
    std::cout<<"CONDITIONAL scenario="<<scenario<<" independent_velocity="<<odometer<<" exclusions="<<exclusions<<" subsequent_selected_commits="<<after<<" formal_eligible=false publication_protected=false\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
