#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace uwb_imu_pl;
int main(int argc,char** argv) {
  if(argc!=3) { std::cerr<<"usage: gate_d_development CONFIG OUTPUT_DIR\n";return 2; }
  std::uint64_t commits=0, marginalized=0;
  try {
    const auto cfg=IntegrityConfigLoader::load(argv[1]);
    RunLogger logger(argv[2],false,true);
    logger.writeResolvedConfig(cfg.resolved_yaml);
    const std::string execution_command = bindExecutionCommandArguments(
        std::string(argv[0]) + " " + argv[1] + " " + argv[2],
        {{"config_path", argv[1]}, {"run_directory", argv[2]},
         {"fde_profile", toString(cfg.resolved_scope.profile)},
         {"fixed_lag_epochs", std::to_string(cfg.incremental.fixed_lag_epochs)},
         {"seed", std::to_string(cfg.seed)}, {"trajectory", "static"},
         {"fault_mode", "none"}, {"fault_anchor_id", "NONE"},
         {"fault_magnitude_m", "0"}, {"packet_loss_prob", "0"},
         {"nlos_probability", "0"}, {"enable_run_logging", "true"},
         {"write_residuals", "false"}, {"write_timing", "true"},
         {"write_global_diagnostics",
          cfg.output.write_global_diagnostics ? "true" : "false"},
         {"output_root", cfg.output.root}});
    logger.writeManifest(makeRunManifest(
        cfg,"development",true,execution_command));
    std::ofstream audit(std::string(argv[2])+"/fixed_lag_commits.csv");
    audit<<std::setprecision(17)<<"epoch,backend_updates,marginalization_count,wall_ms\n";
    IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
    NavigationState initial; initial.position_world_m={0,0,1};
    estimator.initialize(initial,cfg.realtime.prior_sigmas);
    ImuMeasurement boundary; boundary.specific_force_mps2={0,0,cfg.imu.gravity_mps2};
    estimator.ingestImu(boundary);
    auto batch_for=[&](int epoch) {
      UwbBatch b; b.id=BatchId(epoch); b.timestamp=TimestampNs(epoch*50000000LL);
      for(const auto& a:cfg.anchors) {
        UwbMeasurement m; m.id=MeasurementId(epoch*100+a.id.value()); m.factor_id=FactorId(m.id.value());
        m.timestamp=b.timestamp; m.anchor_id=a.id; m.anchor_position_m=a.position_world_m;
        m.sigma_m=cfg.realtime.range_sigma_m; m.range_m=(initial.position_world_m-a.position_world_m).norm();
        b.measurements.push_back(m);
      }
      return b;
    };
    auto feed=[&](int epoch) {
      for(int sample=1;sample<=10;++sample) {
        ImuMeasurement m; m.id=MeasurementId((epoch-1)*10+sample);
        m.timestamp=TimestampNs((epoch-1)*50000000LL+sample*5000000LL);
        m.specific_force_mps2={0,0,cfg.imu.gravity_mps2}; estimator.ingestImu(m);
      }
    };
    // Explicitly count successful backend commits; this is a development
    // transaction/marginalization diagnostic, not a campaign or FDE verdict.
    for(int epoch=1;epoch<=205;++epoch) {
      feed(epoch); const auto start=std::chrono::steady_clock::now();
      auto tx=estimator.prepareEpoch(batch_for(epoch));
      auto plan=EpochCommitPlan::nominalPlan(tx);
      auto receipt=estimator.commitEpoch(std::move(tx),plan);
      if(receipt.backend_updates!=1) throw std::runtime_error("commit update count is not one");
      ++commits; marginalized=estimator.marginalizationCount();
      audit<<epoch<<','<<receipt.backend_updates<<','<<marginalized<<','
          <<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<'\n';
    }
    feed(206); auto b=batch_for(206);
    auto tx=estimator.prepareEpoch(b);
    auto window=estimator.buildIntegrityWindow(tx,IntegrityWindowRequest{});
    auto imu=estimator.buildPendingFactorBlock(tx,tx.imu_group.id);
    auto bridge=estimator.buildPendingFactorBlock(tx,tx.generic_bridge_group.id);
    HypothesisGeneratorConfig export_config; export_config.lazy_action_entities=false;
    auto models=HypothesisGenerator(export_config).generate(window,tx,ImuFaultSubspaceBuilder().build(tx,imu),bridge);
    FrozenCandidateReplay replay; replay.window=window; replay.actions=models.actions;
    replay.input_attempt_id=206; replay.input_timestamp=b.timestamp; replay.transaction_id=tx.id.value();
    replay.config.materialize_dense_oracle_fields=false;
    writeCandidateReplay(std::string(argv[2])+"/after-205-commits.bin",replay);
    const auto before=estimator.backendUpdateCount();
    estimator.discardEpoch(std::move(tx),{FdeStatus::ModelInvalid,"diagnostic frozen export complete",false});
    if(estimator.backendUpdateCount()!=before) throw std::runtime_error("diagnostic prepare/discard mutated backend");
    RealtimeIntegrityPipeline pipeline(&estimator,IntegrityMonitor(cfg.risk,cfg.snapshot.rank_tolerance,cfg.snapshot.max_condition_number),offlineReplayPublicationLimits());
    auto output=pipeline.processUwbBatch(b);
    logger.writeState(output.state); logger.writeIntegrity(output);
    for(const auto& stage:output.stage_timings) if(stage.status=="EXECUTED") {
      TimingRecord t; t.input_attempt_id=output.diagnostics.input_attempt_id;
      t.transaction_id=output.transaction_id; t.window_id=output.window_id;
      t.timestamp=output.timestamp; t.epoch=estimator.currentEpoch(); t.cold=true;
      t.stage=stage.stage; t.wall_ms=stage.wall_ms; logger.writeTiming(t);
    }
    RunSummary summary; summary.processed=1; summary.committed=output.batch_committed;
    summary.rejected=!output.batch_committed; summary.detail="205 prior explicit nominal commits; one complete pipeline diagnostic";
    logger.writeSummary(summary);
    std::cout<<"{\"status\":\"DEVELOPMENT_ONLY\",\"prior_successful_commits\":"<<commits
        <<",\"marginalization_count\":"<<marginalized<<",\"window_model_valid\":"<<window.model_valid
        <<",\"pipeline_backend_updates\":"<<output.backend_updates<<",\"pipeline_fde_status\":\""<<output.fde_status<<"\"}\n";
    if(commits<=200 || !marginalized || !window.model_valid) return 1;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    std::cout<<"{\"status\":\"INVALID\",\"prior_successful_commits\":"<<commits
        <<",\"marginalization_count\":"<<marginalized<<"}\n"; return 2;
  }
}
