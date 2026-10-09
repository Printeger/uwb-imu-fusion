#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace uwb_imu_pl;
int main(int argc,char** argv) {
  if(argc<3 || argc>6) { std::cerr<<"usage: candidate_replay SNAPSHOT OUTPUT_CSV [WORKERS=4] [REPEATS=1] [fast|oracle|imu-power]\n"; return 2; }
  try {
    auto replay=readCandidateReplay(argv[1]);
    const int workers=argc>3?std::stoi(argv[3]):4, repeats=argc>4?std::stoi(argv[4]):1;
    const bool oracle=argc>5 && std::string(argv[5])=="oracle";
    const bool imu_power=argc>5 && std::string(argv[5])=="imu-power";
    if(argc>5 && !oracle && !imu_power && std::string(argv[5])!="fast") throw std::runtime_error("invalid replay mode");
    if((workers!=1 && workers!=4)||repeats<1) throw std::runtime_error("invalid replay arguments");
    std::ofstream out(argv[2]); if(!out) throw std::runtime_error("cannot open replay output");
    if(imu_power) {
      // Offline sensitivity diagnostic only. Never supplies an action or
      // truth label to selection, and never changes the detector/step gate.
      if(repeats!=1 || workers!=1 || !replay.window.numerics ||
          !replay.window.numerics->valid || replay.window.state_layout.size()<2)
        throw std::runtime_error("imu-power needs one valid frozen snapshot");
      const auto& w=replay.window;const auto& n=*w.numerics;
      const auto latest=std::max_element(w.state_layout.begin(),w.state_layout.end(),
          [](const auto& a,const auto& b){return a.epoch<b.epoch;});
      const auto previous=std::find_if(w.state_layout.begin(),w.state_layout.end(),
          [&](const auto& l){return l.epoch+1==latest->epoch;});
      if(previous==w.state_layout.end() || latest->dimension!=15 || previous->dimension!=15)
        throw std::runtime_error("imu-power state ordering unavailable");
      out<<std::setprecision(17)<<"attempt,group_id,axis,unit_fault_whitened_norm,unit_fault_parity_norm,lambda_per_amplitude_squared,unit_fault_state_step,unit_fault_protected_shift,observed_statistic,step_gate,dof\n";
      std::size_t count=0;
      for(const auto& block:w.blocks) {
        if(block.kind!=FactorKind::CombinedImu || block.jacobian_raw.rows()!=15 ||
            block.jacobian_raw.cols()!=w.H.cols() ||
            block.jacobian_raw.middleCols(latest->column_offset,15).norm()==0.)continue;
        Eigen::Matrix<double,15,6> raw=Eigen::Matrix<double,15,6>::Zero();
        raw.topRows<9>()=block.jacobian_raw.block(0,previous->column_offset+9,9,6);
        const auto block_index=static_cast<std::size_t>(&block-w.blocks.data());
        Eigen::MatrixXd response=Eigen::MatrixXd::Zero(w.H.rows(),6);
        response.middleRows(n.block_row_offsets.at(block_index),15)=block.whitener*raw;
        const auto& v=*n.spectral_vectors;
        const Eigen::MatrixXd state=v*n.spectral_inverse_squared.asDiagonal()*v.transpose()*(w.H.transpose()*response);
        const Eigen::MatrixXd parity=response-w.H*state;
        for(int axis=0;axis<6;++axis) {
          out<<replay.input_attempt_id<<','<<block.group_id.value()<<','<<axis<<','
              <<response.col(axis).norm()<<','<<parity.col(axis).norm()<<','
              <<parity.col(axis).squaredNorm()<<','<<state.col(axis).norm()<<','
              <<(w.protected_state_map*state.col(axis)).norm()<<','<<n.statistic<<','
              <<replay.config.max_linearization_step_norm<<','<<n.dof<<'\n';
        }
        ++count;
      }
      if(count!=1)throw std::runtime_error("imu-power needs exactly one current Combined IMU block");
      return 0;
    }
    out<<std::setprecision(17)<<"repeat,input_attempt_id,window_id,action_id,valid,reason,wall_ms,statistic,rank,dof,condition_number,condition_value_kind,condition_lower_bound,condition_upper_bound,certificate_margin,logdet,step_norm,slow_path,numerical_path,fallback_reason,cache_hits,scratch_reuse_count,matrix_free_step_rejected\n";
    RankUpdateEvaluator evaluator(replay.config);
    CandidateWorkerPool pool(4);
    for(int repeat=0;repeat<repeats;++repeat) {
      const auto start=std::chrono::steady_clock::now();
      const auto base=oracle ? BaseCandidateKernel{} : evaluator.factorizeOnce(replay.window, replay.actions);
      std::vector<CandidateEvaluation> results(replay.actions.size());
      pool.run(replay.actions.size(), workers,
          [&](std::size_t i, std::size_t, RankUpdateScratch& scratch) {
        results[i] = oracle
            ? DenseCandidateOracle(replay.config).evaluate(replay.window,replay.actions[i])
            : evaluator.evaluate(base,replay.actions[i],&scratch);
      });
      const double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      std::sort(results.begin(),results.end(),[](const auto& a,const auto& b){return a.action.id<b.action.id;});
      for(const auto& c:results) {
        out<<repeat<<','<<replay.input_attempt_id<<','<<replay.window.id.value()<<','
            <<c.action.id.value()<<','<<c.valid<<','<<std::quoted(c.reason)<<','<<c.wall_ms<<','<<c.statistic<<','
            <<c.rank<<','<<c.dof<<',';
        if (c.diagnostics.condition_value_kind == "EXACT_SVD" ||
            c.diagnostics.condition_value_kind == "EXACT_BASE") out<<c.condition_number;
        out<<','<<std::quoted(c.diagnostics.condition_value_kind)<<','
            <<c.diagnostics.condition_lower_bound<<','<<c.diagnostics.condition_upper_bound<<','
            <<c.diagnostics.certificate_margin<<','<<c.information_logdet<<','<<c.state_increment.norm()<<','
            <<c.exact_slow_path<<','<<std::quoted(c.diagnostics.numerical_path)<<','
            <<std::quoted(c.diagnostics.fallback_reason)<<','<<c.diagnostics.cache_hits<<','
            <<c.diagnostics.scratch_reuse_count<<','<<c.diagnostics.matrix_free_step_rejected<<'\n';
      }
      std::cout<<"repeat="<<repeat<<" candidates="<<results.size()<<" wall_ms="<<wall<<'\n';
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 2; }
}
