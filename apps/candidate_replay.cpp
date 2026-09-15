#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

using namespace uwb_imu_pl;
int main(int argc,char** argv) {
  if(argc<3 || argc>6) { std::cerr<<"usage: candidate_replay SNAPSHOT OUTPUT_CSV [WORKERS=4] [REPEATS=1] [fast|oracle]\n"; return 2; }
  try {
    auto replay=readCandidateReplay(argv[1]);
    const int workers=argc>3?std::stoi(argv[3]):4, repeats=argc>4?std::stoi(argv[4]):1;
    const bool oracle=argc>5 && std::string(argv[5])=="oracle";
    if(argc>5 && std::string(argv[5])!="oracle" && std::string(argv[5])!="fast") throw std::runtime_error("invalid replay mode");
    if((workers!=1 && workers!=4)||repeats<1) throw std::runtime_error("invalid replay arguments");
    std::ofstream out(argv[2]); if(!out) throw std::runtime_error("cannot open replay output");
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
