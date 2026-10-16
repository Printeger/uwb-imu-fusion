#include "rolling_recovery.hpp"
#include "numerical_candidate.hpp"
#include "sc6_provider.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool v,const char* m){if(!v)throw std::runtime_error(m);}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 const std::vector<Eigen::Vector3d> positions{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
 for(bool repeated:{false,true}){
  Sc6Provider src("config/sc7.yaml","S0",20261010,5);auto config=src.cfg;config.anchors.clear();
  auto policy=resolveRollingPolicy(src.law,config);
  IncrementalUwbImuEstimator backend(config,src.lever);backend.initialize(src.initial,config.realtime.prior_sigmas);
  backend.ingestImu(src.imu.front());RollingRecovery recovery(src.initial,policy,true,true);recovery.enableNumericalRecovery(true);
  RollingMonitorView monitor;monitor.open=true;monitor.state.nominal_possible=true;monitor.matches=[](const EpochTransaction&){return true;};
  const AnchorId bad((uint64_t(1)<<48)+7);
  std::deque<ImuMeasurement> replay{src.imu.front()};std::deque<RollingRawState> capsules;
  for(size_t k=1;k<=5;++k){
   for(size_t i=(k-1)*10+1;i<=k*10;++i){backend.ingestImu(src.imu[i]);replay.push_back(src.imu[i]);}
   UwbBatch batch;batch.id=BatchId(9000+k);batch.timestamp=TimestampNs(k*50000000LL);
   const auto truth=src.truth(k*10,true);const auto tag=Eigen::Vector3d(truth.position_world_m+truth.q_world_body*src.lever);
   for(size_t i=0;i<5;++i){if(k==2 && i==0)continue;
    UwbMeasurement m;m.id=MeasurementId(100000+k*100+i);m.factor_id=FactorId(m.id.value());
    m.anchor_id=AnchorId(bad.value()+100*i);m.timestamp=batch.timestamp;m.anchor_position_m=positions[i];
    m.range_m=(tag-positions[i]).norm()+(k==1 && i==0?2.25:0);m.sigma_m=.01;batch.measurements.push_back(m);
   }
   if(repeated && k==1){auto second=batch.measurements.front();second.id=MeasurementId(200000);second.factor_id=FactorId(200000);batch.measurements.push_back(second);}
   // Reordering changes representative row but never held physical identity.
   if(k>=3)std::rotate(batch.measurements.begin(),batch.measurements.begin()+2,batch.measurements.end());
   if(k==1){auto retained=retainRanges(batch,0);require(retained.measurements.size()==4,"physical exclusion did not remove every row");
    auto candidate=numericalUwbCandidate(batch,0);require(candidate.functional_post_test && candidate.functional.gaussian.dof==1 && !candidate.gaussian_reference_qualified,"four-row rank/post qualification");
    require((candidate.functional.tag-tag).norm()<1e-5,"healthy tetrahedral root failed");}
   auto tx=backend.prepareEpoch(batch);auto action=recovery.prepare(tx,monitor,replay,capsules,config,true);
   require(action.excluded_source && *action.excluded_source==bad,"held exclusion followed row index");
   if(k==1)require(action.uwb_alarm && action.excluded==0 && action.actions==12,"dynamic source action census");
   if(k==2)require(action.excluded==-1,"absent source replaced a healthy row");
   if(k>=3)require(action.excluded>=0 && batch.measurements[action.excluded].anchor_id==bad,"returned source identity lost");
   for(const auto& g:tx.uwb_groups)if(std::find(action.plan.groups_to_add.begin(),action.plan.groups_to_add.end(),g.id)!=action.plan.groups_to_add.end())
    for(auto id:g.source_measurements)for(const auto& m:batch.measurements)if(m.id==id)require(m.anchor_id!=bad,"bad source entered chosen factor");
   const auto saved=tx;auto count=backend.backendUpdateCount();auto receipt=backend.commitEpoch(std::move(tx),action.plan);
   recovery.committed(action,saved,receipt,backend.factorLedger());
   require(receipt.backend_updates==1 && backend.backendUpdateCount()==count+1,"not exactly one native update");
  }
 }
 std::cout<<"SC9 recovery PASS native2x5 commits, five physical anchors, repeated-source removal, four-row post/rank, sparse64, reorder/disconnect/return; functional fixture only\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
