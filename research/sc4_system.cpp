#include "sc4_system.hpp"
#include "scenario_material.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <thread>
#include <sys/resource.h>
#include <tbb/global_control.h>

namespace uwb_imu_pl::research {
namespace {
double duration(Sc4Clock::time_point a,Sc4Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int64_t stamp(Sc4Clock::time_point t,Sc4Clock::time_point origin){return std::chrono::duration_cast<std::chrono::nanoseconds>(t-origin).count();}
void delivered(Sc4Output& out,Sc4Clock::time_point origin){
 // Caller owns the complete packet: process() locals and serialization
 // temporaries have now been destroyed. Include that final return/cleanup.
 out.timing.output_ns=stamp(Sc4Clock::now(),origin);
 out.timing.full_ms=(out.timing.output_ns-out.timing.start_ns)/1e6;
 out.timing.arrival_to_output_ms=std::max(0.,(out.timing.output_ns-out.timing.release_ns)/1e6);
 out.timing.measurement_age_ms=out.timing.arrival_to_output_ms;
 out.on_time=out.timing.output_ns<=out.timing.release_ns+50000000;
}
template<class Id> std::string ids(const std::vector<Id>& ids){std::ostringstream out;bool first=true;for(auto id:ids){if(!first)out<<'|';first=false;out<<id.value();}return out.str();}
}
Sc4System::Sc4System(Sc4Input input,bool optimized):input_(std::move(input)){
 auto error=sc4InputError(input_);if(!error.empty())throw std::runtime_error(error);
 backend_=std::make_unique<IncrementalUwbImuEstimator>(input_.config,input_.lever);
 backend_->initialize(input_.initial,input_.config.realtime.prior_sigmas);
 backend_->ingestImu(input_.imu.front());imu_cursor_=1;
 if(input_.matched_source){joint_=std::make_unique<Sc3JointCoordinator>(input_.initial,input_.law,input_.batches.front(),18,!optimized,optimized);joint_->append(input_.imu.front());risk_=std::ldexp(2.*6421,-35);}
}
Sc4Output Sc4System::process(size_t index,bool fde,Sc4Clock::time_point release,Sc4Clock::time_point origin){
 Sc4Output out;out.sequence=++sequence_;out.source=input_.source;out.scope=input_.scope;out.cumulative_risk=risk_;out.version=last_version_;out.physical_expected=joint_?481401:0;
 auto start=Sc4Clock::now();out.timing.start_ns=stamp(start,origin);out.timing.release_ns=stamp(release,origin);out.timing.queue_ms=std::max(0.,duration(release,start));
 if(index>=input_.batches.size())throw std::out_of_range("SC4 input sequence");
 const auto& batch=input_.batches[index];
 if(gap_){out.functional="INPUT_GAP_LATCHED";out.bound_status="CONTINUITY_REFUSED";out.state=backend_->currentState();out.epoch=index+1;out.carry=carry_;}
 else{
  auto execute=[&](){
   auto ingest=Sc4Clock::now();
   while(imu_cursor_<input_.imu.size() && !(batch.timestamp<input_.imu[imu_cursor_].timestamp)){
    const auto& m=input_.imu[imu_cursor_++];backend_->ingestImu(m);
    if(joint_ && m.timestamp.value()<=2500000000LL){auto before=joint_->monitor().alarm;joint_->append(m);if(!before && joint_->monitor().alarm)out.timing.alarm_ns=stamp(Sc4Clock::now(),origin);}
   }
   out.timing.ingest_ms=duration(ingest,Sc4Clock::now());
   auto prepare=Sc4Clock::now();auto tx=backend_->prepareEpoch(batch,EpochPreparationOptions{true,true,true,true});
   out.timing.prepare_ms=duration(prepare,Sc4Clock::now());out.epoch=tx.proposed_epoch;out.transaction=tx.id.value();out.history_epochs=tx.recoverable_history.size();
   EpochCommitPlan plan=EpochCommitPlan::nominalPlan(tx);bool jointAction=false;
   auto candidate=Sc4Clock::now();
   if(joint_ && tx.proposed_epoch<=50){out.action=joint_->prepare(tx,input_.ranges[index],input_.lever,input_.config.bridge.generic,fde);
    auto monitor=joint_->monitor();carry_=carry_ || monitor.alarm;carry_end_=monitor.union_end;
    if(!out.action.legal){backend_->discardEpoch(std::move(tx),DiscardReason{out.action.decision.status,out.action.decision.reason,false});gap_=true;out.functional="RECOVERY_REFUSED";out.bound_status="NO_VALID_TRANSACTION";out.state=backend_->currentState();out.carry=carry_;return;}
    plan=sc2CommitPlan(tx,out.action.imu);plan.fde_status=out.action.decision.status;jointAction=true;out.functional=out.action.status;
   }else{
    out.functional=joint_?"CONTINUOUS_UNPROTECTED_AFTER_HORIZON":"RECORD_NONPROTECTED";
    out.bound_status=joint_?"TASK_HORIZON_EXPIRED":"REAL_SOURCE_UNQUALIFIED";
    // The frozen physical bank is retained. Carry never becomes a new prior.
    if(joint_ && carry_ && tx.begin<carry_end_){plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),tx.imu_group.id),plan.groups_to_add.end());plan.groups_to_add.push_back(tx.generic_bridge_group.id);plan.groups_to_add.push_back(tx.generic_bias_continuity_group.id);plan.bridge_mode=BridgeMode::GenericKinematic;out.action.imu.current_bridge=true;}
    if(!joint_){auto a=uwbEpochRaim(batch,fde);out.action.uwb_alarm=a.alarm;out.action.detector_statistic=a.detection_statistic;out.action.detector_threshold=a.detection_threshold;
     if(fde && a.alarm && a.excluded>=0 && a.status=="FROZEN_MODEL_ONLY"){bool found=false;for(const auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()+1==batch.measurements.size() && std::find(g.source_measurements.begin(),g.source_measurements.end(),batch.measurements[a.excluded].id)==g.source_measurements.end()){plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),tx.uwb_groups.front().id),plan.groups_to_add.end());plan.groups_to_add.push_back(g.id);out.action.excluded=a.excluded;found=true;break;}if(!found)throw std::runtime_error("real native replacement missing");}
    }
   }
   out.timing.candidate_ms=duration(candidate,Sc4Clock::now());out.timing.selected_ns=stamp(Sc4Clock::now(),origin);
   std::vector<MeasurementId> inputIds,retainedIds;for(const auto& m:batch.measurements)inputIds.push_back(m.id);
   for(auto id:plan.groups_to_add)for(const auto& g:tx.uwb_groups)if(g.id==id)retainedIds.insert(retainedIds.end(),g.source_measurements.begin(),g.source_measurements.end());
   out.input_ids=ids(inputIds);out.retained_ids=ids(retainedIds);out.removed_groups=ids(plan.groups_to_remove);
   auto retained=tx;auto commit=Sc4Clock::now();auto before=backend_->backendUpdateCount();auto receipt=backend_->commitEpoch(std::move(tx),plan);
   if(receipt.backend_updates!=1 || backend_->backendUpdateCount()!=before+1)throw std::runtime_error("SC4 nonunique commit");
   if(jointAction)joint_->committed(out.action,retained,receipt);
   out.state=backend_->currentState();out.updates=receipt.backend_updates;out.version=receipt.graph_version;last_version_=out.version;out.timing.commit_ns=stamp(Sc4Clock::now(),origin);out.timing.commit_ms=duration(commit,Sc4Clock::now());
   auto bound=Sc4Clock::now();if(jointAction){out.proof=joint_->issue(input_.ranges[index],retained,receipt,out.state,input_.lever);out.bound_status=out.proof.valid?out.proof.geometry.status:"JOINT_BOUND_REFUSED";out.physical_evaluated=out.proof.evaluated;}
   out.timing.bound_ms=duration(bound,Sc4Clock::now());out.timing.proof_ns=stamp(Sc4Clock::now(),origin);
   out.active_slots=backend_->factorLedger().activeEntries(0,out.epoch).size();out.carry=carry_;out.action.imu.candidate.reference.clear();
  };
  auto cleanup=Sc4Clock::now();execute();out.timing.core_ms=duration(start,Sc4Clock::now());out.timing.cleanup_ms=std::max(0.,out.timing.core_ms-out.timing.ingest_ms-out.timing.prepare_ms-out.timing.candidate_ms-out.timing.commit_ms-out.timing.bound_ms);(void)cleanup;
 }
 out.tag=out.state.position_world_m+out.state.q_world_body*input_.lever;out.finite=out.tag.allFinite();
 // Deliver a complete in-process research packet; CSV is a diagnostic sink.
 // Native transaction/candidate temporaries have been released before output.
 std::ostringstream packet;packet<<std::setprecision(17)<<out.sequence<<' '<<out.state.timestamp.value()<<' '<<out.tag.transpose()<<' '<<out.state.velocity_world_mps.transpose()<<' '<<out.state.q_world_body.coeffs().transpose()<<' '<<out.bound_status<<' '<<out.source<<' '<<out.scope<<' '<<out.proof.valid<<' '<<out.proof.geometry.hpl<<' '<<out.proof.geometry.pl.z()<<' '<<out.cumulative_risk<<' '<<out.retained_ids<<' '<<out.removed_groups<<' '<<out.version<<' '<<out.transaction<<' '<<out.carry;
 auto payload=packet.str();if(payload.empty())throw std::runtime_error("empty output packet");
 auto output=Sc4Clock::now();out.timing.output_ns=stamp(output,origin);out.timing.full_ms=duration(start,output);out.timing.arrival_to_output_ms=std::max(0.,duration(release,output));out.timing.measurement_age_ms=out.timing.arrival_to_output_ms;
 out.core_on_time=out.timing.core_ms<=40;out.on_time=out.timing.arrival_to_output_ms<=50;
 // Late spatial bounds are diagnostic only. No current-time propagation or
 // application/hardware qualification is synthesized.
 out.protected_output=false;return out;
}
Sc4Output Sc4System::reject(size_t index,const std::string& reason,Sc4Clock::time_point release,Sc4Clock::time_point origin){
 sourceGap();auto out=process(index,false,release,origin);out.functional=reason;out.bound_status="CONTINUITY_REFUSED";out.version=last_version_;return out;
}
int runSc4System(int argc,char** argv){
 const tbb::global_control single_thread(tbb::global_control::max_allowed_parallelism,1);
 if(argc<7)throw std::runtime_error("usage: epoch_raim_runner CONFIG CSV SCENE|record:CACHE system on|off EPOCHS [reference|optimized] [offline|paced]");
 auto init=Sc4Clock::now();std::string source=argv[3];int count=std::stoi(argv[6]);bool fde=std::string(argv[5])=="on";
 bool optimized=argc>7 && std::string(argv[7])=="optimized";if(argc>7 && !optimized && std::string(argv[7])!="reference")throw std::runtime_error("unknown SC4 profile");
 bool paced=argc>8 && std::string(argv[8])=="paced";auto cfg=researchLaw(argv[1]);
 if(cfg["sc4_threads"].as<int>()!=1 || cfg["sc4_core_deadline_ms"].as<int>()!=40 || cfg["sc4_full_deadline_ms"].as<int>()!=50 || cfg["sc4_overload_policy"].as<std::string>()!="reject-new-latch-unprotected" || cfg["sc4_after_horizon"].as<std::string>()!="continue-unprotected-preserve-carry" || cfg["sc4_initial_carry_in"].as<std::string>()!="unsupported")throw std::runtime_error("SC4 runtime contract mismatch");
 Sc4System system(source.rfind("record:",0)==0?sc4RecordInput(argv[1],source.substr(7),count):sc4MatchedInput(argv[1],source,count),optimized);double initMs=duration(init,Sc4Clock::now());
 std::ofstream csv(argv[2]);if(!csv)throw std::runtime_error("SC4 output unavailable");csv<<std::setprecision(17);
 csv<<"sequence,epoch,functional,bound_status,finite,qualified,source,scope,timestamp_ns,transaction,version,updates,tag_x,tag_y,tag_z,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,qw,qx,qy,qz,hpl,vpl,cumulative_risk,physical_expected,physical_evaluated,range_masks,excluded,imu_bridge,imu_rejoined,carry,actions_expected,actions_evaluated,rebuilds,history_epochs,active_slots,retained_ids,input_ids,removed_groups,release_ns,start_ns,alarm_ns,selected_ns,commit_ns,proof_ns,output_ns,ingest_ms,prepare_ms,candidate_ms,commit_ms,bound_ms,cleanup_ms,core_ms,full_ms,queue_ms,arrival_to_output_ms,measurement_age_ms,core_on_time,on_time,source_init_ms,max_rss_kb,enqueue_ns,total_rebuilds,total_reintegrations,diagnostic_calls,optimizer_ms,reintegrations_ms,diagnostics_ms,numeric_evaluated,numeric_skipped_post_rejected,numeric_skipped_dominated,source_time_origin_s,delivery_deadline_ns,task_horizon_ns,current_risk_status,formal_eligible,publication_protected,application_available\n";
 auto origin=Sc4Clock::now();Sc4Queue queue(cfg["sc4_queue_capacity"].as<size_t>());size_t next=0,reported=0,firstMissing=system.input().batches.size();std::deque<Sc4QueuedBatch> rejected;
 double loggerMs=0,maxLoggerMs=0;
 auto emit=[&](const Sc4Output& row){auto logging=Sc4Clock::now();struct rusage resource{};getrusage(RUSAGE_SELF,&resource);
  csv<<row.sequence<<','<<row.epoch<<','<<row.functional<<','<<row.bound_status<<','<<row.finite<<','<<row.proof.valid<<','<<row.source<<','<<row.scope<<','<<row.state.timestamp.value()<<','<<row.transaction<<','<<row.version<<','<<row.updates;
  for(const auto& vector:{row.tag,row.state.velocity_world_mps,row.state.accel_bias_mps2,row.state.gyro_bias_radps})for(int j=0;j<3;++j)csv<<','<<vector(j);
  csv<<','<<row.state.q_world_body.w()<<','<<row.state.q_world_body.x()<<','<<row.state.q_world_body.y()<<','<<row.state.q_world_body.z()<<','<<row.proof.geometry.hpl<<','<<row.proof.geometry.pl.z()<<','<<row.cumulative_risk<<','<<row.physical_expected<<','<<row.physical_evaluated<<','<<row.proof.geometry.considered<<','<<row.action.excluded<<','<<row.action.imu.current_bridge<<','<<row.action.imu_rejoined<<','<<row.carry<<','<<row.action.expected_actions<<','<<row.action.evaluated_actions<<','<<row.action.imu.candidate.rebuilds<<','<<row.history_epochs<<','<<row.active_slots<<','<<row.retained_ids<<','<<row.input_ids<<','<<row.removed_groups;
  const auto& t=row.timing;csv<<','<<t.release_ns<<','<<t.start_ns<<','<<t.alarm_ns<<','<<t.selected_ns<<','<<t.commit_ns<<','<<t.proof_ns<<','<<t.output_ns<<','<<t.ingest_ms<<','<<t.prepare_ms<<','<<t.candidate_ms<<','<<t.commit_ms<<','<<t.bound_ms<<','<<t.cleanup_ms<<','<<t.core_ms<<','<<t.full_ms<<','<<t.queue_ms<<','<<t.arrival_to_output_ms<<','<<t.measurement_age_ms<<','<<row.core_on_time<<','<<row.on_time<<','<<initMs<<','<<resource.ru_maxrss<<','<<t.enqueue_ns<<','<<row.action.total_rebuilds<<','<<row.action.total_reintegrations<<','<<row.action.diagnostic_calls<<','<<row.action.optimizer_ms<<','<<row.action.reintegrations_ms<<','<<row.action.diagnostics_ms<<','<<row.action.numeric_evaluated<<','<<row.action.numeric_skipped_post_rejected<<','<<row.action.numeric_skipped_dominated<<','<<system.input().source_time_origin_s<<','<<(t.release_ns+50000000)<<','<<2500000000LL<<','<<(row.proof.valid?"CONDITIONAL_TASK_BOUND":"UNKNOWN_CURRENT_OUTPUT_RISK")<<",0,0,0\n";csv.flush();auto elapsed=duration(logging,Sc4Clock::now());loggerMs+=elapsed;maxLoggerMs=std::max(maxLoggerMs,elapsed);++reported;
 };
 while(reported<system.input().batches.size()){
  auto now=Sc4Clock::now();
  while(next<system.input().batches.size()){
   auto release=paced?origin+std::chrono::nanoseconds(system.input().batches[next].timestamp.value()):now;
   if(paced && now<release)break;
   if(!paced && queue.size())break;
   Sc4QueuedBatch job{next,release,now};if(!queue.push(job)){rejected.push_back(job);firstMissing=std::min(firstMissing,next);}++next;
  }
  if(!queue.size()){
   if(next<system.input().batches.size()){std::this_thread::sleep_until(origin+std::chrono::nanoseconds(system.input().batches[next].timestamp.value()));continue;}
   while(!rejected.empty()){auto job=rejected.front();rejected.pop_front();auto row=system.reject(job.index,"QUEUE_OVERLOAD_REJECTED",job.release,origin);row.timing.enqueue_ns=stamp(job.enqueued,origin);delivered(row,origin);emit(row);}break;
  }
  auto job=queue.pop();
  while(!rejected.empty() && rejected.front().index<job.index){auto drop=rejected.front();rejected.pop_front();auto row=system.reject(drop.index,"QUEUE_OVERLOAD_REJECTED",drop.release,origin);row.timing.enqueue_ns=stamp(drop.enqueued,origin);delivered(row,origin);emit(row);}
  if(job.index>=firstMissing)system.sourceGap();
  auto row=system.process(job.index,fde,job.release,origin);row.timing.enqueue_ns=stamp(job.enqueued,origin);delivered(row,origin);emit(row);
 }
 std::cout<<"SC4 "<<(optimized?"optimized":"reference")<<" source_init_ms="<<initMs<<" epochs="<<count<<" queue_peak="<<queue.peak()<<" diagnostic_logging_ms="<<loggerMs<<" max_logging_ms="<<maxLoggerMs<<'\n';return 0;
}
}
