#pragma once
#include "empirical_sensor_adapter.hpp"
#include "uwb_imu_pl/common/runtime_route_audit.hpp"
#include <sstream>
#include <iomanip>
namespace uwb_imu_pl::research {
inline std::string wireResultHeader(){std::ostringstream out;
 out<<"epoch,batch_id,stamp_ns,core_id,model_status,bound_status,set_valid,status,recovery,imu_alarm,uwb_alarm,excluded,excluded_source,bridge,repair,polluted_prior,rejoined,first_dependency,updates,version,generation,parent,boundary,source_prefix,q,event_risk,registered,visible,measurements,invalid,time_groups,time_live,max_transport,hpl,vpl,raw_qualified,publication_protected,tag_x,tag_y,tag_z,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz,core_ms,prepare_ms,candidate_ms,proof_ms,commit_ms,cleanup_ms,active_states,proof_explicit,slots,ledger,replay,capsules,rss_kb,old_pipeline_constructions,old_engine_calls,old_fde_calls,backend_constructions,source_digest,receipt_digest,ledger_digest,boundary_digest\n";return out.str();}
inline std::string wireResultRecord(const Sc6Result& r,BatchId batch,size_t epoch,double elapsed){
 const auto route=runtimeRouteAuditV1();
 if(route.old_pipeline_constructions || route.old_engine_calls || route.old_fde_calls || route.incremental_backend_constructions!=1)throw std::runtime_error("WIRE_RUNTIME_ROUTE_MISMATCH");
 std::ostringstream out;out<<std::setprecision(17);
  out<<epoch<<','<<batch.value()<<','<<r.state.timestamp.value()<<','<<UnifiedRollingIntegrityCore::implementationId()<<",DECLARED_EMPIRICAL,"<<r.bound.status<<','<<r.bound.valid<<','<<r.action.status<<','<<r.action.recovery<<','<<r.imu_alarm<<','<<r.action.uwb_alarm<<','<<r.action.excluded<<','<<(r.action.excluded_source?std::to_string(r.action.excluded_source->value()):"")<<','<<r.action.current_bridge<<','<<r.action.boundary_repair<<','<<r.action.history_marginalized<<','<<r.action.rejoined<<','<<r.action.first_dependency<<','<<r.backend_updates<<','<<r.version<<','<<r.generation<<','<<r.parent<<','<<r.boundary<<','<<r.source_prefix<<','<<r.q<<','<<r.bound.event_risk<<','<<r.input_counts.registered<<','<<r.input_counts.visible<<','<<r.input_counts.measurements<<','<<r.input_counts.invalid<<','<<r.time_factor_groups<<','<<r.time_committed_factors<<','<<r.max_range_transport<<','<<r.bound.hpl<<','<<r.bound.pl.z()<<",0,0";
  for(const auto& v:std::vector<Eigen::Vector3d>{r.tag,r.state.position_world_m,r.state.velocity_world_mps})for(int j=0;j<3;++j)out<<','<<v[j];
  const auto& q=r.state.q_world_body;out<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z();for(const auto& v:std::vector<Eigen::Vector3d>{r.state.accel_bias_mps2,r.state.gyro_bias_radps})for(int j=0;j<3;++j)out<<','<<v[j];
  out<<','<<elapsed<<','<<r.prepare_ms<<','<<r.candidate_ms<<','<<r.proof_ms<<','<<r.commit_ms<<','<<r.cleanup_ms<<','<<r.active_states<<','<<r.proof_explicit<<','<<r.slots<<','<<r.ledger<<','<<r.replay<<','<<r.capsules<<','<<r.rss<<','<<route.old_pipeline_constructions<<','<<route.old_engine_calls<<','<<route.old_fde_calls<<','<<route.incremental_backend_constructions<<','<<r.source_digest<<','<<r.receipt_digest<<','<<r.ledger_digest<<','<<r.boundary_digest<<'\n';return out.str();}
}
