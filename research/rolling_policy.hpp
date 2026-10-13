#pragma once
#include "scenario_material.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include <set>
#include <gtsam/inference/Symbol.h>

namespace uwb_imu_pl::research {
struct ResolvedRollingPolicyV1 {
 int fgo=30,integrity=30,replay=50,mission=200;
 double budget=1e-6;
 std::string mode="follow_fgo";
};
inline ResolvedRollingPolicyV1 resolveRollingPolicy(const YAML::Node& root,
                                                   IntegrityConfig& config){
 auto n=root["rolling_integrity"];
 if(!n || !n["enabled"].as<bool>() || n["version"].as<int>()!=1)
  throw std::invalid_argument("SC5_EXPLICIT_OPT_IN_REQUIRED");
 ResolvedRollingPolicyV1 p;
 const auto lag=root["fixed_lag_epochs"].as<long long>();
 if(lag<2 || lag>1000 || readEstimationTuningV1(config).nominal_lag_s!=0)
  throw std::invalid_argument("CONFIG_SCOPE_UNSUPPORTED: logical epoch lag required");
 p.fgo=static_cast<int>(lag);p.mode=n["window_mode"].as<std::string>();
 if(p.mode=="follow_fgo"){
  if(n["explicit_epochs"] && !n["explicit_epochs"].IsNull())
   throw std::invalid_argument("CONFLICTING_WINDOW_AUTHORITY");
  p.integrity=p.fgo;
 }else if(p.mode=="explicit"){
  const auto li=n["explicit_epochs"].as<long long>();
  if(li<2 || li>lag)throw std::invalid_argument("CONFIG_SCOPE_UNSUPPORTED: explicit window");
  p.integrity=static_cast<int>(li);
 }else throw std::invalid_argument("UNKNOWN_WINDOW_MODE");
 const auto t=n["mission_epochs"].as<long long>();
 const auto extra=n["recovery_extra_epochs"].as<long long>();
 if(t<1 || t>10000 || extra<0 || extra>1000 ||
    n["risk_mode"].as<std::string>()!="finite_mission" ||
    n["boundary_method"].as<std::string>()!="raw_outer_projection" ||
    n["runtime_resize"].as<std::string>()!="disabled")
  throw std::invalid_argument("CONFIG_SCOPE_UNSUPPORTED");
 p.mission=static_cast<int>(t);p.replay=p.fgo+static_cast<int>(extra);
 p.budget=n["task_exceedance_budget"].as<double>();
 if(!std::isfinite(p.budget) || p.budget<=0 || p.budget>=1)
  throw std::invalid_argument("INVALID_TASK_BUDGET");
 config.incremental.fixed_lag_epochs=p.fgo;config.integrity_window.epochs=p.integrity;
 return p;
}
inline void rollingResize(const ResolvedRollingPolicyV1&,int,int){
 throw std::invalid_argument("RUNTIME_RESIZE_UNSUPPORTED");
}
struct BoundaryMaterialViewV1 {
 TransactionId transaction;uint64_t version=0;
 size_t epoch=0,cutoff=0;TimestampNs begin,end;
 std::set<gtsam::Key> pre_keys,expected_retired;
 std::vector<FactorGroupId> selected;
 size_t raw_first=0,raw_last=0;
};
inline BoundaryMaterialViewV1 captureBeforeUpdate(const EpochTransaction& tx,
 const EpochCommitPlan& plan,const ResolvedRollingPolicyV1& p){
 if(!tx.frozen_values || tx.raw_imu_slice.size()!=11 ||
    tx.raw_imu_slice.front().timestamp!=tx.begin ||
    tx.raw_imu_slice.back().timestamp!=tx.end || tx.previous_epoch+1!=tx.proposed_epoch)
  throw std::invalid_argument("BOUNDARY_RAW_MATERIAL_MISSING");
 BoundaryMaterialViewV1 v;v.transaction=tx.id;v.version=tx.base_graph_version;
 v.epoch=tx.proposed_epoch;v.cutoff=v.epoch+1>size_t(p.integrity)?v.epoch+1-p.integrity:0;
 v.begin=tx.begin;v.end=tx.end;v.selected=plan.groups_to_add;
 v.raw_first=tx.raw_imu_slice.front().id.value();v.raw_last=tx.raw_imu_slice.back().id.value();
 const size_t fcut=v.epoch+1>size_t(p.fgo)?v.epoch+1-p.fgo:0;
 for(auto key:tx.frozen_values->keys()){
  v.pre_keys.insert(key);
  if(gtsam::Symbol(key).index()<fcut)v.expected_retired.insert(key);
 }
 return v;
}
struct BoundaryTransferReceiptV1 {
 size_t cutoff=0,marginalized=0,retained=0;
 uint64_t version=0;bool x0_present=false;
 std::vector<size_t> numerical_boundary_slots;
};
inline BoundaryTransferReceiptV1 finalizeTransfer(const BoundaryMaterialViewV1& v,
 const CommitReceipt& r,const std::vector<gtsam::Key>& keys){
 if(r.transaction_id!=v.transaction || r.graph_version!=v.version+1 ||
    r.committed_epoch!=v.epoch || r.state_timestamp!=v.end || r.backend_updates!=1)
  throw std::invalid_argument("BOUNDARY_RECEIPT_VERSION_MISMATCH");
 std::set<gtsam::Key> post(keys.begin(),keys.end()),actual(r.marginalized_keys.begin(),r.marginalized_keys.end());
 if(actual!=v.expected_retired)throw std::invalid_argument("ACTUAL_MARGINAL_KEYS_MISMATCH");
 for(auto key:v.pre_keys)if((post.count(key)==0)!=bool(actual.count(key)))
  throw std::invalid_argument("BOUNDARY_KEY_ACCOUNTING_MISMATCH");
 for(char c:{'x','v','b'})if(!post.count(gtsam::Symbol(c,v.epoch)))
  throw std::invalid_argument("CURRENT_KEYS_MISSING");
 for(auto id:v.selected)if(!r.group_to_slots.count(id.value()))
  throw std::invalid_argument("SELECTED_SOURCE_RECEIPT_MISMATCH");
 return {v.cutoff,actual.size(),post.size()/3,r.graph_version,
         bool(post.count(gtsam::Symbol('x',0))),r.boundary_factor_slots};
}
}
