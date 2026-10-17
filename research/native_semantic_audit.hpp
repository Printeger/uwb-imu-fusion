#pragma once
#include "rolling_boundary.hpp"
#include "sensor_envelope.hpp"
#include "uwb_imu_pl/estimation/factor_ledger.hpp"
#include <openssl/sha.h>
#include <cstring>
#include <limits>

namespace uwb_imu_pl::research {
// Canonical binary encodings for cross-entry-point equality, not qualification
// tokens. Lengths and integer fields are big-endian uint64; doubles preserve
// their IEEE bits. No addresses, wall-clock timers or object padding are hashed.
class SemanticDigest {
 SHA256_CTX context_;
 public:
 SemanticDigest(){SHA256_Init(&context_);}
 void u(uint64_t value){unsigned char bytes[8];for(int k=7;k>=0;--k){bytes[k]=value&255;value>>=8;}SHA256_Update(&context_,bytes,8);}
 void d(double value){static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559,"IEEE binary64 required");uint64_t bits;std::memcpy(&bits,&value,8);u(bits);}
 void s(const std::string& value){u(value.size());SHA256_Update(&context_,value.data(),value.size());}
 void v(const Eigen::Vector3d& value){for(int j=0;j<3;++j)d(value[j]);}
 void interval(const RollingInterval& value){d(value.lower());d(value.upper());}
 void version(const LinearizationVersion& v){u(v.graph_version);u(v.ordering_version);u(v.noise_model_version);u(v.linpoint_version);}
 template<class T>void integers(const T& values){u(values.size());for(auto value:values)u(value);}
 template<class T>void ids(const T& values){u(values.size());for(auto value:values)u(value.value());}
 template<class T>void strings(const T& values){u(values.size());for(const auto& value:values)s(value);}
 void imu(const ImuMeasurement& m){u(m.id.value());u(m.timestamp.value());v(m.specific_force_mps2);v(m.angular_velocity_radps);u(m.quality_flags);}
 void range(const UwbMeasurement& m){u(m.id.value());u(m.factor_id.value());u(m.anchor_id.value());u(m.tag_id.value());u(m.sequence);u(m.timestamp.value());v(m.anchor_position_m);u(bool(m.lever_arm_body_m));if(m.lever_arm_body_m)v(*m.lever_arm_body_m);d(m.range_m);d(m.sigma_m);u(m.quality_flags);}
 std::string finish(){unsigned char bytes[SHA256_DIGEST_LENGTH];SHA256_Final(bytes,&context_);const char* digits="0123456789abcdef";std::string out;for(auto b:bytes){out+=digits[b>>4];out+=digits[b&15];}return out;}
};
inline std::string receiptDigest(const CommitReceipt& r){
 SemanticDigest h;h.s("sc9-native-receipt-v1");h.u(r.transaction_id.value());h.u(r.action_id.value());h.u(r.graph_version);h.u(r.committed_epoch);h.u(r.state_timestamp.value());h.u(r.backend_updates);h.u(r.integrity_available);
 h.integers(r.added_factor_slots);h.integers(r.removed_factor_slots);h.integers(r.marginalized_keys);h.integers(r.boundary_factor_slots);h.integers(r.reused_factor_slots);
 h.u(r.group_to_slots.size());for(const auto& group:r.group_to_slots){h.u(group.first);h.integers(group.second);}
 h.ids(r.historical_groups_removed);h.ids(r.historical_groups_added);
 h.u(bool(r.recovery_epoch_begin));if(r.recovery_epoch_begin)h.u(*r.recovery_epoch_begin);
 h.u(bool(r.recovery_epoch_end));if(r.recovery_epoch_end)h.u(*r.recovery_epoch_end);return h.finish();
}
inline std::string ledgerDigest(const FactorLedger& ledger){
 auto entries=ledger.entries();std::sort(entries.begin(),entries.end(),[](const auto& a,const auto& b){return a.factor_id<b.factor_id;});
 SemanticDigest h;h.s("sc9-native-ledger-provenance-v1");h.u(ledger.version());h.u(entries.size());
 for(const auto& e:entries){h.u(e.factor_id.value());h.u(e.group_id.value());h.u(uint64_t(e.sensor));h.u(uint64_t(e.kind));h.integers(e.keys);h.ids(e.source_measurements);h.strings(e.source_ids);h.ids(e.associated_fault_units);h.strings(e.fault_units);
  h.u(e.epoch_begin);h.u(e.epoch_end);h.u(e.time_begin.value());h.u(e.time_end.value());h.u(bool(e.backend_slot));if(e.backend_slot)h.u(*e.backend_slot);
  h.u(uint64_t(e.lifecycle));h.u(uint64_t(e.health_at_commit));h.s(e.noise_model_id);h.s(e.model_id);h.version(e.commit_version);
  h.u(bool(e.removed_version));if(e.removed_version)h.version(*e.removed_version);h.u(bool(e.replacement_group));if(e.replacement_group)h.u(e.replacement_group->value());h.u(bool(e.replaces_group));if(e.replaces_group)h.u(e.replaces_group->value());h.u(e.recovery_epoch);
  h.u(bool(e.factor));if(e.factor){h.u(e.factor->dim());h.integers(e.factor->keys());}
 }return h.finish();
}
inline std::string boundaryDigest(const RollingBoundaryCertificateV1& c){
 SemanticDigest h;h.s("sc9-rolling-boundary-v1");h.u(c.generation);h.u(c.parent);h.u(c.version);h.u(c.source_prefix);h.u(c.covered_through);h.u(c.shared_adc_index);h.u(c.committed_time.value());h.d(c.source_risk);h.s(c.event_id);h.s(c.state_method);h.u(c.fault_counts_complete);
 h.u(c.fault_not_started);h.u(c.fault_active);h.u(c.fault_ended_residual);h.u(c.fault_excluded);
 const auto& x=c.separator;h.u(x.epoch);h.u(x.has_tag_relation);if(x.has_tag_relation){for(const auto& a:x.tag_relation)h.interval(a);h.v(x.tag_lever);}
 for(const auto& a:x.p)h.interval(a);for(const auto& a:x.v)h.interval(a);for(const auto& row:x.rotation)for(const auto& a:row)h.interval(a);for(const auto& a:x.bias)h.interval(a);
 h.u(c.dynamic_anchors.size());for(const auto& a:c.dynamic_anchors){h.u(a.first.value());h.v(a.second);h.u(c.anchor_versions.at(a.first));}
 if(c.dynamic_anchors.empty())for(size_t i=0;i<c.anchor_ids.size();++i){h.u(c.anchor_ids[i].value());h.v(c.anchor_positions[i]);}return h.finish();
}
inline std::string sourceDigest(const Sc6Envelope& input,const GeneralizedRawEvidence& e){
 SemanticDigest h;h.s("sc9-epoch-original-source-material-v2-global-speed");h.u(input.epoch);h.u(input.uwb.id.value());h.u(input.uwb.timestamp.value());h.u(input.imu.size());for(const auto& m:input.imu)h.imu(m);
 h.u(input.range_events.size());for(const auto& r:input.range_events){h.range(r.measurement);h.u(r.received.value());h.u(r.calibration_version);h.u(r.valid);}
 h.u(input.uwb.measurements.size());for(const auto& m:input.uwb.measurements)h.range(m);
 h.u(input.uwb.covariance_m2.rows());h.u(input.uwb.covariance_m2.cols());for(int j=0;j<input.uwb.covariance_m2.rows();++j)for(int k=0;k<input.uwb.covariance_m2.cols();++k)h.d(input.uwb.covariance_m2(j,k));h.s(input.uwb.covariance_model_id);
 h.u(e.begin.value());h.u(e.end.value());h.u(uint64_t(e.status));h.s(e.event_id);h.u(e.source_prefix);h.d(e.risk_upper);h.d(e.max_coordinate_radius);
 h.d(e.global_speed);h.interval(e.evolution.duration);for(const auto& a:e.evolution.bias_radius)h.interval(a);
 h.u(e.joint_samples.size());for(const auto& m:e.joint_samples){h.imu(m.raw);h.u(m.integration_end.value());for(const auto& a:m.adc_radius)h.interval(a);for(const auto& a:m.rw_from_begin)h.interval(a);for(const auto& a:m.rw_to_end)h.interval(a);}
 h.u(e.range_noise_radius.size());for(auto radius:e.range_noise_radius)h.d(radius);h.integers(e.range_calibration_versions);return h.finish();
}
}
