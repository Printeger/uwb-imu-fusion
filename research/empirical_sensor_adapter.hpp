#pragma once
#include "unified_rolling_core.hpp"

namespace uwb_imu_pl::research {
// Fixed CAL model and common bootstrap used by file and live adapters. This
// layer has no reference trajectory, injection labels or future-data access.
struct EmpiricalSetup {
 YAML::Node model,law;
 IntegrityConfig config;
 ResolvedRollingPolicyV1 policy;
 JointRawLaw raw_law;
 Sc6MotionDomain domain;
 Eigen::Vector3d lever{.1,-.025,0};
 Eigen::Matrix3d world;
 double range_clock_offset=0;
};
EmpiricalSetup empiricalSetup(const YAML::Node& model);
struct EmpiricalBootstrap {
 std::unique_ptr<UnifiedRollingIntegrityCore> core;
 NavigationState numerical;
 std::string source_digest;
};
EmpiricalBootstrap empiricalBootstrap(const EmpiricalSetup&,AnchorRegistry,
 const std::vector<ImuMeasurement>& arrived_prefix,
 const std::vector<CanonicalRangeEvent>& arrived_potential_ranges,UwbBatch selected_ranges);
struct EmpiricalBatch {
 UwbBatch values;
 std::vector<CanonicalRangeEvent> events;
 // Uncorrected original sample clock, used only for delivery accounting.
 std::optional<TimestampNs> latest_original_valid_range;
};
struct EmpiricalEmission {
 Sc6Result result;
 BatchId batch;
 TimestampNs latest_original_used;
};
class CausalEmpiricalStream {
 EmpiricalSetup setup_;AnchorRegistry anchors_;
 std::deque<ImuMeasurement> imu_;
 std::deque<EmpiricalBatch> batches_;
 std::vector<CanonicalRangeEvent> startup_ranges_;
 std::optional<EmpiricalBatch> startup_batch_;
 EmpiricalBootstrap bootstrap_;
 std::optional<TimestampNs> last_imu_,last_batch_;
 std::optional<TimestampNs> latest_received_;TimestampNs last_used_imu_;
 size_t epochs_=0,startup_batches_=0,imu_received_=0,batches_received_=0;
 bool failed_=false;
 static constexpr size_t imu_capacity_=4096,batch_capacity_=128,startup_range_capacity_=16384;
 void available()const{if(failed_)throw std::logic_error("EMPIRICAL_STREAM_FAILED_CLOSED");}
 public:
 CausalEmpiricalStream(EmpiricalSetup setup,AnchorRegistry anchors):setup_(std::move(setup)),anchors_(std::move(anchors)){}
 void appendImu(const ImuMeasurement&,TimestampNs received);
 void appendBatch(EmpiricalBatch);
 std::optional<EmpiricalEmission> next(bool actions_enabled);
 bool initialized()const{return bool(bootstrap_.core);}
 const std::string& startupDigest()const{return bootstrap_.source_digest;}
 TimestampNs initialTimestamp()const{return bootstrap_.numerical.timestamp;}
 size_t epochs()const{return epochs_;}
 size_t startupBatches()const{return startup_batches_;}
 size_t pendingImu()const{return imu_.size();}
 size_t pendingBatches()const{return batches_.size()+bool(startup_batch_);}
 size_t receivedImu()const{return imu_received_;}
 size_t receivedBatches()const{return batches_received_;}
 uint64_t updates()const{return bootstrap_.core?bootstrap_.core->updateCount():0;}
};
}
