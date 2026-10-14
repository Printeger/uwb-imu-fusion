#pragma once
#include "rolling_raw.hpp"
namespace uwb_imu_pl::research {
struct TagSupportEvidence {
 std::string method="sc7-branch-box-union-tag-v1",event_id,frame="world";
 std::string fault_scope="FULL_TASK_PRODUCT_OUTER_MASK_RELAXATION_NO_PROBABILISTIC_PRUNING";
 std::string budget_policy="ALL_LEAVES_AND_UNRESOLVED_FRONTIER_OUTER";
 std::string roundoff_policy="boost-directed-frounding-math-ffp-contract-off";
 Eigen::Vector3d center,input_low,input_high;
 UwbBatch batch;
 TransactionId transaction;uint64_t graph_version=0,epoch=0,parent=0,source_prefix=0;
 TimestampNs stamp;double q=0;
 std::vector<RawSupportBox> boxes;
};
RawRangeResult issueTagSupport(const RawRangeResult&,const RollingVector&,const Eigen::Vector3d&,
 const UwbBatch&,const CommitReceipt&,uint64_t parent,const std::string& event_id,uint64_t source_prefix,double q);
bool verifyTagSupport(const RawRangeResult&,const Eigen::Vector3d&,const UwbBatch&,
 const CommitReceipt&,uint64_t parent,const std::string& event_id,uint64_t source_prefix,double q);
}
