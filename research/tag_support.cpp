#include "tag_support.hpp"
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
void summarize(RawRangeResult& out,const std::vector<RawSupportBox>& boxes,const Eigen::Vector3d& center){
 out.low.setConstant(INFINITY);out.high.setConstant(-INFINITY);out.hpl=0;
 for(const auto& box:boxes){if(!box.low.allFinite() || !box.high.allFinite() || (box.low.array()>box.high.array()).any() || box.omitted < -1 || box.omitted>7)throw std::invalid_argument("TAG_SUPPORT_BOX_REFUSED");
  Eigen::Vector3d d;for(int j=0;j<3;++j)d(j)=abs(I(box.low(j),box.high(j))-I(center(j))).upper();
  out.hpl=std::max(out.hpl,sqrt(square(I(d.x()))+square(I(d.y()))).upper());out.low=out.low.cwiseMin(box.low);out.high=out.high.cwiseMax(box.high);
 }
 if(boxes.empty())throw std::invalid_argument("TAG_SUPPORT_ALL_BRANCHES_EMPTY");
 for(int j=0;j<3;++j)out.pl(j)=abs(I(out.low(j),out.high(j))-I(center(j))).upper();
}
bool sameBoxes(const std::vector<RawSupportBox>& a,const std::vector<RawSupportBox>& b){if(a.size()!=b.size())return false;for(size_t i=0;i<a.size();++i)if(a[i].low!=b[i].low || a[i].high!=b[i].high || a[i].omitted!=b[i].omitted || a[i].frontier!=b[i].frontier)return false;return true;}
}
RawRangeResult issueTagSupport(const RawRangeResult& geometry,const RollingVector& tag,const Eigen::Vector3d& actual,const UwbBatch& batch,const CommitReceipt& receipt,uint64_t parent,const std::string& event_id,uint64_t prefix,double q){
 if(!geometry.valid || geometry.support.empty() || !actual.allFinite() || !geometry.input_low.allFinite() || !geometry.input_high.allFinite())throw std::invalid_argument("TAG_SUPPORT_GEOMETRY_REFUSED");
 auto evidence=std::make_shared<TagSupportEvidence>();evidence->center=actual;evidence->input_low=geometry.input_low;evidence->input_high=geometry.input_high;evidence->batch=batch;evidence->transaction=receipt.transaction_id;evidence->graph_version=receipt.graph_version;evidence->stamp=receipt.state_timestamp;evidence->epoch=receipt.committed_epoch;evidence->parent=parent;evidence->event_id=event_id;evidence->source_prefix=prefix;evidence->q=q;
 for(auto leaf:geometry.support){bool empty=false;for(int j=0;j<3;++j){leaf.low(j)=std::max(leaf.low(j),tag[j].lower());leaf.high(j)=std::min(leaf.high(j),tag[j].upper());empty|=leaf.low(j)>leaf.high(j);}if(!empty)evidence->boxes.push_back(leaf);}
 RawRangeResult out=geometry;out.support=evidence->boxes;out.tag_support=evidence;out.bound_method=evidence->method;out.source="sc7-joint-leaf-tag-support-v1";summarize(out,evidence->boxes,actual);
 for(int j=0;j<3;++j)out.old_pl(j)=abs(I(geometry.input_low(j),geometry.input_high(j))-I(actual(j))).upper();out.old_hpl=sqrt(square(I(out.old_pl.x()))+square(I(out.old_pl.y()))).upper();
 out.valid=verifyTagSupport(out,actual,batch,receipt,parent,event_id,prefix,q);if(!out.valid)throw std::invalid_argument("TAG_SUPPORT_RECEIPT_OR_NUMERIC_REFUSED");return out;
}
bool verifyTagSupport(const RawRangeResult& out,const Eigen::Vector3d& center,const UwbBatch& batch,const CommitReceipt& receipt,uint64_t parent,const std::string& event_id,uint64_t prefix,double q){
 try{if(!out.tag_support)return false;const auto& e=*out.tag_support;
  if(out.bound_method!=e.method || e.method!="sc7-branch-box-union-tag-v1" || e.center!=center || !center.allFinite() || e.frame!="world" || e.fault_scope!="FULL_TASK_PRODUCT_OUTER_MASK_RELAXATION_NO_PROBABILISTIC_PRUNING" || e.transaction!=receipt.transaction_id || e.graph_version!=receipt.graph_version || e.stamp!=receipt.state_timestamp || e.epoch!=receipt.committed_epoch || e.parent!=parent || e.event_id!=event_id || e.source_prefix!=prefix || e.q!=q || !std::isfinite(q) || q<=0 || receipt.backend_updates!=1 || !sameRawBatch(e.batch,batch) || !sameBoxes(out.support,e.boxes) || e.input_low!=out.input_low || e.input_high!=out.input_high || (e.input_low.array()>e.input_high.array()).any() || e.budget_policy!="ALL_LEAVES_AND_UNRESOLVED_FRONTIER_OUTER" || e.roundoff_policy!="boost-directed-frounding-math-ffp-contract-off")return false;
  for(const auto& b:e.boxes)if((b.low.array()<e.input_low.array()).any() || (b.high.array()>e.input_high.array()).any())return false;
  RawRangeResult recomputed;summarize(recomputed,e.boxes,center);
  return recomputed.hpl==out.hpl && recomputed.pl==out.pl && recomputed.low==out.low && recomputed.high==out.high && out.pl.allFinite() && std::isfinite(out.hpl);
 }catch(...){return false;}
}
}
