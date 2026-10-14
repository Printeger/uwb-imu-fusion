#pragma once
#include <deque>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
namespace uwb_imu_pl::research {
class OpenDeliveryGate {
 size_t capacity_;std::deque<uint64_t> jobs_;uint64_t started_=0,delivered_=0;
 bool gap_=false;
 public:
 explicit OpenDeliveryGate(size_t cap):capacity_(cap){if(!cap)throw std::invalid_argument("ZERO_QUEUE_CAPACITY");}
 bool enqueue(uint64_t id){if(gap_ || jobs_.size()==capacity_){gap_=true;return false;}jobs_.push_back(id);return true;}
 void cancel(uint64_t id){auto it=std::find(jobs_.begin(),jobs_.end(),id);if(it!=jobs_.end())jobs_.erase(it);gap_=true;}
 bool start(uint64_t id){if(gap_ || jobs_.empty() || jobs_.front()!=id || id!=started_+1){gap_=true;return false;}jobs_.pop_front();started_=id;return true;}
 bool deliver(uint64_t epoch,int64_t timestamp,uint64_t version,bool bound,double service_ms,double deadline_ms){
  if(gap_ || epoch!=started_ || epoch<=delivered_ || timestamp!=int64_t(epoch)*50000000LL || version!=epoch+1 || !bound || service_ms>deadline_ms)return false;
  delivered_=epoch;return true;
 }
 bool gap()const{return gap_;}
};
}
