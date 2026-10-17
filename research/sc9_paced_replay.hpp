#pragma once
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <stdexcept>
#include <utility>

namespace uwb_imu_pl::research {
// Independent virtual-sensor releases. A token identifies one immutable
// seeded sensor frame, decoded by the provider under the measured online cost.
// The producer never waits for decoding, proof, optimization or sink output.
class Sc9PacedReleases {
 public:
 using Clock=std::chrono::steady_clock;
 struct Tick {size_t epoch=0;int64_t end=0;Clock::time_point target,released;size_t high_water=0;};
 Sc9PacedReleases(int64_t initial,std::vector<int64_t> ends):initial_(initial),ends_(std::move(ends)),origin_(Clock::now()+std::chrono::milliseconds(100)){
  if(initial_<0||initial_>1000000000000000LL)throw std::invalid_argument("PACED_VIRTUAL_SOURCE_TIME_DOMAIN");
  int64_t previous=initial_;for(auto end:ends_){if(end<=previous||end>1000000000000000LL)throw std::invalid_argument("PACED_RELEASE_ORDER");previous=end;}
  producer_=std::thread([this]{
   for(size_t i=0;i<ends_.size();++i){const auto target=at(ends_[i]);std::unique_lock<std::mutex> lock(mutex_);
    if(changed_.wait_until(lock,target,[this]{return stopped_;}))return;
    if(pending_.size()>=512){overflow_=true;stopped_=true;changed_.notify_all();return;}
    high_water_=std::max(high_water_,pending_.size()+1);
    pending_.push_back({i+1,ends_[i],target,Clock::now(),high_water_});changed_.notify_all();
   }
   std::lock_guard<std::mutex> lock(mutex_);finished_=true;changed_.notify_all();
  });
 }
 ~Sc9PacedReleases(){
  {std::lock_guard<std::mutex> lock(mutex_);stopped_=true;changed_.notify_all();}
  if(producer_.joinable())producer_.join();
 }
 Tick next(size_t epoch){
  std::unique_lock<std::mutex> lock(mutex_);changed_.wait(lock,[this]{return stopped_||finished_||!pending_.empty();});
  if(overflow_)throw std::runtime_error("SIM_PACED_INPUT_QUEUE_OVERFLOW");
  if(pending_.empty())throw std::runtime_error("SIM_PACED_INPUT_SUFFIX_MISSING");
  auto tick=pending_.front();pending_.pop_front();
  if(tick.epoch!=epoch)throw std::runtime_error("SIM_PACED_INPUT_SEQUENCE_GAP");
  return tick;
 }
 size_t pendingCount(){std::lock_guard<std::mutex> lock(mutex_);return pending_.size();}
 bool overflowed(){std::lock_guard<std::mutex> lock(mutex_);return overflow_;}
 Clock::time_point at(int64_t time)const{return origin_+std::chrono::nanoseconds(time-initial_);}
 Clock::time_point deadline(size_t epoch,int64_t measured)const{
  auto limit=at(measured)+std::chrono::milliseconds(50);
  return epoch<ends_.size()?std::min(limit,at(ends_[epoch])):limit;
 }
 static int64_t ns(Clock::time_point value){return std::chrono::duration_cast<std::chrono::nanoseconds>(value.time_since_epoch()).count();}
 private:
 int64_t initial_;std::vector<int64_t> ends_;Clock::time_point origin_;
 std::mutex mutex_;std::condition_variable changed_;std::deque<Tick> pending_;std::thread producer_;
 bool stopped_=false,finished_=false,overflow_=false;size_t high_water_=0;
};
}
