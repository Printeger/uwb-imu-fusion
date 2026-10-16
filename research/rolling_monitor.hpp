#pragma once
#include "sc2_imu_scope.hpp"
#include <functional>
#include <map>
namespace uwb_imu_pl::research {
struct RollingMonitorView {
 Sc2ImuAssessment state;
 bool open=false;uint64_t alert_generation=0;
 std::string event_id;
 size_t active=0,ended=0,excluded=0,not_started=0;
 std::function<bool(const EpochTransaction&)> matches;
 // Optional original-parent clock; present only for generalized actual-time recovery.
 std::function<TimestampNs(MeasurementId)> original_time;
 std::map<MeasurementId,double> range_transport;
 bool has_range_time_model=false;
};
inline RollingMonitorView finiteMonitorView(const Sc2ImuMonitor& monitor,TimestampNs now){
 RollingMonitorView view;view.state=monitor.assessment();view.matches=[&monitor](const auto& tx){return monitor.matches(tx);};
 for(const auto& c:monitor.candidates()){
  if(!c.possible)++view.excluded;else if(now<c.episode.begin)++view.not_started;
  else if(now<c.episode.end)++view.active;else ++view.ended;
 }return view;
}
}
