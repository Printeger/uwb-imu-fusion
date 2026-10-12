#include "sc2_imu_scope.hpp"
#include <boost/numeric/interval.hpp>
#include <set>
namespace uwb_imu_pl::research {
using I=boost::numeric::interval<double>;
Sc2ImuMonitor::Sc2ImuMonitor(const NavigationState& initial,const RawInclusionParameters& law,Sc2ImuScope scope):initial_(initial),law_(law),scope_(scope){
 if(scope.id!="sc2-single-imu-episode-rw-v1" || scope.horizon!=50 || law.horizon!=50 || law.coordinates!=6421 || !law.numeric.valid || !law.initial_sigmas.allFinite() || (law.initial_sigmas.array()<=0).any() || !initial.accel_bias_mps2.allFinite() || !initial.gyro_bias_radps.allFinite() || !std::isfinite(scope.acceleration_bound) || scope.acceleration_bound<0 || !std::isfinite(scope.angular_rate_bound) || scope.angular_rate_bound<0 || initial.timestamp.value()!=0 || !std::isfinite(law.gravity) || law.gravity<=0 || !std::isfinite(law.accel_adc+law.gyro_adc+law.accel_rw+law.gyro_rw) || !law.numeric.bias.allFinite() || (law.numeric.bias.array()<0).any() || !std::isfinite(law.numeric.accel_sample+law.numeric.gyro_sample) || law.numeric.accel_sample<0 || law.numeric.gyro_sample<0 || law.accel_adc<=0 || law.gyro_adc<=0 || law.accel_rw<=0 || law.gyro_rw<=0)throw std::runtime_error("SC2 unregistered source/domain");
 I witness=I(1)+I(std::ldexp(1.,-53));if(witness.lower()>1 || witness.upper()<std::nextafter(1.,INFINITY))throw std::runtime_error("SC2 directed arithmetic unavailable");
 std::set<int> unique;
 for(int duration:scope.durations_ms){if((duration!=50 && duration!=500) || !unique.insert(duration).second)throw std::runtime_error("SC2 unsupported duration");
  for(int onset=1;onset<=50;++onset)for(int axis=0;axis<6;++axis){long long begin=(onset-1)*50000000LL+5000000LL;
   candidates_.push_back({{"imu:axis:"+std::to_string(axis)+":onset:"+std::to_string(onset)+":duration:"+std::to_string(duration),TimestampNs(begin),TimestampNs(begin+duration*1000000LL),axis},-INFINITY,INFINITY,true});
  }
 }
 if(candidates_.size()+1>scope.candidate_capacity)throw std::runtime_error("SC2 candidate capacity refuses whole scope");
 result_.valid=true;result_.possible=candidates_.size();result_.status="NOMINAL_POSSIBLE";
}
void Sc2ImuMonitor::append(const ImuMeasurement& sample){
 if(!result_.valid)return;
 int index=result_.samples;
 if(index>500 || sample.id.value()!=uint64_t(index+1) || sample.timestamp.value()!=index*5000000LL || !sample.specific_force_mps2.allFinite() || !sample.angular_velocity_radps.allFinite()){result_.valid=false;result_.status="RAW_PREFIX_REFUSED";return;}
 std::array<I,6> healthy;std::array<double,6> measured;bool violation=false;
 for(int axis=0;axis<6;++axis){bool accel=axis<3;int j=axis%3;
  I noise=7.*(I(law_.initial_sigmas(axis+9))+double(index)*I(accel?law_.accel_rw:law_.gyro_rw)*sqrt(I(1)/200.)+I(accel?law_.accel_adc:law_.gyro_adc))+I(law_.numeric.bias(axis))+I(accel?law_.numeric.accel_sample:law_.numeric.gyro_sample);
  I physical=accel?I(law_.gravity)+I(scope_.acceleration_bound):I(scope_.angular_rate_bound);
  double radius=(noise+physical).upper();double bias=accel?initial_.accel_bias_mps2(j):initial_.gyro_bias_radps(j);
  healthy[axis]=I(bias)+I(-radius,radius);measured[axis]=accel?sample.specific_force_mps2(j):sample.angular_velocity_radps(j);
  if(!std::isfinite(radius) || !std::isfinite(healthy[axis].lower()) || !std::isfinite(healthy[axis].upper())){result_.valid=false;result_.status="NUMERIC_REFUSED";return;}
  violation=violation || measured[axis]<healthy[axis].lower() || measured[axis]>healthy[axis].upper();
 }
 result_.current_violation=violation;
 if(violation && result_.nominal_possible){result_.first_alarm=sample.timestamp;result_.nominal_possible=false;}
 result_.alarm=!result_.nominal_possible;result_.possible=0;result_.examined=0;
 long long begin=std::numeric_limits<long long>::max(),end=0;
 for(auto& candidate:candidates_)if(candidate.possible){++result_.examined;
  bool active=!(sample.timestamp<candidate.episode.begin) && sample.timestamp<candidate.episode.end;
  for(int axis=0;axis<6;++axis){
   if(active && axis==candidate.episode.axis){I theta=I(measured[axis])-healthy[axis];candidate.theta_low=std::max(candidate.theta_low,theta.lower());candidate.theta_high=std::min(candidate.theta_high,theta.upper());if(candidate.theta_low>candidate.theta_high)candidate.possible=false;}
   else if(measured[axis]<healthy[axis].lower() || measured[axis]>healthy[axis].upper())candidate.possible=false;
  }
  if(candidate.possible){++result_.possible;begin=std::min(begin,(long long)candidate.episode.begin.value());end=std::max(end,(long long)candidate.episode.end.value());}
 }
 recent_.push_back(sample);if(recent_.size()>11)recent_.pop_front();
 ++result_.samples;result_.union_begin=TimestampNs(begin==std::numeric_limits<long long>::max()?0:begin);result_.union_end=TimestampNs(end);
 result_.status=result_.nominal_possible?"NOMINAL_POSSIBLE":result_.possible?"IMU_EPISODE_ALARM":"SCOPE_INCONSISTENT";
}
bool Sc2ImuMonitor::matches(const EpochTransaction& tx)const{
 if(!result_.valid || tx.proposed_epoch<1 || tx.proposed_epoch>50 || result_.samples!=int(10*tx.proposed_epoch+1) || recent_.size()!=tx.raw_imu_slice.size())return false;
 for(size_t i=0;i<recent_.size();++i){const auto& a=recent_[i];const auto& b=tx.raw_imu_slice[i];if(a.id!=b.id || a.timestamp!=b.timestamp || a.specific_force_mps2!=b.specific_force_mps2 || a.angular_velocity_radps!=b.angular_velocity_radps || a.quality_flags!=b.quality_flags)return false;}return true;
}
Sc2FaultSupportClosure sc2SupportClosure(const RawImuEpisode& episode,int through,int horizon){
 Sc2FaultSupportClosure out;
 if(episode.axis<0 || episode.axis>5 || episode.event_id.empty() || episode.begin.value()<5000000 || !(episode.begin<episode.end) || episode.begin.value()%5000000 || episode.end.value()%5000000 || horizon!=50 || through<1 || through>horizon)return out;
 std::set<size_t> pims;
 for(int index=0;index<=10*through;++index){TimestampNs t(index*5000000LL);if(!(t<episode.begin) && t<episode.end){out.raw_samples.push_back(MeasurementId(index+1));if(index>0)pims.insert((index+9)/10);if(index%10==0 && index/10+1<=size_t(through))pims.insert(index/10+1);}}
 out.direct_pim_epochs.assign(pims.begin(),pims.end());
 if(!pims.empty()){out.first_epoch=*pims.begin();for(size_t epoch=out.first_epoch;epoch<=size_t(through);++epoch)out.reference_dependent_epochs.push_back(epoch);}
 out.valid=true;return out;
}
}
