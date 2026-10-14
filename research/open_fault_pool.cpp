#include "open_fault_pool.hpp"
namespace uwb_imu_pl::research {
OpenFaultPool::OpenFaultPool(const NavigationState& s,const RawInclusionParameters& law,double budget,size_t cap,double acceleration,double rate):event_(budget),law_(law),capacity_(cap),acceleration_(acceleration),rate_(rate){
 if(!std::isfinite(acceleration+rate) || acceleration<0 || rate<0 || !cap || !law.numeric.valid || !law.initial_sigmas.allFinite() || (law.initial_sigmas.array()<=0).any() || law.accel_rw<=0 || law.gyro_rw<=0 || law.accel_adc<=0 || law.gyro_adc<=0 || s.timestamp.value()!=0)throw std::invalid_argument("OPEN_SOURCE_SCOPE_INVALID");
 bias_center_.head<3>()=s.accel_bias_mps2;bias_center_.tail<3>()=s.gyro_bias_radps;
 for(auto& a:rw_support_)a=RollingInterval(0);
 result_.valid=true;result_.status="REPEATED_EPISODE_OUTER_AUTOMATON";
}
void OpenFaultPool::append(const ImuMeasurement& sample){
 if(!result_.valid)throw std::invalid_argument(result_.status);
 if(samples_>uint64_t(INT_MAX-1) || sample.id.value()!=samples_+1 || sample.timestamp.value()!=int64_t(samples_)*5000000LL || !sample.specific_force_mps2.allFinite() || !sample.angular_velocity_radps.allFinite())throw std::invalid_argument("OPEN_GLOBAL_SOURCE_GAP_OR_OVERFLOW");
 using I=RollingInterval;const auto now=sample.timestamp;
 // Every expired possible path can re-enter a no-active state, with its
 // historical state uncertainty already absorbed into the raw boundary.
 for(auto it=active_.begin();it!=active_.end();){if(!(now<it->episode.end)){no_active_=true;++ended_;it=active_.erase(it);}else ++it;}
 if(samples_%10==1){births_+=12;if(no_active_)for(int duration:{50,500})for(int axis=0;axis<6;++axis){
  std::string id="imu:axis:"+std::to_string(axis)+":begin:"+std::to_string(now.value())+":duration:"+std::to_string(duration);
  active_.push_back({{id,now,TimestampNs(now.value()+int64_t(duration)*1000000LL),axis},-INFINITY,INFINITY,true});
 }}
 std::array<I,6> healthy;std::array<double,6> measured;bool violation=false;
 const double q=event_.adcRadius(samples_),q0=event_.block(0).q;
 for(int axis=0;axis<6;++axis){bool accel=axis<3;int j=axis%3;
  if(samples_)rw_support_[axis]+=I(q)*I(accel?law_.accel_rw:law_.gyro_rw)*sqrt(I(1)/200.);
  I noise=I(q0)*I(law_.initial_sigmas(axis+9))+rw_support_[axis]+I(q)*I(accel?law_.accel_adc:law_.gyro_adc)+I(2e-10);
  double radius=(noise+I(accel?law_.gravity+acceleration_:rate_)).upper();healthy[axis]=I(bias_center_(axis))+I(-radius,radius);
  measured[axis]=accel?sample.specific_force_mps2(j):sample.angular_velocity_radps(j);
  diagnostic_.measured[axis]=measured[axis];diagnostic_.healthy_low[axis]=healthy[axis].lower();diagnostic_.healthy_high[axis]=healthy[axis].upper();diagnostic_.rw_radius[axis]=rw_support_[axis].upper();
  violation|=measured[axis]<healthy[axis].lower() || measured[axis]>healthy[axis].upper();
 }
 if(violation)no_active_=false;
 diagnostic_.q=q;diagnostic_.violation=violation;diagnostic_.no_active=no_active_;
 for(auto it=active_.begin();it!=active_.end();){bool possible=true;
  for(int axis=0;axis<6;++axis){if(axis==it->episode.axis){I theta=I(measured[axis])-healthy[axis];it->theta_low=std::max(it->theta_low,theta.lower());it->theta_high=std::min(it->theta_high,theta.upper());possible&=it->theta_low<=it->theta_high;}
   else possible&=healthy[axis].lower()<=measured[axis] && measured[axis]<=healthy[axis].upper();}
  if(!possible){++excluded_;it=active_.erase(it);}else ++it;
 }
 if(!no_active_ && active_.empty()){result_.valid=false;result_.status="CONCURRENCY_OR_MODEL_OUTSIDE_SCOPE";throw std::invalid_argument(result_.status);}
 if(violation && !alert_){
  if(records_.size()==capacity_){result_.valid=false;result_.status="FAULT_IDENTITY_RESOURCE_EXHAUSTED";throw std::invalid_argument(result_.status);}
  alert_=true;++generation_;alert_begin_=now;alert_end_=now;
  result_.first_alarm=now;
  for(const auto& c:active_){if(c.episode.begin<alert_begin_)alert_begin_=c.episode.begin;if(alert_end_<c.episode.end)alert_end_=c.episode.end;}
  records_.push_back({generation_,alert_begin_,alert_end_,"observed-alert:"+std::to_string(generation_)+":at:"+std::to_string(now.value())});
 }
 if(alert_){for(const auto& c:active_)if(!(now<c.episode.begin) && !(c.episode.begin<alert_begin_)){if(alert_end_<c.episode.end && !no_active_)alert_end_=c.episode.end;}
  records_.back().end=alert_end_;if(!violation && no_active_ && !(now<alert_end_))alert_=false;}
 recent_.push_back(sample);if(recent_.size()>11)recent_.pop_front();++samples_;
 result_.samples=int(samples_);result_.nominal_possible=no_active_;result_.possible=active_.size();result_.current_violation=violation;
 result_.alarm=alert_;result_.union_begin=alert_begin_;result_.union_end=alert_end_;
}
RollingMonitorView OpenFaultPool::view()const{
 RollingMonitorView v;v.open=true;v.event_id=event_.id();v.alert_generation=generation_;v.state=result_;
 v.active=active_.size();v.ended=ended_;v.excluded=excluded_;
 v.matches=[this](const EpochTransaction& tx){if(!result_.valid || samples_!=10*tx.proposed_epoch+1 || recent_.size()!=tx.raw_imu_slice.size())return false;
  for(size_t j=0;j<recent_.size();++j){const auto& a=recent_[j];const auto& b=tx.raw_imu_slice[j];if(a.id!=b.id || a.timestamp!=b.timestamp || a.specific_force_mps2!=b.specific_force_mps2 || a.angular_velocity_radps!=b.angular_velocity_radps)return false;}return true;};return v;
}
void OpenFaultPool::appendRanges(const UwbBatch& batch,const RollingRawState& state,const Eigen::Vector3d& lever){
 using I=RollingInterval;const uint64_t k=batch.id.value();
 if(k!=uwb_epoch_+1 || state.epoch!=k || batch.measurements.size()!=8)throw std::invalid_argument("UWB_GLOBAL_CLOCK_OR_SOURCE_GAP");
 for(auto i=uwb_active_.begin();i!=uwb_active_.end();){if(i->end<=k){uwb_no_active_=true;++uwb_ended_;i=uwb_active_.erase(i);}else ++i;}
 if(uwb_no_active_)for(unsigned a=0;a<8;++a)for(unsigned d:{1u,3u})uwb_active_.push_back({k,k+d,a,-INFINITY,INFINITY,"uwb:anchor:"+std::to_string(batch.measurements[a].anchor_id.value())+":begin:"+std::to_string(k)+":duration:"+std::to_string(d)});
 auto tag=rollingTag(state,lever);std::array<I,8> theta;
 bool healthy=true;for(unsigned a=0;a<8;++a){I squared(0);const auto& m=batch.measurements[a];for(int j=0;j<3;++j)squared+=square(tag[j]-I(m.anchor_position_m(j)));
  I noise=I(event_.block(k).q)*I(m.sigma_m)+I(1e-8);
  theta[a]=I(m.range_m)-sqrt(squared)+I(-noise.upper(),noise.upper());
  healthy&=theta[a].lower()<=0 && theta[a].upper()>=0;
 }
 if(!healthy)uwb_no_active_=false;
 for(auto i=uwb_active_.begin();i!=uwb_active_.end();){bool possible=true;for(unsigned a=0;a<8;++a){if(a==i->anchor){i->low=std::max(i->low,theta[a].lower());i->high=std::min(i->high,theta[a].upper());possible&=i->low<=i->high;}else possible&=theta[a].lower()<=0 && theta[a].upper()>=0;}
  if(!possible)i=uwb_active_.erase(i);else ++i;
 }
 if(!uwb_no_active_ && uwb_active_.empty())throw std::invalid_argument("UWB_CONCURRENCY_OR_MODEL_OUTSIDE_SCOPE");
 uwb_epoch_=k;
}
}
