#include "joint_raw.hpp"
#include "physical_time.hpp"
#include <map>
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
I sym(I r){return I(-r.upper(),r.upper());}
bool meet(I& a,I b){double low=std::max(a.lower(),b.lower()),high=std::min(a.upper(),b.upper());if(!std::isfinite(low) || !std::isfinite(high))throw std::invalid_argument("JOINT_NONFINITE_INTERVAL");if(low>high)return false;a=I(low,high);return true;}
I hull(I a,I b){return I(std::min(a.lower(),b.lower()),std::max(a.upper(),b.upper()));}
void unite(RollingRawState& a,const RollingRawState& b){for(int j=0;j<3;++j){a.p[j]=hull(a.p[j],b.p[j]);a.v[j]=hull(a.v[j],b.v[j]);for(int k=0;k<3;++k)a.rotation[j][k]=hull(a.rotation[j][k],b.rotation[j][k]);}for(int j=0;j<6;++j)a.bias[j]=hull(a.bias[j],b.bias[j]);}
// Necessary shell projection with BOTH signs retained unless the existing
// force interval proves a sign. All other coordinates retain outward bounds.
bool forceShell(std::array<I,6>& bias,const Eigen::Vector3d& measured,I noise,I lower,I upper){
 for(int pass=0;pass<3;++pass)for(int j=0;j<3;++j){I rest(0);for(int k=0;k<3;++k)if(k!=j)rest+=square(I(measured(k))-bias[k]+noise);
  I cap=square(upper)-I(rest.lower());if(cap.upper()<0)return false;double maxabs=sqrt(I(0,std::max(0.,cap.upper()))).upper();I allowed(-maxabs,maxabs);
  I floor=square(lower)-I(rest.upper());double minabs=sqrt(I(std::max(0.,floor.lower()),std::max(0.,floor.upper()))).lower();I current=I(measured(j))-bias[j]+noise;
  if(current.lower()>=0)allowed=I(minabs,maxabs);else if(current.upper()<=0)allowed=I(-maxabs,-minabs);
  if(!meet(bias[j],I(measured(j))+noise-allowed))return false;
 }return true;
}
bool sameState(const RollingRawState& a,const RollingRawState& b){auto same=[](I x,I y){return x.lower()==y.lower() && x.upper()==y.upper();};if(a.epoch!=b.epoch || a.has_tag_relation!=b.has_tag_relation || a.tag_lever!=b.tag_lever)return false;for(int j=0;j<3;++j){if(!same(a.p[j],b.p[j]) || !same(a.v[j],b.v[j]))return false;for(int k=0;k<3;++k)if(!same(a.rotation[j][k],b.rotation[j][k]))return false;if(a.has_tag_relation && !same(a.tag_relation[j],b.tag_relation[j]))return false;}for(int j=0;j<6;++j)if(!same(a.bias[j],b.bias[j]))return false;return true;}
void validate(const JointRawLaw& l){if(!std::isfinite(l.gravity+l.acceleration+l.rate+l.jerk+l.accel_adc+l.gyro_adc+l.accel_rw+l.gyro_rw) || l.gravity<=0 || std::min({l.acceleration,l.rate,l.jerk,l.accel_adc,l.gyro_adc,l.accel_rw,l.gyro_rw})<0)throw std::invalid_argument("JOINT_LAW_REFUSED");}
}
bool jointInitial(RollingRawState& s,const ImuMeasurement& m,const JointRawLaw& l,const TimeUniformEvent& event){
 validate(l);if(s.epoch || m.id.value()!=1 || m.timestamp.value()!=0 || !m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite())throw std::invalid_argument("JOINT_INITIAL_SOURCE_SCOPE");
 auto result=s;I na=sym(I(event.adcRadius(0))*I(l.accel_adc)+I(2e-10)),ng=sym(I(event.adcRadius(0))*I(l.gyro_adc)+I(2e-10));
 if(!forceShell(result.bias,m.specific_force_mps2,na,I(std::max(0.,(I(l.gravity)-I(l.acceleration)).lower())),I(l.gravity)+I(l.acceleration)))return false;
 for(int j=0;j<3;++j){if(!meet(result.bias[j],I(m.specific_force_mps2(j))-I(l.gravity)*result.rotation[2][j]+sym(I(l.acceleration))+na) || !meet(result.bias[j+3],I(m.angular_velocity_radps(j))+sym(I(l.rate))+ng))return false;
  if(!meet(result.rotation[2][j],(I(m.specific_force_mps2(j))-result.bias[j]+sym(I(l.acceleration))+na)/I(l.gravity)))return false;}
 s=result;return true;
}
JointConstraintEvidence jointContract(RollingRawState& old,RollingRawState& next,const std::vector<ImuMeasurement>& raw,const JointRawLaw& l,const TimeUniformEvent& event,size_t budget,bool dominant_hull_shortcut){
 validate(l);if(next.epoch!=old.epoch+1 || raw.size()!=11)throw std::invalid_argument("JOINT_WINDOW_ORDER");
 for(size_t i=0;i<raw.size();++i)if(raw[i].id.value()!=old.epoch*10+i+1 || raw[i].timestamp.value()!=int64_t(old.epoch*10+i)*5000000LL || !raw[i].specific_force_mps2.allFinite() || !raw[i].angular_velocity_radps.allFinite())throw std::invalid_argument("JOINT_SHARED_SOURCE_GAP");
 JointConstraintEvidence evidence;evidence.first_source=raw.front().id.value();evidence.last_source=raw.back().id.value();
 for(int j=0;j<6;++j)evidence.before[j]=next.bias[j].upper()-next.bias[j].lower();
 auto original_old=old,original_next=next;RollingRawState union_old,union_next;bool any=false;
 const I ds=I(1)/200.,dt=I(1)/20.;const double q=event.block(next.epoch).q;const I accel_noise=sym(I(q)*I(l.accel_adc)+I(2e-10)),gyro_noise=sym(I(q)*I(l.gyro_adc)+I(2e-10));const I accel_step=I(q)*I(l.accel_rw)*sqrt(ds)+I(2e-10),gyro_step=I(q)*I(l.gyro_rw)*sqrt(ds)+I(2e-10);
 // Common physical integral image bounds every branch's final p/v image.
 // Evaluate with the identical sum/order as the measurement integrals, so
 // monotonic directed arithmetic preserves inclusion including roundoff.
 auto common_old=original_old,common_next=original_next;RollingVector max_v{I(0),I(0),I(0)},max_p=max_v;
 for(int i=1;i<=10;++i)for(int j=0;j<3;++j){max_v[j]+=ds*sym(I(l.acceleration));max_p[j]+=ds*(dt-(double(i)-.5)*ds)*sym(I(l.acceleration));}
 bool common_possible=true;for(int j=0;j<3;++j){I dv=max_v[j]+sym(I(l.jerk)*dt*ds/2.+I(4e-10)),dp=max_p[j]+sym(I(l.jerk)*square(dt)*ds/2.+I(4e-10));common_possible&=meet(common_next.v[j],common_old.v[j]+dv);common_possible&=meet(common_old.v[j],common_next.v[j]-dv);common_possible&=meet(common_next.p[j],common_old.p[j]+dt*common_old.v[j]+dp);common_possible&=meet(common_old.p[j],common_next.p[j]-dt*common_old.v[j]-dp);common_possible&=meet(common_old.v[j],(common_next.p[j]-common_old.p[j]-dp)/dt);}
 if(!common_possible)throw std::invalid_argument("JOINT_COMMON_PHYSICS_INCONSISTENT");
 // Fixed coordinate order; no GT, FDE-selected axis or probabilistic pruning.
 const std::array<int,7> order{-1,2,5,0,1,3,4};
 try{for(size_t order_index=0;order_index<order.size();++order_index){int omitted=order[order_index];++evidence.branches;auto a=original_old,b=original_next;bool possible=true;
   if(order_index>=budget){++evidence.frontier;}
   else{
    I theta;bool theta_started=false;RollingVector integral_v{I(0),I(0),I(0)},integral_p=integral_v;
    for(int i=1;i<=10 && possible;++i){const auto& m=raw[i];I t=double(i)*ds,remaining=dt-t;std::array<RollingVector,3> rotation;std::array<I,6> bias,signal;
     for(int j=0;j<3;++j)for(int k=0;k<3;++k){rotation[j][k]=a.rotation[j][k]+sym(I(l.rate)*t);possible&=meet(rotation[j][k],b.rotation[j][k]+sym(I(l.rate)*remaining));possible&=meet(rotation[j][k],I(-1,1));}
     for(int axis=0;axis<6 && possible;++axis){const bool accel=axis<3;int j=axis%3;I rw=accel?accel_step:gyro_step;I from=double(i)*rw,to=double(10-i)*rw;
      bias[axis]=a.bias[axis]+sym(from);possible&=meet(bias[axis],b.bias[axis]+sym(to));if(!possible)break;
      I noise=accel?accel_noise:gyro_noise;I measured(accel?m.specific_force_mps2(j):m.angular_velocity_radps(j));
      I physical=accel?I(l.gravity)*rotation[2][j]+sym(I(l.acceleration)):sym(I(l.rate));
      if(axis==omitted){I candidate=measured-bias[axis]-physical+noise;if(!theta_started){theta=candidate;theta_started=true;}else possible&=meet(theta,candidate);if(!possible)break;possible&=meet(bias[axis],measured-theta-physical+noise);signal[axis]=measured-bias[axis]-theta+noise;}
      else{possible&=meet(bias[axis],measured-physical+noise);signal[axis]=measured-bias[axis]+noise;}
      if(!possible)break;possible&=meet(signal[axis],physical);possible&=meet(a.bias[axis],bias[axis]+sym(from));possible&=meet(b.bias[axis],bias[axis]+sym(to));
     }
     if(!possible)break;
     for(int j=0;j<3;++j){I acceleration(0);for(int k=0;k<3;++k)acceleration+=rotation[j][k]*signal[k];if(j==2)acceleration-=I(l.gravity);possible&=meet(acceleration,sym(I(l.acceleration)));integral_v[j]+=ds*acceleration;integral_p[j]+=ds*(dt-(double(i)-.5)*ds)*acceleration;}
    }
    // Right-endpoint quadrature for continuous bounded-jerk acceleration.
    // Bias RW is sample-grid, never treated as a differentiable process.
    if(possible)for(int j=0;j<3;++j){I dv=integral_v[j]+sym(I(l.jerk)*dt*ds/2.+I(4e-10)),dp=integral_p[j]+sym(I(l.jerk)*square(dt)*ds/2.+I(4e-10));
     possible&=meet(b.v[j],a.v[j]+dv);possible&=meet(a.v[j],b.v[j]-dv);possible&=meet(b.p[j],a.p[j]+dt*a.v[j]+dp);possible&=meet(a.p[j],b.p[j]-dt*a.v[j]-dp);possible&=meet(a.v[j],(b.p[j]-a.p[j]-dp)/dt);}
   }
   if(!possible){++evidence.empty;continue;}
   for(int j=0;j<6;++j)evidence.branch_widths[omitted+1][j]=b.bias[j].upper()-b.bias[j].lower();
   if(!any){union_old=a;union_next=b;any=true;}else{unite(union_old,a);unite(union_next,b);}
   // Exact union-hull dominance: one branch equal to X fixes hull=X since
   // every other contractor is a subset of X. Preserve every unprocessed
   // branch as its original X frontier; no physical explanation is pruned.
   if(dominant_hull_shortcut && ((sameState(union_old,original_old) && sameState(union_next,original_next)) || (budget>=7 && sameState(union_old,common_old) && sameState(union_next,common_next)))){
    const size_t remaining=order.size()-order_index-1;evidence.frontier+=remaining;evidence.branches+=remaining;
    for(size_t k=order_index+1;k<order.size();++k)evidence.branch_widths[order[k]+1]=evidence.before;
    break;
   }
  }
  if(!any)throw std::invalid_argument("JOINT_ALL_BRANCHES_INCONSISTENT");
  old=rollingProject(union_old);next=rollingProject(union_next);evidence.valid=true;
 }catch(const std::exception& error){if(std::string(error.what())=="JOINT_ALL_BRANCHES_INCONSISTENT")throw;old=original_old;next=original_next;evidence.fallback=true;evidence.valid=true;evidence.branches=7;evidence.frontier=7;evidence.empty=0;for(auto& branch:evidence.branch_widths)branch=evidence.before;}
 for(int j=0;j<6;++j)evidence.after[j]=next.bias[j].upper()-next.bias[j].lower();
 return evidence;
}

JointConstraintEvidence jointContract(RollingRawState& old,RollingRawState& next,
 TimestampNs begin,TimestampNs end,const std::vector<JointTimedSample>& samples,
 const JointRawLaw& law,size_t budget,bool reuse_common){
 validate(law);
 if(old.epoch==std::numeric_limits<size_t>::max() || next.epoch!=old.epoch+1 || samples.empty())
  throw std::invalid_argument("JOINT_TIMED_WINDOW_ORDER");
 const I duration=durationSeconds(begin,end);
 TimestampNs previous=begin;
 std::map<MeasurementId,const JointTimedSample*> parents;
 // Validate the complete source schedule before any set mutation. Timing and
 // parent identity errors are input failures, never numeric fallback success.
 for(const auto& s:samples){
  if(!(previous<s.integration_end) || end<s.integration_end || s.integration_end<s.raw.timestamp ||
     !s.raw.specific_force_mps2.allFinite() || !s.raw.angular_velocity_radps.allFinite())
   throw std::invalid_argument("JOINT_TIMED_SOURCE_INVALID");
  auto inserted=parents.emplace(s.raw.id,&s);
  if(!inserted.second){const auto& prior=*inserted.first->second;const auto& p=prior.raw;
   if(p.timestamp!=s.raw.timestamp || p.specific_force_mps2!=s.raw.specific_force_mps2 ||
      p.angular_velocity_radps!=s.raw.angular_velocity_radps)
    throw std::invalid_argument("JOINT_TIMED_PARENT_CONFLICT");
   for(int axis=0;axis<6;++axis)if(prior.adc_radius[axis].lower()!=s.adc_radius[axis].lower() ||
       prior.adc_radius[axis].upper()!=s.adc_radius[axis].upper())
    throw std::invalid_argument("JOINT_TIMED_PARENT_NOISE_CONFLICT");
  }
  for(const auto& radii:{s.adc_radius,s.rw_from_begin,s.rw_to_end})for(auto r:radii)
   if(!std::isfinite(r.lower()) || !std::isfinite(r.upper()) || r.lower()<0)
    throw std::invalid_argument("JOINT_TIMED_SOURCE_RADIUS");
  previous=s.integration_end;
 }
 if(previous!=end)throw std::invalid_argument("JOINT_TIMED_END_MISSING");
 JointConstraintEvidence evidence;
 evidence.method="sc9-nonuniform-raw-imu-state-integral-v2";
 evidence.fault_relaxation="ALL7_PER_RAW_SAMPLE_AXIS_UNION_ARBITRARY_PHASE_OUTER";
 evidence.first_source=samples.front().raw.id.value();evidence.last_source=samples.back().raw.id.value();
 for(int j=0;j<6;++j)evidence.before[j]=next.bias[j].upper()-next.bias[j].lower();
 const auto original_old=old,original_next=next;
 auto a=old,b=next;
 auto distance=[](TimestampNs x,TimestampNs y){return x==y?I(0):x<y?durationSeconds(x,y):durationSeconds(y,x);};
 RollingVector integral_v{I(0),I(0),I(0)},integral_p=integral_v;
 I remainder_v(0),remainder_p(0);previous=begin;
 try{
  for(const auto& sample:samples){
   const I ds=durationSeconds(previous,sample.integration_end);
   const I remaining=sample.integration_end==end?I(0):durationSeconds(sample.integration_end,end);
   const I weight=ds*(remaining+ds/2.);
   const I age=distance(sample.raw.timestamp,sample.integration_end);
   const I from_time=distance(begin,sample.raw.timestamp),to_time=distance(sample.raw.timestamp,end);
   RollingRawState union_old,union_next;RollingVector acceleration_union;bool any=false;
   // x/y differ between branches only in their per-axis bias constraints.
   // Rotation and an axis's healthy/omitted image therefore have identical
   // inputs across branches. Cache only inside this one raw sample. Lazy
   // evaluation preserves the original short-circuit and fallback behavior.
   std::array<RollingVector,3> rotation;bool rotation_ready=false,rotation_possible=false;
   struct AxisImage {bool ready=false,possible=false;I signal,before,after;};
   std::array<std::array<AxisImage,2>,6> images;
   for(int branch=0;branch<7;++branch){
    const int omitted=branch-1;++evidence.branches;
    auto x=a,y=b;RollingVector acceleration{sym(I(law.acceleration)),sym(I(law.acceleration)),sym(I(law.acceleration))};
    bool possible=true;
    if(size_t(branch)>=budget){++evidence.frontier;}
    else{
     std::array<I,6> signal;
     if(!reuse_common || !rotation_ready){rotation_possible=true;
      for(int j=0;j<3;++j)for(int k=0;k<3;++k){
       rotation[j][k]=a.rotation[j][k]+sym(I(law.rate)*from_time);
       rotation_possible&=meet(rotation[j][k],b.rotation[j][k]+sym(I(law.rate)*to_time));
       rotation_possible&=meet(rotation[j][k],I(-1,1));
      }rotation_ready=true;
     }
     possible=rotation_possible;
     for(int axis=0;axis<6 && possible;++axis){
      auto& image=images[axis][axis==omitted?1:0];
      if(!reuse_common || !image.ready){image.ready=true;const bool accel=axis<3;const int j=axis%3;
       I bias=a.bias[axis]+sym(sample.rw_from_begin[axis]);
       image.possible=meet(bias,b.bias[axis]+sym(sample.rw_to_end[axis]));
       if(image.possible){
        I physical=accel?I(law.gravity)*rotation[2][j]+sym(I(law.acceleration)):sym(I(law.rate));
        if(axis==omitted)image.signal=physical; // Arbitrary fault at this raw time.
        else{
         I measured(accel?sample.raw.specific_force_mps2[j]:sample.raw.angular_velocity_radps[j]);
         const I noise=sym(sample.adc_radius[axis]);
         image.possible&=meet(bias,measured-physical+noise);
         image.signal=measured-bias+noise;
         image.possible&=meet(image.signal,physical);
        }
        image.before=a.bias[axis];image.after=b.bias[axis];
        image.possible&=meet(image.before,bias+sym(sample.rw_from_begin[axis]));
        image.possible&=meet(image.after,bias+sym(sample.rw_to_end[axis]));
       }
      }
      possible&=image.possible;
      if(possible){x.bias[axis]=image.before;y.bias[axis]=image.after;signal[axis]=image.signal;}
     }
     if(possible)for(int j=0;j<3;++j){
      I measured_acceleration(0);for(int k=0;k<3;++k)measured_acceleration+=rotation[j][k]*signal[k];
      if(j==2)measured_acceleration-=I(law.gravity);
      possible&=meet(acceleration[j],measured_acceleration);
     }
    }
    if(!possible){++evidence.empty;continue;}
    if(!any){union_old=x;union_next=y;acceleration_union=acceleration;any=true;}
    else{if(reuse_common){for(int j=0;j<6;++j){union_old.bias[j]=hull(union_old.bias[j],x.bias[j]);union_next.bias[j]=hull(union_next.bias[j],y.bias[j]);}}
     else{unite(union_old,x);unite(union_next,y);}
     for(int j=0;j<3;++j)acceleration_union[j]=hull(acceleration_union[j],acceleration[j]);}
   }
   if(!any)throw std::invalid_argument("JOINT_ALL_BRANCHES_INCONSISTENT");
   a=rollingProject(union_old);b=rollingProject(union_next);
   for(int j=0;j<3;++j){integral_v[j]+=ds*acceleration_union[j];integral_p[j]+=weight*acceleration_union[j];}
   // For right sample t_i and arbitrary held parent t_s<=t_i:
   // |a(t)-a(t_s)| <= J*((t_i-t)+|t_i-t_s|).
   // Integrate once and against (T-t), retaining nonuniform third-order term.
   remainder_v+=I(law.jerk)*(square(ds)/2.+age*ds);
   remainder_p+=I(law.jerk)*(remaining*square(ds)/2.+ds*square(ds)/3.+age*weight);
   previous=sample.integration_end;
  }
  for(int j=0;j<3;++j){
   const I dv=integral_v[j]+sym(remainder_v),dp=integral_p[j]+sym(remainder_p);
   if(!meet(b.v[j],a.v[j]+dv) || !meet(a.v[j],b.v[j]-dv) ||
      !meet(b.p[j],a.p[j]+duration*a.v[j]+dp) || !meet(a.p[j],b.p[j]-duration*a.v[j]-dp) ||
      !meet(a.v[j],(b.p[j]-a.p[j]-dp)/duration))throw std::invalid_argument("JOINT_ALL_BRANCHES_INCONSISTENT");
  }
  old=rollingProject(a);next=rollingProject(b);evidence.valid=true;
 }catch(const std::exception& error){
  if(std::string(error.what())=="JOINT_ALL_BRANCHES_INCONSISTENT")throw;
  old=original_old;next=original_next;evidence.valid=true;evidence.fallback=true;
  evidence.branches=7*samples.size();evidence.frontier=evidence.branches;evidence.empty=0;
 }
 for(int j=0;j<6;++j)evidence.after[j]=next.bias[j].upper()-next.bias[j].lower();
 return evidence;
}
}
