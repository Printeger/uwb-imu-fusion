#include "joint_raw.hpp"
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
JointConstraintEvidence jointContract(RollingRawState& old,RollingRawState& next,const std::vector<ImuMeasurement>& raw,const JointRawLaw& l,const TimeUniformEvent& event,size_t budget){
 validate(l);if(next.epoch!=old.epoch+1 || raw.size()!=11)throw std::invalid_argument("JOINT_WINDOW_ORDER");
 for(size_t i=0;i<raw.size();++i)if(raw[i].id.value()!=old.epoch*10+i+1 || raw[i].timestamp.value()!=int64_t(old.epoch*10+i)*5000000LL || !raw[i].specific_force_mps2.allFinite() || !raw[i].angular_velocity_radps.allFinite())throw std::invalid_argument("JOINT_SHARED_SOURCE_GAP");
 JointConstraintEvidence evidence;evidence.first_source=raw.front().id.value();evidence.last_source=raw.back().id.value();
 for(int j=0;j<6;++j)evidence.before[j]=next.bias[j].upper()-next.bias[j].lower();
 auto original_old=old,original_next=next;RollingRawState union_old,union_next;bool any=false;
 const I ds=I(1)/200.,dt=I(1)/20.;const double q=event.block(next.epoch).q;const I accel_noise=sym(I(q)*I(l.accel_adc)+I(2e-10)),gyro_noise=sym(I(q)*I(l.gyro_adc)+I(2e-10));const I accel_step=I(q)*I(l.accel_rw)*sqrt(ds)+I(2e-10),gyro_step=I(q)*I(l.gyro_rw)*sqrt(ds)+I(2e-10);
 try{for(int omitted=-1;omitted<6;++omitted){++evidence.branches;auto a=original_old,b=original_next;bool possible=true;
   if(size_t(omitted+1)>=budget){++evidence.frontier;}
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
  }
  if(!any)throw std::invalid_argument("JOINT_ALL_BRANCHES_INCONSISTENT");
  old=rollingProject(union_old);next=rollingProject(union_next);evidence.valid=true;
 }catch(const std::exception& error){if(std::string(error.what())=="JOINT_ALL_BRANCHES_INCONSISTENT")throw;old=original_old;next=original_next;evidence.fallback=true;evidence.valid=true;evidence.branches=7;evidence.frontier=7;evidence.empty=0;for(auto& branch:evidence.branch_widths)branch=evidence.before;}
 for(int j=0;j<6;++j)evidence.after[j]=next.bias[j].upper()-next.bias[j].lower();
 return evidence;
}
}
