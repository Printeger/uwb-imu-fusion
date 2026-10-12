#include "raw_nonlinear_inclusion.hpp"
#include <boost/numeric/interval.hpp>
#include <boost/numeric/interval/rounded_arith.hpp>
#include <array>
#include <deque>
#include <cmath>
#include <stdexcept>
namespace uwb_imu_pl::research {
namespace {
using I=boost::numeric::interval<double>;
using V=std::array<I,3>;using M=std::array<V,3>;
I norm(const V& v){I s(0);for(auto x:v)s+=square(x);return sqrt(s);}
V fixed(const Eigen::Vector3d& x){return {I(x.x()),I(x.y()),I(x.z())};}
M rotation(const Eigen::Quaterniond& q){
 // Normalize the represented quaternion with directed arithmetic. No libm
 // trigonometric or unverified Eigen matrix arithmetic enters the proof.
 I w(q.w()),x(q.x()),y(q.y()),z(q.z());I n=sqrt(square(w)+square(x)+square(y)+square(z));
 if(n.lower()<=0)throw std::runtime_error("invalid initial attitude");w/=n;x/=n;y/=n;z/=n;
 return {{{1.-2.*(square(y)+square(z)),2.*(x*y-z*w),2.*(x*z+y*w)},
          {2.*(x*y+z*w),1.-2.*(square(x)+square(z)),2.*(y*z-x*w)},
          {2.*(x*z-y*w),2.*(y*z+x*w),1.-2.*(square(x)+square(y))}}};
}
V multiply(const M& a,const V& v){V out{I(0),I(0),I(0)};for(int j=0;j<3;++j)for(int k=0;k<3;++k)out[j]+=a[j][k]*v[k];return out;}
M identity(){return {{{I(1),I(0),I(0)},{I(0),I(1),I(0)},{I(0),I(0),I(1)}}};}
M product(const M& a,const M& b){M c;for(int j=0;j<3;++j)for(int k=0;k<3;++k){c[j][k]=I(0);for(int n=0;n<3;++n)c[j][k]+=a[j][n]*b[n][k];}return c;}
std::pair<M,M> expAndV(const V& phi){
 M a{{{I(0),-phi[2],phi[1]},{phi[2],I(0),-phi[0]},{-phi[1],phi[0],I(0)}}};double alpha=0;
 for(const auto& row:a){I sum(0);for(auto x:row)sum+=abs(x);alpha=std::max(alpha,sum.upper());}
 if(alpha>=20)throw std::runtime_error("source numerical domain refused");
 M e=identity(),v=identity(),term=identity();I scalar(1);
 for(int k=1;k<=48;++k){term=product(term,a);scalar*=I(alpha)/double(k);for(int j=0;j<3;++j)for(int n=0;n<3;++n){term[j][n]/=double(k);e[j][n]+=term[j][n];v[j][n]+=term[j][n]/double(k+1);}}
 I tail=scalar*I(alpha)/49./(1.-I(alpha)/50.);double bound=tail.upper();
 for(int j=0;j<3;++j)for(int n=0;n<3;++n){e[j][n]+=I(-bound,bound);v[j][n]+=I(-bound,bound);}
 return {e,v};
}
I sine(I x){I value(0),power=x,factorial(1);for(int k=1;k<=49;++k){if(k>1){power*=x;factorial*=double(k);}if(k%2==1)value+=(k%4==1?power:-power)/factorial;}
 I rem=abs(power*x)/(factorial*50.);return value+I(-rem.upper(),rem.upper());
}

}
// Independent directed Taylor evaluation of the source-to-input mapping.
// This certifies only floating implementation residue, never estimation error.
RawInputRoundoff certifyRawGenerator(const NavigationState& obs,const Eigen::Matrix<double,15,1>& draws,
 const Eigen::Matrix<double,15,1>& sigmas,const Eigen::Vector3d& amp,const Eigen::Vector3d& freq,
 double rate,double gravity,const Eigen::Matrix<double,6,1>& initialBias,double ba,double bg,double sa,double sg,
 const std::vector<Eigen::Matrix<double,6,1>>& rw,const std::vector<Eigen::Matrix<double,6,1>>& adc,const std::vector<ImuMeasurement>& samples){
 RawInputRoundoff out;if(samples.size()!=adc.size() || rw.size()+1!=samples.size())return out;
 V phi,rho;for(int j=0;j<3;++j){phi[j]=I(sigmas(j))*I(draws(j));rho[j]=I(sigmas(j+3))*I(draws(j+3));}
 auto initial=expAndV(phi);auto observedR=rotation(obs.q_world_body);V pos=multiply(initial.second,rho);pos[2]+=I(1);V dp;I dr(0);
 for(int j=0;j<3;++j){dp[j]=I(obs.position_world_m(j))-pos[j];for(int k=0;k<3;++k)dr+=square(observedR[j][k]-initial.first[j][k]);out.velocity(j)=abs(I(obs.velocity_world_mps(j))-(I(amp(j))*I(freq(j))+I(sigmas(j+6))*I(draws(j+6)))).upper();}
 out.position=norm(dp).upper();out.rotation_matrix=sqrt(dr).upper();
 for(int j=0;j<6;++j){double observed=j<3?obs.accel_bias_mps2(j):obs.gyro_bias_radps(j-3);out.bias(j)=abs(I(observed)-(I(initialBias(j))+I(sigmas(j+9))*I(draws(j+9)))).upper();}
 std::array<I,6> bias;for(int j=0;j<6;++j)bias[j]=I(initialBias(j));I dt=I(1)/200.;I axisnorm=sqrt(I(14));
 for(size_t i=0;i<samples.size();++i){I t=double(i)*dt;V angle,acc;
  for(int j=0;j<3;++j){angle[j]=I(rate)*t*double(j+1)/axisnorm;acc[j]=-I(amp(j))*square(I(freq(j)))*sine(I(freq(j))*t);}acc[2]+=I(gravity);
  auto r=expAndV(angle).first;V force{I(0),I(0),I(0)};for(int j=0;j<3;++j)for(int k=0;k<3;++k)force[j]+=r[k][j]*acc[k];
  if(i)for(int j=0;j<6;++j)bias[j]+=I(j<3?ba:bg)*sqrt(dt)*I(rw[i-1](j));
  for(int j=0;j<3;++j){I f=force[j]+bias[j]+I(sa)*I(adc[i](j));I g=I(rate)*double(j+1)/axisnorm+bias[j+3]+I(sg)*I(adc[i](j+3));out.accel_sample=std::max(out.accel_sample,abs(I(samples[i].specific_force_mps2(j))-f).upper());out.gyro_sample=std::max(out.gyro_sample,abs(I(samples[i].angular_velocity_radps(j))-g).upper());}
 }
 out.valid=std::isfinite(out.position+out.rotation_matrix+out.accel_sample+out.gyro_sample) && out.velocity.allFinite() && out.bias.allFinite() && out.position<=1e-10 && out.rotation_matrix<=1e-10 && out.accel_sample<=1e-10 && out.gyro_sample<=1e-10 && out.velocity.maxCoeff()<=1e-10 && out.bias.maxCoeff()<=1e-10;return out;
}
struct RawNonlinearInclusion::Impl {
 RawInclusionParameters law;NavigationState initial;M r;V p,v;
 std::array<I,4> component_p{I(0),I(0),I(0),I(0)},component_v{I(0),I(0),I(0),I(0)};
 I angular;std::deque<ImuMeasurement> recent;ImuMeasurement previous;int samples=0;bool initialized=false;
 Impl(const NavigationState& obs,const RawInclusionParameters& param):law(param),initial(obs),r(rotation(obs.q_world_body)){
  if(!law.numeric.valid || !std::isfinite(law.risk_budget) || law.risk_budget<=0 || law.risk_budget>=1 || law.horizon!=50 || law.coordinates!=6421 || !law.initial_sigmas.allFinite() || (law.initial_sigmas.array()<=0).any() || !(law.accel_adc>0 && law.gyro_adc>0 && law.accel_rw>0 && law.gyro_rw>0 && law.force_lipschitz>=0 && law.gyro_lipschitz>=0) || !obs.position_world_m.allFinite() || !obs.velocity_world_mps.allFinite() || !obs.accel_bias_mps2.allFinite() || !obs.gyro_bias_radps.allFinite())throw std::runtime_error("unregistered inclusion law");
  I acceleration(0),jerk(0);for(int j=0;j<3;++j){I a(std::abs(law.motion_amplitude(j))),f(std::abs(law.motion_frequency(j)));acceleration+=a*square(f);jerk+=a*square(f)*f;}
  I required=I(std::abs(law.motion_angular_rate))*(I(law.gravity)+acceleration)+jerk;
  if(required.upper()>law.force_lipschitz || law.gyro_lipschitz!=0)throw std::runtime_error("continuous motion derivative domain refused");
  // IEEE directed arithmetic witness. Reject a compiler that folds away
  // rounding-sensitive addition, rather than adding a nominal epsilon.
  I a(1),b(std::ldexp(1.,-53));I c=a+b;
  if(c.lower()>1 || c.upper()<std::nextafter(1.,INFINITY))throw std::runtime_error("directed arithmetic unavailable");
  V rho,theta;for(int j=0;j<3;++j){rho[j]=7.*I(law.initial_sigmas(j+3));theta[j]=7.*I(law.initial_sigmas(j));}
  double radius=(norm(rho)+I(law.numeric.position)).upper();angular=I(0,(norm(theta)+I(law.numeric.rotation_matrix)).upper());
  for(int j=0;j<3;++j){p[j]=I(obs.position_world_m(j))+I(-radius,radius);double ev=(7.*I(law.initial_sigmas(j+6))+I(law.numeric.velocity(j))).upper();v[j]=I(obs.velocity_world_mps(j))+I(-ev,ev);}
 }
};
RawNonlinearInclusion::RawNonlinearInclusion(const NavigationState& s,const RawInclusionParameters& p):impl(new Impl(s,p)){}
RawNonlinearInclusion::~RawNonlinearInclusion()=default;
void RawNonlinearInclusion::append(const ImuMeasurement& m){auto& s=*impl;
 if(!m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite() || m.timestamp.value()!=s.samples*5000000LL || m.id.value()!=uint64_t(s.samples+1) || s.samples>10*s.law.horizon)throw std::runtime_error("raw inclusion source/time/horizon refused");
 s.recent.push_back(m);if(s.recent.size()>11)s.recent.pop_front();
 if(!s.initialized){s.previous=m;s.initialized=true;++s.samples;return;}
 const int end=s.samples;I dt=I(1)/200.;
 V force,gyro,ef,eg;
 for(int j=0;j<3;++j){
  force[j]=(I(s.previous.specific_force_mps2(j))+I(m.specific_force_mps2(j)))/2.-I(s.initial.accel_bias_mps2(j));
  gyro[j]=(I(s.previous.angular_velocity_radps(j))+I(m.angular_velocity_radps(j)))/2.-I(s.initial.gyro_bias_radps(j));
  // All previous RW increments are the SAME primitive variables across H/P.
  // Coordinate-event triangle envelope; no per-epoch probability refresh.
  ef[j]=7.*(I(s.law.initial_sigmas(j+9))+double(end)*I(s.law.accel_rw)*sqrt(dt)+I(s.law.accel_adc))+I(s.law.numeric.bias(j))+I(s.law.numeric.accel_sample);
  eg[j]=7.*(I(s.law.initial_sigmas(j+12))+double(end)*I(s.law.gyro_rw)*sqrt(dt)+I(s.law.gyro_adc))+I(s.law.numeric.bias(j+3))+I(s.law.numeric.gyro_sample);
 }
 I omega=norm(gyro)+norm(eg)+I(s.law.gyro_lipschitz)*dt/2.;
 s.angular+=dt*omega;
 double diameter=std::min(2.,s.angular.upper());
 I error=I(diameter)*norm(force)+norm(ef)+I(s.law.force_lipschitz)*dt/2.;
 V initialBias,rw,adc;
 for(int j=0;j<3;++j){initialBias[j]=7.*I(s.law.initial_sigmas(j+9))+I(s.law.numeric.bias(j));rw[j]=7.*double(end)*I(s.law.accel_rw)*sqrt(dt);adc[j]=7.*I(s.law.accel_adc)+I(s.law.numeric.accel_sample);}
 std::array<I,4> contributions{norm(adc),norm(initialBias),norm(rw),I(diameter)*norm(force)+I(s.law.force_lipschitz)*dt/2.};
 for(int k=0;k<4;++k){s.component_p[k]+=dt*s.component_v[k]+square(dt)*contributions[k]/2.;s.component_v[k]+=dt*contributions[k];}
 auto acc=multiply(s.r,force);acc[2]-=I(s.law.gravity);
 // Exact continuous double integral: error(t) is bounded uniformly on the
 // whole interval by Lipschitz and rotation inequalities, not corner tests.
 for(int j=0;j<3;++j){I enclosure=acc[j]+I(-error.upper(),error.upper());s.p[j]+=dt*s.v[j]+square(dt)*enclosure/2.;s.v[j]+=dt*enclosure;}
 s.previous=m;++s.samples;
}
RawInclusionEvidence RawNonlinearInclusion::tag(const Eigen::Vector3d& lever,const Eigen::Vector3d& actual)const{const auto& s=*impl;RawInclusionEvidence e;
 if(!s.initialized || (s.samples-1)%10!=0 || !lever.allFinite() || !actual.allFinite())return e;
 e.epoch=(s.samples-1)/10;if(e.epoch<1 || e.epoch>s.law.horizon)return e;
 auto offset=multiply(s.r,fixed(lever));e.rotation_diameter=std::min(2.,s.angular.upper());
 I error=I(e.rotation_diameter)*norm(fixed(lever));
 V rho;for(int j=0;j<3;++j)rho[j]=7.*I(s.law.initial_sigmas(j+3));I time=double(s.samples-1)/200.;
 for(int j=0;j<3;++j)e.initial(j)=(norm(rho)+I(s.law.numeric.position)+time*(7.*I(s.law.initial_sigmas(j+6))+I(s.law.numeric.velocity(j)))).upper();
 e.adc=s.component_p[0].upper();e.initial_bias=s.component_p[1].upper();e.bias_rw=s.component_p[2].upper();e.rotation_discretization=s.component_p[3].upper();e.lever=error.upper();
 for(int j=0;j<3;++j){I value=s.p[j]+offset[j]+I(-error.upper(),error.upper());e.low(j)=value.lower();e.high(j)=value.upper();e.reference(j)=value.lower()/2+value.upper()/2.;e.pl(j)=abs(value-I(actual(j))).upper();}
 // Chernoff: P(|Z|>7)<=2 exp(-49/2)<2/2^35, since ln(2)<.7.
 // Union over the ENTIRE inventory, without assuming independent projections.
 e.hpl=sqrt(square(I(e.pl.x()))+square(I(e.pl.y()))).upper();
 e.event_risk=std::ldexp(2.*s.law.coordinates,-35);
 e.valid=e.low.allFinite() && e.high.allFinite() && e.pl.allFinite() && std::isfinite(e.hpl) && e.event_risk<=std::min(1e-6,s.law.risk_budget);return e;
}
RawInclusionEvidence RawNonlinearInclusion::issue(const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever)const{
 auto e=tag(lever,state.position_world_m+state.q_world_body*lever);const auto& s=*impl;
 e.receipt_bound=e.valid && tx.proposed_epoch==size_t(e.epoch) && tx.previous_epoch+1==tx.proposed_epoch && tx.begin.value()==(e.epoch-1)*50000000LL && tx.end.value()==e.epoch*50000000LL && tx.raw_imu_slice.size()==s.recent.size() && receipt.transaction_id==tx.id && receipt.committed_epoch==tx.proposed_epoch && receipt.state_timestamp==tx.end && state.timestamp==tx.end && receipt.graph_version==tx.base_graph_version+1 && receipt.backend_updates==1;
 if(e.receipt_bound)for(size_t i=0;i<s.recent.size();++i){const auto& a=s.recent[i];const auto& b=tx.raw_imu_slice[i];e.receipt_bound=e.receipt_bound && a.id==b.id && a.timestamp==b.timestamp && a.specific_force_mps2==b.specific_force_mps2 && a.angular_velocity_radps==b.angular_velocity_radps;}
 e.valid=e.valid && e.receipt_bound;return e;
}

}
