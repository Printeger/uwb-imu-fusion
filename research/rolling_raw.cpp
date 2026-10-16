#include "rolling_raw.hpp"
#include "physical_time.hpp"
#include <cmath>
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
I norm(const RollingVector& v){I n(0);for(auto x:v)n+=square(x);return sqrt(n);}
I symmetric(I x){return I(-x.upper(),x.upper());}
bool intersect(I& a,I b){double l=std::max(a.lower(),b.lower()),u=std::min(a.upper(),b.upper());if(l>u)return false;a=I(l,u);return true;}
void check(I a){if(!std::isfinite(a.lower()) || !std::isfinite(a.upper()))throw std::runtime_error("RAW_SET_NONFINITE");}
}
RollingRawState rollingInitial(const NavigationState& s,const Eigen::Matrix<double,15,1>& sigma,double q,double rho){
 if(!sigma.allFinite() || (sigma.array()<=0).any() || !std::isfinite(q+rho) || q<=0 || rho<0 || !s.position_world_m.allFinite() || !s.velocity_world_mps.allFinite() || !s.q_world_body.coeffs().allFinite() || !s.accel_bias_mps2.allFinite() || !s.gyro_bias_radps.allFinite())throw std::invalid_argument("RAW_INITIAL_REFUSED");
 I witness=I(1)+I(std::ldexp(1.,-53));if(witness.lower()>1 || witness.upper()<std::nextafter(1.,INFINITY))throw std::runtime_error("DIRECTED_ARITHMETIC_UNAVAILABLE");
 RollingVector phi,rho_vec;for(int j=0;j<3;++j){phi[j]=I(q)*I(sigma(j));rho_vec[j]=I(q)*I(sigma(j+3));}
 I pe=norm(rho_vec)+I(rho),angle=norm(phi)+I(rho);
 I w(s.q_world_body.w()),x(s.q_world_body.x()),y(s.q_world_body.y()),z(s.q_world_body.z());
 I n=sqrt(square(w)+square(x)+square(y)+square(z));if(n.lower()<=0)throw std::runtime_error("RAW_ROTATION_INVALID");w/=n;x/=n;y/=n;z/=n;
 std::array<RollingVector,3> r{{{1.-2.*(square(y)+square(z)),2.*(x*y-z*w),2.*(x*z+y*w)},
  {2.*(x*y+z*w),1.-2.*(square(x)+square(z)),2.*(y*z-x*w)},
  {2.*(x*z-y*w),2.*(y*z+x*w),1.-2.*(square(x)+square(y))}}};
 RollingRawState a;
 for(int j=0;j<3;++j){a.p[j]=I(s.position_world_m(j))+symmetric(pe);a.v[j]=I(s.velocity_world_mps(j))+symmetric(I(q)*I(sigma(j+6))+I(rho));
  for(int k=0;k<3;++k){a.rotation[j][k]=r[j][k]+symmetric(angle);intersect(a.rotation[j][k],I(-1,1));}}
 for(int j=0;j<6;++j)a.bias[j]=I(j<3?s.accel_bias_mps2(j):s.gyro_bias_radps(j-3))+symmetric(I(q)*I(sigma(j+9))+I(rho));
 return rollingProject(a);
}
RollingRawState rollingPropagate(const RollingRawState& old,double acceleration,double angular_rate,double ba,double bg,double q){
 if(!std::isfinite(acceleration+angular_rate+ba+bg+q) || acceleration<0 || angular_rate<0 || ba<0 || bg<0 || q<=0)throw std::invalid_argument("RAW_PROPAGATION_DOMAIN_REFUSED");
 RawTimeStep step;step.duration=I(1)/20.;
 for(int j=0;j<6;++j)step.bias_radius[j]=10.*I(q)*I(j<3?ba:bg)*sqrt(I(1)/200.)+I(2e-10);
 return rollingPropagate(old,acceleration,angular_rate,step);
}
RollingRawState rollingPropagate(const RollingRawState& old,double acceleration,double angular_rate,const RawTimeStep& step){
 if(!std::isfinite(acceleration) || !std::isfinite(angular_rate) || acceleration<0 || angular_rate<0 ||
    !std::isfinite(step.duration.lower()) || !std::isfinite(step.duration.upper()) || step.duration.lower()<=0 ||
    old.epoch==std::numeric_limits<size_t>::max())throw std::invalid_argument("RAW_TIME_PROPAGATION_DOMAIN");
 for(auto r:step.bias_radius)if(!std::isfinite(r.lower()) || !std::isfinite(r.upper()) || r.lower()<0)
  throw std::invalid_argument("RAW_RW_SUPPORT_INVALID");
 RollingRawState a=old;++a.epoch;I dt=step.duration,rp=I(acceleration)*square(dt)/2.,rv=I(acceleration)*dt,rr=I(angular_rate)*dt;
 for(int j=0;j<3;++j){a.p[j]=old.p[j]+dt*old.v[j]+symmetric(rp);a.v[j]=old.v[j]+symmetric(rv);
  for(int k=0;k<3;++k){a.rotation[j][k]=old.rotation[j][k]+symmetric(rr);intersect(a.rotation[j][k],I(-1,1));}}
 // Supplied support is the Minkowski sum of ORIGINAL RW births. A held or
 // interpolated endpoint does not mint another independent posterior draw.
 for(int j=0;j<6;++j)a.bias[j]=old.bias[j]+symmetric(step.bias_radius[j]);
 if(old.has_tag_relation){I lever_norm=sqrt(square(I(old.tag_lever.x()))+square(I(old.tag_lever.y()))+square(I(old.tag_lever.z())));for(int j=0;j<3;++j)a.tag_relation[j]=old.tag_relation[j]+dt*old.v[j]+symmetric(rp+lever_norm*I(angular_rate)*dt);}
 return rollingProject(a);
}
RollingVector rollingTag(const RollingRawState& s,const Eigen::Vector3d& lever){
 if(!lever.allFinite())throw std::invalid_argument("RAW_LEVER_INVALID");
 RollingVector out=s.p;for(int j=0;j<3;++j)for(int k=0;k<3;++k)out[j]+=s.rotation[j][k]*I(lever(k));if(s.has_tag_relation && lever==s.tag_lever)for(int j=0;j<3;++j)if(!intersect(out[j],s.tag_relation[j]))throw std::invalid_argument("TAG_RELATION_EMPTY");return out;
}
bool rollingContractTag(RollingRawState& s,const RollingVector& tag,const Eigen::Vector3d& l){
 if(s.has_tag_relation && l==s.tag_lever)for(int j=0;j<3;++j)if(!intersect(s.tag_relation[j],tag[j]))return false;
 for(int j=0;j<3;++j){I offset(0);for(int k=0;k<3;++k)offset+=s.rotation[j][k]*I(l(k));
  if(!intersect(s.p[j],tag[j]-offset))return false;
  for(int k=0;k<3;++k)if(l(k)!=0){I rhs=tag[j]-s.p[j];for(int h=0;h<3;++h)if(h!=k)rhs-=s.rotation[j][h]*I(l(h));if(!intersect(s.rotation[j][k],rhs/I(l(k))))return false;}
 }return true;
}
bool rollingContractDynamics(RollingRawState& old,RollingRawState& next,double acceleration,I dt){
 if(next.epoch!=old.epoch+1 || !std::isfinite(acceleration) || acceleration<0 ||
    !std::isfinite(dt.lower()) || !std::isfinite(dt.upper()) || dt.lower()<=0)throw std::invalid_argument("RAW_DYNAMICS_ORDER_REFUSED");
 I rp=symmetric(I(acceleration)*square(dt)/2.),rv=symmetric(I(acceleration)*dt);
 for(int j=0;j<3;++j){if(!intersect(old.v[j],(next.p[j]-old.p[j]-rp)/dt) ||
    !intersect(next.p[j],old.p[j]+dt*old.v[j]+rp) ||
    !intersect(old.p[j],next.p[j]-dt*old.v[j]-rp) ||
    !intersect(next.v[j],old.v[j]+rv) || !intersect(old.v[j],next.v[j]-rv))return false;}
 return true;
}
bool rollingContractCoupledDynamics(RollingRawState& old,RollingRawState& next,double acceleration,I dt){
 if(next.epoch!=old.epoch+1 || !std::isfinite(acceleration) || acceleration<0 ||
    !std::isfinite(dt.lower()) || !std::isfinite(dt.upper()) || dt.lower()<=0)throw std::invalid_argument("COUPLED_DYNAMICS_DOMAIN");
 I half=dt/2.,remainder=symmetric(I(acceleration)*square(dt)/4.);
 // p1-p0-dt*(v0+v1)/2 = integral (dt/2-s)*a(s) ds.
 // Norm <= A*dt^2/4; these are coordinate necessary projections.
 for(int j=0;j<3;++j)if(!intersect(next.p[j],old.p[j]+half*(old.v[j]+next.v[j])+remainder) ||
  !intersect(old.p[j],next.p[j]-half*(old.v[j]+next.v[j])-remainder) ||
  !intersect(old.v[j],(next.p[j]-old.p[j]-remainder)/half-next.v[j]) ||
  !intersect(next.v[j],(next.p[j]-old.p[j]-remainder)/half-old.v[j]))return false;
 return true;
}
RollingVector rollingTagSphere(const RollingRawState& state,const Eigen::Vector3d& lever){
 auto tag=rollingTag(state,lever);I r=sqrt(square(I(lever.x()))+square(I(lever.y()))+square(I(lever.z())));
 for(int j=0;j<3;++j)if(!intersect(tag[j],state.p[j]+symmetric(r)))throw std::invalid_argument("LEVER_SPHERE_EMPTY");return tag;
}
bool rollingContractTagSphere(RollingRawState& state,const RollingVector& tag,const Eigen::Vector3d& lever){
 I r=sqrt(square(I(lever.x()))+square(I(lever.y()))+square(I(lever.z())));
 for(int j=0;j<3;++j)if(!intersect(state.p[j],tag[j]+symmetric(r)))return false;
 return rollingContractTag(state,tag,lever);
}
RollingRawState rollingProject(const RollingRawState& s){
 // Coordinate hull Cartesian product is an existential outer projection. No
 // covariance independence or division by an event radius is inferred.
 if(s.has_tag_relation){if(!s.tag_lever.allFinite())throw std::invalid_argument("TAG_LEVER_NONFINITE");for(auto x:s.tag_relation)check(x);}
 for(int j=0;j<3;++j){check(s.p[j]);check(s.v[j]);for(int k=0;k<3;++k)check(s.rotation[j][k]);}for(auto b:s.bias)check(b);return s;
}
}
