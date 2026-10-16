#include "joint_raw.hpp"
#include "physical_time.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
void require(bool v,const char* m){if(!v)throw std::runtime_error(m);}
bool inside(I x,double y){return x.lower()<=y && y<=x.upper();}
template<class F>void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected refusal");}
int main(){try{
 const int64_t origin=917654321000LL,finish=origin+60000000LL;
 const Eigen::Vector3d p0(.2,-.1,1),v0(.2,.1,-.2),a0(.1,-.2,.15),jerk(.3,.1,-.2);
 JointRawLaw law;law.acceleration=1.;law.rate=.4;law.jerk=1.;
 auto rotation=[](double t){return Eigen::AngleAxisd(.3*t,Eigen::Vector3d::UnitZ()).toRotationMatrix();};
 auto position=[&](double t){return Eigen::Vector3d(p0+t*v0+.5*t*t*a0+t*t*t/6*jerk);};
 auto velocity=[&](double t){return Eigen::Vector3d(v0+t*a0+.5*t*t*jerk);};
 for(int fault_case=0;fault_case<4;++fault_case){
  RollingRawState before;
  for(int j=0;j<3;++j){before.p[j]=I(p0[j]-.001,p0[j]+.001);before.v[j]=I(v0[j]-.001,v0[j]+.001);
   for(int k=0;k<3;++k)before.rotation[j][k]=I((j==k?1.:0.)-1e-8,(j==k?1.:0.)+1e-8);}
  for(auto& b:before.bias)b=I(-.02,.02);
  std::array<double,6> q;q.fill(7.);
  std::vector<TimestampNs> births;for(int64_t offset:{11000000LL,25000000LL,37000000LL,49000000LL})births.emplace_back(origin+offset);
  auto step=rawTimeStep(TimestampNs(origin),TimestampNs(finish),TimestampNs(origin),births,{q,q,q,q},.001,.0001);
  auto after=rollingPropagate(before,law.acceleration,law.rate,step);
  std::vector<JointTimedSample> samples;
  for(int index=0;index<6;++index){
   int64_t offset=index==0?0:index==5?49000000LL:births[index-1].value()-origin;
   double t=double(offset)/1e9;
   JointTimedSample s;s.raw.id=MeasurementId((uint64_t(1)<<40)+offset);
   s.raw.timestamp=TimestampNs(origin+offset);
   s.integration_end=TimestampNs(index==0?origin+5000000LL:index==5?finish:origin+offset);
   s.raw.specific_force_mps2=rotation(t).transpose()*(a0+t*jerk+Eigen::Vector3d(0,0,law.gravity));
   s.raw.angular_velocity_radps={0,0,.3};
   // Phase before/middle/after, carry-in, and sequential changing axes.
   if(fault_case==1 && offset>=25000000 && offset<49000000)s.raw.specific_force_mps2.x()+=20;
   if(fault_case==2 && offset<25000000)s.raw.angular_velocity_radps.y()+=1;
   if(fault_case==3){if(offset<25000000)s.raw.specific_force_mps2.z()+=10;else if(offset<49000000)s.raw.angular_velocity_radps.x()-=1;}
   for(int axis=0;axis<6;++axis){s.adc_radius[axis]=I(1e-6);s.rw_from_begin[axis]=step.bias_radius[axis];s.rw_to_end[axis]=step.bias_radius[axis];}
   samples.push_back(s);
  }
  for(size_t budget=0;budget<=7;++budget){
   auto a=before,b=after;auto result=jointContract(a,b,TimestampNs(origin),TimestampNs(finish),samples,law,budget);
   require(result.valid && !result.fallback && result.branches==42,"timed joint incomplete");
   for(int j=0;j<3;++j){require(inside(a.p[j],p0[j]) && inside(a.v[j],v0[j]),"initial truth removed");
    require(inside(b.p[j],position(.06)[j]) && inside(b.v[j],velocity(.06)[j]),"timed integral lost true trajectory");
    for(int k=0;k<3;++k)require(inside(b.rotation[j][k],rotation(.06)(j,k)),"SO3 truth removed");
    require(b.p[j].lower()>=after.p[j].lower() && b.p[j].upper()<=after.p[j].upper(),"operator widened input");}
   for(int j=0;j<6;++j)require(inside(a.bias[j],0) && inside(b.bias[j],0),"bias truth removed");
   if(budget<7)require(result.frontier>0,"unprocessed branch discarded");
  }
  auto invalid=samples;invalid.back().raw.specific_force_mps2.x()+=1;
  refuses([&]{auto a=before,b=after;jointContract(a,b,TimestampNs(origin),TimestampNs(finish),invalid,law);});
 }
 std::cout<<"SC9 joint PASS nonuniform jerk/rotation, carry-in and mid-interval accel/gyro phases, held raw identity, all8 budgets, source tamper; full per-sample fault union\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
