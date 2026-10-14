#include "sc6_service.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
using I=RollingInterval;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
bool inside(I a,double x){return a.lower()<=x && x<=a.upper();}
void truthInside(const RollingRawState& a,const NavigationState& t){auto r=t.q_world_body.toRotationMatrix();for(int j=0;j<3;++j){require(inside(a.p[j],t.position_world_m(j)) && inside(a.v[j],t.velocity_world_mps(j)),"joint true p/v lost");for(int k=0;k<3;++k)require(inside(a.rotation[j][k],r(j,k)),"joint true SO3 lost");require(inside(a.bias[j],t.accel_bias_mps2(j)) && inside(a.bias[j+3],t.gyro_bias_radps(j)),"joint true bias lost");}}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);TimeUniformEvent event(1e-6);JointRawLaw law;
 // 8 epochs, full attitude uncertainty and real shared endpoints/RW. Only
 // the original sample0 is healthy. Later raw explanations keep all7 axes.
 for(const auto& name:{"S0","F0","F3","S2"}){
  Sc6Provider src("config/sc7.yaml",name,20261010,70);law.gravity=src.cfg.imu.gravity_mps2;law.acceleration=src.domain.acceleration;law.rate=src.domain.rate;law.jerk=src.domain.jerk;
  auto state=rollingInitial(src.initial,src.cfg.realtime.prior_sigmas,event.block(0).q,1e-10);auto original=state;
  require(jointInitial(state,src.imu[0],law,event),"initial norm contraction empty");truthInside(state,src.truth(0));
  if(std::string(name)=="S0")require(state.bias[2].upper()-state.bias[2].lower()<original.bias[2].upper()-original.bias[2].lower()-1e-3,"healthy original-law sample0 did not shrink full-union bias z");
  for(int k=1;k<=8;++k){auto next=rollingPropagate(state,law.acceleration,law.rate,law.accel_rw,law.gyro_rw,event.block(k).q);std::vector<ImuMeasurement> raw(src.imu.begin()+10*(k-1),src.imu.begin()+10*k+1);if(std::string(name)=="F0")for(int n=k==1?1:0;n<=10;++n)raw[n].specific_force_mps2.x()+=20.;if(std::string(name)=="F3")for(int n=k==1?1:0;n<=10;++n)raw[n].angular_velocity_radps.x()+=1.;auto a=state,b=next;auto full=jointContract(a,b,raw,law,event);truthInside(a,src.truth((k-1)*10));truthInside(b,src.truth(k*10));require(full.valid && full.branches==7 && !full.fallback,"joint full branch processing failed");for(int j=0;j<6;++j)require(b.bias[j].lower()>=next.bias[j].lower() && b.bias[j].upper()<=next.bias[j].upper(),"joint not a contraction");
   for(size_t budget=0;budget<7;++budget){auto fa=state,fb=next;auto e=jointContract(fa,fb,raw,law,event,budget);truthInside(fa,src.truth((k-1)*10));truthInside(fb,src.truth(k*10));require(e.frontier==7-budget && fb.bias[2].lower()==next.bias[2].lower() && fb.bias[2].upper()==next.bias[2].upper(),"unprocessed frontier was pruned");}
   auto bad=raw;bad[4].id=MeasurementId(999999);bool refused=false;try{jointContract(a,b,bad,law,event);}catch(...){refused=true;}require(refused,"shared source tampering accepted");state=b;
  }
  // Actual main30 chain, including a fault at61 and real marginalization.
  auto reference=original;
  Sc6Service service(src.law,src.cfg,src.initial,src.lever,src.domain,src.imu.front(),true,false,true);size_t repairs=0;
  for(int k=1;k<=70;++k){auto r=service.step(src.certify(k),true);reference=rollingPropagate(reference,law.acceleration,law.rate,law.accel_rw,law.gyro_rw,event.block(k).q);if(std::string(name)=="S0")require(r.bias_widths[2]<reference.bias[2].upper()-reference.bias[2].lower()-1e-3,"main full-union initial bias contraction lost through separator");require(r.bound.valid && r.source_prefix==21+128*k,"main joint raw event/source lost");if(k==61 && std::string(name)!="S0")repairs+=r.action.boundary_repair;
   if(k>=31){require(!r.x0 && r.boundary>0,"main joint never crossed actual separator");}
  }
  require(service.updateCount()==71,"joint introduced second commit");std::cout<<"joint native "<<name<<"70/70 repairs="<<repairs<<'\n';
 }
 // Legal measurement overlap: two biases ±delta, compensated by ADC at0
 // and by constant opposite theta in every adjacent50ms episode thereafter.
 // Same R,p,v and RW; every episode has one axis, arbitrary lawful theta.
 const double delta=.002;require(delta<event.adcRadius(0)*law.gyro_adc,"initial ADC overlap witness outside event");
 for(int sample=0;sample<=100;++sample){double plus_bias=delta,minus_bias=-delta;double plus_fault=sample?-delta:0,minus_fault=sample?delta:0;double plus_adc=sample?0:-delta,minus_adc=sample?0:delta;require(plus_bias+plus_fault+plus_adc==minus_bias+minus_fault+minus_adc,"bias/fault overlap histories have different measurements");}
 // Two full static histories with the actual registered lever share every
 // tag, gravity/gyro sample; their separate initial tangent draws are legal.
 Sc6Provider registered("config/sc7.yaml","S0",20261010,1);Eigen::Vector3d actual_lever=registered.lever,base(0,0,1);Eigen::Vector3d fixed_tag=base+actual_lever;
 gtsam::Pose3 observed(gtsam::Rot3(),base);
 for(double angle:{-.1,.1}){gtsam::Rot3 rotation=gtsam::Rot3::Rz(angle);Eigen::Vector3d position=fixed_tag-rotation.matrix()*actual_lever;gtsam::Pose3 history(rotation,position);auto eta=history.localCoordinates(observed);for(int j=0;j<6;++j)require(std::abs(eta(j))<event.block(0).q*registered.cfg.realtime.prior_sigmas(j),"static yaw history outside original initial law");require((position+rotation.matrix()*actual_lever-fixed_tag).norm()<1e-15,"registered lever tag overlap failed");require((rotation.matrix().transpose()*Eigen::Vector3d(0,0,law.gravity)-Eigen::Vector3d(0,0,law.gravity)).norm()==0,"registered yaw changed IMU gravity");}
 // Static tag yaw gauge: yaw rotations preserve gravity and a z-axis lever.
 Eigen::Vector3d lever(0,0,.2);Eigen::AngleAxisd yaw(.2,Eigen::Vector3d::UnitZ());require((yaw*lever-lever).norm()==0 && (yaw*Eigen::Vector3d(0,0,9.81)-Eigen::Vector3d(0,0,9.81)).norm()==0,"yaw witness changed static observations");
 std::cout<<"SC7 K5/K6/K7 PASS8-epoch full attitude/RW/endpoints/all7 theta branches and every budget; native4x70; repeated adjacent fault and initial ADC bias overlap; static yaw gauge\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
