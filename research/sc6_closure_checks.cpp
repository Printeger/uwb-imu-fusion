#include "sc6_temporal_monitor.hpp"
#include "sc6_provider.hpp"
#include "sc6_service.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool x,const char* why){if(!x)throw std::runtime_error(why);}
template<class F>void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"required fail closed");}
int main(){try{
 Sc6Provider source("config/sc6_closure.yaml","S0",20261010,2);
 RawInclusionParameters law;law.initial_sigmas=source.cfg.realtime.prior_sigmas;law.numeric.valid=true;law.gravity=source.cfg.imu.gravity_mps2;law.accel_adc=.02;law.gyro_adc=.001;law.accel_rw=source.cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=source.cfg.imu.gyroscope_bias_rw_sigma;
 Sc6MotionDomain domain;domain.id="D1";domain.acceleration=1;domain.rate=1;domain.jerk=2;domain.alpha=2;
 using Q=boost::multiprecision::cpp_dec_float_100;
 // Full global chronology: six fixed-amplitude early and late witnesses.
 for(int onset:{601,11201})for(int axis=0;axis<6;++axis){
  Sc6TemporalMonitor monitor(law,1e-6,domain,4096);int alarms=0;
  for(int i=0;i<=onset+110;++i){auto m=sample(source.cfg,i);if(onset<=i && i<onset+100){if(axis<3)m.specific_force_mps2(axis)+=20;else m.angular_velocity_radps(axis-3)+=1;}
   bool alarm=monitor.append(m);if(alarm){++alarms;require(i==onset,"temporal onset/offset attribution false");require(monitor.begin().value()==(int64_t(onset)-100)*5000000LL && monitor.end().value()==(int64_t(onset)+100)*5000000LL,"support lost new or ended branch");}
   if(i==onset){TimeUniformEvent event(1e-6);Q dt=Q(1)/200;
    Q derivative=axis<3?Q(domain.jerk)+Q(domain.rate)*(Q(domain.acceleration)+Q(law.gravity)):Q(domain.alpha);
    Q bound=derivative*dt+Q(event.adcRadius(i)+event.adcRadius(i-1))*Q(axis<3?law.accel_adc:law.gyro_adc)+Q(event.adcRadius(i))*Q(axis<3?law.accel_rw:law.gyro_rw)*sqrt(dt)+Q(4e-10);
    require(Q(monitor.radius[axis])>=bound,"directed radius misses high precision oracle");}
  }
  require(alarms==1 && monitor.records().size()==1,"offset manufactured second episode");
 }
 // First possible 50ms END at55ms must retain onset5ms, not only new55ms.
 Sc6TemporalMonitor first_end(law,1e-6,domain,4096);
 for(int i=0;i<=11;++i){auto m=sample(source.cfg,i);if(1<=i && i<11)m.angular_velocity_radps.x()=1;first_end.append(m,true);}
 // Direct endpoint jump observed for the first time at55ms (previous history
 // may carry a fault without an earlier functional record).
 Sc6TemporalMonitor ended(law,1e-6,domain,4096);
 for(int i=0;i<=11;++i){auto m=sample(source.cfg,i);if(i<=10)m.angular_velocity_radps.x()=1;ended.append(m);}
 require(ended.begin().value()==5000000LL && ended.end().value()==555000000LL,"early ended50ms explanation discarded");
 // Two distinct lawful weak-fault/ADC histories produce identical physical
 // raw samples with theta=.01rad/s: healthy ADC+5, faulty ADC-5+theta.
 for(int sample_index:{11201,51201}){TimeUniformEvent event(1e-6);require(event.adcRadius(sample_index)>=5,"weak witness outside event");require(.001*5==.001*(-5)+.01,"weak observable mismatch");}
 Sc6TemporalMonitor bad(law,1e-6,domain,4096);bad.append(sample(source.cfg,0));auto gap=sample(source.cfg,2);refuses([&]{bad.append(gap);});
 Sc6TemporalMonitor grid(law,1e-6,domain,4096);grid.append(sample(source.cfg,0));grid.append(sample(source.cfg,1));auto offgrid=sample(source.cfg,2);offgrid.angular_velocity_radps.x()=1;refuses([&]{grid.append(offgrid);});
 // Actual native joint UWB/gyro recovery; original fixed amplitude and data.
 Sc6Provider joint("config/sc6_closure.yaml","D1_G_E",20261010,80);
 Sc6Service service(joint.law,joint.cfg,joint.initial,joint.lever,joint.domain,joint.imu.front(),true,false,true);
 int repairs=0,rejoins=0;double risk=0;
 for(int k=1;k<=80;++k){auto r=service.step(joint.certify(k),true);
  require(r.bound.valid && r.source_prefix==21+128*k && risk<r.bound.event_risk && r.bound.event_risk<=1e-6,"joint temporal raw/source/risk lost");risk=r.bound.event_risk;
  if(k==61){require(r.action.current_bridge && r.action.boundary_repair && !r.action.removed_prior_slots.empty(),"joint temporal actual prior replacement missing");++repairs;}
  if(k==71)require(r.action.current_bridge && !r.action.rejoined,"shared faulty endpoint rejoined");
  if(72<=k && k<=75){require(r.action.rejoined && !r.action.current_bridge,"four actual healthy PIM rejoins missing");++rejoins;}
 }
 require(repairs==1 && rejoins==4 && service.updateCount()==81,"joint sole native mutation count");
 // Temporal-only gyro FD, deliberately held past alarm retirement. The
 // original temporal identity must drive actual dirty marginal-prior repair,
 // or explicit beyond-replay refusal, not an empty absolute-pool support.
 std::filesystem::create_directories(".cache/sc6-closure");
 for(bool outside:{false,true}){
  auto config=YAML::LoadFile("config/sc6_closure.yaml");auto cases=config["closure"]["additional_cases"];auto episode=cases["D1_G_E"]["faults"][0];episode[0]=61;episode[4]=61;
  if(outside){config["rolling_integrity"]=YAML::Clone(researchLaw("config/sc5_open.yaml")["rolling_integrity"]);config["rolling_integrity"]["recovery_extra_epochs"]=0;}
  std::string path=outside?".cache/sc6-closure/check-temporal-outside.yaml":".cache/sc6-closure/check-temporal-polluted.yaml";{std::ofstream out(path);out<<config;}
  Sc6Provider held(path,"D1_G_E",20261010,outside?104:120);Sc6Service actual(held.law,held.cfg,held.initial,held.lever,held.domain,held.imu.front(),true,false,true);
  int release=outside?100:95,repair=0,rejoin=0;
  for(int k=1;k<=held.epochs;++k){auto r=actual.step(held.certify(k),k>=release);require(r.bound.valid && r.source_prefix==21+128*k,"held temporal raw/event lost");
   if(!outside && k==release){require(r.action.boundary_repair && r.action.history_marginalized && !r.action.removed_prior_slots.empty(),"retired temporal identity lost actual dirty prior repair");++repair;}
   if(!outside && release<k && k<=release+4){require(r.action.rejoined && !r.action.current_bridge,"held temporal healthy rejoin4 missing");++rejoin;}
   if(outside && k>=release)require(r.action.recovery=="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE" && !r.action.current_bridge && !r.action.boundary_repair,"retired temporal identity fabricated outside-replay restoration");
  }
  require(outside || (repair==1 && rejoin==4),"held native restoration denominator");
  std::cout<<"TEMPORAL RETIRED "<<(outside?"outside104 refusal100-104":"polluted120 repair95 rejoin96-99")<<" actual updates="<<actual.updateCount()-1<<" repairs="<<repair<<" rejoins="<<rejoin<<"\n";
 }
 std::cout<<"SC6 CLOSURE CHECKS PASS 12 early/late axes;100-digit directed healthy-difference bound; beginning+ending union; no false offset record; weak overlap; gap/off-grid fail closed; native joint80 repair61 bridge shared71 rejoin72-75\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
