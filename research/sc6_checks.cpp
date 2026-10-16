#include "sc6_service.hpp"
#include "sc6_provider.hpp"
#include "sc6_bias_prototype.hpp"
#include "sc6_delivery.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool x,const char* reason){if(!x)throw std::runtime_error(reason);}
int main(){try{
 using Q=boost::multiprecision::cpp_dec_float_100;
 // Independent high precision oracle for scalar analytic trajectory values.
 for(int kind=0;kind<4;++kind){Sc6MotionRecipe recipe;recipe.kind=kind;auto domain=sc6Domain(recipe);require(domain.acceleration==(kind?1:.04),"domain mismatch");
  for(int i:{0,1,399,799,800,801,1199,1200,1999,2000,2399,2400,100000}){auto m=sc6MotionAt(i,recipe);Q t=Q(i)/200;for(int j=0;j<3;++j){Q expected;if(kind==1){int phase=i%2400,travel=phase>=1200?phase-1200:phase;Q u=Q(std::min(travel,800))/800;Q q=10*pow(u,3)-15*pow(u,4)+6*pow(u,5);expected=Q(j==0?.8:j==1?.4:.2)*(phase>=1200?1-q:q);}else{Q a(kind==0?recipe.amp(j):j==0?.8:j==1?.6:.4),f(kind==0?recipe.freq(j):j==0?.6:j==1?.5:.4);expected=a*sin(f*t);}if(j==2)expected+=1;require(Q(m.p[j].lower())<=expected && expected<=Q(m.p[j].upper()),"motion interval misses independent oracle");}}
 }
 NoisyMotionScenario old("config/sc5_open.yaml","OE_MIXED",20261010,65);Sc6Provider newer("config/sc6.yaml","B1",20261010,65);double source_delta=0;for(int i=0;i<=650;++i){source_delta=std::max(source_delta,(old.imu[i].specific_force_mps2-newer.imu[i].specific_force_mps2).cwiseAbs().maxCoeff());source_delta=std::max(source_delta,(old.imu[i].angular_velocity_radps-newer.imu[i].angular_velocity_radps).cwiseAbs().maxCoeff());}for(int k=0;k<65;++k)for(int a=0;a<8;++a)source_delta=std::max(source_delta,std::abs(old.uwb[k].measurements[a].range_m-newer.uwb[k].measurements[a].range_m));double init_delta=(old.initial_observation.position_world_m-newer.initial.position_world_m).norm()+(old.initial_observation.velocity_world_mps-newer.initial.velocity_world_mps).norm()+(old.initial_observation.q_world_body.coeffs()-newer.initial.q_world_body.coeffs()).norm();double cert_delta=0;for(int e=1;e<=65;++e){auto env=newer.certify(e);for(int a=0;a<8;++a)cert_delta=std::max(cert_delta,std::abs(old.range_certificates[e-1].rho_num[a]-env.range.rho_num[a]));}std::cout<<"INITIAL/CERT DELTA "<<init_delta<<"/"<<cert_delta<<"\n";std::cout<<"LEGACY RAW DELTA "<<source_delta<<"\n";require(source_delta<=1e-13,"legacy raw drift");
 Sc6Provider input("config/sc6.yaml","S1",20261010,3);auto envelope=input.certify(1);require(envelope.source_valid,"new source invalid");
 Sc6Service a(input.law,input.cfg,input.initial,input.lever,input.domain,input.imu[0]),b(input.law,input.cfg,input.initial,input.lever,input.domain,input.imu[0]);
 auto first=a.step(envelope,true);input.imu.back().specific_force_mps2.setConstant(999);input.uwb.back().measurements[0].range_m=999;auto second=b.step(envelope,true);
 require(first.state.position_world_m==second.state.position_world_m && first.state.accel_bias_mps2==second.state.accel_bias_mps2 && first.bound.hpl==second.bound.hpl && first.action.excluded==second.action.excluded,"future noninterference failed");
 Sc6Delivery sink;require(!sink.accept(1,0,2,true,1),"bad stamp delivered");require(!sink.accept(1,50000000,2,true,50.1),"late delivered");require(!sink.currentProtected(),"late protection retained");require(sink.accept(2,100000000,3,true,1),"source continuity lost after timeout");require(!sink.accept(2,100000000,3,true,1),"duplicate receipt delivered");sink.sourceGap();require(!sink.accept(3,150000000,4,true,1),"gap delivered stale protection");
 using I=RollingInterval;RollingRawState prev,next;next.epoch=1;for(int j=0;j<3;++j){prev.p[j]=I(0);prev.v[j]=I(0);next.p[j]=I(-.00125,.00125);next.v[j]=I(0);}require(rollingContractCoupledDynamics(prev,next,1),"coupled positive empty");Q extremum=Q(1)/1600;require(Q(next.p[0].lower())<=-extremum && extremum<=Q(next.p[0].upper()) && next.p[0].upper()<.00125,"bang-bang extremum or contraction lost");
 auto healthy=sc6BiasConstraint(I(0),I(.5),I(2),1,I(3),I(-.2,.2),I(-10,10),I(-.01,.01),I(0));require(healthy.feasible && healthy.bias.lower()<=1.8 && healthy.bias.upper()>=2.2 && healthy.bias.upper()-healthy.bias.lower()<.41,"bias informative prototype");
 auto ambiguous=sc6BiasConstraint(I(0),I(.5),I(2),1,I(3),I(0),I(-10,10),I(0),I(-INFINITY,INFINITY));require(ambiguous.feasible && ambiguous.bias.lower()<=1 && ambiguous.bias.upper()>=2,"fault-bias cancellation dropped solution");
 // y=3, a=1: (b=1,f=1) and (b=2,f=0) both exact lawful solutions.
 auto positive1=sc6BiasConstraint(I(0),I(.5),I(2),1,I(3),I(0),I(-10,10),I(0),I(1));require(positive1.bias.lower()<=1 && positive1.bias.upper()>=1,"shared-fault prototype");
 RollingRawState rotations;for(int j=0;j<3;++j){rotations.p[j]=I(0);for(int h=0;h<3;++h)rotations.rotation[j][h]=I(-1,1);}auto sphere=rollingTagSphere(rotations,Eigen::Vector3d(.2,-.1,.08));Q lever_norm=sqrt(Q(.2)*Q(.2)+Q(.1)*Q(.1)+Q(.08)*Q(.08));require(Q(sphere[0].lower())<=-lever_norm && lever_norm<=Q(sphere[0].upper()) && sphere[0].upper()<.38,"SO3 norm sphere misses extrema");
 std::array<RawRangeShell,8> shells;for(auto& shell:shells)shell={0,20};RawRangeBudget scarce;scarce.max_boxes=0;scarce.tight_shell=true;auto frontier=contractRawRangeBox(Eigen::Vector3d(-10,-10,-10),Eigen::Vector3d(10,10,10),input.uwb[0],shells,Eigen::Vector3d::Zero(),scarce);require(frontier.valid && frontier.frontier==9 && frontier.low.x()==-10 && frontier.high.x()==10,"budget frontier lost");
 // Real polluted-prior restoration in the SAME frozen SC6 shared service.
 // Operator-held FDE is a test control, not an automatic detection latency.
 std::filesystem::create_directories(".cache/sc6-20261014");
 for(bool outside:{false,true}){
  auto config=YAML::LoadFile("config/sc6.yaml");config["sc6"]["cases"]["F0"]["faults"][0][0]=outside?61:20;config["sc6"]["cases"]["F0"]["faults"][0][4]=outside?61:20;
  if(outside){auto root=researchLaw("config/sc5_open.yaml");config["rolling_integrity"]=YAML::Clone(root["rolling_integrity"]);config["rolling_integrity"]["recovery_extra_epochs"]=0;}
  const std::string path=outside?".cache/sc6-20261014/check-outside.yaml":".cache/sc6-20261014/check-polluted.yaml";{std::ofstream output(path);output<<config;}
  Sc6Provider delayed(path,"F0",20261010,outside?104:80);Sc6Service shared(delayed.law,delayed.cfg,delayed.initial,delayed.lever,delayed.domain,delayed.imu[0],true,false,true);
  const int release=outside?100:55;int repairs=0,rejoined=0;
  for(int k=1;k<=delayed.epochs;++k){auto env=delayed.certify(k);auto r=shared.step(env,k>=release);require(r.bound.valid && r.source_prefix==21+128*k,"delayed raw/event lost");
   if(k==(outside?61:20))require(r.action.uwb_alarm && r.action.uwb_alarm_time!=std::chrono::steady_clock::time_point{} && r.alarm_time<=r.selected_time,"actual UWB alarm clock missing");
   if(k==release && !outside){require(r.action.boundary_repair && r.action.history_marginalized && r.action.removed_prior_slots.size()>0 && !r.x0,"polluted marginal prior not actually repaired");++repairs;}
   if(!outside && k>release && k<=release+4){require(r.action.rejoined && !r.action.current_bridge,"real original IMU did not rejoin4");++rejoined;}
   if(outside && k>=release)require(r.action.recovery=="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE" && !r.action.current_bridge && !r.action.boundary_repair,"outside replay fake restoration");
  }
  std::cout<<"SC6 DELAYED "<<(outside?"outside104 safe refusal100-104":"polluted80 repair55 rejoin56-59")<<" updates="<<shared.updateCount()-1<<" repairs="<<repairs<<" rejoins="<<rejoined<<"\n";
 }
 std::cout<<"SC6 CHECKS PASS coupled bang-bang oracle, SO3 sphere, bias positive/cancellation, all-mask frontier, analytic interval oracle, frozen domains, certified new motion and future/GT-free shared step\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
