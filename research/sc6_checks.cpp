#include "sc6_service.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iostream>
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
 std::cout<<"SC6 W1 CHECKS PASS analytic interval oracle, frozen domains, certified new motion and future/GT-free shared step\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
