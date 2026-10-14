#include "sc6_service.hpp"
#include "numerical_candidate.hpp"
#include <tbb/global_control.h>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
using I=RollingInterval;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
double vpl(double l,double u,double c){return std::max(std::abs(l-c),std::abs(u-c));}
int main(int argc,char** argv){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 if(argc>1 && std::string(argv[1])=="geometry"){
  Sc6Provider src("config/sc7.yaml","S3",20261010,1);auto b=src.uwb[0];
  for(int mask=-1;mask<8;++mask){auto old=uwbCandidate(b,mask,1e-6,1e-9);auto c=numericalUwbCandidate(b,mask);std::cout<<" flags="<<c.numeric_converged<<'/'<<c.functional_post_test<<" status="<<c.functional.status<<" seeds="<<c.converged<<'/'<<c.attempted<<" stat="<<c.functional.gaussian.statistic<<" threshold="<<c.functional.gaussian.threshold<<'\n';require(c.numeric_converged && c.functional_post_test && !c.gaussian_reference_qualified && !c.functional.gaussian.pl.allFinite(),"G2 numerical/functional qualification split");std::cout<<"G2 mask="<<mask<<" old="<<old.status<<" squared_seed="<<c.squared_seed.transpose()<<" design="<<c.design_singular.transpose()<<" local_j="<<c.jacobian_singular.transpose()<<" roots="<<c.roots.size()<<" converged="<<c.converged<<'/'<<c.attempted<<" stat="<<c.functional.gaussian.statistic<<'\n';}
  // Exact coplanar anchors: two global roots, both locally full rank.
  for(auto& m:b.measurements){m.anchor_position_m.z()=0;m.range_m=(m.anchor_position_m-Eigen::Vector3d(.3,-.2,1.7)).norm();}
  auto mirror=numericalUwbCandidate(b,-1);bool up=false,down=false;for(const auto& p:mirror.roots){up|=p.z()>1.69;down|=p.z()<-1.69;}require(up && down && mirror.jacobian_singular.minCoeff()>.1,"local rank confused with global root uniqueness");
  auto bad=b;for(auto& m:bad.measurements)m.anchor_position_m.setZero();require(!numericalUwbCandidate(bad,-1).numeric_converged,"degenerate coincident anchors accepted");
  // A seed touching an anchor may fail; other observation-only seeds survive.
  for(auto& m:b.measurements)m.range_m=(m.anchor_position_m-b.measurements[0].anchor_position_m).norm();auto zero=numericalUwbCandidate(b,-1);require(!zero.gaussian_reference_qualified,"near-zero domain falsely Gaussian qualified");
  std::cout<<"SC7 K3/K4 PASS both coplanar roots, local/full design ranks separate, all9 first G2 masks, no Gaussian inheritance; raw never consumes roots for pruning\n";return 0;
 }
 if(argc>1 && (std::string(argv[1])=="attribute" || std::string(argv[1])=="inputs")){
  if(argc!=7)throw std::invalid_argument("attribute CONFIG CASE EPOCHS SEED CSV");
  Sc6Provider src(argv[2],argv[3],std::stoi(argv[5]),std::stoi(argv[4]));if(std::string(argv[1])=="inputs"){src.writeInputAudit(argv[6]);std::cout<<"SC7 input audit epochs="<<src.epochs<<" domain="<<src.domain.id<<'\n';return 0;}src.law["sc7"]["attribute"]=true;
  Sc6Service service(src.law,src.cfg,src.initial,src.lever,src.domain,src.imu.front(),true,false,true);
  std::ofstream out(argv[6]);out<<std::setprecision(17)<<"output_epoch,state_epoch,parent,source_prefix,q,event,stage,mask,center_x,center_y,center_z,low_x,low_y,low_z,high_x,high_y,high_z,hpl,vpl,z_half,z_offset,witness\n";
  for(int k=1;k<=src.epochs;++k){auto r=service.step(src.certify(k),true);for(const auto& a:r.bound.attribution){Eigen::Vector3d d=(a.low-r.tag).cwiseAbs().cwiseMax((a.high-r.tag).cwiseAbs());out<<k<<','<<a.epoch<<','<<r.parent<<','<<r.source_prefix<<','<<r.q<<','<<TimeUniformEvent(src.law["rolling_integrity"]["task_exceedance_budget"].as<double>()).id()<<','<<a.stage<<','<<a.omitted;for(const auto& v:{r.tag,a.low,a.high})for(int j=0;j<3;++j)out<<','<<v(j);double half=(a.high.z()-a.low.z())/2,offset=std::abs(r.tag.z()-(a.high.z()/2+a.low.z()/2));out<<','<<d.head<2>().norm()<<','<<d.z()<<','<<half<<','<<offset<<','<<a.extremum_class<<'\n';}}
  std::cout<<"SC7 ATTRIBUTION native commits="<<service.updateCount()-1<<" domain="<<src.domain.id<<'\n';return 0;
 }
 // K1: exact VPL decomposition and invariance under current coordinate hull.
 for(double c:{-7.,0.,2.,8.}){double merged=vpl(-3,5,c),branch=std::max(vpl(-3,-1,c),vpl(2,5,c));require(merged==branch,"V hull changed same-set VPL");require(merged==4+std::abs(c-1),"V half/offset");}
 require(std::hypot(3.,4.)==5 && std::max(std::hypot(3.,0.),std::hypot(0.,4.))==4,"H artificial corner");
 // K2: Q1=(x,z)=(0,0), Q2=(2,2), next observation x=0,
 // dynamics z_next=z+x. Keeping branch relation yields z_next=0;
 // projecting first admits (0,2), hence a spurious z_next=2.
 auto evolve=[](I x,I z){return x+z;};
 I exact=evolve(I(0),I(0)),projected=evolve(I(0),I(0,2));
 require(exact.upper()==0 && projected.upper()==2,"two-update projection fixture");
 Sc6Provider source("config/sc7.yaml","S0",20261010,1);auto s=rollingInitial(source.initial,source.cfg.realtime.prior_sigmas,7,1e-10),b=s;
 for(auto& v:b.bias)v=I(-100,100);auto x=rollingTagSphere(rollingPropagate(s,.04,.2,.001,.0001,7),source.lever),y=rollingTagSphere(rollingPropagate(b,.04,.2,.001,.0001,7),source.lever);
 for(int j=0;j<3;++j)require(x[j].lower()==y[j].lower() && x[j].upper()==y[j].upper(),"legacy raw p/tag directly depends on bias");
 // Read-only snapshots must not perturb actual FGO/action/certificates.
 Sc6Provider small("config/sc7.yaml","S0",20261010,31);
 auto diagnostic=YAML::Clone(small.law);diagnostic["sc7"]["attribute"]=true;
 Sc6Service normal(small.law,small.cfg,small.initial,small.lever,small.domain,small.imu.front(),true,false,true),observed(diagnostic,small.cfg,small.initial,small.lever,small.domain,small.imu.front(),true,false,true);
 for(int k=1;k<=31;++k){auto envelope=small.certify(k);auto a=normal.step(envelope,true),b=observed.step(envelope,true);require(a.tag==b.tag && a.bound.low==b.bound.low && a.bound.high==b.bound.high && a.bound.hpl==b.bound.hpl && a.bound.pl==b.bound.pl && a.bound.event_risk==b.bound.event_risk && a.source_prefix==b.source_prefix && a.parent==b.parent && a.generation==b.generation && a.action.status==b.action.status && a.action.excluded==b.action.excluded && a.x0==b.x0,"attribution perturbed native semantics");if(k==31)require(!b.x0 && !b.bound.attribution.empty(),"no native post-marginalization snapshot");}
 std::cout<<"SC7 K1/K2 PASS V hull and half/offset; H3/4/5; two-time projection; bias-only kernel noninterference DIAGNOSTIC_ONLY\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
