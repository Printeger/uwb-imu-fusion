#include "rolling_raw.hpp"
#include "gaussian_sufficient_summary.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <Eigen/SVD>
#include <iostream>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
void check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
bool near(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b){return (a-b).norm()<1e-9*(1+b.norm());}
int main(){try{
 // One JOINT two-column leaf carries coexistence, including its off diagonal;
 // independently computed dense SVD projects all original white rows.
 Eigen::MatrixXd a(12,4),b(12,2);Eigen::VectorXd y(12);
 for(int i=0;i<12;++i){y(i)=.2*std::cos(i*.7);for(int j=0;j<4;++j)a(i,j)=std::sin((i+1)*(j+1)*.37)+(i==j?2:0);b(i,0)=i<6?1+.1*i:0;b(i,1)=i>2?std::cos(.2*i):0;}
 GaussianSufficientSummary summary(4);summary.declareFault("UWB+IMU",2);
 summary.append(a.topRows(5),y.head(5),{{"UWB+IMU",b.topRows(5)}});
 summary.append(a.bottomRows(7),y.tail(7),{{"UWB+IMU",b.bottomRows(7)}});
 summary.retire(2);
 Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,2);c(0,0)=1;c(1,1)=1;c(2,0)=.3;
 auto e=summary.evaluate(c,1e-6,1e-9);
 Eigen::JacobiSVD<Eigen::MatrixXd> svd(a,Eigen::ComputeFullU|Eigen::ComputeFullV);
 Eigen::MatrixXd gain=svd.matrixV()*svd.singularValues().cwiseInverse().asDiagonal()*svd.matrixU().leftCols(4).transpose();
 Eigen::MatrixXd parity=Eigen::MatrixXd::Identity(12,12)-a*gain;
 auto f=e.faults.at("UWB+IMU");
 check(e.reference.dof==8,"lost residual dof");
 check(std::abs(e.reference.statistic-(parity*y).squaredNorm())<1e-10,"residual energy");
 check(near(e.reference.mean,(gain*y).tail(2)),"retained state mean");
 check(near(e.reference.covariance,(gain*gain.transpose()).bottomRightCorner(2,2)),"retained covariance");
 check(near(f.residual_gram,b.transpose()*parity*b),"joint fault Gram/cross");
 check(near(f.profile_rhs,b.transpose()*parity*y),"joint fault rhs");
 check(near(f.response,c*gain.bottomRows(2)*b),"joint fault response");
 check(std::abs(f.residual_gram(0,1))>.1,"oracle cross fixture degenerate");
 GaussianSufficientSummary rank_deficient(2);rank_deficient.append(Eigen::MatrixXd::Zero(3,2),Eigen::VectorXd::Zero(3),{});
 check(rank_deficient.evaluate(Eigen::MatrixXd::Zero(3,2),1e-6,1e-9).reference.status=="INDETERMINATE","rank-deficient certificate admitted");
 // Shared ADC endpoint is a single retained latent coordinate, one prior.
 Eigen::MatrixXd shared(3,2);shared<<1,1,1,-1,0,1;Eigen::VectorXd z(3);z<<.2,-.1,0;
 GaussianSufficientSummary s(2);s.append(shared,z,{});Eigen::MatrixXd protect=Eigen::MatrixXd::Zero(3,2);protect(0,0)=1;
 auto se=s.evaluate(protect,1e-6,1e-9,false);
 check(near(se.reference.covariance,(shared.transpose()*shared).inverse()),"shared endpoint covariance");
 Eigen::MatrixXd wrongly_independent(4,3);wrongly_independent<<1,1,0,1,0,-1,0,1,0,0,0,1;
 auto wrong=(wrongly_independent.transpose()*wrongly_independent).inverse().eval();
 check(std::abs(wrong(0,0)-se.reference.covariance(0,0))>.1,"duplicate endpoint counterexample");
 using I=RollingInterval;NavigationState initial;initial.q_world_body=Eigen::Quaterniond::Identity();
 Eigen::Matrix<double,15,1> sigma=Eigen::Matrix<double,15,1>::Constant(.01);
 auto x=rollingInitial(initial,sigma,8,1e-10);
 auto propagated=rollingPropagate(x,.04,.2,.001,.0001,8);
 check(propagated.bias[0].upper()>x.bias[0].upper(),"RW forgotten");
 // Rational endpoint oracle, 100 digits, independently evaluate exact reach.
 using MP=boost::multiprecision::cpp_dec_float_100;
 for(int j=0;j<3;++j)for(int sign:{-1,1}){
  MP exact=MP(sign<0?x.p[j].lower():x.p[j].upper())+MP(1)/20*MP(sign<0?x.v[j].lower():x.v[j].upper())+MP(sign)*MP(.04)/800;
  check(MP(propagated.p[j].lower())<=exact && exact<=MP(propagated.p[j].upper()),"100-digit reach endpoint");
 }
 RollingRawState old=x;old.p={I(0),I(0),I(0)};old.v={I(-1,1),I(-1,1),I(-1,1)};
 auto next=rollingPropagate(old,0,.2,.001,.0001,8);
 RollingVector tag{I(.024,.026),I(0),I(0)};
 check(rollingContractTag(next,tag,Eigen::Vector3d::Zero()) && rollingContractDynamics(old,next,0),"tag/dynamics contraction");
 check(old.v[0].lower()>.47 && old.v[0].upper()<.53,"tag did not propagate to velocity");
 auto later=rollingPropagate(next,0,.2,.001,.0001,8);
 check(later.p[0].lower()>.047,"contraction not consumed by next boundary");
 auto speed=x;speed.p={I(0),I(0),I(0)};speed.v[0]=I(-.01,.01);
 for(int k=0;k<40;++k)speed=rollingPropagate(speed,0,0,0,0,8);
 check(speed.p[0].upper()>=.02,"small velocity forgotten counterexample");
 auto rotation=x;rotation.p={I(0),I(0),I(0)};for(auto& row:rotation.rotation)for(auto& entry:row)entry=I(-1,1);
 auto lever=Eigen::Vector3d(1,0,0);RollingVector zero{I(0),I(0),I(0)};
 check(rollingContractTag(rotation,zero,lever),"rotation tag inverse");
 check(rotation.rotation[0][0].lower()<=0 && rotation.rotation[0][0].upper()>=0,"rotation inverse dropped feasible rotation");
 auto uncertain=x;for(auto& row:uncertain.rotation)for(auto& entry:row)entry=I(-1,1);
 auto uncertain_tag=rollingTag(uncertain,lever);check(uncertain_tag[0].upper()-uncertain_tag[0].lower()>=2,"lever uncertainty reset");
 auto changed=x;changed.v[0]=I(-2,2);auto same_raw=rollingPropagate(changed,.04,.2,.001,.0001,8);
 check(same_raw.p[0].upper()>propagated.p[0].upper(),"parent ignored");
 UwbBatch batch;for(int k=0;k<8;++k){UwbMeasurement m;m.anchor_position_m=Eigen::Vector3d(k,0,0);batch.measurements.push_back(m);}
 std::array<RawRangeShell,8> shells;for(auto& shell:shells)shell={0,10};
 // Analytic retired relation: z_old=v_old+theta+noise=0,
 // |noise|<=.005. H0 gives[-.005,.005]; an ended episode with
 // theta in[-.035,.035] projects to[-.04,.04]. No live fault row remains.
 auto past_healthy=x,past_fault=x;past_healthy.p=past_fault.p={I(0),I(0),I(0)};
 past_healthy.v[0]=I(-.005,.005);past_fault.v[0]=I(-.04,.04);
 auto healthy_next=rollingPropagate(past_healthy,0,0,0,0,8),fault_next=rollingPropagate(past_fault,0,0,0,0,8);
 Eigen::Vector3d hl,hu,fl,fu;for(int j=0;j<3;++j){hl(j)=healthy_next.p[j].lower();hu(j)=healthy_next.p[j].upper();fl(j)=fault_next.p[j].lower();fu(j)=fault_next.p[j].upper();}
 auto healthy_pl=contractRawRangeBox(hl,hu,batch,shells,Eigen::Vector3d::Zero());
 auto fault_pl=contractRawRangeBox(fl,fu,batch,shells,Eigen::Vector3d::Zero());
 check(healthy_pl.valid && fault_pl.valid && fault_pl.pl.x()>healthy_pl.pl.x(),"retired fault projection forgotten with identical current raw");
 auto fallback=contractRawRangeBox(Eigen::Vector3d(-1,-1,-1),Eigen::Vector3d(1,1,1),batch,shells,Eigen::Vector3d::Zero(),{0,8,0,false});
 check(fallback.valid && fallback.considered==9 && fallback.frontier==9 && fallback.retained==9,"frontier resource dropped physical branch");
 std::cout<<"W2 PASS dense SVD residual/dof/state/joint2-column cross; shared endpoint; 100-digit reach; p/v feedback; rotation/lever; RW; parent dependency; frontier9 retained\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
