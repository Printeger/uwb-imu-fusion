#include "gaussian_sufficient_summary.hpp"
#include <iostream>
using namespace uwb_imu_pl::research;
void require(bool condition,const char* label){if(!condition)throw std::runtime_error(label);}
void close(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b,double tolerance,const char* label){require(a.rows()==b.rows() && a.cols()==b.cols() && (a-b).norm()<=tolerance*(1+a.norm()),label);}
int main(){
 int cases=0;
 // Regression of the old169-versus248 dimension loss, with exactly the
 // known273.201 realized parity energy. No detector field is overwritten.
 Eigen::MatrixXd regression_h=Eigen::MatrixXd::Zero(251,3);regression_h.topRows(3).setIdentity();Eigen::VectorXd regression_z=Eigen::VectorXd::Zero(251);regression_z(250)=std::sqrt(273.201);
 GaussianSufficientSummary dimension_regression(3);dimension_regression.append(regression_h,regression_z,{});
 auto regression=dimension_regression.evaluate(Eigen::Matrix3d::Identity(),1e-6,1e-9).reference;
 double old_threshold=boost::math::quantile(boost::math::complement(boost::math::chi_squared(169),1e-6));
 require(regression.dof==248 && regression.statistic>old_threshold && regression.statistic<regression.threshold,"epoch31 dimension regression");
 std::cout<<"DIMENSION_REGRESSION dof="<<regression.dof<<" statistic="<<regression.statistic<<" threshold="<<regression.threshold<<" old_threshold="<<old_threshold<<" alarm=0\n";
 for(bool optimized:{false,true})for(double correlation:{0.,.35,-.2})for(int removed:{-1,3,14})for(int dimension:{1,6}){
  const int m=24,n=6;Eigen::MatrixXd a(m,n),b=Eigen::MatrixXd::Zero(m,dimension);Eigen::VectorXd z(m);
  for(int i=0;i<m;++i){z(i)=.01*std::sin(.7*i);for(int j=0;j<n;++j)a(i,j)=std::sin(.43*(i+1)*(j+1))+(i==j?2.:0.);}
  for(int k=0;k<dimension;++k){b(2+k,k)=1.;b(8+k,k)=.7;b(17+k,k)=-.2;} // one physical latent response spans three blocks
  Eigen::MatrixXd sigma=Eigen::MatrixXd::Identity(m,m);sigma(7,8)=sigma(8,7)=correlation;
  std::vector<int> rows;for(int i=0;i<m;++i)if(i!=removed)rows.push_back(i);
  Eigen::MatrixXd ar(rows.size(),n),br(rows.size(),dimension);Eigen::VectorXd zr(rows.size());
  for(size_t i=0;i<rows.size();++i){ar.row(i)=a.row(rows[i]);br.row(i)=b.row(rows[i]);zr(i)=z(rows[i]);}
  Eigen::LLT<Eigen::MatrixXd> chol(principal(sigma,rows));Eigen::MatrixXd aw=chol.matrixL().solve(ar),bw=chol.matrixL().solve(br);Eigen::VectorXd zw=chol.matrixL().solve(zr);
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,n);c.rightCols(3).setIdentity();
  auto oracle=gaussianReference(ar,zr,principal(sigma,rows),c,{br},1e-6,1e-9);
  GaussianSufficientSummary summary(n);summary.optimized=optimized;summary.declareFault("physical-episode",dimension);
  for(int first=0;first<aw.rows();first+=7){int length=std::min(7,int(aw.rows())-first);summary.append(aw.middleRows(first,length),zw.segment(first,length),{{"physical-episode",bw.middleRows(first,length)}});}
  auto before=summary.evaluate(c,1e-6,1e-9);
  require(before.reference.status==oracle.status && before.reference.dof==oracle.dof && before.total_rank==n,"rank/dof/status");
  require(std::abs(before.reference.statistic-oracle.statistic)<1e-12 && std::abs(before.reference.threshold-oracle.threshold)<1e-12,"statistic/threshold");
  close(before.reference.mean,oracle.mean,1e-11,"mean");close(before.reference.covariance,oracle.covariance,1e-11,"covariance");close(before.reference.pl,oracle.pl,1e-10,"PL");
  auto pb=(oracle.parity*oracle.whitener*br).eval();auto pz=(oracle.parity*oracle.whitener*zr).eval();auto& fault=before.faults.at("physical-episode");
  close(fault.residual_gram,pb.transpose()*pb,1e-11,"fault Gram");close(fault.profile_rhs,pb.transpose()*pz,1e-11,"fault profile");close(fault.response,c*oracle.gain*oracle.whitener*br,1e-11,"protected fault response");
  close(fault.profile_estimate,pb.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(pz),1e-11,"fault profile estimate");
  summary.retire(3);auto after=summary.evaluate(c.rightCols(3),1e-6,1e-9);
  require(after.reference.dof==oracle.dof && after.total_rank==n,"retirement dof/rank");
  close(after.reference.mean,oracle.mean.tail(3),1e-11,"retired mean");close(after.reference.covariance,oracle.covariance.bottomRightCorner(3,3),1e-11,"retired covariance");close(after.reference.pl,oracle.pl,1e-10,"retired PL");
  close(after.faults.at("physical-episode").residual_gram,fault.residual_gram,1e-11,"retired fault Gram");++cases;
 }
 // Future-correlated observations: shared standard-normal source is an
 // explicit nuisance variable with exactly ONE independent prior.
 Eigen::MatrixXd h(5,2);h<<1,0,0,1,1,1,2,-1,-1,2;Eigen::VectorXd z(5);z<<.01,-.02,.03,.01,-.02;
 Eigen::VectorXd source(5);source<<0,.3,.5,-.2,0;Eigen::MatrixXd b=Eigen::MatrixXd::Zero(5,1);b(1)=1;b(2)=.8;
 Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,2);c.topRows(2).setIdentity();
 auto oracle=gaussianReference(h,z,Eigen::MatrixXd::Identity(5,5)+source*source.transpose(),c,{b},1e-6,1e-9);
 GaussianSufficientSummary shared(3);shared.declareFault("carry",1);
 Eigen::MatrixXd prior=Eigen::MatrixXd::Zero(1,3);prior(0,2)=1.;shared.append(prior,Eigen::VectorXd::Zero(1),{});
 for(int first:{0,2}){int length=first==0?2:3;Eigen::MatrixXd block(length,3);block.leftCols(2)=h.middleRows(first,length);block.col(2)=source.segment(first,length);shared.append(block,z.segment(first,length),{{"carry",b.middleRows(first,length)}});}
 Eigen::MatrixXd cs=Eigen::MatrixXd::Zero(3,3);cs.leftCols(2)=c;auto actual=shared.evaluate(cs,1e-6,1e-9);
 require(actual.reference.dof==oracle.dof,"shared-source dof");close(actual.reference.mean.head(2),oracle.mean,1e-11,"shared-source mean");close(actual.reference.covariance.topLeftCorner(2,2),oracle.covariance,1e-11,"shared-source covariance");close(actual.reference.pl,oracle.pl,1e-10,"shared-source PL");
 auto independent=gaussianReference(h,z,Eigen::MatrixXd::Identity(5,5),c,{b},1e-6,1e-9);require((independent.covariance-oracle.covariance).norm()>1e-4,"correlation counterexample");
 bool late_refused=false;try{shared.declareFault("invented-old-joint",2);}catch(const std::runtime_error&){late_refused=true;}require(late_refused,"late historical fault insertion must fail closed");
 GaussianSufficientSummary zero(2);zero.declareFault("removed-source",1);zero.append(h,z,{});
 require(zero.evaluate(c,1e-6,1e-9).faults.at("removed-source").classification=="EXACT_HARMLESS","literal removed fault zero");
 GaussianSufficientSummary alias_history(2);alias_history.declareFault("retired-alias",1,0);alias_history.append(h,z,{{"retired-alias",h.col(0)}});alias_history.retire(1);
 alias_history.append(Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Zero(1),{{"retired-alias",Eigen::MatrixXd::Constant(1,1,1e-11)}});
 require(alias_history.evaluate(c.rightCols(1),1e-6,1e-9).faults.at("retired-alias").classification=="INDETERMINATE","retired alias cannot cover new source support");
 // Structural witnesses survive the transform; tiny unidentified residuals
 // are refused rather than assigned zero risk.
 for(int relation:{1,2,0}){GaussianSufficientSummary s(2);s.declareFault("alias",1,relation==0?-1:0);s.append(h,z,{{"alias",relation==0?Eigen::MatrixXd(1e-11*b):Eigen::MatrixXd(h.col(0))}});
  auto protected_map=c;if(relation==1)protected_map.col(0).setZero();
  auto result=s.evaluate(protected_map,1e-6,1e-9);require(result.faults.at("alias").classification==(relation==1?"EXACT_HARMLESS":relation==2?"DANGEROUS_NULLSPACE":"INDETERMINATE"),"nullspace classification");}
 // Arbitrarily many mutually-exclusive old scalar faults still require
 // separate profiles. Independent coordinate impulses span N dimensions;
 // their N independent B_i'z values cannot be reconstructed from one energy.
 Eigen::Vector3d u(1,2,3),v(3,2,1);require(u.squaredNorm()==v.squaredNorm() && u(0)!=v(0),"profile information lower bound");
 // Predeclared pure linear Gaussian growth case; this80-step test is NOT an
 // extension of the production/replay50-epoch probability horizon.
 GaussianSufficientSummary chain(1);chain.append(Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Zero(1),{});
 Eigen::MatrixXd dense_h=Eigen::MatrixXd::Zero(161,81);dense_h(0,0)=1.;Eigen::VectorXd dense_z=Eigen::VectorXd::Zero(161);std::vector<Eigen::MatrixXd> dense_faults;
 for(int e=1;e<=80;++e){chain.declareFault("step"+std::to_string(e),1,-1,true);chain.addStateColumns(1);
  Eigen::MatrixXd a(2,2);a<<-1,1,0,1;Eigen::Vector2d rhs(.01*std::sin(e),.01*std::cos(e));Eigen::MatrixXd f(2,1);f<<0,1;
  chain.append(a,rhs,{{"step"+std::to_string(e),f}});chain.retire(1);
  dense_h(2*e-1,e-1)=-1.;dense_h(2*e-1,e)=dense_h(2*e,e)=1.;dense_z.segment<2>(2*e-1)=rhs;
  Eigen::MatrixXd column=Eigen::MatrixXd::Zero(161,1);column(2*e)=1.;dense_faults.push_back(column);
  if(e==32 || e==50 || e==80)std::cout<<"SUMMARY_GROWTH steps="<<e<<" states="<<chain.states()<<" state_rows="<<chain.h.rows()<<" faults="<<chain.faults.size()<<" numeric_bytes="<<chain.bytes()<<"\n";
 }
 Eigen::MatrixXd dense_c=Eigen::MatrixXd::Zero(3,81);dense_c(0,80)=1.;Eigen::MatrixXd compact_c=Eigen::MatrixXd::Zero(3,1);compact_c(0)=1.;
 auto dense_chain=gaussianReference(dense_h,dense_z,Eigen::MatrixXd::Identity(161,161),dense_c,dense_faults,1e-6,1e-9);
 auto compact_chain=chain.evaluate(compact_c,1e-6,1e-9);
 require(compact_chain.reference.dof==80 && compact_chain.reference.status==dense_chain.status,"growth dof/status");
 close(compact_chain.reference.mean,dense_chain.mean.tail(1),1e-10,"growth mean");close(compact_chain.reference.pl,dense_chain.pl,1e-10,"growth PL");
 std::cout<<"summary_parameter_cases="<<cases<<" shared_source=PASS nullspace=PASS residual_dof=PASS candidate_raw_deletion=PASS\n";
}
