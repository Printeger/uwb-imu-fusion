#pragma once
#include "gaussian_reference.hpp"
#include <Eigen/QR>
#include <map>
#include <stdexcept>

namespace uwb_imu_pl::research {
// Mutually exclusive fault leaves need B_i' P B_i and B_i' P z, not
// cross-leaf B_i' P B_j. Each leaf's [B_i,z] has its own square root.
// Lost independent residual dimensions are counted independently of its rank.
struct SummaryFault {
 Eigen::MatrixXd top, residual;
 int alias_state=-1; // -2: certified alias of an eliminated, unprotected state
 bool ever_nonzero=false;
};
struct SummaryFaultResult {
 Eigen::MatrixXd residual_gram, response;
 Eigen::VectorXd profile_rhs;
 Eigen::VectorXd profile_estimate;
 std::string classification="INDETERMINATE";
};
struct SummaryEvaluation {
 GaussianReference reference;
 std::map<std::string,SummaryFaultResult> faults;
 int state_rank=0,total_rank=0;
};
struct SummaryRetirement {Eigen::MatrixXd coefficients;Eigen::VectorXd offset;Eigen::MatrixXd conditional_covariance;};
class GaussianSufficientSummary {
 public:
 Eigen::MatrixXd h;
 Eigen::VectorXd d;
 std::map<std::string,SummaryFault> faults;
 long long equations=0,residual_dimensions=0,eliminated_states=0;
 double residual_norm=0.;
 size_t orthogonal_updates=0;
 explicit GaussianSufficientSummary(int states=0):h(0,states),d(0){}
 int states()const{return h.cols();}
 size_t bytes()const{
  size_t n=h.size()+d.size();for(const auto& f:faults)n+=f.second.top.size()+f.second.residual.size();return n*sizeof(double);
 }
 void addStateColumns(int count){
  if(count<0)throw std::runtime_error("negative summary state extension");
  int old=states();h.conservativeResize(h.rows(),old+count);h.rightCols(count).setZero();
 }
 void declareFault(const std::string& id,int dimension,int alias_state=-1,bool past_support_zero=false){
  if(dimension<=0 || faults.count(id))throw std::runtime_error("duplicate/invalid summary fault");
  if(equations>0 && !past_support_zero)throw std::runtime_error("UNAVAILABLE_HISTORICAL_FAULT_INSERTION");
  if(alias_state>=states() || (alias_state>=0 && (dimension!=1 || h.rows()!=0)))throw std::runtime_error("alias requires complete raw lineage");
  SummaryFault f;f.top=Eigen::MatrixXd::Zero(h.rows(),dimension);f.residual.resize(0,dimension+1);f.alias_state=alias_state;faults.emplace(id,std::move(f));
 }
 // Rows must already be jointly independent/white, or have explicitly retained
 // frontier noise variables and their ONE Gaussian prior. No mixed-row deletion.
 void append(const Eigen::MatrixXd& a,const Eigen::VectorXd& z,
             const std::map<std::string,Eigen::MatrixXd>& b){
  if(a.cols()!=states() || a.rows()!=z.size() || !a.allFinite() || !z.allFinite())throw std::runtime_error("invalid summary append");
  for(const auto& item:b)if(!faults.count(item.first) || item.second.rows()!=a.rows() || item.second.cols()!=faults.at(item.first).top.cols() || !item.second.allFinite())throw std::runtime_error("foreign summary fault rows");
  int old=h.rows(),m=old+a.rows(),keep=std::min(m,states());
  Eigen::MatrixXd stack(m,states());stack.topRows(old)=h;stack.bottomRows(a.rows())=a;
  Eigen::VectorXd rhs(m);rhs.head(old)=d;rhs.tail(a.rows())=z;
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(stack);
  Eigen::VectorXd rotated=qr.householderQ().adjoint()*rhs;
  for(auto& item:faults){auto& f=item.second;Eigen::MatrixXd fb=Eigen::MatrixXd::Zero(m,f.top.cols());fb.topRows(old)=f.top;
   auto it=b.find(item.first);if(it!=b.end())fb.bottomRows(a.rows())=it->second;
   bool nonzero=it!=b.end() && (it->second.array()!=0.).any();f.ever_nonzero=f.ever_nonzero || nonzero;
   if(f.alias_state==-2 && nonzero)f.alias_state=-1;
   if(f.alias_state>=0){bool alias=it!=b.end()?(it->second.col(0).array()==a.col(f.alias_state).array()).all():(a.col(f.alias_state).array()==0.).all();if(!alias)f.alias_state=-1;}
   Eigen::MatrixXd transformed=qr.householderQ().adjoint()*fb;
   appendResidual(f,transformed.bottomRows(m-keep),rotated.tail(m-keep));f.top=transformed.topRows(keep);
  }
  residual_norm=std::hypot(residual_norm,rotated.tail(m-keep).norm());residual_dimensions+=m-keep;
  h=(qr.householderQ().adjoint()*stack).topRows(keep);d=rotated.head(keep);equations+=a.rows();++orthogonal_updates;
 }
 // Eliminate only a full-rank old-state block. Frontier sources stay columns.
 SummaryRetirement retire(int count){
  if(count<=0 || count>=states() || count>h.rows())throw std::runtime_error("invalid summary retirement");
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(h.leftCols(count));qr.setThreshold(1e-10);
  if(qr.rank()!=count)throw std::runtime_error("INDETERMINATE_OLD_STATE_RANK");
  Eigen::MatrixXd rotated=qr.householderQ().adjoint()*h.rightCols(states()-count);
  Eigen::VectorXd rhs=qr.householderQ().adjoint()*d;
  auto upper=qr.matrixR().topLeftCorner(count,count).eval();SummaryRetirement recovery;
  recovery.offset=qr.colsPermutation()*upper.triangularView<Eigen::Upper>().solve(rhs.head(count));
  recovery.coefficients=-(qr.colsPermutation()*upper.triangularView<Eigen::Upper>().solve(rotated.topRows(count))).eval();
  Eigen::MatrixXd conditional_gain=qr.colsPermutation()*upper.triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(count,count));
  recovery.conditional_covariance=conditional_gain*conditional_gain.transpose();
  for(auto& item:faults){item.second.top=(qr.householderQ().adjoint()*item.second.top).bottomRows(h.rows()-count).eval();
   if(item.second.alias_state>=0)item.second.alias_state=item.second.alias_state<count?-2:item.second.alias_state-count;}
  d=rhs.tail(h.rows()-count);h=rotated.bottomRows(h.rows()-count);equations-=count;eliminated_states+=count;++orthogonal_updates;
  return recovery;
 }
 SummaryEvaluation evaluate(const Eigen::MatrixXd& c,double alpha,double tail,bool include_faults=true)const{
  SummaryEvaluation out;auto& r=out.reference;
  if(c.rows()!=3 || c.cols()!=states() || !c.allFinite() || h.rows()<states() || states()==0 || !(alpha>0 && alpha<1 && tail>0 && tail<1))return out;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h,Eigen::ComputeFullU|Eigen::ComputeFullV);auto s=svd.singularValues();
  if(s.size()==0 || s.maxCoeff()==0)return out;
  out.state_rank=(s.array()>1e-9*s.maxCoeff()).count();out.total_rank=out.state_rank+eliminated_states;
  if(out.state_rank!=states())return out; // no gauge jitter or numerical-zero certificate
  auto gain=(svd.matrixV()*s.cwiseInverse().asDiagonal()*svd.matrixU().leftCols(states()).transpose()).eval();
  auto parity=svd.matrixU().rightCols(h.rows()-states()).transpose().eval();
  Eigen::VectorXd pz=parity*d;
  r.mean=gain*d;r.covariance=gain*gain.transpose();r.dof=equations-states();if(r.dof<=0)return out;
  r.statistic=residual_norm*residual_norm+pz.squaredNorm();
  r.threshold=boost::math::quantile(boost::math::complement(boost::math::chi_squared(r.dof),alpha));
  r.status=r.statistic>r.threshold?"POST_REJECTED":"FROZEN_MODEL_ONLY";
  double k=boost::math::quantile(boost::math::complement(boost::math::normal(),tail/6));
  r.pl=k*(c*r.covariance*c.transpose()).diagonal().cwiseMax(0.).cwiseSqrt();
  if(!include_faults){if(r.status=="POST_REJECTED")r.pl.setConstant(INFINITY);return out;}
  double radius=std::sqrt(r.threshold)+std::sqrt(boost::math::quantile(boost::math::complement(boost::math::chi_squared(r.dof),tail)));
  Eigen::Vector3d bound=Eigen::Vector3d::Zero();
  Eigen::MatrixXd protected_gain=c*gain;
  for(const auto& item:faults){const auto& f=item.second;int dim=f.top.cols();
   Eigen::MatrixXd pb(parity.rows()+f.residual.rows(),dim);pb.topRows(parity.rows())=parity*f.top;pb.bottomRows(f.residual.rows())=f.residual.leftCols(dim);
   Eigen::VectorXd rz(pz.size()+f.residual.rows());rz.head(pz.size())=pz;rz.tail(f.residual.rows())=f.residual.col(dim);
   SummaryFaultResult fr;fr.residual_gram=pb.transpose()*pb;fr.profile_rhs=pb.transpose()*rz;fr.response=protected_gain*f.top;
   Eigen::JacobiSVD<Eigen::MatrixXd> fs(pb,Eigen::ComputeThinU|Eigen::ComputeFullV);auto singular=fs.singularValues();
   if(singular.size()<dim || singular.minCoeff()<=1e-12){fr.classification="INDETERMINATE";
    if(!f.ever_nonzero || f.alias_state==-2 || (f.alias_state>=0 && (c.col(f.alias_state).array()==0.).all()))fr.classification="EXACT_HARMLESS";
    else if(f.alias_state>=0)fr.classification="DANGEROUS_NULLSPACE";}
   else if(singular.minCoeff()<=1e-8)fr.classification="INDETERMINATE";
   else {fr.classification="MONITORABLE";fr.profile_estimate=fs.solve(rz);auto slope=(fr.response*fs.matrixV()*singular.cwiseInverse().asDiagonal()).eval();
    for(int j=0;j<3;++j)bound(j)=std::max(bound(j),std::nextafter(radius*slope.row(j).norm(),INFINITY));}
   out.faults.emplace(item.first,fr);
   if(include_faults && r.status!="POST_REJECTED" && fr.classification!="MONITORABLE" && fr.classification!="EXACT_HARMLESS")r.status=fr.classification;
  }
  r.pl+=bound;
  if(include_faults && r.status!="FROZEN_MODEL_ONLY")r.pl.setConstant(INFINITY);
  return out;
 }
 private:
 static void appendResidual(SummaryFault& f,const Eigen::MatrixXd& b,const Eigen::VectorXd& z){
  if(b.rows()==0)return;
  int old=f.residual.rows(),m=old+b.rows(),n=b.cols()+1;
  Eigen::MatrixXd joint(m,n);joint.topRows(old)=f.residual;joint.bottomLeftCorner(b.rows(),b.cols())=b;joint.bottomRightCorner(z.size(),1)=z;
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(joint);f.residual=(qr.householderQ().adjoint()*joint).topRows(std::min(m,n));
 }
};
}
