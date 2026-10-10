#pragma once
// Research-only dense oracle. Production estimator/backend is not duplicated.
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>
#include <vector>
#include <string>
namespace uwb_imu_pl::research {
struct GaussianReference {
  std::string status="INDETERMINATE";
  Eigen::VectorXd mean;
  Eigen::MatrixXd covariance, gain, parity, whitener;
  Eigen::Vector3d pl=Eigen::Vector3d::Constant(INFINITY);
  double statistic=INFINITY, threshold=0.;
  int dof=0;
};
// Exact structural witnesses only: no small-value test can certify a zero.
// Literal raw column aliases require no floating matrix multiplication.
inline int exactFaultRelation(const Eigen::MatrixXd& b,const Eigen::MatrixXd& c,
                              const Eigen::MatrixXd* a) {
  bool harmless=true;
  for(int j=0;j<b.cols();++j){
    if((b.col(j).array()==0.).all())continue;
    bool certified=false;
    if(a)for(int k=0;k<a->cols();++k)if((b.col(j).array()==a->col(k).array()).all()){
      if((c.col(k).array()!=0.).any())return 2; // exact dangerous state alias
      certified=true;break;
    }
    harmless=harmless && certified;
  }
  return harmless?1:0;
}
inline GaussianReference gaussianFaultBounds(GaussianReference r,const Eigen::MatrixXd& c,
    const std::vector<Eigen::MatrixXd>& faults,double tail,bool batched=false,
    const Eigen::MatrixXd* source_design=nullptr) {
  if(r.statistic>r.threshold){r.status="POST_REJECTED";return r;}
  // Each candidate pays tail for three coordinate Gaussian errors and tail for
  // its parity noise. All fault leaves share that candidate noise event.
  double k=boost::math::quantile(boost::math::complement(boost::math::normal(),tail/6));
  r.pl=k*(c*r.covariance*c.transpose()).diagonal().cwiseMax(0.).cwiseSqrt();
  const double radius=std::sqrt(r.threshold)+std::sqrt(boost::math::quantile(
      boost::math::complement(boost::math::chi_squared(r.dof),tail)));
  Eigen::Vector3d fault_bound=Eigen::Vector3d::Zero();
  int total=0;for(const auto& b:faults){
    if(b.rows()!=r.whitener.cols() || b.cols()==0 || !b.allFinite())return GaussianReference{};
    total+=b.cols();
  }
  Eigen::MatrixXd batch_bw,batch_z,batch_g,error_z,error_g;
  if(batched && total){
    Eigen::MatrixXd b(r.whitener.cols(),total);int offset=0;
    for(const auto& f:faults){b.middleCols(offset,f.cols())=f;offset+=f.cols();}
    batch_bw=r.whitener*b;batch_z=r.parity*batch_bw;
    Eigen::MatrixXd cg=c*r.gain;batch_g=cg*batch_bw;
    // Componentwise dot-product error bounds for fixed stored operands.
    // Factor 2 also conservatively covers rounding of these bound products.
    const double eps=std::numeric_limits<double>::epsilon();
    auto gamma=[&](int n){return 2*n*eps/(1-n*eps);};
    Eigen::MatrixXd ebw=gamma(b.rows())*r.whitener.cwiseAbs()*b.cwiseAbs();
    Eigen::MatrixXd ecg=gamma(c.cols())*c.cwiseAbs()*r.gain.cwiseAbs();
    error_z=gamma(batch_bw.rows())*r.parity.cwiseAbs()*batch_bw.cwiseAbs()+r.parity.cwiseAbs()*ebw;
    error_g=gamma(batch_bw.rows())*cg.cwiseAbs()*batch_bw.cwiseAbs()+cg.cwiseAbs()*ebw+ecg*batch_bw.cwiseAbs()+ecg*ebw;
  }
  int offset=0;
  for(const auto& b:faults) {
    Eigen::MatrixXd bw,zz,g;
    bool guarded=batched && b.cols()==1;
    if(guarded){bw=batch_bw.middleCols(offset,b.cols());zz=batch_z.middleCols(offset,b.cols());g=batch_g.middleCols(offset,b.cols());
      const double error=error_z.middleCols(offset,b.cols()).norm();
      if(!zz.allFinite() || zz.norm()-error<=1e-7)guarded=false;
    }
    if(!guarded){bw=r.whitener*b;zz=r.parity*bw;g=c*r.gain*bw;}
    if(!zz.allFinite() || !g.allFinite())return GaussianReference{};
    Eigen::JacobiSVD<Eigen::MatrixXd> f(zz,Eigen::ComputeFullV|Eigen::ComputeThinU);
    const auto fs=f.singularValues();Eigen::VectorXd inv=Eigen::VectorXd::Zero(fs.size());
    if(fs.size()<b.cols() || (fs.array()<=1e-12).any()){
      const int relation=exactFaultRelation(b,c,source_design);
      if(relation==1){offset+=b.cols();continue;}
      r.pl.setConstant(INFINITY);
      r.status=relation==2?"DANGEROUS_NULLSPACE":"INDETERMINATE";
      return r;
    }
    for(int j=0;j<fs.size();++j) {
      if(fs(j)>1e-8)inv(j)=1/fs(j);
      else {r.pl.setConstant(INFINITY);r.status="INDETERMINATE";return r;}
    }
    Eigen::MatrixXd slope=g*f.matrixV().leftCols(fs.size())*inv.asDiagonal();
    for(int j=0;j<3;++j){
      double value=radius*slope.row(j).norm();
      if(guarded)value=radius*(g.row(j).norm()+error_g.block(j,offset,1,1).norm())/
          (zz.norm()-error_z.middleCols(offset,1).norm());
      fault_bound(j)=std::max(fault_bound(j),std::nextafter(value,INFINITY));
    }
    offset+=b.cols();
  }
  r.pl+=fault_bound;r.status="FROZEN_MODEL_ONLY";return r;
}

inline double riskFloor(double nominal,double other,const std::vector<double>& priors,
                        double alpha,double beta,int candidates) {
  long double total=nominal+other;
  for(double p:priors)total+=p*std::max(alpha,beta)*candidates;
  return std::nextafter(double(total),INFINITY);
}
inline GaussianReference gaussianReference(const Eigen::MatrixXd& a,const Eigen::VectorXd& z,
    const Eigen::MatrixXd& sigma,const Eigen::MatrixXd& c,const std::vector<Eigen::MatrixXd>& faults,
    double alpha,double tail,bool batched=false) {
  GaussianReference r;
  if(a.rows()!=z.size() || sigma.rows()!=a.rows() || sigma.cols()!=a.rows() || c.rows()!=3 || c.cols()!=a.cols()
      || !a.allFinite() || !z.allFinite() || !sigma.allFinite() || !sigma.isApprox(sigma.transpose(),1e-12) || !c.allFinite() || !(alpha>0 && alpha<1 && tail>0 && tail<1))return r;
  Eigen::LLT<Eigen::MatrixXd> chol(sigma);
  if(chol.info()!=Eigen::Success)return r;
  r.whitener=chol.matrixL().solve(Eigen::MatrixXd::Identity(a.rows(),a.rows()));
  const Eigen::MatrixXd h=r.whitener*a;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h,Eigen::ComputeThinV|Eigen::ComputeFullU);
  const auto s=svd.singularValues();
  if(s.size()==0 || s.minCoeff()<1e-9*s.maxCoeff())return r; // unresolved gauge/rank: never jitter
  r.dof=h.rows()-h.cols();if(r.dof<=0)return r;
  r.gain=svd.matrixV()*s.cwiseInverse().asDiagonal()*svd.matrixU().leftCols(h.cols()).transpose();
  r.parity=svd.matrixU().rightCols(r.dof).transpose();
  r.mean=r.gain*r.whitener*z;r.covariance=r.gain*r.gain.transpose();
  r.statistic=(r.parity*r.whitener*z).squaredNorm();
  r.threshold=boost::math::quantile(boost::math::complement(boost::math::chi_squared(r.dof),alpha));
  return gaussianFaultBounds(std::move(r),c,faults,tail,batched,&a);
}
// Select in RAW coordinates first, then factor the covariance principal block.
inline Eigen::MatrixXd principal(const Eigen::MatrixXd& s,const std::vector<int>& rows) {
  Eigen::MatrixXd out(rows.size(),rows.size());
  for(size_t i=0;i<rows.size();++i)for(size_t j=0;j<rows.size();++j)out(i,j)=s(rows[i],rows[j]);
  return out;
}
}
