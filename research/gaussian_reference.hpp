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
inline double riskFloor(double nominal,double other,const std::vector<double>& priors,
                        double alpha,double beta,int candidates) {
  long double total=nominal+other;
  for(double p:priors)total+=p*std::max(alpha,beta)*candidates;
  return std::nextafter(double(total),INFINITY);
}
inline GaussianReference gaussianReference(const Eigen::MatrixXd& a,const Eigen::VectorXd& z,
    const Eigen::MatrixXd& sigma,const Eigen::MatrixXd& c,const std::vector<Eigen::MatrixXd>& faults,
    double alpha,double tail) {
  GaussianReference r;
  if(a.rows()!=z.size() || sigma.rows()!=a.rows() || sigma.cols()!=a.rows() || c.rows()!=3 || c.cols()!=a.cols()
      || !a.allFinite() || !z.allFinite() || !sigma.allFinite() || !c.allFinite() || !(alpha>0 && alpha<1 && tail>0 && tail<1))return r;
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
  // Each candidate pays tail for three coordinate Gaussian errors and tail for
  // its parity noise. All fault leaves share that candidate noise event.
  double k=boost::math::quantile(boost::math::complement(boost::math::normal(),tail/6));
  r.pl=k*(c*r.covariance*c.transpose()).diagonal().cwiseMax(0.).cwiseSqrt();
  const double radius=std::sqrt(r.threshold)+std::sqrt(boost::math::quantile(
      boost::math::complement(boost::math::chi_squared(r.dof),tail)));
  Eigen::Vector3d fault_bound=Eigen::Vector3d::Zero();
  for(const auto& b:faults) {
    Eigen::MatrixXd bw=r.whitener*b, zz=r.parity*bw, g=c*r.gain*bw;
    Eigen::JacobiSVD<Eigen::MatrixXd> f(zz,Eigen::ComputeFullV|Eigen::ComputeThinU);
    const auto fs=f.singularValues();Eigen::VectorXd inv=Eigen::VectorXd::Zero(fs.size());
    for(int j=0;j<fs.size();++j) {
      if(fs(j)>1e-8)inv(j)=1/fs(j);
      else if(fs(j)>1e-12){r.pl.setConstant(INFINITY);r.status="INDETERMINATE";return r;}
      else if((g*f.matrixV().col(j)).norm()>1e-10){r.pl.setConstant(INFINITY);r.status="DANGEROUS_NULLSPACE";return r;}
    }
    for(int j=fs.size();j<f.matrixV().cols();++j)
      if((g*f.matrixV().col(j)).norm()>1e-10){r.pl.setConstant(INFINITY);r.status="DANGEROUS_NULLSPACE";return r;}
    Eigen::MatrixXd slope=g*f.matrixV().leftCols(fs.size())*inv.asDiagonal();
    for(int j=0;j<3;++j)fault_bound(j)=std::max(fault_bound(j),radius*slope.row(j).norm());
  }
  r.pl+=fault_bound;r.status="FROZEN_MODEL_ONLY";return r;
}
// Select in RAW coordinates first, then factor the covariance principal block.
inline Eigen::MatrixXd principal(const Eigen::MatrixXd& s,const std::vector<int>& rows) {
  Eigen::MatrixXd out(rows.size(),rows.size());
  for(size_t i=0;i<rows.size();++i)for(size_t j=0;j<rows.size();++j)out(i,j)=s(rows[i],rows[j]);
  return out;
}
}
