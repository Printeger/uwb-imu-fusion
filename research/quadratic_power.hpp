#pragma once
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>
namespace uwb_imu_pl::research {
struct QuadraticPower {
  double mean=0.,variance=0.,miss_lower=0.,miss_upper=1.;
  bool standard_noncentral_chi_squared=false;
};
// Conditional frozen linear Gaussian law. With Q2 a parity basis, K=Q2'ΩQ2
// and μ=Q2'D f, S=||N(μ,K)||² is a GENERALIZED noncentral quadratic form.
// A raw shared sample generally destroys the ordinary nc-chi² premise.
// Cantelli bounds need no quadrature, seed, threshold tuning or qualification.
inline QuadraticPower quadraticPower(const Eigen::MatrixXd& h,const Eigen::VectorXd& shift,
    const Eigen::MatrixXd& omega,double threshold) {
  if(h.rows()!=shift.size() || omega.rows()!=h.rows() || omega.cols()!=h.rows() ||
      !h.allFinite() || !shift.allFinite() || !omega.allFinite() || !std::isfinite(threshold) || threshold<0.)
    throw std::invalid_argument("invalid conditional quadratic law");
  if(!omega.isApprox(omega.transpose(),1e-12))throw std::invalid_argument("asymmetric conditional covariance");
  Eigen::LDLT<Eigen::MatrixXd> covariance(omega);
  if(covariance.info()!=Eigen::Success || covariance.vectorD().minCoeff()<0.)throw std::invalid_argument("indefinite conditional covariance");
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(h);qr.setThreshold(1e-10);
  if(qr.rank()!=h.cols() || h.rows()<=h.cols())throw std::runtime_error("quadratic parity rank unresolved");
  const Eigen::MatrixXd q=qr.householderQ()*Eigen::MatrixXd::Identity(h.rows(),h.rows());
  const Eigen::MatrixXd q2=q.rightCols(h.rows()-h.cols());
  const Eigen::MatrixXd k=q2.transpose()*omega*q2;const Eigen::VectorXd mu=q2.transpose()*shift;
  QuadraticPower result;result.mean=k.trace()+mu.squaredNorm();
  result.variance=2*k.squaredNorm()+4*mu.dot(k*mu);
  result.standard_noncentral_chi_squared=(k-Eigen::MatrixXd::Identity(k.rows(),k.cols())).norm()<1e-10;
  if(result.mean>threshold)result.miss_upper=result.variance/(result.variance+std::pow(result.mean-threshold,2));
  else if(result.mean<threshold)result.miss_lower=1-result.variance/(result.variance+std::pow(threshold-result.mean,2));
  return result;
}
} // namespace
