#include "uifgo/fde_math.h"
#include <Eigen/Eigenvalues>
#include <Eigen/SparseQR>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <stdexcept>
namespace uifgo {
double FdeChiSquareQuantile(size_t dof, double probability) {
  if (!dof || !std::isfinite(probability) || probability <= 0.0 ||
      probability >= 1.0)
    throw std::invalid_argument("FDE requires positive DoF and probability in (0,1)");
  return boost::math::quantile(boost::math::chi_squared_distribution<double>(dof),probability);
}
FdeQuadraticTest FdeCovarianceTest(const Eigen::VectorXd& e,const Eigen::MatrixXd& C) {
  FdeQuadraticTest t; RecoverabilityOptions o;
  if (C.rows()!=e.size() || C.cols()!=e.size() || !e.size() || !C.allFinite() || !e.allFinite()) {t.reason="INVALID_INPUT";return t;}
  if ((C-C.transpose()).norm()>o.symmetry_absolute_tolerance+o.symmetry_relative_tolerance*C.norm()) {t.reason="ASYMMETRIC";return t;}
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig((C+C.transpose())*.5);
  if(eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() || !eig.eigenvectors().allFinite()){t.reason="EIGEN_FAILED";return t;}
  t.spectrum=eig.eigenvalues();
  t.rank_threshold=std::max(o.rank_absolute_tolerance,o.rank_relative_tolerance*t.spectrum.cwiseAbs().maxCoeff());
  if(t.spectrum.minCoeff() < -t.rank_threshold){t.reason="NOT_PSD";return t;}
  const Eigen::VectorXd v=eig.eigenvectors().transpose()*e;
  for(int i=0;i<v.size();++i) if(t.spectrum[i]>t.rank_threshold){++t.rank;t.statistic+=v[i]*v[i]/t.spectrum[i];}else t.null_component_norm+=v[i]*v[i];
  t.null_component_norm=std::sqrt(t.null_component_norm);
  if(!t.rank){t.reason="ZERO_REDUNDANCY";return t;}
  if(!std::isfinite(t.statistic)){t.reason="NONFINITE_STATISTIC";return t;}
  t.threshold=FdeChiSquareQuantile(t.rank);t.rejected=t.statistic>t.threshold;t.valid=true;t.reason="OK_RANGE_QUADRATIC_NULL_COMPONENT_REPORTED";return t;
}
FdeProjectionBlock FdeFullProjectionBlock(const Eigen::SparseMatrix<double>& A,const Eigen::VectorXd& e,const std::vector<size_t>& rows){
  FdeProjectionBlock out;out.certificate=ComputeSparseResidualProjectionDiagonal(A,rows);
  if(!out.certificate.valid)return out;
  if(e.size()!=A.rows() || !e.allFinite())throw std::invalid_argument("INVALID_RESIDUAL");
  Eigen::SparseMatrix<double> F=A;Eigen::VectorXd scales(A.cols());
  for(int j=0;j<F.cols();++j){
    double largest=0, sum=0;
    for(Eigen::SparseMatrix<double>::InnerIterator it(F,j);it;++it) largest=std::max(largest,std::abs(it.value()));
    for(Eigen::SparseMatrix<double>::InnerIterator it(F,j);it;++it) sum+=std::pow(it.value()/largest,2);
    const double norm=largest*std::sqrt(sum);scales[j]=1./norm;
    if(!std::isfinite(scales[j]))throw std::runtime_error("ORIGINAL_UNIT_RESPONSE_UNAVAILABLE");
    for(Eigen::SparseMatrix<double>::InnerIterator it(F,j);it;++it)it.valueRef()/=norm;
  }
  Eigen::SparseQR<Eigen::SparseMatrix<double>,Eigen::COLAMDOrdering<int>> qr;qr.setPivotThreshold(0);qr.compute(F);
  if(qr.info()!=Eigen::Success || qr.rank()!=out.certificate.rank)throw std::runtime_error("QR_CERTIFICATE_MISMATCH");
  Eigen::MatrixXd E=Eigen::MatrixXd::Zero(A.rows(),rows.size());for(size_t j=0;j<rows.size();++j)E(rows[j],j)=1;
  const Eigen::MatrixXd x=qr.solve(E);const Eigen::MatrixXd Z=E-F*x;
  out.response=scales.asDiagonal()*x;
  if(qr.info()!=Eigen::Success || !x.allFinite() || !Z.allFinite() || !out.response.allFinite())
    throw std::runtime_error("NONFINITE_PROJECTION_RESPONSE");
  out.Pww.resize(rows.size(),rows.size());for(size_t j=0;j<rows.size();++j)out.Pww.row(j)=Z.row(rows[j]);
  out.projected_residual=e-F*qr.solve(e);
  if(!out.projected_residual.allFinite())throw std::runtime_error("NONFINITE_PROJECTED_RESIDUAL");
  out.symmetry_error=(out.Pww-out.Pww.transpose()).norm();
  out.idempotence_error=(Z.transpose()*Z-out.Pww).norm();
  out.orthogonality_error=(F.transpose()*Z).norm()/F.norm();
  RecoverabilityOptions o;
  double tol=o.orthogonality_absolute_tolerance+o.orthogonality_relative_tolerance+out.certificate.projection_residual_floor;
  if(out.symmetry_error>tol*std::max(1.,out.Pww.norm()) || out.idempotence_error>tol*std::max(1.,out.Pww.norm()) || out.orthogonality_error>tol)throw std::runtime_error("BLOCK_PROJECTOR_AUDIT_FAILED");
  for(size_t j=0;j<rows.size();++j)if(std::abs(out.Pww(j,j)-out.certificate.diagonal[j])>tol)throw std::runtime_error("DIAGONAL_MISMATCH");
  return out;
}
}
