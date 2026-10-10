#pragma once
#include "gaussian_reference.hpp"
#include <Eigen/QR>
#include <stdexcept>
namespace uwb_imu_pl::research {
// Fixed-model transformation only. Eliminate old states after GLOBAL joint
// whitening, keep EVERY residual dimension. Never choose detector dof from z
// or from the rank of [fault,z]. All source/remainder maps and future cross
// covariance travel through the same transform. Delete raw rows BEFORE this.
struct StatisticalHistory {
 Eigen::MatrixXd a,b,j,sigma,cross,transform;
 Eigen::VectorXd z;
 int eliminated=0;
};
inline StatisticalHistory statisticalHistory(const Eigen::MatrixXd& old,
 const Eigen::MatrixXd& retained,const Eigen::VectorXd& z,
 const Eigen::MatrixXd& sigma,const Eigen::MatrixXd& b,
 const Eigen::MatrixXd& j,const Eigen::MatrixXd& future_cross) {
 const int m=z.size(), n=old.cols();
 if(old.rows()!=m || retained.rows()!=m || sigma.rows()!=m || sigma.cols()!=m ||
    b.rows()!=m || j.rows()!=m || future_cross.rows()!=m || n>=m ||
    !old.allFinite() || !retained.allFinite() || !z.allFinite() ||
    !sigma.allFinite() || !sigma.isApprox(sigma.transpose(),1e-12))
  throw std::runtime_error("invalid statistical history input");
 Eigen::LLT<Eigen::MatrixXd> chol(sigma);if(chol.info()!=Eigen::Success)
  throw std::runtime_error("history covariance not positive definite");
 Eigen::MatrixXd w=chol.matrixL().solve(Eigen::MatrixXd::Identity(m,m));
 Eigen::MatrixXd q=Eigen::MatrixXd::Identity(m,m);
 if(n){Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(w*old);qr.setThreshold(1e-10);
  if(qr.rank()!=n)throw std::runtime_error("history old-state rank unresolved");
  q=qr.householderQ();}
 StatisticalHistory out;out.eliminated=n;
 out.transform=q.rightCols(m-n).transpose()*w;
 out.a=out.transform*retained;out.z=out.transform*z;out.b=out.transform*b;
 out.j=out.transform*j;out.sigma=out.transform*sigma*out.transform.transpose();
 out.cross=out.transform*future_cross;
 return out;
}
}
