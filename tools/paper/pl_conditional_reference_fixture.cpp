#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <iomanip>
#include <iostream>

// Independent transcription of the equations at PL commit ae54fb8c. This
// executable deliberately does not link the IE conditional core.
int main() {
  Eigen::Matrix<double,15,15> p =
      Eigen::Matrix<double,15,15>::Identity() * 0.01;
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3,15);
  h(0,0)=1.0; h(1,1)=1.0; h(2,2)=1.0;
  Eigen::Matrix3d r;
  r << 1.0,0.2,0.0, 0.2,2.0,0.1, 0.0,0.1,1.5;
  Eigen::Vector3d nu(0.2,-0.3,0.1);
  Eigen::LLT<Eigen::Matrix3d> llt(r);
  Eigen::Matrix3d w = llt.matrixL().solve(Eigen::Matrix3d::Identity());
  Eigen::Vector3d nuw = w*nu;
  Eigen::MatrixXd hw = w*h;
  Eigen::Matrix3d sw = hw*p*hw.transpose()+Eigen::Matrix3d::Identity();
  const double statistic = nuw.dot(sw.ldlt().solve(nuw));
  const double threshold = boost::math::quantile(
      boost::math::chi_squared(3),1.0-1e-5);
  std::cout << std::setprecision(17)
            << "{\"source_commit\":\"ae54fb8ca55dfbfaf64fe45615b6bcd106548a93\","
            << "\"dof\":3,\"p_fa\":1e-5,\"statistic\":" << statistic
            << ",\"threshold\":" << threshold
            << ",\"passed\":" << (statistic<=threshold ? "true":"false")
            << "}\n";
  return 0;
}
