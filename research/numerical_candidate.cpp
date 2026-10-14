#include "numerical_candidate.hpp"
#include <Eigen/Eigenvalues>
namespace uwb_imu_pl::research {
NumericalCandidateResult numericalUwbCandidate(const UwbBatch& batch,int excluded,double alpha){
 NumericalCandidateResult out;auto& r=out.functional;r.excluded=excluded;
 auto b=retainRanges(batch,excluded);const int n=b.measurements.size();
 if(n<5 || !(alpha>0 && alpha<1)){r.status="NUMERICAL_INPUT_REFUSED";return out;}
 Eigen::Vector3d origin=Eigen::Vector3d::Zero();for(const auto& m:b.measurements){if(!m.anchor_position_m.allFinite() || !std::isfinite(m.range_m)){r.status="NUMERICAL_INPUT_REFUSED";return out;}origin+=m.anchor_position_m;}origin/=n;
 Eigen::MatrixXd design(n-1,3),centered(n,3);Eigen::VectorXd rhs(n-1);
 double scale=0;const auto& first=b.measurements[0];
 for(int i=0;i<n;++i){centered.row(i)=(b.measurements[i].anchor_position_m-origin).transpose();scale=std::max(scale,centered.row(i).norm());if(i){const auto& m=b.measurements[i];design.row(i-1)=2*(m.anchor_position_m-first.anchor_position_m).transpose();rhs(i-1)=m.anchor_position_m.squaredNorm()-first.anchor_position_m.squaredNorm()-m.range_m*m.range_m+first.range_m*first.range_m;}}
 if(!(scale>1e-9) || !std::isfinite(scale)){r.status="NUMERICAL_GEOMETRY_REFUSED";return out;}
 out.design_singular=Eigen::JacobiSVD<Eigen::MatrixXd>(design).singularValues();out.squared_seed=design.colPivHouseholderQr().solve(rhs);
 Eigen::LLT<Eigen::MatrixXd> chol(validatedUwbCovariance(b));if(chol.info()!=Eigen::Success){r.status="NUMERICAL_COVARIANCE_REFUSED";return out;}
 Eigen::MatrixXd w=chol.matrixL().solve(Eigen::MatrixXd::Identity(n,n));
 Eigen::JacobiSVD<Eigen::MatrixXd> plane(centered,Eigen::ComputeFullV);Eigen::Vector3d normal=plane.matrixV().col(2);int axis;normal.cwiseAbs().maxCoeff(&axis);if(normal(axis)<0)normal=-normal;
 // Bounded, deterministic initial pool. Clipping is numerical initialization
 // only; it never prunes a physical root or modifies a raw set.
 Eigen::Vector3d ls=(out.squared_seed-origin)/scale;for(int j=0;j<3;++j)ls(j)=std::clamp(ls(j),-2.,2.);
 std::vector<Eigen::Vector3d> seeds{ls,.25*normal,-.25*normal,normal,-normal,Eigen::Vector3d::Zero()};
 auto evaluate=[&](const Eigen::Vector3d& x,Eigen::MatrixXd* jac){Eigen::VectorXd residual(n);Eigen::Vector3d p=origin+scale*x;if(jac)jac->resize(n,3);for(int i=0;i<n;++i){Eigen::Vector3d delta=p-b.measurements[i].anchor_position_m;double length=delta.norm();if(!std::isfinite(length) || length<1e-8){residual.setConstant(INFINITY);return residual;}residual(i)=b.measurements[i].range_m-length;if(jac)jac->row(i)=scale*delta.transpose()/length;}return Eigen::VectorXd(w*residual);};
 double best=INFINITY;Eigen::Vector3d chosen=Eigen::Vector3d::Constant(NAN);
 for(const auto& seed:seeds){++out.attempted;Eigen::Vector3d x=seed;double lambda=1e-3,trust=1.;bool converged=false;
  for(int it=0;it<80;++it){Eigen::MatrixXd j;Eigen::VectorXd z=evaluate(x,&j);if(!z.allFinite())break;Eigen::MatrixXd h=w*j;Eigen::Matrix3d a=h.transpose()*h;Eigen::Vector3d gradient=h.transpose()*z;
   // Retain exact residual curvature. Near an almost planar minimizer the
   // tiny GN z curvature otherwise forces damping that stalls well-observed
   // x/y directions. A positive shift handles nonconvex seeds safely.
   Eigen::VectorXd weights=w.transpose()*z;Eigen::Vector3d point=origin+scale*x;
   for(int i=0;i<n;++i){Eigen::Vector3d delta=point-b.measurements[i].anchor_position_m;double distance=delta.norm();Eigen::Vector3d unit=delta/distance;a-=weights(i)*scale*scale/distance*(Eigen::Matrix3d::Identity()-unit*unit.transpose());}
   double minimum=Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(a).eigenvalues().minCoeff();if(minimum<1e-9)a.diagonal().array()+=1e-9-minimum;
   if(gradient.lpNorm<Eigen::Infinity>()<1e-7){converged=true;break;}
   Eigen::Vector3d diagonal=a.diagonal().cwiseAbs().cwiseMax(1.);Eigen::Matrix3d regular=a;regular.diagonal()+=lambda*diagonal;Eigen::Vector3d step=regular.ldlt().solve(gradient);if(!step.allFinite())break;
   if(step.norm()>trust)step*=trust/step.norm();double predicted=2*gradient.dot(step)-step.dot(a*step);
   Eigen::VectorXd trial=evaluate(x+step,nullptr);double decrease=z.squaredNorm()-trial.squaredNorm();
   if(trial.allFinite() && predicted>0 && decrease>0){x+=step;++out.accepted_steps;double ratio=decrease/predicted;if(ratio>.75){lambda=std::max(1e-12,lambda/3);trust=std::min(2.,trust*2);}else if(ratio<.25){lambda*=4;trust*=.5;}if(step.norm()<1e-9 || decrease<1e-12*(1+z.squaredNorm())){converged=true;break;}}
   else{++out.rejected_steps;lambda*=10;trust*=.5;if(step.norm()<1e-8 && gradient.norm()<1e-4){converged=true;break;}if(lambda>1e18 || trust<1e-12)break;}
  }
  Eigen::VectorXd final=evaluate(x,nullptr);if(!converged || !final.allFinite())continue;++out.converged;Eigen::Vector3d root=origin+scale*x;double objective=final.squaredNorm();bool duplicate=false;for(const auto& old:out.roots)duplicate|=(root-old).norm()<1e-5;if(!duplicate){out.roots.push_back(root);out.objectives.push_back(objective);}
  // Stable seed order breaks numerically indistinguishable ties; no GT/prior.
  if(objective<best-1e-10){best=objective;chosen=root;}
 }
 if(!chosen.allFinite()){r.status="NUMERICAL_NO_CONVERGED_ROOT";return out;}
 out.numeric_converged=true;r.tag=chosen;
 Eigen::MatrixXd final_j;Eigen::VectorXd final_z=evaluate((chosen-origin)/scale,&final_j);final_j/=scale;
 out.jacobian_singular=Eigen::JacobiSVD<Eigen::MatrixXd>(final_j).singularValues();
 Eigen::VectorXd final_raw(n);for(int i=0;i<n;++i)final_raw(i)=b.measurements[i].range_m-(chosen-b.measurements[i].anchor_position_m).norm();
 r.gaussian=gaussianReference(final_j,final_raw,validatedUwbCovariance(b),Eigen::Matrix3d::Identity(),{},alpha,1e-9);
 // Covariance/Jacobian are local diagnostics. Suppress PL and old qualification.
 out.functional_post_test=r.gaussian.dof>0 && std::isfinite(r.gaussian.statistic) && r.gaussian.statistic<=r.gaussian.threshold;
 r.gaussian.pl.setConstant(INFINITY);r.gaussian.status="MULTIROOT_LOCAL_DIAGNOSTIC_NO_ALPHA_QUALIFICATION";
 r.status="NUMERICAL_FUNCTIONAL_ONLY_GAUSSIAN_UNQUALIFIED";
 for(int i=0;i<int(batch.measurements.size());++i)if(i!=excluded)r.retained.push_back(i);
 return out;
}
}
