#pragma once
#include "epoch_raim.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
namespace uwb_imu_pl::research {
struct PredictionConditionsV1 {
 bool covariance_certified=false, independent_current_noise=false;
 bool clean_history=false, clean_imu=false, no_current_reuse=false;
 // These are explicit frozen-model assumptions, never inferred from an NIS pass.
};
inline std::string predictionRefusal(const PredictionConditionsV1& c) {
 if(!c.no_current_reuse)return "CURRENT_UWB_REUSED";
 if(!c.covariance_certified)return "PREDICTION_COVARIANCE_UNQUALIFIED";
 if(!c.independent_current_noise)return "PREDICTION_RANGE_DEPENDENCE_UNQUALIFIED";
 if(!c.clean_imu)return "IMU_PREDICTION_CONTAMINATED";
 if(!c.clean_history)return "HISTORY_PREDICTION_CONTAMINATED";
 return "";
}
inline Eigen::Matrix<double,3,15> tagMap(const NavigationState& s,const Eigen::Vector3d& lever) {
 gtsam::Pose3 pose(gtsam::Rot3(s.q_world_body.toRotationMatrix()),s.position_world_m);
 gtsam::Matrix36 j;pose.transformFrom(lever,j);
 Eigen::Matrix<double,3,15> c=Eigen::Matrix<double,3,15>::Zero();c.leftCols<6>()=j;return c;
}
inline EpochRaim fusedCandidate(const UwbBatch& batch,int excluded,
 const ResearchEpochPredictionV1& prediction,const Eigen::Vector3d& lever,
 const PredictionConditionsV1& conditions,double alpha,double tail) {
 EpochRaim r;r.excluded=excluded;r.status=predictionRefusal(conditions);
 if(!r.status.empty())return r;
 if(!prediction.prior.excludes_current_uwb || prediction.excluded_batch_id!=batch.id || prediction.prior.mean.timestamp!=batch.timestamp){r.status="PREDICTION_LINEAGE_FAILED";return r;}
 auto b=retainRanges(batch,excluded);int n=b.measurements.size();
 if(!spatialAnchors(b)){r.status="GEOMETRY_OR_MIRROR_UNQUALIFIED";return r;}
 const auto& state=prediction.prior.mean;auto c=tagMap(state,lever);
 gtsam::Pose3 pose(gtsam::Rot3(state.q_world_body.toRotationMatrix()),state.position_world_m);
 gtsam::Matrix hpose;Eigen::VectorXd innovation=-UwbPoseBatchFactor(1,b,lever).evaluateError(pose,hpose);
 Eigen::MatrixXd h=Eigen::MatrixXd::Zero(n,15);h.leftCols(6)=hpose;
 Eigen::MatrixXd p=prediction.prior.covariance;
 Eigen::LLT<Eigen::MatrixXd> pc(p);
 if(pc.info()!=Eigen::Success || !p.allFinite() || !p.isApprox(p.transpose(),1e-10)){r.status="PREDICTION_COVARIANCE_UNQUALIFIED";return r;}
 // Prior and measurement are ONE joint LS problem. The prior pseudo-observation
 // error is stochastic; it is not counted again as a separate measurement.
 Eigen::MatrixXd a=Eigen::MatrixXd::Zero(15+n,15),sigma=Eigen::MatrixXd::Zero(15+n,15+n);
 a.topRows(15).setIdentity();a.bottomRows(n)=h;
 sigma.topLeftCorner(15,15)=p;sigma.bottomRightCorner(n,n)=validatedUwbCovariance(b);
 Eigen::VectorXd z=Eigen::VectorXd::Zero(15+n);z.tail(n)=innovation;
 std::vector<Eigen::MatrixXd> faults;
 for(int i=0;i<n;++i){Eigen::MatrixXd f=Eigen::MatrixXd::Zero(15+n,1);f(15+i)=1.;faults.push_back(f);}
 r.gaussian=gaussianReference(a,z,sigma,c,faults,alpha,tail);
 r.status=r.gaussian.status;
 if(!r.gaussian.mean.size())return r;
 r.tag=state.position_world_m+state.q_world_body*lever+c*r.gaussian.mean;
 responses(r,c,15,n);
 for(int i=0;i<int(batch.measurements.size());++i)if(i!=excluded)r.retained.push_back(i);
 // Local domain check is numerical qualification, not a remainder proof.
 if(r.gaussian.mean.head(3).norm()>.25 || r.gaussian.mean.segment(3,3).norm()>.25){r.status="NONLINEAR_DOMAIN_FAILED";r.gaussian.pl.setConstant(INFINITY);}
 return r;
}
inline EpochRaim fusedEpochPl(const UwbBatch& b,const ResearchEpochPredictionV1& p,
 const Eigen::Vector3d& lever,const PredictionConditionsV1& c,bool fde=true,double alpha=1e-6,double tail=1e-9) {
 return epochFde(b.measurements.size(),fde,[&](int i){return fusedCandidate(b,i,p,lever,c,alpha,tail);});
}
} // namespace
