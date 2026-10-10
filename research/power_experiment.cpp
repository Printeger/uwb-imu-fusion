#include "scenario_material.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <Eigen/QR>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
int main(int argc,char** argv) {
  try {
    if(argc!=3)throw std::invalid_argument("usage: fde_research_power CONFIG OUTPUT_CSV");
    const auto original=IntegrityConfigLoader::load(argv[1]);
    std::ofstream out(argv[2]);if(!out)throw std::runtime_error("output unavailable");
    out<<std::setprecision(17)<<"case,epoch,axis,duration_s,raw_blocks,parity_energy,lambda_at_amplitude,amplitude,p_md_conditional,threshold,dof,unit_step,unit_protected_shift,orthogonality,whitening_derivative_difference\n";
    for(int profile=0;profile<3;++profile) {
      auto cfg=original;const bool rotating=profile==1;
      if(profile==2)for(auto& a:cfg.anchors)a.position_world_m.z()=1.1;
      IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
      estimator.initialize(initialState(),cfg.realtime.prior_sigmas);estimator.ingestImu(sample(cfg,0,rotating));
      for(int epoch=1;epoch<=12;++epoch) {
        auto tx=prepare(estimator,cfg,epoch,rotating);
        if(epoch==12 || (profile==0 && epoch==3)) {
          IntegrityWindowRequest req;req.epochs=epoch;
          const auto w=estimator.buildIntegrityWindow(tx,req);
          DetectorRiskContext risk;risk.p_fa_per_test=cfg.detector.p_fa_per_test;
          risk.continuity_horizon_tests=cfg.detector.continuity_horizon_tests;
          const auto detector=JointWindowDetector{}.evaluate(w,risk);
          if(!detector.numerically_valid)throw std::runtime_error("power detector invalid: "+detector.reason);
          Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(w.H);
          qr.setThreshold(cfg.snapshot.rank_tolerance);
          if(qr.rank()!=w.H.cols())throw std::runtime_error("power QR rank unresolved");
          std::map<std::uint64_t,EpochTransaction> material;
          material.emplace(tx.imu_group.id.value(),tx);
          for(const auto& h:tx.recoverable_history)for(const auto& group:h.groups)
            if(group.kind==FactorKind::CombinedImu)material.emplace(group.id.value(),historicalEpisodeTransaction(h));
          for(int axis:{0,5})for(double duration:{.05,.25,.5}) {
            // Same physical onset for duration/history comparisons. Longer
            // episodes at epoch3 are right-censored, not a fabricated full
            // observation of the future tail.
            RawImuEpisode episode{"duration-v1",TimestampNs(105000000),TimestampNs::fromSeconds(.105+duration),axis};
            Eigen::VectorXd response=Eigen::VectorXd::Zero(w.H.rows());
            Eigen::VectorXd dependent=response;int offset=0,blocks=0;
            for(const auto& b:w.blocks) {
              const auto it=material.find(b.group_id.value());
              if(b.kind==FactorKind::CombinedImu && it!=material.end()) {
                const auto r=episodeResponse(it->second,b,episode);
                response.segment(offset,15)=r.frozen_whitened;
                dependent.segment(offset,15)=r.input_dependent_whitened;
                blocks+=r.affected_trapezoids>0;
              }
              offset+=b.residual_whitened.size();
            }
            const Eigen::VectorXd step=qr.solve(response),parity=response-w.H*step;
            const double gamma=parity.squaredNorm(),amp=axis==0?20.:.8;
            const double lambda=gamma*amp*amp;
            const double md=boost::math::cdf(boost::math::non_central_chi_squared_distribution<double>(detector.dof,lambda),detector.squared_threshold);
            const double ortho=(w.H.transpose()*parity).norm()/std::max(1.,w.H.norm()*response.norm());
            if(!std::isfinite(md) || ortho>1e-10)throw std::runtime_error("power numerical check failed");
            out<<(profile==0?"stationary":profile==1?"rotating":"flat_geometry")<<','<<epoch<<','<<axis<<','<<duration<<','<<blocks<<','<<gamma<<','<<lambda<<','<<amp<<','<<md<<','<<detector.squared_threshold<<','<<detector.dof<<','<<step.norm()<<','<<(w.protected_state_map*step).norm()<<','<<ortho<<','<<(response-dependent).norm()<<'\n';
          }
        }
        const auto plan=EpochCommitPlan::nominalPlan(tx);estimator.commitEpoch(std::move(tx),plan);
      }
    }
    std::cout<<"VERIFIED 4 directed windows; Gaussian p_md is conditional on frozen covariance; no injected amplitude extrapolation\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
