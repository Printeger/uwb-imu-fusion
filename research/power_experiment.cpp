#include "scenario_material.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <Eigen/QR>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
namespace {
// Analytic motion is used only to generate sensors. No trajectory/velocity
// factor is supplied to the estimator. Initial velocity is zero in both cases.
Eigen::Vector3d displacement(double t) {
  return { .8*std::sin(4*t)-3.2*t, .6*(1-std::cos(3*t)), .3*std::sin(2*t)-.6*t };
}
Eigen::Vector3d acceleration(double t) {
  return {-12.8*std::sin(4*t),5.4*std::cos(3*t),-1.2*std::sin(2*t)};
}
int directions(const IntegrityConfig& cfg,const char* output) {
  // Deterministic differentiation checks for the raw motion generator.
  for(double t:{0.,.2,.6}) {
    const double h=1e-4;
    if(((displacement(t+h)-2*displacement(t)+displacement(t-h))/(h*h)-acceleration(t)).norm()>1e-6)
      throw std::runtime_error("analytic sensor generator acceleration mismatch");
  }
  std::ofstream out(output);if(!out)throw std::runtime_error("output unavailable");
  out<<std::setprecision(17)<<"case,axis,duration_s,raw_blocks,gamma,lambda,amplitude,p_md_conditional,threshold,dof,unit_step,unit_position_shift,required_amplitude_linear,step_limited_amplitude,finite_parity,finite_signal_relative_error,baseline_statistic,orthogonality,classification\n";
  for(bool excited:{false,true}) {
    IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
    estimator.initialize(initialState(),cfg.realtime.prior_sigmas);
    auto sensor=[&](int i) {
      auto s=sample(cfg,i,excited);
      if(excited)s.specific_force_mps2+=attitude(i*.005,true).conjugate()*acceleration(i*.005);
      return s;
    };
    estimator.ingestImu(sensor(0));
    for(int epoch=1;epoch<=12;++epoch) {
      for(int i=(epoch-1)*10+1;i<=epoch*10;++i)estimator.ingestImu(sensor(i));
      auto raw=ranges(cfg,epoch);
      if(excited)for(auto& r:raw.measurements)
        r.range_m=(r.anchor_position_m-Eigen::Vector3d(0,0,1)-displacement(epoch*.05)).norm();
      auto tx=estimator.prepareEpoch(raw);
      if(epoch==12) {
        IntegrityWindowRequest req;req.epochs=12;const auto w=estimator.buildIntegrityWindow(tx,req);
        DetectorRiskContext risk;risk.p_fa_per_test=cfg.detector.p_fa_per_test; risk.continuity_horizon_tests=cfg.detector.continuity_horizon_tests;
        const auto detector=JointWindowDetector{}.evaluate(w,risk);
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(w.H);qr.setThreshold(cfg.snapshot.rank_tolerance);
        if(!detector.numerically_valid || !detector.passed || qr.rank()!=w.H.cols())throw std::runtime_error("direction snapshot rank/baseline unresolved");
        const double boundary=ProtectionLevelV2::detectionBoundaryNoncentralitySquared(detector.dof,detector.squared_threshold,cfg.fault_models.imu.p_md);
        std::map<FactorGroupId,EpochTransaction> material;material.emplace(tx.imu_group.id,tx);
        for(const auto& history:tx.recoverable_history)for(const auto& group:history.groups)
          if(group.kind==FactorKind::CombinedImu)material.emplace(group.id,historicalEpisodeTransaction(history));
        for(int axis=0;axis<6;++axis)for(double duration:{.05,.5}) {
          RawImuEpisode episode{"directions-v1",TimestampNs(105000000),TimestampNs::fromSeconds(.105+duration),axis};
          Eigen::VectorXd response=Eigen::VectorXd::Zero(w.H.rows()),finite=response;
          const double amp=axis<3?20.:.8;int offset=0,blocks=0;
          for(const auto& b:w.blocks) {
            const auto it=material.find(b.group_id);
            if(b.kind==FactorKind::CombinedImu && it!=material.end()) {
              const auto r=episodeResponse(it->second,b,episode);response.segment(offset,15)=r.frozen_whitened;
              if(r.affected_trapezoids) {
                ++blocks;
                const auto e0=episodeFactorError(it->second,integrateEpisode(it->second,episode,0.));
                const auto ef=episodeFactorError(it->second,integrateEpisode(it->second,episode,amp));
                finite.segment(offset,15)=-b.whitener*(ef-e0);
              }
            }
            offset+=b.residual_whitened.size();
          }
          const Eigen::VectorXd step=qr.solve(response),parity=response-w.H*step;
          const double gamma=parity.squaredNorm(),lambda=gamma*amp*amp;
          const double shift=(w.protected_state_map*step).norm();
          if(blocks!=(duration<.1?2:10))throw std::runtime_error("episode adjacency/support mismatch");
          if(!excited && axis==5 && (gamma>1e-20 || shift>1e-10))throw std::runtime_error("stationary yaw counterexample violated");
          if(excited && axis==5 && gamma<1e-6)throw std::runtime_error("excited yaw observation absent");
          const double ortho=(w.H.transpose()*parity).norm()/std::max(1.,w.H.norm()*response.norm());
          if(ortho>1e-10)throw std::runtime_error("direction projection failed");
          const double required=gamma>1e-20?std::sqrt(boundary/gamma):std::numeric_limits<double>::infinity();
          const double cap=cfg.integrity_window.max_linearization_step_norm/step.norm();
          const Eigen::VectorXd finite_parity=finite-w.H*qr.solve(finite);
          const double finite_relative=(finite-amp*response).norm()/std::max(1.,finite.norm());
          if(!std::isfinite(finite_relative) || finite_relative>1e-3)throw std::runtime_error("raw finite signal leaves validated local model");
          const double md=boost::math::cdf(boost::math::non_central_chi_squared_distribution<double>(detector.dof,lambda),detector.squared_threshold);
          const char* classification=gamma<1e-20?(shift>1e-10?"dangerous_null_direction":"position_harmless_first_order_null"):
            (required>cap?"power_vs_all_in_step_linear":"linear_power_candidate_only");
          // This cap concerns the all-in fit, not a theorem that every
          // exclusion candidate must fail its own independent step check.
          out<<(excited?"translation_rotation":"stationary")<<','<<axis<<','<<duration<<','<<blocks<<','<<gamma<<','<<lambda<<','<<amp<<','<<md<<','<<detector.squared_threshold<<','<<detector.dof<<','<<step.norm()<<','<<shift<<','<<required<<','<<cap<<','<<finite_parity.squaredNorm()<<','<<finite_relative<<','<<detector.squared_parity_statistic<<','<<ortho<<','<<classification<<'\n';
        }
      }
      auto plan=EpochCommitPlan::nominalPlan(tx);estimator.commitEpoch(std::move(tx),plan);
    }
  }
  std::cout<<"VERIFIED two six-axis snapshots; finite raw corruption; conditional power only, no recovery qualification\n";
  return 0;
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=3 && argc!=4)throw std::invalid_argument("usage: fde_research_power CONFIG OUTPUT_CSV [directions]");
    const auto original=IntegrityConfigLoader::load(argv[1]);
    if(argc==4) {if(std::string(argv[3])!="directions")throw std::invalid_argument("invalid mode");return directions(original,argv[2]);}
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
