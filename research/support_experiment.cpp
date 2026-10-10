#include "scenario_material.hpp"
#include "quadratic_power.hpp"
#include "uwb_imu_pl/integrity/history_fault_summary.hpp"
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
// Independent oracle changes each raw sample first, then uses an unmasked
// production-style trapezoid loop. It never calls integrateEpisode.
gtsam::PreintegratedCombinedMeasurements oracle(const EpochTransaction& tx,const RawImuEpisode& e,double amplitude) {
  auto samples=tx.raw_imu_slice;
  for(auto& s:samples)if(!(s.timestamp<e.begin) && s.timestamp<e.end) {
    if(e.axis<3)s.specific_force_mps2(e.axis)+=amplitude;
    else s.angular_velocity_radps(e.axis-3)+=amplitude;
  }
  auto pim=*tx.preintegration;pim.resetIntegration();
  for(std::size_t i=1;i<samples.size();++i)
    pim.integrateMeasurement(.5*(samples[i-1].specific_force_mps2+samples[i].specific_force_mps2),
        .5*(samples[i-1].angular_velocity_radps+samples[i].angular_velocity_radps),
        samples[i].timestamp.seconds()-samples[i-1].timestamp.seconds());
  return pim;
}
int main(int argc,char** argv) {
  try {
    if(argc!=3)throw std::invalid_argument("usage: fde_research_support CONFIG OUTPUT_CSV");
    const auto cfg=IntegrityConfigLoader::load(argv[1]);
    std::ofstream out(argv[2]);require(bool(out),"output unavailable");
    out<<std::setprecision(17)<<"motion,axis,epoch,samples,trapezoids,raw_dp_norm,raw_dv_norm,frozen_norm,dependent_norm,covariance_derivative_norm,convergence,oracle_relative,history_identity_relative\n";
    int checks=0;
    for(bool rotating:{false,true}) {
      IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
      estimator.initialize(initialState(),cfg.realtime.prior_sigmas);estimator.ingestImu(sample(cfg,0,rotating));
      auto first=prepare(estimator,cfg,1,rotating);
      const auto first_plan=EpochCommitPlan::nominalPlan(first);
      estimator.commitEpoch(std::move(first),first_plan);
      auto second=prepare(estimator,cfg,2,rotating);
      require(!second.recoverable_history.empty(),"historical raw material missing");
      auto historical=historicalEpisodeTransaction(second.recoverable_history.back());
      for(int axis=0;axis<6;++axis) {
        RawImuEpisode e{"raw-batch-v1",TimestampNs(5000000),TimestampNs(55000000),axis};
        for(const auto* tx:{&historical,&second}) {
          // Historical rows are reconstructed at their recorded frozen states.
          LinearizedFactorBlock block;
          block.whitener=gtsam::noiseModel::Gaussian::Covariance(tx->preintegration->preintMeasCov())->R();
          const auto r=episodeResponse(*tx,block,e);
          constexpr double epsilon=2e-5;
          const Eigen::VectorXd independent=-(episodeFactorError(*tx,oracle(*tx,e,epsilon))-
              episodeFactorError(*tx,oracle(*tx,e,-epsilon)))/(2*epsilon);
          const double difference=(independent-r.raw).norm()/std::max(1e-12,independent.norm());
          require(difference<2e-6,"independent reintegration response mismatch");
          require(r.convergence_relative<2e-6,"FD step convergence unresolved");
          require(r.affected_samples==(tx==&historical?10u:1u),"shared sample support lost");
          require(r.affected_trapezoids==(tx==&historical?10u:1u),"shared trapezoid support lost");
          // Real H/z, same fault parameter carried through orthogonal history
          // elimination. Verify minimum-cost identity against independent QR.
          IntegrityWindowRequest request;request.epochs=2;
          const auto w=estimator.buildIntegrityWindow(second,request);
          Eigen::VectorXd response=Eigen::VectorXd::Zero(w.H.rows());
          int offset=0;bool found=false;
          for(const auto& b:w.blocks) {
            if(b.kind==FactorKind::CombinedImu && b.jacobian_raw.middleCols((tx->proposed_epoch)*15,15).norm()>0.) {
              response.segment(offset,15)=b.whitener*r.raw;found=true;break;
            }
            offset+=b.residual_whitened.size();
          }
          require(found,"factor response lineage missing");
          HistoryFaultSummaryInput input{w.H.leftCols(15),w.H.rightCols(w.H.cols()-15),response,w.z};
          const auto summary=buildHistoryFaultSummary(input);
          require(summary.valid,"history orthogonal mapping failed");
          const Eigen::VectorXd xb=Eigen::VectorXd::Constant(input.h_boundary.cols(),.003);
          const double amplitude=.02;
          const Eigen::VectorXd rhs=input.rhs-input.h_boundary*xb-input.fault_map*Eigen::VectorXd::Constant(1,amplitude);
          const Eigen::VectorXd xo=input.h_old_state.colPivHouseholderQr().solve(rhs);
          const double full=(input.h_old_state*xo-rhs).squaredNorm();
          const double reduced=(summary.R_b*xb+summary.T_b*Eigen::VectorXd::Constant(1,amplitude)-summary.d_b).squaredNorm()+
              (summary.F_b*Eigen::VectorXd::Constant(1,amplitude)-summary.d_perp).squaredNorm();
          const double identity=std::abs(full-reduced)/std::max(1.,full);
          require(identity<1e-9,"history cost/parameter identity failed");
          out<<rotating<<','<<axis<<','<<tx->proposed_epoch<<','<<r.affected_samples<<','<<r.affected_trapezoids<<','
              <<r.raw.segment<3>(3).norm()<<','<<r.raw.segment<3>(6).norm()<<','<<r.frozen_whitened.norm()<<','
              <<r.input_dependent_whitened.norm()<<','<<r.covariance_derivative.norm()<<','<<r.convergence_relative<<','<<difference<<','<<identity<<'\n';
          checks+=5;
        }
      }
      // Compose BOTH adjacent factors under one physical parameter before
      // history elimination. This preserves cross terms absent from two
      // independently charged interval faults.
      IntegrityWindowRequest request;request.epochs=2;
      const auto window=estimator.buildIntegrityWindow(second,request);
      for(int axis=0;axis<6;++axis) {
        RawImuEpisode episode{"shared-episode",TimestampNs(5000000),TimestampNs(55000000),axis};
        Eigen::VectorXd joined=Eigen::VectorXd::Zero(window.H.rows());int offset=0,affected=0;
        for(const auto& block:window.blocks) {
          if(block.kind==FactorKind::CombinedImu) {
            const auto& material=block.jacobian_raw.middleCols(30,15).norm()>0.?second:historical;
            const auto r=episodeResponse(material,block,episode);
            joined.segment(offset,15)=r.frozen_whitened;affected+=r.affected_trapezoids>0;
          }
          offset+=block.residual_whitened.size();
        }
        require(affected==2,"joint boundary episode missing adjacent factor");
        HistoryFaultSummaryInput input{window.H.leftCols(15),window.H.rightCols(window.H.cols()-15),joined,window.z};
        const auto summary=buildHistoryFaultSummary(input);require(summary.valid,"shared history map invalid");
        const auto xb=Eigen::VectorXd::Constant(input.h_boundary.cols(),.013);
        const Eigen::VectorXd rhs=input.rhs-input.h_boundary*xb-input.fault_map*Eigen::VectorXd::Constant(1,.07);
        const double original=(input.h_old_state*input.h_old_state.colPivHouseholderQr().solve(rhs)-rhs).squaredNorm();
        const double mapped=(summary.R_b*xb+summary.T_b*Eigen::VectorXd::Constant(1,.07)-summary.d_b).squaredNorm()+
            (summary.F_b*Eigen::VectorXd::Constant(1,.07)-summary.d_perp).squaredNorm();
        require(std::abs(original-mapped)<1e-9*std::max(1.,original),"shared raw history identity failed");checks+=3;
      }
      // Nonzero biasHat, endpoint acceleration/gyro biases and bias drift:
      // verify actual Combined bias correction, not an interval-scaled J_bias.
      auto biased=second;auto values=std::make_shared<gtsam::Values>(*second.frozen_values);
      const gtsam::imuBias::ConstantBias left_bias(Eigen::Vector3d(.1,-.2,.3),Eigen::Vector3d(.01,-.02,.03));
      const gtsam::imuBias::ConstantBias right_bias(Eigen::Vector3d(.12,-.19,.29),Eigen::Vector3d(.012,-.018,.029));
      values->update(gtsam::Symbol('b',second.previous_epoch),left_bias);
      values->update(gtsam::Symbol('b',second.proposed_epoch),right_bias);biased.frozen_values=values;
      auto biased_pim=std::make_shared<gtsam::PreintegratedCombinedMeasurements>(*second.preintegration);
      biased_pim->resetIntegrationAndSetBias(left_bias);biased.preintegration=biased_pim;
      LinearizedFactorBlock biased_block;biased_block.whitener=Eigen::MatrixXd::Identity(15,15);
      for(int axis:{0,5}) {
        RawImuEpisode episode{"nonzero-bias",TimestampNs(55000000),TimestampNs(105000000),axis};
        const auto response=episodeResponse(biased,biased_block,episode);
        const auto fast=episodeResponse(biased,biased_block,episode,1e-5,true);
        const Eigen::VectorXd independent=-(episodeFactorError(biased,oracle(biased,episode,2e-5))-episodeFactorError(biased,oracle(biased,episode,-2e-5)))/4e-5;
        require((independent-response.raw).norm()<2e-6*std::max(1.,response.raw.norm()),"nonzero-bias oracle failed");
        require((fast.raw-response.raw).norm()==0. && (fast.input_dependent_whitened-response.input_dependent_whitened).norm()==0.,"sensitive fallback changed numerics");checks+=2;
      }
      // Independent raw-sample noise at the shared boundary induces a
      // CROSS-factor covariance. This is separate from each PIM's propagated
      // covariance derivative and is not silently absorbed by diagonal Q.
      Eigen::MatrixXd joint_jacobian=Eigen::MatrixXd::Zero(30,6);
      LinearizedFactorBlock identity;identity.whitener=Eigen::MatrixXd::Identity(15,15);
      for(int axis=0;axis<6;++axis) {
        RawImuEpisode boundary{"boundary-noise",TimestampNs(50000000),TimestampNs(50000001),axis};
        joint_jacobian.col(axis).head(15)=episodeResponse(historical,identity,boundary).raw;
        joint_jacobian.col(axis).tail(15)=episodeResponse(second,identity,boundary).raw;
      }
      // Declared research noise law, not a hardware calibration or production Q.
      Eigen::Matrix<double,6,6> raw_cov=Eigen::Matrix<double,6,6>::Identity();
      raw_cov.topLeftCorner<3,3>()*=.02*.02;raw_cov.bottomRightCorner<3,3>()*=.001*.001;
      const Eigen::MatrixXd joint_cov=joint_jacobian*raw_cov*joint_jacobian.transpose();
      require(joint_cov.topRightCorner(15,15).norm()>1e-12,"shared noise cross covariance lost");
      require(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(joint_cov).eigenvalues().minCoeff()>-1e-12,"shared covariance not PSD");
      Eigen::MatrixXd noise_columns=Eigen::MatrixXd::Zero(window.H.rows(),6);int noise_offset=0;
      for(const auto& block:window.blocks) {
        if(block.kind==FactorKind::CombinedImu) {
          const bool current=block.jacobian_raw.middleCols(30,15).norm()>0.;
          noise_columns.middleRows(noise_offset,15)=block.whitener*joint_jacobian.middleRows(current?15:0,15);
        }
        noise_offset+=block.residual_whitened.size();
      }
      const Eigen::MatrixXd omega=Eigen::MatrixXd::Identity(window.H.rows(),window.H.rows())+noise_columns*raw_cov*noise_columns.transpose();
      DetectorRiskContext detector_risk;detector_risk.p_fa_per_test=cfg.detector.p_fa_per_test;detector_risk.continuity_horizon_tests=cfg.detector.continuity_horizon_tests;
      const auto detector=JointWindowDetector{}.evaluate(window,detector_risk);
      const auto independent_power=quadraticPower(window.H,20*noise_columns.col(0),Eigen::MatrixXd::Identity(window.H.rows(),window.H.rows()),detector.squared_threshold);
      const auto coupled_power=quadraticPower(window.H,20*noise_columns.col(0),omega,detector.squared_threshold);
      require(independent_power.standard_noncentral_chi_squared,"independent quadratic premise wrong");
      require(!coupled_power.standard_noncentral_chi_squared,"shared noise incorrectly called nc-chi-square");
      require(coupled_power.mean>=independent_power.mean && coupled_power.variance>=independent_power.variance,"PSD noise moments lost");checks+=3;
      std::cout<<"motion="<<rotating<<" generalized_mean="<<coupled_power.mean<<" generalized_variance="<<coupled_power.variance<<" Cantelli_md_interval=["<<coupled_power.miss_lower<<','<<coupled_power.miss_upper<<"]\n";
      std::cout<<"motion="<<rotating<<" shared_sample_cross_covariance_norm="<<std::setprecision(17)<<joint_cov.topRightCorner(15,15).norm()<<'\n';checks+=2;
      // Missing/wrong support is rejected or exactly zero, never fabricated.
      const RawImuEpisode absent{"absent",TimestampNs(1000000000),TimestampNs(1100000000),0};
      LinearizedFactorBlock b;b.whitener=Eigen::MatrixXd::Identity(15,15);
      const auto slow=episodeResponse(second,b,absent),fast=episodeResponse(second,b,absent,1e-5,true);
      require(slow.raw.norm()==0. && fast.raw.norm()==0.,"absent support not zero");
      require((slow.covariance_derivative-fast.covariance_derivative).norm()==0.,"empty covariance mismatch");
      require(fast.integrations==1 && slow.integrations==4,"empty-support work count wrong");checks+=3;
      auto invalid=second;invalid.raw_imu_slice.front().specific_force_mps2.x()=std::numeric_limits<double>::quiet_NaN();
      for(bool optimized:{false,true}) {
        bool rejected=false;try{episodeResponse(invalid,b,absent,1e-5,optimized);}catch(const std::exception&){rejected=true;}
        require(rejected,"empty-support invalid material exception suppressed");++checks;
      }
    }
    std::cout<<"VERIFIED research support checks="<<checks<<"; production publication_protected=false\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
