#pragma once
#include "uwb_imu_pl/estimation/epoch_transaction.hpp"
#include <Eigen/Core>
#include <memory>
namespace uwb_imu_pl::research {
// This reference contains continuous physical truth, not a fitted chart.
// It consumes no truth state, fault labels, optimizer convergence or AL.
struct RawInputRoundoff {
 double position=0,rotation_matrix=0,accel_sample=0,gyro_sample=0;
 Eigen::Vector3d velocity=Eigen::Vector3d::Zero();Eigen::Matrix<double,6,1> bias=Eigen::Matrix<double,6,1>::Zero();bool valid=false;
};
RawInputRoundoff certifyRawGenerator(const NavigationState&,const Eigen::Matrix<double,15,1>& initial_normals,
 const Eigen::Matrix<double,15,1>& sigmas,const Eigen::Vector3d& amplitude,const Eigen::Vector3d& frequency,
 double rate,double gravity,const Eigen::Matrix<double,6,1>& initial_bias,double accel_rw,double gyro_rw,double accel_adc,double gyro_adc,
 const std::vector<Eigen::Matrix<double,6,1>>& bias_normals,const std::vector<Eigen::Matrix<double,6,1>>& adc_normals,const std::vector<ImuMeasurement>& samples);
struct RawInclusionParameters {
 RawInputRoundoff numeric;
 Eigen::Matrix<double,15,1> initial_sigmas;
 double accel_adc=.02,gyro_adc=.001,accel_rw=.001,gyro_rw=.0001;
 Eigen::Vector3d motion_amplitude{.1,.08,.05},motion_frequency{.5,.4,.3};double motion_angular_rate=.2;
 double force_lipschitz=2.1,gyro_lipschitz=0,gravity=9.81;
 double risk_budget=1e-6;int horizon=50,coordinates=6421;
};
struct RawInclusionEvidence {
 Eigen::Vector3d low=Eigen::Vector3d::Constant(NAN),high=low,reference=low,pl=low,initial=low;
 double adc=0,initial_bias=0,bias_rw=0,rotation_discretization=0,lever=0;
 double hpl=INFINITY,event_risk=INFINITY,rotation_diameter=INFINITY;
 TransactionId transaction_id;uint64_t graph_version=0;TimestampNs timestamp;
 Eigen::Vector3d committed_tag=Eigen::Vector3d::Constant(NAN),committed_lever=committed_tag;
 int epoch=0,horizon=50;std::string source_law_id="raw-source-rw-v1",scope="healthy-IMU-single-UWB-episode-50-v1";bool valid=false,receipt_bound=false;
};
class RawNonlinearInclusion {
 struct Impl;std::unique_ptr<Impl> impl;
 public:
 RawNonlinearInclusion(const NavigationState&,const RawInclusionParameters&);
 ~RawNonlinearInclusion();
 void append(const ImuMeasurement&); // immutable prefix, shared sample once
 RawInclusionEvidence tag(const Eigen::Vector3d& lever,const Eigen::Vector3d& actual)const;
 RawInclusionEvidence issue(const EpochTransaction&,const CommitReceipt&,const NavigationState&,const Eigen::Vector3d&)const;
};
}
