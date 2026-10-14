#pragma once
#include "scenario_material.hpp"
#include "sc6_motion.hpp"
namespace uwb_imu_pl::research {
struct Sc6Envelope {size_t epoch;std::vector<ImuMeasurement> imu;UwbBatch uwb;RawRangeLawEvidence range;bool source_valid=false;};
// Online service receives only this envelope, domain and original law/config.
class Sc6Provider {
 Sc6MotionRecipe recipe_;
 std::vector<Eigen::Matrix<double,6,1>> biases_,rw_,adc_,injections_;
 std::vector<std::array<double,8>> range_draws_,range_faults_;
 Eigen::Matrix<double,15,1> initial_draws_;
 double carry_=0;
 public:
 YAML::Node law,manifest;IntegrityConfig cfg;Eigen::Vector3d lever;
 Sc6MotionDomain domain;NavigationState initial;
 std::vector<ImuMeasurement> imu;std::vector<UwbBatch> uwb;
 int epochs;
 Sc6Provider(const std::string& path,const std::string& id,int seed,int count=0);
 NavigationState truth(int index,bool uwb_time=false)const;
 Sc6Envelope certify(size_t epoch); // privileged simulation adapter, timed online
 const Sc6MotionRecipe& recipe()const{return recipe_;}
};
}
