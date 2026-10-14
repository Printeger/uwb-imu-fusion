#include "sc6_provider.hpp"
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
double middle(I x){return x.lower()/2+x.upper()/2;}
}
NavigationState Sc6Provider::truth(int index,bool uwb_time)const{
 NavigationState s;s.timestamp=TimestampNs(int64_t(index)*5000000LL);double t=uwb_time?(index/10)*.05:index*.005;
 if(recipe_.kind==0){s.position_world_m=recipe_.amp.cwiseProduct((recipe_.freq*t).array().sin().matrix())+Eigen::Vector3d(0,0,1);s.velocity_world_mps=recipe_.amp.cwiseProduct(recipe_.freq).cwiseProduct((recipe_.freq*t).array().cos().matrix());s.q_world_body=Eigen::Quaterniond(Eigen::AngleAxisd(.2*t,Eigen::Vector3d(1,2,3).normalized()));}
 else{
  if(recipe_.kind==1){int phase=index%2400;bool reverse=phase>=1200;int travel=reverse?phase-1200:phase;double u=std::min(travel,800)/800.;double q=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5),qp=(30*u*u-60*std::pow(u,3)+30*std::pow(u,4))/4.;if(travel>=800){q=1;qp=0;}s.position_world_m=Eigen::Vector3d(.8,.4,.2)*(reverse?1-q:q)+Eigen::Vector3d(0,0,1);s.velocity_world_mps=Eigen::Vector3d(.8,.4,.2)*(reverse?-qp:qp);}
  else{s.position_world_m=Eigen::Vector3d(.8*std::sin(.6*t),.6*std::sin(.5*t),1+.4*std::sin(.4*t));s.velocity_world_mps={.48*std::cos(.6*t),.3*std::cos(.5*t),.16*std::cos(.4*t)};}
  s.q_world_body=Eigen::Quaterniond(Eigen::AngleAxisd(.3*std::sin(.4*t),Eigen::Vector3d::UnitX()))*Eigen::Quaterniond(Eigen::AngleAxisd(.4*std::sin(.3*t),Eigen::Vector3d::UnitY()))*Eigen::Quaterniond(Eigen::AngleAxisd(.5*std::sin(.2*t),Eigen::Vector3d::UnitZ()));
 }
 if(!biases_.empty()){s.accel_bias_mps2=biases_.at(index).head<3>();s.gyro_bias_radps=biases_.at(index).tail<3>();}return s;
}
Sc6Provider::Sc6Provider(const std::string& path,const std::string& id,int seed,int count):law(researchLaw(path)),cfg(IntegrityConfigLoader::load(law["imu_base_config"].as<std::string>())){
 manifest=law["sc6"]["cases"][id];if(!manifest)throw std::invalid_argument("SC6_CASE_UNKNOWN");
 epochs=count?count:manifest["epochs"].as<int>();recipe_.kind=std::stoi(manifest["motion"].as<std::string>().substr(1));domain=sc6Domain(recipe_);lever=researchVector(law["lever"]);
 cfg.incremental.fixed_lag_epochs=law["fixed_lag_epochs"].as<int>();cfg.integrity_window.epochs=law["window_epochs"].as<int>();cfg.realtime.range_sigma_m=law["range_sigma_m"].as<double>();
 // The same frozen domain is consumed by the real bridge factory.
 if(recipe_.kind){cfg.bridge.generic.acceleration_bound_mps2=domain.acceleration;cfg.bridge.generic.angular_rate_bound_radps=domain.rate;cfg.bridge.generic.angular_acceleration_bound_radps2=domain.alpha;cfg.bridge.generic.calibration_id="sc6-D1-simulation-motion-v1";}
 std::string geometry=manifest["geometry"].as<std::string>();if(geometry!="G0")for(int a=0;a<8;++a){cfg.anchors[a].position_world_m={a&1?4.:-4.,a&2?4.:-4.,a&4?(geometry=="G1"?4.:.001):0.};}
 std::mt19937 bias_rng(seed+23497),initial_rng(seed+15401),imu_rng(seed),uwb_rng(seed+7919);std::normal_distribution<double> bn(0,1),in(0,1),normal(0,1);
 Eigen::Matrix<double,6,1> bias;bias.head<3>()=researchVector(law["true_initial_accel_bias_mps2"]);bias.tail<3>()=researchVector(law["true_initial_gyro_bias_radps"]);biases_.push_back(bias);
 for(int i=1;i<=epochs*10;++i){Eigen::Matrix<double,6,1> draw;for(int j=0;j<6;++j){draw(j)=bn(bias_rng);bias(j)+=(j<3?cfg.imu.accelerometer_bias_rw_sigma:cfg.imu.gyroscope_bias_rw_sigma)*std::sqrt(.005)*draw(j);}rw_.push_back(draw);biases_.push_back(bias);}
 initial=truth(0);Eigen::Matrix<double,15,1> eta;for(int j=0;j<15;++j){initial_draws_(j)=in(initial_rng);eta(j)=cfg.realtime.prior_sigmas(j)*initial_draws_(j);}auto pose=gtsam::Pose3(gtsam::Rot3(initial.q_world_body.toRotationMatrix()),initial.position_world_m).retract(eta.head<6>());initial.position_world_m=pose.translation();initial.q_world_body=Eigen::Quaterniond(pose.rotation().matrix());initial.velocity_world_mps+=eta.segment<3>(6);initial.accel_bias_mps2+=eta.segment<3>(9);initial.gyro_bias_radps+=eta.tail<3>();
 for(int i=0;i<=epochs*10;++i){auto s=truth(i);auto m=sample(cfg,i);double t=i*.005;Eigen::Vector3d acc,omega;
  if(recipe_.kind==0){acc=-recipe_.amp.cwiseProduct(recipe_.freq.cwiseProduct(recipe_.freq)).cwiseProduct((recipe_.freq*t).array().sin().matrix());omega=.2*Eigen::Vector3d(1,2,3).normalized();}
  else if(recipe_.kind==1){int phase=i%2400;bool rev=phase>=1200;int travel=rev?phase-1200:phase;double u=std::min(travel,800)/800.;double qpp=travel>=800?0:(60*u-180*u*u+120*u*u*u)/16.;acc=Eigen::Vector3d(.8,.4,.2)*(rev?-qpp:qpp);}
  else acc={-.288*std::sin(.6*t),-.15*std::sin(.5*t),-.064*std::sin(.4*t)};
  if(recipe_.kind){Eigen::Matrix3d ry=Eigen::AngleAxisd(.4*std::sin(.3*t),Eigen::Vector3d::UnitY()).toRotationMatrix(),rz=Eigen::AngleAxisd(.5*std::sin(.2*t),Eigen::Vector3d::UnitZ()).toRotationMatrix();omega=rz.transpose()*ry.transpose()*Eigen::Vector3d(.12*std::cos(.4*t),0,0)+rz.transpose()*Eigen::Vector3d(0,.12*std::cos(.3*t),0)+Eigen::Vector3d(0,0,.10*std::cos(.2*t));}
  m.specific_force_mps2=s.q_world_body.conjugate()*(acc+Eigen::Vector3d(0,0,cfg.imu.gravity_mps2));m.angular_velocity_radps=omega;m.specific_force_mps2+=s.accel_bias_mps2;m.angular_velocity_radps+=s.gyro_bias_radps;
  Eigen::Matrix<double,6,1> draw,fault=Eigen::Matrix<double,6,1>::Zero();for(int j=0;j<3;++j){draw(j)=normal(imu_rng);draw(j+3)=normal(imu_rng);m.specific_force_mps2(j)+=law["raw_accel_sigma_mps2"].as<double>()*draw(j);m.angular_velocity_radps(j)+=law["raw_gyro_sigma_radps"].as<double>()*draw(j+3);}
  for(auto f:manifest["faults"]){int onset=f[0].as<int>(),axis=f[1].as<int>();int64_t begin=(onset-1)*50000000LL+5000000LL;if(m.timestamp.value()>=begin && m.timestamp.value()<begin+f[2].as<int>()*1000000LL){fault(axis)+=f[3].as<double>();if(axis<3)m.specific_force_mps2(axis)+=f[3].as<double>();else m.angular_velocity_radps(axis-3)+=f[3].as<double>();}}
  adc_.push_back(draw);injections_.push_back(fault);imu.push_back(m);
 }
 for(int e=1;e<=epochs;++e){auto b=ranges(cfg,e);auto s=truth(e*10,true);Eigen::Vector3d tag=s.position_world_m+s.q_world_body*lever;std::array<double,8> draws{},faults{};for(int a=0;a<8;++a){auto& m=b.measurements[a];draws[a]=normal(uwb_rng);m.range_m=(m.anchor_position_m-tag).norm()+cfg.realtime.range_sigma_m*draws[a];}for(auto f:manifest["faults"]){int onset=f[4].as<int>();if(e>=onset && e<onset+3){faults[0]+=2.25;b.measurements[0].range_m+=2.25;}}range_draws_.push_back(draws);range_faults_.push_back(faults);uwb.push_back(b);}
}
Sc6Envelope Sc6Provider::certify(size_t epoch){
 // Privileged draw-based qualification is computed only for the arrived prefix,
 // and its actual cost belongs to service. It conveys no fault identity.
 Sc6Envelope out;out.epoch=epoch;int begin=int(epoch-1)*10,end=int(epoch)*10;
 out.imu.assign(imu.begin()+begin+1,imu.begin()+end+1);out.uwb=uwb.at(epoch-1);bool valid=true;
 if(epoch==1){Eigen::Vector3d amp=recipe_.kind==1?Eigen::Vector3d::Zero():recipe_.kind?Eigen::Vector3d(.8,.6,.4):recipe_.amp;Eigen::Vector3d freq=recipe_.kind==1?Eigen::Vector3d::Zero():recipe_.kind?Eigen::Vector3d(.6,.5,.4):recipe_.freq;
  auto fake=sample(cfg,0);fake.specific_force_mps2+=biases_[0].head<3>();fake.angular_velocity_radps=biases_[0].tail<3>();
  auto init=certifyRawGenerator(initial,initial_draws_,cfg.realtime.prior_sigmas,amp,freq,0,cfg.imu.gravity_mps2,biases_[0],cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),{}, {Eigen::Matrix<double,6,1>::Zero()},{fake});valid&=init.valid;
 }
 for(int i=begin;i<=end;++i){auto motion=sc6MotionAt(i,recipe_);std::array<I,6> b;for(int j=0;j<6;++j){b[j]=I(biases_[begin](j))+I(-carry_,carry_);for(int k=begin+1;k<=i;++k)b[j]+=I(j<3?cfg.imu.accelerometer_bias_rw_sigma:cfg.imu.gyroscope_bias_rw_sigma)*sqrt(I(1)/200.)*I(rw_[k-1](j));}
  for(int j=0;j<3;++j){I force(0);for(int k=0;k<3;++k)force+=motion.r[k][j]*(motion.a[k]+I(k==2?cfg.imu.gravity_mps2:0));I predicted=force+b[j]+I(law["raw_accel_sigma_mps2"].as<double>())*I(adc_[i](j))+I(injections_[i](j));I gyro=motion.omega[j]+b[j+3]+I(law["raw_gyro_sigma_radps"].as<double>())*I(adc_[i](j+3))+I(injections_[i](j+3));valid&=abs(I(imu[i].specific_force_mps2(j))-predicted).upper()<=2e-10 && abs(I(imu[i].angular_velocity_radps(j))-gyro).upper()<=2e-10;}
  if(i==end){double next=0;for(int j=0;j<6;++j)next=std::max(next,abs(I(biases_[end](j))-b[j]).upper());valid&=next<=1e-10;carry_=next;}
 }
 out.range=recipe_.kind==0?certifyRawRanges(out.uwb,recipe_.amp,recipe_.freq,.2,lever,range_draws_.at(epoch-1),range_faults_.at(epoch-1)):certifySc6Ranges(out.uwb,sc6MotionAt(end,recipe_),lever,range_draws_.at(epoch-1),range_faults_.at(epoch-1));out.source_valid=valid && out.range.valid;return out;
}
}
