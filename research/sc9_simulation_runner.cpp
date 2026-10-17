#include "unified_rolling_core.hpp"
#include "measurement_bootstrap.hpp"
#include "numerical_candidate.hpp"
#include "sc6_provider.hpp"
#include "uwb_imu_pl/common/runtime_route_audit.hpp"
#include "sc9_paced_replay.hpp"
#include <tbb/global_control.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
namespace {
void require(bool b,const char* what){if(!b)throw std::runtime_error(what);}
double mid(I x){return x.lower()/2+x.upper()/2;}
constexpr int64_t origin=1664959757027770002LL;
struct Truth {Eigen::Vector3d p,v,a,w;Eigen::Matrix3d r;};
Truth analytic(int64_t ns){
 const double t=ns/1e9;Truth x;x.p={.8*std::sin(.6*t),.6*std::sin(.5*t),1+.4*std::sin(.4*t)};
 x.v={.48*std::cos(.6*t),.3*std::cos(.5*t),.16*std::cos(.4*t)};
 x.a={-.288*std::sin(.6*t),-.15*std::sin(.5*t),-.064*std::sin(.4*t)};
 const auto rx=Eigen::AngleAxisd(.3*std::sin(.4*t),Eigen::Vector3d::UnitX()).toRotationMatrix();
 const auto ry=Eigen::AngleAxisd(.4*std::sin(.3*t),Eigen::Vector3d::UnitY()).toRotationMatrix();
 const auto rz=Eigen::AngleAxisd(.5*std::sin(.2*t),Eigen::Vector3d::UnitZ()).toRotationMatrix();
 x.r=rx*ry*rz;x.w=(ry*rz).transpose()*Eigen::Vector3d(.12*std::cos(.4*t),0,0)+rz.transpose()*Eigen::Vector3d(0,.12*std::cos(.3*t),0)+Eigen::Vector3d(0,0,.1*std::cos(.2*t));return x;
}
struct Draw {SourceCoordinate key;double z;I scale=I(0);};
struct Generator {
 JointRawLaw law;Eigen::Vector3d lever;std::mt19937 ar{20261017},br{20284514},rr{20268936};std::normal_distribution<double> normal;
 std::array<I,6> bias,bias_support;std::vector<Draw> draws;uint64_t raw_count=0,range_count=0;int64_t last_ns=0;
 double residue=0;std::vector<Eigen::Vector3d> anchors;
 Generator(const JointRawLaw& l,Eigen::Vector3d v,bool nondegenerate_five=false):law(l),lever(v){for(auto& b:bias)b=I(0);for(auto& b:bias_support)b=I(0);
  anchors={{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
  if(nondegenerate_five)anchors[4].z()=4;
  for(size_t k=5;k<12;++k)anchors.push_back({4*std::cos(double(k)),4*std::sin(double(k)),double(k%3)*2});
 }
 AnchorId aid(size_t k)const{return AnchorId((uint64_t(1)<<49)+113*k);}
 void witness(double observed,I exact){const double e=abs(I(observed)-exact).upper();residue=std::max(residue,e);require(std::isfinite(e)&&e<=1e-8,"SIM_NUMERICAL_SOURCE_WITNESS_FAILED");}
 ImuMeasurement imu(int64_t ns,int axis,double fault){
  const auto exact=sc6D1MotionAtNanoseconds(ns);const auto x=analytic(ns);ImuMeasurement m;m.id=MeasurementId((uint64_t(1)<<42)+ ++raw_count);m.timestamp=TimestampNs(origin+ns);
  for(int j=0;j<6;++j){if(raw_count>1){const double z=normal(br);const I scale=I(j<3?law.accel_rw:law.gyro_rw)*.5*sqrt(I(double(ns-last_ns))/1e9);bias[j]+=scale*I(z);draws.push_back({{SourceKind::BiasIncrement,0,m.id.value(),unsigned(j)},z,scale});}}
  const Eigen::Vector3d f=x.r.transpose()*(x.a+Eigen::Vector3d(0,0,law.gravity));
  for(int j=0;j<6;++j){const double z=normal(ar),sigma=j<3?law.accel_adc:law.gyro_adc;draws.push_back({{SourceKind::Adc,0,m.id.value(),unsigned(j)},z});
   I ideal=j<3?I(0):exact.omega[j-3];if(j<3)for(int k=0;k<3;++k)ideal+=exact.r[k][j]*(exact.a[k]+I(k==2?law.gravity:0));
   const double injected=j==axis?fault:0;const double observed=(j<3?f[j]:x.w[j-3])+mid(bias[j])+.5*sigma*z+injected;
   witness(observed,ideal+bias[j]+I(.5*sigma)*I(z)+I(injected));
   if(j<3)m.specific_force_mps2[j]=observed;else m.angular_velocity_radps[j-3]=observed;
  }
  last_ns=ns;return m;
 }
 CanonicalRangeEvent range(size_t epoch,size_t k,int64_t ns,bool valid,double fault){
  auto x=analytic(ns);auto exact=sc6D1MotionAtNanoseconds(ns);UwbMeasurement m;m.id=MeasurementId((uint64_t(1)<<60)+ ++range_count);m.factor_id=FactorId(m.id.value());m.anchor_id=aid(k);m.timestamp=TimestampNs(origin+ns);m.anchor_position_m=anchors[k];m.sigma_m=.02;m.lever_arm_body_m=lever;m.sequence=epoch;
  const double z=normal(rr);draws.push_back({{SourceKind::Range,m.anchor_id.value(),m.id.value(),0},z});
  m.range_m=(x.p+x.r*lever-anchors[k]).norm()+.01*z+fault;I sum(0);
  for(int j=0;j<3;++j){I d=exact.p[j]-I(anchors[k][j]);for(int c=0;c<3;++c)d+=exact.r[j][c]*I(lever[c]);sum+=square(d);}
  witness(m.range_m,sqrt(sum)+I(.01)*I(z)+I(fault));return {m,TimestampNs(origin+ns),1,valid};
 }
 bool audit(const GeneralizedSourceLedger& ledger){bool inside=true;
  for(const auto& d:draws){const double q=ledger.event().birth(d.key).q;require(q>=2,"SIM_PROXY_DOMINATION_Q_DOMAIN");inside&=std::abs(d.z)<=q;
   if(d.key.kind==SourceKind::BiasIncrement)bias_support[d.key.axis]+=I(q)*d.scale;
   const double proxy=d.key.kind==SourceKind::Range?.02:d.key.kind==SourceKind::Adc?(d.key.axis<3?law.accel_adc:law.gyro_adc):0;
   if(proxy)require(.5*proxy*q+residue<=proxy*q,"SIM_PROXY_NUMERICAL_DOMINATION_FAILED");
  }draws.clear();return inside;
 }
};
}
int main(int argc,char** argv){try{
 if(argc<4 || argc>9)throw std::invalid_argument("usage: sc9_simulation_runner COUNT healthy|U|A|G|J|repeated OUTPUT [FGO_LAG] [INTEGRITY_WINDOW] [EXOGENOUS_SCHEDULE|-] [original|nondegenerate5] [paced]");
 const size_t count=std::stoull(argv[1]);const std::string fault=argv[2];require(count>0&&count<=10000,"SIM_COUNT_DOMAIN");
 require(fault=="healthy"||fault=="U"||fault=="A"||fault=="G"||fault=="J"||fault=="repeated","SIM_FAULT_DOMAIN");
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 YAML::Node schedule(YAML::NodeType::Undefined);std::vector<int64_t> imu_times;size_t batch_cursor=0,imu_cursor=0,n=12;int64_t initial_ns=500000000;
 const std::string geometry=argc>=8?argv[7]:"original";require(geometry=="original"||geometry=="nondegenerate5","SIM_GEOMETRY_ID");
 const bool paced=argc==9;require(!paced||std::string(argv[8])=="paced","SIM_PACING_MODE");
 if(argc>=7&&std::string(argv[6])!="-"){schedule=YAML::LoadFile(argv[6]);require(schedule["schema"].as<std::string>()=="sc9-exogenous-schedule-v1","SIM_SCHEDULE_SCHEMA");
  n=schedule["source_anchor_ids"].size();require(n>=4&&n<=12,"SIM_PROVIDER_GEOMETRY_SCOPE");
  imu_times=schedule["imu_ns"].as<std::vector<int64_t>>();require(!imu_times.empty(),"SIM_EMPTY_IMU");
  for(size_t i=1;i<imu_times.size();++i)require(imu_times[i]>imu_times[i-1]&&imu_times[i]-imu_times[i-1]<=50000000,"SIM_EXTERNAL_IMU_GAP");
  const auto batches=schedule["batches"];bool found=false;
  for(;batch_cursor<batches.size();++batch_cursor){size_t visible=0;for(const auto& r:batches[batch_cursor]["ranges"])visible+=r["valid"].as<bool>();
   if(batches[batch_cursor]["end_ns"].as<int64_t>()>=500000000&&visible>=4){found=true;break;}}
  require(found,"SIM_EXTERNAL_BOOTSTRAP_MISSING");const auto end=batches[batch_cursor]["end_ns"].as<int64_t>();
  const auto it=std::lower_bound(imu_times.begin(),imu_times.end(),end);require(it!=imu_times.end(),"SIM_EXTERNAL_BOOTSTRAP_IMU_MISSING");initial_ns=*it;
  require(batch_cursor+count<batches.size(),"SIM_EXTERNAL_SCHEDULE_TOO_SHORT");
 }
 require(geometry=="original"||(schedule&&n==5),"SIM_FIVE_ANCHOR_FIXTURE_SCOPE");
 Sc6Provider base("config/sc7.yaml","S0",20261010,1);auto cfg=base.cfg;auto root=YAML::Clone(base.law);cfg.anchors.clear();cfg.imu.max_gap_s=.025;cfg.incremental.max_time_skew_s=.02;cfg.realtime.prior_sigmas.setConstant(.1);
 if(schedule)cfg.imu.max_gap_s=.05;
 const int lag=argc>4?std::stoi(argv[4]):30,window=argc>5?std::stoi(argv[5]):lag;root["fixed_lag_epochs"]=lag;
 if(window!=lag){root["rolling_integrity"]["window_mode"]="explicit";root["rolling_integrity"]["explicit_epochs"]=window;}
 const auto policy=resolveRollingPolicy(root,cfg);Sc6MotionDomain domain;domain.id="D1-continuous-ns-v1";domain.acceleration=1;domain.rate=1;domain.jerk=2;domain.alpha=2;domain.global_speed=1;
 JointRawLaw law;law.gravity=cfg.imu.gravity_mps2;law.acceleration=1;law.rate=1;law.jerk=2;law.global_speed=1;
 cfg.imu.accelerometer_sigma=law.accel_adc;cfg.imu.gyroscope_sigma=law.gyro_adc;cfg.imu.accelerometer_bias_rw_sigma=law.accel_rw;cfg.imu.gyroscope_bias_rw_sigma=law.gyro_rw;
 cfg.bridge.generic.acceleration_bound_mps2=1;cfg.bridge.generic.angular_rate_bound_radps=1;cfg.bridge.generic.angular_acceleration_bound_radps2=2;cfg.bridge.generic.calibration_id=domain.id;
 Generator generator(law,base.lever,geometry=="nondegenerate5");GeneralizedCoreInitialization model;model.raw_law=law;
 model.sources=std::make_unique<GeneralizedSourceLedger>(policy.budget,law,ModelEvidenceStatus::VerifiedForDeclaredModel);
 auto* sources=model.sources.get();ImuMeasurement last;
 if(schedule){while(imu_cursor<imu_times.size()&&imu_times[imu_cursor]<=initial_ns){last=generator.imu(imu_times[imu_cursor++],-1,0);sources->append(last);}}
 else for(int64_t ns=0;ns<=initial_ns;ns+=10000000){last=generator.imu(ns,-1,0);sources->append(last);}
 for(size_t k=0;k<n;++k){AnchorRecord a;a.id=generator.aid(k);a.position_world_m=generator.anchors[k];a.covariance_m2.setZero();a.frame="world";a.map_version=geometry=="original"?"sc9-synthetic-dynamic-v1":"sc9-synthetic-nondegenerate5-v1";model.anchors.declare(a,1);}
 UwbBatch startup;startup.id=BatchId(1);startup.timestamp=last.timestamp;
 if(schedule){for(size_t b=0;b<=batch_cursor;++b)for(const auto& row:schedule["batches"][b]["ranges"]){const auto k=row["anchor"].as<size_t>();require(k<n,"SIM_EXTERNAL_ANCHOR");
   auto e=generator.range(0,k,row["stamp_ns"].as<int64_t>(),row["valid"].as<bool>(),0);sources->registerPotentialRange(e);if(b==batch_cursor&&e.valid)startup.measurements.push_back(e.measurement);}
  ++batch_cursor;
 }else for(size_t k=0;k<n;++k){auto e=generator.range(0,k,initial_ns,true,0);sources->registerPotentialRange(e);startup.measurements.push_back(e.measurement);}
 bool event=generator.audit(*sources);
 // Initial bias is not an independent draw. The original zero bias plus
 // every startup RW latent coordinate must fit the registered initial box
 // on the SAME source event, regardless of the realized noise values.
 for(const auto& b:generator.bias_support)require(b.upper()<=.02,"SIM_STARTUP_RW_OUTSIDE_INITIAL_DOMAIN");
 RollingRawState physical;for(int j=0;j<3;++j){physical.p[j]=I(0);physical.v[j]=I(-1,1);for(int k=0;k<3;++k)physical.rotation[j][k]=I(-1,1);}for(auto& b:physical.bias)b=I(-.02,.02);
 model.initial_set=measurementInitial(*sources,startup,physical,base.lever,1,1,ModelEvidenceStatus::VerifiedForDeclaredModel,"D1-independent-latent-proxy-and-directed-roundoff-v1");
 NavigationState numerical;numerical.timestamp=startup.timestamp;numerical.position_world_m=numericalUwbCandidate(startup,-1).functional.tag-base.lever;
 UnifiedRollingIntegrityCore core(root,cfg,numerical,base.lever,domain,last,std::move(model));core.enableSemanticAudit();
 std::ofstream out(argv[3]);require(bool(out),"SIM_OUTPUT_OPEN");out<<std::setprecision(17);
 out<<"epoch,stamp_ns,status,bound_status,latent_event,contained,imu_alarm,uwb_alarm,excluded_source,bridge,repair,rejoined,first_dependency,updates,generation,boundary,prefix,risk,registered,visible,measurements,hpl,vpl,herror,verror,core_ms,slots,ledger,replay,capsules,rss,source_digest,receipt_digest,boundary_digest,provider_plus_core_ms,period_ms,prepare_ms,candidate_ms,proof_ms,commit_ms,cleanup_ms,finalize_ms\n";
 std::ofstream sink,delivery;std::unique_ptr<Sc9PacedReleases> releases;
 if(paced){
  sink.open(std::string(argv[3])+".sink.csv");delivery.open(std::string(argv[3])+".delivery.csv");require(bool(sink)&&bool(delivery),"SIM_SINK_OPEN");
  sink<<std::setprecision(17)<<"epoch,generation,status,hpl,vpl,tag_x,tag_y,tag_z,source_digest,receipt_digest,boundary_digest\n";
  delivery<<std::setprecision(17)<<"epoch,measurement_ns,measurement_target_ns,source_batch_end_ns,release_target_ns,release_actual_ns,decode_start_ns,sink_ns,deadline_ns,core_ms,provider_plus_core_ms,release_lateness_ms,queue_ms,measurement_to_sink_ms,queue_high,timely,timely_spatial\n";
  std::vector<int64_t> ends;int64_t t=initial_ns;
  for(size_t epoch=1;epoch<=count;++epoch){t=schedule?schedule["batches"][batch_cursor+epoch-1]["end_ns"].as<int64_t>():t+(epoch%2?47000003:63000007);ends.push_back(t);}
  releases=std::make_unique<Sc9PacedReleases>(initial_ns,std::move(ends));
 }
 int64_t previous=initial_ns,onset=-1;size_t bridge=0,repair=0,rejoin=0,exclude=0,covered=0,uwb_remaining=0;double max_ms=0;
 for(size_t epoch=1;epoch<=count;++epoch){Sc9PacedReleases::Tick release;if(releases)release=releases->next(epoch);
  const auto provider_start=std::chrono::steady_clock::now();const int64_t end=schedule?schedule["batches"][batch_cursor]["end_ns"].as<int64_t>():previous+(epoch%2?47000003:63000007);require(end>previous,"SIM_EXTERNAL_EPOCH_ORDER");if(releases)require(end==release.end,"SIM_RELEASE_SOURCE_TIME_MISMATCH");const size_t phase=fault=="repeated"?(epoch-1)%400+1:epoch;
  if(phase==80&&fault!="healthy"){onset=previous+13000000;if(fault=="U"||fault=="J"||fault=="repeated")uwb_remaining=3;}
  const int axis=(fault=="G"?3:0)+(fault=="repeated"?int(((epoch-1)/400)%3):0);const bool imu_fault=fault!="healthy"&&fault!="U";
  Sc6Envelope input;input.epoch=epoch;input.uwb.id=BatchId(epoch+1);input.uwb.timestamp=TimestampNs(origin+end);input.ready=input.uwb.timestamp;
  const int64_t step=((epoch-1)/10)%2?2500000:10000000;
  while(schedule?(imu_cursor<imu_times.size()&&imu_times[imu_cursor]<=end):generator.last_ns+step<=end){const int64_t ns=schedule?imu_times[imu_cursor++]:generator.last_ns+step;const bool active=imu_fault&&onset>=0&&ns>=onset&&ns<onset+500000000;
   last=generator.imu(ns,axis,active?(fault=="G"?1.:20.):0.);input.imu.push_back(last);}
  auto add_range=[&](size_t k,int64_t ns,bool valid){require(k<n,"SIM_EXTERNAL_ANCHOR");
   const bool bad=uwb_remaining&&k==0&&valid;if(bad)--uwb_remaining;
   auto e=generator.range(epoch,k,ns,valid,bad?2.25:0);e.received=input.ready;input.range_events.push_back(e);if(valid)input.uwb.measurements.push_back(e.measurement);
  };
  if(schedule){for(const auto& row:schedule["batches"][batch_cursor++]["ranges"])add_range(row["anchor"].as<size_t>(),row["stamp_ns"].as<int64_t>(),row["valid"].as<bool>());}
  else{for(size_t k=0;k<n;++k)add_range(k,end-int64_t(k%3)*1000000,k!=epoch%n&&k!=(epoch+1)%n);
   if(epoch%4==0)add_range(n-1,end-1500000,(n-1)!=epoch%n&&(n-1)!=(epoch+1)%n);}
  const bool actions=!(fault=="J"||fault=="repeated")||phase<80||phase>=113;
  const auto start=std::chrono::steady_clock::now();const auto r=core.step(input,actions);const auto finished=std::chrono::steady_clock::now();const double elapsed=std::chrono::duration<double,std::milli>(finished-start).count();const double inclusive=std::chrono::duration<double,std::milli>(finished-provider_start).count();max_ms=std::max(max_ms,elapsed);
  if(releases){
   require(r.bound.valid&&r.bound.status=="GENERALIZED_CONDITIONAL_RAW_BOUND_VERIFIED"&&r.generation==epoch&&r.backend_updates==1,"SIM_SINK_RECEIPT_REFUSED");
   require(r.source_digest.size()==64&&r.receipt_digest.size()==64&&r.boundary_digest.size()==64,"SIM_SINK_AUDIT_MISSING");
   // Deliver the actual estimate/bound/receipt before privileged draw or GT
   // scoring. The sink clock is sampled only after this complete row flushes.
   sink<<epoch<<','<<r.generation<<','<<r.bound.status<<','<<r.bound.hpl<<','<<r.bound.pl.z()<<','<<r.tag.x()<<','<<r.tag.y()<<','<<r.tag.z()<<','<<r.source_digest<<','<<r.receipt_digest<<','<<r.boundary_digest<<'\n';sink.flush();if(!sink)throw std::runtime_error("SIM_SINK_WRITE_FAILED_AFTER_COMMIT epoch="+std::to_string(epoch)+" generation="+std::to_string(r.generation));
   const auto accepted=Sc9PacedReleases::Clock::now();int64_t measured=end;
   for(const auto& range:input.range_events)measured=std::min(measured,range.measurement.timestamp.value()-origin);
   auto ms=[](auto delta){return std::chrono::duration<double,std::milli>(delta).count();};
   const auto deadline=releases->deadline(epoch,measured);const bool timely=elapsed<=40&&accepted<=deadline;
   delivery<<epoch<<','<<origin+measured<<','<<Sc9PacedReleases::ns(releases->at(measured))<<','<<origin+end<<','<<Sc9PacedReleases::ns(release.target)<<','<<Sc9PacedReleases::ns(release.released)<<','<<Sc9PacedReleases::ns(provider_start)<<','<<Sc9PacedReleases::ns(accepted)<<','<<Sc9PacedReleases::ns(deadline)<<','<<elapsed<<','<<inclusive<<','<<ms(release.released-release.target)<<','<<ms(provider_start-release.released)<<','<<ms(accepted-releases->at(measured))<<','<<release.high_water<<','<<timely<<','<<(timely&&r.bound.hpl<=2&&r.bound.pl.z()<=3)<<'\n';delivery.flush();if(!delivery)throw std::runtime_error("SIM_DELIVERY_LOG_FAILED_AFTER_SINK epoch="+std::to_string(epoch));
  }
  event&=generator.audit(*sources);const auto truth=sc6D1MotionAtNanoseconds(end);Eigen::Vector3d tag;bool contained=true;
  for(int j=0;j<3;++j){I t=truth.p[j];for(int k=0;k<3;++k)t+=truth.r[j][k]*I(base.lever[k]);tag[j]=mid(t);contained&=r.bound.low[j]<=t.lower()&&t.upper()<=r.bound.high[j];}
  require(r.bound.valid&&r.bound.status=="GENERALIZED_CONDITIONAL_RAW_BOUND_VERIFIED","SIM_BOUND_NOT_VERIFIED");require(!event||contained,"SIM_CONDITIONAL_TRUTH_LOST");
  require(r.generation==epoch&&r.backend_updates==1&&core.updateCount()==epoch+1,"SIM_COMMIT_DISCONTINUITY");
  const auto route=runtimeRouteAuditV1();require(route.old_engine_calls==0&&route.old_pipeline_constructions==0&&route.old_fde_calls==0&&route.incremental_backend_constructions==1,"SIM_RUNTIME_ROUTE");
  const Eigen::Vector3d error=r.tag-tag;covered+=contained;bridge+=r.action.current_bridge;repair+=r.action.boundary_repair;rejoin+=r.action.rejoined;exclude+=r.action.excluded_source.has_value();
  out<<epoch<<','<<r.state.timestamp.value()<<','<<r.action.status<<','<<r.bound.status<<','<<event<<','<<contained<<','<<r.imu_alarm<<','<<r.action.uwb_alarm<<','<<(r.action.excluded_source?std::to_string(r.action.excluded_source->value()):"")<<','<<r.action.current_bridge<<','<<r.action.boundary_repair<<','<<r.action.rejoined<<','<<r.action.first_dependency<<','<<r.backend_updates<<','<<r.generation<<','<<r.boundary<<','<<r.source_prefix<<','<<r.bound.event_risk<<','<<r.input_counts.registered<<','<<r.input_counts.visible<<','<<r.input_counts.measurements<<','<<r.bound.hpl<<','<<r.bound.pl.z()<<','<<error.head<2>().norm()<<','<<std::abs(error.z())<<','<<elapsed<<','<<r.slots<<','<<r.ledger<<','<<r.replay<<','<<r.capsules<<','<<r.rss<<','<<r.source_digest<<','<<r.receipt_digest<<','<<r.boundary_digest<<','<<inclusive<<','<<double(end-previous)/1e6<<','<<r.prepare_ms<<','<<r.candidate_ms<<','<<r.proof_ms<<','<<r.commit_ms<<','<<r.cleanup_ms<<','<<r.finalize_ms<<'\n';out.flush();previous=end;
 }
 std::cout<<(schedule?"B2":"B3")<<" count="<<count<<" event="<<event<<" contained="<<covered<<" bridge="<<bridge<<" repair="<<repair<<" rejoin="<<rejoin<<" exclusions="<<exclude<<" numerical_residue="<<generator.residue<<" max_core_ms="<<max_ms<<'\n';
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
