#include "sc6_service.hpp"
#include <tbb/global_control.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
namespace uwb_imu_pl::research {
namespace {using Clock=std::chrono::steady_clock;using Time=Clock::time_point;double millis(Time a,Time b){return std::chrono::duration<double,std::milli>(b-a).count();}}
int runSc6System(int argc,char** argv){
 if(argc<9)throw std::invalid_argument("sc6 CONFIG CSV CASE on|off COUNT PROFILE offline|paced [SEED]");
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 bool paced=std::string(argv[8])=="paced",fde=std::string(argv[5])=="on";std::string profile=argv[7];if(profile!="baseline-adapter" && profile!="tight-reference" && profile!="tight-optimized")throw std::invalid_argument("SC6_PROFILE_UNKNOWN");
 Sc6Provider provider(argv[1],argv[3],argc>9?std::stoi(argv[9]):20261010,std::stoi(argv[6]));Sc6Service service(provider.law,provider.cfg,provider.initial,provider.lever,provider.domain,provider.imu.front(),profile!="baseline-adapter",argc>10 && std::string(argv[10])=="shadow");
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("OUTPUT_UNAVAILABLE");out<<std::setprecision(17);
 out<<"epoch,updates,qualified,timely,status,recovery,version,generation,parent,boundary,x0,source_prefix,risk,q,shared_q,alerts,bridge,rejoined,excluded,repair,removed_priors,removed_groups,hpl,vpl,old_hpl,old_vpl,bias_width,slots,live,ledger,replay,capsules,rss,source_ms,prepare_ms,candidate_ms,proof_ms,commit_ms,cleanup_ms,core_ms,delivery_ms,measure_ms,release_ms,enqueue_ms,ready_ms,start_ms,alarm_ms,selected_ms,commit_time_ms,proof_time_ms,sink_ms,tag_x,tag_y,tag_z,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz,hpe,vpe,ape,velocity_error,rotation_error,accel_bias_error,gyro_bias_error,gt_px,gt_py,gt_pz,gt_qw,gt_qx,gt_qy,gt_qz,formal_eligible,publication_protected,application_available\n";
 struct Release {int index;Time enqueue;};std::deque<Release> queue;std::mutex mutex;std::condition_variable cv;bool gap=false,done=false;size_t released=0,enqueued=0;const size_t capacity=80;Time origin=Clock::now();
 std::thread producer;if(paced)producer=std::thread([&]{for(int i=1;i<=provider.epochs*10;++i){std::this_thread::sleep_until(origin+std::chrono::nanoseconds(int64_t(i)*5000000LL));auto now=Clock::now();{std::lock_guard<std::mutex> lock(mutex);++released;if(queue.size()>=capacity)gap=true;if(!gap){queue.push_back({i,now});++enqueued;}}cv.notify_one();}{std::lock_guard<std::mutex> lock(mutex);done=true;}cv.notify_all();});
 uint64_t delivered=0;size_t committed=0,refused=0;std::string failure;Sc6Result sink_packet;
 for(int k=1;k<=provider.epochs;++k){Time ready=Clock::now(),enqueue=ready;bool valid=true;
  if(paced){std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return gap || done || (!queue.empty() && queue.back().index>=10*k);});if(gap)valid=false;else{for(int i=(k-1)*10+1;i<=10*k;++i){if(queue.empty() || queue.front().index!=i){valid=false;gap=true;break;}enqueue=queue.front().enqueue;queue.pop_front();}ready=enqueue;}}
  if(!valid || !failure.empty()){out<<k<<",0,0,0,SOURCE_GAP_OR_ABORT,"<<(failure.empty()?"NOT_ATTEMPTED":failure)<<'\n';++refused;continue;}
  Time start=Clock::now();try{
   auto envelope=provider.certify(k);Time source_done=Clock::now();auto result=service.step(envelope,fde);Time core_done=Clock::now();double core=millis(start,core_done);
   // Payload copy and receiver gate belong to delivery, before GT scoring.
   sink_packet=result;bool receipt_ok=result.epoch==delivered+1 && result.state.timestamp.value()==int64_t(k)*50000000LL && result.version==uint64_t(k+1) && result.bound.valid;
   Time sink=Clock::now();double age=paced?millis(origin+std::chrono::nanoseconds(int64_t(k)*50000000LL),sink):millis(start,sink);bool timely=receipt_ok && age<=50;if(receipt_ok)delivered=k;
   auto gt=provider.truth(k*10,true);Eigen::Vector3d tag=result.state.position_world_m+result.state.q_world_body*provider.lever;Eigen::Vector3d error=tag-(gt.position_world_m+gt.q_world_body*provider.lever);double rot=Eigen::AngleAxisd(gt.q_world_body.conjugate()*result.state.q_world_body).angle();
   auto relative=[&](Time t){return millis(origin,t);};double alarm=relative(source_done)+result.prepare_ms,selected=alarm+result.candidate_ms,commit=selected+result.proof_ms+result.commit_ms;
   out<<k<<",1,"<<result.bound.valid<<','<<timely<<','<<result.bound.status<<','<<result.action.recovery<<','<<result.version<<','<<result.generation<<','<<result.parent<<','<<result.boundary<<','<<result.x0<<','<<result.source_prefix<<','<<result.bound.event_risk<<','<<result.q<<','<<result.shared_q<<','<<result.alerts<<','<<result.action.current_bridge<<','<<result.action.rejoined<<','<<result.action.excluded<<','<<result.action.boundary_repair<<','<<result.action.removed_prior_slots.size()<<','<<result.action.plan.groups_to_remove.size()<<','<<result.bound.hpl<<','<<result.bound.pl.z()<<','<<result.old_hpl<<','<<result.old_vpl<<','<<result.bias_width<<','<<result.slots<<','<<result.live<<','<<result.ledger<<','<<result.replay<<','<<result.capsules<<','<<result.rss<<','<<millis(start,source_done)<<','<<result.prepare_ms<<','<<result.candidate_ms<<','<<result.proof_ms<<','<<result.commit_ms<<','<<result.cleanup_ms<<','<<core<<','<<age<<','<<k*50.<<','<<k*50.<<','<<relative(enqueue)<<','<<relative(ready)<<','<<relative(start)<<','<<alarm<<','<<selected<<','<<commit<<','<<relative(core_done)<<','<<relative(sink);
   for(const auto& v:std::vector<Eigen::Vector3d>{tag,result.state.position_world_m,result.state.velocity_world_mps})for(int j=0;j<3;++j)out<<','<<v(j);const auto& q=result.state.q_world_body;out<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z();for(const auto& v:std::vector<Eigen::Vector3d>{result.state.accel_bias_mps2,result.state.gyro_bias_radps})for(int j=0;j<3;++j)out<<','<<v(j);
   out<<','<<error.head<2>().norm()<<','<<std::abs(error.z())<<','<<error.norm()<<','<<(result.state.velocity_world_mps-gt.velocity_world_mps).norm()<<','<<rot<<','<<(result.state.accel_bias_mps2-gt.accel_bias_mps2).norm()<<','<<(result.state.gyro_bias_radps-gt.gyro_bias_radps).norm();for(int j=0;j<3;++j)out<<','<<gt.position_world_m(j);out<<','<<gt.q_world_body.w()<<','<<gt.q_world_body.x()<<','<<gt.q_world_body.y()<<','<<gt.q_world_body.z()<<",0,0,0\n";++committed;
   if(k%500==0 || result.action.boundary_repair)std::cout<<"SC6 epoch="<<k<<" repair="<<result.action.boundary_repair<<" alerts="<<result.alerts<<" H/V="<<result.bound.hpl<<'/'<<result.bound.pl.z()<<" core/age="<<core<<'/'<<age<<std::endl;
  }catch(const std::exception& e){failure=e.what();out<<k<<",0,0,0,ABORT,"<<failure<<'\n';++refused;std::cerr<<"SC6 FAILURE epoch="<<k<<' '<<failure<<std::endl;}
 }
 if(producer.joinable())producer.join();std::cout<<"SC6 denominator scheduled="<<provider.epochs<<" released_imu="<<released<<" enqueued_imu="<<enqueued<<" committed="<<committed<<" refused="<<refused<<" profile="<<profile<<std::endl;return failure.empty()?0:2;
}
}
