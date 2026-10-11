#include "hybrid_prediction.hpp"
#include <Eigen/QR>
#include <iostream>
using namespace uwb_imu_pl::research;
void check(bool b,const char* name){if(!b)throw std::runtime_error(name);}
void rawTighteningDiagnostics(const std::string&,const std::string&);
int main(int argc,char** argv){try{
 Eigen::MatrixXd one=Eigen::MatrixXd::Ones(1,1),zero=Eigen::MatrixXd::Zero(1,1),c=Eigen::MatrixXd::Ones(3,1);
 HybridHistoryEvidence empty;empty.status="BOUNDED";
 Eigen::VectorXd y=Eigen::VectorXd::Constant(1,.4);
 auto old=hybridBound(one,one,one,zero,c,y,empty,-1,1e-6,1);
 auto joint=hybridJointEpisodeBound(old,one,c,y,empty,{},1);
 double scalarOracle=1/std::sqrt(2.)+.5*(.4+std::sqrt(2.));
 check(std::abs(joint.gaussian.pl.x()-scalarOracle)<1e-9 && joint.gaussian.pl.x()<old.gaussian.pl.x(),"independent scalar interval oracle");
 HybridHistoryEvidence history;history.status="BOUNDED";history.segments.push_back({"shared",Eigen::VectorXd::Constant(1,-1),-10,10});
 y.setZero();old=hybridBound(one,one,one,zero,c,y,history,-1,1e-6,1);
 joint=hybridJointEpisodeBound(old,one,c,y,history,{0},1);
 check(joint.gaussian.pl.x()>=10 && joint.gaussian.pl.x()<old.gaussian.pl.x(),"shared-parameter innovation cancellation covered");
 auto dangerous=history;dangerous.segments[0].low=-INFINITY;dangerous.segments[0].high=INFINITY;
 check(hybridJointEpisodeBound(old,one,c,y,dangerous,{0},1).status=="UNBOUNDED_PROVEN","exact harmful unbounded joint nullspace");
 auto ended=hybridJointEpisodeBound(hybridBound(one,one,one,zero,c,Eigen::VectorXd::Constant(1,-10),history,-1,1e-6,1),one,c,Eigen::VectorXd::Constant(1,-10),history,{-1},1);
 check(ended.gaussian.pl.x()>=5 && ended.status=="FIXED_MODEL_VERIFIED","stopped historical episode remains covered");
 auto harmless=hybridJointEpisodeBound(old,one,Eigen::MatrixXd::Zero(3,1),y,dangerous,{0},1);
 check(harmless.status!="UNBOUNDED_PROVEN","structurally harmless joint nullspace");
 auto near=history;near.segments[0].direction(0)+=1e-11;
 check(hybridJointEpisodeBound(old,one,c,y,near,{0},1).status=="INDETERMINATE","near joint nullspace rejected");
 // Analytic selection counterexamples, no seed search/Monte Carlo evidence.
 double epsilon=1e-6,scalarQ=boost::math::quantile(boost::math::complement(boost::math::normal(),epsilon/2));
 double adaptiveFailure=boost::math::cdf(boost::math::complement(boost::math::chi_squared(100),scalarQ*scalarQ));
 double visitedOnlyFailure=-std::expm1(512*std::log1p(-epsilon));
 check(adaptiveFailure>.99 && visitedOnlyFailure>500*epsilon,"data-dependent projection and visited-only charging counterexamples");
 std::cout<<"LOW_DIM_NEGATIVE continuous_unit_noise_direction D=100 q="<<scalarQ<<" actual_tail="<<adaptiveFailure<<"; max_of_512 visited=1 actual_tail="<<visitedOnlyFailure<<"\n";
 int checked=0;double maxOld=0,maxJoint=0,maxLow=0;
 // Complete 9x9 strategy family, same 15D matrices/noises/episode scope.
 // Independent oracle keeps every 21D state and solves full-prefix QR.
 for(int first=-1;first<8;++first)for(int second=-1;second<8;++second){
  HybridFixedLaw law,lowLaw;lowLaw.simultaneous_projections=true;
  HybridHistoryProducer producer(law),lowProducer(lowLaw);
  Eigen::MatrixXd a=Eigen::MatrixXd::Zero(21,21);a.topLeftCorner(15,15).diagonal().setConstant(1/law.initial_sigma);a.bottomRightCorner(6,6).diagonal().setConstant(1/law.endpoint_sigma);
  Eigen::VectorXd z=Eigen::VectorXd::Zero(21),f=Eigen::VectorXd::Zero(21),observations;
  Eigen::VectorXd truth=Eigen::VectorXd::Zero(15);double amplitude=.002;
  for(int epoch=1;epoch<=2;++epoch){
   observations=Eigen::VectorXd::Zero(8);for(int i=0;i<8;++i)observations(i)=law.range_sigma*.2*std::sin(i+epoch);observations(0)+=amplitude;
   auto p=hybridPredict(law,producer,producer.query(),epoch,epoch-1),lp=hybridPredict(lowLaw,lowProducer,lowProducer.query(),epoch,epoch-1);
   auto selected=hybridFixedCurrent(law,producer,p,observations,true,2,3,true);
   auto lowSelected=hybridFixedCurrent(lowLaw,lowProducer,lp,observations,true,2,3,true);
   check(selected.status=="FIXED_MODEL_VERIFIED" && lowSelected.status=="FIXED_MODEL_VERIFIED","validated fixed consumer adaptive selection");
   int excluded=epoch==1?first:second;std::vector<int> active;
   for(const auto& segment:p.history.segments){int source,onset,duration;check(hybridEpisodeIdentity(segment.episode,source,onset,duration),"fixed provenance");active.push_back(epoch<onset+duration?source:-1);}
   auto base=hybridBound(p.covariance,law.H,Eigen::MatrixXd::Identity(8,8)*law.range_sigma*law.range_sigma,p.N,law.C,observations-law.H*p.mean,p.history,excluded,law.alpha,p.history.radius);
   std::vector<JointEpisodeSupport> supports;
   auto b=hybridJointEpisodeBound(base,law.H,law.C,observations-law.H*p.mean,p.history,active,p.history.radius,&supports);
   auto lb=hybridBound(lp.covariance,law.H,Eigen::MatrixXd::Identity(8,8)*law.range_sigma*law.range_sigma,lp.N,law.C,observations-law.H*lp.mean,lp.history,excluded,law.alpha,lp.history.radius);
   auto tight=hybridJointEpisodeBound(lb,law.H,law.C,observations-law.H*lp.mean,lp.history,active,lp.history.radius);
   check(b.status=="FIXED_MODEL_VERIFIED" && tight.status=="FIXED_MODEL_VERIFIED","candidate acceptance under both events");
   check((b.gaussian.pl.array()<=base.gaussian.pl.array()+1e-9).all(),"joint contains no independent-max enlargement");
   int rows=a.rows(),cols=a.cols(),m=b.retained.size();a.conservativeResize(rows+21+m,cols+21);a.topRightCorner(rows,21).setZero();a.bottomRows(21+m).setZero();
   z.conservativeResize(rows+21+m);z.tail(21+m).setZero();f.conservativeResize(rows+21+m);f.tail(21+m).setZero();
   a.block(rows,cols-21,15,15)=-law.F/law.process_sigma;a.block(rows,cols-6,15,6)=-law.L/law.process_sigma;
   a.block(rows,cols,15,15)=Eigen::MatrixXd::Identity(15,15)/law.process_sigma;a.block(rows,cols+15,15,6)=-law.L/law.process_sigma;
   a.block(rows+15,cols+15,6,6)=Eigen::MatrixXd::Identity(6,6)/law.endpoint_sigma;
   for(int i=0;i<m;++i){int source=b.retained[i];a.block(rows+21+i,cols,1,15)=law.H.row(source)/law.range_sigma;z(rows+21+i)=observations(source)/law.range_sigma;if(source==0)f(rows+21+i)=1/law.range_sigma;}
   auto qr=a.colPivHouseholderQr();Eigen::VectorXd fullMean=qr.solve(z),response=-qr.solve(f);
   Eigen::Vector3d error=law.C*(truth-fullMean.segment(cols,15));
   check((error.cwiseAbs().array()<=b.gaussian.pl.array()).all() && (error.cwiseAbs().array()<=tight.gaussian.pl.array()).all(),"independent full-prefix realization covered");
   check((law.C*fullMean.segment(cols,15)-law.C*(p.mean+b.gaussian.mean)).norm()<1e-8,"same full-prefix and sequential center");
   if(epoch==2 && first!=0)for(const auto& support:supports)if(support.episode=="source:0:onset:1:length:3"){
    check((support.response-law.C*response.segment(cols,15)).norm()<1e-8,"shared episode full-prefix fault response oracle");
    check(support.low<=amplitude && support.high>=amplitude,"shared true amplitude in joint feasible interval");
   }
   maxOld=std::max(maxOld,base.gaussian.pl.norm());maxJoint=std::max(maxJoint,b.gaussian.pl.norm());maxLow=std::max(maxLow,tight.gaussian.pl.norm());
   std::vector<uint64_t> ids;for(int source:b.retained)ids.push_back(epoch*100+source+1);
   producer.committed(epoch,epoch,observations,b.retained,ids);lowProducer.committed(epoch,epoch,observations,b.retained,ids);++checked;
  }
 }
 // Strong current fault and already-stopped historical explanation tests.
 HybridFixedLaw faultLaw;HybridHistoryProducer faultProducer(faultLaw);
 std::vector<int> kept{0,1,2,3,4,5,6,7};std::vector<uint64_t> sourceIds{101,102,103,104,105,106,107,108};
 faultProducer.committed(1,1,Eigen::VectorXd::Zero(8),kept,sourceIds);
 auto prediction=hybridPredict(faultLaw,faultProducer,faultProducer.query(),2,1);Eigen::VectorXd injected=Eigen::VectorXd::Zero(8);injected(0)=.5;
 check(hybridFixedCurrent(faultLaw,faultProducer,prediction,injected,false,2,3,true).excluded==0,"current alarm complete family excludes correct source");
 HybridFixedLaw oldLaw,lowLaw;lowLaw.simultaneous_projections=true;
 check(lowLaw.radius()<oldLaw.radius() && lowLaw.projectionCount()>std::pow(9.,24),"complete family event count fixed before data");
 std::cout<<"TIGHTENING_ORACLE complete_paths=81 candidates="<<checked<<" q_ball="<<oldLaw.radius()<<" q_projection="<<lowLaw.radius()<<" family_upper="<<lowLaw.projectionCount()<<" risk="<<lowLaw.epsilon<<" max_PL_norm old/joint/joint-low="<<maxOld<<"/"<<maxJoint<<"/"<<maxLow<<" scalar/coverage/acceptance/nullspace/cancellation/full-prefix PASS\n";
 if(argc>2)rawTighteningDiagnostics(argv[1],argv[2]);
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
