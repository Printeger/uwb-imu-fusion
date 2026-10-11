#include "hybrid_prediction.hpp"
#include <iostream>
#include <Eigen/QR>
using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
void runHybridNative(const std::string&,bool);
int main(int argc,char** argv){try{
 HybridFixedLaw law;HybridHistoryProducer producer(law),replay(law);
 size_t nonzero=0,finite=0;
 Eigen::MatrixXd dense=Eigen::MatrixXd::Zero(21,21);
 dense.topLeftCorner(15,15).diagonal().setConstant(1/law.initial_sigma);
 dense.bottomRightCorner(6,6).diagonal().setConstant(1/law.endpoint_sigma);
 Eigen::VectorXd denseZ=Eigen::VectorXd::Zero(21),denseFault=Eigen::VectorXd::Zero(21);
 for(int epoch=1;epoch<=20;++epoch){
  auto h=producer.query();auto p=hybridPredict(law,producer,h,epoch,epoch-1);
  require(p.status=="FIXED_MODEL_VERIFIED","H/P production");
  if(epoch>1){
   Eigen::MatrixXd omitted=law.F*h.covariance.topLeftCorner(15,15)*law.F.transpose()+law.L*h.covariance.bottomRightCorner(6,6)*law.L.transpose();
   omitted.diagonal().array()+=law.process_sigma*law.process_sigma;
   omitted+=law.endpoint_sigma*law.endpoint_sigma*law.L*law.L.transpose();
   require((p.covariance-omitted).norm()>1e-12,"shared endpoint cross omission discovered");
  }
  Eigen::VectorXd ranges=Eigen::VectorXd::Zero(8);
  // Weak, fixed hidden episode: no label or truth enters H/P/B.
  if(epoch>=3 && epoch<=5)ranges(0)=.002;
  auto b=hybridFixedCurrent(law,producer,p,ranges,false);
  require(b.status=="FIXED_MODEL_VERIFIED","joint positive bound");
  if(b.history.norm()>0)++nonzero;
  ++finite;
  std::vector<uint64_t> ids;for(int i:b.retained)ids.push_back(epoch*100+i+1);
  producer.committed(epoch,epoch,ranges,b.retained,ids);replay.committed(epoch,epoch,ranges,b.retained,ids);
  // Independent unretired raw-prefix normal-equation oracle. The online
  // producer retires 21 columns every epoch; this oracle keeps every column.
  int oldRows=dense.rows(),oldCols=dense.cols();
  dense.conservativeResize(oldRows+29,oldCols+21);dense.topRightCorner(oldRows,21).setZero();dense.bottomRows(29).setZero();
  denseZ.conservativeResize(oldRows+29);denseZ.tail(29).setZero();denseFault.conservativeResize(oldRows+29);denseFault.tail(29).setZero();
  dense.block(oldRows,oldCols-21,15,15)=-law.F/law.process_sigma;
  dense.block(oldRows,oldCols-6,15,6)=-law.L/law.process_sigma;
  dense.block(oldRows,oldCols,15,15)=Eigen::MatrixXd::Identity(15,15)/law.process_sigma;
  dense.block(oldRows,oldCols+15,15,6)=-law.L/law.process_sigma;
  dense.block(oldRows+15,oldCols+15,6,6)=Eigen::MatrixXd::Identity(6,6)/law.endpoint_sigma;
  dense.block(oldRows+21,oldCols,8,15)=law.H/law.range_sigma;
  denseZ.tail(8)=ranges/law.range_sigma;
  if(epoch>=3 && epoch<=5)denseFault(oldRows+21)=1/law.range_sigma;
  Eigen::MatrixXd info=dense.transpose()*dense;
  auto factor=info.ldlt();Eigen::VectorXd oracle=factor.solve(dense.transpose()*denseZ);
  Eigen::MatrixXd fullCov=factor.solve(Eigen::MatrixXd::Identity(info.rows(),info.cols()));
  auto next=producer.query();
  require((oracle.tail(21)-next.mean).norm()<1e-8 && (fullCov.bottomRightCorner(21,21)-next.covariance).norm()<1e-9,"H retirement vs raw prefix oracle");
  if(epoch>=3){Eigen::VectorXd response=factor.solve(dense.transpose()*denseFault);
   for(const auto& segment:next.segments)if(segment.episode=="source:0:onset:3:length:3")require((segment.direction+response.tail(21)).norm()<1e-8,"H historical response vs raw prefix oracle");
  }
  require(next.epoch==size_t(epoch),"commit-before-H");
  if(epoch==8){
   auto stable=producer.query();
   for(int field=0;field<12;++field){auto forged=stable;
    if(field==0)forged.model="other";
    if(field==1)forged.epoch++;
    if(field==2)forged.horizon++;
    if(field==3)forged.radius*=2;
    if(field==4)forged.risk=0;
    if(field==5)forged.retained_sources.pop_back();
    if(field==6)forged.mean(0)+=1;
    if(field==7)forged.covariance(0,0)*=2;
    if(field==8)forged.segments[0].low=0;
    if(field==9)forged.segments[0].direction(3)+=1;
    if(field==10)forged.segments[0].episode="other";
    if(field==11)forged.status="UNKNOWN";
    require(hybridPredict(law,producer,forged,9,8).status!="FIXED_MODEL_VERIFIED","complete H semantic tamper");
   }
   auto nextP=hybridPredict(law,producer,stable,9,8);nextP.covariance.resize(1,1);
   require(hybridFixedCurrent(law,producer,nextP,Eigen::VectorXd::Zero(8),false).status=="EVIDENCE_REFUSED","P covariance tamper");
   producer.invalidate("STALE");require(hybridPredict(law,producer,producer.query(),9,8).status!="FIXED_MODEL_VERIFIED","stale rejection");
   producer.rebuiltFrom(replay);require(producer.query().status=="BOUNDED","recipe rebuild");
  }
  auto altered=producer.query();altered.risk=0;require(hybridPredict(law,producer,altered,epoch+1,epoch).status!="FIXED_MODEL_VERIFIED","risk tamper");
 }
 require(nonzero>0 && finite==20,"nonempty history positive denominator");
 // Independent scalar cancellation oracle: d=M, f=-M, y=0. Exact true error
 // is M, and history-output plus the eta part of current bound is M.
 HybridHistoryEvidence h;h.status="BOUNDED";Eigen::VectorXd direction(1);direction<<1;
 h.segments.push_back({"scalar",direction,-10,10});
 Eigen::MatrixXd one=Eigen::MatrixXd::Ones(1,1),zero=Eigen::MatrixXd::Zero(1,1),c=Eigen::MatrixXd::Ones(3,1);
 auto b=hybridBound(one,one,one,zero,c,Eigen::VectorXd::Zero(1),h,-1,1e-6,1);
 HybridHistoryEvidence currentOnly;currentOnly.status="BOUNDED";
 auto old=hybridBound(one,one,one,zero,c,Eigen::VectorXd::Zero(1),currentOnly,-1,1e-6,1);
 require(old.gaussian.pl(0)+b.history(0)<10,"output-history-only cancellation counterexample");
 require(std::abs(b.eta-10/std::sqrt(2.))<1e-10 && std::abs(b.history(0)-5)<1e-10 && b.gaussian.pl(0)>=10,"history cancellation");
 // Nonzero N: independent direct S/K normal equations versus augmented QR.
 Eigen::MatrixXd P=Eigen::MatrixXd::Identity(15,15),H=law.H,R=Eigen::MatrixXd::Identity(8,8);
 R.array()+=.12;
 Eigen::MatrixXd N=.1*H.transpose();HybridHistoryEvidence empty;empty.status="BOUNDED";
 auto correlated=hybridBound(P,H,R,N,law.C,Eigen::VectorXd::Ones(8),empty,2,1e-6,10);
 std::vector<int> kept;for(int i=0;i<8;++i)if(i!=2)kept.push_back(i);
 Eigen::MatrixXd hh(7,15),nn(15,7);for(int i=0;i<7;++i){hh.row(i)=H.row(kept[i]);nn.col(i)=N.col(kept[i]);}
 Eigen::MatrixXd ss=hh*P*hh.transpose()+principal(R,kept)+hh*nn+nn.transpose()*hh.transpose();
 Eigen::MatrixXd kk=ss.ldlt().solve((P*hh.transpose()+nn).transpose()).transpose();
 require((kk-correlated.K).norm()<1e-10,"nonzero N gain");
 require((kk*Eigen::VectorXd::Ones(7)-correlated.gaussian.mean).norm()<1e-10,"minus N augmented mean");
 Eigen::MatrixXd joint=Eigen::MatrixXd::Zero(22,22),design=Eigen::MatrixXd::Zero(22,15);joint.topLeftCorner(15,15)=P;joint.bottomRightCorner(7,7)=principal(R,kept);joint.topRightCorner(15,7)=-nn;joint.bottomLeftCorner(7,15)=-nn.transpose();design.topRows(15).setIdentity();design.bottomRows(7)=hh;
 Eigen::VectorXd observations=Eigen::VectorXd::Zero(22);observations.tail(7).setOnes();Eigen::LLT<Eigen::MatrixXd> white(joint);
 Eigen::MatrixXd wd=white.matrixL().solve(design);Eigen::VectorXd wz=white.matrixL().solve(observations);
 Eigen::VectorXd independentQR=wd.colPivHouseholderQr().solve(wz);
 require((independentQR-correlated.gaussian.mean).norm()<1e-10,"correlated joint QR");
 auto wrongSign=hybridBound(P,H,R,-N,law.C,Eigen::VectorXd::Ones(8),empty,2,1e-6,10);
 require((wrongSign.gaussian.covariance-correlated.gaussian.covariance).norm()>1e-3,"N sign negative");
 auto independent=hybridBound(P,H,R,Eigen::MatrixXd::Zero(15,8),law.C,Eigen::VectorXd::Ones(8),empty,2,1e-6,10);
 Eigen::MatrixXd independentS=hh*P*hh.transpose()+principal(R,kept);
 Eigen::MatrixXd independentK=independentS.ldlt().solve((P*hh.transpose()).transpose()).transpose();
 require((independent.K-independentK).norm()<1e-10,"N zero original B kernel");
 auto staleReplay=replay;producer.invalidate("UNKNOWN");
 // Replay was rebuilt from the actual prefix. It cannot be replaced by a fresh
 // initializer or a timestamp-only query carrying the same model name.
 HybridHistoryProducer fresh(law);bool rejected=false;try{producer.rebuiltFrom(fresh);}catch(const std::exception&){rejected=true;}require(rejected,"stale refresh request");producer.rebuiltFrom(staleReplay);
 rejected=false;try{producer.committed(20,20,Eigen::VectorXd::Zero(8),{0},{2001});}catch(const std::exception&){rejected=true;}require(rejected && producer.query().epoch==20,"duplicate/missing receipt does not advance");
 HybridHistoryProducer exhausted(law,1);require(exhausted.query().status=="EXHAUSTED","resource rejection");
 auto hist=producer.query();auto bad=hist;bad.generation++;require(hybridPredict(law,producer,bad,21,20).status!="FIXED_MODEL_VERIFIED","version refusal");
 std::cout<<"hybrid fixed H/P/B: outputs="<<finite<<" nonzero_history="<<nonzero<<" common_ball_risk="<<law.epsilon<<" radius="<<law.radius()<<" full_C_calls=0; cancellation/correlation/lifecycle PASS\n";
 if(argc>1){runHybridNative(argv[1],false);runHybridNative(argv[1],true);}
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
