#pragma once
#include "hybrid_evidence.hpp"
#include <cstdio>
namespace uwb_imu_pl::research {
inline bool sameMatrix(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b){return a.rows()==b.rows() && a.cols()==b.cols() && a.allFinite() && b.allFinite() && a.isApprox(b,0);}
inline bool sameHistory(const HybridHistoryEvidence& a,const HybridHistoryEvidence& b){
 if(a.status!=b.status || a.model!=b.model || a.epoch!=b.epoch || a.generation!=b.generation || a.horizon!=b.horizon || a.radius!=b.radius || a.risk!=b.risk || a.retained_sources!=b.retained_sources || a.segments.size()!=b.segments.size())return false;
 if(a.mean.size()!=b.mean.size() || a.covariance.rows()!=b.covariance.rows() || a.covariance.cols()!=b.covariance.cols() || !a.mean.isApprox(b.mean,0) || !a.covariance.isApprox(b.covariance,0))return false;
 for(size_t i=0;i<a.segments.size();++i){const auto& x=a.segments[i];const auto& y=b.segments[i];if(x.episode!=y.episode || x.low!=y.low || x.high!=y.high || x.direction.size()!=y.direction.size() || !x.direction.isApprox(y.direction,0))return false;}
 return true;
}
struct HybridPrediction {
 std::string status="HISTORY_REFUSED",source="initial+committed-square-root+shared-endpoint+pending-process";
 size_t epoch=0,generation=0;
 Eigen::VectorXd mean;Eigen::MatrixXd covariance,N,transport;
 HybridHistoryEvidence history;
};
inline HybridPrediction hybridPredict(const HybridFixedLaw& law,const HybridHistoryProducer& producer,
 const HybridHistoryEvidence& supplied,size_t epoch,size_t generation){
 HybridPrediction p;p.epoch=epoch;p.generation=generation;
 const auto actual=producer.query();
 if(!law.same(producer.law()) || !sameHistory(supplied,actual) || actual.status!="BOUNDED" || actual.epoch+1!=epoch || actual.generation!=generation || epoch>size_t(law.horizon))return p;
 p.transport=Eigen::MatrixXd::Zero(15,21);p.transport.leftCols(15)=law.F;p.transport.rightCols(6)=law.L;
 p.mean=p.transport*actual.mean;
 // This explicitly retains F Cov(x,endpoint) L' and its transpose, in addition
 // to L Cov(endpoint) L'. The new endpoint is independent of the prefix.
 p.covariance=p.transport*actual.covariance*p.transport.transpose();
 p.covariance.diagonal().array()+=law.process_sigma*law.process_sigma;
 p.covariance+=law.endpoint_sigma*law.endpoint_sigma*law.L*law.L.transpose();
 p.N=Eigen::MatrixXd::Zero(15,law.ranges); // registered independent UWB sources
 p.history=actual;for(auto& s:p.history.segments)s.direction=p.transport*s.direction;
 p.history.mean=p.mean;p.history.covariance=p.covariance;
 p.status="FIXED_MODEL_VERIFIED";return p;
}
struct HybridBound {
 GaussianReference gaussian;
 Eigen::MatrixXd K,S;
 Eigen::Vector3d nominal=Eigen::Vector3d::Zero(),current=Eigen::Vector3d::Zero(),history=Eigen::Vector3d::Zero();
 Eigen::Vector3d slope=Eigen::Vector3d::Zero(),threshold_part=Eigen::Vector3d::Zero(),noise_part=Eigen::Vector3d::Zero(),eta_part=Eigen::Vector3d::Zero(),joint_fault=Eigen::Vector3d::Zero();
 double eta=0;int excluded=-1;std::vector<int> retained;
 std::string status="EVIDENCE_REFUSED";
};
// Core works for N != 0 as well. N is Cov(true-minus-predicted error, v);
// hence the augmented pseudo-observation cross block is -N.
inline HybridBound hybridBound(const Eigen::MatrixXd& P,const Eigen::MatrixXd& H,
 const Eigen::MatrixXd& R,const Eigen::MatrixXd& N,const Eigen::MatrixXd& C,
 const Eigen::VectorXd& innovation,const HybridHistoryEvidence& history,
 int excluded,double alpha,double q){
 HybridBound b;b.excluded=excluded;
 int d=P.rows(),n=H.rows();if(history.status!="BOUNDED" || P.cols()!=d || H.cols()!=d || N.rows()!=d || N.cols()!=n || R.rows()!=n || R.cols()!=n || C.cols()!=d || innovation.size()!=n || !(q>0))return b;
 for(const auto& segment:history.segments)if(segment.direction.size()!=d || !segment.direction.allFinite() || !std::isfinite(segment.low) || !std::isfinite(segment.high) || segment.low>segment.high)return b;
 for(int i=0;i<n;++i)if(i!=excluded)b.retained.push_back(i);
 int m=b.retained.size();Eigen::MatrixXd h(m,d),cross(d,m);Eigen::VectorXd y(m);
 for(int i=0;i<m;++i){h.row(i)=H.row(b.retained[i]);cross.col(i)=N.col(b.retained[i]);y(i)=innovation(b.retained[i]);}
 Eigen::MatrixXd r=principal(R,b.retained);
 Eigen::MatrixXd a=Eigen::MatrixXd::Zero(d+m,d),sigma=Eigen::MatrixXd::Zero(d+m,d+m);
 a.topRows(d).setIdentity();a.bottomRows(m)=h;sigma.topLeftCorner(d,d)=P;sigma.bottomRightCorner(m,m)=r;
 sigma.topRightCorner(d,m)=-cross;sigma.bottomLeftCorner(m,d)=-cross.transpose();
 Eigen::VectorXd z=Eigen::VectorXd::Zero(d+m);z.tail(m)=y;
 b.gaussian=gaussianReference(a,z,sigma,C,{},alpha,1e-9);
 if(b.gaussian.mean.size()!=d)return b;
 b.S=h*P*h.transpose()+r+h*cross+cross.transpose()*h.transpose();
 Eigen::LLT<Eigen::MatrixXd> sc(b.S);if(sc.info()!=Eigen::Success)return b;
 b.K=sc.solve((P*h.transpose()+cross).transpose()).transpose();
 Eigen::MatrixXd w=sc.matrixL().solve(Eigen::MatrixXd::Identity(m,m));
 b.eta=history.imageNorm(w*h);
 // The common primitive ball proves both nominal output and innovation noise
 // bounds for all adaptive subset histories. It also bounds the nominal NIS.
 b.gaussian.threshold=std::pow(q+b.eta,2);
 b.gaussian.statistic=(w*y).squaredNorm();
 b.history.setZero();Eigen::MatrixXd histmap=C*(Eigen::MatrixXd::Identity(d,d)-b.K*h);
 for(int j=0;j<3;++j){b.history(j)=history.support(histmap.row(j));b.nominal(j)=q*std::sqrt(std::max(0.,(C*b.gaussian.covariance*C.transpose())(j,j)));}
 double acceptance=std::sqrt(b.gaussian.threshold)+q+b.eta;
 Eigen::MatrixXd response=C*b.K;
 for(int i=0;i<m;++i){double denominator=w.col(i).norm();if(denominator<=1e-10){b.status="INDETERMINATE";return b;}b.slope=b.slope.cwiseMax(response.col(i).cwiseAbs()/denominator);}
 b.threshold_part=b.slope*std::sqrt(b.gaussian.threshold);b.noise_part=b.slope*q;b.eta_part=b.slope*b.eta;
 b.current=b.slope*acceptance;
 b.gaussian.pl=b.nominal+b.current+b.history;
 b.status=b.gaussian.statistic<=b.gaussian.threshold?"FIXED_MODEL_VERIFIED":"POST_REJECTED";
 return b;
}
struct JointEpisodeSupport {
 std::string episode;double low=0,high=0,innovation_norm=0;
 Eigen::Vector3d response=Eigen::Vector3d::Zero(),support=Eigen::Vector3d::Zero();
};
// Return the original source/anchor ID, onset and duration. Reject unknown
// provenance rather than assigning a new independent historical parameter.
inline bool hybridEpisodeIdentity(const std::string& id,int& source,int& onset,int& duration,bool raw=false){
 int end=0;
 const char* format=raw?"prefix-anchor:%d:onset:%d:duration:%d%n":"source:%d:onset:%d:length:%d%n";
 return std::sscanf(id.c_str(),format,&source,&onset,&duration,&end)==3 && end==int(id.size()) && source>=0 && onset>=1 && (duration==1 || duration==3);
}
// Same P/S/K and acceptance threshold as the old envelope. On the SAME noise
// event, fit one shared amplitude to the full observed innovation and intersect
// its historical feasible interval. Optimize the combined output BEFORE taking
// maxima over episodes. No extra Gaussian event and no fault-prior charging.
inline HybridBound hybridJointEpisodeBound(HybridBound base,const Eigen::MatrixXd& H,
 const Eigen::MatrixXd& C,const Eigen::VectorXd& innovation,const HybridHistoryEvidence& history,
 const std::vector<int>& continuing_columns,double q,std::vector<JointEpisodeSupport>* details=nullptr){
 if(details)details->clear();
 auto refused=[&](const std::string& status){base.status=status;base.gaussian.pl.setConstant(INFINITY);return base;};
 if(base.K.size()==0 || continuing_columns.size()!=history.segments.size() || !(q>0) || !std::isfinite(q) || H.rows()!=innovation.size() || C.rows()!=3 || C.cols()!=H.cols() || !H.allFinite() || !C.allFinite() || !innovation.allFinite())return refused("EVIDENCE_REFUSED");
 for(const auto& segment:history.segments)if(segment.direction.size()!=H.cols() || !segment.direction.allFinite() || std::isnan(segment.low) || std::isnan(segment.high) || segment.low>segment.high)return refused("EVIDENCE_REFUSED");
 int m=base.retained.size(),d=H.cols();Eigen::MatrixXd h(m,d);Eigen::VectorXd y(m);
 for(int i=0;i<m;++i){h.row(i)=H.row(base.retained[i]);y(i)=innovation(base.retained[i]);}
 Eigen::LLT<Eigen::MatrixXd> chol(base.S);if(chol.info()!=Eigen::Success){base.status="INDETERMINATE";return base;}
 Eigen::VectorXd wy=chol.matrixL().solve(y);
 Eigen::MatrixXd histmap=C*(Eigen::MatrixXd::Identity(d,d)-base.K*h);
 bool feasible=wy.norm()<=q; // the no-fault explanation
 base.joint_fault.setZero();
 auto episode=[&](const std::string& id,const Eigen::VectorXd& direction,int column,double low,double high){
  Eigen::VectorXd b=Eigen::VectorXd::Zero(m);
  if(column>=0){auto it=std::find(base.retained.begin(),base.retained.end(),column);if(it!=base.retained.end())b(it-base.retained.begin())=1;}
  Eigen::VectorXd a=chol.matrixL().solve(h*direction+b);
  Eigen::Vector3d u=histmap*direction-C*base.K*b;
  double norm=a.norm();
  // Literal source-column alias witness. Computed roundoff zero alone cannot
  // prove a harmless or dangerous nullspace. This matches the old oracle's
  // refusal policy for unresolved numerical rank.
  bool literalNull=(direction.array()==0.).all() && (b.array()==0.).all();int unit=-1;
  for(int j=0;j<direction.size();++j)if(direction(j)!=0){if(unit!=-1 || std::abs(direction(j))!=1){unit=-2;break;}unit=j;}
  if(unit>=0)literalNull=(b.array()==(-direction(unit)*h.col(unit)).array()).all();
  if((a.array()==0.).all()){
   if(!literalNull){base.status="INDETERMINATE";return;}
   u=C*direction; // a==0 implies the exact response is CD, independent of K

   if(wy.norm()>q)return;
   if(!std::isfinite(low) || !std::isfinite(high)){
    if((u.array()!=0.).any()){base.status="UNBOUNDED_PROVEN";return;}
    low=high=0; // structurally harmless, not epsilon-based harmlessness
   }
  }else{
   if(norm<1e-8){base.status="INDETERMINATE";return;}
   double center=a.dot(wy)/a.squaredNorm();Eigen::VectorXd perpendicular=wy-a*center;
   double slack=q*q-perpendicular.squaredNorm();
   double pad=1e-12*(1+q*q+wy.squaredNorm()); // outward roundoff allowance
   if(slack < -pad)return;
   double half=std::sqrt(std::max(0.,slack)+pad)/norm;
   low=std::max(low,center-half);high=std::min(high,center+half);
   if(low>high)return;
  }
  feasible=true;Eigen::Vector3d support=u.cwiseAbs()*std::max(std::abs(low),std::abs(high));
  base.joint_fault=base.joint_fault.cwiseMax(support);
  if(details)details->push_back({id,low,high,norm,u,support});
 };
 for(size_t i=0;i<history.segments.size();++i){const auto& s=history.segments[i];
  if(continuing_columns[i]<-1 || continuing_columns[i]>=H.rows())return refused("EVIDENCE_REFUSED");
  episode(s.episode,s.direction,continuing_columns[i],s.low,s.high);
  if(base.status=="UNBOUNDED_PROVEN" || base.status=="INDETERMINATE")return refused(base.status);
 }
 // All current onsets (duration1/3 have identical current projections) remain
 // covered with unbounded amplitudes, even if old measurements were excluded.
 for(int column:base.retained){episode("current-source:"+std::to_string(column),Eigen::VectorXd::Zero(d),column,-INFINITY,INFINITY);
  if(base.status=="UNBOUNDED_PROVEN" || base.status=="INDETERMINATE")return refused(base.status);
 }
 base.gaussian.pl=base.nominal+base.joint_fault;
 if(!feasible){base.status="JOINT_MODEL_INCONSISTENT";base.gaussian.pl.setConstant(INFINITY);}
 return base;
}
inline HybridBound hybridFixedCurrent(const HybridFixedLaw& law,const HybridHistoryProducer& producer,const HybridPrediction& p,
 const Eigen::VectorXd& ranges,bool a_alarm=true,double horizontalAL=2,double verticalAL=3,bool shared_episode=false){
 auto expected=hybridPredict(law,producer,producer.query(),p.epoch,p.generation);
 if(p.status!="FIXED_MODEL_VERIFIED" || expected.status!=p.status || p.source!=expected.source || !sameHistory(p.history,expected.history) || p.mean.size()!=expected.mean.size() || !sameMatrix(p.mean,expected.mean) || !sameMatrix(p.covariance,expected.covariance) || !sameMatrix(p.N,expected.N) || !sameMatrix(p.transport,expected.transport))return {};
 Eigen::MatrixXd r=Eigen::MatrixXd::Identity(law.ranges,law.ranges)*law.range_sigma*law.range_sigma;
 Eigen::VectorXd innovation=ranges-law.H*p.mean;
 std::vector<int> active;
 if(shared_episode)for(const auto& segment:p.history.segments){int source,onset,duration;
  if(!hybridEpisodeIdentity(segment.episode,source,onset,duration) || source>=law.ranges || onset>=int(p.epoch))return {};
  active.push_back(int(p.epoch)<onset+duration?source:-1);
 }
 auto candidate=[&](int excluded){auto bound=hybridBound(p.covariance,law.H,r,p.N,law.C,innovation,p.history,excluded,law.alpha,p.history.radius);
  if(shared_episode)bound=hybridJointEpisodeBound(std::move(bound),law.H,law.C,innovation,p.history,active,p.history.radius);
  if(bound.status=="FIXED_MODEL_VERIFIED" && (bound.gaussian.pl.head<2>().norm()>horizontalAL || bound.gaussian.pl.z()>verticalAL))bound.status="AL_REJECTED";
  return bound;
 };
 auto keep=candidate(-1);
 if(!a_alarm && keep.gaussian.dof>0 && keep.gaussian.statistic<=keep.gaussian.threshold)return keep;
 HybridBound best;double score=INFINITY;
 // Complete fixed family, including KEEP. Selection never drops a hypothesis.
 for(int i=-1;i<law.ranges;++i){auto b=candidate(i);if(b.status=="FIXED_MODEL_VERIFIED" && b.gaussian.statistic<score){score=b.gaussian.statistic;best=std::move(b);}}
 return best;
}
} // namespace
