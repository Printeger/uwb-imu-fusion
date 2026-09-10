#pragma once
#include "a18_certificate.h"
#include "a17_graph_io.h"
namespace a18 {
inline void put(Raw&r,const std::string&p,size_t i,const std::string&n,const gtsam::Matrix&m){DMat d(m.rows(),std::vector<double>(m.cols()));for(int a=0;a<m.rows();++a)for(int b=0;b<m.cols();++b){if(!std::isfinite(m(a,b)))nonfinite("GRAPH_CONSTANT_OR_STATE_"+n);d[a][b]=m(a,b);}r[{p,i,n}]=std::move(d);}
inline void put1(Raw&r,const std::string&p,size_t i,const std::string&n,double x){put(r,p,i,n,gtsam::Matrix::Constant(1,1,x));}
inline void states(Raw&r,const std::string&p,const gtsam::Values&v){for(auto k:v.keys()){auto sym=gtsam::Symbol(k);if(sym.chr()=='x')put(r,p,sym.index(),"X",v.at<gtsam::Pose3>(k).matrix());else if(sym.chr()=='v')put(r,p,sym.index(),"V",v.at<gtsam::Vector3>(k));else if(sym.chr()=='b')put(r,p,sym.index(),"B",v.at<gtsam::imuBias::ConstantBias>(k).vector());else unsupported("STATE_TYPE");}}
inline void nativeLog(Raw&r,const std::string&p,size_t i,const std::string&n,const gtsam::Matrix3&R){double trace=R.trace();put1(r,p,i,n+"_branch",trace+1<.001?3:(trace-3 < -1e-6?2:1));}
struct LiveGraph {
 const gtsam::NonlinearFactorGraph& graph;Data constants;
 LiveGraph(const gtsam::NonlinearFactorGraph&g,const std::vector<uifgo::DevelopmentRangeConstant>&range):graph(g){
  std::map<size_t,uifgo::DevelopmentRangeConstant> ranges;for(auto&r:range)if(!ranges.emplace(r.factor_index,r).second)unsupported("DUPLICATE_RANGE");
  auto&raw=constants.raw;put1(raw,"CONST",0,"epsilon",std::numeric_limits<double>::epsilon());put1(raw,"CONST",0,"log_near_zero_threshold",-1e-6);put1(raw,"CONST",0,"log_near_pi_threshold",.001);put1(raw,"CONST",0,"pose_log_threshold",1e-10);
  for(size_t i=0;i<g.size();++i){if(!g[i])unsupported("NULL_FACTOR");auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(g[i]);if(!nf)unsupported("UNKNOWN_FACTOR");auto nm=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(nf->noiseModel());if(!nm)unsupported("NON_GAUSSIAN");put(raw,"CONST",i,"whitening_R",nm->R());Factor f{i,"",{},nf->dim()};for(auto k:g[i]->keys())f.keys.push_back(gtsam::DefaultKeyFormatter(k));
   if(auto im=boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(g[i])){f.type="IMU";const auto&p=im->preintegratedMeasurements();if(p.p().omegaCoriolis||p.p().body_P_sensor||p.p().use2ndOrderCoriolis)unsupported("PIM_OPTIONS");put(raw,"CONST",i,"pim",p.preintegrated());put(raw,"CONST",i,"H_bias_acc",p.preintegrated_H_biasAcc());put(raw,"CONST",i,"H_bias_gyro",p.preintegrated_H_biasOmega());put(raw,"CONST",i,"bias_hat",p.biasHat().vector());put(raw,"CONST",i,"gravity",p.p().n_gravity);put1(raw,"CONST",i,"dt",p.deltaTij());put(raw,"CONST",i,"pim_covariance",p.preintMeasCov());}
   else if(auto pr=boost::dynamic_pointer_cast<uifgo::PaperPosePriorFactor>(g[i])){f.type="POSE_PRIOR";put(raw,"CONST",i,"prior",pr->prior().matrix());}
   else if(auto pr=boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(g[i])){f.type="V_PRIOR";put(raw,"CONST",i,"prior",pr->prior());}
   else if(auto pr=boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(g[i])){f.type="B_PRIOR";put(raw,"CONST",i,"prior",pr->prior().vector());}
   else if(ranges.count(i)){f.type="RANGE";auto&r=ranges.at(i);auto ex=boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(g[i]);if(!ex||ex->measured()!=r.measurement||g[i]->keys()!=gtsam::KeyVector{r.pose_key})unsupported("RANGE_GRAPH_IDENTITY");put(raw,"CONST",i,"anchor",r.anchor);put(raw,"CONST",i,"lever",r.lever);put1(raw,"CONST",i,"beta",r.conditional_beta);put1(raw,"CONST",i,"measurement",r.measurement);}
   else unsupported("UNKNOWN_FACTOR");constants.factors.push_back(std::move(f));
  }
 }
 Data pair(const gtsam::Values&base,const gtsam::Values&trial,const gtsam::VectorValues&delta,const gtsam::GaussianFactorGraph&linear)const{
  if(base.keys()!=trial.keys()||base.zeroVectors().size()!=delta.size()||linear.size()!=graph.size())unsupported("KEY_FACTOR_DIMENSION");Data d=constants;states(d.raw,"BASE",base);states(d.raw,"TRIAL",trial);
  for(auto&kv:delta){if(!base.exists(kv.first)||size_t(kv.second.size())!=base.at(kv.first).dim())unsupported("DELTA_KEY_DIMENSION");put(d.raw,"DELTA",gtsam::Symbol(kv.first).index(),std::string(1,gtsam::Symbol(kv.first).chr()),kv.second);}
  for(size_t i=0;i<graph.size();++i){
   for(auto pair:{std::make_pair("BASE",&base),std::make_pair("TRIAL",&trial)}){const auto&v=*pair.second;std::string phase=pair.first;
    if(auto im=boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graph[i])){const auto&p=im->preintegratedMeasurements();auto k=im->keys();gtsam::NavState si(v.at<gtsam::Pose3>(k[0]),v.at<gtsam::Vector3>(k[1]));auto bc=p.biasCorrectedDelta(v.at<gtsam::imuBias::ConstantBias>(k[4]));auto pred=p.predict(si,v.at<gtsam::imuBias::ConstantBias>(k[4]));nativeLog(d.raw,phase,i,"imu_log",v.at<gtsam::Pose3>(k[2]).rotation().transpose()*pred.R());put1(d.raw,phase,i,"exp_near_zero",bc.head<3>().squaredNorm()<=std::numeric_limits<double>::epsilon());}
    else if(auto pr=boost::dynamic_pointer_cast<uifgo::PaperPosePriorFactor>(graph[i])){auto relative=v.at<gtsam::Pose3>(pr->key()).between(pr->prior());nativeLog(d.raw,phase,i,"pose_log",relative.rotation().matrix());put1(d.raw,phase,i,"pose_log_small",gtsam::Rot3::Logmap(relative.rotation()).norm()<1e-10);}
   }
   if(!linear[i])unsupported("NULL_LINEAR_FACTOR");gtsam::JacobianFactor jf(*linear[i]);auto ab=jf.jacobian();auto&A=ab.first;gtsam::Vector r=-ab.second;std::vector<LinearRow>rows(A.rows());for(int row=0;row<A.rows();++row)rows[row]={i,size_t(row),{},r[row]};int col=0;for(auto it=jf.begin();it!=jf.end();++it){auto block=jf.getA(it);auto&de=delta.at(*it);if(de.size()!=block.cols())unsupported("LINEAR_DIM");for(int j=0;j<block.cols();++j)for(int row=0;row<A.rows();++row)if(A(row,col+j)!=0){if(!std::isfinite(A(row,col+j))||!std::isfinite(r[row]))nonfinite("LINEAR");rows[row].ad.emplace_back(A(row,col+j),de[j]);}col+=block.cols();}if(col!=A.cols())unsupported("LINEAR_COLUMNS");for(auto&row:rows)if(!row.ad.empty())d.linear.push_back(std::move(row));
  }
  return d;
 }
};
inline void dump(const std::string&dir,const Data&d){std::filesystem::create_directory(dir);auto f=output(dir+"/exact_binary64.csv");f<<"phase,index,name,row,column,hex,bits\n";for(auto&kv:d.raw){auto&k=kv.first;auto&m=kv.second;gtsam::Matrix a(m.size(),m[0].size());for(size_t i=0;i<m.size();++i)for(size_t j=0;j<m[0].size();++j)a(i,j)=m[i][j];emit(f,std::get<0>(k),std::get<1>(k),std::get<2>(k),a);}auto fm=output(dir+"/factors.csv");fm<<"factor,type,keys,dimension\n";for(auto&a:d.factors){fm<<a.index<<','<<a.type<<',';for(size_t i=0;i<a.keys.size();++i)fm<<(i?";":"")<<a.keys[i]<<":"<<uint64_t(gtsam::Symbol(a.keys[i][0],std::stoull(a.keys[i].substr(1))));fm<<','<<a.dim<<'\n';}}
}
