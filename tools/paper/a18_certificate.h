#pragma once
#include "a18_interval.h"
#include <map>
#include <tuple>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
namespace a18 {
using DMat=std::vector<std::vector<double>>;using RawKey=std::tuple<std::string,size_t,std::string>;using Raw=std::map<RawKey,DMat>;
struct Factor {size_t index;std::string type;std::vector<std::string> keys;size_t dim;};
struct LinearRow {size_t factor,row;std::vector<std::pair<double,double>> ad;double r;};
struct Data {Raw raw;std::vector<Factor> factors;std::vector<LinearRow> linear;};
inline std::vector<std::string> csvsplit(std::string s,char sep=','){if(!s.empty()&&s.back()=='\r')s.pop_back();std::vector<std::string>v;std::stringstream in(s);std::string x;while(std::getline(in,x,sep))v.push_back(x);return v;}
inline double exact(const std::string&s){char*end;double x=strtod(s.c_str(),&end);if(*end)unsupported("HEX_INPUT");if(!std::isfinite(x))nonfinite("HEX_INPUT");return x;}
inline Data load(const std::string&dir){Data d;std::ifstream f(dir+"/exact_binary64.csv");if(!f)unsupported("INPUT_MISSING");std::string s;std::getline(f,s);std::map<RawKey,std::map<std::pair<int,int>,double>> tmp;
 while(std::getline(f,s)){auto v=csvsplit(s);if(v.size()!=7)unsupported("INPUT_COLUMNS");double x=exact(v[5]);uint64_t b;memcpy(&b,&x,8);if(b!=std::stoull(v[6],nullptr,16))unsupported("BITS_MISMATCH");if(!tmp[{v[0],std::stoull(v[1]),v[2]}].emplace(std::make_pair(std::stoi(v[3]),std::stoi(v[4])),x).second)unsupported("DUPLICATE_INPUT");}
 for(auto&kv:tmp){int nr=0,nc=0;for(auto&x:kv.second){nr=std::max(nr,x.first.first+1);nc=std::max(nc,x.first.second+1);}if(kv.second.size()!=size_t(nr*nc))unsupported("MATRIX_HOLES");DMat m(nr,std::vector<double>(nc));for(auto&x:kv.second)m[x.first.first][x.first.second]=x.second;d.raw.emplace(kv.first,std::move(m));}
 f.close();f.open(dir+"/factors.csv");if(!f)unsupported("FACTOR_INPUT_MISSING");std::getline(f,s);while(std::getline(f,s)){auto v=csvsplit(s);if(v.size()!=4)unsupported("FACTOR_COLUMNS");Factor a{std::stoull(v[0]),v[1],{},std::stoull(v[3])};for(auto&k:csvsplit(v[2],';'))a.keys.push_back(csvsplit(k,':')[0]);if(a.index!=d.factors.size())unsupported("FACTOR_ORDER");d.factors.push_back(a);}
 f.close();f.open(dir+"/linear.csv");if(!f)unsupported("LINEAR_INPUT_MISSING");std::getline(f,s);std::map<std::pair<size_t,size_t>,LinearRow> rows;
 while(std::getline(f,s)){auto v=csvsplit(s);if(v.size()!=7)unsupported("LINEAR_COLUMNS");auto key=std::make_pair(std::stoull(v[0]),std::stoull(v[1]));double r=exact(v[6]);auto it=rows.find(key);if(it==rows.end())it=rows.emplace(key,LinearRow{key.first,key.second,{},r}).first;else if(it->second.r!=r)unsupported("LINEAR_R_MISMATCH");it->second.ad.emplace_back(exact(v[4]),exact(v[5]));}
 for(auto&kv:rows)d.linear.push_back(std::move(kv.second));return d;
}
struct Evaluator {
 const Raw& raw;std::map<RawKey,Mat> cache;std::vector<std::string> branches;
 explicit Evaluator(const Raw&r):raw(r){}
 const Mat& mat(const std::string&p,size_t i,const std::string&n){RawKey k{p,i,n};auto it=cache.find(k);if(it!=cache.end())return it->second;auto in=raw.find(k);if(in==raw.end())unsupported("MISSING_MATRIX_"+p+"_"+n);Mat m;for(auto&r:in->second){Vec v;for(double x:r)v.emplace_back(x);m.push_back(std::move(v));}return cache.emplace(k,std::move(m)).first->second;}
 Vec vec(const std::string&p,size_t i,const std::string&n){Vec v;for(auto&r:mat(p,i,n))v.push_back(r.at(0));return v;}
 I scalar(const std::string&p,size_t i,const std::string&n){return mat(p,i,n).at(0).at(0);}
 void branch(const std::string&p,size_t i,const std::string&n,int b){auto it=raw.find({p,i,n});if(it==raw.end()||it->second.at(0).at(0)!=b)unsupported("NATIVE_BRANCH_MISMATCH_"+n);branches.push_back(p+","+std::to_string(i)+","+n+","+std::to_string(b));}
 Vec log(const Mat&R,const std::string&p,size_t i,const std::string&prefix){I trace=R[0][0]+R[1][1]+R[2][2],t3=trace-3;if(less(trace+1,scalar("CONST",0,"log_near_pi_threshold")))unsupported("NEAR_PI");I mag;int b;if(less(t3,scalar("CONST",0,"log_near_zero_threshold"))){I theta=fun((trace-1)/2,"acos");mag=theta/(2*fun(theta,"sin"));b=2;}else{mag=I(.5)-t3/12+t3*t3/60;b=1;}branch(p,i,prefix+"_branch",b);return scale(mag,{R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]});}
 Mat exp(const Vec&w,const std::string&p,size_t i){I t2=dot(w,w);bool near=less(t2,scalar("CONST",0,"epsilon"),true);branch(p,i,"exp_near_zero",near);Mat W=skew(w),eye={{1,0,0},{0,1,0},{0,0,1}};if(near)return madd(eye,W);I theta=fun(t2,"sqrt");Mat K=ms(I(1)/theta,W),KK=mm(K,K);I s=fun(theta/2,"sin");return madd(madd(eye,ms(fun(theta,"sin"),K)),ms(2*s*s,KK));}
 Vec residual(const Factor&f,const std::string&p){size_t i=f.index;auto state=[&](size_t j)->const Mat&{auto&k=f.keys.at(j);return mat(p,std::stoull(k.substr(1)),std::string(1,std::toupper(k[0])));};auto vstate=[&](size_t j){Vec v;for(auto&r:state(j))v.push_back(r[0]);return v;};auto c=[&](const std::string&n){return vec("CONST",i,n);};
 if(f.type=="RANGE"){auto&X=state(0);Vec d=sub(add(mv(rot(X),c("lever")),pos(X)),c("anchor"));return {norm(d)+scalar("CONST",i,"beta")-scalar("CONST",i,"measurement")};}
 if(f.type=="V_PRIOR"||f.type=="B_PRIOR")return sub(vstate(0),c("prior"));
 if(f.type=="POSE_PRIOR"){auto&X=state(0);auto&P=mat("CONST",i,"prior");Mat R=mm(tr(rot(X)),rot(P));Vec T=mv(tr(rot(X)),sub(pos(P),pos(X)));Vec w=log(R,p,i,"pose_log");I theta=norm(w);bool small=less(theta,scalar("CONST",0,"pose_log_threshold"));branch(p,i,"pose_log_small",small);Vec u=T;if(!small){Mat W=skew(scale(I(1)/theta,w));Vec WT=mv(W,T);I ta=fun(theta/2,"tan");u=add(sub(T,scale(theta/2,WT)),scale(1-theta/(2*ta),mv(W,WT)));}return scale(-1,cat(w,u));}
 if(f.type=="IMU"){auto&Xi=state(0);Vec vi=vstate(1);auto&Xj=state(2);Vec vj=vstate(3),bi=vstate(4),bj=vstate(5);Mat Ri=rot(Xi),Rj=rot(Xj);Vec bd=sub(bi,c("bias_hat"));Vec bc=add(add(c("pim"),mv(mat("CONST",i,"H_bias_acc"),slice(bd,0,3))),mv(mat("CONST",i,"H_bias_gyro"),slice(bd,3,6)));I dt=scalar("CONST",i,"dt"),dt22=I(.5)*dt*dt;Vec g=c("gravity");Vec xi=cat(cat(slice(bc,0,3),add(add(slice(bc,3,6),scale(dt,mv(tr(Ri),vi))),scale(dt22,mv(tr(Ri),g)))),add(slice(bc,6,9),scale(dt,mv(tr(Ri),g))));Mat Rp=mm(Ri,exp(slice(xi,0,3),p,i));Vec pp=add(pos(Xi),mv(Ri,slice(xi,3,6))),vp=add(vi,mv(Ri,slice(xi,6,9)));Vec w=log(mm(tr(Rj),Rp),p,i,"imu_log");return cat(cat(cat(w,mv(tr(Rj),sub(pp,pos(Xj)))),mv(tr(Rj),sub(vp,vj))),sub(bi,bj));}
 unsupported("FACTOR_"+f.type);
 }
};
struct Certificate {I P,D;Decision decision;std::vector<I> factors;std::vector<std::string> branches;};
inline Certificate certify(const Data&d){Certificate c;Evaluator ev(d.raw);for(auto&f:d.factors){Vec a=ev.residual(f,"BASE"),b=ev.residual(f,"TRIAL");if(a.size()!=f.dim||b.size()!=f.dim)unsupported("FACTOR_RESIDUAL_DIM");auto&R=ev.mat("CONST",f.index,"whitening_R");I df=paired(mv(R,a),mv(R,b));c.factors.push_back(df);c.D=c.D+df;}
 for(auto&r:d.linear){I v;for(auto&ad:r.ad)v=v+I(ad.first)*I(ad.second);c.P=c.P-I(r.r)*v-I(.5)*v*v;}
 c.decision=choose(c.P,c.D);c.branches=std::move(ev.branches);return c;
}
inline void write(const std::string&path,const Certificate&c){std::ofstream f(path);if(!f)unsupported("OUTPUT");f<<std::setprecision(17)<<"{\"policy\":\"PAPER_CERTIFIED_PAIR_REDUCTION_V1\",\"implementation\":\"A18_CPP_MPFR_333_V1\",\"bits\":333,\"P_lo\":\""<<c.P.lo.str(DOWN)<<"\",\"P_hi\":\""<<c.P.hi.str(UP)<<"\",\"D_lo\":\""<<c.D.lo.str(DOWN)<<"\",\"D_hi\":\""<<c.D.hi.str(UP)<<"\",\"status\":\""<<c.decision.status<<"\",\"reason\":\""<<c.decision.reason<<"\",\"fidelity_lo\":\""<<c.decision.ratio.lo.str(DOWN)<<"\",\"fidelity_hi\":\""<<c.decision.ratio.hi.str(UP)<<"\",\"fidelity_binary64\":"<<c.decision.fidelity<<",\"factor_count\":"<<c.factors.size()<<",\"branch_count\":"<<c.branches.size()<<",\"P_lo_binary\":\""<<c.P.lo.binary()<<"\",\"P_hi_binary\":\""<<c.P.hi.binary()<<"\",\"D_lo_binary\":\""<<c.D.lo.binary()<<"\",\"D_hi_binary\":\""<<c.D.hi.binary()<<"\",\"fidelity_lo_binary\":\""<<c.decision.ratio.lo.binary()<<"\",\"fidelity_hi_binary\":\""<<c.decision.ratio.hi.binary()<<"\"}\n";}
inline void writeDetails(const std::string&dir,const Certificate&c){std::ofstream f(dir+"/factors.csv");f<<"factor,D_lo,D_hi\n";for(size_t i=0;i<c.factors.size();++i)f<<i<<','<<c.factors[i].lo.str(DOWN)<<','<<c.factors[i].hi.str(UP)<<'\n';std::ofstream b(dir+"/branches.csv");b<<"phase,factor,name,branch\n";for(auto&s:c.branches)b<<s<<'\n';}
}
