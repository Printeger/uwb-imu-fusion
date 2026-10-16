#include "physical_time.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
void require(bool v,const char* m){if(!v)throw std::runtime_error(m);}
template<class F>void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected refusal");}
bool contains(I x,long double point){return (long double)x.lower()<=point && point<=(long double)x.upper();}
int main(){try{
 for(auto pair:std::vector<std::pair<int64_t,int64_t>>{{123456789000LL,123469289007LL},{-100,1},{INT64_MIN,INT64_MAX}}){
  const auto d=durationSeconds(TimestampNs(pair.first),TimestampNs(pair.second));
  const long double exact=((long double)pair.second-(long double)pair.first)/1000000000.L;
  require(contains(d,exact),"nanosecond conversion underbound");
 }
 refuses([]{durationSeconds(TimestampNs(1),TimestampNs(1));});
 for(int64_t hns:{1LL,7123456LL,50321098LL,141123456LL,2000000000LL}){
  const int64_t start=123456789000LL;const long double h=(long double)hns/1e9L;
  RollingRawState old;
  for(int j=0;j<3;++j){old.p[j]=I(j);old.v[j]=I(j-1);for(int k=0;k<3;++k)old.rotation[j][k]=I(j==k?1:0);}
  for(auto& bias:old.bias)bias=I(0);
  std::vector<TimestampNs> births{TimestampNs(start+hns)};
  std::array<double,6> q;q.fill(7);
  auto step=rawTimeStep(TimestampNs(start),TimestampNs(start+hns),TimestampNs(start),births,{q},.01,.001);
  auto next=rollingPropagate(old,2.,.5,step);
  // Independent analytic constant-acceleration and sample-grid RW histories.
  for(int sign:{-1,1})for(int j=0;j<3;++j){
   const long double a=sign*1.L;const long double p=j+(j-1)*h+a*h*h/2,v=j-1+a*h;
   require(contains(next.p[j],p) && contains(next.v[j],v),"nonuniform physics lost trajectory");
   require(contains(next.bias[j],sign*.01L*7*std::sqrt(h)),"RW support lost truth");
  }
  auto a=old,b=next;
  require(rollingContractDynamics(a,b,2.,step.duration) && rollingContractCoupledDynamics(a,b,2.,step.duration),"valid dynamic relation rejected");
  for(int j=0;j<3;++j)require(contains(b.p[j],j+(j-1)*h+h*h/2),"contractor removed analytic history");
  const Eigen::Vector3d lever(.1,-.025,0);
  auto transport=rangeTransportRadius(next,TimestampNs(start),TimestampNs(start+hns),lever,2.,.5);
  require(transport.upper()>=0,"negative transport");
  require(rangeTransportRadius(next,TimestampNs(start+hns),TimestampNs(start+hns),lever,2.,.5).upper()==0,"zero-span discrepancy");
 }
 // Irregular jump grid with a held tail: sum actual source sqrt(dt), no
 // Brownian increment for the artificial end stamp. Shared boundary reuse
 // uses the previous RAW time, which can precede this processing interval.
 std::array<double,6> q;q.fill(7);
 auto irregular=rawTimeStep(TimestampNs(1000000000),TimestampNs(1050000000),TimestampNs(997000000),
  {TimestampNs(1011000000),TimestampNs(1024000000),TimestampNs(1036000000)},{q,q,q},.01,.001);
 long double exact=.07L*(std::sqrt(.014L)+std::sqrt(.013L)+std::sqrt(.012L));
 require(contains(irregular.bias_radius[0],exact),"shared-parent RW time lost");
 auto held=rawTimeStep(TimestampNs(1000000000),TimestampNs(1010000000),TimestampNs(997000000),{},{},.01,.001);
 require(held.bias_radius[0].upper()==0 && held.duration.lower()>0,"hold minted random source");
 refuses([&]{rawTimeStep(TimestampNs(0),TimestampNs(100),TimestampNs(0),{TimestampNs(50),TimestampNs(50)},{q,q},.01,.001);});
 refuses([&]{rawTimeStep(TimestampNs(0),TimestampNs(100),TimestampNs(1),{},{},.01,.001);});
 // Independent rotating-lever range-transport oracle. Evaluate real points
 // at each measurement's time; endpoint v support is supplied, never assumed.
 RollingRawState endpoint;for(auto& v:endpoint.v)v=I(-2,2);
 const Eigen::Vector3d lever(.3,-.1,.2),anchor(2,-3,1);
 for(int k=1;k<=100;++k){double h=k*.001;Eigen::Vector3d v(.5,-.3,.2),acc(.4,.2,-.1),p(1,2,3);
  auto r0=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ());auto r1=Eigen::AngleAxisd(.7+.4*h,Eigen::Vector3d::UnitZ());
  auto end=p+h*v+.5*h*h*acc;
  double range_change=std::abs((end+r1*lever-anchor).norm()-(p+r0*lever-anchor).norm());
  auto radius=rangeTransportRadius(endpoint,TimestampNs(5000000000LL),TimestampNs(5000000000LL+k*1000000LL),lever,1.,.4);
  require(range_change<=radius.upper(),"asynchronous range transport underbound");
 }
 std::cout<<"SC9 time PASS actual ns/overflow, nonuniform dynamics/RW, shared/hold source, analytic trajectories and asynchronous rotating lever oracle\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
