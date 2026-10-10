#pragma once
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <vector>
#include <boost/math/distributions/normal.hpp>
#include <cmath>
#include <string>
#include <stdexcept>
namespace uwb_imu_pl::research {
inline double upper(double x){return std::nextafter(x,INFINITY);}
// One predetermined coordinate event covers every raw ADC/range and initial
// coordinate in the entire horizon, and hence arbitrary adaptive references.
// No assumption of Gaussian noise conditional on a fitted root is made.
struct RawNoiseEvent {
 int coordinates; double risk,q;
 static double coordinateRadius(int n,double delta){
  if(n<=0 || !(delta>0 && delta<1))throw std::runtime_error("invalid raw event");
  return upper(boost::math::quantile(boost::math::complement(boost::math::normal(),delta/(2*n))));
 }
 RawNoiseEvent(int n,double delta):coordinates(n),risk(delta),
   q(coordinateRadius(n,delta)){
  if(n<=0 || !(delta>0 && delta<1))throw std::runtime_error("invalid raw event");
 }
};
struct TagRemainder {bool valid=false;double radius=INFINITY,pose=INFINITY,range=INFINITY;};
// GTSAM Pose3 EXPMAP right perturbation. Translation is R V(phi) rho;
// ||V(phi)-I|| <= theta/2; Exp rotation remainder <= theta^2/2.
// Euclidean tag displacement <= ||rho|| + min(theta,2)||lever||.
inline TagRemainder tagRemainder(double distance,double theta,double rho,double lever){
 TagRemainder r;if(!(distance>0 && theta>=0 && theta<3.141592653589793 && rho>=0 && lever>=0))return r;
 r.radius=upper(rho+std::min(theta,2.)*lever);
 r.pose=upper(.5*theta*rho+.5*theta*theta*lever);
 if(r.radius>=distance)return r;
 r.range=upper(r.pose+r.radius*r.radius/(2*(distance-r.radius)));r.valid=true;return r;
}
struct NavigationDomain {
 double position,velocity,rotation,accel_bias,gyro_bias;
};
struct RangeDomainBranch {
 int excluded_anchor=-1;
 Eigen::Vector3d center=Eigen::Vector3d::Constant(NAN),box=Eigen::Vector3d::Constant(INFINITY);
 bool valid=false;
};
// Same raw event, no new measurement or Gaussian prior. For any branch whose
// retained anchors are healthy, squared-range differences give A*t=b+error,
// |error_i| <= (2|r_i|eps+eps^2)+(2|r_ref|eps+eps^2).
// All one-anchor exclusions are retained: choosing the smallest fitted box
// would silently assume the fault identity. This bounds the world TAG only;
// it does not identify attitude, velocity, bias or a Combined-factor domain.
inline std::vector<RangeDomainBranch> rangeDomainBranches(
 const std::vector<Eigen::Vector3d>& anchors,const std::vector<double>& ranges,double eps){
 std::vector<RangeDomainBranch> out;
 if(anchors.size()!=ranges.size() || anchors.size()<5 || !std::isfinite(eps) || eps<0)return out;
 for(size_t omit=0;omit<anchors.size();++omit){
  RangeDomainBranch branch;branch.excluded_anchor=int(omit);
  std::vector<int> rows;for(size_t i=0;i<anchors.size();++i)if(i!=omit)rows.push_back(int(i));
  const int ref=rows.front();Eigen::MatrixXd a(rows.size()-1,3);Eigen::VectorXd b(rows.size()-1),error(rows.size()-1);
  bool finite=true;
  for(size_t k=1;k<rows.size();++k){int i=rows[k];
   finite=finite && anchors[i].allFinite() && anchors[ref].allFinite() && std::isfinite(ranges[i]) && std::isfinite(ranges[ref]);
   a.row(k-1)=2*(anchors[i]-anchors[ref]).transpose();
   b(k-1)=anchors[i].squaredNorm()-anchors[ref].squaredNorm()-ranges[i]*ranges[i]+ranges[ref]*ranges[ref];
   error(k-1)=upper(2*(std::abs(ranges[i])+std::abs(ranges[ref]))*eps+2*eps*eps);
  }
  if(finite){Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);qr.setThreshold(1e-12);
   if(qr.rank()==3){auto inverse=qr.solve(Eigen::MatrixXd::Identity(a.rows(),a.rows())).eval();
    branch.center=inverse*b;branch.box=inverse.cwiseAbs()*error;
    branch.valid=branch.center.allFinite() && branch.box.allFinite();}}
  out.push_back(branch);
 }
 return out;
}
// Domain centered on the *observed* initial constant-velocity path. Physical
// world acceleration/angle-motion bounds are declared generator assumptions.
// It does not claim an algorithm's fitted trajectory is inside this ball.
inline NavigationDomain physicalDomain(double t,const Eigen::Matrix<double,15,1>& sigmas,
 const RawNoiseEvent& event,double accel,double rate){
 double k=event.q*std::sqrt(3.);
 double theta=k*sigmas.head<3>().maxCoeff();
 double p=k*sigmas.segment<3>(3).maxCoeff();
 double v=k*sigmas.segment<3>(6).maxCoeff();
 return {upper(p+t*v+.5*accel*t*t),upper(v+accel*t),upper(theta+rate*t),
  upper(k*sigmas.segment<3>(9).maxCoeff()),upper(k*sigmas.tail<3>().maxCoeff())};
}
// Full predetermined family charge, not number of visited fitted roots.
// Overflow is a refusal. This guard can reject an unusable covering family
// before enumerating it; it never grants adaptive Gaussian qualification.
inline double fixedReferenceFamilyCharge(double radius,double grid_spacing,
 int horizon,int candidates,double tail){
 if(radius<0 || grid_spacing<=0 || horizon<=0 || candidates<=0)return INFINITY;
 long double side=2*std::ceil(radius/grid_spacing)+1;
 long double count=side*side*side; // position only, so a LOWER family charge
 return upper(double(2*tail*horizon*candidates*count));
}
struct PimRemainder {
 double rotation,position,velocity;
};
// Uniform discrete rotation/force integration bound: for sample perturbations
// ||delta omega||<=g and ||delta force||<=a, product-of-rotations first variation
// has quadratic remainder <= (t*g)^2/2. Velocity includes gyro/force cross
// coupling; position includes the full double integral. World gravity is fixed
// and cancels here. This does not certify Combined factor chart/bias correction.
inline PimRemainder healthyIntegrationRemainder(double t,double gyro_radius,
 double accel_radius,double force_norm){
 if(!std::isfinite(t) || !std::isfinite(gyro_radius) || !std::isfinite(accel_radius) || !std::isfinite(force_norm) || t<0 || gyro_radius<0 || accel_radius<0 || force_norm<0)
  throw std::runtime_error("invalid PIM domain");
 long double theta=(long double)t*gyro_radius;
 long double force_rem=.5L*theta*theta*force_norm+theta*accel_radius;
 return {upper(double(.5L*theta*theta)),upper(double(.5L*t*t*force_rem)),upper(double(t*force_rem))};
}
struct RawDomainAssessment {
 std::string status="INITIAL_LAW_MISSING";
 double range_remainder=INFINITY,tag_radius=INFINITY,event_risk=0;
 bool initial_law_valid=false,adaptive_event_valid=false,range_domain_valid=false;
 bool raw_conditional_valid=false; // healthy PIM/history uniform bounds pending
};
}
