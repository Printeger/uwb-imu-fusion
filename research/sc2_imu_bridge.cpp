#include "sc2_imu_bridge.hpp"
#include <boost/numeric/interval.hpp>
namespace uwb_imu_pl::research {
using I=boost::numeric::interval<double>;
using V=std::array<I,3>;using M=std::array<V,3>;
I norm(V v){I s(0);for(auto x:v)s+=square(x);return sqrt(s);}
M rotation(const Eigen::Quaterniond& q){I w(q.w()),x(q.x()),y(q.y()),z(q.z());I n=sqrt(square(w)+square(x)+square(y)+square(z));if(n.lower()<=0)throw std::runtime_error("SC2 invalid initial rotation");w/=n;x/=n;y/=n;z/=n;return {{{1.-2.*(square(y)+square(z)),2.*(x*y-z*w),2.*(x*z+y*w)},{2.*(x*y+z*w),1.-2.*(square(x)+square(z)),2.*(y*z-x*w)},{2.*(x*z-y*w),2.*(y*z+x*w),1.-2.*(square(x)+square(y))}}};}
Sc2IndependentBridge::Sc2IndependentBridge(const NavigationState& initial,const RawInclusionParameters& law,Sc2ImuScope scope):initial_(initial),law_(law),scope_(scope){
 Sc2ImuMonitor check(initial,law,scope);
 if(!initial.position_world_m.allFinite() || !initial.velocity_world_mps.allFinite() || !initial.q_world_body.coeffs().allFinite() || !law.numeric.bias_process.allFinite() || (law.numeric.bias_process.array()<0).any() || !law.numeric.velocity.allFinite() || (law.numeric.velocity.array()<0).any() || !std::isfinite(law.numeric.position+law.numeric.rotation_matrix) || law.numeric.position<0 || law.numeric.rotation_matrix<0 || !std::isfinite(law.risk_budget) || law.risk_budget<std::ldexp(2.*6421,-35))throw std::runtime_error("SC2 invalid initial bridge source");
 // Validate independent physical domain from the registered analytic motion
 // parameters, not a truth error or a measured force corrupted by a fault.
 V acceleration;for(int j=0;j<3;++j)acceleration[j]=abs(I(law.motion_amplitude(j)))*square(I(law.motion_frequency(j)));
 if(!law.motion_amplitude.allFinite() || !law.motion_frequency.allFinite() || !std::isfinite(law.motion_angular_rate) || norm(acceleration).upper()>scope.acceleration_bound || std::abs(law.motion_angular_rate)>scope.angular_rate_bound)throw std::runtime_error("SC2 physical motion domain refused");
 rotation(initial.q_world_body);
}
Sc2StateEnvelope Sc2IndependentBridge::envelope(int epoch,const Eigen::Vector3d& lever)const{
 Sc2StateEnvelope e;if(epoch<1 || epoch>50 || !lever.allFinite())return e;
 auto r=rotation(initial_.q_world_body);I t=I(double(epoch))/20.;V rho,phi,l;
 for(int j=0;j<3;++j){rho[j]=7.*I(law_.initial_sigmas(j+3));phi[j]=7.*I(law_.initial_sigmas(j));l[j]=I(lever(j));}
 I p_error=norm(rho)+I(law_.numeric.position),angle=norm(phi)+I(law_.numeric.rotation_matrix)+I(scope_.angular_rate_bound)*t;
 double diameter=std::min(2.,angle.upper());I lever_error=I(diameter)*norm(l);
 for(int j=0;j<3;++j){I v_error=7.*I(law_.initial_sigmas(j+6))+I(law_.numeric.velocity(j));I velocity=I(initial_.velocity_world_mps(j))+I(-v_error.upper(),v_error.upper());I physical_position=I(scope_.acceleration_bound)*square(t)/2.;I physical_velocity=I(scope_.acceleration_bound)*t;
  I position=I(initial_.position_world_m(j))+I(-p_error.upper(),p_error.upper())+t*velocity+I(-physical_position.upper(),physical_position.upper());
  I terminal_velocity=velocity+I(-physical_velocity.upper(),physical_velocity.upper());
  e.position_low(j)=position.lower();e.position_high(j)=position.upper();e.velocity_low(j)=terminal_velocity.lower();e.velocity_high(j)=terminal_velocity.upper();
  I tag=position;for(int k=0;k<3;++k)tag+=r[j][k]*l[k];tag+=I(-lever_error.upper(),lever_error.upper());e.tag_low(j)=tag.lower();e.tag_high(j)=tag.upper();
  for(int k=0;k<3;++k){I rotation_entry=r[j][k]+I(-diameter,diameter);e.rotation_low(j,k)=std::max(-1.,rotation_entry.lower());e.rotation_high(j,k)=std::min(1.,rotation_entry.upper());}
 }
 for(int axis=0;axis<6;++axis){I error=7.*(I(law_.initial_sigmas(axis+9))+double(epoch*10)*I(axis<3?law_.accel_rw:law_.gyro_rw)*sqrt(I(1)/200.))+I(law_.numeric.bias(axis))+I(law_.numeric.bias_process(axis));double observation=axis<3?initial_.accel_bias_mps2(axis):initial_.gyro_bias_radps(axis-3);I bias=I(observation)+I(-error.upper(),error.upper());e.bias_low(axis)=bias.lower();e.bias_high(axis)=bias.upper();}
 e.epoch=epoch;e.time_s=double(epoch)/20.;e.rotation_diameter=diameter;e.event_risk=std::ldexp(2.*6421,-35);
 e.valid=e.position_low.allFinite() && e.position_high.allFinite() && e.velocity_low.allFinite() && e.velocity_high.allFinite() && e.rotation_low.allFinite() && e.rotation_high.allFinite() && e.bias_low.allFinite() && e.bias_high.allFinite() && e.tag_low.allFinite() && e.tag_high.allFinite();return e;
}
RawRangeResult Sc2IndependentBridge::issue(const Sc2ImuMonitor& monitor,const RawRangeLawEvidence& range,const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever)const{
 RawRangeResult refused;refused.source="raw-imu-fault-independent-bridge-uwb-v1";refused.relaxation="ALL_HEALTHY_UWB_SC2_SINGLE_IMU_EPISODE";
 auto a=monitor.assessment();std::array<RawRangeShell,8> shells;
 if(!a.valid || (!a.nominal_possible && !a.possible) || !monitor.matches(tx) || tx.previous_epoch+1!=tx.proposed_epoch || tx.begin.value()!=(static_cast<long long>(tx.proposed_epoch)-1)*50000000LL || tx.end.value()!=static_cast<long long>(tx.proposed_epoch)*50000000LL || receipt.transaction_id!=tx.id || receipt.committed_epoch!=tx.proposed_epoch || receipt.graph_version!=tx.base_graph_version+1 || receipt.backend_updates!=1 || receipt.state_timestamp!=tx.end || state.timestamp!=tx.end || !rawRangeShells(range,tx.uwb_batch,tx.proposed_epoch,shells)){refused.reason="SC2 source/scope/prefix/batch/receipt refused";return refused;}
 auto e=envelope(tx.proposed_epoch,lever);if(!e.valid)return refused;
 Eigen::Vector3d actual=state.position_world_m+state.q_world_body*lever;
 auto r=contractRawRangeBox(e.tag_low,e.tag_high,tx.uwb_batch,shells,actual,{256,8,0,true});r.source=refused.source;r.relaxation=refused.relaxation;r.event_risk=e.event_risk;return r;
}
}
