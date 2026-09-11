#pragma once
#include <filesystem>
#include <functional>
#include "a18_live_graph.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <gtsam/linear/linearExceptions.h>
namespace fs=std::filesystem;
// Explicit diagnostic I/O only; never participates in numerical decisions.
inline bool compact_diagnostic_output = false;
struct Reply {std::string status;double fidelity=0;};
class CertifiedLm:public LevenbergMarquardtOptimizer {
 using State=gtsam::internal::LevenbergMarquardtState;
 a18::LiveGraph live_;std::function<void()> fault_;std::string out_;std::ofstream trials_;uifgo::NavigationScales scales_;
 public:
 size_t calls=0,trials=0,solves=0,accepted=0,rejected=0,unresolved=0;
 std::string failure; double certificate_seconds=0;
 std::vector<uifgo::InexactHandoffTrialAudit> last_search;
 CertifiedLm(const NonlinearFactorGraph&g,const Values&v,const LevenbergMarquardtParams&p,const std::vector<uifgo::DevelopmentRangeConstant>& ranges,const std::string&out,const uifgo::NavigationScales&scales,std::function<void()> fault={}):LevenbergMarquardtOptimizer(g,v,p),live_(g,ranges),fault_(std::move(fault)),out_(out),trials_(output(out+"/trials.csv")),scales_(scales){
  if(p.diagonalDamping||!p.useFixedLambdaFactor||p.lambdaFactor!=10||p.lambdaUpperBound!=1e5||p.lambdaLowerBound!=0||p.minModelFidelity!=1e-3||p.relativeErrorTol!=0||p.getLinearSolverType()!="SEQUENTIAL_CHOLESKY")throw std::runtime_error("A17_UNSUPPORTED_LM_PARAMETERS");
  trials_<<"call,trial_in_call,total_trial,lambda_before,lambda_after,delta_norm,error_before,error_native_trial,status,fidelity_lower_binary64,accepted_total,rejected_total,unresolved_total,elapsed_s,artifact\n";
 }
 CertifiedLm(const NonlinearFactorGraph&g,const Values&v,const LevenbergMarquardtParams&p,const std::vector<uifgo::DevelopmentRangeConstant>& ranges,const std::string&out,std::function<void()> fault={}):CertifiedLm(g,v,p,ranges,out,uifgo::NavigationScales{},std::move(fault)){}
 VectorValues fixedDirection(const GaussianFactorGraph&linear){++solves;return solve(buildDampedSystem(linear,VectorValues()),params_);}
 GaussianFactorGraph::shared_ptr iterate()override{
  ++calls;last_search.clear();auto linear=linearize();size_t local=0;
  while(true){++local;++trials;auto*s=static_cast<State*>(state_.get());double lb=s->lambda,eb=error();auto begin=std::chrono::steady_clock::now();std::string dir=out_+"/call"+std::to_string(calls)+"_trial"+std::to_string(local);fs::create_directory(dir);Reply reply;double dn=std::numeric_limits<double>::quiet_NaN(),et=dn;
   try{
    auto delta=fixedDirection(*linear);dn=delta.norm();const Values base=values();Values trial=base.retract(delta);et=graph_.error(trial);
    if(!std::isfinite(dn)||!std::isfinite(et))throw std::runtime_error("NUMERIC_REFERENCE_NONFINITE:NATIVE_TRIAL");
    if(fault_)fault_();
    auto cb=std::chrono::steady_clock::now();
    auto data=live_.pair(base,trial,delta,*linear);
	    auto cert=a18::certify(data);
    certificate_seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-cb).count();
    if (!compact_diagnostic_output) { a18::dump(dir,data);writeLinear(dir,*linear,delta); }
    a18::write(dir+"/certificate.json",cert);
    if (!compact_diagnostic_output) {auto detail=dir+"/certificate_details";fs::create_directory(detail);a18::writeDetails(detail,cert);}
	    reply.status=cert.decision.status;reply.fidelity=cert.decision.fidelity;
	    uifgo::InexactHandoffTrialAudit guard;guard.trial_index=local;guard.certificate_valid=true;guard.certificate_status=reply.status;guard.predicted_lo=cert.P.lo.d(a18::DOWN);guard.predicted_hi=cert.P.hi.d(a18::UP);guard.actual_decrease_lo=cert.D.lo.d(a18::DOWN);guard.actual_decrease_hi=cert.D.hi.d(a18::UP);guard.scaled_navigation_step=uifgo::MaxScaledValuesStep(base,trial,scales_,1.0);last_search.push_back(std::move(guard));
    if(reply.status!="ACCEPT"&&reply.status!="REJECT")failure=reply.status+":"+cert.decision.reason;
    if(reply.status=="ACCEPT"){
     // Worker certifies RNDD conversion too. Do not recalculate the quotient.
     if(!(reply.fidelity>params_.minModelFidelity))throw std::runtime_error("CERTIFICATE_FIDELITY_CONVERSION_INVALID");
     state_=s->decreaseLambda(params_,reply.fidelity,std::move(trial),et);++accepted;
    }else if(reply.status=="REJECT"){s->increaseLambda(params_);++rejected;}
    else{++unresolved;if(failure.empty())failure=reply.status;}
	   }catch(const IndeterminantLinearSystemException&){reply.status="LINEAR_SOLVE_FAILED";uifgo::InexactHandoffTrialAudit guard;guard.trial_index=local;guard.certificate_status=reply.status;last_search.push_back(std::move(guard));s->increaseLambda(params_);++rejected;}
	   catch(const std::exception&e){reply.status=a18::exceptionStatus(e);uifgo::InexactHandoffTrialAudit guard;guard.trial_index=local;guard.certificate_status=reply.status;last_search.push_back(std::move(guard));failure=reply.status+":"+e.what();++unresolved;}
   double secs=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
   trials_<<calls<<','<<local<<','<<trials<<','<<lb<<','<<lambda()<<','<<dn<<','<<eb<<','<<et<<','<<reply.status<<','<<reply.fidelity<<','<<accepted<<','<<rejected<<','<<unresolved<<','<<secs<<','<<fs::path(dir).filename().string()<<'\n';trials_.flush();
   if(!failure.empty())return linear;
   if(reply.status=="ACCEPT")return linear;
   if(lambda()>=params_.lambdaUpperBound){failure="CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED";return linear;}
  }
 }
};

inline std::string r03ValuesIdentity(const Values& v) {std::ostringstream bytes;bytes<<"phase,index,name,row,column,hex,bits\n";values(bytes,"R03_LAST_ACCEPTED",v);return "r03values-sha256:"+uifgo::Sha256Hex(bytes.str());}

inline bool r03InexactHandoffGuard(const uifgo::InexactHandoffAudit& h,const std::string& failure,size_t unresolved,bool state_and_metadata_valid) {
 bool trials_ok=!h.last_lambda_search_trials.empty();
 for(const auto&t:h.last_lambda_search_trials)
  trials_ok=trials_ok&&t.certificate_valid&&t.certificate_status=="REJECT"&&
      std::isfinite(t.predicted_lo)&&std::isfinite(t.predicted_hi)&&
      std::isfinite(t.actual_decrease_hi)&&
      std::isfinite(t.scaled_navigation_step)&&t.predicted_lo>0.0&&
      t.actual_decrease_hi<=0.0&&
      t.scaled_navigation_step<=h.scaled_step_tolerance&&
      t.predicted_hi<=h.objective_increase_allowance;
 const bool base_ok=failure=="CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED"&&
      h.accepted_update_count>0&&h.last_accepted_generic_convergence&&
      h.last_accepted_stationarity_valid&&
      std::isfinite(h.last_accepted_scaled_navigation_step)&&
      std::isfinite(h.scaled_step_tolerance)&&h.scaled_step_tolerance>=0.0&&
      h.last_accepted_scaled_navigation_step<=h.scaled_step_tolerance&&
      unresolved==0&&state_and_metadata_valid&&
      !h.last_accepted_values_identity.empty();
 return base_ok&&trials_ok;
}

inline std::string strictJsonNumberOrNull(double value) {
 if(!std::isfinite(value))return "null";
 std::ostringstream text;text<<std::setprecision(17)<<value;return text.str();
}

inline void writeInexactHandoffAudit(const std::string& path,bool inner_converged,const uifgo::InexactHandoffAudit& handoff,bool guard_evaluated,const std::string& guard_reason) {
 if(inner_converged&&handoff.qualified)throw std::runtime_error("HANDOFF_AUDIT_CONVERGED_AND_QUALIFIED");
 if(handoff.qualified&&(!guard_evaluated||handoff.handoff_count!=1))throw std::runtime_error("HANDOFF_AUDIT_QUALIFIED_STATE_INVALID");
 if(handoff.status=="GUARD_REJECTED"&&!guard_evaluated)throw std::runtime_error("HANDOFF_AUDIT_REJECTION_NOT_EVALUATED");
 if(!guard_evaluated&&(!handoff.last_lambda_search_trials.empty()||handoff.qualified))throw std::runtime_error("HANDOFF_AUDIT_UNEVALUATED_HAS_RESULT");
 if(guard_reason.empty())throw std::runtime_error("HANDOFF_AUDIT_REASON_EMPTY");
 if(handoff.qualified){
  if(!std::isfinite(handoff.last_accepted_scaled_navigation_step)||!std::isfinite(handoff.last_accepted_scaled_navigation_gradient)||!std::isfinite(handoff.last_accepted_gradient_roundoff_allowance)||!std::isfinite(handoff.scaled_step_tolerance)||!std::isfinite(handoff.objective_increase_allowance)||handoff.last_accepted_values_identity.empty())throw std::runtime_error("HANDOFF_AUDIT_QUALIFIED_NONFINITE_OR_INCOMPLETE");
  for(const auto&t:handoff.last_lambda_search_trials)if(!t.certificate_valid||!std::isfinite(t.predicted_lo)||!std::isfinite(t.predicted_hi)||!std::isfinite(t.actual_decrease_lo)||!std::isfinite(t.actual_decrease_hi)||!std::isfinite(t.scaled_navigation_step))throw std::runtime_error("HANDOFF_AUDIT_QUALIFIED_TRIAL_INVALID");
 }
 auto h=output(path);h<<std::setprecision(17)<<"{\"schema\":\"PAPER_STAGE2_INEXACT_HANDOFF_V1\",\"inner_converged\":"<<(inner_converged?"true":"false")<<",\"guard_evaluated\":"<<(guard_evaluated?"true":"false")<<",\"guard_evaluation_reason\":\""<<guard_reason<<"\",\"qualified\":"<<(handoff.qualified?"true":"false")<<",\"status\":\""<<handoff.status<<"\",\"trigger_reason\":\""<<handoff.trigger_reason<<"\",\"accepted_updates\":"<<handoff.accepted_update_count<<",\"last_accepted_generic\":"<<(handoff.last_accepted_generic_convergence?"true":"false")<<",\"last_accepted_stationarity_valid\":"<<(handoff.last_accepted_stationarity_valid?"true":"false")<<",\"last_accepted_step\":"<<strictJsonNumberOrNull(handoff.last_accepted_scaled_navigation_step)<<",\"last_accepted_gradient\":"<<strictJsonNumberOrNull(handoff.last_accepted_scaled_navigation_gradient)<<",\"gradient_roundoff\":"<<strictJsonNumberOrNull(handoff.last_accepted_gradient_roundoff_allowance)<<",\"step_tolerance\":"<<strictJsonNumberOrNull(handoff.scaled_step_tolerance)<<",\"objective_allowance\":"<<strictJsonNumberOrNull(handoff.objective_increase_allowance)<<",\"last_accepted_values_identity\":\""<<handoff.last_accepted_values_identity<<"\",\"handoff_count\":"<<handoff.handoff_count<<",\"trials\":[";for(size_t i=0;i<handoff.last_lambda_search_trials.size();++i){const auto&t=handoff.last_lambda_search_trials[i];h<<(i?",":"")<<"{\"trial\":"<<t.trial_index<<",\"certificate_valid\":"<<(t.certificate_valid?"true":"false")<<",\"status\":\""<<t.certificate_status<<"\",\"P_lo\":"<<strictJsonNumberOrNull(t.predicted_lo)<<",\"P_hi\":"<<strictJsonNumberOrNull(t.predicted_hi)<<",\"D_lo\":"<<strictJsonNumberOrNull(t.actual_decrease_lo)<<",\"D_hi\":"<<strictJsonNumberOrNull(t.actual_decrease_hi)<<",\"scaled_step\":"<<strictJsonNumberOrNull(t.scaled_navigation_step)<<"}";}h<<"]}\n";
}

inline uifgo::CheckedLmResult runCertified(size_t outer,const NonlinearFactorGraph&g,const Values&v,const uifgo::CheckedLmOptions&o,const std::vector<uifgo::DevelopmentRangeConstant>&ranges,const std::string&out,bool allow_inexact_handoff=false) {
 fs::create_directory(out);LevenbergMarquardtParams p;p.setMaxIterations(o.max_iterations);p.setRelativeErrorTol(0);p.setAbsoluteErrorTol(o.absolute_tolerance);p.setLinearSolverType("SEQUENTIAL_CHOLESKY");
 CertifiedLm lm(g,v,p,ranges,out,o.navigation_scales);uifgo::CheckedLmResult result;auto&c=result.convergence;c.policy_version=allow_inexact_handoff?"PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1":"PAPER_CERTIFIED_PAIR_REDUCTION_V1";c.stationarity_qualification_enabled=true;c.initial_error_valid=true;c.initial_error=lm.error();c.relative_tolerance=o.relative_tolerance;c.absolute_tolerance=o.absolute_tolerance;c.error_tolerance=p.errorTol;c.optimizer_internal_relative_tolerance=0;c.optimizer_internal_small_change_stop_enabled=false;
 auto&handoff=result.inexact_handoff;handoff.enabled=allow_inexact_handoff;handoff.status=allow_inexact_handoff?"ENABLED_NOT_TRIGGERED":"NOT_ENABLED";handoff.trigger_reason=handoff.status;handoff.scaled_step_tolerance=o.inexact_handoff_scaled_step_tolerance;
 auto calls=output(out+"/calls.csv");calls<<"call,error,lambda,trials,accepted,rejected,unresolved,generic,stationarity_valid,stationary,max_scaled_gradient,roundoff,certificate_s,reason\n";
 double prev=lm.error();result.reason="CONDITIONAL_LM_STATIONARITY_NOT_REACHED";
 bool last_accepted_generic=false;bool last_accepted_stationarity_valid=false;double last_accepted_step=std::numeric_limits<double>::quiet_NaN(),last_accepted_gradient=std::numeric_limits<double>::quiet_NaN(),last_accepted_roundoff=std::numeric_limits<double>::quiet_NaN();std::string last_accepted_identity;
 for(int i=0;i<o.max_iterations;++i){const Values before=lm.values();const size_t accepted_before=lm.accepted;lm.iterate();auto a=uifgo::AuditNavigationStationarity(g,lm.values(),o.navigation_scales,o.navigation_stationarity_tolerance_objective,o.gradient_roundoff_safety_factor);result.last_qualification_stationarity=a;bool generic=false;
  if(lm.failure.empty()){generic=checkConvergence(o.relative_tolerance,o.absolute_tolerance,p.errorTol,prev,lm.error());++c.convergence_check_count;c.generic_convergence_count+=generic;c.check_evaluated=true;c.check_result=generic;c.previous_error=prev;c.current_error=lm.error();c.qualification_last_evaluated=generic;if(generic){++c.qualification_evaluation_count;c.qualification_last_passed=a.valid&&a.stationary;c.qualification_last_iteration=lm.calls;}if(!a.valid)result.reason="CONDITIONAL_LM_STATIONARITY_AUDIT_INVALID";else if(generic&&a.stationary){result.converged=true;result.reason="CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY";}}
  else result.reason=lm.failure;
  if(lm.accepted>accepted_before){last_accepted_generic=generic;last_accepted_stationarity_valid=a.valid;last_accepted_step=uifgo::MaxScaledValuesStep(before,lm.values(),o.navigation_scales,1.0);last_accepted_gradient=a.max_scaled_gradient_objective;last_accepted_roundoff=a.roundoff_allowance_objective;last_accepted_identity=r03ValuesIdentity(lm.values());}
  if(allow_inexact_handoff&&!lm.failure.empty()){
   handoff.accepted_update_count=lm.accepted;handoff.last_accepted_generic_convergence=last_accepted_generic;handoff.last_accepted_stationarity_valid=last_accepted_stationarity_valid;handoff.last_accepted_scaled_navigation_step=last_accepted_step;handoff.last_accepted_scaled_navigation_gradient=last_accepted_gradient;handoff.last_accepted_gradient_roundoff_allowance=last_accepted_roundoff;handoff.last_accepted_values_identity=last_accepted_identity;handoff.last_lambda_search_trials=lm.last_search;handoff.objective_increase_allowance=uifgo::Binary64ObjectiveIncreaseAllowance(lm.error(),lm.error());
   if(r03InexactHandoffGuard(handoff,lm.failure,lm.unresolved,uifgo::GraphAndValuesKeysMatch(g,lm.values()))){handoff.qualified=true;handoff.status="INNER_NUMERICAL_STALL_INEXACT";handoff.trigger_reason="LAMBDA_EXHAUSTED_AFTER_SMALL_GENERIC_ACCEPT_WITH_ALL_SMALL_CERTIFIED_NON_DESCENT_TRIALS";handoff.handoff_count=1;result.reason="INNER_NUMERICAL_STALL_INEXACT";}else{handoff.status="GUARD_REJECTED";handoff.trigger_reason="ONE_OR_MORE_INEXACT_HANDOFF_GUARDS_FAILED";}
  }
  calls<<lm.calls<<','<<lm.error()<<','<<lm.lambda()<<','<<lm.trials<<','<<lm.accepted<<','<<lm.rejected<<','<<lm.unresolved<<','<<generic<<','<<a.valid<<','<<a.stationary<<','<<a.max_scaled_gradient_objective<<','<<a.roundoff_allowance_objective<<','<<lm.certificate_seconds<<','<<result.reason<<'\n';calls.flush();
  if(result.converged||!lm.failure.empty()||!a.valid)break;prev=lm.error();
 }
 result.values=lm.values();result.iterations=lm.iterations();result.inner_iterations=lm.getInnerIterations();result.lambda=lm.lambda();c.iterate_call_count=lm.calls;c.lambda_trial_count=lm.trials;c.rejected_lambda_trial_count=lm.rejected;c.accepted_update_count=lm.accepted;c.lambda_trial_accounting_status=lm.unresolved?"COMPLETE_WITH_EXPLICIT_UNRESOLVED":"COMPLETE";
 c.added_diagnostics_seconds=lm.certificate_seconds;
 auto status=output(out+"/block_status.json");status<<"{\"outer\":"<<outer<<",\"converged\":"<<(result.converged?"true":"false")<<",\"reason\":\""<<result.reason<<"\",\"calls\":"<<lm.calls<<",\"trials\":"<<lm.trials<<",\"accepted\":"<<lm.accepted<<",\"rejected\":"<<lm.rejected<<",\"unresolved\":"<<lm.unresolved<<",\"certificate_s\":"<<lm.certificate_seconds<<",\"handoff_qualified\":"<<(handoff.qualified?"true":"false")<<",\"handoff_count\":"<<handoff.handoff_count<<"}\n";
 if(allow_inexact_handoff){const bool guard_evaluated=!lm.failure.empty();const std::string guard_reason=guard_evaluated?(handoff.qualified?"EVALUATED_QUALIFIED":"EVALUATED_NOT_QUALIFIED"):(result.converged?"NOT_EVALUATED_INNER_CONVERGED":"NOT_EVALUATED_NO_NUMERICAL_STALL");writeInexactHandoffAudit(out+"/inexact_handoff.json",result.converged,handoff,guard_evaluated,guard_reason);}
 return result;
}
