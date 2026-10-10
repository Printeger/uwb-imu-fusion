#include "common_reference_selection.hpp"
#include "prior_evidence.hpp"
#include <iostream>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
int main(int argc,char** argv) {
 try {
  if(argc!=2)throw std::runtime_error("CONFIG required");
  int checks=0;
  auto require=[&](bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);};
  const auto cfg=IntegrityConfigLoader::load(argv[1]);
  const auto healthy=ranges(cfg,3);const auto truth=Eigen::Vector3d(0,0,1);
  const auto good=rangePositionSet(healthy);require(good.valid,"healthy set refused");
  require(transferBound(good,truth).maxCoeff()<1e-8,"healthy precision");
  auto faulty=healthy;faulty.measurements.front().range_m+=2.25;
  const auto single=rangePositionSet(faulty);require(single.valid,"single fault missing");
  require(transferBound(single,truth).maxCoeff()<1e-8,"inconsistent subsets inflated into false union");
  faulty.measurements.back().range_m+=1.75;
  require(!rangePositionSet(faulty).valid,"two faults not refused in this fixture");
  auto flat=ranges(cfg,3,false,0.,true);require(!rangePositionSet(flat).valid,"flat geometry not refused");
  // Physical UWB+IMU counterexample, not just a singular abstract matrix:
  // static worlds reflected about the anchor plane have identical ranges,
  // gravity and gyro inputs at every epoch. A bootstrap branch does not add
  // an independently qualified sensor observation of the true branch.
  const Eigen::Vector3d mirror(0,0,1.2);
  for(const auto& m:flat.measurements)
    require(std::abs((m.anchor_position_m-truth).norm()-(m.anchor_position_m-mirror).norm())<1e-12,
            "coplanar reflected world not measurement-identical");
  auto tangent=healthy;
  for(auto& m:tangent.measurements) {
    m.anchor_position_m.z()=truth.z();m.range_m=(m.anchor_position_m-truth).norm();
    require(m.range_m>0. && (truth-m.anchor_position_m).normalized().z()==0.,
            "vertical protected direction not in range linear nullspace");
  }
  require(!rangePositionSet(tangent).valid,"dangerous vertical geometry accepted");
  auto invalid=healthy;invalid.measurements.front().range_m=std::numeric_limits<double>::quiet_NaN();
  require(!rangePositionSet(invalid).valid,"NaN accepted");
  const Eigen::Vector3d shifted(.02,-.03,.04);
  const auto bound=transferBound(single,truth+shifted);
  require((shifted.cwiseAbs().array()<=bound.array()).all(),"actual output triangle not covered");
  LinearizedIntegrityWindow window;window.id=WindowId(7);window.protected_state_map=Eigen::Matrix3d::Identity();
  BoundRiskContextV1 ctx;ctx.window_id=window.id;ctx.valid_from=ctx.valid_until=healthy.timestamp;
  ctx.model_id="simulation-only/deterministic-range-union-v1";ctx.scope_id="simulation-only/one-episode-v1";ctx.manifest_id="simulation-only/raw-batch-v1";
  for(const auto* term:{"omitted","envelope","bridge","history","model"})ctx.evidence[term]={RiskEvidenceDomainV1::SimulationConditional,ctx.model_id+"|"+term};
  auto& in=ctx.inputs;in.omitted_scope_complete=in.omitted_event_bound_known=in.omitted_event_bound_validated=true;
  in.envelope_event_bound_known=in.envelope_event_bound_validated=true;
  in.bridge_escape_validated=in.history_escape_validated=in.model_escape_validated=true;
  FaultHypothesisV2 h;h.id=HypothesisId(1);h.prior_probability_bound=1e-4;h.p_md_allocation=1e-3;h.hmi_allocation=conservativeRemainingHypothesisRisk(cfg.risk_v2);
  RiskBudgetV2 risk;std::vector<FaultHypothesisV2> hypotheses{h};
  ctx.risk_contract_id=riskContractIdentityV1(risk,hypotheses);
  std::vector<CandidateEvaluation> candidates(2);
  for(int i=0;i<2;++i){auto& c=candidates[i];c.valid=c.post_detector_passed=c.covers_plausible_set=true;
   c.action.id=ExclusionActionId(i+1);c.action.recoverability=HistoryRecoverability::Recoverable;
   c.state_increment=shifted*(i+1);c.pl_xyz_m=relativeTransferBound(good,truth,c.state_increment);
   c.hpl_m=c.pl_xyz_m.head<2>().norm();c.vpl_m=c.pl_xyz_m.z();c.information_logdet=0.;}
  FdeDecision rejected;rejected.status=FdeStatus::RiskBudgetInvalid;RiskLedger ledger;
  const auto accepted=commonRangeReferenceSelection(rejected,good,healthy,truth,window,risk,hypotheses,ctx,candidates,ledger);
  require(accepted.commit_allowed && ledger.closes,"legal common proof failed");
  auto throws=[&](auto set,auto raw,auto output){try{commonRangeReferenceSelection(rejected,set,raw,truth,window,risk,hypotheses,ctx,output,ledger);return false;}catch(const std::runtime_error&){return true;}};
  verifyRangeOutputs(good,healthy,truth,window,risk,hypotheses,ctx,candidates);++checks;
  auto missing_evidence=ctx;missing_evidence.evidence.erase("model");bool rejected_evidence=false;
  try{verifyRangeOutputs(good,healthy,truth,window,risk,hypotheses,missing_evidence,candidates);}catch(const std::runtime_error&){rejected_evidence=true;}
  require(rejected_evidence,"nonempty model identity substituted for missing evidence");
  auto forged=candidates;forged.back().vpl_m+=.001;bool rejected_output=false;
  try{verifyRangeOutputs(good,healthy,truth,window,risk,hypotheses,ctx,forged);}catch(const std::runtime_error&){rejected_output=true;}
  require(rejected_output,"successful KEEP/IMU consumer accepted forged VPL");
  auto altered=candidates;altered.front().pl_xyz_m.x()+=.001;require(throws(good,healthy,altered),"external PL tamper accepted");
  altered=candidates;altered.back().hpl_m+=.001;require(throws(good,healthy,altered),"HPL tamper accepted");
  auto changed=good;changed.centres.front().x()+=.001;require(throws(changed,healthy,candidates),"source centre tamper accepted");
  changed=good;changed.radius.clear();require(throws(changed,healthy,candidates),"malformed range proof accepted");
  require(!transferBound(changed,truth).allFinite(),"malformed range proof not fail closed");
  auto other_contract=ctx;ctx.model_id="simulation-only/foreign-model";
  require(!commonRangeReferenceSelection(rejected,good,healthy,truth,window,risk,hypotheses,ctx,candidates,ledger).commit_allowed,"foreign contract accepted");ctx=other_contract;
  auto crossed=healthy;crossed.timestamp=TimestampNs(healthy.timestamp.value()+1);require(throws(good,crossed,candidates),"cross-time proof accepted");
  ctx.inputs.model_formal_eligible=true;
  require(!commonRangeReferenceSelection(rejected,good,healthy,truth,window,risk,hypotheses,ctx,candidates,ledger).commit_allowed,"deployment path accepted");
  bool duplicate=false;try{PriorEvidence::simulation("simulation-only/test","model","scope",1.,{{"a",.1,.1},{"a",.2,.1}});}catch(const std::invalid_argument&){duplicate=true;}
  require(duplicate,"duplicate atom accepted");
  bool nan=false;try{PriorEvidence::simulation("simulation-only/test","model","scope",1.,{{"a",std::numeric_limits<double>::quiet_NaN(),.1}});}catch(const std::invalid_argument&){nan=true;}
  require(nan,"nonfinite probability accepted");
  bool future=false;try{PriorEvidence::simulation("simulation-only/test","model","scope",1.,{{"a",.1,.1,1.1}});}catch(const std::invalid_argument&){future=true;}
  require(future,"out-of-scope physical onset accepted");
  std::cout<<"VERIFIED isolated contract checks="<<checks<<"; formal_eligible=false publication_protected=false\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
