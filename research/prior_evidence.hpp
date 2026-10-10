#pragma once
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace uwb_imu_pl::research {
// Read-only prototype. Simulation probabilities come from a declared finite
// categorical law, NOT a calibration name or a user-set validated boolean.
struct SimulationAtom {
  std::string predicate; // physical sensor/axis/raw-time episode, never fitted ID
  double probability=0.;
  double duration_seconds=0.;
  double begin_seconds=0.; // event onset; tail may extend past evidence horizon
};
struct PriorQuery {
  std::string contract_id,model_id,scope_id,time_basis,predicate;
  double begin_seconds=0.,end_seconds=0.;
  bool deployment=false;
};
class PriorEvidence {
 public:
  PriorEvidence()=default; // fail closed, even with a nonempty calibration id
  static PriorEvidence simulation(std::string contract,std::string model,
      std::string scope,double horizon,std::vector<SimulationAtom> atoms) {
    if(contract.rfind("simulation-only/",0)!=0 || model.empty() || scope.empty() ||
       !std::isfinite(horizon) || !(horizon>0.))throw std::invalid_argument("invalid simulation prior identity");
    long double mass=0.;
    for(std::size_t i=0;i<atoms.size();++i) {
      const auto& a=atoms[i];
      if(a.predicate.empty() || !std::isfinite(a.probability) || a.probability<0. ||
         !std::isfinite(a.duration_seconds) || !(a.duration_seconds>0.) || a.duration_seconds>horizon || !std::isfinite(a.begin_seconds) || a.begin_seconds<0. || a.begin_seconds>horizon)
        throw std::invalid_argument("invalid simulation atom");
      for(std::size_t j=0;j<i;++j)if(atoms[j].predicate==a.predicate)throw std::invalid_argument("duplicate physical atom");
      mass+=static_cast<long double>(a.probability);
    }
    if(mass>1.)throw std::invalid_argument("categorical mass exceeds one");
    PriorEvidence e;e.contract_=std::move(contract);e.model_=std::move(model);
    e.scope_=std::move(scope);e.horizon_=horizon;e.atoms_=std::move(atoms);
    e.nominal_mass_=static_cast<double>(1.-mass);return e;
  }
  std::optional<double> bound(const PriorQuery& q) const {
    if(q.deployment || contract_.empty() || q.contract_id!=contract_ || q.model_id!=model_ ||
       q.scope_id!=scope_ || q.time_basis!="per_declared_simulation_horizon" ||
       !std::isfinite(q.begin_seconds) || !std::isfinite(q.end_seconds) ||
       q.begin_seconds<0. || q.end_seconds>horizon_ || q.end_seconds<q.begin_seconds)return {};
    for(const auto& a:atoms_)if(a.predicate==q.predicate)return a.probability;
    return {};
  }
  // No independence inferred. For events from this categorical space, two
  // distinct exact atoms are mutually exclusive by construction; identical
  // atom intersection is itself. Outside this space the result is UNKNOWN.
  std::optional<double> joint(PriorQuery a,const PriorQuery& b) const {
    const auto p=bound(a),q=bound(b);if(!p || !q)return {};
    return a.predicate==b.predicate ? *p : 0.;
  }
  const std::vector<SimulationAtom>& atoms() const {return atoms_;}
  double nominalMass() const {return nominal_mass_;}
  const char* qualification() const {return contract_.empty()?"UNQUALIFIED":"CONDITIONAL";}
 private:
  std::string contract_,model_,scope_;
  double horizon_=0.,nominal_mass_=0.;
  std::vector<SimulationAtom> atoms_;
};
} // namespace
