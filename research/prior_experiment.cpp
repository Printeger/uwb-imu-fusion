#include "prior_evidence.hpp"
#include <iostream>
using namespace uwb_imu_pl::research;
int main() {
  try {
    int checks=0;
    const auto require=[&](bool value){if(!value)throw std::runtime_error("prior evidence check failed");++checks;};
    PriorQuery q{"simulation-only/raw-episode/v1","stationary-v1","single-episode-v1","per_declared_simulation_horizon","imu/x/samples11..20",0.,.6,false};
    require(!PriorEvidence{}.bound(q));
    const auto e=PriorEvidence::simulation(q.contract_id,q.model_id,q.scope_id,.6,
        {{q.predicate,1e-5,.05},{"uwb/anchor1/epoch3",1e-4,.05}});
    require(e.bound(q)==std::optional<double>(1e-5));
    auto altered=q;altered.deployment=true;require(!e.bound(altered));
    altered=q;altered.model_id="another-model";require(!e.bound(altered));
    altered=q;altered.end_seconds=.7;require(!e.bound(altered));
    altered=q;altered.time_basis="per_hour";require(!e.bound(altered));
    altered=q;altered.predicate="uwb/anchor1/epoch3";require(e.joint(q,altered)==std::optional<double>(0.));
    require(e.joint(q,q)==std::optional<double>(1e-5));
    require(e.nominalMass()>.999);
    bool rejected=false;try {PriorEvidence::simulation("production-risk-v1",q.model_id,q.scope_id,.6,{});}catch(const std::invalid_argument&){rejected=true;}require(rejected);
    rejected=false;try {PriorEvidence::simulation(q.contract_id,q.model_id,q.scope_id,.6,{{"a",.7,.1},{"b",.7,.1}});}catch(const std::invalid_argument&){rejected=true;}require(rejected);
    std::cout<<"VERIFIED PriorEvidence checks="<<checks<<" hardware=UNQUALIFIED simulation=CONDITIONAL production=REFUSED\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
