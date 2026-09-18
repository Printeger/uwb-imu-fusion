#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <stdexcept>
#include <type_traits>

namespace uwb_imu_pl {
namespace {
// The v1 codec uses fixed-width integers and IEEE binary64 on little-endian
// hosts. An endian marker rejects incompatible hosts instead of misreading.
struct Codec {
  std::istream* in = nullptr;
  std::ostream* out = nullptr;
  template<class T> void scalar(T& x) {
    static_assert(std::is_arithmetic<T>::value || std::is_enum<T>::value, "scalar");
    if (out) out->write(reinterpret_cast<const char*>(&x), sizeof(x));
    else in->read(reinterpret_cast<char*>(&x), sizeof(x));
    if ((out && !*out) || (in && !*in)) throw std::runtime_error("truncated replay or write failure");
  }
  void size(std::uint64_t& n) { scalar(n); if (n > 100000000) throw std::runtime_error("replay size limit"); }
  void text(std::string& s) {
    std::uint64_t n = s.size(); size(n); if(in) s.resize(n);
    if(out) out->write(s.data(), n); else in->read(s.data(), n);
    if ((out && !*out) || (in && !*in)) throw std::runtime_error("replay string IO failure");
  }
  template<class Tag> void id(StrongId<Tag>& x) { std::uint64_t v=x.value(); scalar(v); if(in) x=StrongId<Tag>(v); }
  void timestamp(TimestampNs& t) { std::int64_t v=t.value(); scalar(v); if(in) t=TimestampNs(v); }
  template<class T, class F> void vector(std::vector<T>& xs, F f) {
    std::uint64_t n=xs.size(); size(n); if(in) xs.resize(n); for(auto& x:xs) f(x);
  }
  template<class T> void ids(std::vector<T>& xs) { vector(xs,[&](T& x){id(x);}); }
  template<class T> void matrix(T& m) {
    std::uint64_t r=m.rows(), c=m.cols(); size(r); size(c);
    if (r*c > 100000000 || (T::ColsAtCompileTime == 1 && c != 1) ||
        (T::RowsAtCompileTime == 3 && r != 3)) throw std::runtime_error("invalid replay matrix shape");
    if(in) m.resize(r,c);
    for(std::uint64_t j=0;j<c;++j) for(std::uint64_t i=0;i<r;++i) scalar(m(i,j));
  }
  void version(LinearizationVersion& v) {
    scalar(v.graph_version); scalar(v.ordering_version); scalar(v.noise_model_version); scalar(v.linpoint_version);
  }
  void block(LinearizedFactorBlock& b) {
    id(b.group_id); scalar(b.kind); scalar(b.sensor); scalar(b.role);
    matrix(b.jacobian_raw); matrix(b.residual_raw); matrix(b.covariance); matrix(b.whitener);
    matrix(b.jacobian_whitened); matrix(b.residual_whitened);
    vector(b.window_column_indices,[&](int& x){scalar(x);}); ids(b.fault_units);
    scalar(b.effective_weight); text(b.whitening_model_id); version(b.version);
  }
  void optional(std::optional<std::size_t>& x) {
    bool has=x.has_value(); scalar(has); std::uint64_t v=x.value_or(0); scalar(v);
    if(in) { x.reset(); if(has) x=static_cast<std::size_t>(v); }
  }
};
void transfer(Codec& c, FrozenCandidateReplay& r) {
  std::string schema="uwb-imu-pl/frozen-candidates/v4"; c.text(schema);
  if(schema!="uwb-imu-pl/frozen-candidates/v1" &&
      schema!="uwb-imu-pl/frozen-candidates/v2" &&
      schema!="uwb-imu-pl/frozen-candidates/v3" &&
      schema!="uwb-imu-pl/frozen-candidates/v4")
    throw std::runtime_error("unknown replay schema");
  std::uint64_t endian=0x0102030405060708ULL; c.scalar(endian);
  if(endian!=0x0102030405060708ULL) throw std::runtime_error("incompatible replay byte order");
  c.scalar(r.input_attempt_id); c.timestamp(r.input_timestamp); c.scalar(r.transaction_id);
  auto& w=r.window;
  c.id(w.id); c.version(w.version); c.matrix(w.H); c.matrix(w.z);
  c.matrix(w.base_information); c.matrix(w.base_information_rhs); c.matrix(w.protected_state_map);
  c.scalar(w.rank); c.scalar(w.dof); c.scalar(w.condition_number); c.scalar(w.model_valid); c.text(w.reason);
  c.scalar(w.detector_first_epoch); c.scalar(w.recovery_first_epoch);
  auto& p=w.capabilities;
  c.scalar(p.includes_boundary_prior); c.scalar(p.includes_pending_imu); c.scalar(p.includes_pending_uwb);
  c.scalar(p.complete_factor_provenance); c.scalar(p.history_provenance_valid); c.scalar(p.fixed_lag_maturity_valid);
  c.scalar(p.no_duplicate_rows); c.scalar(p.every_active_factor_accounted_once); c.scalar(p.frozen_slot_identity_valid);
  c.vector(w.state_layout,[&](StateLayoutEntry& s){ c.scalar(s.epoch);
    c.vector(s.keys,[&](gtsam::Key& k){c.scalar(k);}); c.scalar(s.column_offset);
    c.scalar(s.dimension); c.scalar(s.protected_current_state); });
  c.vector(w.slot_accounting,[&](FactorSlotAccounting& s){ c.scalar(s.slot);
    bool has=s.group_id.has_value(); c.scalar(has); FactorGroupId id=s.group_id.value_or(FactorGroupId{});
    c.id(id); if(c.in){s.group_id.reset(); if(has) s.group_id=id;}
    c.scalar(s.explicit_window_block); c.scalar(s.boundary_input); c.scalar(s.pointer_identity_valid); });
  if (schema=="uwb-imu-pl/frozen-candidates/v4") {
    c.vector(w.factor_inventory,[&](FrozenWindowFactorInventoryEntry& e) {
      c.id(e.group_id); c.scalar(e.epoch); c.scalar(e.kind); c.scalar(e.sensor);
      c.scalar(e.disposition);
      c.vector(e.keys,[&](gtsam::Key& key){ c.scalar(key); });
      c.vector(e.slots,[&](std::size_t& slot){ c.scalar(slot); });
    });
  }
  c.vector(w.blocks,[&](LinearizedFactorBlock& b){c.block(b);});
  // Deduplicate exact serialized content, never ID alone.
  std::map<std::string,std::uint64_t> dictionary;
  std::vector<LinearizedFactorBlock> blocks;
  std::vector<std::vector<std::uint64_t>> refs;
  if(c.out) for(auto& action:r.actions) {
    refs.emplace_back();
    for(auto& b:action.added_blocks) {
      std::ostringstream bytes(std::ios::binary); Codec encode{nullptr,&bytes}; encode.block(b);
      const auto inserted=dictionary.emplace(bytes.str(),blocks.size());
      if(inserted.second) blocks.push_back(b);
      refs.back().push_back(inserted.first->second);
    }
  }
  c.vector(blocks,[&](LinearizedFactorBlock& b){c.block(b);});
  std::size_t index=0;
  c.vector(r.actions,[&](ExclusionAction& a){
    c.id(a.id); c.ids(a.covered_units); c.ids(a.covered_modes);
    c.vector(a.physical_source_ids,[&](std::string& s){c.text(s);});
    c.ids(a.groups_to_remove); c.ids(a.groups_to_add); c.scalar(a.bridge_mode);
    c.scalar(a.exclusion_cardinality); c.text(a.action_model_id); c.scalar(a.recoverability);
    c.optional(a.recovery_epoch_begin); c.optional(a.recovery_epoch_end);
    std::vector<std::uint64_t> ids; if(c.out) ids=refs.at(index);
    c.vector(ids,[&](std::uint64_t& x){c.scalar(x);});
    if(c.in) for(auto id:ids) { if(id>=blocks.size()) throw std::runtime_error("invalid replay block reference"); a.added_blocks.push_back(blocks.at(id)); }
    ++index;
  });
  auto& cfg=r.config;
  c.scalar(cfg.rank_tolerance); c.scalar(cfg.max_condition_number); c.scalar(cfg.max_linearization_step_norm);
  c.scalar(cfg.materialize_dense_oracle_fields); c.scalar(cfg.exact_slow_path_condition);
  if(schema=="uwb-imu-pl/frozen-candidates/v2" ||
      schema=="uwb-imu-pl/frozen-candidates/v3") {
    c.scalar(cfg.enable_shared_cache); c.scalar(cfg.enable_early_step_gate);
  } else if (c.in) {
    cfg.enable_shared_cache = false;
    cfg.enable_early_step_gate = false;
    cfg.enable_numerical_certificate = false;
  }
  if (schema=="uwb-imu-pl/frozen-candidates/v3" ||
      schema=="uwb-imu-pl/frozen-candidates/v4") {
    c.scalar(cfg.enable_numerical_certificate);
    c.scalar(cfg.force_exact_condition_number);
  } else if (c.in) {
    // v1/v2 artifacts retain their original P2 numerical semantics.
    cfg.enable_numerical_certificate = false;
    cfg.force_exact_condition_number = true;
  }
}
}  // namespace
void writeCandidateReplay(const std::string& path, const FrozenCandidateReplay& replay) {
  std::ofstream out(path,std::ios::binary|std::ios::trunc);
  if(!out) throw std::runtime_error("cannot open replay: "+path);
  FrozenCandidateReplay copy=replay; Codec c{nullptr,&out}; transfer(c,copy);
}
FrozenCandidateReplay readCandidateReplay(const std::string& path) {
  std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("cannot open replay: "+path);
  FrozenCandidateReplay r; Codec c{&in,nullptr}; transfer(c,r);
  if(in.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("trailing replay data");
  // Decomposition objects are process-local and intentionally absent from the
  // codec. Recreate and rebind them to the decoded content before any replay.
  finalizeIntegrityWindow(&r.window, r.config.rank_tolerance,
                          r.config.max_condition_number);
  return r;
}
}  // namespace uwb_imu_pl
