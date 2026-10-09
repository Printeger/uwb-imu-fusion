#pragma once

#include <atomic>
#include <cstdint>

namespace uwb_imu_pl {

struct NumericalWorkSnapshot {
  std::uint64_t base_svd = 0;
  std::uint64_t base_llt = 0;
  std::uint64_t base_state_solves = 0;
  std::uint64_t llt_state_solve_calls = 0;
  std::uint64_t svd_state_solve_calls = 0;
  std::uint64_t detector_reference_qr = 0;
  std::uint64_t candidate_reference_svd = 0;
  std::uint64_t candidate_inner_llt = 0;
  std::uint64_t fault_gram_eigen = 0;
  std::uint64_t fault_gram_svd = 0;
  std::uint64_t fault_gram_ldlt = 0;
  std::uint64_t low_dim_fault_gram = 0;
  std::uint64_t generic_fault_gram_fallback = 0;
  std::uint64_t hypothesis_parallel_blocks = 0;
  std::uint64_t hypothesis_shared_hits = 0;
  std::uint64_t hypothesis_shared_misses = 0;
  std::uint64_t covariance_rhs_solves = 0;
  std::uint64_t covariance_rhs_columns = 0;
  std::uint64_t spectral_rhs_solves = 0;
  std::uint64_t spectral_rhs_columns = 0;
  std::uint64_t numerical_contract_mismatches = 0;
  std::uint64_t imu_oracle_reintegrations = 0;
  std::uint64_t square_root_factorizations = 0;
  std::uint64_t square_root_information_solves = 0;
  std::uint64_t square_root_information_columns = 0;
  std::uint64_t square_root_qt_applications = 0;
  std::uint64_t square_root_qt_columns = 0;
  std::uint64_t square_root_symbolic_hits = 0;
  std::uint64_t square_root_symbolic_misses = 0;
  std::uint64_t square_root_fallbacks = 0;
  std::uint64_t square_root_certificate_holds = 0;
  // B2: fault-mode cross blocks are computed on demand only.
  std::uint64_t fault_mode_columns = 0;
  std::uint64_t fault_cross_blocks = 0;
  std::uint64_t fault_cross_block_cache_hits = 0;
  std::uint64_t all_mode_gram_columns = 0;
  // B2: compact-mode storage accounting (per-mode dense allocations are the
  // thing the compact path must eliminate from the hot loop).
  std::uint64_t mode_dense_allocations = 0;
  std::uint64_t mode_dense_allocation_rows = 0;
  std::uint64_t mode_dense_allocation_columns = 0;
  // B2: compact footprint actually built (rows and columns of the per-mode
  // blocks that only cover the factor groups the mode touches).
  std::uint64_t compact_mode_rows = 0;
  std::uint64_t compact_mode_columns = 0;
  // Rows the padded form would have materialized for the same windows.
  std::uint64_t compact_padded_equivalent_rows = 0;
  std::uint64_t compact_capacity_fallbacks = 0;
  std::uint64_t hypothesis_capacity_refusals = 0;
  // B4: lazy FDE accounting.  A healthy frame must show zero bridge blocks and
  // exactly the KEEP_ALL action entity; the deferred count records how many
  // entities the lazy path avoided materializing.
  std::uint64_t action_entities_constructed = 0;
  std::uint64_t bridge_blocks_built = 0;
  std::uint64_t candidate_graph_built = 0;
  std::uint64_t action_entities_deferred = 0;
  std::uint64_t evidence_calls_fault_path = 0;
  std::uint64_t evidence_calls_health_path = 0;
  // C1-b: history-summary construction work (dense Householder elimination of
  // the frozen boundary rows) and its explicit states.  `boundary_rows` is the
  // extracted input volume, `input_columns` the system width (old + boundary +
  // fault + rhs), `fault_columns` the injected historical fault columns and
  // `emitted_rows`/`perp_rows` what the condensed block contributes to the
  // window (the perp rows are the detector-only rows carrying kappa_b).
  std::uint64_t history_summary_builds = 0;
  std::uint64_t history_boundary_rows = 0;
  std::uint64_t history_input_columns = 0;
  std::uint64_t history_fault_columns = 0;
  std::uint64_t history_emitted_rows = 0;
  std::uint64_t history_perp_rows = 0;
  std::uint64_t history_capacity_refusals = 0;
  std::uint64_t history_summary_invalid = 0;
  // P1-01/R07 identity/indexing work.  A content scan reads the complete
  // frozen window; an identity reuse consumes the typed immutable handle.
  std::uint64_t window_content_hash_scans = 0;
  std::uint64_t frozen_identity_builds = 0;
  std::uint64_t frozen_identity_reuses = 0;
  std::uint64_t descriptor_id_lookups = 0;
  std::uint64_t descriptor_linear_scans = 0;
  std::uint64_t block_rhs_solve_batches = 0;
  std::uint64_t block_rhs_unique_blocks = 0;
  std::uint64_t frozen_admission_constant_validations = 0;
  std::uint64_t pl_payload_validations = 0;
  std::uint64_t pl_payload_validation_reuses = 0;
  std::uint64_t frozen_hypothesis_validations = 0;
  std::uint64_t frozen_hypothesis_validation_reuses = 0;
};

// Counts actual decomposition/solve construction points.  The counters are
// process-wide so development runners can report work spanning many windows.
class NumericalWorkCounters {
 public:
  static NumericalWorkSnapshot snapshot() {
    return {base_svd_.load(),
            base_llt_.load(),
            base_state_solves_.load(),
            llt_state_solve_calls_.load(),
            svd_state_solve_calls_.load(),
            detector_reference_qr_.load(),
            candidate_reference_svd_.load(),
            candidate_inner_llt_.load(),
            fault_gram_eigen_.load(),
            fault_gram_svd_.load(),
            fault_gram_ldlt_.load(),
            low_dim_fault_gram_.load(),
            generic_fault_gram_fallback_.load(),
            hypothesis_parallel_blocks_.load(),
            hypothesis_shared_hits_.load(),
            hypothesis_shared_misses_.load(),
            covariance_rhs_solves_.load(),
            covariance_rhs_columns_.load(),
            spectral_rhs_solves_.load(),
            spectral_rhs_columns_.load(),
            numerical_contract_mismatches_.load(),
            imu_oracle_reintegrations_.load(),
            square_root_factorizations_.load(),
            square_root_information_solves_.load(),
            square_root_information_columns_.load(),
            square_root_qt_applications_.load(),
            square_root_qt_columns_.load(),
            square_root_symbolic_hits_.load(),
            square_root_symbolic_misses_.load(),
            square_root_fallbacks_.load(),
            square_root_certificate_holds_.load(),
            fault_mode_columns_.load(),
            fault_cross_blocks_.load(),
            fault_cross_block_cache_hits_.load(),
            all_mode_gram_columns_.load(),
            mode_dense_allocations_.load(),
            mode_dense_allocation_rows_.load(),
            mode_dense_allocation_columns_.load(),
            compact_mode_rows_.load(),
            compact_mode_columns_.load(),
            compact_padded_equivalent_rows_.load(),
            compact_capacity_fallbacks_.load(),
            hypothesis_capacity_refusals_.load(),
            action_entities_constructed_.load(),
            bridge_blocks_built_.load(),
            candidate_graph_built_.load(),
            action_entities_deferred_.load(),
            evidence_calls_fault_path_.load(),
            evidence_calls_health_path_.load(),
            history_summary_builds_.load(),
            history_boundary_rows_.load(),
            history_input_columns_.load(),
            history_fault_columns_.load(),
            history_emitted_rows_.load(),
            history_perp_rows_.load(),
            history_capacity_refusals_.load(),
            history_summary_invalid_.load(),
            window_content_hash_scans_.load(),
            frozen_identity_builds_.load(),
            frozen_identity_reuses_.load(),
            descriptor_id_lookups_.load(),
            descriptor_linear_scans_.load(),
            block_rhs_solve_batches_.load(),
            block_rhs_unique_blocks_.load(),
            frozen_admission_constant_validations_.load(),
            pl_payload_validations_.load(),
            pl_payload_validation_reuses_.load(),
            frozen_hypothesis_validations_.load(),
            frozen_hypothesis_validation_reuses_.load()};
  }
  static void reset() {
    base_svd_ = 0;
    base_llt_ = 0;
    base_state_solves_ = 0;
    llt_state_solve_calls_ = 0;
    svd_state_solve_calls_ = 0;
    detector_reference_qr_ = 0;
    candidate_reference_svd_ = 0;
    candidate_inner_llt_ = 0;
    fault_gram_eigen_ = 0;
    fault_gram_svd_ = 0;
    fault_gram_ldlt_ = 0;
    low_dim_fault_gram_ = 0;
    generic_fault_gram_fallback_ = 0;
    hypothesis_parallel_blocks_ = 0;
    hypothesis_shared_hits_ = 0;
    hypothesis_shared_misses_ = 0;
    covariance_rhs_solves_ = 0;
    covariance_rhs_columns_ = 0;
    spectral_rhs_solves_ = 0;
    spectral_rhs_columns_ = 0;
    numerical_contract_mismatches_ = 0;
    imu_oracle_reintegrations_ = 0;
    square_root_factorizations_ = 0;
    square_root_information_solves_ = 0;
    square_root_information_columns_ = 0;
    square_root_qt_applications_ = 0;
    square_root_qt_columns_ = 0;
    square_root_symbolic_hits_ = 0;
    square_root_symbolic_misses_ = 0;
    square_root_fallbacks_ = 0;
    square_root_certificate_holds_ = 0;
    fault_mode_columns_ = 0;
    fault_cross_blocks_ = 0;
    fault_cross_block_cache_hits_ = 0;
    all_mode_gram_columns_ = 0;
    mode_dense_allocations_ = 0;
    mode_dense_allocation_rows_ = 0;
    mode_dense_allocation_columns_ = 0;
    compact_mode_rows_ = 0;
    compact_mode_columns_ = 0;
    compact_padded_equivalent_rows_ = 0;
    compact_capacity_fallbacks_ = 0;
    hypothesis_capacity_refusals_ = 0;
    action_entities_constructed_ = 0;
    bridge_blocks_built_ = 0;
    candidate_graph_built_ = 0;
    action_entities_deferred_ = 0;
    evidence_calls_fault_path_ = 0;
    evidence_calls_health_path_ = 0;
    history_summary_builds_ = 0;
    history_boundary_rows_ = 0;
    history_input_columns_ = 0;
    history_fault_columns_ = 0;
    history_emitted_rows_ = 0;
    history_perp_rows_ = 0;
    history_capacity_refusals_ = 0;
    history_summary_invalid_ = 0;
    window_content_hash_scans_ = 0;
    frozen_identity_builds_ = 0;
    frozen_identity_reuses_ = 0;
    descriptor_id_lookups_ = 0;
    descriptor_linear_scans_ = 0;
    block_rhs_solve_batches_ = 0;
    block_rhs_unique_blocks_ = 0;
    frozen_admission_constant_validations_ = 0;
    pl_payload_validations_ = 0;
    pl_payload_validation_reuses_ = 0;
    frozen_hypothesis_validations_ = 0;
    frozen_hypothesis_validation_reuses_ = 0;
  }
  static void baseSvd() { ++base_svd_; }
  static void baseLlt() { ++base_llt_; }
  static void baseStateSolve() { ++base_state_solves_; }
  static void lltStateSolve() { ++llt_state_solve_calls_; }
  static void svdStateSolve() { ++svd_state_solve_calls_; }
  static void detectorReferenceQr() { ++detector_reference_qr_; }
  static void candidateReferenceSvd() { ++candidate_reference_svd_; }
  static void candidateInnerLlt() { ++candidate_inner_llt_; }
  static void faultGramEigen(std::uint64_t count = 1) {
    fault_gram_eigen_ += count;
  }
  static void faultGramSvd(std::uint64_t count = 1) {
    fault_gram_svd_ += count;
  }
  static void faultGramLdlt(std::uint64_t count = 1) {
    fault_gram_ldlt_ += count;
  }
  static void lowDimFaultGram(std::uint64_t count = 1) {
    low_dim_fault_gram_ += count;
  }
  static void genericFaultGramFallback(std::uint64_t count = 1) {
    generic_fault_gram_fallback_ += count;
  }
  static void hypothesisParallelBlocks(std::uint64_t count = 1) {
    hypothesis_parallel_blocks_ += count;
  }
  static void hypothesisSharedHit() { ++hypothesis_shared_hits_; }
  static void hypothesisSharedMiss() { ++hypothesis_shared_misses_; }
  static void covarianceRhsSolve(std::uint64_t columns) {
    ++covariance_rhs_solves_;
    covariance_rhs_columns_ += columns;
  }
  static void spectralRhsSolve(std::uint64_t columns) {
    ++spectral_rhs_solves_;
    spectral_rhs_columns_ += columns;
  }
  static void numericalContractMismatch() { ++numerical_contract_mismatches_; }
  static void imuOracleReintegration() { ++imu_oracle_reintegrations_; }
  static void squareRootFactorization() { ++square_root_factorizations_; }
  static void squareRootInformationSolve(std::uint64_t columns) {
    ++square_root_information_solves_;
    square_root_information_columns_ += columns;
  }
  static void squareRootQtApplication(std::uint64_t columns) {
    ++square_root_qt_applications_;
    square_root_qt_columns_ += columns;
  }
  static void squareRootSymbolicHit() { ++square_root_symbolic_hits_; }
  static void squareRootSymbolicMiss() { ++square_root_symbolic_misses_; }
  static void squareRootFallback() { ++square_root_fallbacks_; }
  static void squareRootCertificateHold() { ++square_root_certificate_holds_; }
  static void faultModeColumns(std::uint64_t columns) {
    fault_mode_columns_ += columns;
  }
  static void faultCrossBlock() { ++fault_cross_blocks_; }
  static void faultCrossBlockCacheHit() { ++fault_cross_block_cache_hits_; }
  static void allModeGramColumns(std::uint64_t columns) {
    all_mode_gram_columns_ += columns;
  }
  static void modeDenseAllocation(std::uint64_t rows, std::uint64_t columns) {
    ++mode_dense_allocations_;
    mode_dense_allocation_rows_ += rows;
    mode_dense_allocation_columns_ += columns;
  }
  static void compactModeStorage(std::uint64_t rows, std::uint64_t columns) {
    compact_mode_rows_ += rows;
    compact_mode_columns_ += columns;
  }
  static void compactPaddedEquivalent(std::uint64_t rows) {
    compact_padded_equivalent_rows_ += rows;
  }
  static void compactCapacityFallback() { ++compact_capacity_fallbacks_; }
  static void hypothesisCapacityRefusal() { ++hypothesis_capacity_refusals_; }
  static void actionEntitiesConstructed(std::uint64_t count) {
    action_entities_constructed_ += count;
  }
  static void bridgeBlocksBuilt(std::uint64_t count) {
    bridge_blocks_built_ += count;
  }
  static void candidateGraphBuilt() { ++candidate_graph_built_; }
  static void actionEntitiesDeferred(std::uint64_t count) {
    action_entities_deferred_ += count;
  }
  static void evidenceCallFaultPath() { ++evidence_calls_fault_path_; }
  static void evidenceCallHealthPath() { ++evidence_calls_health_path_; }
  static void historySummaryBuild(std::uint64_t rows, std::uint64_t columns,
                                  std::uint64_t fault_columns,
                                  std::uint64_t emitted_rows) {
    ++history_summary_builds_;
    history_boundary_rows_ += rows;
    history_input_columns_ += columns;
    history_fault_columns_ += fault_columns;
    history_emitted_rows_ += emitted_rows;
  }
  static void historyPerpRows(std::uint64_t rows) {
    history_perp_rows_ += rows;
  }
  static void historyCapacityRefusal() { ++history_capacity_refusals_; }
  static void historySummaryInvalid() { ++history_summary_invalid_; }
  static void windowContentHashScan() { ++window_content_hash_scans_; }
  static void frozenIdentityBuild() { ++frozen_identity_builds_; }
  static void frozenIdentityReuse(std::uint64_t count = 1) {
    frozen_identity_reuses_ += count;
  }
  static void descriptorIdLookup(std::uint64_t count = 1) {
    descriptor_id_lookups_ += count;
  }
  static void descriptorLinearScan(std::uint64_t count = 1) {
    descriptor_linear_scans_ += count;
  }
  static void blockRhsSolveBatch(std::uint64_t unique_blocks) {
    ++block_rhs_solve_batches_;
    block_rhs_unique_blocks_ += unique_blocks;
  }
  static void frozenAdmissionConstantValidation() {
    ++frozen_admission_constant_validations_;
  }
  static void plPayloadValidation() { ++pl_payload_validations_; }
  static void plPayloadValidationReuse() { ++pl_payload_validation_reuses_; }
  static void frozenHypothesisValidation() { ++frozen_hypothesis_validations_; }
  static void frozenHypothesisValidationReuse() { ++frozen_hypothesis_validation_reuses_; }

 private:
  inline static std::atomic<std::uint64_t> base_svd_{0};
  inline static std::atomic<std::uint64_t> base_llt_{0};
  inline static std::atomic<std::uint64_t> base_state_solves_{0};
  inline static std::atomic<std::uint64_t> llt_state_solve_calls_{0};
  inline static std::atomic<std::uint64_t> svd_state_solve_calls_{0};
  inline static std::atomic<std::uint64_t> detector_reference_qr_{0};
  inline static std::atomic<std::uint64_t> candidate_reference_svd_{0};
  inline static std::atomic<std::uint64_t> candidate_inner_llt_{0};
  inline static std::atomic<std::uint64_t> fault_gram_eigen_{0};
  inline static std::atomic<std::uint64_t> fault_gram_svd_{0};
  inline static std::atomic<std::uint64_t> fault_gram_ldlt_{0};
  inline static std::atomic<std::uint64_t> low_dim_fault_gram_{0};
  inline static std::atomic<std::uint64_t> generic_fault_gram_fallback_{0};
  inline static std::atomic<std::uint64_t> hypothesis_parallel_blocks_{0};
  inline static std::atomic<std::uint64_t> hypothesis_shared_hits_{0};
  inline static std::atomic<std::uint64_t> hypothesis_shared_misses_{0};
  inline static std::atomic<std::uint64_t> covariance_rhs_solves_{0};
  inline static std::atomic<std::uint64_t> covariance_rhs_columns_{0};
  inline static std::atomic<std::uint64_t> spectral_rhs_solves_{0};
  inline static std::atomic<std::uint64_t> spectral_rhs_columns_{0};
  inline static std::atomic<std::uint64_t> numerical_contract_mismatches_{0};
  inline static std::atomic<std::uint64_t> imu_oracle_reintegrations_{0};
  inline static std::atomic<std::uint64_t> square_root_factorizations_{0};
  inline static std::atomic<std::uint64_t> square_root_information_solves_{0};
  inline static std::atomic<std::uint64_t> square_root_information_columns_{0};
  inline static std::atomic<std::uint64_t> square_root_qt_applications_{0};
  inline static std::atomic<std::uint64_t> square_root_qt_columns_{0};
  inline static std::atomic<std::uint64_t> compact_mode_rows_{0};
  inline static std::atomic<std::uint64_t> compact_padded_equivalent_rows_{0};
  inline static std::atomic<std::uint64_t> compact_mode_columns_{0};
  inline static std::atomic<std::uint64_t> compact_capacity_fallbacks_{0};
  inline static std::atomic<std::uint64_t> hypothesis_capacity_refusals_{0};
  inline static std::atomic<std::uint64_t> action_entities_constructed_{0};
  inline static std::atomic<std::uint64_t> bridge_blocks_built_{0};
  inline static std::atomic<std::uint64_t> candidate_graph_built_{0};
  inline static std::atomic<std::uint64_t> action_entities_deferred_{0};
  inline static std::atomic<std::uint64_t> evidence_calls_fault_path_{0};
  inline static std::atomic<std::uint64_t> evidence_calls_health_path_{0};
  inline static std::atomic<std::uint64_t> history_summary_builds_{0};
  inline static std::atomic<std::uint64_t> history_boundary_rows_{0};
  inline static std::atomic<std::uint64_t> history_input_columns_{0};
  inline static std::atomic<std::uint64_t> history_fault_columns_{0};
  inline static std::atomic<std::uint64_t> history_emitted_rows_{0};
  inline static std::atomic<std::uint64_t> history_perp_rows_{0};
  inline static std::atomic<std::uint64_t> history_capacity_refusals_{0};
  inline static std::atomic<std::uint64_t> history_summary_invalid_{0};
  inline static std::atomic<std::uint64_t> square_root_symbolic_hits_{0};
  inline static std::atomic<std::uint64_t> square_root_symbolic_misses_{0};
  inline static std::atomic<std::uint64_t> square_root_fallbacks_{0};
  inline static std::atomic<std::uint64_t> square_root_certificate_holds_{0};
  inline static std::atomic<std::uint64_t> fault_mode_columns_{0};
  inline static std::atomic<std::uint64_t> fault_cross_blocks_{0};
  inline static std::atomic<std::uint64_t> fault_cross_block_cache_hits_{0};
  inline static std::atomic<std::uint64_t> all_mode_gram_columns_{0};
  inline static std::atomic<std::uint64_t> mode_dense_allocations_{0};
  inline static std::atomic<std::uint64_t> mode_dense_allocation_rows_{0};
  inline static std::atomic<std::uint64_t> mode_dense_allocation_columns_{0};
  inline static std::atomic<std::uint64_t> window_content_hash_scans_{0};
  inline static std::atomic<std::uint64_t> frozen_identity_builds_{0};
  inline static std::atomic<std::uint64_t> frozen_identity_reuses_{0};
  inline static std::atomic<std::uint64_t> descriptor_id_lookups_{0};
  inline static std::atomic<std::uint64_t> descriptor_linear_scans_{0};
  inline static std::atomic<std::uint64_t> block_rhs_solve_batches_{0};
  inline static std::atomic<std::uint64_t> block_rhs_unique_blocks_{0};
  inline static std::atomic<std::uint64_t>
      frozen_admission_constant_validations_{0};
  inline static std::atomic<std::uint64_t> pl_payload_validations_{0};
  inline static std::atomic<std::uint64_t> pl_payload_validation_reuses_{0};
  inline static std::atomic<std::uint64_t> frozen_hypothesis_validations_{0};
  inline static std::atomic<std::uint64_t> frozen_hypothesis_validation_reuses_{0};
};

}  // namespace uwb_imu_pl
