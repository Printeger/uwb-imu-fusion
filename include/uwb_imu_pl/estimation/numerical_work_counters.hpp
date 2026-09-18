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
};

// Counts actual decomposition/solve construction points.  The counters are
// process-wide so development runners can report work spanning many windows.
class NumericalWorkCounters {
 public:
  static NumericalWorkSnapshot snapshot() {
    return {base_svd_.load(), base_llt_.load(), base_state_solves_.load(),
            llt_state_solve_calls_.load(), svd_state_solve_calls_.load(),
            detector_reference_qr_.load(),
            candidate_reference_svd_.load(), candidate_inner_llt_.load(),
            fault_gram_eigen_.load(), fault_gram_svd_.load(),
            fault_gram_ldlt_.load(), low_dim_fault_gram_.load(),
            generic_fault_gram_fallback_.load(),
            hypothesis_parallel_blocks_.load(),
            hypothesis_shared_hits_.load(), hypothesis_shared_misses_.load(),
            covariance_rhs_solves_.load(), covariance_rhs_columns_.load(),
            spectral_rhs_solves_.load(), spectral_rhs_columns_.load(),
            numerical_contract_mismatches_.load(),
            imu_oracle_reintegrations_.load()};
  }
  static void reset() {
    base_svd_ = 0; base_llt_ = 0; base_state_solves_ = 0;
    llt_state_solve_calls_ = 0; svd_state_solve_calls_ = 0;
    detector_reference_qr_ = 0;
    candidate_reference_svd_ = 0; candidate_inner_llt_ = 0;
    fault_gram_eigen_ = 0; fault_gram_svd_ = 0; fault_gram_ldlt_ = 0;
    low_dim_fault_gram_ = 0; generic_fault_gram_fallback_ = 0;
    hypothesis_parallel_blocks_ = 0;
    hypothesis_shared_hits_ = 0; hypothesis_shared_misses_ = 0;
    covariance_rhs_solves_ = 0; covariance_rhs_columns_ = 0;
    spectral_rhs_solves_ = 0; spectral_rhs_columns_ = 0;
    numerical_contract_mismatches_ = 0; imu_oracle_reintegrations_ = 0;
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
    ++covariance_rhs_solves_; covariance_rhs_columns_ += columns;
  }
  static void spectralRhsSolve(std::uint64_t columns) {
    ++spectral_rhs_solves_; spectral_rhs_columns_ += columns;
  }
  static void numericalContractMismatch() { ++numerical_contract_mismatches_; }
  static void imuOracleReintegration() { ++imu_oracle_reintegrations_; }

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
};

}  // namespace uwb_imu_pl
