#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/SparseCore>

#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "uifgo/nlos_recoverability.h"

#ifndef UIFGO_SOURCE_DIR
#define UIFGO_SOURCE_DIR "."
#endif

namespace {

Eigen::MatrixXd MatrixFromNode(const YAML::Node& node, size_t columns_hint = 0) {
  const size_t rows = node.size();
  size_t columns = columns_hint;
  if (rows > 0 && node[0].IsSequence()) columns = node[0].size();
  Eigen::MatrixXd matrix(rows, columns);
  matrix.setZero();
  for (size_t row = 0; row < rows; ++row) {
    if (!node[row].IsSequence() || node[row].size() != columns)
      throw std::runtime_error("matrix fixture is not rectangular");
    for (size_t column = 0; column < columns; ++column)
      matrix(row, column) = node[row][column].as<double>();
  }
  return matrix;
}

Eigen::SparseMatrix<double> SparseFromNode(const YAML::Node& node) {
  const Eigen::MatrixXd dense = MatrixFromNode(node);
  return dense.sparseView();
}

void PrintAudit(const std::string& id,
                const uifgo::RecoverabilityResult& result,
                int frozen_rank = -1) {
  std::cout << "case=" << id << " frozen_svd_rank=" << frozen_rank
            << " naive_qr_rank=" << result.naive_qr_pivot_rank
            << " sparse_qr_rank=" << result.sparse_qr_rank << " permutation=";
  for (int index : result.active_qr_permutation) std::cout << index << ':';
  std::cout << " condition_1=" << result.qr_condition_estimate_1
            << " rcond_1=" << result.qr_rcond_estimate_1
            << " condition_iterations=" << result.condition_estimator_iterations
            << " condition_converged=" << result.condition_estimator_converged
            << " orthogonality=" << result.orthogonality_residual
            << " status=" << uifgo::RecoverabilityStatusName(result.status)
            << '\n';
}

bool NearMatrix(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs,
                double atol = 1e-10, double rtol = 1e-7) {
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) return false;
  for (Eigen::Index row = 0; row < lhs.rows(); ++row)
    for (Eigen::Index column = 0; column < lhs.cols(); ++column) {
      const double limit =
          atol + rtol * std::max(std::abs(lhs(row, column)),
                                 std::abs(rhs(row, column)));
      if (std::abs(lhs(row, column) - rhs(row, column)) > limit) return false;
    }
  return true;
}

}  // namespace

TEST(NlosRecoverability, SparseRankCasesRespectFrozenReferenceBoundary) {
  const YAML::Node root = YAML::LoadFile(
      std::string(UIFGO_SOURCE_DIR) +
      "/test/fixtures/recoverability/sparse_rank_cases.json");
  ASSERT_EQ(root["schema_version"].as<std::string>(),
            "recoverability-sparse-rank-v1");
  ASSERT_GE(root["cases"].size(), 10u);
  for (const auto& item : root["cases"]) {
    const std::string id = item["id"].as<std::string>();
    const auto result = uifgo::ComputeSparseRecoverability(
        SparseFromNode(item["F"]), MatrixFromNode(item["G"]));
    const int frozen_rank = item["frozen_rank"].as<int>();
    std::cout << " frozen_threshold="
              << item["frozen_threshold"].as<double>()
              << " frozen_singular_values=";
    for (const auto& value : item["frozen_singular_values"])
      std::cout << value.as<double>() << ':';
    std::cout << '\n';
    PrintAudit(id, result, frozen_rank);
    EXPECT_EQ(result.naive_qr_pivot_rank,
              item["naive_eigen_qr_rank"].as<int>())
        << id;
    std::set<std::string> allowed;
    for (const auto& status : item["allowed_sparse_status"])
      allowed.insert(status.as<std::string>());
    EXPECT_TRUE(allowed.count(uifgo::RecoverabilityStatusName(result.status)))
        << id << ": " << result.reason;
    if (result.valid_score()) {
      EXPECT_TRUE(NearMatrix(result.R, MatrixFromNode(item["frozen_R"])))
          << id << "\nactual R:\n" << result.R;
      EXPECT_TRUE(std::isfinite(result.n_minus_r_min_eigenvalue)) << id;
      EXPECT_GE(result.n_minus_r_min_eigenvalue,
                -result.n_minus_r_psd_tolerance) << id;
    } else {
      EXPECT_TRUE(result.s_is_infinite) << id;
    }
    if (id == "command_counterexample") {
      EXPECT_EQ(frozen_rank, 1);
      EXPECT_EQ(result.naive_qr_pivot_rank, 2);
      EXPECT_EQ(result.status,
                uifgo::RecoverabilityStatus::SPARSE_RANK_UNCERTAIN);
      EXPECT_TRUE(result.s_is_infinite);
    }
  }
}

TEST(NlosRecoverability,
     TriangularAndDuplicateMultiplicityRequireFullSpectrumCertificate) {
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3, 1);
  G(2, 0) = 1.0;
  Eigen::MatrixXd triangular(3, 3);
  triangular << 1.0, 1.0, 0.0,
                0.0, 1e-6, 1.0,
                0.0, 0.0, 1e-6;
  const auto triangular_result = uifgo::ComputeSparseRecoverability(
      triangular.sparseView(), G);
  EXPECT_EQ(triangular_result.status,
            uifgo::RecoverabilityStatus::SPARSE_RANK_UNCERTAIN);
  EXPECT_FALSE(triangular_result.frozen_rank_certified);
  EXPECT_TRUE(triangular_result.s_is_infinite);

  Eigen::PermutationMatrix<Eigen::Dynamic> permutation(3);
  permutation.indices() << 2, 0, 1;
  const auto permuted_result = uifgo::ComputeSparseRecoverability(
      (triangular * permutation).sparseView(), G);
  EXPECT_EQ(permuted_result.status,
            uifgo::RecoverabilityStatus::SPARSE_RANK_UNCERTAIN);
  EXPECT_TRUE(permuted_result.s_is_infinite);

  auto duplicate_case = [](Eigen::Index duplicate_count) {
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(2, duplicate_count + 1);
    F.row(0).setOnes();
    F(1, duplicate_count) = 1e-9;
    Eigen::MatrixXd local_G(2, 1);
    local_G << 0.0, 1.0;
    return uifgo::ComputeSparseRecoverability(F.sparseView(), local_G);
  };
  const auto few_duplicates = duplicate_case(1);
  ASSERT_TRUE(few_duplicates.frozen_rank_certified)
      << few_duplicates.reason;
  EXPECT_EQ(few_duplicates.frozen_F_rank, 2);
  EXPECT_EQ(few_duplicates.status,
            uifgo::RecoverabilityStatus::RANK_DEFICIENT);

  const auto many_duplicates = duplicate_case(300);
  EXPECT_EQ(many_duplicates.status,
            uifgo::RecoverabilityStatus::SPARSE_RANK_UNCERTAIN);
  EXPECT_FALSE(many_duplicates.frozen_rank_certified);
  EXPECT_TRUE(many_duplicates.s_is_infinite);
  EXPECT_GT(many_duplicates.rank_threshold_upper,
            few_duplicates.rank_threshold_upper);
}

TEST(NlosRecoverability,
     WeakInformationCrossScaleAndNonfiniteOptionsAreAudited) {
  const double weak = 5e-11;
  Eigen::SparseMatrix<double> no_nuisance(1, 0);
  Eigen::MatrixXd weak_G = Eigen::MatrixXd::Constant(1, 1, std::sqrt(weak));
  const auto weak_N = uifgo::ComputeSparseRecoverability(no_nuisance, weak_G);
  ASSERT_EQ(weak_N.status, uifgo::RecoverabilityStatus::OK) << weak_N.reason;
  EXPECT_NEAR(weak_N.R(0, 0), weak, 1e-24);
  EXPECT_NEAR(weak_N.s_m, 1.0 / std::sqrt(weak), 1e-8);
  EXPECT_LT(weak_N.n_minus_r_psd_tolerance, 1e-20);

  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(2, 1);
  F(0, 0) = 1.0;
  Eigen::MatrixXd weak_R_G(2, 1);
  weak_R_G << std::sqrt(1.0 - weak), std::sqrt(weak);
  const auto weak_R =
      uifgo::ComputeSparseRecoverability(F.sparseView(), weak_R_G);
  ASSERT_EQ(weak_R.status, uifgo::RecoverabilityStatus::OK) << weak_R.reason;
  EXPECT_EQ(weak_R.R_rank, 1);
  EXPECT_NEAR(weak_R.s_m, 1.0 / std::sqrt(weak), 1e-8);

  Eigen::MatrixXd orthogonal_G(2, 1);
  orthogonal_G << 0.0, 1.0;
  auto small = uifgo::ComputeSparseRecoverability(
      F.sparseView(), 1e-4 * orthogonal_G);
  auto large = uifgo::ComputeSparseRecoverability(
      F.sparseView(), 1e4 * orthogonal_G);
  ASSERT_EQ(small.status, uifgo::RecoverabilityStatus::OK) << small.reason;
  ASSERT_EQ(large.status, uifgo::RecoverabilityStatus::OK) << large.reason;
  EXPECT_NEAR(small.eta, large.eta, 1e-14);
  EXPECT_NEAR(small.s_m / large.s_m, 1e8, 1e-6);

  uifgo::RecoverabilityOptions invalid_options;
  invalid_options.symmetry_relative_tolerance =
      std::numeric_limits<double>::infinity();
  const auto invalid = uifgo::ComputeSparseRecoverability(
      F.sparseView(), orthogonal_G, invalid_options);
  EXPECT_EQ(invalid.status, uifgo::RecoverabilityStatus::INVALID_INPUT);
  EXPECT_TRUE(invalid.s_is_infinite);
}

TEST(NlosRecoverability, OriginalT03FixturesMatchOnSupportedDomain) {
  const YAML::Node root = YAML::LoadFile(
      std::string(UIFGO_SOURCE_DIR) +
      "/test/fixtures/recoverability/golden_cases.json");
  ASSERT_EQ(root["schema_version"].as<std::string>(),
            "recoverability-golden-v2");
  ASSERT_EQ(root["cases"].size(), 13u);
  for (const auto& item : root["cases"]) {
    const std::string id = item["id"].as<std::string>();
    const size_t nuisance_columns =
        item["expected"]["shape"]["nuisance_columns"].as<size_t>();
    const Eigen::MatrixXd F_dense = MatrixFromNode(item["F"], nuisance_columns);
    const auto result = uifgo::ComputeSparseRecoverability(
        F_dense.sparseView(), MatrixFromNode(item["G"]));
    const int frozen_rank =
        item["expected"]["rank"]["F_scaled"].as<int>();
    std::cout << " frozen_threshold="
              << item["expected"]["thresholds_used"]["F_rank"].as<double>()
              << " frozen_singular_values=";
    for (const auto& value :
         item["expected"]["spectrum"]["F_scaled_singular_values_desc"])
      std::cout << value.as<double>() << ':';
    std::cout << '\n';
    PrintAudit(id, result, frozen_rank);
    ASSERT_TRUE(result.valid_score()) << id << ": " << result.reason;
    EXPECT_TRUE(NearMatrix(result.N, MatrixFromNode(item["expected"]["N"])))
        << id;
    EXPECT_TRUE(NearMatrix(result.R, MatrixFromNode(item["expected"]["R"])))
        << id << "\nactual R:\n" << result.R;
    const std::string expected_status =
        item["expected"]["status"].as<std::string>();
    EXPECT_EQ(std::string(uifgo::RecoverabilityStatusName(result.status)),
              expected_status)
        << id << ": " << result.reason;
    EXPECT_NEAR(result.eta, item["expected"]["eta"].as<double>(), 1e-10)
        << id;
    if (item["expected"]["s_is_infinite"].as<bool>()) {
      EXPECT_TRUE(result.s_is_infinite) << id;
    } else {
      EXPECT_FALSE(result.s_is_infinite) << id;
      EXPECT_NEAR(result.s_m, item["expected"]["s_m"].as<double>(),
                  1e-10 + 1e-7 * std::abs(result.s_m))
          << id;
    }
  }
}

TEST(NlosRecoverability, PermutationAndExactDependencyKeepColumnMapping) {
  Eigen::MatrixXd dense(4, 4);
  dense << 0.0, 1.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0,
           0.0, 1.0, 1.0, 1.0,
           0.0, 0.0, 0.0, 0.0;
  Eigen::MatrixXd G(4, 1);
  G << 0.0, 0.0, 1.0, 1.0;
  const auto result =
      uifgo::ComputeSparseRecoverability(dense.sparseView(), G);
  PrintAudit("mapping", result);
  ASSERT_TRUE(result.valid_score()) << result.reason;
  ASSERT_EQ(result.nuisance_columns.size(), 4u);
  EXPECT_TRUE(result.nuisance_columns[0].exact_zero);
  EXPECT_TRUE(result.nuisance_columns[2].exact_duplicate);
  EXPECT_EQ(result.nuisance_columns[2].representative_original_index, 1);
  EXPECT_EQ(result.exact_zero_columns, 1u);
  EXPECT_EQ(result.exact_duplicate_columns, 1u);
}

TEST(NlosRecoverability, NonfiniteAndLostResolutionNeverExportFiniteS) {
  Eigen::SparseMatrix<double> F(1, 1);
  F.insert(0, 0) = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd G = Eigen::MatrixXd::Ones(1, 1);
  auto invalid = uifgo::ComputeSparseRecoverability(F, G);
  EXPECT_EQ(invalid.status, uifgo::RecoverabilityStatus::INVALID_INPUT);
  EXPECT_TRUE(invalid.s_is_infinite);

  uifgo::RecoverabilityOptions options;
  options.condition_estimator_max_iterations = 1;
  Eigen::MatrixXd lost_F(3, 2);
  lost_F << 1.0, 0.0, 0.0, 1.0, 1.0, 1.0;
  Eigen::MatrixXd lost_G(3, 1);
  lost_G << 1.0, 0.0, 0.0;
  auto lost = uifgo::ComputeSparseRecoverability(
      lost_F.sparseView(), lost_G, options);
  EXPECT_EQ(lost.status,
            uifgo::RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST);
  EXPECT_TRUE(lost.s_is_infinite);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(NlosRecoverability, LcbInverseDiagonalAndInvalidInputs) {
  uifgo::RecoverabilityResult r;
  r.status=uifgo::RecoverabilityStatus::OK;
  r.R.resize(2,2); r.R << 4,3,3,9;
  r.R_rank=2; r.amplitude_columns=2; r.R_rank_pd_threshold=1e-10;
  auto s=uifgo::LocalAmplitudeSigmas(r);
  ASSERT_EQ(s.size(),2u);
  EXPECT_NEAR(s[0],std::sqrt(1.0/3.0),1e-12);
  EXPECT_NEAR(s[1],std::sqrt(4.0/27.0),1e-12);
  EXPECT_GT(s[0],1.0/std::sqrt(r.R(0,0)));
  r.R << 9,3,3,4;
  auto perm=uifgo::LocalAmplitudeSigmas(r);
  ASSERT_EQ(perm.size(),2u); EXPECT_DOUBLE_EQ(perm[1],s[0]);
  r.R << 1,1,1,1; EXPECT_TRUE(uifgo::LocalAmplitudeSigmas(r).empty());
  r.R << 1,2,2,1; EXPECT_TRUE(uifgo::LocalAmplitudeSigmas(r).empty());
  r.R(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(uifgo::LocalAmplitudeSigmas(r).empty());
  r.R.setIdentity(); r.status=uifgo::RecoverabilityStatus::RANK_DEFICIENT;
  EXPECT_TRUE(r.valid_score()); EXPECT_TRUE(uifgo::LocalAmplitudeSigmas(r).empty());
}
