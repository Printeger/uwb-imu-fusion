#pragma once

// B2 (§5.8): coverage certificates and grouped fault envelopes.
//
// The frozen plausible set is enumerated exactly: every hypothesis of the
// frozen registry is traversed in registry order (no top-K, no truncation).
// Coverage is then certified leaf by leaf:
//
//   * EXACT          - the leaf is enumerated by its own hypothesis;
//   * UPPER_ENVELOPE - the leaf is served by a group mode whose fault map
//                      spans the leaf exactly (A_leaf = A_group * T) and whose
//                      risk bound dominates the leaf's own exact bound;
//   * UNCOVERED      - no exact hypothesis and no verified envelope serves the
//                      leaf.  Coverage is then incomplete and the protected
//                      output must be treated as unavailable.
//
// An envelope is only accepted when both obligations are discharged:
//   1. the inclusion identity A_leaf = A_group * T holds within the configured
//      tolerance (the proof stores the transform and the residual), and
//   2. the envelope bound B_env(leaf) is not below the leaf's independently
//      computed bound B_leaf(leaf) beyond the dominance tolerance.
// Envelopes that fail either check are rejected and reported with the reason;
// they are never silently used.

#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace uwb_imu_pl {

enum class CoverageLabel {
  Exact = 0,
  UpperEnvelope = 1,
  Uncovered = 2,
};

const char* toString(CoverageLabel label);

struct CoverageInclusionProof {
  FaultModeId leaf;
  FaultModeId group;
  // Leaf parameters = transform * group parameters.
  Eigen::MatrixXd transform;
  bool verified = false;
  double relative_residual = 0.0;
  std::string reason;
};

struct CoverageEnvelope {
  std::uint64_t id = 0;
  FaultModeId group;
  std::vector<FaultModeId> covered_modes;
  std::vector<CoverageInclusionProof> proofs;
  bool dominant = false;
  // One-sided relative dominance (P7, implemented in P7 commit): the envelope
  // is accepted only when ratio = B_env / B_leaf >= 1 - dominance_tolerance.
  // `dominance_margin` is the **raw** ratio - 1 (minimum over the covered
  // leaves), so an accepted envelope satisfies margin >= -dominance_tolerance;
  // the tolerance is only a binary64 noise floor for the shared identity
  // (observed identity noise ~1e-16), never a licence to under-cover a leaf.
  // `dominance_ratio` carries the same minimum as a raw ratio for audit.
  double dominance_margin = 0.0;
  double dominance_ratio = 0.0;
  bool accepted = false;
  std::string reason;
};

struct CoverageCapacity {
  std::size_t max_envelopes = 64;
  std::size_t max_group_members = 64;
  // Inclusion identity residual bound and one-sided dominance slack (both
  // relative).  The dominance slack only absorbs floating-point noise in the
  // shared Gram identity; it is not a licence to under-cover a leaf.
  double inclusion_tolerance = 1e-9;
  double dominance_tolerance = 1e-9;
};

// Decision rule for the dominance obligation, exposed so the narrow-band
// behaviour can be unit tested directly: an envelope is dominant iff
// env_slope >= leaf_slope * (1 - relative_tolerance).  A non-positive leaf
// bound (no monitorable direction) is trivially dominant.
bool envelopeDominanceAccepts(double envelope_slope, double leaf_slope,
                              double relative_tolerance);

// Raw margin ratio - 1 used when the envelope is accepted.
double envelopeDominanceMargin(double envelope_slope, double leaf_slope);

struct CoverageCertificate {
  std::size_t hypothesis_count = 0;
  std::size_t exact_count = 0;
  std::size_t enveloped_count = 0;
  std::size_t uncovered_count = 0;
  bool complete = false;
  bool capacity_exceeded = false;
  std::vector<CoverageEnvelope> envelopes;
  std::vector<FaultModeId> uncovered_modes;
  std::string reason;
};

// Exact traversal: every mode is enumerated as its own EXACT leaf.
CoverageCertificate buildExactCoverageCertificate(
    const std::vector<FaultModeBasis>& modes);

// Grouped envelopes.  `groups` are the candidate envelope modes (typically the
// modes with the largest physical scope); every mode in `modes` is assigned to
// the first group that provably spans it, otherwise it stays an EXACT leaf.
// The window supplies the whitened fault maps and the frozen solves used for
// the dominance obligation; the builder never mutates the window.
CoverageCertificate buildGroupedCoverageCertificate(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    const std::vector<FaultModeBasis>& groups,
    const CoverageCapacity& capacity = {});

CoverageLabel coverageLabelFor(const CoverageCertificate& certificate,
                               FaultModeId mode);
std::uint64_t coverageEnvelopeIdFor(const CoverageCertificate& certificate,
                                    FaultModeId mode);

}  // namespace uwb_imu_pl
