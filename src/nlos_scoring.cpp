#include "uifgo/nlos_scoring.h"

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include "uifgo/uwb_factor.h"

namespace uifgo {
namespace {

class Sha256 {
 public:
  void Update(const void* data, size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < size; ++i) {
      buffer_[buffer_size_++] = bytes[i];
      if (buffer_size_ == buffer_.size()) Transform();
      bit_count_ += 8;
    }
  }

  std::string FinalHex() {
    buffer_[buffer_size_++] = 0x80;
    if (buffer_size_ > 56) {
      while (buffer_size_ < 64) buffer_[buffer_size_++] = 0;
      Transform();
    }
    while (buffer_size_ < 56) buffer_[buffer_size_++] = 0;
    for (int shift = 56; shift >= 0; shift -= 8)
      buffer_[buffer_size_++] =
          static_cast<unsigned char>((bit_count_ >> shift) & 0xff);
    Transform();
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::uint32_t word : state_) out << std::setw(8) << word;
    return out.str();
  }

 private:
  static std::uint32_t Rotate(std::uint32_t value, unsigned count) {
    return (value >> count) | (value << (32 - count));
  }
  void Transform() {
    static constexpr std::array<std::uint32_t, 64> k = {{
      0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
      0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
      0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
      0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
      0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
      0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
      0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
      0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2}};
    std::array<std::uint32_t, 64> w{};
    for (size_t i = 0; i < 16; ++i)
      w[i] = (static_cast<std::uint32_t>(buffer_[4*i]) << 24) |
             (static_cast<std::uint32_t>(buffer_[4*i+1]) << 16) |
             (static_cast<std::uint32_t>(buffer_[4*i+2]) << 8) |
             static_cast<std::uint32_t>(buffer_[4*i+3]);
    for (size_t i = 16; i < 64; ++i) {
      const auto s0 = Rotate(w[i-15],7) ^ Rotate(w[i-15],18) ^ (w[i-15] >> 3);
      const auto s1 = Rotate(w[i-2],17) ^ Rotate(w[i-2],19) ^ (w[i-2] >> 10);
      w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    auto a=state_[0], b=state_[1], c=state_[2], d=state_[3];
    auto e=state_[4], f=state_[5], g=state_[6], h=state_[7];
    for (size_t i = 0; i < 64; ++i) {
      const auto s1 = Rotate(e,6) ^ Rotate(e,11) ^ Rotate(e,25);
      const auto choice = (e & f) ^ (~e & g);
      const auto t1 = h + s1 + choice + k[i] + w[i];
      const auto s0 = Rotate(a,2) ^ Rotate(a,13) ^ Rotate(a,22);
      const auto majority = (a & b) ^ (a & c) ^ (b & c);
      const auto t2 = s0 + majority;
      h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
    state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
    buffer_size_ = 0;
  }
  std::array<std::uint32_t,8> state_{{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                      0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
  std::array<unsigned char,64> buffer_{};
  size_t buffer_size_ = 0;
  std::uint64_t bit_count_ = 0;
};

void HashU64(Sha256* hash, std::uint64_t value) {
  std::array<unsigned char,8> bytes{};
  for (int i = 7; i >= 0; --i) { bytes[i] = value & 0xff; value >>= 8; }
  hash->Update(bytes.data(), bytes.size());
}

void HashString(Sha256* hash, const std::string& value) {
  HashU64(hash, value.size());
  hash->Update(value.data(), value.size());
}

void HashDouble(Sha256* hash, double value) {
  if (value == 0.0) value = 0.0;  // canonicalize negative zero
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  HashU64(hash, bits);
}

std::string LinearizationId(
    const SegmentOverlapGroup& group,
    const std::vector<size_t>& original_indices,
    const std::vector<KeyColumnMeta>& key_columns,
    const std::vector<FactorRowMeta>& factor_rows,
    const Eigen::SparseMatrix<double>& F,
    const Eigen::MatrixXd& G, const Eigen::VectorXd& rhs) {
  Sha256 hash;
  HashString(&hash, "uifgo-t05-linearization-v2");
  HashString(&hash, group.group_id);
  HashU64(&hash, group.segment_ordinals.size());
  for (size_t ordinal : group.segment_ordinals) HashU64(&hash, ordinal);
  HashU64(&hash, original_indices.size());
  for (size_t index : original_indices) HashU64(&hash, index);
  HashU64(&hash, key_columns.size());
  for (const auto& meta : key_columns) {
    HashU64(&hash, meta.key); HashU64(&hash, meta.offset);
    HashU64(&hash, meta.dimension); HashString(&hash, meta.role);
  }
  HashU64(&hash, factor_rows.size());
  for (const auto& meta : factor_rows) {
    HashU64(&hash, meta.original_factor_index); HashU64(&hash, meta.row_offset);
    HashU64(&hash, meta.row_count); HashU64(&hash, meta.obs_id);
    HashString(&hash, meta.factor_type); HashString(&hash, meta.segment_id);
  }
  HashU64(&hash, F.rows()); HashU64(&hash, F.cols()); HashU64(&hash, F.nonZeros());
  for (Eigen::Index column = 0; column < F.outerSize(); ++column)
    for (Eigen::SparseMatrix<double>::InnerIterator it(F, column); it; ++it) {
      HashU64(&hash, it.row()); HashU64(&hash, it.col()); HashDouble(&hash, it.value());
    }
  HashU64(&hash, G.rows()); HashU64(&hash, G.cols());
  for (Eigen::Index row = 0; row < G.rows(); ++row)
    for (Eigen::Index column = 0; column < G.cols(); ++column)
      HashDouble(&hash, G(row,column));
  HashU64(&hash, rhs.size());
  for (Eigen::Index row = 0; row < rhs.size(); ++row) HashDouble(&hash, rhs[row]);
  return "t05lin-sha256:" + hash.FinalHex();
}

std::string KeyRole(gtsam::Key key, const std::set<gtsam::Key>& amplitudes) {
  if (amplitudes.count(key)) return "GROUP_AMPLITUDE";
  switch (gtsam::Symbol(key).chr()) {
    case 'x': return "POSE_NUISANCE";
    case 'v': return "VELOCITY_NUISANCE";
    case 'b': return "IMU_BIAS_NUISANCE";
    case 'z': return "STATIC_BETA_NUISANCE";
    case 'a': return "ANCHOR_NUISANCE";
    case 'l': return "LEVER_NUISANCE";
    default: return "OTHER_NUISANCE";
  }
}

bool IsAmplitudeKey(gtsam::Key key) {
  return gtsam::Symbol(key).chr() == 'c';
}

void ValidateScoreInputs(const SegmentRefitResult& refit,
                         const SupportPartition& support,
                         const PaperInputPlan& plan,
                         const std::set<std::uint64_t>& intentionally_absent) {
  if (support.segments.empty() || refit.segments.size() != support.segments.size())
    throw std::invalid_argument("support/refit segment coverage mismatch");

  std::unordered_map<std::uint64_t, const ObservationRecord*> observations;
  std::set<std::uint64_t> planned_observations;
  for (const auto& observation : plan.observations) {
    if (observation.obs_id == 0 ||
        !observations.emplace(observation.obs_id, &observation).second)
      throw std::invalid_argument("plan contains zero or duplicate obs_id");
    if (observation.valid && observation.planned &&
        !intentionally_absent.count(observation.obs_id)) {
      if (!(observation.nominal_sigma > 0.0) ||
          !std::isfinite(observation.nominal_sigma) ||
          !std::isfinite(observation.raw_range))
        throw std::invalid_argument("planned observation has invalid numerics");
      planned_observations.insert(observation.obs_id);
    }
  }

  std::unordered_map<size_t, const SupportSegment*> support_by_ordinal;
  std::unordered_map<std::string, const SupportSegment*> support_by_id;
  std::unordered_map<std::uint64_t, const SupportSegment*> support_by_obs;
  for (const auto& segment : support.segments) {
    if (segment.segment_id.empty() || !std::isfinite(segment.start_time) ||
        !std::isfinite(segment.end_time) || segment.start_time > segment.end_time ||
        segment.obs_ids.empty() || segment.observation_count != segment.obs_ids.size() ||
        !support_by_ordinal.emplace(segment.segment_ordinal, &segment).second ||
        !support_by_id.emplace(segment.segment_id, &segment).second)
      throw std::invalid_argument("support segment identity or extent is invalid");
    for (std::uint64_t obs_id : segment.obs_ids) {
      const auto observation = observations.find(obs_id);
      if (observation == observations.end() || !observation->second->valid ||
          !observation->second->planned ||
          observation->second->tag_id != segment.tag_id ||
          observation->second->anchor_id != segment.anchor_id ||
          !support_by_obs.emplace(obs_id, &segment).second)
        throw std::invalid_argument(
            "candidate obs_id ownership/link/plan membership is invalid");
    }
  }

  std::set<size_t> estimate_ordinals;
  for (const auto& estimate : refit.segments) {
    const auto found = support_by_ordinal.find(estimate.segment_ordinal);
    if (found == support_by_ordinal.end() ||
        !estimate_ordinals.insert(estimate.segment_ordinal).second)
      throw std::invalid_argument("refit segment has invalid or duplicate ordinal");
    const SupportSegment& segment = *found->second;
    if (estimate.segment_id != segment.segment_id ||
        estimate.tag_id != segment.tag_id || estimate.anchor_id != segment.anchor_id ||
        estimate.observation_count != segment.obs_ids.size() ||
        estimate.start_time != segment.start_time ||
        estimate.end_time != segment.end_time ||
        estimate.amplitude_key != gtsam::Symbol('c', estimate.segment_ordinal))
      throw std::invalid_argument("refit segment metadata disagrees with support");
  }

  std::set<std::uint64_t> mapped_observations;
  std::set<std::uint64_t> mapped_candidates;
  std::set<gtsam::Key> graph_keys;
  for (size_t index = 0; index < refit.graph.size(); ++index) {
    const auto factor = refit.graph.at(index);
    const auto& meta = refit.factor_metadata[index];
    if (!factor || meta.factor_index != index)
      throw std::invalid_argument("factor index metadata is invalid");
    const std::vector<gtsam::Key> actual_keys(factor->keys().begin(),
                                              factor->keys().end());
    for (gtsam::Key key : actual_keys) {
      graph_keys.insert(key);
      if (!refit.values.exists(key))
        throw std::invalid_argument("scoring graph key is absent from Values");
    }
    if (actual_keys != meta.keys)
      throw std::invalid_argument("factor metadata keys differ from actual keys");
    const bool has_amplitude =
        std::any_of(actual_keys.begin(), actual_keys.end(), IsAmplitudeKey);
    if (meta.factor_type == "preserved_non_uwb") {
      if (meta.obs_id != 0 || !meta.segment_id.empty() || has_amplitude)
        throw std::invalid_argument("preserved factor metadata is inconsistent");
      continue;
    }
    if (meta.factor_type != "uwb_range" &&
        meta.factor_type != "uwb_segment_range")
      throw std::invalid_argument("factor type is not in the scoring whitelist");
    const auto observation = observations.find(meta.obs_id);
    if (observation == observations.end() || !observation->second->valid ||
        !observation->second->planned ||
        !mapped_observations.insert(meta.obs_id).second)
      throw std::invalid_argument("UWB obs_id mapping is invalid or duplicated");
    const auto candidate = support_by_obs.find(meta.obs_id);
    if (meta.factor_type == "uwb_range") {
      if (candidate != support_by_obs.end() || !meta.segment_id.empty() ||
          has_amplitude ||
          std::find(actual_keys.begin(), actual_keys.end(),
                    gtsam::Symbol('x', observation->second->keyframe_id)) ==
              actual_keys.end())
        throw std::invalid_argument("noncandidate UWB factor metadata is inconsistent");
      continue;
    }
    if (candidate == support_by_obs.end() ||
        meta.segment_id != candidate->second->segment_id)
      throw std::invalid_argument("candidate factor is outside declared support");
    const gtsam::Key expected_amplitude =
        gtsam::Symbol('c', candidate->second->segment_ordinal);
    if (std::find(actual_keys.begin(), actual_keys.end(),
                  gtsam::Symbol('x', observation->second->keyframe_id)) ==
        actual_keys.end())
      throw std::invalid_argument("candidate factor has wrong pose key");
    size_t expected_count = 0;
    for (gtsam::Key key : actual_keys) {
      if (key == expected_amplitude) ++expected_count;
      else if (IsAmplitudeKey(key))
        throw std::invalid_argument("candidate factor references another segment");
    }
    if (expected_count != 1 || !mapped_candidates.insert(meta.obs_id).second)
      throw std::invalid_argument("candidate amplitude/ownership mapping is invalid");
  }
  if (mapped_observations != planned_observations)
    throw std::invalid_argument("planned UWB observations are not completely covered");
  std::set<std::uint64_t> candidate_observations;
  for (const auto& item : support_by_obs) candidate_observations.insert(item.first);
  if (mapped_candidates != candidate_observations)
    throw std::invalid_argument("candidate support is not completely covered");
  const auto value_key_vector = refit.values.keys();
  const std::set<gtsam::Key> value_keys(value_key_vector.begin(),
                                        value_key_vector.end());
  if (graph_keys != value_keys)
    throw std::invalid_argument("scoring graph and Values key sets differ");
}

}  // namespace

std::vector<SegmentOverlapGroup> BuildClosedIntervalOverlapGroups(
    const std::vector<SegmentEstimate>& segments) {
  std::vector<const SegmentEstimate*> ordered;
  ordered.reserve(segments.size());
  for (const auto& segment : segments) ordered.push_back(&segment);
  std::sort(ordered.begin(), ordered.end(), [](const auto* lhs, const auto* rhs) {
    return std::tie(lhs->start_time, lhs->end_time, lhs->segment_id) <
           std::tie(rhs->start_time, rhs->end_time, rhs->segment_id);
  });
  std::vector<SegmentOverlapGroup> groups;
  for (const auto* segment : ordered) {
    if (groups.empty() || segment->start_time > groups.back().end_time) {
      SegmentOverlapGroup group;
      group.group_id = "group_" + std::to_string(groups.size());
      group.start_time = segment->start_time;
      group.end_time = segment->end_time;
      group.segment_ordinals.push_back(segment->segment_ordinal);
      groups.push_back(std::move(group));
    } else {
      groups.back().end_time = std::max(groups.back().end_time,
                                        segment->end_time);
      groups.back().segment_ordinals.push_back(segment->segment_ordinal);
    }
  }
  return groups;
}

std::vector<GroupRecoverabilityScore> ScoreRefitRecoverability(
    const SegmentRefitResult& refit, const SupportPartition& support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options) {
  if (!refit.successful())
    throw std::invalid_argument("recoverability requires converged Stage 2");
  if (cfg.calib_lever || cfg.calib_anchor || cfg.calib_td)
    throw std::invalid_argument("scoring input has unsupported free calibration");
  if (cfg.calib_range_bias && !cfg.fixed_beta_by_link.empty())
    throw std::invalid_argument("fixed and online beta cannot both enter scoring");
  if (refit.factor_metadata.size() != refit.graph.size())
    throw std::invalid_argument("factor metadata and graph sizes differ");
  ValidateScoreInputs(refit, support, plan, {});

  std::unordered_map<std::uint64_t, const ObservationRecord*> observations;
  for (const auto& observation : plan.observations)
    observations.emplace(observation.obs_id, &observation);
  std::unordered_map<size_t, const SupportSegment*> support_by_ordinal;
  for (const auto& segment : support.segments)
    support_by_ordinal.emplace(segment.segment_ordinal, &segment);
  std::unordered_map<std::string, size_t> ordinal_by_id;
  for (const auto& segment : support.segments)
    ordinal_by_id.emplace(segment.segment_id, segment.segment_ordinal);

  const auto groups = BuildClosedIntervalOverlapGroups(refit.segments);
  std::vector<GroupRecoverabilityScore> output;
  for (const auto& group : groups) {
    const auto started = std::chrono::steady_clock::now();
    GroupRecoverabilityScore score;
    score.group = group;
    std::set<size_t> group_ordinals(group.segment_ordinals.begin(),
                                    group.segment_ordinals.end());
    std::set<gtsam::Key> amplitude_keys;
    for (size_t ordinal : group.segment_ordinals)
      amplitude_keys.insert(gtsam::Symbol('c', ordinal));

    gtsam::NonlinearFactorGraph graph;
    std::vector<size_t> original_indices;
    for (size_t index = 0; index < refit.graph.size(); ++index) {
      const auto& meta = refit.factor_metadata[index];
      bool include = meta.factor_type != "uwb_segment_range";
      if (!include) {
        const auto found = ordinal_by_id.find(meta.segment_id);
        if (found == ordinal_by_id.end())
          throw std::invalid_argument("segment factor has unknown segment ID");
        include = group_ordinals.count(found->second) != 0;
      }
      if (include) {
        graph.add(refit.graph.at(index));
        original_indices.push_back(index);
      }
    }
    size_t included_group_candidates = 0;
    for (size_t original : original_indices) {
      const auto& meta = refit.factor_metadata[original];
      if (meta.factor_type != "uwb_segment_range") continue;
      const auto found = ordinal_by_id.find(meta.segment_id);
      if (found == ordinal_by_id.end() || !group_ordinals.count(found->second))
        throw std::invalid_argument("scoring factor mask includes another group");
      ++included_group_candidates;
    }
    size_t expected_group_candidates = 0;
    for (size_t ordinal : group.segment_ordinals)
      expected_group_candidates += support_by_ordinal.at(ordinal)->obs_ids.size();
    if (included_group_candidates != expected_group_candidates)
      throw std::invalid_argument("scoring factor mask has incomplete group coverage");
    const auto linear = graph.linearize(refit.values);
    if (!linear || linear->empty())
      throw std::runtime_error("scoring graph linearization is empty");

    const auto dimensions = linear->getKeyDimMap();
    gtsam::Ordering ordering;
    for (const auto& key_dimension : dimensions)
      if (!amplitude_keys.count(key_dimension.first))
        ordering.push_back(key_dimension.first);
    const size_t nuisance_dimension = [&]() {
      size_t total = 0;
      for (gtsam::Key key : ordering) total += dimensions.at(key);
      return total;
    }();
    for (gtsam::Key key : amplitude_keys) {
      if (!dimensions.count(key))
        throw std::runtime_error("group amplitude key missing from scoring graph");
      ordering.push_back(key);
    }

    size_t offset = 0;
    for (gtsam::Key key : ordering) {
      KeyColumnMeta meta;
      meta.key = key;
      meta.offset = offset;
      meta.dimension = dimensions.at(key);
      meta.role = KeyRole(key, amplitude_keys);
      score.key_columns.push_back(meta);
      offset += meta.dimension;
    }

    size_t rows = 0, columns = 0;
    const auto entries = linear->sparseJacobian(ordering, rows, columns);
    if (columns != offset + 1)
      throw std::runtime_error("augmented Jacobian column count mismatch");
    std::vector<Eigen::Triplet<double>> f_entries;
    score.G_whitened = Eigen::MatrixXd::Zero(rows, amplitude_keys.size());
    score.rhs_whitened = Eigen::VectorXd::Zero(rows);
    for (const auto& entry : entries) {
      const int row = std::get<0>(entry);
      const int column = std::get<1>(entry);
      const double value = std::get<2>(entry);
      if (column < static_cast<int>(nuisance_dimension))
        f_entries.emplace_back(row, column, value);
      else if (column < static_cast<int>(offset))
        score.G_whitened(row, column - nuisance_dimension) = value;
      else
        score.rhs_whitened[row] = value;
    }
    score.F_whitened.resize(rows, nuisance_dimension);
    score.F_whitened.setFromTriplets(f_entries.begin(), f_entries.end());
    score.F_whitened.makeCompressed();

    size_t row_offset = 0;
    for (size_t local = 0; local < original_indices.size(); ++local) {
      const size_t original = original_indices[local];
      const auto gaussian = refit.graph.at(original)->linearize(refit.values);
      if (!gaussian) throw std::runtime_error("factor linearization is null");
      const size_t row_count = gaussian->augmentedJacobian().rows();
      const auto& source = refit.factor_metadata[original];
      score.factor_rows.push_back({original, row_offset, row_count,
                                   source.obs_id, source.factor_type,
                                   source.segment_id});
      row_offset += row_count;
    }
    if (row_offset != rows)
      throw std::runtime_error("factor-to-row accounting mismatch");

    score.linearization_id = LinearizationId(
        group, original_indices, score.key_columns, score.factor_rows,
        score.F_whitened, score.G_whitened, score.rhs_whitened);

    score.numerical = ComputeSparseRecoverability(
        score.F_whitened, score.G_whitened, options);
    bool group_eligible = true;
    for (size_t ordinal : group.segment_ordinals) {
      const auto estimate_it = std::find_if(
          refit.segments.begin(), refit.segments.end(),
          [&](const SegmentEstimate& item) { return item.segment_ordinal == ordinal; });
      if (estimate_it == refit.segments.end())
        throw std::runtime_error("group refers to missing segment estimate");
      const SupportSegment& segment = *support_by_ordinal.at(ordinal);
      double sum = 0.0;
      for (std::uint64_t obs_id : segment.obs_ids) {
        const ObservationRecord& observation = *observations.at(obs_id);
        const auto pose = refit.values.at<gtsam::Pose3>(
            gtsam::Symbol('x', observation.keyframe_id));
        const auto anchor = std::find_if(
            cfg.anchors.begin(), cfg.anchors.end(),
            [&](const AnchorConfig& item) { return item.id == observation.anchor_id; });
        if (anchor == cfg.anchors.end()) throw std::runtime_error("anchor missing");
        const double geometric =
            (gtsam::Vector3(pose.transformFrom(cfg.lever_arm_init)) -
             gtsam::Vector3(anchor->pos)).norm();
        const double online_beta = cfg.calib_range_bias
            ? refit.values.at<double>(gtsam::Symbol('z', observation.anchor_id))
            : 0.0;
        const double residual = geometric +
            FixedBetaForLink(cfg, observation.tag_id, observation.anchor_id) +
            online_beta +
            refit.values.at<double>(gtsam::Symbol('c', ordinal)) -
            observation.raw_range;
        const double normalized = residual / observation.nominal_sigma;
        sum += normalized * normalized;
      }
      SegmentFitScore fit;
      fit.segment_ordinal = ordinal;
      fit.segment_id = estimate_it->segment_id;
      fit.gamma = sum / segment.obs_ids.size();
      fit.short_support_debug = estimate_it->short_support_debug;
      fit.boundary = estimate_it->boundary;
      score.segment_fit.push_back(fit);
      group_eligible = group_eligible && !fit.short_support_debug && !fit.boundary;
    }
    score.eligible = group_eligible;
    score.valid_score_exported = group_eligible && score.numerical.valid_score();
    if (!group_eligible)
      score.status = "INELIGIBLE_SHORT_OR_BOUNDARY_DEBUG";
    else
      score.status = RecoverabilityStatusName(score.numerical.status);
    score.elapsed_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    output.push_back(std::move(score));
  }
  return output;
}

std::vector<GroupRecoverabilityScore> ScoreRefitRecoverability(
    const SegmentRefitResult& refit, const OracleSupport& support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options) {
  return ScoreRefitRecoverability(refit, ToSupportPartition(support), plan,
                                  cfg, options);
}

std::vector<GroupRecoverabilityScore> ScoreFinalRefitRecoverability(
    const SegmentRefitResult& final_refit,
    const SupportPartition& accepted_support,
    const SupportPartition& frozen_full_support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options) {
  std::set<std::uint64_t> accepted;
  for (const auto& segment : accepted_support.segments)
    accepted.insert(segment.obs_ids.begin(), segment.obs_ids.end());
  std::set<std::uint64_t> intentionally_absent;
  for (const auto& segment : frozen_full_support.segments) {
    for (std::uint64_t obs_id : segment.obs_ids) {
      if (!accepted.count(obs_id)) intentionally_absent.insert(obs_id);
    }
  }
  if (!final_refit.successful())
    throw std::invalid_argument("final recoverability requires converged refit");
  if (cfg.calib_lever || cfg.calib_anchor || cfg.calib_td)
    throw std::invalid_argument("final scoring input has unsupported free calibration");
  if (cfg.calib_range_bias && !cfg.fixed_beta_by_link.empty())
    throw std::invalid_argument("fixed and online beta cannot both enter final scoring");
  if (final_refit.factor_metadata.size() != final_refit.graph.size())
    throw std::invalid_argument("final factor metadata and graph sizes differ");
  ValidateScoreInputs(final_refit, accepted_support, plan,
                      intentionally_absent);

  // The validated final graph now has exactly the same structural contract as
  // a regular Stage-2 scoring input over accepted_support.  Reuse the physical
  // assembly without weakening the normal validator by passing a plan view in
  // which only intentionally absent suppressed observations are unplanned.
  PaperInputPlan final_plan = plan;
  for (auto& observation : final_plan.observations) {
    if (intentionally_absent.count(observation.obs_id))
      observation.planned = false;
  }
  return ScoreRefitRecoverability(final_refit, accepted_support, final_plan,
                                  cfg, options);
}

}  // namespace uifgo
