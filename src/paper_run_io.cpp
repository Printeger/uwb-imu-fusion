#include "uifgo/paper_run_io.h"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <typeinfo>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

#include "uifgo/hash_utils.h"
#include "uifgo/nlos_solver_utils.h"

namespace uifgo {
namespace {

void Field(std::ostringstream* bytes, const std::string& value) {
  *bytes << value.size() << ':' << value << '\n';
}

void Require(const std::string& value, const char* name) {
  if (value.empty()) throw std::invalid_argument(std::string(name) + " is empty");
}

std::string ReadAll(const boost::filesystem::path& path) {
  std::ifstream input(path.string(), std::ios::binary);
  if (!input) throw std::runtime_error("cannot read artifact " + path.string());
  std::ostringstream bytes;
  bytes << input.rdbuf();
  return bytes.str();
}

std::string CsvFirstField(const std::string& line) {
  if (line.empty()) return {};
  if (line.front() != '"') {
    const size_t comma = line.find(',');
    return line.substr(0, comma);
  }
  std::string value;
  for (size_t i = 1; i < line.size(); ++i) {
    if (line[i] != '"') {
      value.push_back(line[i]);
    } else if (i + 1 < line.size() && line[i + 1] == '"') {
      value.push_back('"');
      ++i;
    } else {
      return value;
    }
  }
  throw std::runtime_error("unterminated quoted CSV field");
}

void VerifyInferenceIdColumn(const boost::filesystem::path& path,
                             const std::string& inference_id) {
  if (path.extension() != ".csv") return;
  std::ifstream input(path.string());
  if (!input) throw std::runtime_error("cannot read CSV artifact");
  std::string line;
  if (!std::getline(input, line) || CsvFirstField(line) != "inference_id")
    return;
  size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (line.empty()) continue;
    if (CsvFirstField(line) != inference_id)
      throw std::runtime_error("inference_id mismatch in " +
                               path.filename().string() + " row " +
                               std::to_string(row));
  }
}

void RequireJsonCount(const std::string& json, const std::string& field,
                      size_t expected) {
  const std::string token = "\"" + field + "\": " +
                            std::to_string(expected);
  if (json.find(token) == std::string::npos)
    throw std::runtime_error("exported summary count mismatch: " + field);
}

std::uint64_t DoubleBits(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 size mismatch");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

double BitsDouble(std::uint64_t bits) {
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::vector<std::string> SplitSimpleCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

void AtomicWrite(const boost::filesystem::path& path,
                 const std::string& bytes) {
  const boost::filesystem::path temporary(path.string() + ".tmp");
  {
    std::ofstream output(temporary.string(), std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("STAGE2_EXPORT_OPEN_FAILED:" + path.string());
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.flush();
    if (!output) {
      boost::filesystem::remove(temporary);
      throw std::runtime_error("STAGE2_EXPORT_WRITE_FAILED:" + path.string());
    }
  }
  if (std::rename(temporary.string().c_str(), path.string().c_str()) != 0) {
    boost::filesystem::remove(temporary);
    throw std::runtime_error("STAGE2_EXPORT_RENAME_FAILED:" + path.string() +
                             ":errno=" + std::to_string(errno));
  }
}

void EmitStage2Scalar(std::ostringstream* output, gtsam::Key key,
                      const char* type, size_t dimension,
                      const std::string& coordinate, double value,
                      size_t* scalar_count) {
  if (!std::isfinite(value))
    throw std::runtime_error("STAGE2_EXPORT_NONFINITE_VALUE:" +
                             gtsam::DefaultKeyFormatter(key) + ":" + coordinate);
  const gtsam::Symbol symbol(key);
  *output << key << ',' << symbol.chr() << ',' << symbol.index() << ',' << type
          << ',' << dimension << ',' << coordinate << ',' << std::hexfloat
          << value << std::defaultfloat << ',' << std::hex << std::setw(16)
          << std::setfill('0') << DoubleBits(value) << std::dec
          << std::setfill(' ') << '\n';
  ++*scalar_count;
}

std::string SerializeStage2Values(const gtsam::Values& values,
                                  size_t* scalar_count) {
  std::ostringstream output;
  output << "key_value,key_symbol,key_index,value_type,dimension,coordinate,value_hex,bits\n";
  *scalar_count = 0;
  for (gtsam::Key key : values.keys()) {
    const char symbol = gtsam::Symbol(key).chr();
    if (symbol == 'x') {
      const gtsam::Matrix4 matrix = values.at<gtsam::Pose3>(key).matrix();
      for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
          EmitStage2Scalar(&output, key, "Pose3", 6,
                           "m" + std::to_string(row) + std::to_string(column),
                           matrix(row, column), scalar_count);
    } else if (symbol == 'v') {
      const gtsam::Vector3 vector = values.at<gtsam::Vector3>(key);
      for (int index = 0; index < 3; ++index)
        EmitStage2Scalar(&output, key, "Vector3", 3,
                         "v" + std::to_string(index), vector[index], scalar_count);
    } else if (symbol == 'b') {
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key);
      const gtsam::Vector6 vector = bias.vector();
      for (int index = 0; index < 3; ++index)
        EmitStage2Scalar(&output, key, "ConstantBias", 6,
                         "accel" + std::to_string(index), vector[index], scalar_count);
      for (int index = 0; index < 3; ++index)
        EmitStage2Scalar(&output, key, "ConstantBias", 6,
                         "gyro" + std::to_string(index), vector[index + 3], scalar_count);
    } else if (symbol == 'c') {
      EmitStage2Scalar(&output, key, "double", 1, "value",
                       values.at<double>(key), scalar_count);
    } else {
      throw std::runtime_error("STAGE2_EXPORT_UNSUPPORTED_VALUE:" +
                               gtsam::DefaultKeyFormatter(key));
    }
  }
  return output.str();
}

void RequireFactorMetadata(const gtsam::NonlinearFactorGraph& graph,
                           const std::vector<RefitFactorMeta>& metadata) {
  if (metadata.size() != graph.size())
    throw std::runtime_error("STAGE2_EXPORT_FACTOR_METADATA_COUNT_MISMATCH");
  std::set<size_t> seen;
  for (const auto& item : metadata) {
    if (item.factor_index >= graph.size() || !graph[item.factor_index] ||
        !seen.insert(item.factor_index).second)
      throw std::runtime_error("STAGE2_EXPORT_FACTOR_METADATA_INDEX_INVALID");
    const auto actual_keys = graph[item.factor_index]->keys();
    if (item.keys.size() != actual_keys.size() ||
        !std::equal(item.keys.begin(), item.keys.end(), actual_keys.begin()))
      throw std::runtime_error("STAGE2_EXPORT_FACTOR_METADATA_KEYS_MISMATCH");
  }
  if (seen.size() != graph.size())
    throw std::runtime_error("STAGE2_EXPORT_FACTOR_METADATA_INCOMPLETE");
}

}  // namespace

std::string ComputeCommonPreparationId(
    const CommonPreparationIdentityInput& input) {
  Require(input.raw_source_sha256, "raw_source_sha256");
  Require(input.window_role_split_cutoff_sha256,
          "window_role_split_cutoff_sha256");
  Require(input.observation_ledger_sha256, "observation_ledger_sha256");
  Require(input.input_plan_sha256, "input_plan_sha256");
  Require(input.nominal_sigma_sha256, "nominal_sigma_sha256");
  Require(input.initialization_rule, "initialization_rule");
  Require(input.initial_values_sha256, "initial_values_sha256");
  Require(input.calibration_sha256, "calibration_sha256");
  Require(input.physical_graph_sha256, "physical_graph_sha256");
  Require(input.factor_metadata_sha256, "factor_metadata_sha256");
  Require(input.solver_precision_sha256, "solver_precision_sha256");
  std::ostringstream bytes;
  bytes << "uifgo-t09-common-preparation-v1\n";
  Field(&bytes, input.raw_source_sha256);
  Field(&bytes, input.window_role_split_cutoff_sha256);
  Field(&bytes, input.observation_ledger_sha256);
  Field(&bytes, input.input_plan_sha256);
  Field(&bytes, input.nominal_sigma_sha256);
  Field(&bytes, input.initialization_rule);
  Field(&bytes, input.initial_values_sha256);
  Field(&bytes, input.calibration_sha256);
  Field(&bytes, input.physical_graph_sha256);
  Field(&bytes, input.factor_metadata_sha256);
  Field(&bytes, input.solver_precision_sha256);
  return "t09common-sha256:" + Sha256Hex(bytes.str());
}

std::string ComputeFinalRequestId(const FinalRequestIdentityInput& input) {
  Require(input.stage2_cache_id, "stage2_cache_id");
  Require(input.canonical_mode, "canonical_mode");
  Require(input.policy_version, "policy_version");
  Require(input.thresholds_sha256, "thresholds_sha256");
  Require(input.threshold_provenance, "threshold_provenance");
  Require(input.final_refit_score_config_sha256,
          "final_refit_score_config_sha256");
  Require(input.solver_sha256, "solver_sha256");
  Require(input.common_preparation_id, "common_preparation_id");
  std::ostringstream bytes;
  bytes << "uifgo-t09-final-request-v1\n";
  Field(&bytes, input.stage2_cache_id);
  Field(&bytes, input.canonical_mode);
  Field(&bytes, input.policy_version);
  Field(&bytes, input.thresholds_sha256);
  Field(&bytes, input.threshold_provenance);
  Field(&bytes, input.final_refit_score_config_sha256);
  Field(&bytes, input.solver_sha256);
  Field(&bytes, input.common_preparation_id);
  return "t09finalrequest-sha256:" + Sha256Hex(bytes.str());
}

ExportVerificationResult VerifyInferenceExport(
    const std::string& output_directory, const InferenceResult& result,
    const std::vector<std::string>& exported_files) {
  ExportVerificationResult output;
  std::string identity_reason;
  if (!VerifyInferenceContentIdentity(result, &identity_reason)) {
    output.reason = identity_reason;
    return output;
  }
  try {
    const boost::filesystem::path root(output_directory);
    const auto identity_path = root / "final_content_identity.json";
    if (!boost::filesystem::is_regular_file(identity_path))
      throw std::runtime_error("final_content_identity.json is missing");
    const std::string identity = ReadAll(identity_path);
    if (identity.find(result.inference_id) == std::string::npos ||
        identity.find(result.content_identity.graph_linearization_sha256) ==
            std::string::npos ||
        identity.find(result.content_identity.values_sha256) ==
            std::string::npos ||
        identity.find(result.content_identity.context_sha256) ==
            std::string::npos)
      throw std::runtime_error("exported content identity does not match result");
    const auto summary_path = root / "final_inference_summary.json";
    if (!boost::filesystem::is_regular_file(summary_path))
      throw std::runtime_error("final_inference_summary.json is missing");
    const std::string summary = ReadAll(summary_path);
    if (summary.find(result.inference_id) == std::string::npos)
      throw std::runtime_error("summary inference_id does not match result");
    RequireJsonCount(summary, "final_graph_factor_count",
                     result.final_graph.size());
    RequireJsonCount(summary, "final_values_count",
                     result.final_values.size());
    for (const auto& name : exported_files) {
      if (name.empty() || name.find('/') != std::string::npos ||
          name == "." || name == "..")
        throw std::runtime_error("exported artifact name is unsafe");
      const auto path = root / name;
      if (!boost::filesystem::is_regular_file(path))
        throw std::runtime_error("exported artifact is missing: " + name);
      VerifyInferenceIdColumn(path, result.inference_id);
      output.artifact_sha256.push_back(
          name + "=sha256:" + Sha256FileHex(path.string()));
      ++output.checked_files;
    }
    output.ok = true;
    output.status = "VERIFIED";
    output.reason =
        "IN_MEMORY_IDENTITY_COUNTS_ROWS_AND_ARTIFACT_HASHES_VERIFIED";
  } catch (const std::exception& error) {
    output.reason = error.what();
  }
  return output;
}

gtsam::Values ReadDevelopmentStage2Values(const std::string& values_csv) {
  std::ifstream input(values_csv);
  if (!input) throw std::runtime_error("STAGE2_VALUES_READ_OPEN_FAILED");
  std::string line;
  if (!std::getline(input, line) || line !=
      "key_value,key_symbol,key_index,value_type,dimension,coordinate,value_hex,bits")
    throw std::runtime_error("STAGE2_VALUES_HEADER_INVALID");
  struct Entry {
    char symbol = 0;
    size_t index = 0;
    std::string type;
    size_t dimension = 0;
    std::map<std::string, double> coordinates;
  };
  std::map<gtsam::Key, Entry> entries;
  size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (line.empty()) continue;
    const auto fields = SplitSimpleCsv(line);
    if (fields.size() != 8)
      throw std::runtime_error("STAGE2_VALUES_COLUMN_COUNT:" + std::to_string(row));
    const gtsam::Key key = static_cast<gtsam::Key>(std::stoull(fields[0]));
    if (fields[1].size() != 1)
      throw std::runtime_error("STAGE2_VALUES_SYMBOL_INVALID");
    const char symbol = fields[1][0];
    const size_t index = std::stoull(fields[2]);
    if (gtsam::Symbol(key).chr() != symbol || gtsam::Symbol(key).index() != index)
      throw std::runtime_error("STAGE2_VALUES_KEY_ENCODING_MISMATCH");
    const size_t dimension = std::stoull(fields[4]);
    size_t consumed = 0;
    const std::uint64_t bits = std::stoull(fields[7], &consumed, 16);
    if (consumed != fields[7].size())
      throw std::runtime_error("STAGE2_VALUES_BITS_INVALID");
    const double value = BitsDouble(bits);
    if (!std::isfinite(value))
      throw std::runtime_error("STAGE2_VALUES_NONFINITE");
    char* hex_end = nullptr;
    errno = 0;
    const double hex_value = std::strtod(fields[6].c_str(), &hex_end);
    if (errno != 0 || hex_end != fields[6].c_str() + fields[6].size() ||
        DoubleBits(hex_value) != bits)
      throw std::runtime_error("STAGE2_VALUES_HEX_BITS_MISMATCH");
    auto& entry = entries[key];
    if (entry.type.empty()) {
      entry.symbol = symbol;
      entry.index = index;
      entry.type = fields[3];
      entry.dimension = dimension;
    } else if (entry.symbol != symbol || entry.index != index ||
               entry.type != fields[3] || entry.dimension != dimension) {
      throw std::runtime_error("STAGE2_VALUES_KEY_METADATA_COLLISION");
    }
    if (!entry.coordinates.emplace(fields[5], value).second)
      throw std::runtime_error("STAGE2_VALUES_DUPLICATE_COORDINATE");
  }
  if (entries.empty()) throw std::runtime_error("STAGE2_VALUES_EMPTY");
  gtsam::Values values;
  for (const auto& item : entries) {
    const gtsam::Key key = item.first;
    const Entry& entry = item.second;
    if (entry.symbol == 'x' && entry.type == "Pose3" &&
        entry.dimension == 6 && entry.coordinates.size() == 16) {
      gtsam::Matrix4 matrix;
      for (int row_index = 0; row_index < 4; ++row_index)
        for (int column = 0; column < 4; ++column)
          matrix(row_index, column) = entry.coordinates.at(
              "m" + std::to_string(row_index) + std::to_string(column));
      values.insert(key, gtsam::Pose3(matrix));
    } else if (entry.symbol == 'v' && entry.type == "Vector3" &&
               entry.dimension == 3 && entry.coordinates.size() == 3) {
      gtsam::Vector3 vector;
      for (int index = 0; index < 3; ++index)
        vector[index] = entry.coordinates.at("v" + std::to_string(index));
      values.insert(key, vector);
    } else if (entry.symbol == 'b' && entry.type == "ConstantBias" &&
               entry.dimension == 6 && entry.coordinates.size() == 6) {
      gtsam::Vector3 accel, gyro;
      for (int index = 0; index < 3; ++index) {
        accel[index] = entry.coordinates.at("accel" + std::to_string(index));
        gyro[index] = entry.coordinates.at("gyro" + std::to_string(index));
      }
      values.insert(key, gtsam::imuBias::ConstantBias(accel, gyro));
    } else if (entry.symbol == 'c' && entry.type == "double" &&
               entry.dimension == 1 && entry.coordinates.size() == 1) {
      values.insert(key, entry.coordinates.at("value"));
    } else {
      throw std::runtime_error("STAGE2_VALUES_TYPE_DIMENSION_OR_COORDINATES_INVALID");
    }
  }
  return values;
}

void MarkDevelopmentStage2ScoreComplete(
    const std::string& stage2_directory) {
  const boost::filesystem::path path =
      boost::filesystem::path(stage2_directory) / "stage2_export_manifest.json";
  std::string bytes = ReadAll(path);
  const std::string before = "\"score_complete\": false";
  const std::string after = "\"score_complete\": true";
  const size_t position = bytes.find(before);
  if (position == std::string::npos || bytes.find(before, position + 1) !=
                                          std::string::npos ||
      bytes.find("\"solver_converged\": true") == std::string::npos ||
      bytes.find("\"export_complete\": true") == std::string::npos)
    throw std::runtime_error("STAGE2_SCORE_STATUS_MANIFEST_INVALID");
  bytes.replace(position, before.size(), after);
  AtomicWrite(path, bytes);
}

bool DevelopmentStage2ValuesBitEqual(const gtsam::Values& expected,
                                     const gtsam::Values& actual,
                                     std::string* reason) {
  if (expected.keys() != actual.keys()) {
    if (reason) *reason = "KEY_SET_MISMATCH";
    return false;
  }
  for (gtsam::Key key : expected.keys()) {
    if (typeid(expected.at(key)) != typeid(actual.at(key)) ||
        expected.at(key).dim() != actual.at(key).dim()) {
      if (reason) *reason = "TYPE_OR_DIMENSION_MISMATCH:" +
                            gtsam::DefaultKeyFormatter(key);
      return false;
    }
    std::vector<double> left, right;
    const char symbol = gtsam::Symbol(key).chr();
    if (symbol == 'x') {
      const auto a = expected.at<gtsam::Pose3>(key).matrix();
      const auto b = actual.at<gtsam::Pose3>(key).matrix();
      left.assign(a.data(), a.data() + 16); right.assign(b.data(), b.data() + 16);
    } else if (symbol == 'v') {
      const auto a = expected.at<gtsam::Vector3>(key);
      const auto b = actual.at<gtsam::Vector3>(key);
      left.assign(a.data(), a.data() + 3); right.assign(b.data(), b.data() + 3);
    } else if (symbol == 'b') {
      const auto a = expected.at<gtsam::imuBias::ConstantBias>(key).vector();
      const auto b = actual.at<gtsam::imuBias::ConstantBias>(key).vector();
      left.assign(a.data(), a.data() + 6); right.assign(b.data(), b.data() + 6);
    } else if (symbol == 'c') {
      left.push_back(expected.at<double>(key)); right.push_back(actual.at<double>(key));
    } else {
      if (reason) *reason = "UNSUPPORTED_KEY:" + gtsam::DefaultKeyFormatter(key);
      return false;
    }
    for (size_t index = 0; index < left.size(); ++index)
      if (DoubleBits(left[index]) != DoubleBits(right[index])) {
        if (reason) *reason = "BINARY64_MISMATCH:" +
                              gtsam::DefaultKeyFormatter(key) + ":" +
                              std::to_string(index);
        return false;
      }
  }
  if (reason) *reason = "EXACT_BINARY64_MATCH";
  return true;
}

DevelopmentStage2ExportResult WriteDevelopmentStage2Bundle(
    const std::string& stage2_directory,
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<RefitFactorMeta>& factor_metadata,
    const InferenceIdentityContext& identity_context) {
  DevelopmentStage2ExportResult result;
  try {
    const boost::filesystem::path root(stage2_directory);
    if (!boost::filesystem::is_directory(root))
      throw std::runtime_error("STAGE2_EXPORT_DIRECTORY_MISSING");
    const auto manifest_path = root / "stage2_export_manifest.json";
    if (boost::filesystem::exists(manifest_path))
      throw std::runtime_error("STAGE2_EXPORT_OVERWRITE_FORBIDDEN");
    if (!GraphAndValuesKeysMatch(graph, values))
      throw std::runtime_error("STAGE2_EXPORT_GRAPH_VALUES_KEY_MISMATCH");
    RequireFactorMetadata(graph, factor_metadata);
    size_t scalar_count = 0;
    const std::string values_bytes = SerializeStage2Values(values, &scalar_count);

    std::ostringstream factors;
    factors << "factor_index,obs_id,factor_type,segment_id,keys\n";
    for (const auto& meta : factor_metadata) {
      factors << meta.factor_index << ',' << meta.obs_id << ','
              << meta.factor_type << ',' << meta.segment_id << ',';
      for (size_t index = 0; index < meta.keys.size(); ++index)
        factors << (index ? ";" : "") << meta.keys[index] << ':'
                << gtsam::DefaultKeyFormatter(meta.keys[index]);
      factors << '\n';
    }

    const auto identity = ComputeInferenceContentIdentity(graph, values,
                                                           identity_context);
    std::ostringstream identity_bytes;
    identity_bytes << "{\n  \"schema\": \"A19_R06_STAGE2_CONTENT_IDENTITY_V1\","
                   << "\n  \"graph_linearization_sha256\": \""
                   << identity.graph_linearization_sha256 << "\","
                   << "\n  \"values_sha256\": \"" << identity.values_sha256
                   << "\",\n  \"context_sha256\": \"" << identity.context_sha256
                   << "\",\n  \"factor_count\": " << graph.size()
                   << ",\n  \"values_count\": " << values.size()
                   << ",\n  \"scalar_count\": " << scalar_count
                   << ",\n  \"consumable\": false\n}\n";

    const auto values_pending = root / "final_values.csv.pending";
    const auto factors_pending = root / "factor_metadata.csv.pending";
    const auto identity_pending = root / "content_identity.json.pending";
    AtomicWrite(values_pending, values_bytes);
    AtomicWrite(factors_pending, factors.str());
    AtomicWrite(identity_pending, identity_bytes.str());

    const gtsam::Values restored = ReadDevelopmentStage2Values(values_pending.string());
    std::string comparison_reason;
    if (!DevelopmentStage2ValuesBitEqual(values, restored, &comparison_reason))
      throw std::runtime_error("STAGE2_EXPORT_ROUNDTRIP_FAILED:" + comparison_reason);
    const auto restored_identity = ComputeInferenceContentIdentity(
        graph, restored, identity_context);
    if (restored_identity.graph_linearization_sha256 !=
            identity.graph_linearization_sha256 ||
        restored_identity.values_sha256 != identity.values_sha256 ||
        restored_identity.context_sha256 != identity.context_sha256)
      throw std::runtime_error("STAGE2_EXPORT_CONTENT_IDENTITY_ROUNDTRIP_MISMATCH");

    const auto publish = [&](const boost::filesystem::path& pending,
                             const boost::filesystem::path& final) {
      if (boost::filesystem::exists(final))
        throw std::runtime_error("STAGE2_EXPORT_OUTPUT_EXISTS:" + final.string());
      if (std::rename(pending.string().c_str(), final.string().c_str()) != 0)
        throw std::runtime_error("STAGE2_EXPORT_PUBLISH_RENAME_FAILED:" + final.string());
    };
    publish(values_pending, root / "final_values.csv");
    publish(factors_pending, root / "factor_metadata.csv");
    publish(identity_pending, root / "content_identity.json");
    std::ostringstream manifest;
    manifest << "{\n  \"schema\": \"A19_R06_STAGE2_EXPORT_MANIFEST_V1\","
             << "\n  \"solver_converged\": true,"
             << "\n  \"export_complete\": true,"
             << "\n  \"score_complete\": false,"
             << "\n  \"roundtrip\": \"EXACT_BINARY64_MATCH\","
             << "\n  \"values_count\": " << values.size()
             << ",\n  \"scalar_count\": " << scalar_count
             << ",\n  \"graph_linearization_sha256\": \""
             << identity.graph_linearization_sha256 << "\","
             << "\n  \"values_sha256\": \"" << identity.values_sha256 << "\","
             << "\n  \"final_values_sha256\": \"sha256:"
             << Sha256FileHex((root / "final_values.csv").string()) << "\","
             << "\n  \"factor_metadata_sha256\": \"sha256:"
             << Sha256FileHex((root / "factor_metadata.csv").string()) << "\","
             << "\n  \"content_identity_sha256\": \"sha256:"
             << Sha256FileHex((root / "content_identity.json").string()) << "\","
             << "\n  \"evaluation_label\": \"UNBLINDED_DEVELOPMENT\","
             << "\n  \"consumable\": false\n}\n";
    AtomicWrite(manifest_path, manifest.str());
    result.ok = true;
    result.status = "COMPLETE";
    result.reason = "EXACT_BINARY64_ROUNDTRIP_AND_CONTENT_IDENTITY_VERIFIED";
    result.values_count = values.size();
    result.scalar_count = scalar_count;
    result.graph_linearization_sha256 = identity.graph_linearization_sha256;
    result.values_sha256 = identity.values_sha256;
  } catch (const std::exception& error) {
    result.reason = error.what();
  }
  return result;
}

}  // namespace uifgo
