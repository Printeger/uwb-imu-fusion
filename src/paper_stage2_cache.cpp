#include "uifgo/paper_stage2_cache.h"

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "uifgo/hash_utils.h"

namespace uifgo {
namespace {

constexpr const char* kFixedLabel =
    "RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY";

void Field(std::ostringstream* bytes, const std::string& value) {
  *bytes << value.size() << ':' << value << '\n';
}

bool SafePayloadName(const std::string& name) {
  return !name.empty() && name != "." && name != ".." &&
         name.find('/') == std::string::npos &&
         name.find('\\') == std::string::npos;
}

std::string Json(const std::string& value) {
  std::ostringstream out;
  for (unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          const char hex[] = "0123456789abcdef";
          out << "\\u00" << hex[c >> 4] << hex[c & 15];
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  return out.str();
}

std::string Required(const YAML::Node& node, const char* key) {
  if (!node[key] || !node[key].IsScalar())
    throw std::runtime_error(std::string("cache manifest missing ") + key);
  return node[key].as<std::string>();
}

}  // namespace

const char* Stage2CacheNamespaceName(Stage2CacheNamespace value) {
  return value == Stage2CacheNamespace::AUTO_DISCOVERY
             ? "AUTO_DISCOVERY"
             : "FIXED_PARTITION_DEBUG";
}

void RequireStage2CommonPreparation(const Stage2CacheManifest& manifest,
                                    const std::string& requested_common_id) {
  if (requested_common_id.empty() || manifest.common_preparation_id != requested_common_id)
    throw std::runtime_error("CACHE_COMMON_PREPARATION_ID_MISMATCH");
}

std::string ComputeStage2CacheId(const Stage2CacheManifest& manifest) {
  std::ostringstream bytes;
  bytes << "uifgo-t09-stage2-cache-v2\n";
  Field(&bytes, Stage2CacheNamespaceName(manifest.cache_namespace));
  Field(&bytes, manifest.debug_label);
  Field(&bytes, manifest.common_preparation_id);
  Field(&bytes, manifest.source_identity);
  Field(&bytes, manifest.producer_common_config_sha256);
  Field(&bytes, manifest.producer_stage2_config_sha256);
  Field(&bytes, manifest.stage1_config_sha256);
  Field(&bytes, manifest.stage2_refit_config_sha256);
  Field(&bytes, manifest.stage3_score_config_sha256);
  Field(&bytes, manifest.support_partition_sha256);
  Field(&bytes, manifest.observation_mapping_sha256);
  Field(&bytes, manifest.stage2_graph_linearization_sha256);
  Field(&bytes, manifest.stage2_values_sha256);
  Field(&bytes, manifest.factor_metadata_sha256);
  Field(&bytes, manifest.stage2_trace_sha256);
  Field(&bytes, manifest.score_table_sha256);
  Field(&bytes, manifest.producer_commit);
  Field(&bytes, manifest.producer_binary_sha256);
  Field(&bytes, manifest.producer_abi_sha256);
  Field(&bytes, manifest.producer_toolchain_sha256);
  Field(&bytes, manifest.stage2_status);
  Field(&bytes, manifest.score_status);
  for (const auto& payload : manifest.payloads) {
    Field(&bytes, payload.name);
    Field(&bytes, payload.sha256);
  }
  return "t09stage2cache-sha256:" + Sha256Hex(bytes.str());
}

bool ValidateStage2CacheManifest(const Stage2CacheManifest& manifest,
                                 std::string* reason) {
  auto fail = [&](const std::string& value) {
    if (reason) *reason = value;
    return false;
  };
  if (manifest.schema != "uifgo_t09_stage2_cache_v2")
    return fail("CACHE_SCHEMA_MISMATCH");
  if (manifest.cache_namespace == Stage2CacheNamespace::FIXED_PARTITION_DEBUG) {
    if (manifest.debug_label != kFixedLabel)
      return fail("FIXED_CACHE_DEBUG_LABEL_MISMATCH");
  } else if (!manifest.debug_label.empty() &&
             manifest.debug_label.find("FIXED_PARTITION") !=
                 std::string::npos) {
    return fail("AUTO_CACHE_HAS_FIXED_PARTITION_LABEL");
  }
  const std::string* required[] = {
      &manifest.common_preparation_id, &manifest.source_identity,
      &manifest.producer_common_config_sha256,
      &manifest.producer_stage2_config_sha256,
      &manifest.stage1_config_sha256,
      &manifest.stage2_refit_config_sha256,
      &manifest.stage3_score_config_sha256,
      &manifest.support_partition_sha256,
      &manifest.observation_mapping_sha256,
      &manifest.stage2_graph_linearization_sha256,
      &manifest.stage2_values_sha256, &manifest.factor_metadata_sha256,
      &manifest.stage2_trace_sha256, &manifest.score_table_sha256,
      &manifest.producer_commit, &manifest.producer_binary_sha256,
      &manifest.producer_abi_sha256, &manifest.producer_toolchain_sha256,
      &manifest.stage2_status, &manifest.score_status};
  for (const auto* value : required)
    if (value->empty()) return fail("CACHE_REQUIRED_IDENTITY_EMPTY");
  if (manifest.producer_common_config_sha256.rfind(
          "t09commonconfig-sha256:", 0) != 0 ||
      manifest.producer_stage2_config_sha256.rfind(
          "t09stage2config-sha256:", 0) != 0 ||
      manifest.stage1_config_sha256.rfind("sha256:", 0) != 0 ||
      manifest.stage2_refit_config_sha256.rfind("sha256:", 0) != 0 ||
      manifest.stage3_score_config_sha256.rfind("sha256:", 0) != 0)
    return fail("CACHE_PRODUCER_CONFIG_IDENTITY_INVALID");
  if (manifest.stage2_status != "CONVERGED")
    return fail("CACHE_STAGE2_NOT_CONVERGED");
  if (manifest.score_status != "COMPLETE" &&
      manifest.score_status != "COMPLETE_WITH_SCORE_UNAVAILABLE")
    return fail("CACHE_SCORE_TABLE_INCOMPLETE");
  if (manifest.payloads.empty()) return fail("CACHE_PAYLOADS_EMPTY");
  for (const auto& payload : manifest.payloads)
    if (!SafePayloadName(payload.name) ||
        payload.sha256.rfind("sha256:", 0) != 0)
      return fail("CACHE_PAYLOAD_IDENTITY_INVALID");
  const std::string actual = ComputeStage2CacheId(manifest);
  if (!manifest.cache_id.empty() && manifest.cache_id != actual)
    return fail("CACHE_ID_MISMATCH");
  if (reason) reason->clear();
  return true;
}

void PublishStage2CacheManifest(const std::string& cache_directory,
                                Stage2CacheManifest manifest) {
  namespace fs = boost::filesystem;
  manifest.cache_id = ComputeStage2CacheId(manifest);
  std::string reason;
  if (!ValidateStage2CacheManifest(manifest, &reason))
    throw std::invalid_argument(reason);
  const fs::path directory(cache_directory);
  if (!fs::exists(directory)) fs::create_directories(directory);
  if (!fs::is_directory(directory))
    throw std::runtime_error("cache target is not a directory");
  for (const auto& payload : manifest.payloads) {
    const fs::path path = directory / payload.name;
    if (!fs::is_regular_file(path))
      throw std::runtime_error("cache payload is missing: " + payload.name);
    const std::string actual = "sha256:" + Sha256FileHex(path.string());
    if (actual != payload.sha256)
      throw std::runtime_error("cache payload hash mismatch: " + payload.name);
  }
  const fs::path final = directory / "stage2_cache_manifest.json";
  if (fs::exists(final))
    throw std::runtime_error("refusing to overwrite Stage-2 cache manifest");
  const fs::path staging = directory / "stage2_cache_manifest.json.staging";
  if (fs::exists(staging)) fs::remove(staging);
  {
    std::ofstream out(staging.string());
    if (!out) throw std::runtime_error("cannot create Stage-2 cache manifest");
    out << "{\n  \"schema\": \"" << manifest.schema
        << "\",\n  \"cache_namespace\": \""
        << Stage2CacheNamespaceName(manifest.cache_namespace)
        << "\",\n  \"debug_label\": \"" << Json(manifest.debug_label)
        << "\",\n  \"common_preparation_id\": \""
        << Json(manifest.common_preparation_id)
        << "\",\n  \"source_identity\": \""
        << Json(manifest.source_identity)
        << "\",\n  \"producer_common_config_sha256\": \""
        << Json(manifest.producer_common_config_sha256)
        << "\",\n  \"producer_stage2_config_sha256\": \""
        << Json(manifest.producer_stage2_config_sha256)
        << "\",\n  \"stage1_config_sha256\": \""
        << Json(manifest.stage1_config_sha256)
        << "\",\n  \"stage2_refit_config_sha256\": \""
        << Json(manifest.stage2_refit_config_sha256)
        << "\",\n  \"stage3_score_config_sha256\": \""
        << Json(manifest.stage3_score_config_sha256)
        << "\",\n  \"support_partition_sha256\": \""
        << Json(manifest.support_partition_sha256)
        << "\",\n  \"observation_mapping_sha256\": \""
        << Json(manifest.observation_mapping_sha256)
        << "\",\n  \"stage2_graph_linearization_sha256\": \""
        << Json(manifest.stage2_graph_linearization_sha256)
        << "\",\n  \"stage2_values_sha256\": \""
        << Json(manifest.stage2_values_sha256)
        << "\",\n  \"factor_metadata_sha256\": \""
        << Json(manifest.factor_metadata_sha256)
        << "\",\n  \"stage2_trace_sha256\": \""
        << Json(manifest.stage2_trace_sha256)
        << "\",\n  \"score_table_sha256\": \""
        << Json(manifest.score_table_sha256)
        << "\",\n  \"producer_commit\": \""
        << Json(manifest.producer_commit)
        << "\",\n  \"producer_binary_sha256\": \""
        << Json(manifest.producer_binary_sha256)
        << "\",\n  \"producer_abi_sha256\": \""
        << Json(manifest.producer_abi_sha256)
        << "\",\n  \"producer_toolchain_sha256\": \""
        << Json(manifest.producer_toolchain_sha256)
        << "\",\n  \"stage2_status\": \""
        << Json(manifest.stage2_status)
        << "\",\n  \"score_status\": \""
        << Json(manifest.score_status)
        << "\",\n  \"cache_id\": \"" << manifest.cache_id
        << "\",\n  \"payloads\": [\n";
    for (size_t i = 0; i < manifest.payloads.size(); ++i) {
      if (i) out << ",\n";
      out << "    {\"name\": \"" << Json(manifest.payloads[i].name)
          << "\", \"sha256\": \"" << manifest.payloads[i].sha256
          << "\"}";
    }
    out << "\n  ]\n}\n";
  }
  fs::rename(staging, final);
}

Stage2CacheManifest ReadStage2CacheManifest(
    const std::string& manifest_path) {
  const YAML::Node node = YAML::LoadFile(manifest_path);
  Stage2CacheManifest manifest;
  manifest.schema = Required(node, "schema");
  const std::string cache_namespace = Required(node, "cache_namespace");
  if (cache_namespace == "AUTO_DISCOVERY")
    manifest.cache_namespace = Stage2CacheNamespace::AUTO_DISCOVERY;
  else if (cache_namespace == "FIXED_PARTITION_DEBUG")
    manifest.cache_namespace = Stage2CacheNamespace::FIXED_PARTITION_DEBUG;
  else
    throw std::runtime_error("unknown Stage-2 cache namespace");
  manifest.debug_label = Required(node, "debug_label");
  manifest.common_preparation_id = Required(node, "common_preparation_id");
  manifest.source_identity = Required(node, "source_identity");
  manifest.producer_common_config_sha256 =
      Required(node, "producer_common_config_sha256");
  manifest.producer_stage2_config_sha256 =
      Required(node, "producer_stage2_config_sha256");
  manifest.stage1_config_sha256 = Required(node, "stage1_config_sha256");
  manifest.stage2_refit_config_sha256 =
      Required(node, "stage2_refit_config_sha256");
  manifest.stage3_score_config_sha256 =
      Required(node, "stage3_score_config_sha256");
  manifest.support_partition_sha256 =
      Required(node, "support_partition_sha256");
  manifest.observation_mapping_sha256 =
      Required(node, "observation_mapping_sha256");
  manifest.stage2_graph_linearization_sha256 =
      Required(node, "stage2_graph_linearization_sha256");
  manifest.stage2_values_sha256 = Required(node, "stage2_values_sha256");
  manifest.factor_metadata_sha256 = Required(node, "factor_metadata_sha256");
  manifest.stage2_trace_sha256 = Required(node, "stage2_trace_sha256");
  manifest.score_table_sha256 = Required(node, "score_table_sha256");
  manifest.producer_commit = Required(node, "producer_commit");
  manifest.producer_binary_sha256 =
      Required(node, "producer_binary_sha256");
  manifest.producer_abi_sha256 = Required(node, "producer_abi_sha256");
  manifest.producer_toolchain_sha256 =
      Required(node, "producer_toolchain_sha256");
  manifest.stage2_status = Required(node, "stage2_status");
  manifest.score_status = Required(node, "score_status");
  manifest.cache_id = Required(node, "cache_id");
  if (!node["payloads"] || !node["payloads"].IsSequence())
    throw std::runtime_error("cache manifest payloads are missing");
  for (const auto& item : node["payloads"])
    manifest.payloads.push_back(
        {Required(item, "name"), Required(item, "sha256")});
  std::string reason;
  if (!ValidateStage2CacheManifest(manifest, &reason))
    throw std::runtime_error(reason);
  const boost::filesystem::path root =
      boost::filesystem::path(manifest_path).parent_path();
  for (const auto& payload : manifest.payloads) {
    const auto path = root / payload.name;
    if (!boost::filesystem::is_regular_file(path) ||
        "sha256:" + Sha256FileHex(path.string()) != payload.sha256)
      throw std::runtime_error("cache payload verification failed: " +
                               payload.name);
  }
  return manifest;
}

}  // namespace uifgo
