#include "uwb_imu_pl/config/fault_manifest.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

const char* kImplementedModels[] = {
    "epoch_independent_step", "persistent_constant_step", "affine_ramp",
    "interval_constant_axis",
};

const char* kKnownUnsupportedFamilies[] = {
    "two_uwb", "imu_imu", "same_device_multiaxis",
};

std::string readAll(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open fault manifest: " + path);
  std::ostringstream out;
  out << input.rdbuf();
  return out.str();
}

void requireMap(const YAML::Node& node, const std::string& path) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error(path + " must be a mapping");
  }
}

void rejectUnknown(const YAML::Node& node, const std::string& path,
                   const std::vector<std::string>& allowed) {
  if (!node || !node.IsMap()) return;
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::runtime_error("unknown key " + path + "." + key);
    }
  }
}

std::string requiredText(const YAML::Node& node, const std::string& key,
                         const std::string& path) {
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error("missing required key " + path + "." + key);
  }
  const std::string value = node[key].as<std::string>();
  if (value.empty()) {
    throw std::runtime_error("empty required key " + path + "." + key);
  }
  return value;
}

int requiredInt(const YAML::Node& node, const std::string& key,
                const std::string& path) {
  if (!node[key] || !node[key].IsScalar()) {
    throw std::runtime_error("missing required key " + path + "." + key);
  }
  return node[key].as<int>();
}

std::vector<std::string> requiredStringList(const YAML::Node& node,
                                            const std::string& key,
                                            const std::string& path) {
  if (!node[key] || !node[key].IsSequence()) {
    throw std::runtime_error("missing required list " + path + "." + key);
  }
  std::vector<std::string> out;
  for (const auto& item : node[key]) out.push_back(item.as<std::string>());
  return out;
}

bool listed(const std::vector<std::string>& values, const std::string& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

}  // namespace

bool FaultManifestLoader::modelTypeImplemented(const std::string& model_type) {
  for (const char* candidate : kImplementedModels) {
    if (model_type == candidate) return true;
  }
  return false;
}

FaultManifest FaultManifestLoader::load(const std::string& path) {
  const YAML::Node root = YAML::Load(readAll(path));
  requireMap(root, "fault_manifest");
  rejectUnknown(root, "fault_manifest",
                {"schema_version", "manifest_id", "protected_quantity",
                 "position_reference", "max_fault_order",
                 "enabled_single_families", "enabled_pair_families",
                 "unsupported_families", "events", "families"});
  FaultManifest manifest;
  manifest.schema_version = requiredInt(root, "schema_version", "fault_manifest");
  if (manifest.schema_version != 1) {
    throw std::runtime_error("fault manifest schema_version must be 1");
  }
  manifest.manifest_id = requiredText(root, "manifest_id", "fault_manifest");
  manifest.protected_quantity =
      requiredText(root, "protected_quantity", "fault_manifest");
  manifest.position_reference =
      requiredText(root, "position_reference", "fault_manifest");
  manifest.max_fault_order =
      requiredInt(root, "max_fault_order", "fault_manifest");
  if (manifest.max_fault_order != 1 && manifest.max_fault_order != 2) {
    throw std::runtime_error("fault manifest max_fault_order must be 1 or 2");
  }
  manifest.enabled_single_families =
      requiredStringList(root, "enabled_single_families", "fault_manifest");
  manifest.enabled_pair_families =
      requiredStringList(root, "enabled_pair_families", "fault_manifest");
  manifest.unsupported_families =
      requiredStringList(root, "unsupported_families", "fault_manifest");
  if (!root["events"] || !root["events"].IsSequence()) {
    throw std::runtime_error("fault_manifest.events must be a list");
  }
  for (const auto& node : root["events"]) {
    requireMap(node, "fault_manifest.events[]");
    rejectUnknown(node, "fault_manifest.events[]",
                  {"source_id", "common_cause_group", "event_id", "model_type",
                   "parameter_units", "parameter_sharing", "onset_domain",
                   "end_condition", "physical_time_support", "amplitude_model",
                   "prior_bound_source", "prior_time_basis", "evidence_status",
                   "included_hypothesis_families", "omitted_event_set",
                   "history_retention_or_reset_rule", "allowed_actions",
                   "projection_eligibility"});
    FaultManifestEvent event;
    event.source_id = requiredText(node, "source_id", "events[]");
    event.common_cause_group = requiredText(node, "common_cause_group", "events[]");
    event.event_id = requiredText(node, "event_id", "events[]");
    event.model_type = requiredText(node, "model_type", "events[]");
    event.parameter_units = requiredText(node, "parameter_units", "events[]");
    event.parameter_sharing = requiredText(node, "parameter_sharing", "events[]");
    event.onset_domain = requiredText(node, "onset_domain", "events[]");
    event.end_condition = requiredText(node, "end_condition", "events[]");
    event.physical_time_support =
        requiredText(node, "physical_time_support", "events[]");
    event.amplitude_model = requiredText(node, "amplitude_model", "events[]");
    event.prior_bound_source =
        requiredText(node, "prior_bound_source", "events[]");
    event.prior_time_basis = requiredText(node, "prior_time_basis", "events[]");
    event.evidence_status = requiredText(node, "evidence_status", "events[]");
    event.included_hypothesis_families =
        requiredStringList(node, "included_hypothesis_families", "events[]");
    event.omitted_event_set = requiredStringList(node, "omitted_event_set", "events[]");
    event.history_retention_or_reset_rule =
        requiredText(node, "history_retention_or_reset_rule", "events[]");
    event.allowed_actions = requiredStringList(node, "allowed_actions", "events[]");
    event.projection_eligibility =
        requiredText(node, "projection_eligibility", "events[]");
    manifest.events.push_back(std::move(event));
  }
  if (!root["families"] || !root["families"].IsSequence()) {
    throw std::runtime_error("fault_manifest.families must be a list");
  }
  for (const auto& node : root["families"]) {
    requireMap(node, "fault_manifest.families[]");
    rejectUnknown(node, "fault_manifest.families[]",
                  {"family_id", "scope", "members"});
    FaultManifestFamily family;
    family.family_id = requiredText(node, "family_id", "families[]");
    family.scope = requiredText(node, "scope", "families[]");
    family.members = requiredStringList(node, "members", "families[]");
    manifest.families.push_back(std::move(family));
  }
  validateAgainstCapabilities(manifest);
  manifest.resolved_yaml = serializeResolvedManifest(manifest);
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : manifest.resolved_yaml) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream digest;
  digest << std::hex << std::setfill('0') << std::setw(16) << hash;
  manifest.digest = digest.str();
  return manifest;
}

void FaultManifestLoader::validateAgainstCapabilities(
    const FaultManifest& manifest) {
  std::set<std::string> event_ids;
  for (const auto& event : manifest.events) {
    if (!event_ids.insert(event.event_id).second) {
      throw std::runtime_error("duplicate fault manifest event_id " + event.event_id);
    }
  }
  std::set<std::string> family_ids;
  for (const auto& family : manifest.families) {
    if (!family_ids.insert(family.family_id).second) {
      throw std::runtime_error("duplicate fault manifest family " + family.family_id);
    }
    if (family.scope != "single" && family.scope != "pair") {
      throw std::runtime_error("family " + family.family_id + " has invalid scope");
    }
    for (const auto& member : family.members) {
      if (!event_ids.count(member)) {
        throw std::runtime_error("family " + family.family_id +
                                 " references unknown event " + member);
      }
    }
  }
  const std::set<std::string> known_unsupported(
      std::begin(kKnownUnsupportedFamilies), std::end(kKnownUnsupportedFamilies));
  for (const auto& family : manifest.unsupported_families) {
    if (!known_unsupported.count(family)) {
      throw std::runtime_error("unsupported_families entry " + family +
                               " is not a recognized generator limitation");
    }
    if (listed(manifest.enabled_pair_families, family) ||
        listed(manifest.enabled_single_families, family)) {
      throw std::runtime_error("enabled family " + family +
                               " is simultaneously declared unsupported");
    }
  }
  for (const auto& family : manifest.enabled_single_families) {
    if (!family_ids.count(family)) {
      throw std::runtime_error("enabled single family " + family + " is not declared");
    }
  }
  for (const auto& family : manifest.enabled_pair_families) {
    if (!family_ids.count(family)) {
      throw std::runtime_error("enabled pair family " + family + " is not declared");
    }
  }
  if (manifest.max_fault_order == 2 && manifest.enabled_pair_families.empty()) {
    throw std::runtime_error(
        "max_fault_order=2 requires at least one declared pair family");
  }
  // Structural rule: the numerical projection eligibility is a function of the
  // model type only; action labels must never select the kernel.
  for (const auto& event : manifest.events) {
    const bool implemented = modelTypeImplemented(event.model_type);
    const std::string expected = implemented ? "algebraic" : "unsupported";
    if (event.projection_eligibility != expected) {
      throw std::runtime_error(
          "event " + event.event_id + ": projection_eligibility must be '" +
          expected + "' for model_type '" + event.model_type +
          "' (action labels must not change numerical eligibility)");
    }
    if (event.amplitude_model != "unbounded_subspace" &&
        event.amplitude_model != "validated_bounded_set") {
      throw std::runtime_error("event " + event.event_id +
                               " has unknown amplitude_model");
    }
    if (event.evidence_status != "IMPLEMENTED_UNVERIFIED" &&
        event.evidence_status != "NOT_IMPLEMENTED") {
      throw std::runtime_error(
          "event " + event.event_id +
          " has unknown evidence_status; use IMPLEMENTED_UNVERIFIED or "
          "NOT_IMPLEMENTED");
    }
    if (event.amplitude_model == "validated_bounded_set" &&
        event.evidence_status != "NOT_IMPLEMENTED") {
      throw std::runtime_error(
          "event " + event.event_id +
          " uses validated_bounded_set, which is not implemented; declare "
          "evidence_status: NOT_IMPLEMENTED instead of claiming support");
    }
  }
  // Enabled families may only contain implemented model types.
  std::set<std::string> enabled;
  enabled.insert(manifest.enabled_single_families.begin(),
                 manifest.enabled_single_families.end());
  enabled.insert(manifest.enabled_pair_families.begin(),
                 manifest.enabled_pair_families.end());
  for (const auto& family : manifest.families) {
    if (!enabled.count(family.family_id)) continue;
    for (const auto& member : family.members) {
      for (const auto& event : manifest.events) {
        if (event.event_id != member) continue;
        if (!modelTypeImplemented(event.model_type)) {
          throw std::runtime_error(
              "enabled family " + family.family_id + " contains event " + member +
              " with unsupported model_type " + event.model_type);
        }
      }
    }
  }
}

std::string FaultManifestLoader::serializeResolvedManifest(
    const FaultManifest& manifest) {
  YAML::Node root;
  root["schema_version"] = manifest.schema_version;
  root["manifest_id"] = manifest.manifest_id;
  root["protected_quantity"] = manifest.protected_quantity;
  root["position_reference"] = manifest.position_reference;
  root["max_fault_order"] = manifest.max_fault_order;
  for (const auto& family : manifest.enabled_single_families) {
    root["enabled_single_families"].push_back(family);
  }
  for (const auto& family : manifest.enabled_pair_families) {
    root["enabled_pair_families"].push_back(family);
  }
  for (const auto& family : manifest.unsupported_families) {
    root["unsupported_families"].push_back(family);
  }
  for (const auto& event : manifest.events) {
    YAML::Node node;
    node["source_id"] = event.source_id;
    node["common_cause_group"] = event.common_cause_group;
    node["event_id"] = event.event_id;
    node["model_type"] = event.model_type;
    node["parameter_units"] = event.parameter_units;
    node["parameter_sharing"] = event.parameter_sharing;
    node["onset_domain"] = event.onset_domain;
    node["end_condition"] = event.end_condition;
    node["physical_time_support"] = event.physical_time_support;
    node["amplitude_model"] = event.amplitude_model;
    node["prior_bound_source"] = event.prior_bound_source;
    node["prior_time_basis"] = event.prior_time_basis;
    node["evidence_status"] = event.evidence_status;
    for (const auto& family : event.included_hypothesis_families) {
      node["included_hypothesis_families"].push_back(family);
    }
    for (const auto& omitted : event.omitted_event_set) {
      node["omitted_event_set"].push_back(omitted);
    }
    node["history_retention_or_reset_rule"] = event.history_retention_or_reset_rule;
    for (const auto& action : event.allowed_actions) {
      node["allowed_actions"].push_back(action);
    }
    node["projection_eligibility"] = event.projection_eligibility;
    root["events"].push_back(node);
  }
  for (const auto& family : manifest.families) {
    YAML::Node node;
    node["family_id"] = family.family_id;
    node["scope"] = family.scope;
    for (const auto& member : family.members) node["members"].push_back(member);
    root["families"].push_back(node);
  }
  std::ostringstream out;
  out << root;
  return out.str();
}

}  // namespace uwb_imu_pl
