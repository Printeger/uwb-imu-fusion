#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "uifgo/hash_utils.h"
#include "uifgo/pl_bidirectional_support.h"

namespace fs = boost::filesystem;
namespace {

constexpr const char* kSourceHead =
    "63b0062664c7fb4af372da1609128e8aff2b646a";

struct Args {
  enum class Action { NONE, CALIBRATE, DETECT, INTERSECT } action = Action::NONE;
  fs::path conditional_csv;
  fs::path split_manifest;
  fs::path backward_calibration;
  fs::path forward_trace;
  fs::path forward_status;
  fs::path backward_trace;
  fs::path backward_status;
  fs::path output_dir;
  std::string partition = "all";
};

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated CSV quote");
  fields.push_back(field);
  return fields;
}

std::string Escape(const std::string& value) {
  std::ostringstream out;
  for (const char c : value) {
    if (c == '\\' || c == '"') out << '\\';
    if (c == '\n') out << "\\n";
    else if (c != '\r') out << c;
  }
  return out.str();
}

std::ofstream Open(const fs::path& path) {
  std::ofstream out(path.string(), std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << std::setprecision(17);
  return out;
}

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&]() {
      if (++i >= argc) throw std::invalid_argument("missing CLI value");
      return std::string(argv[i]);
    };
    if (arg == "--conditional-csv") args.conditional_csv = take();
    else if (arg == "--clean-split-manifest") args.split_manifest = take();
    else if (arg == "--backward-calibration") args.backward_calibration = take();
    else if (arg == "--forward-trace") args.forward_trace = take();
    else if (arg == "--forward-status") args.forward_status = take();
    else if (arg == "--backward-trace") args.backward_trace = take();
    else if (arg == "--backward-status") args.backward_status = take();
    else if (arg == "--output-dir") args.output_dir = take();
    else if (arg == "--partition") args.partition = take();
    else if (arg == "--calibrate-backward") args.action = Args::Action::CALIBRATE;
    else if (arg == "--backward-detect") args.action = Args::Action::DETECT;
    else if (arg == "--intersect") args.action = Args::Action::INTERSECT;
    else if (arg == "--help") {
      std::cout
          << "Usage:\n"
          << "  pl_bidirectional_cusum_preflight --calibrate-backward --conditional-csv FILE --clean-split-manifest FILE --output-dir DIR\n"
          << "  pl_bidirectional_cusum_preflight --backward-detect --conditional-csv FILE --backward-calibration FILE --partition all|validation --output-dir DIR\n"
          << "  pl_bidirectional_cusum_preflight --intersect --forward-trace FILE --forward-status FILE --backward-trace FILE --backward-status FILE --output-dir DIR\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (args.action == Args::Action::NONE || args.output_dir.empty())
    throw std::invalid_argument("action and --output-dir required");
  if (args.action == Args::Action::CALIBRATE &&
      (args.conditional_csv.empty() || args.split_manifest.empty()))
    throw std::invalid_argument("backward calibration inputs missing");
  if (args.action == Args::Action::DETECT &&
      (args.conditional_csv.empty() || args.backward_calibration.empty()))
    throw std::invalid_argument("backward detection inputs missing");
  if (args.action == Args::Action::INTERSECT &&
      (args.forward_trace.empty() || args.forward_status.empty() ||
       args.backward_trace.empty() || args.backward_status.empty()))
    throw std::invalid_argument("intersection inputs missing");
  if (args.partition != "all" && args.partition != "validation")
    throw std::invalid_argument("partition must be all or validation");
  return args;
}

std::string ReadAll(const fs::path& path) {
  std::ifstream in(path.string(), std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path.string());
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

double JsonNumber(const std::string& json, const std::string& field) {
  const std::string token = "\"" + field + "\"";
  size_t position = json.find(token);
  if (position == std::string::npos) throw std::runtime_error("missing JSON field " + field);
  position = json.find(':', position + token.size());
  if (position == std::string::npos) throw std::runtime_error("malformed JSON field " + field);
  return std::stod(json.substr(position + 1));
}

std::string JsonString(const std::string& json, const std::string& field) {
  const std::string token = "\"" + field + "\"";
  size_t position = json.find(token);
  if (position == std::string::npos) throw std::runtime_error("missing JSON field " + field);
  position = json.find(':', position + token.size());
  position = json.find('"', position + 1);
  if (position == std::string::npos) throw std::runtime_error("malformed JSON string " + field);
  const size_t end = json.find('"', position + 1);
  if (end == std::string::npos) throw std::runtime_error("malformed JSON string " + field);
  return json.substr(position + 1, end - position - 1);
}

void VerifySiblingSeal(const fs::path& path) {
  const fs::path seal(path.string() + ".sha256");
  std::ifstream in(seal.string());
  std::string expected;
  if (!in || !(in >> expected))
    throw std::runtime_error("missing seal " + seal.string());
  if (expected != uifgo::Sha256FileHex(path.string()))
    throw std::runtime_error("seal mismatch " + path.string());
}

std::vector<uifgo::PlCusumInputRow> ReadConditionalRows(const fs::path& path) {
  std::ifstream in(path.string(), std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path.string());
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty conditional CSV");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const auto header = SplitCsv(line);
  std::map<std::string, size_t> column;
  for (size_t i = 0; i < header.size(); ++i) column[header[i]] = i;
  const std::vector<std::string> required{
      "timestamp", "group_id", "keyframe_id", "source_order", "tag_id",
      "anchor_id", "obs_id", "conditional_z", "diagnostic_valid"};
  for (const auto& name : required)
    if (!column.count(name)) throw std::runtime_error("missing CSV column " + name);
  std::vector<uifgo::PlCusumInputRow> rows;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto fields = SplitCsv(line);
    auto get = [&](const std::string& name) -> const std::string& {
      const size_t index = column.at(name);
      if (index >= fields.size()) throw std::runtime_error("short CSV row");
      return fields[index];
    };
    uifgo::PlCusumInputRow row;
    row.timestamp = std::stod(get("timestamp"));
    row.group_id = get("group_id");
    row.keyframe_id = std::stoull(get("keyframe_id"));
    row.source_order = std::stoull(get("source_order"));
    row.tag_id = std::stoi(get("tag_id"));
    row.anchor_id = std::stoi(get("anchor_id"));
    row.obs_id = std::stoull(get("obs_id"));
    const auto& z = get("conditional_z");
    row.conditional_z = z.empty() ? std::numeric_limits<double>::quiet_NaN()
                                  : std::stod(z);
    const auto& valid = get("diagnostic_valid");
    row.diagnostic_valid = valid == "1" || valid == "true" || valid == "TRUE";
    rows.push_back(std::move(row));
  }
  return rows;
}

std::vector<uifgo::PlCusumInputRow> Select(
    const std::vector<uifgo::PlCusumInputRow>& rows,
    double lower, double upper) {
  std::vector<uifgo::PlCusumInputRow> selected;
  for (const auto& row : rows)
    if (row.timestamp >= lower && row.timestamp <= upper) selected.push_back(row);
  return selected;
}

void WriteCalibration(const Args& args,
                      const std::vector<uifgo::PlCusumInputRow>& rows) {
  VerifySiblingSeal(args.split_manifest);
  const std::string split = ReadAll(args.split_manifest);
  const double start = JsonNumber(split, "calibration_start");
  const double end = JsonNumber(split, "calibration_end");
  const auto selected = Select(rows, start, end);
  uifgo::PlBackwardCusumOptions options;
  options.threshold = std::numeric_limits<double>::max();
  options.original_clean_split_manifest_hash =
      "sha256:" + uifgo::Sha256FileHex(args.split_manifest.string());
  options.original_clean_input_hash =
      "sha256:" + uifgo::Sha256FileHex(args.conditional_csv.string());
  const auto result = uifgo::EvaluatePlBackwardCusum(selected, options);
  if (!result.valid) throw std::runtime_error("backward calibration failed: " + result.status);
  const double threshold = std::max(5.0, result.max_b + 1.0);
  std::set<uifgo::PlCusumLinkKey> links;
  for (const auto& row : selected) links.insert({row.tag_id, row.anchor_id});
  auto out = Open(args.output_dir / "backward_cusum_calibration.json");
  out << "{\n  \"schema\": \"pl_backward_cusum_calibration_v1\",\n"
      << "  \"algorithm_version\": \"" << uifgo::kPlBackwardCusumAlgorithmVersion << "\",\n"
      << "  \"kappa_backward\": 0.5,\n"
      << "  \"B_calibration_max\": " << result.max_b << ",\n"
      << "  \"h_backward\": " << threshold << ",\n"
      << "  \"threshold_rule\": \"max(5.0,B_calibration_max+1.0)\",\n"
      << "  \"original_clean_split_manifest_hash\": \""
      << options.original_clean_split_manifest_hash << "\",\n"
      << "  \"original_clean_input_hash\": \""
      << options.original_clean_input_hash << "\",\n"
      << "  \"calibration_start\": " << start << ",\n"
      << "  \"calibration_end\": " << end << ",\n"
      << "  \"validation_start\": " << JsonNumber(split, "validation_start") << ",\n"
      << "  \"validation_end\": " << JsonNumber(split, "validation_end") << ",\n"
      << "  \"row_count\": " << selected.size() << ",\n"
      << "  \"link_count\": " << links.size() << ",\n"
      << "  \"invalid_count\": " << result.invalid_row_count << ",\n"
      << "  \"source_head\": \"" << kSourceHead << "\",\n"
      << "  \"truth_access_count\": 0\n}\n";
}

void WriteSupportJson(const fs::path& path,
                      const uifgo::SupportPartition& support,
                      const std::string& identity,
                      const std::string& forward = {},
                      const std::string& backward = {}) {
  auto out = Open(path);
  out << "{\n  \"schema\": \"" << support.schema << "\",\n"
      << "  \"provider\": \"" << support.provider << "\",\n"
      << "  \"identity\": \"" << identity << "\",\n";
  if (!forward.empty())
    out << "  \"forward_detector_identity\": \"" << forward << "\",\n"
        << "  \"backward_closure_identity\": \"" << backward << "\",\n"
        << "  \"final_intersection_rule\": \""
        << uifgo::kPlBidirectionalIntersectionRule << "\",\n";
  out << "  \"partition_hash\": \"" << support.partition_hash << "\",\n"
      << "  \"segments\": [";
  for (size_t i = 0; i < support.segments.size(); ++i) {
    const auto& segment = support.segments[i];
    out << (i ? ",\n" : "\n") << "    {\"segment_id\": \""
        << Escape(segment.segment_id) << "\", \"segment_ordinal\": "
        << segment.segment_ordinal << ", \"tag_id\": " << segment.tag_id
        << ", \"anchor_id\": " << segment.anchor_id
        << ", \"start_time\": " << segment.start_time
        << ", \"end_time\": " << segment.end_time
        << ", \"observation_count\": " << segment.observation_count
        << ", \"obs_ids\": [";
    for (size_t j = 0; j < segment.obs_ids.size(); ++j)
      out << (j ? "," : "") << segment.obs_ids[j];
    out << "]}";
  }
  out << "\n  ]\n}\n";
}

void WriteBackwardDetection(const Args& args,
                            const std::vector<uifgo::PlCusumInputRow>& all_rows) {
  VerifySiblingSeal(args.backward_calibration);
  const std::string calibration = ReadAll(args.backward_calibration);
  std::vector<uifgo::PlCusumInputRow> rows = all_rows;
  if (args.partition == "validation")
    rows = Select(all_rows, JsonNumber(calibration, "validation_start"),
                  JsonNumber(calibration, "validation_end"));
  uifgo::PlBackwardCusumOptions options;
  options.kappa = JsonNumber(calibration, "kappa_backward");
  options.threshold = JsonNumber(calibration, "h_backward");
  options.original_clean_split_manifest_hash =
      JsonString(calibration, "original_clean_split_manifest_hash");
  options.original_clean_input_hash =
      JsonString(calibration, "original_clean_input_hash");
  options.calibration_hash =
      "sha256:" + uifgo::Sha256FileHex(args.backward_calibration.string());
  const auto result = uifgo::EvaluatePlBackwardCusum(rows, options);
  if (!result.valid) throw std::runtime_error("backward detector failed: " + result.status);

  auto trace = Open(args.output_dir / "backward_cusum_trace.csv");
  trace << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,conditional_z,diagnostic_valid,reverse_index,kappa_backward,h_backward,increment,B_before,B_after,reset,reset_reason,reverse_excursion_id,reverse_excursion_start_time,threshold_crossing,first_crossing_for_excursion,backward_candidate\n";
  for (const auto& row : result.trace) {
    trace << row.input.timestamp << ',' << row.input.group_id << ','
          << row.input.keyframe_id << ',' << row.input.source_order << ','
          << row.input.tag_id << ',' << row.input.anchor_id << ','
          << row.input.obs_id << ',';
    if (std::isfinite(row.input.conditional_z)) trace << row.input.conditional_z;
    trace << ',' << row.input.diagnostic_valid << ',' << row.reverse_index << ','
          << options.kappa << ',' << options.threshold << ',';
    if (std::isfinite(row.increment)) trace << row.increment;
    trace << ',' << row.b_before << ',' << row.b_after << ',' << row.reset
          << ',' << row.reset_reason << ',' << row.reverse_excursion_id << ',';
    if (std::isfinite(row.reverse_excursion_start_time))
      trace << row.reverse_excursion_start_time;
    trace << ',' << row.threshold_crossing << ','
          << row.first_crossing_for_excursion << ','
          << row.backward_candidate << '\n';
  }
  auto candidates = Open(args.output_dir / "backward_cusum_candidates.csv");
  candidates << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,reverse_excursion_id,backward_candidate\n";
  for (const auto& row : result.trace)
    if (row.backward_candidate)
      candidates << row.input.timestamp << ',' << row.input.group_id << ','
                 << row.input.keyframe_id << ',' << row.input.source_order << ','
                 << row.input.tag_id << ',' << row.input.anchor_id << ','
                 << row.input.obs_id << ',' << row.reverse_excursion_id << ",1\n";
  WriteSupportJson(args.output_dir / "backward_cusum_support.json",
                   result.support, result.closure_identity);
  auto status = Open(args.output_dir / "backward_status.json");
  status << "{\n  \"schema\": \"pl_backward_cusum_status_v1\",\n"
         << "  \"closure_identity\": \"" << result.closure_identity << "\",\n"
         << "  \"processing_direction\": \"descending_physical_time\",\n"
         << "  \"kappa_backward\": " << options.kappa << ",\n"
         << "  \"h_backward\": " << options.threshold << ",\n"
         << "  \"calibration_hash\": \"" << options.calibration_hash << "\",\n"
         << "  \"input_hash\": \"sha256:"
         << uifgo::Sha256FileHex(args.conditional_csv.string()) << "\",\n"
         << "  \"row_count\": " << result.trace.size() << ",\n"
         << "  \"valid_row_count\": " << result.valid_row_count << ",\n"
         << "  \"invalid_row_count\": " << result.invalid_row_count << ",\n"
         << "  \"link_count\": " << result.per_link_max_b.size() << ",\n"
         << "  \"alarm_count\": " << result.alarm_count << ",\n"
         << "  \"alarm_link_count\": " << result.alarm_link_count << ",\n"
         << "  \"segment_count\": " << result.segment_count << ",\n"
         << "  \"max_B\": " << result.max_b << ",\n"
         << "  \"per_link_max_B\": [";
  size_t index = 0;
  for (const auto& item : result.per_link_max_b)
    status << (index++ ? ",\n" : "\n") << "    {\"tag_id\": "
           << item.first.tag_id << ", \"anchor_id\": "
           << item.first.anchor_id << ", \"max_B\": " << item.second << "}";
  status << "\n  ],\n  \"group_statistic_used_for_decision\": false,\n"
         << "  \"standalone_detection_created\": false,\n"
         << "  \"truth_access_count\": 0,\n"
         << "  \"production\": false\n}\n";
}

std::vector<uifgo::PlBidirectionalMembershipRow> ReadMembership(
    const fs::path& path, bool forward) {
  std::ifstream in(path.string(), std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path.string());
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty membership trace");
  const auto header = SplitCsv(line);
  std::map<std::string, size_t> column;
  for (size_t i = 0; i < header.size(); ++i) column[header[i]] = i;
  const std::string candidate_name = forward ? "candidate" : "backward_candidate";
  const std::vector<std::string> required{
      "timestamp", "group_id", "keyframe_id", "source_order", "tag_id",
      "anchor_id", "obs_id", "conditional_z", "diagnostic_valid", candidate_name};
  for (const auto& name : required)
    if (!column.count(name)) throw std::runtime_error("missing trace column " + name);
  std::vector<uifgo::PlBidirectionalMembershipRow> rows;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto fields = SplitCsv(line);
    auto get = [&](const std::string& name) -> const std::string& {
      return fields.at(column.at(name));
    };
    uifgo::PlBidirectionalMembershipRow output;
    output.input.timestamp = std::stod(get("timestamp"));
    output.input.group_id = get("group_id");
    output.input.keyframe_id = std::stoull(get("keyframe_id"));
    output.input.source_order = std::stoull(get("source_order"));
    output.input.tag_id = std::stoi(get("tag_id"));
    output.input.anchor_id = std::stoi(get("anchor_id"));
    output.input.obs_id = std::stoull(get("obs_id"));
    const auto& z = get("conditional_z");
    output.input.conditional_z = z.empty() ? std::numeric_limits<double>::quiet_NaN()
                                           : std::stod(z);
    output.input.diagnostic_valid = get("diagnostic_valid") == "1";
    if (forward) output.forward_candidate = get(candidate_name) == "1";
    else output.backward_candidate = get(candidate_name) == "1";
    rows.push_back(std::move(output));
  }
  return rows;
}

void WriteIntersection(const Args& args) {
  const auto forward = ReadMembership(args.forward_trace, true);
  const auto backward = ReadMembership(args.backward_trace, false);
  const std::string forward_identity =
      JsonString(ReadAll(args.forward_status), "detector_identity");
  const std::string backward_identity =
      JsonString(ReadAll(args.backward_status), "closure_identity");
  const auto result = uifgo::IntersectPlCusumSupport(
      forward, backward, forward_identity, backward_identity);
  if (!result.valid) throw std::runtime_error("intersection failed: " + result.status);
  auto csv = Open(args.output_dir / "bidirectional_support.csv");
  csv << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,forward_candidate,backward_candidate,final_candidate,segment_id,segment_ordinal\n";
  for (const auto& row : result.rows) {
    csv << row.input.timestamp << ',' << row.input.group_id << ','
        << row.input.keyframe_id << ',' << row.input.source_order << ','
        << row.input.tag_id << ',' << row.input.anchor_id << ','
        << row.input.obs_id << ',' << row.forward_candidate << ','
        << row.backward_candidate << ',' << row.final_candidate << ','
        << row.segment_id << ',';
    if (row.segment_ordinal != std::numeric_limits<size_t>::max())
      csv << row.segment_ordinal;
    csv << '\n';
  }
  WriteSupportJson(args.output_dir / "bidirectional_support.json",
                   result.support, result.support_identity,
                   forward_identity, backward_identity);
  auto status = Open(args.output_dir / "bidirectional_status.json");
  size_t forward_count = 0, backward_count = 0, final_count = 0;
  for (const auto& row : result.rows) {
    forward_count += row.forward_candidate;
    backward_count += row.backward_candidate;
    final_count += row.final_candidate;
  }
  status << "{\n  \"schema\": \"pl_bidirectional_cusum_status_v1\",\n"
         << "  \"support_identity\": \"" << result.support_identity << "\",\n"
         << "  \"forward_detector_identity\": \"" << forward_identity << "\",\n"
         << "  \"backward_closure_identity\": \"" << backward_identity << "\",\n"
         << "  \"final_intersection_rule\": \""
         << uifgo::kPlBidirectionalIntersectionRule << "\",\n"
         << "  \"row_count\": " << result.rows.size() << ",\n"
         << "  \"forward_candidate_count\": " << forward_count << ",\n"
         << "  \"backward_candidate_count\": " << backward_count << ",\n"
         << "  \"final_candidate_count\": " << final_count << ",\n"
         << "  \"segment_count\": " << result.support.segments.size() << ",\n"
         << "  \"truth_access_count\": 0,\n"
         << "  \"production\": false\n}\n";
}

}  // namespace

int main(int argc, char** argv) try {
  const Args args = ParseArgs(argc, argv);
  if (fs::exists(args.output_dir))
    throw std::runtime_error("refusing to overwrite output directory");
  fs::create_directories(args.output_dir);
  if (args.action == Args::Action::CALIBRATE)
    WriteCalibration(args, ReadConditionalRows(args.conditional_csv));
  else if (args.action == Args::Action::DETECT)
    WriteBackwardDetection(args, ReadConditionalRows(args.conditional_csv));
  else
    WriteIntersection(args);
  return 0;
} catch (const std::exception& error) {
  std::cerr << "pl_bidirectional_cusum_preflight: " << error.what() << '\n';
  return 2;
}
