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
#include "uifgo/pl_persistent_cusum.h"

namespace fs = boost::filesystem;
namespace {

constexpr const char* kTaskSourceHead =
    "af0394a1b03edddd3e94d45a24c0c8d9a4a9c6b1";

struct Args {
  enum class Action { NONE, SPLIT, CALIBRATE, DETECT } action = Action::NONE;
  fs::path conditional_csv;
  fs::path calibration;
  fs::path split_manifest;
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

std::string JsonEscape(const std::string& value) {
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
    else if (arg == "--calibration") args.calibration = take();
    else if (arg == "--split-manifest") args.split_manifest = take();
    else if (arg == "--output-dir") args.output_dir = take();
    else if (arg == "--partition") args.partition = take();
    else if (arg == "--split-clean") args.action = Args::Action::SPLIT;
    else if (arg == "--calibrate-clean") args.action = Args::Action::CALIBRATE;
    else if (arg == "--detect") args.action = Args::Action::DETECT;
    else if (arg == "--help") {
      std::cout
          << "Usage:\n"
          << "  pl_persistent_cusum_preflight --split-clean --conditional-csv FILE --output-dir DIR\n"
          << "  pl_persistent_cusum_preflight --calibrate-clean --conditional-csv FILE --split-manifest FILE --output-dir DIR\n"
          << "  pl_persistent_cusum_preflight --detect --conditional-csv FILE --calibration FILE --partition all|validation --output-dir DIR\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (args.action == Args::Action::NONE || args.conditional_csv.empty() ||
      args.output_dir.empty())
    throw std::invalid_argument("action, --conditional-csv and --output-dir required");
  if (args.action == Args::Action::CALIBRATE && args.split_manifest.empty())
    throw std::invalid_argument("calibration requires --split-manifest");
  if (args.action == Args::Action::DETECT && args.calibration.empty())
    throw std::invalid_argument("detection requires --calibration");
  if (args.partition != "all" && args.partition != "validation")
    throw std::invalid_argument("partition must be all or validation");
  return args;
}

std::vector<uifgo::PlCusumInputRow> ReadRows(const fs::path& path) {
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
  size_t line_number = 1;
  while (std::getline(in, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    const auto fields = SplitCsv(line);
    auto get = [&](const std::string& name) -> const std::string& {
      const size_t index = column.at(name);
      if (index >= fields.size())
        throw std::runtime_error("short CSV row " + std::to_string(line_number));
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
    const std::string z = get("conditional_z");
    row.conditional_z = z.empty() ? std::numeric_limits<double>::quiet_NaN()
                                  : std::stod(z);
    const std::string valid = get("diagnostic_valid");
    row.diagnostic_valid = valid == "1" || valid == "true" || valid == "TRUE";
    rows.push_back(std::move(row));
  }
  return rows;
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
  const std::string actual = uifgo::Sha256FileHex(path.string());
  if (expected != actual)
    throw std::runtime_error("seal mismatch " + path.string());
}

struct Split {
  double t_min = 0.0;
  double t_max = 0.0;
  double split = 0.0;
  double calibration_end = 0.0;
  double validation_start = 0.0;
};

Split ComputeSplit(const std::vector<uifgo::PlCusumInputRow>& rows) {
  std::vector<double> valid;
  for (const auto& row : rows)
    if (row.diagnostic_valid && std::isfinite(row.conditional_z) &&
        std::isfinite(row.timestamp)) valid.push_back(row.timestamp);
  if (valid.empty()) throw std::runtime_error("no valid clean timestamps");
  const auto bounds = std::minmax_element(valid.begin(), valid.end());
  Split split;
  split.t_min = *bounds.first;
  split.t_max = *bounds.second;
  split.split = split.t_min + 0.60 * (split.t_max - split.t_min);
  split.calibration_end = split.split - 0.5;
  split.validation_start = split.split + 0.5;
  return split;
}

void WriteSplit(const Args& args,
                const std::vector<uifgo::PlCusumInputRow>& rows) {
  const Split split = ComputeSplit(rows);
  size_t calibration_count = 0, validation_count = 0, guard_count = 0;
  std::map<uifgo::PlCusumLinkKey, std::pair<size_t, size_t>> links;
  for (const auto& row : rows) {
    auto& counts = links[{row.tag_id, row.anchor_id}];
    if (row.timestamp <= split.calibration_end) {
      ++calibration_count;
      ++counts.first;
    } else if (row.timestamp >= split.validation_start) {
      ++validation_count;
      ++counts.second;
    } else {
      ++guard_count;
    }
  }
  auto out = Open(args.output_dir / "clean_split_manifest.json");
  out << "{\n"
      << "  \"schema\": \"pl_persistent_clean_split_v1\",\n"
      << "  \"input_sha256\": \"sha256:"
      << uifgo::Sha256FileHex(args.conditional_csv.string()) << "\",\n"
      << "  \"t_min\": " << split.t_min << ",\n"
      << "  \"t_max\": " << split.t_max << ",\n"
      << "  \"duration\": " << split.t_max - split.t_min << ",\n"
      << "  \"split_timestamp\": " << split.split << ",\n"
      << "  \"calibration_start\": " << split.t_min << ",\n"
      << "  \"calibration_end\": " << split.calibration_end << ",\n"
      << "  \"validation_start\": " << split.validation_start << ",\n"
      << "  \"validation_end\": " << split.t_max << ",\n"
      << "  \"guard_band_s\": 1.0,\n"
      << "  \"guard_band_row_count\": " << guard_count << ",\n"
      << "  \"calibration_row_count\": " << calibration_count << ",\n"
      << "  \"validation_row_count\": " << validation_count << ",\n"
      << "  \"per_link_row_counts\": [";
  size_t index = 0;
  for (const auto& item : links) {
    out << (index++ ? ",\n" : "\n")
        << "    {\"tag_id\": " << item.first.tag_id
        << ", \"anchor_id\": " << item.first.anchor_id
        << ", \"calibration\": " << item.second.first
        << ", \"validation\": " << item.second.second << "}";
  }
  out << "\n  ]\n}\n";
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
  const std::string manifest = ReadAll(args.split_manifest);
  const double start = JsonNumber(manifest, "calibration_start");
  const double end = JsonNumber(manifest, "calibration_end");
  const double validation_start = JsonNumber(manifest, "validation_start");
  const double validation_end = JsonNumber(manifest, "validation_end");
  auto calibration_rows = Select(rows, start, end);
  uifgo::PlCusumOptions options;
  options.threshold = std::numeric_limits<double>::max();
  const auto result = uifgo::EvaluatePlPersistentCusum(calibration_rows, options);
  if (!result.valid) throw std::runtime_error("calibration core failed: " + result.status);
  const double h = std::max(5.0, result.max_g + 1.0);
  std::set<uifgo::PlCusumLinkKey> links;
  for (const auto& row : calibration_rows) links.insert({row.tag_id, row.anchor_id});
  const std::string split_hash =
      "sha256:" + uifgo::Sha256FileHex(args.split_manifest.string());
  auto out = Open(args.output_dir / "cusum_calibration.json");
  out << "{\n"
      << "  \"schema\": \"pl_persistent_cusum_calibration_v1\",\n"
      << "  \"algorithm_version\": \""
      << uifgo::kPlPersistentCusumAlgorithmVersion << "\",\n"
      << "  \"kappa\": 0.5,\n"
      << "  \"G_calibration_max\": " << result.max_g << ",\n"
      << "  \"h_locked\": " << h << ",\n"
      << "  \"threshold_rule\": \"max(5.0,G_calibration_max+1.0)\",\n"
      << "  \"clean_split_manifest_hash\": \"" << split_hash << "\",\n"
      << "  \"calibration_input_hash\": \"sha256:"
      << uifgo::Sha256FileHex(args.conditional_csv.string()) << "\",\n"
      << "  \"calibration_start\": " << start << ",\n"
      << "  \"calibration_end\": " << end << ",\n"
      << "  \"validation_start\": " << validation_start << ",\n"
      << "  \"validation_end\": " << validation_end << ",\n"
      << "  \"row_count\": " << calibration_rows.size() << ",\n"
      << "  \"link_count\": " << links.size() << ",\n"
      << "  \"invalid_row_count\": " << result.invalid_row_count << ",\n"
      << "  \"source_commit\": \"" << kTaskSourceHead << "\",\n"
      << "  \"current_repo_head\": \"" << kTaskSourceHead << "\"\n"
      << "}\n";
}

void WriteSupportJson(const fs::path& path,
                      const uifgo::PlCusumResult& result) {
  auto out = Open(path);
  const auto& support = result.support;
  out << "{\n  \"schema\": \"" << support.schema << "\",\n"
      << "  \"provider\": \"" << support.provider << "\",\n"
      << "  \"partition_rule_version\": \""
      << support.partition_rule_version << "\",\n"
      << "  \"detector_identity\": \"" << result.detector_identity << "\",\n"
      << "  \"partition_hash\": \"" << support.partition_hash << "\",\n"
      << "  \"segments\": [";
  for (size_t i = 0; i < support.segments.size(); ++i) {
    const auto& segment = support.segments[i];
    out << (i ? ",\n" : "\n") << "    {\"segment_id\": \""
        << JsonEscape(segment.segment_id) << "\", \"segment_ordinal\": "
        << segment.segment_ordinal << ", \"tag_id\": " << segment.tag_id
        << ", \"anchor_id\": " << segment.anchor_id
        << ", \"estimated_onset_time\": " << segment.start_time
        << ", \"end_time\": " << segment.end_time
        << ", \"observation_count\": " << segment.observation_count
        << ", \"obs_ids\": [";
    for (size_t j = 0; j < segment.obs_ids.size(); ++j)
      out << (j ? "," : "") << segment.obs_ids[j];
    out << "]}";
  }
  out << "\n  ]\n}\n";
}

void WriteDetection(const Args& args,
                    const std::vector<uifgo::PlCusumInputRow>& all_rows) {
  VerifySiblingSeal(args.calibration);
  const std::string calibration = ReadAll(args.calibration);
  const double kappa = JsonNumber(calibration, "kappa");
  const double threshold = JsonNumber(calibration, "h_locked");
  const double validation_start = JsonNumber(calibration, "validation_start");
  const double validation_end = JsonNumber(calibration, "validation_end");
  const std::string split_hash =
      JsonString(calibration, "clean_split_manifest_hash");
  std::vector<uifgo::PlCusumInputRow> rows = all_rows;
  if (args.partition == "validation")
    rows = Select(all_rows, validation_start, validation_end);
  uifgo::PlCusumOptions options;
  options.kappa = kappa;
  options.threshold = threshold;
  options.calibration_hash =
      "sha256:" + uifgo::Sha256FileHex(args.calibration.string());
  options.split_manifest_hash = split_hash;
  const auto result = uifgo::EvaluatePlPersistentCusum(rows, options);
  if (!result.valid) throw std::runtime_error("detector core failed: " + result.status);

  auto trace = Open(args.output_dir / "cusum_trace.csv");
  trace << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,conditional_z,diagnostic_valid,kappa,h_locked,increment,G_before,G_after,reset,reset_reason,excursion_id,excursion_start_time,threshold_crossing,first_alarm_for_excursion,candidate\n";
  for (const auto& row : result.trace) {
    trace << row.input.timestamp << ',' << row.input.group_id << ','
          << row.input.keyframe_id << ',' << row.input.source_order << ','
          << row.input.tag_id << ',' << row.input.anchor_id << ','
          << row.input.obs_id << ',';
    if (std::isfinite(row.input.conditional_z)) trace << row.input.conditional_z;
    trace << ',' << row.input.diagnostic_valid << ',' << kappa << ','
          << threshold << ',';
    if (std::isfinite(row.increment)) trace << row.increment;
    trace << ',' << row.g_before << ',' << row.g_after << ',' << row.reset
          << ',' << row.reset_reason << ',' << row.excursion_id << ',';
    if (std::isfinite(row.excursion_start_time)) trace << row.excursion_start_time;
    trace << ',' << row.threshold_crossing << ','
          << row.first_alarm_for_excursion << ',' << row.candidate << '\n';
  }
  auto candidates = Open(args.output_dir / "cusum_candidates.csv");
  candidates << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,excursion_id,candidate\n";
  for (const auto& row : result.trace)
    if (row.candidate)
      candidates << row.input.timestamp << ',' << row.input.group_id << ','
                 << row.input.keyframe_id << ',' << row.input.source_order << ','
                 << row.input.tag_id << ',' << row.input.anchor_id << ','
                 << row.input.obs_id << ',' << row.excursion_id << ",1\n";
  WriteSupportJson(args.output_dir / "cusum_support.json", result);
  fs::copy_file(args.output_dir / "cusum_support.json",
                args.output_dir / "support_partition.json",
                fs::copy_option::overwrite_if_exists);

  auto status = Open(args.output_dir / "cusum_status.json");
  status << "{\n  \"schema\": \"pl_persistent_cusum_status_v1\",\n"
         << "  \"provider\": \"" << uifgo::kPlPersistentCusumProvider << "\",\n"
         << "  \"detector_identity\": \"" << result.detector_identity << "\",\n"
         << "  \"kappa\": " << kappa << ",\n"
         << "  \"h_locked\": " << threshold << ",\n"
         << "  \"calibration_hash\": \"" << options.calibration_hash << "\",\n"
         << "  \"split_manifest_hash\": \"" << split_hash << "\",\n"
         << "  \"input_hash\": \"sha256:"
         << uifgo::Sha256FileHex(args.conditional_csv.string()) << "\",\n"
         << "  \"row_count\": " << result.trace.size() << ",\n"
         << "  \"valid_row_count\": " << result.valid_row_count << ",\n"
         << "  \"invalid_row_count\": " << result.invalid_row_count << ",\n"
         << "  \"link_count\": " << result.per_link_max_g.size() << ",\n"
         << "  \"alarm_count\": " << result.alarm_count << ",\n"
         << "  \"alarm_link_count\": " << result.alarm_link_count << ",\n"
         << "  \"segment_count\": " << result.segment_count << ",\n"
         << "  \"max_G\": " << result.max_g << ",\n"
         << "  \"per_link_max_G\": [";
  size_t index = 0;
  for (const auto& item : result.per_link_max_g)
    status << (index++ ? ",\n" : "\n") << "    {\"tag_id\": "
           << item.first.tag_id << ", \"anchor_id\": "
           << item.first.anchor_id << ", \"max_G\": " << item.second << "}";
  status << "\n  ],\n  \"reset_counts\": {";
  index = 0;
  for (const auto& item : result.reset_counts)
    status << (index++ ? ", " : "") << "\"" << item.first << "\": " << item.second;
  status << "},\n  \"group_statistic_used_for_decision\": false,\n"
         << "  \"truth_access_count\": 0,\n"
         << "  \"production\": false\n}\n";
}

}  // namespace

int main(int argc, char** argv) try {
  const Args args = ParseArgs(argc, argv);
  if (fs::exists(args.output_dir))
    throw std::runtime_error("refusing to overwrite output directory");
  fs::create_directories(args.output_dir);
  const auto rows = ReadRows(args.conditional_csv);
  if (args.action == Args::Action::SPLIT) WriteSplit(args, rows);
  else if (args.action == Args::Action::CALIBRATE) WriteCalibration(args, rows);
  else WriteDetection(args, rows);
  return 0;
} catch (const std::exception& error) {
  std::cerr << "pl_persistent_cusum_preflight: " << error.what() << '\n';
  return 2;
}
