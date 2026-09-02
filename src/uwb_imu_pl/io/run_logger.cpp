#include "uwb_imu_pl/io/run_logger.hpp"

#include "uwb_imu_pl/config/integrity_config.hpp"

#include <boost/filesystem.hpp>
#include <gtsam/config.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

std::string csv(const std::string& value) {
  std::string escaped = value;
  std::size_t pos = 0;
  while ((pos = escaped.find('"', pos)) != std::string::npos) {
    escaped.insert(pos, 1, '"');
    pos += 2;
  }
  return "\"" + escaped + "\"";
}

std::string json(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size() + 2);
  for (char c : value) {
    switch (c) {
      case '"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += c; break;
    }
  }
  return "\"" + escaped + "\"";
}

void requireOpen(const std::ofstream& stream, const std::string& path) {
  if (!stream) throw std::runtime_error("cannot open run output: " + path);
}

std::string cpuModel() {
  std::ifstream input("/proc/cpuinfo");
  std::string line;
  while (std::getline(input, line)) {
    const std::string key = "model name";
    if (line.compare(0, key.size(), key) == 0) {
      const auto colon = line.find(':');
      return colon == std::string::npos ? line : line.substr(colon + 2);
    }
  }
  return "unknown";
}

}  // namespace

RunLogger::RunLogger(const std::string& output_directory,
                     bool write_residuals, bool write_timing)
    : directory_(output_directory), write_residuals_(write_residuals),
      write_timing_(write_timing) {
  boost::filesystem::create_directories(directory_);
  states_.open(directory_ + "/states.csv");
  if (write_residuals_) residuals_.open(directory_ + "/residuals.csv");
  integrity_.open(directory_ + "/integrity.csv");
  if (write_timing_) timing_.open(directory_ + "/timing.csv");
  events_.open(directory_ + "/events.csv");
  ground_truth_.open(directory_ + "/ground_truth.csv");
  fault_truth_.open(directory_ + "/fault_truth.csv");
  requireOpen(states_, directory_ + "/states.csv");
  if (write_residuals_) requireOpen(residuals_, directory_ + "/residuals.csv");
  requireOpen(integrity_, directory_ + "/integrity.csv");
  if (write_timing_) requireOpen(timing_, directory_ + "/timing.csv");
  requireOpen(events_, directory_ + "/events.csv");
  requireOpen(ground_truth_, directory_ + "/ground_truth.csv");
  requireOpen(fault_truth_, directory_ + "/fault_truth.csv");
  states_ << "timestamp_ns,state_id,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  if (write_residuals_) {
    residuals_ << "timestamp_ns,factor_id,anchor_id,row_role,raw,whitened\n";
  }
  integrity_ << "timestamp_ns,group_size,measurement_model_valid,"
                "global_statistic,global_threshold,global_dof,global_passed,"
                "postfit_statistic,postfit_threshold,postfit_dof,postfit_passed,"
                "conditional_statistic,conditional_threshold,conditional_dof,"
                "conditional_passed,conditional_formal,pl_x,pl_y,pl_z,hpl_m,"
                "vpl_m,availability,label,formal_eligible,risk_budget_valid,"
                "allocated_hmi_risk,hmi_risk_requirement,batch_committed,reason\n";
  if (write_timing_) {
    timing_ << "timestamp_ns,epoch,stage,wall_ms,problem_size,hypothesis_count,"
               "factor_count,cold_warm,success\n";
  }
  events_ << "timestamp_ns,sequence,event,detail\n";
  ground_truth_ << "timestamp_ns,px,py,pz,qw,qx,qy,qz\n";
  fault_truth_ << "timestamp_ns,sequence,anchor_id,fault_mode,active,outage,"
                  "injected_bias_m,true_range_m\n";
}

void RunLogger::writeResolvedConfig(const std::string& yaml) const {
  std::ofstream out(directory_ + "/resolved_config.yaml");
  requireOpen(out, directory_ + "/resolved_config.yaml");
  out << yaml;
}

void RunLogger::writeManifest(const RunManifest& m) const {
  std::ofstream out(directory_ + "/run_manifest.json");
  requireOpen(out, directory_ + "/run_manifest.json");
  out << "{\n"
      << "  \"schema_version\": " << json(m.schema_version) << ",\n"
      << "  \"created_utc\": " << json(m.created_utc) << ",\n"
      << "  \"git_sha\": " << json(m.git_sha) << ",\n"
      << "  \"git_dirty\": " << (m.git_dirty ? "true" : "false") << ",\n"
      << "  \"config_path\": " << json(m.config_path) << ",\n"
      << "  \"config_hash\": " << json(m.config_hash) << ",\n"
      << "  \"seed\": " << m.seed << ",\n"
      << "  \"build_type\": " << json(m.build_type) << ",\n"
      << "  \"compiler\": " << json(m.compiler) << ",\n"
      << "  \"os\": " << json(m.os) << ",\n"
      << "  \"cpu\": " << json(m.cpu) << ",\n"
      << "  \"ram_bytes\": " << m.ram_bytes << ",\n"
      << "  \"gtsam_version\": " << json(m.gtsam_version) << ",\n"
      << "  \"eigen_version\": " << json(m.eigen_version) << "\n}\n";
}

void RunLogger::writeState(const NavigationState& s) {
  states_ << s.timestamp.value() << ',' << s.id.value() << ','
          << s.position_world_m.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.q_world_body.w() << ',' << s.q_world_body.vec().transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.velocity_world_mps.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.accel_bias_mps2.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.gyro_bias_radps.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << '\n';
}

void RunLogger::writeResidual(TimestampNs t, FactorId factor, AnchorId anchor,
                              RowRole role, double raw, double whitened) {
  if (!write_residuals_) return;
  const char* role_name = role == RowRole::Measurement ? "Measurement" :
      (role == RowRole::TrustedPrior ? "TrustedPrior" : "Regularizer");
  residuals_ << t.value() << ',' << factor.value() << ',' << anchor.value()
             << ',' << role_name << ',' << raw << ',' << whitened << '\n';
}

void RunLogger::writeIntegrity(const IntegrityOutput& o) {
  const auto& p = o.protection_level;
  const DetectorResult& conditional = o.detector;
  integrity_ << o.timestamp.value() << ',' << o.measurement_group_size << ','
             << o.measurement_model_valid << ','
             << o.global_detector.statistic << ','
             << o.global_detector.threshold << ',' << o.global_detector.dof
             << ',' << o.global_detector.passed << ','
             << o.postfit_detector.statistic << ','
             << o.postfit_detector.threshold << ',' << o.postfit_detector.dof
             << ',' << o.postfit_detector.passed << ','
             << conditional.statistic << ',' << conditional.threshold << ','
             << conditional.dof << ',' << conditional.passed << ','
             << (conditional.detector_type ==
                 "conditional_current_uwb_innovation_chi_square" &&
                 conditional.numerically_valid) << ','
             << p.pl_xyz_m.x() << ',' << p.pl_xyz_m.y() << ',' << p.pl_xyz_m.z()
             << ',' << p.hpl_m << ',' << p.vpl_m << ','
             << toString(p.availability) << ',' << toString(p.label) << ','
             << p.formal_eligible << ',' << p.risk_budget_valid << ','
             << p.allocated_hmi_risk << ',' << p.hmi_risk_requirement << ','
             << o.batch_committed << ',' << csv(p.reason) << '\n';
}

void RunLogger::writeTiming(TimestampNs t, const std::string& stage,
                            double wall_ms, bool success) {
  TimingRecord record;
  record.timestamp = t;
  record.stage = stage;
  record.wall_ms = wall_ms;
  record.success = success;
  writeTiming(record);
}

void RunLogger::writeTiming(const TimingRecord& record) {
  if (!write_timing_) return;
  timing_ << record.timestamp.value() << ',' << record.epoch << ','
          << csv(record.stage) << ',' << record.wall_ms << ','
          << record.problem_size << ',' << record.hypothesis_count << ','
          << record.factor_count << ',' << (record.cold ? "cold" : "warm")
          << ',' << record.success << '\n';
}

void RunLogger::writeEvent(TimestampNs t, const std::string& event,
                           const std::string& detail) {
  writeEvent(t, next_event_sequence_++, event, detail);
}

void RunLogger::writeEvent(TimestampNs t, std::uint64_t sequence,
                           const std::string& event,
                           const std::string& detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  next_event_sequence_ = std::max(next_event_sequence_, sequence + 1);
  events_ << t.value() << ',' << sequence << ',' << csv(event) << ','
          << csv(detail) << '\n';
}

void RunLogger::writeGroundTruth(const GroundTruthRecord& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  ground_truth_ << record.timestamp.value() << ','
                << record.position_world_m.transpose().format(
                       Eigen::IOFormat(Eigen::FullPrecision,
                                       Eigen::DontAlignCols, ","))
                << ',' << record.q_world_body.w() << ','
                << record.q_world_body.vec().transpose().format(
                       Eigen::IOFormat(Eigen::FullPrecision,
                                       Eigen::DontAlignCols, ","))
                << '\n';
}

void RunLogger::writeFaultTruth(const FaultTruthRecord& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  fault_truth_ << record.timestamp.value() << ',' << record.sequence << ','
               << record.anchor_id.value() << ',' << csv(record.fault_mode)
               << ',' << record.active << ',' << record.outage << ','
               << record.injected_bias_m << ',' << record.true_range_m << '\n';
}

void RunLogger::writeSummary(const std::string& status,
                             const std::string& detail) const {
  RunSummary summary;
  summary.status = status;
  summary.detail = detail;
  writeSummary(summary);
}

void RunLogger::writeSummary(const RunSummary& summary) const {
  std::ofstream out(directory_ + "/summary.json");
  requireOpen(out, directory_ + "/summary.json");
  out << "{\n"
      << "  \"status\": " << json(summary.status) << ",\n"
      << "  \"processed\": " << summary.processed << ",\n"
      << "  \"committed\": " << summary.committed << ",\n"
      << "  \"rejected\": " << summary.rejected << ",\n"
      << "  \"errors\": " << summary.errors << ",\n"
      << "  \"metrics\": {\"core_total_ms\": " << summary.core_total_ms
      << ", \"end_to_end_total_ms\": " << summary.end_to_end_total_ms
      << "},\n"
      << "  \"detail\": " << json(summary.detail) << "\n}\n";
}

RunManifest makeRunManifest(const IntegrityConfig& config,
                            const std::string& git_sha, bool git_dirty) {
  RunManifest manifest;
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::ostringstream utc;
  utc << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%SZ");
  manifest.created_utc = utc.str();
  manifest.git_sha = git_sha;
  manifest.git_dirty = git_dirty;
  manifest.config_path = config.source_path;
  manifest.config_hash = config.config_hash;
  manifest.resolved_config = config.resolved_yaml;
  manifest.seed = config.seed;
#ifdef NDEBUG
  manifest.build_type = "Release";
#else
  manifest.build_type = "Debug";
#endif
  manifest.compiler = __VERSION__;
  manifest.eigen_version = std::to_string(EIGEN_WORLD_VERSION) + "." +
      std::to_string(EIGEN_MAJOR_VERSION) + "." + std::to_string(EIGEN_MINOR_VERSION);
  struct utsname system_name {};
  manifest.os = uname(&system_name) == 0 ?
      std::string(system_name.sysname) + " " + system_name.release : "unknown";
  manifest.cpu = cpuModel();
  struct sysinfo memory_info {};
  if (sysinfo(&memory_info) == 0) {
    manifest.ram_bytes = static_cast<std::uint64_t>(memory_info.totalram) *
        static_cast<std::uint64_t>(memory_info.mem_unit);
  }
#ifdef GTSAM_VERSION_STRING
  manifest.gtsam_version = GTSAM_VERSION_STRING;
#else
  manifest.gtsam_version = "4.2.x-required";
#endif
  return manifest;
}

}  // namespace uwb_imu_pl
