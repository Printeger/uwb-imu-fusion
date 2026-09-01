#include "uwb_imu_pl/io/run_logger.hpp"

#include "uwb_imu_pl/config/integrity_config.hpp"

#include <boost/filesystem.hpp>

#include <chrono>
#include <iomanip>
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

void requireOpen(const std::ofstream& stream, const std::string& path) {
  if (!stream) throw std::runtime_error("cannot open run output: " + path);
}

}  // namespace

RunLogger::RunLogger(const std::string& output_directory)
    : directory_(output_directory) {
  boost::filesystem::create_directories(directory_);
  states_.open(directory_ + "/states.csv");
  residuals_.open(directory_ + "/residuals.csv");
  integrity_.open(directory_ + "/integrity.csv");
  timing_.open(directory_ + "/timing.csv");
  events_.open(directory_ + "/events.csv");
  requireOpen(states_, directory_ + "/states.csv");
  requireOpen(residuals_, directory_ + "/residuals.csv");
  requireOpen(integrity_, directory_ + "/integrity.csv");
  requireOpen(timing_, directory_ + "/timing.csv");
  requireOpen(events_, directory_ + "/events.csv");
  states_ << "timestamp_ns,state_id,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  residuals_ << "timestamp_ns,factor_id,anchor_id,row_role,raw,whitened\n";
  integrity_ << "timestamp_ns,detector,statistic,threshold,dof,passed,global_graph_statistic,uwb_postfit_statistic,conditional_statistic,pl_x,pl_y,pl_z,hpl_box,vpl,availability,label,batch_committed,reason\n";
  timing_ << "timestamp_ns,stage,wall_ms,success\n";
  events_ << "timestamp_ns,event,detail\n";
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
      << "  \"schema_version\": " << csv(m.schema_version) << ",\n"
      << "  \"created_utc\": " << csv(m.created_utc) << ",\n"
      << "  \"git_sha\": " << csv(m.git_sha) << ",\n"
      << "  \"git_dirty\": " << (m.git_dirty ? "true" : "false") << ",\n"
      << "  \"config_path\": " << csv(m.config_path) << ",\n"
      << "  \"config_hash\": " << csv(m.config_hash) << ",\n"
      << "  \"seed\": " << m.seed << ",\n"
      << "  \"build_type\": " << csv(m.build_type) << ",\n"
      << "  \"compiler\": " << csv(m.compiler) << ",\n"
      << "  \"os\": " << csv(m.os) << ",\n"
      << "  \"gtsam_version\": " << csv(m.gtsam_version) << ",\n"
      << "  \"eigen_version\": " << csv(m.eigen_version) << "\n}\n";
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
  const char* role_name = role == RowRole::Measurement ? "Measurement" :
      (role == RowRole::TrustedPrior ? "TrustedPrior" : "Regularizer");
  residuals_ << t.value() << ',' << factor.value() << ',' << anchor.value()
             << ',' << role_name << ',' << raw << ',' << whitened << '\n';
}

void RunLogger::writeIntegrity(const IntegrityOutput& o) {
  const auto& p = o.protection_level;
  integrity_ << o.timestamp.value() << ',' << csv(o.detector.detector_type)
             << ',' << o.detector.statistic << ',' << o.detector.threshold
             << ',' << o.detector.dof << ',' << o.detector.passed << ','
             << o.global_graph_residual_statistic << ','
             << o.uwb_postfit_residual_statistic << ','
             << o.conditional_innovation_statistic << ','
             << p.pl_xyz_m.x() << ',' << p.pl_xyz_m.y() << ',' << p.pl_xyz_m.z()
             << ',' << p.hpl_box_m << ',' << p.vpl_m << ','
             << toString(p.availability) << ',' << toString(p.label) << ','
             << o.batch_committed << ',' << csv(p.reason) << '\n';
}

void RunLogger::writeTiming(TimestampNs t, const std::string& stage,
                            double wall_ms, bool success) {
  timing_ << t.value() << ',' << csv(stage) << ',' << wall_ms << ',' << success << '\n';
}

void RunLogger::writeEvent(TimestampNs t, const std::string& event,
                           const std::string& detail) {
  events_ << t.value() << ',' << csv(event) << ',' << csv(detail) << '\n';
}

void RunLogger::writeSummary(const std::string& status,
                             const std::string& detail) const {
  std::ofstream out(directory_ + "/summary.json");
  requireOpen(out, directory_ + "/summary.json");
  out << "{\"status\":" << csv(status) << ",\"detail\":" << csv(detail) << "}\n";
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
  return manifest;
}

}  // namespace uwb_imu_pl
