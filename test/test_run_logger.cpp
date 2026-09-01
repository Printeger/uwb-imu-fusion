#include "uwb_imu_pl/io/run_logger.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

TEST(RunLogger, OptionalCsvCreationFollowsConfiguration) {
  const std::string disabled = "/tmp/uwb_imu_pl_logger_disabled";
  const std::string enabled = "/tmp/uwb_imu_pl_logger_enabled";
  boost::filesystem::remove_all(disabled);
  boost::filesystem::remove_all(enabled);
  {
    uwb_imu_pl::RunLogger logger(disabled, false, false);
    logger.writeResidual(uwb_imu_pl::TimestampNs(0), {}, {},
                         uwb_imu_pl::RowRole::Measurement, 1.0, 2.0);
    logger.writeTiming(uwb_imu_pl::TimestampNs(0), "test", 1.0, true);
  }
  EXPECT_FALSE(boost::filesystem::exists(disabled + "/residuals.csv"));
  EXPECT_FALSE(boost::filesystem::exists(disabled + "/timing.csv"));
  EXPECT_TRUE(boost::filesystem::exists(disabled + "/integrity.csv"));
  {
    uwb_imu_pl::RunLogger logger(enabled, true, true);
  }
  EXPECT_TRUE(boost::filesystem::exists(enabled + "/residuals.csv"));
  EXPECT_TRUE(boost::filesystem::exists(enabled + "/timing.csv"));
  boost::filesystem::remove_all(disabled);
  boost::filesystem::remove_all(enabled);
}
