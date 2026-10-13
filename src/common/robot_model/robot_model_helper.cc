#include "coverage_path_planning/common/log/log.hh"
#include "coverage_path_planning/common/robot_model/robot_model_helper.hh"

#include "coverage_path_planning/common/robot_model/robot_config_yaml.hh"

#include <filesystem>

namespace coverage_path_planning
{

RobotModelHelper const&
RobotModelHelper::instance()
{
  static RobotModelHelper const helper;
  return helper;
}


RobotModelHelper::RobotModelHelper()
{
  auto const path = std::filesystem::path(COVERAGE_PATH_PLANNING_SOURCE_DIR) /
                    "data/robot.yaml";
  auto const config = YAML::LoadFile(path.string());
  log::info("RobotConfig:{}", YAML::Dump(config));
  robot_config_ = config.as<RobotConfig>();
}
} // namespace coverage_path_planning
