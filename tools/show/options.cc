#include "options.hh"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace show
{
namespace
{
template <typename T>
T
required(YAML::Node const& node, std::string const& key)
{
  auto const value = node[key];
  if(!value || value.IsNull())
  {
    throw std::invalid_argument("missing show.yaml setting: " + key);
  }
  return value.as<T>();
}

} // namespace

Options
load_options()
{
  auto const project_root =
      std::filesystem::path(COVERAGE_PATH_PLANNING_SOURCE_DIR);
  auto const config_path = project_root / "configs/show.yaml";
  auto const yaml = YAML::LoadFile(config_path.string());
  if(!yaml.IsMap() || !yaml["replay"].IsMap() || !yaml["foxglove"].IsMap())
  {
    throw std::invalid_argument("show.yaml requires replay and foxglove maps");
  }
  auto const resolve_path = [&](std::string const& text)
  {
    if(text.empty())
    {
      throw std::invalid_argument("show.yaml contains an empty path");
    }
    auto const path = std::filesystem::path(text);
    return (path.is_absolute() ? path : project_root / path).lexically_normal();
  };

  Options options;
  options.input = resolve_path(required<std::string>(yaml, "log_file"));
  auto const replay = yaml["replay"];
  options.speed = required<double>(replay, "speed");
  options.frame_rate_hz = required<double>(replay, "frame_rate_hz");
  options.start_delay = required<double>(replay, "start_delay_seconds");
  if(auto const spacing = replay["footprint_spacing_m"])
  {
    options.footprint_spacing_m = spacing.as<double>();
  }
  if(auto const yaw_step = replay["footprint_yaw_step_deg"])
  {
    options.footprint_yaw_step_deg = yaw_step.as<double>();
  }
  options.loop = required<bool>(replay, "loop");
  if(auto const final_only = replay["final_only"])
  {
    options.final_only = final_only.as<bool>();
  }
  options.show_footprint = required<bool>(replay, "show_footprint");
  options.show_trajectory = required<bool>(replay, "show_trajectory");
  if(auto const water = yaml["water_leak"])
  {
    if(!water.IsMap())
    {
      throw std::invalid_argument("water_leak must be a map");
    }
    if(water["enabled"])
    {
      options.water_leak_enabled = water["enabled"].as<bool>();
    }
    if(water["suction"])
    {
      options.suction = water["suction"].as<bool>();
    }
  }
  if(!std::isfinite(options.speed) || options.speed < 0.0 ||
     !std::isfinite(options.frame_rate_hz) || options.frame_rate_hz <= 0.0 ||
     options.frame_rate_hz > 1000.0 || !std::isfinite(options.start_delay) ||
     options.start_delay < 0.0 || !std::isfinite(options.footprint_spacing_m) ||
     options.footprint_spacing_m <= 0.0 ||
     !std::isfinite(options.footprint_yaw_step_deg) ||
     options.footprint_yaw_step_deg <= 0.0 ||
     options.footprint_yaw_step_deg > 180.0)
  {
    throw std::invalid_argument("invalid replay settings in show.yaml");
  }

  auto const foxglove = yaml["foxglove"];
  options.websocket = required<bool>(foxglove, "websocket");
  options.host = required<std::string>(foxglove, "host");
  options.frame_id = required<std::string>(foxglove, "frame_id");
  auto const port = required<int>(foxglove, "port");
  if(options.host.empty() || options.frame_id.empty() || port < 1 ||
     port > 65535)
  {
    throw std::invalid_argument("invalid foxglove settings in show.yaml");
  }
  options.port = static_cast<std::uint16_t>(port);
  if(auto const mcap = foxglove["mcap_path"]; mcap && !mcap.IsNull())
  {
    options.mcap_path = resolve_path(mcap.as<std::string>()).string();
  }
  if(!options.websocket && !options.mcap_path)
  {
    throw std::invalid_argument("show.yaml must enable websocket or mcap_path");
  }
  return options;
}

} // namespace show
