#include "frame_export.hh"

#include "local_map_log.hh"
#include "navigation_log.hh"

#include <optional>
#include <stdexcept>
#include <utility>

namespace pdlog
{
namespace
{
nlohmann::json
source(LogLine const& line, std::optional<std::int64_t> time_ms)
{
  return {
      {"line", line.line_number},
      {"robot_time_ns",
       time_ms ? nlohmann::json(*time_ms * 1000000) : nlohmann::json(nullptr)}};
}

nlohmann::json
payload(std::string const& line)
{
  return nlohmann::json::parse(line.substr(line.find('{')));
}
} // namespace

nlohmann::json
exportFrame(ILogLineReader& reader,
            std::string_view timeline,
            std::int64_t time)
{
  if(timeline != "robot_time" && timeline != "log_line")
  {
    throw std::invalid_argument("select robot_time or log_line; other "
                                "timelines do not align maps and navigation");
  }
  nlohmann::json result = {
      {"schema_version", 1},
      {"selection", {{"timeline", timeline}, {"time", time}}},
      {"local_map", nullptr},
      {"foot_print", nullptr},
      {"navigation", nullptr},
      {"skipped_invalid_records", 0}};
  LogClock clock;
  std::optional<nlohmann::json> pending_footprint;
  std::optional<std::int64_t> map_key;
  std::optional<std::int64_t> navigation_key;
  std::optional<LocalMapFrame> selected_frame;
  std::int64_t frame_index = 0;
  std::size_t skipped = 0;
  while(auto line = reader.next_line())
  {
    auto const time_ms = clock.observe(line->text);
    auto const key =
        timeline == "log_line"
            ? std::optional<std::int64_t>(line->line_number)
            : (time_ms ? std::optional<std::int64_t>(*time_ms * 1000000)
                       : std::nullopt);
    // 使用与回放相同的解析、坏行处理和配对规则。
    std::optional<NavigationSample> navigation;
    try
    {
      navigation = parseNavigationLine(line->text);
    }
    catch(std::exception const&)
    {
      ++skipped;
      continue;
    }
    if(navigation)
    {
      if(key && *key <= time && (!navigation_key || *key >= *navigation_key))
      {
        navigation_key = key;
        result["navigation"] = {{"source", source(*line, time_ms)},
                                {"linear_speed", navigation->linear_speed},
                                {"linear_speed_unit", "m/s"},
                                {"angular_speed", navigation->angular_speed},
                                {"angular_speed_unit", "rad/s"},
                                {"status", navigation->status}};
      }
      continue;
    }

    std::optional<LocalMapFrame> frame;
    try
    {
      if(parseFootprintLine(line->text))
      {
        // 保留日志中的 double 精度，而不从渲染用的 float 顶点反推。
        pending_footprint = payload(line->text);
        (*pending_footprint)["source"] = source(*line, time_ms);
        (*pending_footprint)["coordinate_system"] = "world";
        (*pending_footprint)["unit"] = "m";
        continue;
      }
      frame = parseLocalMapLine(line->text);
    }
    catch(std::exception const&)
    {
      ++skipped;
      pending_footprint.reset();
      continue;
    }
    if(!frame)
    {
      continue;
    }
    auto footprint = std::exchange(pending_footprint, std::nullopt);
    if(key && *key <= time && (!map_key || *key >= *map_key))
    {
      map_key = key;
      auto map = payload(line->text);
      map.erase("data");
      selected_frame = std::move(frame);
      map["encoding"] = "class_id_0_to_3";
      map["row_order"] = "bottom_to_top";
      map["coordinate_system"] = "world";
      map["position_semantics"] = "map_center";
      map["source"] = source(*line, time_ms);
      map["frame"] = frame_index;
      double const half = map.at("map_size").get<double>() *
                          map.at("map_scale").get<double>() / 2;
      map["origin"] = {map.at("pos").at(0).get<double>() - half,
                       map.at("pos").at(1).get<double>() - half};
      result["local_map"] = std::move(map);
      result["foot_print"] =
          footprint ? std::move(*footprint) : nlohmann::json(nullptr);
    }
    ++frame_index;
  }
  if(result.at("local_map").is_null())
  {
    throw std::runtime_error("no map at or before the selected time; select a "
                             "time containing a local_map");
  }
  // 只为最终选中的地图生成 JSON 栅格，避免每张候选地图都做一次转换。
  auto cells = nlohmann::json::array();
  for(std::size_t row = 0; row < selected_frame->size; ++row)
  {
    auto values = nlohmann::json::array();
    for(std::size_t col = 0; col < selected_frame->size; ++col)
    {
      values.push_back(
          selected_frame->map.cells[row * selected_frame->size + col]);
    }
    cells.push_back(std::move(values));
  }
  result["local_map"]["data"] = std::move(cells);
  result["skipped_invalid_records"] = skipped;
  return result;
}
} // namespace pdlog
