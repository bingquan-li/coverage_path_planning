#include "local_map_log.hh"

#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace pdlog
{
namespace
{
float
finiteFloat(double value, char const* field)
{
  if(!std::isfinite(value) ||
     std::abs(value) > std::numeric_limits<float>::max())
  {
    throw std::runtime_error(std::string(field) + " must be a finite float");
  }
  return static_cast<float>(value);
}

std::optional<nlohmann::json>
parseRecord(std::string_view line, std::string_view type)
{
  auto const begin = line.find('{');
  if(begin == std::string_view::npos)
  {
    return std::nullopt;
  }
  auto const json = nlohmann::json::parse(line.substr(begin), nullptr, false);
  if(json.is_discarded())
  {
    if(line.find(type) != std::string_view::npos)
    {
      throw std::runtime_error("invalid " + std::string(type) + " JSON");
    }
    return std::nullopt;
  }
  if(!json.is_object() || !json.contains("type") || json.at("type") != type)
  {
    return std::nullopt;
  }
  return json;
}
} // namespace

std::optional<LocalMapFrame>
parseLocalMapLine(std::string_view line)
{
  auto const record = parseRecord(line, "local_map");
  if(!record)
  {
    return std::nullopt;
  }
  auto const& json = *record;

  auto const& size_json = json.at("map_size");
  if(!size_json.is_number_integer() || size_json <= 0 ||
     size_json > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::runtime_error("map_size must be a positive uint32 integer");
  }
  auto const size = size_json.get<std::uint32_t>();
  auto const resolution =
      finiteFloat(json.at("map_scale").get<double>(), "map_scale");
  if(resolution <= 0.0F)
  {
    throw std::runtime_error("map_scale must be positive");
  }
  auto const& pos = json.at("pos");
  if(!pos.is_array() || pos.size() != 2)
  {
    throw std::runtime_error("pos must contain exactly [x, y]");
  }
  double const x = pos.at(0).get<double>();
  double const y = pos.at(1).get<double>();
  auto const center_x = finiteFloat(x, "pos.x");
  auto const center_y = finiteFloat(y, "pos.y");
  double const half_extent = static_cast<double>(size) * resolution * 0.5;
  auto const origin_x = finiteFloat(x - half_extent, "origin.x");
  auto const origin_y = finiteFloat(y - half_extent, "origin.y");

  auto map =
      local_map_decoder::decodeLocalMapData(json.at("data").get<std::string>());
  std::uint64_t const cell_count = static_cast<std::uint64_t>(size) * size;
  if(map.packed.size() != (cell_count + 3) / 4)
  {
    throw std::runtime_error("map_size does not match decoded cell count");
  }
  // 每字节包含 4 个栅格；奇数边长时，最后一个字节可以带尾部填充位。
  map.cells.resize(static_cast<std::size_t>(cell_count));
  return LocalMapFrame{std::move(map),
                       size,
                       resolution,
                       center_x,
                       center_y,
                       origin_x,
                       origin_y};
}

std::optional<RobotFootprint>
parseFootprintLine(std::string_view line)
{
  auto const record = parseRecord(line, "foot_print");
  if(!record)
  {
    return std::nullopt;
  }
  auto const& data = record->at("data");
  if(!data.is_array() || data.size() < 3)
  {
    throw std::runtime_error(
        "foot_print.data must contain at least 3 vertices");
  }
  RobotFootprint footprint;
  footprint.vertices.reserve(data.size() + 1);
  for(auto const& point : data)
  {
    if(!point.is_array() || point.size() != 2)
    {
      throw std::runtime_error("foot_print vertex must contain exactly [x, y]");
    }
    footprint.vertices.push_back(
        {finiteFloat(point.at(0).get<double>(), "foot_print.x"),
         finiteFloat(point.at(1).get<double>(), "foot_print.y")});
  }
  std::set<std::array<float, 2>> const distinct(footprint.vertices.begin(),
                                                footprint.vertices.end());
  if(distinct.size() < 3)
  {
    throw std::runtime_error(
        "foot_print requires at least 3 distinct vertices");
  }
  if(footprint.vertices.front() != footprint.vertices.back())
  {
    footprint.vertices.push_back(footprint.vertices.front());
  }
  return footprint;
}

std::optional<GlobalPathSegment>
parseGlobalPathsLine(std::string_view line)
{
  auto const record = parseRecord(line, "global_paths");
  if(!record)
    return std::nullopt;
  auto const& number = record->at("number_of_segment");
  auto const& total = record->at("total_number_of_segments");
  if(!total.is_number_integer() || total <= 0 ||
     total > std::numeric_limits<std::uint32_t>::max() ||
     !number.is_number_integer() || number < 0 || number >= total)
  {
    throw std::runtime_error("global_paths requires 0 <= number_of_segment < "
                             "total_number_of_segments");
  }
  auto const& data = record->at("data");
  if(!data.is_array())
    throw std::runtime_error("global_paths.data must be an array");
  GlobalPathSegment segment{number.get<std::uint32_t>(),
                            total.get<std::uint32_t>(),
                            {}};
  segment.vertices.reserve(data.size());
  for(auto const& point : data)
  {
    if(!point.is_array() || point.size() != 2)
    {
      throw std::runtime_error(
          "global_paths vertex must contain exactly [x, y]");
    }
    segment.vertices.push_back(
        {finiteFloat(point.at(0).get<double>(), "global_paths.x"),
         finiteFloat(point.at(1).get<double>(), "global_paths.y")});
  }
  return segment;
}

std::optional<GlobalPaths>
GlobalPathsAssembler::push(GlobalPathSegment segment)
{
  if(segment.total == 0 || segment.number >= segment.total)
  {
    reset();
    throw std::runtime_error("invalid global_paths segment index");
  }
  if(segment.number == 0)
  {
    reset();
    total_ = segment.total;
  }
  if(total_ == 0 || total_ != segment.total)
  {
    reset();
    throw std::runtime_error(
        "global_paths missing segment 0 or inconsistent segment count");
  }
  segments_[segment.number] = std::move(segment.vertices);
  if(segments_.size() != total_)
    return std::nullopt;
  GlobalPaths paths;
  paths.reserve(segments_.size());
  for(auto& [number, vertices] : segments_)
    paths.push_back(std::move(vertices));
  reset();
  return paths;
}
} // namespace pdlog
