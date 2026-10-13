#include "local_map_log.hh"

#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
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
} // namespace

std::optional<LocalMapFrame>
parseLocalMapLine(std::string_view line)
{
  auto const begin = line.find('{');
  if(begin == std::string_view::npos)
  {
    return std::nullopt;
  }
  auto const json = nlohmann::json::parse(line.substr(begin), nullptr, false);
  if(json.is_discarded())
  {
    if(line.find("local_map") != std::string_view::npos)
    {
      throw std::runtime_error("invalid local_map JSON");
    }
    return std::nullopt;
  }
  if(!json.is_object() || !json.contains("type") ||
     json.at("type") != "local_map")
  {
    return std::nullopt;
  }

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
} // namespace pdlog
