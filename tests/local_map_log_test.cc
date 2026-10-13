#include "local_map_log.hh"

#include <nlohmann/json.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
void
require(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

template <class F>
void
expectFailure(F action)
{
  try
  {
    action();
  }
  catch(std::exception const&)
  {
    return;
  }
  throw std::runtime_error("expected invalid local_map to fail");
}

nlohmann::json
sample()
{
  // gzip 解压后为 0x1b，按高位优先展开为 0、1、2、3。
  return {{"type", "local_map"},
          {"data", "H4sIAAAAAAACA5MGAGEmZ1gBAAAA"},
          {"map_scale", 0.5},
          {"map_size", 2},
          {"pos", {-1.0, 2.0}}};
}
} // namespace

int
main()
{
  try
  {
    require(!pdlog::parseLocalMapLine("ordinary local_map diagnostic\n"),
            "plain text should be ignored");
    require(!pdlog::parseLocalMapLine("prefix: {broken other JSON}"),
            "unrelated malformed JSON should be ignored");
    require(!pdlog::parseLocalMapLine("prefix: {}"),
            "missing type should be ignored");
    auto other = sample();
    other["type"] = "lidar_map";
    other["data"] = "invalid";
    require(!pdlog::parseLocalMapLine(other.dump()), "exact type filtering");

    auto const frame = pdlog::parseLocalMapLine(
        "21:40:25 362(85050)D/debug_bridge: " + sample().dump() + "\r\n");
    require(frame && frame->size == 2 && frame->resolution == 0.5F,
            "metadata parsing");
    require(frame->map.cells == local_map_decoder::Bytes({0, 1, 2, 3}),
            "base64, gzip and 2-bit cell decoding");
    require(frame->center_x == -1.0F && frame->center_y == 2.0F &&
                frame->origin_x == -1.5F && frame->origin_y == 1.5F,
            "pos is the center, not the lower-left corner");
    auto odd = sample();
    odd["map_size"] = 3;
    odd["data"] = "H4sIAAAAAAACA5OWdgAAiR/YMAMAAAA=";
    auto const odd_frame = pdlog::parseLocalMapLine(odd.dump());
    require(odd_frame && odd_frame->map.cells.size() == 9 &&
                odd_frame->map.cells.back() == 1,
            "packed padding for an odd-sized map");

    for(auto const* key : {"data", "map_size", "map_scale", "pos"})
    {
      auto missing = sample();
      missing.erase(key);
      expectFailure([&] { pdlog::parseLocalMapLine(missing.dump()); });
    }
    auto const invalid = [](char const* field, nlohmann::json value)
    {
      auto json = sample();
      json[field] = std::move(value);
      expectFailure([&] { pdlog::parseLocalMapLine(json.dump()); });
    };
    for(auto const& size : {nlohmann::json(0),
                            nlohmann::json(-1),
                            nlohmann::json(2.5),
                            nlohmann::json("2"),
                            nlohmann::json(4294967296ULL)})
    {
      invalid("map_size", size);
    }
    invalid("map_size", 300); // 有效压缩流，但尺寸不匹配。
    invalid("map_size", true);
    invalid("map_scale", 0);
    invalid("map_scale", true);
    invalid("map_scale", -0.01);
    invalid("map_scale", 1e100);
    invalid("map_scale", 1e-100);
    invalid("pos", {1});
    invalid("pos", {1, 2, 3});
    invalid("pos", {1, 1e100});
    invalid("pos", {"x", 2});
    invalid("pos", {true, 2});
    invalid("data", "bad-base64");
    invalid("data", "AAAA");                         // Base64 有效，gzip 无效。
    invalid("data", "H4sIAAAAAAACA5MGAGEmZ1gBAAAB"); // gzip 长度校验损坏。
    expectFailure(
        [] { pdlog::parseLocalMapLine("prefix: {\"type\":\"local_map\""); });

    auto footprint_json = nlohmann::json::parse(R"({"type":"foot_print","data":[
      [-3.045,-2.266],[-3.158,-2.242],[-3.47,-1.879],[-3.476,-1.763],
      [-2.76,-1.082],[-2.547,-1.107],[-2.255,-1.447],[-2.263,-1.661],
      [-3.045,-2.266]]})");
    auto const footprint = pdlog::parseFootprintLine(
        "21:40:43 470(85050)D/debug_bridge: " + footprint_json.dump() + "\r\n");
    require(footprint && footprint->vertices.size() == 9,
            "already closed footprint must not gain another vertex");
    require(
        footprint->vertices.front() == std::array<float, 2>{-3.045F, -2.266F} &&
            footprint->vertices[1] == std::array<float, 2>{-3.158F, -2.242F} &&
            footprint->vertices.back() == footprint->vertices.front(),
        "footprint must preserve world coordinates and vertex order");
    footprint_json["data"].erase(8);
    auto const open_footprint =
        pdlog::parseFootprintLine(footprint_json.dump());
    require(open_footprint && open_footprint->vertices == footprint->vertices,
            "open footprint must be closed exactly once");
    require(!pdlog::parseFootprintLine(sample().dump()),
            "footprint parser must ignore local_map");
    require(!pdlog::parseLocalMapLine(footprint_json.dump()),
            "map parser must ignore foot_print");
    for(auto const& data : {nlohmann::json::array(),
                            nlohmann::json({{1, 2}, {3, 4}}),
                            nlohmann::json({{1, 2}, {3, 4}, {1, 2}}),
                            nlohmann::json({{1, 2, 3}, {3, 4}, {5, 6}}),
                            nlohmann::json({{"x", 2}, {3, 4}, {5, 6}}),
                            nlohmann::json({{1e100, 2}, {3, 4}, {5, 6}})})
    {
      footprint_json["data"] = data;
      expectFailure([&] { pdlog::parseFootprintLine(footprint_json.dump()); });
    }
    footprint_json.erase("data");
    expectFailure([&] { pdlog::parseFootprintLine(footprint_json.dump()); });
    expectFailure(
        [] { pdlog::parseFootprintLine("prefix: {\"type\":\"foot_print\""); });
    std::cout << "local_map parser checks passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
