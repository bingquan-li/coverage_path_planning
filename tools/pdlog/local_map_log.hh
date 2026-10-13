#pragma once

#include "local_map_decoder.h"

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

namespace pdlog
{
struct LocalMapFrame
{
  local_map_decoder::LocalMapData map;
  std::uint32_t size;
  float resolution;
  float center_x;
  float center_y;
  float origin_x;
  float origin_y;
};

// 普通日志返回 nullopt；local_map 的 JSON、元数据或压缩数据损坏时抛异常。
std::optional<LocalMapFrame>
parseLocalMapLine(std::string_view line);

struct RobotFootprint
{
  // world 坐标（米），保留日志顺序，首尾闭合。
  std::vector<std::array<float, 2>> vertices;
};

// 普通日志返回 nullopt；foot_print 的 JSON 或顶点无效时抛异常。
std::optional<RobotFootprint>
parseFootprintLine(std::string_view line);

using GlobalPaths = std::vector<std::vector<std::array<float, 2>>>;

struct GlobalPathSegment
{
  std::uint32_t number;
  std::uint32_t total;
  std::vector<std::array<float, 2>> vertices;
};

std::optional<GlobalPathSegment>
parseGlobalPathsLine(std::string_view line);

// 0 号段开启新一组路径；收齐后按编号输出，各段之间不额外连线。
// 协议没有规划 ID，因此只接受以 0 号段开头的组。
class GlobalPathsAssembler
{
public:
  std::optional<GlobalPaths>
  push(GlobalPathSegment segment);
  bool
  pending() const
  {
    return total_ != 0;
  }
  void
  reset()
  {
    total_ = 0;
    segments_.clear();
  }

private:
  std::uint32_t total_ = 0;
  std::map<std::uint32_t, std::vector<std::array<float, 2>>> segments_;
};
} // namespace pdlog
