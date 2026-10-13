#pragma once

#include "local_map_decoder.h"

#include <cstdint>
#include <optional>
#include <string_view>

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
} // namespace pdlog
