#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace pdlog
{
struct NavigationSample
{
  double linear_speed;
  double angular_speed;
  std::string status;
};

// 仅匹配 move 标签下的 navigation result；坏记录抛异常，其他日志返回 nullopt。
std::optional<NavigationSample>
parseNavigationLine(std::string_view line);

class LogClock
{
public:
  // HH:mm:ss 毫秒 → 从首日午夜起的毫秒数。支持跨午夜和小范围乱序。
  // 无有效时间前缀时返回 nullopt，不沿用上一行时间。
  std::optional<std::int64_t>
  observe(std::string_view line);

private:
  std::optional<std::int64_t> latest_;
};
} // namespace pdlog
