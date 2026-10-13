#include "navigation_log.hh"

#include <charconv>
#include <cmath>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace pdlog
{
std::optional<NavigationSample>
parseNavigationLine(std::string_view line)
{
  auto const tag = line.find("/move:");
  if(tag == std::string_view::npos || tag < 2 || line[tag - 2] != ')' ||
     std::string_view("VDIWEF").find(line[tag - 1]) == std::string_view::npos ||
     line.substr(0, tag).find('{') != std::string_view::npos)
  {
    return std::nullopt;
  }
  auto payload = line.substr(tag + 6);
  auto const first = payload.find_first_not_of(" \t");
  if(first == std::string_view::npos)
  {
    return std::nullopt;
  }
  payload.remove_prefix(first);
  constexpr std::string_view prefix = "navigation result speed:";
  if(payload.substr(0, prefix.size()) != prefix)
  {
    return std::nullopt;
  }

  std::istringstream values{std::string(payload.substr(prefix.size()))};
  values.imbue(std::locale::classic());
  NavigationSample result{};
  std::string linear;
  std::string angular;
  std::string status;
  std::string trailing;
  auto const parse_speed = [](std::string_view token, double& speed)
  {
    // from_chars 不接受前导 +，但日志中的有符号正数是有效数据。
    if(!token.empty() && token.front() == '+')
    {
      token.remove_prefix(1);
      if(!token.empty() && token.front() == '-')
      {
        return false;
      }
    }
    auto const parsed =
        std::from_chars(token.data(), token.data() + token.size(), speed);
    return parsed.ec == std::errc{} &&
           parsed.ptr == token.data() + token.size() && std::isfinite(speed);
  };
  if(!(values >> linear >> angular >> status) ||
     !parse_speed(linear, result.linear_speed) ||
     !parse_speed(angular, result.angular_speed) ||
     status.substr(0, 7) != "status:" || status.size() == 7 ||
     (values >> trailing))
  {
    throw std::runtime_error(
        "invalid navigation result (expected two finite speeds and status)");
  }
  // 新增状态无需改解析器，原样保留名称。
  result.status = status.substr(7);
  return result;
}

std::optional<std::int64_t>
LogClock::observe(std::string_view line)
{
  if(line.size() < 11 || line[2] != ':' || line[5] != ':' || line[8] != ' ')
  {
    return std::nullopt;
  }
  auto digit = [](char c) { return c >= '0' && c <= '9'; };
  for(auto index : {0, 1, 3, 4, 6, 7})
  {
    if(!digit(line[index]))
    {
      return std::nullopt;
    }
  }
  int const hour = (line[0] - '0') * 10 + line[1] - '0';
  int const minute = (line[3] - '0') * 10 + line[4] - '0';
  int const second = (line[6] - '0') * 10 + line[7] - '0';
  std::size_t cursor = 9;
  while(cursor < line.size() && line[cursor] == ' ')
  {
    ++cursor;
  }
  auto const start = cursor;
  int millis = 0;
  while(cursor < line.size() && digit(line[cursor]) && cursor - start < 3)
  {
    millis = millis * 10 + line[cursor++] - '0';
  }
  if(hour > 23 || minute > 59 || second > 59 || cursor == start ||
     cursor >= line.size() || line[cursor] != '(')
  {
    return std::nullopt;
  }
  constexpr std::int64_t day = 24 * 60 * 60 * 1000;
  std::int64_t time = ((hour * 60 + minute) * 60 + second) * 1000 + millis;
  if(latest_)
  {
    // 按最近的日期展开，避免轻微倒序被误认为跨天；跨午夜迟到的记录亦然。
    time += ((*latest_ - time + day / 2) / day) * day;
    if(time - *latest_ > day / 2)
    {
      time -= day;
    }
  }
  if(!latest_ || time > *latest_)
  {
    latest_ = time;
  }
  return time;
}
} // namespace pdlog
