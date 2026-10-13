#pragma once

#include "log_reader.hh"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string_view>

namespace pdlog
{
// 与 Viewer 一致：按选定时间取各实体最近的数据，地图与轮廓保持配对。
// robot_time 的单位为纳秒；log_line 为原始行号。
nlohmann::json
exportFrame(ILogLineReader& reader,
            std::string_view timeline,
            std::int64_t time);
} // namespace pdlog
