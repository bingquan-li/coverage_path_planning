#include "frame_export.hh"

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Json = nlohmann::json;

class Lines final : public pdlog::ILogLineReader
{
public:
  explicit Lines(std::vector<std::string> lines)
    : lines_(std::move(lines))
  {}
  std::optional<pdlog::LogLine>
  next_line() override
  {
    if(cursor_ == lines_.size())
      return std::nullopt;
    auto text = lines_[cursor_++];
    return pdlog::LogLine{"fixture.pdlog", cursor_, std::move(text)};
  }

private:
  std::vector<std::string> lines_;
  std::size_t cursor_ = 0;
};

void
require(bool condition, char const* message)
{
  if(!condition)
    throw std::runtime_error(message);
}

std::string
map(std::string const& time)
{
  return time + "(1)D/debug_bridge: " +
         Json{{"type", "local_map"},
              {"map_size", 2},
              {"map_scale", 0.5},
              {"pos", {0.123456789012345, 2.0}},
              {"data", "H4sIAAAAAAACA5MGAGEmZ1gBAAAA"}}
             .dump();
}

std::string
foot(std::string const& time)
{
  return time + "(1)D/debug_bridge: " +
         Json{{"type", "foot_print"},
              {"data", {{0.123456789012345, 1.0}, {2.0, 1.0}, {1.0, 2.0}}}}
             .dump();
}

std::string
nav(std::string const& time, std::string const& values)
{
  return time + "(1)D/move: navigation result speed:" + values;
}

Json
snapshot(std::vector<std::string> lines,
         std::string_view timeline,
         std::int64_t time)
{
  Lines reader{std::move(lines)};
  return pdlog::exportFrame(reader, timeline, time);
}
} // namespace

int
main()
{
  try
  {
    std::vector<std::string> const lines{
        foot("00:00:00 001"),
        nav("00:00:00 002", "0 0 status:Stuck"),
        map("00:00:00 003"),
        nav("00:00:00 005", "-0.1 -0.05 status:Escaping"),
        map("00:00:00 006"),
        nav("00:00:00 004", "+0.2 0.3 status:Navigating")};
    auto result = snapshot(lines, "robot_time", 4500000);
    require(result["local_map"]["data"] == Json{{0, 1}, {2, 3}},
            "original class IDs and bottom-first rows");
    require(result["local_map"]["pos"][0] == 0.123456789012345,
            "preserve map position precision");
    require(result["foot_print"]["data"][0][0] == 0.123456789012345,
            "preserve footprint precision");
    require(result["foot_print"]["data"].size() == 3,
            "preserve original open footprint vertices");
    require(result["local_map"]["frame"] == 0 &&
                result["local_map"]["source"]["line"] == 3,
            "selected map provenance");
    require(result["navigation"]["status"] == "Navigating" &&
                result["navigation"]["source"]["line"] == 6,
            "latest time must include late-arriving records, exclude future "
            "records");
    result = snapshot(lines, "robot_time", 5500000);
    require(result["navigation"]["status"] == "Escaping",
            "out-of-order record must not replace a later timestamp");
    require(!result["foot_print"].is_null(),
            "footprint belongs to selected map");
    result = snapshot(lines, "robot_time", 6000000);
    require(result["foot_print"].is_null() && result["local_map"]["frame"] == 1,
            "missing footprint clears old one");
    result = snapshot(lines, "log_line", 3);
    require(result["navigation"]["status"] == "Stuck",
            "line selection excludes later file lines");

    result = snapshot({foot("00:00:00 001"),
                       nav("00:00:00 002", "bad 0 status:Stuck"),
                       map("00:00:00 003")},
                      "log_line",
                      3);
    require(!result["foot_print"].is_null() && result["navigation"].is_null() &&
                result["skipped_invalid_records"] == 1,
            "bad navigation does not break pairing or fabricate a state");
    result = snapshot({foot("00:00:00 001"),
                       "00:00:00 002(1)D/debug_bridge: {broken local_map",
                       map("00:00:00 003")},
                      "log_line",
                      3);
    require(
        result["foot_print"].is_null() && result["local_map"]["frame"] == 0,
        "bad maps invalidate pending footprint and do not advance frame index");
    result = snapshot({map("00:00:00 001"),
                       nav("00:00:00 002", "0 0 status:Stuck"),
                       nav("00:00:00 002", "1 2 status:NewState")},
                      "robot_time",
                      2000000);
    require(result["navigation"]["status"] == "NewState",
            "timestamp tie follows logging order");
    result = snapshot(
        {map("23:59:59 999"), nav("00:00:00 005", "1 2 status:Navigating")},
        "robot_time",
        86400005000000);
    require(result["navigation"]["status"] == "Navigating",
            "cross-midnight selection");
    result = snapshot({map("00:00:00 001"),
                       nav("00:00:00 002", "0 0 status:Stuck"),
                       nav("", "1 2 status:NewState")},
                      "robot_time",
                      3000000);
    require(result["navigation"]["status"] == "Stuck",
            "timestamp-free record must not inherit preceding time");
    result = snapshot({map(""), nav("", "1 2 status:NewState")}, "log_line", 2);
    require(result["local_map"]["source"]["robot_time_ns"].is_null(),
            "line selection supports timestamp-free data");
    for(auto const* timeline : {"frame", "robot_time"})
    {
      bool failed = false;
      try
      {
        snapshot(lines, timeline, 0);
      }
      catch(std::exception const&)
      {
        failed = true;
      }
      require(failed, "unsupported timeline or no map must fail");
    }
    std::cout << "frame_export_test passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
