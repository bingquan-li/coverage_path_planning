#include "frame_export.hh"

#include <charconv>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string_view>

int
main(int argc, char const* const* argv)
{
  if(argc != 4)
  {
    std::cerr
        << "usage: local_map_export INPUT.pdlog robot_time|log_line TIME\n"
           "  TIME: integer nanoseconds for robot_time, line number for "
           "log_line.\n"
           "  Writes one snapshot as JSON to stdout.\n";
    return 2;
  }
  try
  {
    std::string_view const text = argv[3];
    std::int64_t time = 0;
    auto const parsed =
        std::from_chars(text.data(), text.data() + text.size(), time);
    if(parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    {
      throw std::invalid_argument("TIME must be an int64 integer");
    }
    pdlog::PdlogReader reader{argv[1]};
    auto snapshot = pdlog::exportFrame(reader, argv[2], time);
    snapshot["source_file"] = std::filesystem::canonical(argv[1]).string();
    // 紧凑 JSON 避免 300x300 个数各占一行，可用 JSON 工具按需格式化。
    std::cout << snapshot.dump() << '\n';
    if(!std::cout)
    {
      throw std::runtime_error("failed to write snapshot");
    }
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << "local_map_export: " << error.what() << '\n';
    return 1;
  }
}
