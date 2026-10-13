#pragma once

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace coverage_path_planning::log
{

struct Options
{
  bool console = true;
  bool file = true;
#ifdef COVERAGE_PATH_PLANNING_SOURCE_DIR
  std::string log_directory =
      std::string(COVERAGE_PATH_PLANNING_SOURCE_DIR) + "/logs";
#else
  std::string log_directory = "logs";
#endif
  std::string file_path; // Optional fixed path instead of a new file in logs/.
  bool truncate_file = false;
  spdlog::level::level_enum level = spdlog::level::info;
  std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v";
};

namespace detail
{

inline std::mutex&
mutex()
{
  static std::mutex value;
  return value;
}

inline std::shared_ptr<spdlog::logger>&
current()
{
  static std::shared_ptr<spdlog::logger> value;
  return value;
}

inline std::filesystem::path
new_log_path(std::string const& directory)
{
  static std::atomic<unsigned long long> sequence{0};
  auto const now = std::chrono::system_clock::now();
  auto const time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
#ifdef _WIN32
  localtime_s(&local_time, &time);
  auto const pid = _getpid();
#else
  localtime_r(&time, &local_time);
  auto const pid = getpid();
#endif
  auto const milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) %
      std::chrono::seconds(1);

  std::ostringstream name;
  name << "coverage_path_planning_"
       << std::put_time(&local_time, "%Y-%m-%d_%H-%M-%S") << '_' << std::setw(3)
       << std::setfill('0') << milliseconds.count() << '_' << pid << '_'
       << sequence.fetch_add(1, std::memory_order_relaxed) << ".log";
  return std::filesystem::path(directory) / name.str();
}

inline std::shared_ptr<spdlog::logger>
make_logger(Options const& options)
{
  std::vector<spdlog::sink_ptr> sinks;
  if(options.console)
  {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }
  if(options.file)
  {
    auto const path = options.file_path.empty()
                          ? new_log_path(options.log_directory)
                          : std::filesystem::path(options.file_path);
    if(!path.parent_path().empty())
    {
      std::filesystem::create_directories(path.parent_path());
    }
    sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        path.string(),
        options.truncate_file));
  }
  if(sinks.empty())
  {
    throw std::invalid_argument("log: enable console or file output");
  }

  auto result = std::make_shared<spdlog::logger>("coverage_path_planning",
                                                 sinks.begin(),
                                                 sinks.end());
  result->set_level(options.level);
  result->set_pattern(options.pattern);
  result->flush_on(spdlog::level::err);
  return result;
}

} // namespace detail

// Replaces the logger. Existing shared_ptr copies remain valid.
inline void
initialize(Options const& options = {})
{
  auto replacement = detail::make_logger(options);
  std::scoped_lock const lock(detail::mutex());
  detail::current() = std::move(replacement);
}

// Initializes console and file logging on first use.
inline std::shared_ptr<spdlog::logger>
get()
{
  std::scoped_lock const lock(detail::mutex());
  auto& current = detail::current();
  if(!current)
  {
    current = detail::make_logger(Options{});
  }
  return current;
}

inline void
set_level(spdlog::level::level_enum level)
{
  get()->set_level(level);
}

inline void
flush()
{
  get()->flush();
}

template <typename... Args>
inline void
trace(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->trace(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void
debug(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->debug(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void
info(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->info(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void
warn(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->warn(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void
error(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->error(format, std::forward<Args>(args)...);
}

template <typename... Args>
inline void
critical(spdlog::format_string_t<Args...> format, Args&&... args)
{
  get()->critical(format, std::forward<Args>(args)...);
}

} // namespace coverage_path_planning::log
