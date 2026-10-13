#include "config.hh"

#include <algorithm>
#include <ostream>
#include <stdexcept>
#include <string_view>
#include <yaml-cpp/yaml.h>

namespace pdlog
{
namespace
{
// 严格校验固定宽度时间，拒绝 2 月 30 日、24:00 和尾部多余字符等输入。
// 使用公历日期运算，不调用 mktime：配置和文件名采用相同的本地时间表示，
// 仅比较其先后，不受运行机器时区或夏令时影响，也不隐式转换时区。
Timestamp
parse_timestamp(std::string const& text)
{
  auto const invalid = [&]
  { return std::invalid_argument("invalid timestamp: " + text); };
  if(text.size() != 19 || text[4] != '-' || text[7] != '-' || text[10] != ' ' ||
     text[13] != ':' || text[16] != ':')
  {
    throw invalid();
  }
  auto const number = [&](std::size_t offset, std::size_t length)
  {
    unsigned value = 0;
    for(std::size_t index = offset; index < offset + length; ++index)
    {
      if(text[index] < '0' || text[index] > '9')
      {
        throw invalid();
      }
      value = value * 10 + static_cast<unsigned>(text[index] - '0');
    }
    return value;
  };
  auto const year = number(0, 4);
  auto const month = number(5, 2);
  auto const day = number(8, 2);
  auto const hour = number(11, 2);
  auto const minute = number(14, 2);
  auto const second = number(17, 2);
  std::chrono::year_month_day const date{
      std::chrono::year{static_cast<int>(year)},
      std::chrono::month{month},
      std::chrono::day{day}};
  if(year == 0 || !date.ok() || hour > 23 || minute > 59 || second > 59)
  {
    throw invalid();
  }
  return std::chrono::sys_days{date} + std::chrono::hours{hour} +
         std::chrono::minutes{minute} + std::chrono::seconds{second};
}

// null 表示无界；非空值必须精确到分钟。缺失字段按配置错误处理，避免拼错
// 字段名时意外扩大处理范围。起止相同表示选择这一分钟内的全部文件。
std::optional<Timestamp>
load_time_bound(YAML::Node const& range, char const* key)
{
  auto const value = range[key];
  if(!value)
  {
    throw std::invalid_argument(std::string("missing time_range.") + key);
  }
  if(value.IsNull())
  {
    return std::nullopt;
  }

  if(!value.IsScalar() || value.as<std::string>().size() != 16)
  {
    throw std::invalid_argument(std::string("time_range.") + key +
                                " must be null or YYYY-MM-DD HH:MM");
  }
  return parse_timestamp(value.as<std::string>() + ":00");
}

} // namespace

// 所有配置均在扫描和解压前校验。目录相对于 YAML 文件所在目录解析，
// 因而同一份配置不会随启动目录变化而指向另一批日志。
ConversionConfig
load_config(std::filesystem::path const& config_path)
{
  auto const config = YAML::LoadFile(config_path.string());
  if(!config.IsMap())
  {
    throw std::invalid_argument("configuration must be a YAML map");
  }
  ConversionConfig options;
  auto const directory = config["input_dir"];
  if(!directory.IsScalar() || directory.as<std::string>().empty())
  {
    throw std::invalid_argument("input_dir must be a non-empty path");
  }
  options.input_dir = directory.as<std::string>();
  if(options.input_dir.is_relative())
  {
    options.input_dir = std::filesystem::absolute(config_path).parent_path() /
                        options.input_dir;
  }
  options.input_dir = options.input_dir.lexically_normal();

  auto const range = config["time_range"];
  if(!range.IsMap())
  {
    throw std::invalid_argument("time_range must contain start and end");
  }
  options.start = load_time_bound(range, "start");
  options.end = load_time_bound(range, "end");
  if(options.start && options.end && *options.start > *options.end)
  {
    throw std::invalid_argument("time_range.start must not be later than end");
  }

  // 空列表明确表示不启用关键词过滤；缺失、null 或错误类型均拒绝。
  auto const entries = config["keywords"];
  if(!entries.IsSequence())
  {
    throw std::invalid_argument(
        "keywords must be a YAML list (use [] for all lines)");
  }
  options.keywords.reserve(entries.size());
  for(std::size_t index = 0; index < entries.size(); ++index)
  {
    auto const entry = entries[index];
    if(!entry.IsScalar() || entry.as<std::string>().empty())
    {
      throw std::invalid_argument("keywords[" + std::to_string(index) +
                                  "] must be a non-empty scalar");
    }
    options.keywords.push_back(entry.as<std::string>());
  }
  if(auto const format = config["input_format"])
  {
    if(!format.IsScalar())
    {
      throw std::invalid_argument("input_format must be pdlog or text");
    }
    auto const value = format.as<std::string>();
    if(value == "text")
    {
      options.input_format = InputFormat::Text;
    }
    else if(value != "pdlog")
    {
      throw std::invalid_argument("input_format must be pdlog or text");
    }
  }
  return options;
}

namespace
{
// 文件名协议固定，直接校验字段即可，无需正则。最后一段数字仅用于识别
// 合法文件名，不参与时间解析；时间相同的文件稍后按完整文件名确定顺序。
std::optional<Timestamp>
filename_timestamp(std::string const& name, InputFormat format)
{
  constexpr std::string_view prefix = "NavigationService.g3log.";
  std::string_view const suffix =
      format == InputFormat::Pdlog ? ".pdlog" : ".log";
  if(!name.starts_with(prefix) || !name.ends_with(suffix))
  {
    return std::nullopt;
  }

  if(name.size() < prefix.size() + 17 + suffix.size())
  {
    throw std::invalid_argument("expected YYYYMMDD-HHMMSS.numeric_id");
  }

  auto const body =
      name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  if(body.size() < 17 || body[8] != '-' || body[15] != '.' ||
     !std::ranges::all_of(body.substr(16),
                          [](char ch) { return ch >= '0' && ch <= '9'; }))
  {
    throw std::invalid_argument("expected YYYYMMDD-HHMMSS.numeric_id");
  }

  return parse_timestamp(body.substr(0, 4) + "-" + body.substr(4, 2) + "-" +
                         body.substr(6, 2) + " " + body.substr(9, 2) + ":" +
                         body.substr(11, 2) + ":" + body.substr(13, 2));
}

} // namespace

// 先收集整个目录再排序，不能依赖目录迭代顺序或文件修改时间。
// 仅扫描一层，忽略其他类型文件；命名类似但时间无效的文件会报告并跳过。
// 扫描失败则抛异常，调用方不会开始解压不完整的候选列表。
std::vector<LogFile>
collect_files(ConversionConfig const& options, std::ostream& diagnostics)
{
  std::vector<LogFile> files;
  for(auto const& entry :
      std::filesystem::directory_iterator(options.input_dir))
  {
    if(!entry.is_regular_file())
    {
      continue;
    }
    std::optional<Timestamp> time;
    try
    {
      time = filename_timestamp(entry.path().filename().string(),
                                options.input_format);
    }
    catch(std::invalid_argument const& error)
    {
      diagnostics << "pdlog_decompress: skipping invalid filename "
                  << entry.path() << ": " << error.what() << '\n';
      continue;
    }

    if(!time)
    {
      continue;
    }

    // 截断秒数后比较，等价于包含 start:00 到 end:59，且不会在上界加一分钟
    // 时溢出。只筛选文件名时间，不尝试猜测文件内部的时间覆盖区间。
    auto const minute = std::chrono::floor<std::chrono::minutes>(*time);
    if((options.start && minute < *options.start) ||
       (options.end && minute > *options.end))
    {
      continue;
    }

    files.push_back({entry.path(), *time, options.input_format});
  }

  std::ranges::sort(files,
                    [](LogFile const& left, LogFile const& right)
                    {
                      if(left.time != right.time)
                      {
                        return left.time < right.time;
                      }
                      return left.path.filename() < right.path.filename();
                    });

  return files;
}

} // namespace pdlog
