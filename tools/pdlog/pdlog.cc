#include "application.hh"

#include <cstdio>
#include <exception>
#include <iostream>
#include <string>
#include <utility>

// 命令行只选择配置文件。目录、时间范围和关键词集中在 YAML 中维护。
int
main(int argc, char* const* argv)
{
  char const* config_path =
      COVERAGE_PATH_PLANNING_SOURCE_DIR "/configs/pdlog.yaml";
  // 记录用户是否已经指定过 --config，防止重复指定。
  bool config_given = false;
  auto const usage = [](std::ostream& output)
  { output << "usage: pdlog [--config CONFIG.yaml]\n"; };

  for(int index = 1; index < argc; ++index)
  {
    std::string const argument = argv[index];
    if(argument == "--help")
    {
      usage(std::cout);
      return 0;
    }

    if(argument == "--config")
    {
      if(config_given)
      {
        std::cerr << "pdlog_decompress: --config may only be specified once\n";
        return 2;
      }
      // 获取下一个参数，检查 --config 后面有没有路径。
      ++index;
      if(index >= argc || argv[index][0] == '\0')
      {
        std::cerr << "pdlog_decompress: --config requires a non-empty path\n";
        return 2;
      }
      config_path = argv[index];
      config_given = true;
    }
    else
    {
      std::cerr << "pdlog_decompress: unexpected argument: " << argument
                << " (configure input_dir, time_range and keywords in YAML)\n";
      usage(std::cerr);
      return 2;
    }
  }

  pdlog::ConversionConfig config;
  try
  {
    config = pdlog::load_config(config_path);
  }
  catch(std::exception const& error)
  {
    std::cerr << "pdlog_decompress: invalid configuration " << config_path
              << ": " << error.what() << '\n';
    return 2;
  }

  try
  {
    pdlog::ConversionApplication const application{std::move(config),
                                             stdout,
                                             std::cerr};
    auto const result = application.run();
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << "pdlog_decompress: " << error.what() << '\n';
    return 1;
  }
}
