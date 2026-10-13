#include "local_map_log.hh"
#include "log_reader.hh"
#include "navigation_log.hh"

#include <rerun.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <locale>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

void
showGlobalPaths(rerun::RecordingStream const& rec,
                pdlog::GlobalPaths const& paths)
{
  // 全局坐标直接绘制；不闭合、不简化，也不跨段连线。
  if(std::all_of(paths.begin(),
                 paths.end(),
                 [](auto const& path) { return path.empty(); }))
  {
    rec.log("world/global_paths", rerun::Clear::RECURSIVE);
    return;
  }
  std::vector<rerun::LineStrip3D> strips;
  std::vector<rerun::Position3D> singletons;
  for(auto const& path : paths)
  {
    std::vector<rerun::Position3D> points;
    for(auto const& point : path)
      points.emplace_back(point[0], point[1], 0.04F);
    if(points.size() == 1)
      singletons.push_back(points.front());
    strips.emplace_back(std::move(points));
  }
  rec.log("world/global_paths",
          rerun::LineStrips3D(strips)
              .with_colors(rerun::Color(85, 190, 145))
              .with_radii(rerun::Radius::ui_points(1.5F)));
  // 单点段没有线段，额外用点标出；下一组不再包含单点时清除。
  if(singletons.empty())
    rec.log("world/global_paths/singletons", rerun::Clear::FLAT);
  else
    rec.log("world/global_paths/singletons",
            rerun::Points3D(singletons)
                .with_colors(rerun::Color(85, 190, 145))
                .with_radii(rerun::Radius::ui_points(2.5F)));
}

void
showNavigation(rerun::RecordingStream const& rec,
               pdlog::NavigationSample const& sample,
               std::string_view line,
               std::size_t line_number)
{
  rec.log("navigation/linear_speed", rerun::Scalars(sample.linear_speed));
  rec.log("navigation/angular_speed", rerun::Scalars(sample.angular_speed));

  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::setprecision(8) << "Status: " << sample.status
       << "\n\nLinear speed:  " << sample.linear_speed << " m/s"
       << "\nAngular speed: " << sample.angular_speed << " rad/s"
       << "\n\nSource line: " << line_number;
  auto const prefix_end = line.find('(');
  if(prefix_end != std::string_view::npos)
  {
    text << "\nLog time: " << line.substr(0, prefix_end);
  }
  // latest-at 查询让文本跟随时间游标，而不是始终显示整个文件的最后状态。
  rec.log("navigation/current", rerun::TextDocument(text.str()));

  auto color = rerun::Color(160, 170, 185);
  auto level = rerun::TextLogLevel::Info;
  if(sample.status == "Navigating")
  {
    color = rerun::Color(90, 165, 205);
  }
  else if(sample.status == "Escaping")
  {
    color = rerun::Color(230, 170, 65);
    level = rerun::TextLogLevel::Warning;
  }
  else if(sample.status == "Stuck" || sample.status == "FailEscape")
  {
    color = rerun::Color(220, 95, 95);
    level = rerun::TextLogLevel::Error;
  }
  // 保留每条状态及速度，而不只记录状态切换，以便核对原始日志。
  rec.log("navigation/events",
          rerun::TextLog(sample.status + " | " + std::string(line))
              .with_level(level)
              .with_color(color));
}

void
showRobotFootprint(rerun::RecordingStream const& rec,
                   pdlog::RobotFootprint const& footprint)
{
  std::vector<rerun::Position3D> outline;
  outline.reserve(footprint.vertices.size());
  for(auto const& vertex : footprint.vertices)
  {
    // foot_print 已包含 world 坐标及朝向，不再叠加 local_map.pos。
    // 略高于地图平面，避免轮廓和地图重叠闪烁。
    outline.emplace_back(vertex[0], vertex[1], 0.02F);
  }
  rec.log("world/robot/footprint",
          rerun::LineStrips3D(rerun::LineStrip3D(outline))
              .with_radii(rerun::Radius::ui_points(1.5F))
              .with_colors(rerun::Color(90, 165, 205)));
}

void
showLocalMap(rerun::RecordingStream const& rec,
             local_map_decoder::LocalMapData const& map,
             std::uint32_t width,
             std::uint32_t height,
             float resolution, // 米/格
             float origin_x,   // 地图左下角在 world 中的位置
             float origin_y,
             float origin_yaw = 0.0F, // 地图相对 world 的朝向，弧度
             bool rows_from_bottom = true)
{
  std::size_t const cell_count = static_cast<std::size_t>(width) * height;

  if(width == 0 || height == 0 || map.cells.size() != cell_count)
  {
    throw std::runtime_error("地图尺寸与栅格数量不匹配");
  }

  if(!std::isfinite(resolution) || resolution <= 0.0F)
  {
    throw std::runtime_error("地图分辨率必须为正数");
  }

  if(!std::isfinite(origin_x) || !std::isfinite(origin_y) ||
     !std::isfinite(origin_yaw))
  {
    throw std::runtime_error("地图位姿必须为有限数值");
  }

  // GridMap 的 colormap 只适用于单通道数据；RGBA 即使不设置 colormap，
  // 当前 Viewer 也会显示忽略色图的提示。使用 L/U8 + Costmap 消除该冲突。
  // 以下只是 0~3 类别的显示索引，不是原始地图的导航代价值。
  // Costmap 的 0 为透明，其余类别使用逐渐增强的颜色。
  static constexpr std::array<std::uint8_t, 4> display_values{0, 32, 192, 254};
  std::vector<std::uint8_t> pixels(cell_count);

  for(std::size_t row = 0; row < height; ++row)
  {
    // 图像缓冲区顶部行在前；地图位姿仍以左下角为原点。
    std::size_t const image_row = rows_from_bottom ? height - 1 - row : row;

    for(std::size_t col = 0; col < width; ++col)
    {
      auto const value = map.cells.at(row * width + col);
      pixels[image_row * width + col] = display_values.at(value);
    }
  }
  // 与机器人、轨迹共享 world 坐标系。Blueprint 的 Spatial2D/3D View
  // 可设置 origin="world"、contents="world/**"，无需单独的像素坐标视图。
  rec.log("world/local_map",
          rerun::archetypes::GridMap()
              .with_data(rerun::ImageBuffer(pixels))
              .with_format(
                  rerun::components::ImageFormat({width, height},
                                                 rerun::ColorModel::L,
                                                 rerun::ChannelDatatype::U8))
              .with_colormap(rerun::Colormap::Costmap)
              .with_cell_size(resolution)
              .with_opacity(0.85F)
              .with_draw_order(-10.0F)
              .with_translation({origin_x, origin_y, 0.0F})
              .with_quaternion(
                  rerun::Quaternion::from_xyzw(0.0F,
                                               0.0F,
                                               std::sin(origin_yaw * 0.5F),
                                               std::cos(origin_yaw * 0.5F))));
}

int
main(int argc, char const* const* argv)
{
  auto const usage = [](std::ostream& out)
  {
    out << "usage: local_map INPUT.pdlog [--save OUTPUT.rrd]\n"
           "  Default: spawn Rerun and replay maps, footprints, global paths "
           "and navigation.\n"
           "  --save: write a recording without opening the Viewer.\n";
  };
  std::filesystem::path input;
  std::optional<std::filesystem::path> output;
  for(int i = 1; i < argc; ++i)
  {
    std::string const arg = argv[i];
    if(arg == "--help")
    {
      usage(std::cout);
      return 0;
    }
    if(arg == "--save")
    {
      if(output || i + 1 >= argc || argv[i + 1][0] == '\0' ||
         argv[i + 1][0] == '-')
      {
        std::cerr << "local_map: --save requires one output path\n";
        return 2;
      }
      output = argv[++i];
    }
    else if(!arg.empty() && arg[0] != '-' && input.empty())
    {
      input = arg;
    }
    else
    {
      std::cerr << "local_map: unexpected argument: " << arg << '\n';
      usage(std::cerr);
      return 2;
    }
  }
  if(input.empty())
  {
    usage(std::cerr);
    return 2;
  }

  try
  {
    if(output && (std::filesystem::weakly_canonical(input) ==
                      std::filesystem::weakly_canonical(*output) ||
                  (std::filesystem::exists(*output) &&
                   std::filesystem::equivalent(input, *output))))
    {
      throw std::runtime_error("input and output must be different files");
    }
    // 与 pdlog 工具共用 PPMd8 文件头解析、流式解压和结束检查。
    pdlog::PdlogReader reader{input};
    rerun::RecordingStream const rec("local_map");
    if(output)
    {
      rec.save(output->string()).exit_on_failure();
    }
    else
    {
      rec.spawn().exit_on_failure();
    }
    rec.try_log_file_from_path(LOCAL_MAP_BLUEPRINT_PATH).exit_on_failure();
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    rec.log_static("navigation/linear_speed",
                   rerun::SeriesLines()
                       .with_names("Linear speed (m/s)")
                       .with_interpolation_mode(
                           rerun::components::InterpolationMode::StepAfter)
                       .with_colors(rerun::Color(90, 165, 205)));
    rec.log_static("navigation/angular_speed",
                   rerun::SeriesLines()
                       .with_names("Angular speed (rad/s)")
                       .with_interpolation_mode(
                           rerun::components::InterpolationMode::StepAfter)
                       .with_colors(rerun::Color(230, 170, 65)));

    std::size_t lines_read = 0;
    std::size_t skipped = 0;
    std::int64_t frames = 0;
    std::size_t unpaired = 0;
    std::size_t navigation_samples = 0;
    std::size_t path_updates = 0;
    pdlog::GlobalPathsAssembler path_assembler;
    pdlog::LogClock log_clock;
    std::optional<pdlog::RobotFootprint> pending_footprint;
    std::size_t footprint_line = 0;
    while(auto line = reader.next_line())
    {
      ++lines_read;
      auto const time_ms = log_clock.observe(line->text);
      // 所有记录共享真实日志时间和原始行号；frame 仅用于地图序号。
      auto const set_record_time = [&]()
      {
        rec.disable_timeline("frame");
        rec.set_time_sequence("log_line",
                              static_cast<std::int64_t>(line->line_number));
        if(time_ms)
        {
          rec.set_time_duration_nanos("robot_time", *time_ms * 1000000);
        }
        else
        {
          rec.disable_timeline("robot_time");
        }
      };
      // 路径和状态独立于地图更新，坏行也不能消费待配对轮廓。
      try
      {
        if(auto segment = pdlog::parseGlobalPathsLine(line->text))
        {
          if(segment->number == 0 && path_assembler.pending())
          {
            ++skipped;
            std::cerr << "local_map: " << input << ':' << line->line_number
                      << ": incomplete global_paths replaced by segment 0\n";
          }
          if(auto paths = path_assembler.push(std::move(*segment)))
          {
            set_record_time();
            showGlobalPaths(rec, *paths);
            ++path_updates;
            if(!time_ms)
              std::cerr << "local_map: " << input << ':' << line->line_number
                        << ": missing log timestamp; use log_line timeline\n";
          }
          continue;
        }
      }
      catch(std::exception const& error)
      {
        ++skipped;
        path_assembler.reset();
        std::cerr << "local_map: " << line->source_file << ':'
                  << line->line_number << ": " << error.what() << '\n';
        continue;
      }
      try
      {
        if(auto navigation = pdlog::parseNavigationLine(line->text))
        {
          set_record_time();
          showNavigation(rec, *navigation, line->text, line->line_number);
          ++navigation_samples;
          if(!time_ms)
          {
            std::cerr << "local_map: " << input << ':' << line->line_number
                      << ": missing log timestamp; use log_line timeline\n";
          }
          continue;
        }
      }
      catch(std::exception const& error)
      {
        ++skipped;
        std::cerr << "local_map: " << line->source_file << ':'
                  << line->line_number << ": " << error.what() << '\n';
        continue;
      }
      std::optional<pdlog::LocalMapFrame> frame;
      try
      {
        if(auto footprint = pdlog::parseFootprintLine(line->text))
        {
          if(pending_footprint)
          {
            ++unpaired;
            std::cerr << "local_map: " << input << ':' << footprint_line
                      << ": foot_print replaced before a matching local_map\n";
          }
          pending_footprint = std::move(footprint);
          footprint_line = line->line_number;
          continue;
        }
        frame = pdlog::parseLocalMapLine(line->text);
      }
      catch(std::exception const& error)
      {
        ++skipped;
        // 坏地图会消费对应轮廓，坏轮廓会使当前配对失效，避免串到下一帧。
        pending_footprint.reset();
        std::cerr << "local_map: " << line->source_file << ':'
                  << line->line_number << ": " << error.what() << '\n';
        continue;
      }
      if(!frame)
      {
        continue;
      }

      // 一个轮廓仅供随后的一张地图使用；中间的其他类型日志不影响配对。
      auto const footprint = std::exchange(pending_footprint, std::nullopt);
      // 地图和轮廓在同一帧更新，不能用 log_static 覆盖历史地图。
      set_record_time();
      rec.set_time_sequence("frame", frames);
      if(!time_ms)
      {
        std::cerr << "local_map: " << input << ':' << line->line_number
                  << ": missing log timestamp; use log_line timeline\n";
      }
      showLocalMap(rec,
                   frame->map,
                   frame->size,
                   frame->size,
                   frame->resolution,
                   frame->origin_x,
                   frame->origin_y);
      if(footprint)
      {
        showRobotFootprint(rec, *footprint);
      }
      else
      {
        // Rerun 默认沿用上一帧数据；缺失对应轮廓时必须显式清除旧轮廓。
        rec.log("world/robot/footprint", rerun::Clear::FLAT);
        ++unpaired;
        std::cerr << "local_map: " << input << ':' << line->line_number
                  << ": no preceding foot_print; showing map only\n";
      }
      ++frames;
    }
    if(pending_footprint)
    {
      ++unpaired;
      std::cerr << "local_map: " << input << ':' << footprint_line
                << ": foot_print has no following local_map\n";
    }
    if(path_assembler.pending())
    {
      ++skipped;
      std::cerr << "local_map: incomplete global_paths at end of file\n";
    }
    rec.flush_blocking().exit_on_failure();
    std::cerr << "local_map: read " << lines_read << " lines, logged " << frames
              << " frames, " << navigation_samples << " navigation samples, "
              << path_updates << " global_paths updates, skipped " << skipped
              << " invalid/incomplete records, " << unpaired
              << " unpaired records\n";
    if(frames == 0 && navigation_samples == 0 && path_updates == 0)
    {
      std::cerr << "local_map: no valid map, navigation or global_paths "
                   "records found\n";
    }
    return (frames > 0 || navigation_samples > 0 || path_updates > 0) &&
                   skipped == 0 && unpaired == 0
               ? 0
               : 1;
  }
  catch(std::exception const& error)
  {
    std::cerr << "local_map: " << error.what() << '\n';
    return 1;
  }
}
