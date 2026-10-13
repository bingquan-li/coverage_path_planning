#include "local_map_log.hh"
#include "log_reader.hh"

#include <rerun.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
           "  Default: spawn Rerun and replay local_map frames.\n"
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

    std::size_t lines_read = 0;
    std::size_t skipped = 0;
    std::int64_t frames = 0;
    while(auto line = reader.next_line())
    {
      ++lines_read;
      std::optional<pdlog::LocalMapFrame> frame;
      try
      {
        frame = pdlog::parseLocalMapLine(line->text);
      }
      catch(std::exception const& error)
      {
        ++skipped;
        std::cerr << "local_map: " << line->source_file << ':'
                  << line->line_number << ": " << error.what() << '\n';
        continue;
      }
      if(!frame)
      {
        continue;
      }

      // 地图和机器人位置在同一帧更新，不能用 log_static 覆盖历史地图。
      rec.set_time_sequence("frame", frames);
      rec.set_time_sequence("log_line",
                            static_cast<std::int64_t>(line->line_number));
      showLocalMap(rec,
                   frame->map,
                   frame->size,
                   frame->size,
                   frame->resolution,
                   frame->origin_x,
                   frame->origin_y);
      rec.log("world/robot/position",
              rerun::Points3D({{frame->center_x, frame->center_y, 0.2F}})
                  .with_radii({0.15F})
                  .with_colors({rerun::Color(90, 165, 205)}));
      ++frames;
    }
    rec.flush_blocking().exit_on_failure();
    std::cerr << "local_map: read " << lines_read << " lines, logged " << frames
              << " frames, skipped " << skipped
              << " invalid local_map records\n";
    if(frames == 0)
    {
      std::cerr << "local_map: no valid local_map frames found\n";
    }
    return frames > 0 && skipped == 0 ? 0 : 1;
  }
  catch(std::exception const& error)
  {
    std::cerr << "local_map: " << error.what() << '\n';
    return 1;
  }
}
