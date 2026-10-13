#include "replay.hh"

#include "log_parser.hh"
#include "trajectory.hh"
#include "coverage_path_planning/common/log/log.hh"
#include "coverage_path_planning/viz/viz.hh"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace show
{
namespace
{
namespace log = coverage_path_planning::log;
using namespace std::chrono_literals;

std::sig_atomic_t volatile running = 1;

void
stop(int /*unused*/)
{
  running = 0;
}

void
sleep_until(std::chrono::steady_clock::time_point target)
{
  while((running != 0) && std::chrono::steady_clock::now() < target)
  {
    auto const remaining = target - std::chrono::steady_clock::now();
    std::this_thread::sleep_for(
        std::min(remaining, std::chrono::steady_clock::duration(50ms)));
  }
}

void
publish_robot(Frame const& frame,
              coverage_path_planning::Footprint const& footprint,
              planning_viz::DrawContext ctx)
{
  planning_viz::RobotOdometry const odom{
      .position = frame.state.position,
      .yaw = frame.state.yaw,
      .linear_velocity = frame.state.linear_velocity,
      .angular_velocity = frame.state.angular_velocity,};
  planning_viz::odometry(odom, ctx);

  auto const vertices = make_footprint(frame.state, footprint);
  planning_viz::footprint(vertices, ctx);
}

// 负责把整份日志读入内存、生成静态轨迹，然后按设定节奏逐帧发布机器人状态。
void
replay(Options const& options,
       coverage_path_planning::Footprint const& footprint)
{
  if(footprint.points.size() < 3)
  {
    throw std::runtime_error("robot footprint requires at least three points");
  }
  LogParser parser(options.input);
  std::vector<Frame> frames;
  Frame frame;
  while(running != 0 && parser.next_frame(frame))
  {
    frames.push_back(frame);
  }
  if(running == 0)
  {
    return;
  }
  if(frames.empty())
  {
    throw std::runtime_error("log contains no valid pose records");
  }
  std::optional<WaterLeakSimulation> water;
  if(options.water_leak_enabled)
  {
    water.emplace(frames, options.suction);
  }
  // 根据所有位姿和机器人的轮廓生成一组多边形
  auto const polygons = build_trajectory(frames,
                                         footprint,
                                         options.footprint_spacing_m,
                                         options.footprint_yaw_step_deg);
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(frames.size());

  for(auto const& frame : frames)
  {
    positions.push_back(frame.state.position);
  }

  // 一条路径，所以外层只有一个 Points3。
  planning_viz::Points3 path_views{planning_viz::Points3{positions}};

  std::vector<planning_viz::Points3> polygon_views;
  polygon_views.reserve(polygons.size());
  for(auto const& polygon : polygons)
  {
    polygon_views.emplace_back(polygon);
  }
  log::info("Full trajectory: {} log poses, {} robot polygons",
            frames.size(),
            polygons.size());

  std::uint64_t last_stamp = 0;
  auto const next_context = [&]
  {
    auto const now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    last_stamp = std::max(last_stamp + 1, static_cast<std::uint64_t>(now));
    return planning_viz::DrawContext{.stamp_ns = last_stamp,
                                     .frame_id = options.frame_id,};
  };
  auto const publish_trail = [&](planning_viz::DrawContext ctx)
  {
    if(options.show_footprint)
    {
      planning_viz::trajectory_footprints(polygon_views, ctx);
    }
    if(options.show_trajectory && positions.size() >= 2)
    {
      planning_viz::history_path(path_views, ctx);
    }
  };
  auto const publish_water = [&](CleaningFootprint const& cleaning,
                                 WaterLeakSnapshot const& snapshot,
                                 planning_viz::DrawContext ctx)
  {
    planning_viz::water_leak({.cloth = cleaning.cloth,
                              .squeegee = cleaning.squeegee,
                              .wet_triangles = snapshot.wet_triangles,
                              .wet_boundary_lines = snapshot.wet_boundary_lines,
                              .suction = options.suction,},
                             ctx);
  };

  if(options.final_only)
  {
    // Compute every pose in order without playback delays or intermediate scenes.
    // Keep the resulting geometry for late subscribers; do not re-simulate it.
    WaterLeakSnapshot snapshot;
    if(water)
    {
      for(auto const& pose : frames)
      {
        if(running == 0)
        {
          return;
        }
        water->advance(pose);
      }
      if(running == 0)
      {
        return;
      }
      snapshot = water->snapshot();
    }
    auto const cleaning = make_cleaning_footprint(frames.back().state);
    log::info("Final result ready; publishing the complete trajectory and wet "
              "regions");
    while(running != 0)
    {
      auto const ctx = next_context();
      publish_robot(frames.back(), footprint, ctx);
      publish_trail(ctx);
      if(water)
      {
        publish_water(cleaning, snapshot, ctx);
      }
      if(!options.loop)
      {
        break;
      }
      sleep_until(std::chrono::steady_clock::now() + 1s);
    }
    return;
  }

  // `frame_interval` 是两帧之间的最短间隔。例如帧率为 10 Hz 时，间隔是 100 ms。
  auto const frame_interval =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / options.frame_rate_hz));
  auto next_frame_time = std::chrono::steady_clock::now();
  auto next_trajectory_time = next_frame_time;
  std::size_t cycle = 0;
  // 播放整个数据
  do
  {
    if(water)
    {
      water->reset();
    }
    auto const first_time = frames.front().log_time;
    auto const start = std::chrono::steady_clock::now();
    std::size_t poses = 0;
    std::size_t velocities = 0;
    // 播放当前所有的数据
    for(auto const& frame : frames)
    {
      if(running == 0)
      {
        break;
      }
      auto target = next_frame_time;
      // 如果 `speed > 0`，还会根据帧的日志时间计算它相对第一帧的播放偏移：
      if(options.speed > 0.0)
      {
        auto const elapsed = std::chrono::duration<double, std::milli>(
                                 frame.log_time - first_time) /
                             options.speed;
        target = std::max(
            target,
            start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    elapsed));
      }
      sleep_until(target);
      if(running == 0)
      {
        break;
      }
      auto const ctx = next_context();
      // - 每帧都会调用 `publish_robot()`，更新当前机器人的 odometry 和 footprint。同一实体被新位姿替换，所以 Foxglove 里显示的是移动中的机器人。
      publish_robot(frame, footprint, ctx);
      if(water)
      {
        water->advance(frame);
        auto const cleaning = make_cleaning_footprint(frame.state);
        auto const snapshot = water->snapshot();
        publish_water(cleaning, snapshot, ctx);
      }
      ++poses;
      velocities += frame.velocity_count;
      // Publish the entire static trail periodically so late subscribers see it.
      // 这里是每一帧重新绘制所有的 polygon_footprints
      // - 完整轨迹每帧都会检查是否该发布，但只有到 `next_trajectory_time` 时才调用 `trajectory_footprints()`：**每轮第一次立即调用，之后约每秒一次**。
      if(std::chrono::steady_clock::now() >= next_trajectory_time)
      {
        publish_trail(ctx);
        next_trajectory_time = std::chrono::steady_clock::now() + 1s;
      }
      next_frame_time = std::chrono::steady_clock::now() + frame_interval;
    }
    if(poses > 0)
    {
      ++cycle;
      log::info("Replay cycle {}: {} poses, {} velocity records",
                cycle,
                poses,
                velocities);
    }
  }
  // 循环结束后，如果 `loop` 为真且没有收到停止信号，就从头再播一遍。
  while(running != 0 && options.loop);
}
} // namespace

int
run(Options const& options, coverage_path_planning::Footprint const& footprint)
{
  try
  {
    running = 1;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    planning_viz::Config config;
    config.host = options.host;
    config.port = options.port;
    config.default_frame = options.frame_id;
    config.websocket_enabled = options.websocket;
    config.mcap_path = options.mcap_path;
    if(options.mcap_path)
    {
      auto const parent =
          std::filesystem::path(*options.mcap_path).parent_path();
      if(!parent.empty())
      {
        std::filesystem::create_directories(parent);
      }
    }
    std::string error;
    if(!planning_viz::init(config, error))
    {
      throw std::runtime_error("visualization initialization failed: " + error);
    }
    int result = 0;
    try
    {
      if(options.websocket)
      {
        log::info("Connect Foxglove to ws://{}:{}; in the 3D panel set "
                  "Fixed frame={}, Display frame={}, enable "
                  "/tf, /planning/footprint, /planning/trajectory and "
                  "/planning/water_leak; "
                  "plot velocity from /planning/odometry",
                  options.host,
                  options.port,
                  options.frame_id,
                  options.frame_id);
      }
      if(options.start_delay > 0.0 && options.websocket)
      {
        sleep_until(
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(options.start_delay)));
      }
      if(running != 0)
      {
        replay(options, footprint);
      }
    }
    catch(std::exception const& ex)
    {
      log::error("Replay failed: {}", ex.what());
      result = 1;
    }
    if(!planning_viz::shutdown(error))
    {
      log::error("Visualization shutdown failed: {}", error);
      result = 1;
    }
    return result;
  }
  catch(std::exception const& ex)
  {
    log::error("show: {}", ex.what());
    return 1;
  }
}

} // namespace show
