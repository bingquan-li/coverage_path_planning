#include "coverage_path_planning/viz/viz.hh"


#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

std::sig_atomic_t volatile running = 1;

void
stop(int /*unused*/)
{
  running = 0;
}

void
publish_demo()
{
  using namespace std::chrono;
  constexpr double radius = 3.0;
  constexpr double angle_step = 2.0 * std::numbers::pi / 120.0;

  std::vector<Eigen::Vector3d> global_path;
  global_path.reserve(121);
  for(int i = 0; i <= 120; ++i)
  {
    auto const angle = static_cast<double>(i) * angle_step;
    global_path.emplace_back(radius * std::cos(angle),
                             radius * std::sin(angle),
                             0.0);
  }

  std::array<Eigen::Vector3d, 4> const body_corners{
      Eigen::Vector3d{0.4, 0.25, 0.0},
      Eigen::Vector3d{0.4, -0.25, 0.0},
      Eigen::Vector3d{-0.4, -0.25, 0.0},
      Eigen::Vector3d{-0.4, 0.25, 0.0},};

  std::vector<Eigen::Vector3d> local_path;
  local_path.reserve(21);
  // Run for about 30 seconds, or stop early with Ctrl-C/SIGTERM.
  for(int cycle = 0; cycle < 600 && running != 0; ++cycle)
  {
    auto const now =
        duration_cast<nanoseconds>(system_clock::now().time_since_epoch())
            .count();
    if(now < 0)
    {
      throw std::out_of_range("negative system timestamp");
    }
    planning_viz::DrawContext const ctx{.stamp_ns =
                                            static_cast<std::uint64_t>(now),
                                        .frame_id = "map",};
    auto const angle = static_cast<double>(cycle) * angle_step;
    auto const x = radius * std::cos(angle);
    auto const y = radius * std::sin(angle);
    auto const yaw = angle + std::numbers::pi / 2.0;

    std::array<Eigen::Vector3d, 4> footprint_in_map;
    for(std::size_t i = 0; i < body_corners.size(); ++i)
    {
      auto const& p = body_corners[i];
      footprint_in_map[i] =
          Eigen::Vector3d{x + std::cos(yaw) * p.x() - std::sin(yaw) * p.y(),
                          y + std::sin(yaw) * p.x() + std::cos(yaw) * p.y(),
                          0.0};
    }

    // Repeat the global path so clients connecting later can see it.
    planning_viz::global_path(global_path, ctx);
    // Briefly clear the local path during each period to demonstrate deletion.
    if(cycle % 120 >= 100)
    {
      planning_viz::clear_local_path(ctx);
    }
    else
    {
      local_path.clear();
      for(int i = 0; i <= 20; ++i)
      {
        auto const local_angle = angle + static_cast<double>(i) * angle_step;
        local_path.emplace_back(radius * std::cos(local_angle),
                                radius * std::sin(local_angle),
                                0.0);
      }
      planning_viz::local_path(local_path, ctx);
    }
    planning_viz::footprint(footprint_in_map, ctx);
    std::this_thread::sleep_for(50ms);
  }
}

} // namespace

int
main(int argc, char* argv[])
{
  if(argc > 2)
  {
    std::cerr << "Usage: " << argv[0] << " [new-recording.mcap]\n";
    return 1;
  }
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);

  planning_viz::Config config;
  if(argc == 2)
  {
    config.mcap_path = argv[1];
  }
  std::string error;
  if(!planning_viz::init(config, error))
  {
    std::cerr << "Failed to initialize visualization: " << error << '\n';
    return 1;
  }

  int result = 0;
  try
  {
    std::cout << "Connect Foxglove to ws://127.0.0.1:8765, use the map frame, "
                 "and enable the /planning/* topics in a 3D panel.\n"
                 "The demo exits after about 30 seconds or on Ctrl-C.\n";
    publish_demo();
  }
  catch(std::exception const& ex)
  {
    std::cerr << "Demo failed: " << ex.what() << '\n';
    result = 1;
  }

  // All publishing has finished. Explicit shutdown reports recording errors.
  if(!planning_viz::shutdown(error))
  {
    std::cerr << "Failed to shut down visualization: " << error << '\n';
    result = 1;
  }
  return result;
}
