#include "../tools/show/trajectory.hh"
#include "../tools/show/replay.hh"
#include "coverage_path_planning/viz/converters.hh"

#include <cmath>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace
{
void
check(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

show::Frame
pose(int milliseconds, double x, double y, double yaw = 0.0)
{
  show::Frame frame;
  frame.log_time = std::chrono::milliseconds(milliseconds);
  frame.state.position = {x, y, 0.0};
  frame.state.yaw = yaw;
  return frame;
}

double
mesh_area(show::Polygon const& mesh)
{
  check(mesh.size() % 3 == 0, "incomplete triangle");
  double area = 0.0;
  for(std::size_t i = 0; i < mesh.size(); i += 3)
  {
    Eigen::Vector2d const a = (mesh[i + 1] - mesh[i]).head<2>();
    Eigen::Vector2d const b = (mesh[i + 2] - mesh[i]).head<2>();
    auto const cross = a.x() * b.y() - a.y() * b.x();
    check(cross > 0.0, "water triangles must face +Z");
    area += cross / 2.0;
  }
  return area;
}

void
check_snapshot(show::WaterLeakSnapshot const& snapshot)
{
  check(std::abs(mesh_area(snapshot.wet_triangles) - snapshot.wet_area_m2) <
            1e-9,
        "mesh must preserve wet cells exactly, without overlapping rectangles");
  check(snapshot.wet_area_m2 <= snapshot.covered_area_m2,
        "wet exceeds covered");
}

void
unit_tests()
{
  std::vector<show::Frame> frames{pose(0, 0, 0),
                                  pose(1000, .5, 0),
                                  pose(2000, 1, 0),};
  show::WaterLeakSimulation sim(frames);
  check(sim.snapshot().wet_area_m2 == 0.0,
        "future poses must not deposit water");
  sim.advance(frames.front());
  auto const first = sim.snapshot();
  check(std::abs(first.wet_area_m2 - .026144) < 1e-12,
        "stationary reference area");
  sim.advance(frames[1]);
  sim.advance(frames[2]);
  auto const forward = sim.snapshot();
  check_snapshot(forward);
  check(std::abs(forward.wet_area_m2 - .034648) < 1e-12,
        "forward reference wet area");
  check(std::abs(forward.covered_area_m2 - .330144) < 1e-12,
        "forward reference covered area");
  check(forward.clusters == 1, "forward trailing water is one cluster");
  // A cleaned point inside the swept strip must not be filled by an enclosing polygon.
  for(std::size_t i = 0; i < forward.wet_triangles.size(); i += 6)
  {
    auto const& low = forward.wet_triangles[i];
    auto const& high = forward.wet_triangles[i + 2];
    check(low.x() >= .25 || high.x() <= .25 || low.y() >= 0 || high.y() <= 0,
          "mesh incorrectly fills a dry part of the path");
  }
  sim.reset();
  check(sim.snapshot().wet_triangles.empty(),
        "reset must clear previous water");
  sim.advance(frames.front());
  check(sim.snapshot().wet_area_m2 == first.wet_area_m2,
        "loop must restart from the first frame");
  show::WaterLeakSimulation no_suction(frames, false);
  for(auto const& f : frames)
  {
    no_suction.advance(f);
  }
  auto const all_wet = no_suction.snapshot();
  check_snapshot(all_wet);
  check(all_wet.leak_rate() == 100.0,
        "suction off must leave all covered cells wet");

  std::vector<show::Frame> gap{pose(0, 0, 0), pose(1000, 2, 0)};
  show::WaterLeakSimulation gap_sim(gap, false);
  for(auto const& f : gap)
  {
    gap_sim.advance(f);
  }
  check(gap_sim.snapshot().clusters == 2,
        "distance gaps must not connect water regions");
  gap = {pose(0, 0, 0), pose(5000, .5, 0)};
  show::WaterLeakSimulation time_gap(gap, false);
  for(auto const& f : gap)
  {
    time_gap.advance(f);
  }
  check(time_gap.snapshot().clusters == 2,
        "time gaps must not connect water regions");

  auto const cleaning =
      show::make_cleaning_footprint(pose(0, 1, 2, std::numbers::pi / 2).state);
  auto const center = (cleaning.cloth[0] + cleaning.cloth[2]) / 2.0;
  check(std::abs(center.x() - 1) < 1e-12 &&
            std::abs(center.y() - 1.885) < 1e-12,
        "cloth uses the wheel axle and heading");
  check(std::abs(cleaning.squeegee.front().y() - (2 - .161187875)) < 1e-12,
        "squeegee endpoint must match CAD");
  planning_viz::WaterLeakView const view{cleaning.cloth,
                                   cleaning.squeegee,
                                   forward.wet_triangles,
                                   forward.wet_boundary_lines,};
  auto const scene =
      planning_viz::detail::make_water_leak_update(view, "map", 1);
  check(scene.entities.size() == 3,
        "must render cloth, squeegee and wet regions");
  check(scene.entities[0].lines[0].type ==
            foxglove::messages::LinePrimitive::LineType::LINE_LOOP,
        "cloth must be closed");
  check(scene.entities[1].lines[0].type ==
            foxglove::messages::LinePrimitive::LineType::LINE_STRIP,
        "squeegee must not have a closing chord");
  check(scene.entities[2].triangles[0].points.size() ==
            forward.wet_triangles.size(),
        "mesh publication");
  check(scene.entities[2].lines[0].points.size() ==
            forward.wet_boundary_lines.size(),
        "boundary publication");
  for(auto const& entity : scene.entities)
  {
    check(entity.texts.empty(),
          "water leak visualization must not show statistics text");
  }
  auto const empty = planning_viz::detail::make_water_leak_update({} "map", 2);
  check(empty.entities.empty() && empty.deletions.size() == 3,
        "empty state clears all entities");

  bool rejected = false;
  try
  {
    std::vector<show::Frame> huge{pose(0, 0, 0), pose(1, 1e10, 1e10)};
    show::WaterLeakSimulation const invalid(huge);
  }
  catch(std::runtime_error const&)
  {
    rejected = true;
  }
  check(rejected, "outlier bounds must be rejected before allocating the grid");
}
} // namespace

int
main(int argc, char* const* argv)
{
  try
  {
    if(argc == 2 && std::string(argv[1]) == "--stdin")
    {
      std::size_t count = 0;
      bool suction = false;
      std::cin >> count >> suction;
      std::vector<show::Frame> frames;
      for(std::size_t i = 0; i < count; ++i)
      {
        int t = 0;
        double x = NAN;
        double y = NAN;
        double yaw = NAN;
        std::cin >> t >> x >> y >> yaw;
        frames.push_back(pose(t, x, y, yaw));
      }
      show::WaterLeakSimulation sim(frames, suction);
      for(auto const& f : frames)
      {
        sim.advance(f);
      }
      auto const snapshot = sim.snapshot();
      check_snapshot(snapshot);
      std::cout << std::setprecision(17) << snapshot.wet_area_m2 << ' '
                << snapshot.covered_area_m2 << ' ' << snapshot.clusters << '\n';
      return 0;
    }
    unit_tests();
    if(argc == 3 && (std::string(argv[1]) == "--record" ||
                     std::string(argv[1]) == "--record-final"))
    {
      show::Options options;
      options.input = std::string(argv[2]) + ".log";
      check(!std::filesystem::exists(options.input),
            "fixture path already exists");
      std::ofstream log(options.input);
      for(int i = 0; i < 6; ++i)
      {
        log << "0:12:00:0" << i << " 000 current pos " << i * .1
            << " 0 0 yaw 0\n";
      }
      log.close();
      options.mcap_path = argv[2];
      options.frame_id = "map";
      options.frame_rate_hz = 1000;
      options.final_only = std::string(argv[1]) == "--record-final";
      options.show_trajectory = true;
      coverage_path_planning::Footprint footprint;
      footprint.points = {{-.2, -.2}, {.3, -.2}, {.3, .2}, {-.2, .2}};
      check(show::run(options, footprint) == 0, "file replay failed");
    }
    std::cout << "Water leak geometry, temporal state, regions and publication "
                 "checks passed\n";
  }
  catch(std::exception const& ex)
  {
    std::cerr << ex.what() << '\n';
    return 1;
  }
}
