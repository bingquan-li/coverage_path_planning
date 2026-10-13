#include "coverage_path_planning/viz/viz.hh"

#include "coverage_path_planning/viz/backend.hh"

namespace planning_viz
{

namespace
{

detail::Backend&
backend()
{
  static detail::Backend instance;
  return instance;
}

} // namespace

bool
init(Config const& config, std::string& error)
{
  return backend().init(config, error);
}

bool
shutdown(std::string& error)
{
  return backend().shutdown(error);
}

void
global_path(Points3 points, DrawContext ctx)
{
  backend().global_path(points, ctx);
}

void
local_path(Points3 points, DrawContext ctx)
{
  backend().local_path(points, ctx);
}

void
history_path(Points3 segments, DrawContext ctx)
{
  backend().history_path(segments, ctx);
}

void
trajectory(Points3 points, DrawContext ctx)
{
  backend().trajectory(points, ctx);
}

void
trajectory_footprints(std::span<Points3 const> polygons, DrawContext ctx)
{
  backend().trajectory_footprints(polygons, ctx);
}

void
footprint(Points3 vertices, DrawContext ctx)
{
  backend().footprint(vertices, ctx);
}

void
water_leak(WaterLeakView const& view, DrawContext ctx)
{
  backend().water_leak(view, ctx);
}

void
odometry(RobotOdometry const& state, DrawContext ctx)
{
  backend().odometry(state, ctx);
}

void
clear_local_path(DrawContext ctx)
{
  backend().clear_local_path(ctx);
}

} // namespace planning_viz
