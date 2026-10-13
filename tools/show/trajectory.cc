#include "trajectory.hh"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace show
{
namespace
{
constexpr double cell_m = 0.002;
constexpr double cad_distance_m = 0.198964015465;
constexpr double brush_rear_m = 0.115;
constexpr double brush_width_m = 0.300;
constexpr double brush_contact_m = 0.080;
constexpr double squeegee_width_m = 0.436420125;
constexpr double squeegee_rear_m = 0.161187875;
constexpr double suction_stroke_m = 0.008;
constexpr std::size_t min_cluster_cells = 625;

CleaningFootprint
local_cleaning_footprint(double expansion = 0.0)
{
  CleaningFootprint result;
  auto const half_width = brush_width_m / 2.0 + expansion;
  auto const half_contact = brush_contact_m / 2.0 + expansion;
  // Local robot coordinates: x forward, y left.
  result.cloth = {{-brush_rear_m - half_contact, -half_width, 0.0},
                  {-brush_rear_m - half_contact, half_width, 0.0},
                  {-brush_rear_m + half_contact, half_width, 0.0},
                  {-brush_rear_m + half_contact, -half_width, 0.0},};
  auto const half = squeegee_width_m / 2.0;
  auto const h = cad_distance_m - squeegee_rear_m;
  auto const radius = (half * half + h * h) / (2.0 * h);
  auto const samples =
      std::max(65, static_cast<int>(std::ceil(squeegee_width_m / 0.0015)) + 1);
  for(int i = 0; i < samples; ++i)
  {
    auto const u = -half + squeegee_width_m * i / (samples - 1);
    auto const z =
        -cad_distance_m + radius - std::sqrt(radius * radius - u * u);
    result.squeegee.emplace_back(z, u, 0.0);
  }
  return result;
}

Polygon
transform(Polygon const& local, RobotState const& state)
{
  Polygon result;
  result.reserve(local.size());
  auto const c = std::cos(state.yaw);
  auto const s = std::sin(state.yaw);
  for(auto const& p : local)
  {
    result.emplace_back(state.position.x() + c * p.x() - s * p.y(),
                        state.position.y() + s * p.x() + c * p.y(),
                        state.position.z());
  }
  return result;
}
} // namespace

CleaningFootprint
make_cleaning_footprint(RobotState const& state)
{
  static auto const local = local_cleaning_footprint();
  return {transform(local.cloth, state), transform(local.squeegee, state)};
}

struct WaterLeakSimulation::Impl
{
  double min_x{};
  double min_y{};
  double ground_z{};
  bool suction{};
  cv::Mat wet;
  cv::Mat covered;
  CleaningFootprint local =
      local_cleaning_footprint(cell_m * std::numbers::sqrt2 / 2.0);
  std::optional<Frame> previous;

  [[nodiscard]] std::vector<cv::Point>
  pixels(Polygon const& polygon, RobotState const& state) const
  {
    std::vector<cv::Point> result;
    for(auto const& p : transform(polygon, state))
    {
      // nearbyint, like numpy.rint, uses nearest-even rounding.
      result.emplace_back(
          static_cast<int>(std::nearbyint((p.x() - min_x) / cell_m - 0.5)),
          static_cast<int>(std::nearbyint((p.y() - min_y) / cell_m - 0.5)));
    }
    return result;
  }

  void
  paint(RobotState const& state)
  {
    auto const cloth = pixels(local.cloth, state);
    cv::fillConvexPoly(wet, cloth, cv::Scalar(1), cv::LINE_8);
    cv::fillConvexPoly(covered, cloth, cv::Scalar(1), cv::LINE_8);
    if(suction)
    {
      auto const arc = pixels(local.squeegee, state);
      auto const thickness = static_cast<int>(
          std::ceil((suction_stroke_m + cell_m * std::numbers::sqrt2) / cell_m));
      cv::polylines(wet, arc, false, cv::Scalar(0), thickness, cv::LINE_8);
    }
  }
};

WaterLeakSimulation::WaterLeakSimulation(std::span<Frame const> frames,
                                         bool suction)
  : impl_(std::make_unique<Impl>())
{
  if(frames.empty())
  {
    throw std::invalid_argument("water leak simulation requires a pose");
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = -min_x;
  for(auto const& frame : frames)
  {
    if(!frame.state.position.allFinite() || !std::isfinite(frame.state.yaw))
    {
      throw std::invalid_argument("non-finite cleaning pose");
    }
    min_x = std::min(min_x, frame.state.position.x());
    min_y = std::min(min_y, frame.state.position.y());
    max_x = std::max(max_x, frame.state.position.x());
    max_y = std::max(max_y, frame.state.position.y());
  }
  // Default mechanisms fit within 0.6 m, matching the Python reference grid.
  impl_->min_x = min_x - 0.6;
  impl_->min_y = min_y - 0.6;
  auto const width = std::ceil((max_x + 0.6 - impl_->min_x) / cell_m);
  auto const height = std::ceil((max_y + 0.6 - impl_->min_y) / cell_m);
  // Bound the two dense masks to 256 MiB; reject outlier coordinates before allocation.
  constexpr double max_cells = 128.0 * 1024.0 * 1024.0;
  if(!std::isfinite(width * height) || width * height > max_cells)
  {
    throw std::runtime_error("water leak grid exceeds 256 MiB; select a "
                             "shorter log or disable water_leak.enabled");
  }
  impl_->wet = cv::Mat::zeros(static_cast<int>(height),
                              static_cast<int>(width),
                              CV_8UC1);
  impl_->covered = cv::Mat::zeros(impl_->wet.size(), CV_8UC1);
  impl_->ground_z = frames.front().state.position.z();
  impl_->suction = suction;
}

WaterLeakSimulation::~WaterLeakSimulation() = default;

void
WaterLeakSimulation::reset()
{
  impl_->wet.setTo(0);
  impl_->covered.setTo(0);
  impl_->previous.reset();
}

void
WaterLeakSimulation::advance(Frame const& frame)
{
  if(!frame.state.position.allFinite() || !std::isfinite(frame.state.yaw))
  {
    throw std::invalid_argument("non-finite cleaning pose");
  }
  if(impl_->previous)
  {
    auto const& a = *impl_->previous;
    if(frame.log_time < a.log_time)
    {
      throw std::invalid_argument(
          "cleaning poses must be in time order; reset between replays");
    }
    auto const d = std::hypot(frame.state.position.x() - a.state.position.x(),
                              frame.state.position.y() - a.state.position.y());
    auto const yaw_delta = frame.state.yaw - a.state.yaw;
    auto const da = std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
    if(d <= 0.75 && frame.log_time - a.log_time <= std::chrono::seconds(4))
    {
      auto const count = std::max(
          1,
          static_cast<int>(std::ceil(std::max(d, std::abs(da) * 0.5) / 0.005)));
      for(int j = 1; j <= count; ++j)
      {
        auto const u = static_cast<double>(j) / count;
        RobotState state;
        state.position =
            a.state.position + (frame.state.position - a.state.position) * u;
        state.yaw = a.state.yaw + da * u;
        impl_->paint(state);
      }
    }
    else
    {
      // A log gap is not evidence that the robot cleaned the connecting segment.
      impl_->paint(frame.state);
    }
  }
  else
  {
    impl_->paint(frame.state);
  }
  impl_->previous = frame;
}

WaterLeakSnapshot
WaterLeakSimulation::snapshot() const
{
  WaterLeakSnapshot result;
  auto const edge = [&](int x0, int y0, int x1, int y1)
  {
    auto const z = impl_->ground_z + 0.006;
    result.wet_boundary_lines.emplace_back(impl_->min_x + x0 * cell_m,
                                           impl_->min_y + y0 * cell_m,
                                           z);
    result.wet_boundary_lines.emplace_back(impl_->min_x + x1 * cell_m,
                                           impl_->min_y + y1 * cell_m,
                                           z);
  };
  result.covered_area_m2 = cv::countNonZero(impl_->covered) * cell_m * cell_m;
  struct Run
  {
    int start;
    int end;
    std::size_t label;
    std::size_t rectangle;
  };
  std::vector<std::size_t> parent;
  std::vector<std::size_t> sizes;
  std::vector<cv::Rect> rectangles;
  auto const root = [&](std::size_t i)
  {
    while(parent[i] != i)
    {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  };
  std::vector<Run> previous;
  std::size_t wet_cells = 0;
  for(int y = 0; y < impl_->wet.rows; ++y)
  {
    auto const* row = impl_->wet.ptr<unsigned char>(y);
    std::vector<Run> current;
    std::size_t first_overlap = 0;
    for(int x = 0; x < impl_->wet.cols;)
    {
      if(row[x] == 0)
      {
        ++x;
        continue;
      }
      auto const start = x;
      while(x < impl_->wet.cols && row[x] != 0)
      {
        ++x;
      }
      // Draw only exposed cell edges, including hole boundaries. Never outline
      // the internal rectangles used to compress the triangle mesh.
      edge(start, y, start, y + 1);
      edge(x, y, x, y + 1);
      for(auto const neighbor_y : {y - 1, y + 1})
      {
        auto const* neighbor = neighbor_y >= 0 && neighbor_y < impl_->wet.rows
                                   ? impl_->wet.ptr<unsigned char>(neighbor_y)
                                   : nullptr;
        auto const boundary_y = neighbor_y < y ? y : y + 1;
        for(int column = start; column < x;)
        {
          if(neighbor != nullptr && neighbor[column] != 0)
          {
            ++column;
            continue;
          }
          auto const first = column;
          while(column < x && (neighbor == nullptr || neighbor[column] == 0))
          {
            ++column;
          }
          edge(first, boundary_y, column, boundary_y);
        }
      }
      auto const size = static_cast<std::size_t>(x - start);
      wet_cells += size;
      auto label = parent.size();
      parent.push_back(label);
      sizes.push_back(size);
      auto rectangle = rectangles.size();
      while(first_overlap < previous.size() &&
            previous[first_overlap].end <= start)
      {
        ++first_overlap;
      }
      for(auto k = first_overlap; k < previous.size() && previous[k].start < x;
          ++k)
      {
        auto a = root(label);
        auto b = root(previous[k].label);
        if(a != b)
        {
          if(sizes[a] < sizes[b])
          {
            std::swap(a, b);
          }
          parent[b] = a;
          sizes[a] += sizes[b];
          label = a;
        }
        if(previous[k].start == start && previous[k].end == x)
        {
          rectangle = previous[k].rectangle;
        }
      }
      if(rectangle == rectangles.size())
      {
        rectangles.emplace_back(start, y, x - start, 1);
      }
      else
      {
        ++rectangles[rectangle].height;
      }
      current.push_back({start, x, label, rectangle});
    }
    previous = std::move(current);
  }
  result.wet_area_m2 = wet_cells * cell_m * cell_m;
  for(std::size_t i = 0; i < parent.size(); ++i)
  {
    if(parent[i] == i && sizes[i] >= min_cluster_cells)
    {
      ++result.clusters;
    }
  }
  result.wet_triangles.reserve(rectangles.size() * 6);
  for(auto const& rect : rectangles)
  {
    auto const x0 = impl_->min_x + rect.x * cell_m;
    auto const x1 = x0 + rect.width * cell_m;
    auto const y0 = impl_->min_y + rect.y * cell_m;
    auto const y1 = y0 + rect.height * cell_m;
    auto const z = impl_->ground_z + 0.005;
    result.wet_triangles.insert(result.wet_triangles.end(),
                                {{x0, y0, z},
                                 {x1, y0, z},
                                 {x1, y1, z},
                                 {x0, y0, z},
                                 {x1, y1, z},
                                 {x0, y1, z},});
  }
  return result;
}

Polygon
make_footprint(RobotState const& state,
               coverage_path_planning::Footprint const& footprint)
{
  Polygon vertices;
  vertices.reserve(footprint.points.size());
  auto const c = std::cos(state.yaw);
  auto const s = std::sin(state.yaw);
  for(auto const& point : footprint.points)
  {
    vertices.emplace_back(state.position.x() + c * point.x() - s * point.y(),
                          state.position.y() + s * point.x() + c * point.y(),
                          state.position.z());
  }
  return vertices;
}

// Select real log poses, using distance and accumulated heading change together.
// This also samples in-place turns and handles the -pi/pi yaw boundary.
std::vector<Polygon>
build_trajectory(std::span<Frame const> frames,
                 coverage_path_planning::Footprint const& footprint,
                 double distance_step_m,
                 double yaw_step_deg)
{
  std::vector<Polygon> polygons;
  if(frames.empty())
  {
    return polygons;
  }
  auto const yaw_step = yaw_step_deg * std::numbers::pi / 180.0;

  polygons.push_back(make_footprint(frames.front().state, footprint));
  double progress = 0.0;
  std::size_t last_selected = 0;
  for(std::size_t i = 1; i < frames.size(); ++i)
  {
    auto const& previous = frames[i - 1].state;
    auto const& current = frames[i].state;
    auto const distance = (current.position - previous.position).norm();
    auto const angle = std::abs(
        std::remainder(current.yaw - previous.yaw, 2.0 * std::numbers::pi));
    progress += distance / distance_step_m + angle / yaw_step;
    if(progress >= 1.0)
    {
      polygons.push_back(make_footprint(current, footprint));
      last_selected = i;
      progress = 0.0;
    }
  }
  auto const& last = frames.back().state;
  auto const& selected = frames[last_selected].state;
  if((last.position - selected.position).norm() > 1e-9 ||
     std::abs(std::remainder(last.yaw - selected.yaw, 2.0 * std::numbers::pi)) >
         1e-9)
  {
    polygons.push_back(make_footprint(last, footprint));
  }
  return polygons;
}

} // namespace show
