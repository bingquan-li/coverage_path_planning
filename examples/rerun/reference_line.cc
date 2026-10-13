#include <cmath>

#include <numbers>
#ifndef REFERENCE_LINE_GEOMETRY_ONLY
#include <rerun.hpp>
#endif
#include <array>
#include <utility>


#include <algorithm>
#include <stdexcept>
#include <vector>


constexpr double pi = std::numbers::pi;

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Geometry
{
  std::vector<double> s;         // Accumulated polyline length [m].
  std::vector<double> curvature; // Signed curvature [1/m]. Left positive.
  std::vector<bool>
      sharp_corner; // Discrete turns >= 30 degrees use offset joins.
};

// y = amplitude * sin(2*pi*x / wavelength).
// x_length is the x-axis span, NOT the arc length.
// Uniform x sampling gives nonuniform arc-length sampling.
std::vector<Point2D>
generateSinePath2(double x_length = 16.0,
                  double amplitude = 0.8,
                  double wavelength = 4.0,
                  double max_dx = 0.02)
{
  if(!std::isfinite(x_length) || !(x_length > 0.0) ||
     !std::isfinite(amplitude) || !std::isfinite(wavelength) ||
     !(wavelength > 0.0) || !std::isfinite(max_dx) || !(max_dx > 0.0))
  {
    throw std::invalid_argument("Invalid sine-path parameters");
  }
  double const count = std::max(2.0, std::ceil(x_length / max_dx));
  if(!std::isfinite(count) || count > 10000000.0)
  {
    throw std::invalid_argument("Too many path samples");
  }
  auto const segments = static_cast<std::size_t>(count);
  std::vector<Point2D> points;
  points.reserve(segments + 1);
  double const omega = 2.0 * pi / wavelength;
  for(std::size_t i = 0; i <= segments; ++i)
  {
    double const x = x_length * (static_cast<double>(i) / segments);
    points.push_back({x, amplitude * std::sin(omega * x)});
  }
  return points;
}


// All distances are in metres. +x right, +y up; the corner turns LEFT.
// Open path: straight -> quarter circle -> straight -> S-shaped section.
// Joins have continuous tangent. Straight/circle joins have curvature jumps.
// s_length is the forward span of the S-section, NOT its arc length.
// max_ds bounds the distance between adjacent samples.
inline std::vector<Point2D>
generateSinePath(double first_straight = 3.0,
                 double corner_radius = 0.6,
                 double second_straight = 3.0,
                 double s_length = 8.0,
                 double s_amplitude = 0.8,
                 double max_ds = 0.02)
{
  if(!std::ranges::all_of(std::array{first_straight,
                                     corner_radius,
                                     second_straight,
                                     s_length,
                                     max_ds,},
                          [](double data)
                          { return std::isfinite(data) && data > 0; }))
  {
    throw std::invalid_argument("Lengths, radius and max_ds must be positive");
  }

  // s_amplitude 就是振幅 A
  if(!std::isfinite(s_amplitude))
  {
    throw std::invalid_argument("Invalid S amplitude");
  }

  // Conservative arc-length bound for the S-section sampling.
  // S-截面采样的保守弧长界。
  // 前进 s_length 米，正弦相位变化 2π，完成一个周期。
  // omega 表示每前进 1 米，相位变化多少弧度。
  // 这里保证的是 弧长 一定会变化一个周期
  double const omega = 2.0 * pi / s_length;
  // 用于确定采样数量
  double const s_speed_bound =
      std::hypot(1.0, 3.0 * std::abs(s_amplitude) * omega);

  auto const segmentCount = [max_ds](double length_bound)
  {
    double const count = std::max(1.0, std::ceil(length_bound / max_ds));
    if(!std::isfinite(count) || count > 2500000.0)
    {
      throw std::invalid_argument("Too many samples");
    }
    return static_cast<std::size_t>(count);
  };

  auto const n1 = segmentCount(first_straight);
  auto const n2 = segmentCount(2 * pi * corner_radius / 2.0);
  auto const n3 = segmentCount(second_straight);
  auto const n4 = segmentCount(s_length * s_speed_bound);

  std::vector<Point2D> points;
  points.reserve(1 + n1 + n2 + n3 + n4);
  points.push_back({0.0, 0.0});

  // 1. Straight along +x.
  for(std::size_t i = 1; i <= n1; ++i)
  {
    points.push_back({first_straight * (static_cast<double>(i) / n1), 0.0});
  }

  // 2. Quarter circle: heading +x -> +y, curvature +1/radius.
  for(std::size_t i = 1; i <= n2; ++i)
  {
    double const angle = 0.5 * pi * (0.01 * static_cast<double>(i));
    points.push_back({first_straight + corner_radius * std::sin(angle),
                      corner_radius * (1.0 - std::cos(angle)),});
  }
  // double const x0 = first_straight + corner_radius;
  // points.back() = {x0, corner_radius};
  double x0 = points.back().x;
  double y0 = points.back().y;
  // 3. Straight along +y.
  for(std::size_t i = 1; i <= n3; ++i)
  {
    points.push_back({x0, y0 + second_straight * (static_cast<double>(i) / n3)});
  }

  // 4. S-section in local (forward, left) coordinates.
  // lateral(u) = amplitude * sin(2*pi*u / length)^3.
  // At both endpoints lateral, lateral' and lateral'' are zero.
  // Positive/negative lateral lobes give alternating left/right turns.
  x0 = points.back().x;
  y0 = points.back().y;
  for(std::size_t i = 1; i <= n4; ++i)
  {
    double const t = static_cast<double>(i) / n4;
    double const sine = std::sin(2.0 * pi * t);
    double const lateral = s_amplitude * sine * sine * sine;
    points.push_back({x0 - lateral, y0 + s_length * t});
  }
  points.back() = {x0, y0 + s_length};
  return points;
}


// For an OPEN path in Cartesian coordinates (x right, y up).
// Three-point signed circumcircle curvature, valid for nonuniform spacing.
// Empty input returns empty arrays. One/two points have zero curvature.
// Endpoint curvature copies the nearest interior estimate (an approximation).
// Reject repeated/near-repeated adjacent points and degenerate triples.
Geometry
compute_geometry(std::vector<Point2D> const& points, double min_distance = 1e-9)
{
  if(!std::isfinite(min_distance) || !(min_distance > 0.0))
  {
    throw std::invalid_argument("Invalid minimum point distance");
  }
  for(auto const& p : points)
  {
    if(!std::isfinite(p.x) || !std::isfinite(p.y))
    {
      throw std::invalid_argument("Nonfinite point coordinate");
    }
  }

  std::size_t const n = points.size();
  Geometry result{std::vector<double>(n, 0.0),
                  std::vector<double>(n, 0.0),
                  std::vector<bool>(n, false),};

  for(std::size_t i = 1; i < n; ++i)
  {
    double const ds = std::hypot(points[i].x - points[i - 1].x,
                                 points[i].y - points[i - 1].y);
    if(!std::isfinite(ds) || ds <= min_distance)
    {
      throw std::invalid_argument("Repeated/invalid adjacent points");
    }
    result.s[i] = result.s[i - 1] + ds;
    if(!std::isfinite(result.s[i]))
    {
      throw std::invalid_argument("Arc-length overflow");
    }
  }

  // 两个点不考虑曲率
  if(n < 3)
  {
    return result;
  }

  // 带符号的 Menger 曲率, 三点法估计曲率: 用相连的三个点,构造一个圆,用这个圆的半径的倒数计算曲率
  // 三角形的面积: S = abc / (4*R)
  // 二维叉积可以计算三角形的面积: 2S = abs(axb) = abs(ax*by - ay*bx)
  // k = 1/R = 2 * S / abc = 2 * (axby - aybx) / abc
  //           但是对其进行一个单位化计算  (a / a x b / b), 得到的就是两个方向向量的叉积, 他们的叉积就是从前一段方向转到后一段方向的有符号转交的正弦
  //                       ==>    signed_sine = (axb) / ab = sin(phi)
  // 这里也可以得到, 曲率的计算方式: k = 2 * sin(phi) / c
  for(std::size_t i = 1; i + 1 < n; ++i)
  {
    double const ax = points[i].x - points[i - 1].x;
    double const ay = points[i].y - points[i - 1].y;
    double const bx = points[i + 1].x - points[i].x;
    double const by = points[i + 1].y - points[i].y;
    double const a = std::hypot(ax, ay);
    double const b = std::hypot(bx, by);
    double const c = std::hypot(points[i + 1].x - points[i - 1].x,
                                points[i + 1].y - points[i - 1].y);
    if(!std::isfinite(c) || c <= min_distance)
    {
      throw std::invalid_argument("Degenerate three-point stencil");
    }
    // Equivalent to 2 * cross(a_vec, b_vec) / (a * b * c).
    double const signed_sine = (ax / a) * (by / b) - (ay / a) * (bx / b);
    result.curvature[i] = 2.0 * signed_sine / c;
    double const cosine = (ax / a) * (bx / b) + (ay / a) * (by / b);
    result.sharp_corner[i] = cosine <= std::cos(pi / 6.0);
  }
  result.curvature.front() = result.sharp_corner[1] ? 0.0 : result.curvature[1];
  result.curvature.back() =
      result.sharp_corner[n - 2] ? 0.0 : result.curvature[n - 2];
  return result;
}


// Analytic reference for the GENERATED sine path, not used by computeGeometry.
double
sineCurvature(double x, double amplitude = 0.8, double wavelength = 4.0)
{
  double const omega = 2.0 * pi / wavelength;
  double const dy = amplitude * omega * std::cos(omega * x);
  double const ddy = -amplitude * omega * omega * std::sin(omega * x);
  return ddy / std::pow(1.0 + dy * dy, 1.5);
}

// curvature[i]：第 i 个路径点的有符号曲率
// s[i]：第 i 个路径点的累计弧长，要求非递减
// rate：每米路径允许的最大宽度变化
// default_width：每侧期望宽度 [m]，两侧均返回正数。
// safety：内侧宽度与曲率半径的最大比值，必须在 (0, 1) 内。
// 仅约束局部偏移；不同路段之间的全局相交需要单独检测。
std::pair<std::vector<double>, std::vector<double>>
compute_widths(std::vector<double> const& curvature,
               std::vector<double> const& s,
               double rate = 0.3,
               double default_width = 2.0,
               double safety = 0.8,
               std::vector<bool> const& sharp_corners = {})
{
  if((!sharp_corners.empty() && sharp_corners.size() != s.size()) ||
     curvature.size() != s.size() || !std::isfinite(rate) || !(rate > 0.0) ||
     !std::isfinite(default_width) || !(default_width > 0.0) ||
     !std::isfinite(safety) || safety <= 0.0 || safety >= 1.0)
  {
    throw std::invalid_argument("Invalid width parameters");
  }

  std::size_t const n = curvature.size();
  std::vector<double> left_width(n, default_width);
  std::vector<double> right_width(n, default_width);

  for(std::size_t i = 0; i < n; ++i)
  {
    if(!std::isfinite(curvature[i]) || !std::isfinite(s[i]) ||
       (i > 0 && s[i] < s[i - 1]))
    {
      throw std::invalid_argument("Invalid path data");
    }

    // 尖角不是光滑圆弯：保留期望宽度，由偏移边界的接头和裁环处理。
    if(!sharp_corners.empty() && sharp_corners[i])
    {
      continue;
    }
    double const k = curvature[i];
    // 左转限制左侧，右转限制右侧；直线无需曲率限宽。
    if(k > 0.0)
    {
      left_width[i] = std::min(default_width, safety / k);
    }
    else if(k < 0.0)
    {
      right_width[i] = std::min(default_width, safety / -k);
    }
  }

  // 前向：限制离开窄区域后的宽度恢复速度。
  for(std::size_t i = 1; i < n; ++i)
  {
    double const ds = s[i] - s[i - 1];
    left_width[i] = std::min(left_width[i], left_width[i - 1] + rate * ds);
    right_width[i] = std::min(right_width[i], right_width[i - 1] + rate * ds);
  }

  // 反向：让进入窄区域前的宽度提前收窄。
  for(std::size_t i = n; i > 1; --i)
  {
    std::size_t const j = i - 2;
    double const ds = s[j + 1] - s[j];
    left_width[j] = std::min(left_width[j], left_width[j + 1] + rate * ds);
    right_width[j] = std::min(right_width[j], right_width[j + 1] + rate * ds);
  }

  return {left_width, right_width};
}

// Build an offset polyline with bounded miter joins (bevel for very sharp turns).
// Unlike point-normal offsets, a corner may emit two points. Inner offset loops
// are trimmed at their intersections; output indices no longer match the path.
std::vector<Point2D>
buildOffsetBoundary(std::vector<Point2D> const& points,
                    std::vector<double> const& widths,
                    double side)
{
  if(points.size() != widths.size() || points.size() < 2 ||
     (side != 1.0 && side != -1.0))
  {
    throw std::invalid_argument("Invalid offset boundary parameters");
  }
  std::vector<Point2D> normals;
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    if(!std::isfinite(points[i].x) || !std::isfinite(points[i].y) ||
       !std::isfinite(widths[i]) || widths[i] <= 0.0)
    {
      throw std::invalid_argument("Invalid offset boundary data");
    }
    if(i == 0)
    {
      continue;
    }
    double const dx = points[i].x - points[i - 1].x;
    double const dy = points[i].y - points[i - 1].y;
    double const length = std::hypot(dx, dy);
    if(!std::isfinite(length) || length <= 1e-9)
    {
      throw std::invalid_argument("Repeated/invalid offset points");
    }
    normals.push_back({-dy / length, dx / length});
  }
  std::vector<Point2D> raw;
  auto const offset = [&](std::size_t i, Point2D normal)
  {
    raw.push_back({points[i].x + side * widths[i] * normal.x,
                   points[i].y + side * widths[i] * normal.y,});
  };
  offset(0, normals.front());
  for(std::size_t i = 1; i + 1 < points.size(); ++i)
  {
    auto const a = normals[i - 1];
    auto const b = normals[i];
    double const denominator = 1.0 + a.x * b.x + a.y * b.y;
    // Miter length / width = sqrt(2 / denominator), limited to 4.
    if(denominator >= 2.0 / 16.0)
    {
      offset(i, {(a.x + b.x) / denominator, (a.y + b.y) / denominator});
    }
    else
    {
      offset(i, a);
      offset(i, b);
    }
  }
  offset(points.size() - 1, normals.back());

  auto const cross = [](Point2D a, Point2D b) { return a.x * b.y - a.y * b.x; };
  auto const subtract = [](Point2D a, Point2D b)
  { return Point2D{a.x - b.x, a.y - b.y}; };
  std::vector<Point2D> boundary;
  // Remove loops in traversal order. This is O(n^2), suitable for this example;
  // it is not a polygon union or a guarantee against left/right global overlaps.
  for(auto const end : raw)
  {
    if(!boundary.empty())
    {
      auto const start = boundary.back();
      auto const direction = subtract(end, start);
      for(std::size_t j = 0; j + 2 < boundary.size(); ++j)
      {
        auto const a = boundary[j];
        auto const edge = subtract(boundary[j + 1], a);
        double const denominator = cross(edge, direction);
        if(std::abs(denominator) <= 1e-12 * std::hypot(edge.x, edge.y) *
                                        std::hypot(direction.x, direction.y))
        {
          continue;
        }
        auto const delta = subtract(start, a);
        double const t = cross(delta, direction) / denominator;
        double const u = cross(delta, edge) / denominator;
        if(t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
        {
          Point2D const intersection{a.x + t * edge.x, a.y + t * edge.y};
          boundary.resize(j + 1);
          if(std::hypot(intersection.x - a.x, intersection.y - a.y) > 1e-9)
          {
            boundary.push_back(intersection);
          }
          break;
        }
      }
    }
    if(boundary.empty() ||
       std::hypot(end.x - boundary.back().x, end.y - boundary.back().y) > 1e-9)
    {
      boundary.push_back(end);
    }
  }
  return boundary;
}

struct BoundaryPoints
{
  Point2D left;
  Point2D right;
};

BoundaryPoints
computeBoundaryPoints(Point2D const& center,
                      double yaw,
                      double left_width,
                      double right_width)
{
  double const nx = -std::sin(yaw);
  double const ny = std::cos(yaw);

  return {{center.x + left_width * nx, center.y + left_width * ny},
          {center.x - right_width * nx, center.y - right_width * ny},};
}

// points：中心路径点
// s：之前 computeGeometry() 得到的累计弧长
// i：当前点索引
//
// 前提：至少两个点，相邻点不重复，s 严格递增。
Point2D
estimateUnitTangent(std::vector<Point2D> const& points,
                    std::vector<double> const& s,
                    std::size_t i)
{
  std::size_t const n = points.size();
  double tx = NAN;
  double ty = NAN;

  if(i == 0)
  {
    // 起点：前向差分。
    tx = points[1].x - points[0].x;
    ty = points[1].y - points[0].y;
  }
  else if(i == n - 1)
  {
    // 终点：后向差分。
    tx = points[i].x - points[i - 1].x;
    ty = points[i].y - points[i - 1].y;
  }
  else
  {
    // 内点：按不等间距弧长构造三点差分。
    double const h0 = s[i] - s[i - 1];
    double const h1 = s[i + 1] - s[i];

    double const vx0 = (points[i].x - points[i - 1].x) / h0;
    double const vy0 = (points[i].y - points[i - 1].y) / h0;

    double const vx1 = (points[i + 1].x - points[i].x) / h1;
    double const vy1 = (points[i + 1].y - points[i].y) / h1;

    tx = (h1 * vx0 + h0 * vx1) / (h0 + h1);
    ty = (h1 * vy0 + h0 * vy1) / (h0 + h1);
  }

  double const norm = std::hypot(tx, ty);
  if(norm <= 1e-12)
  {
    throw std::runtime_error("无法确定路径切线方向");
  }

  return {tx / norm, ty / norm};
}

#ifndef REFERENCE_LINE_GEOMETRY_ONLY
int
main()
{
  rerun::RecordingStream const rec("reference_line");
  rec.spawn().exit_on_failure();

  // 世界坐标系：X 向右、Y 向前、Z 向上。
  rec.log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  // 生成分段轨迹：直线 -> 左转四分之一圆 -> 直线 -> S 弯。
  // 长度单位均为 m。
  try
  {
    auto const points2 = generateSinePath(10.0, // 第一段直线长度
                                          1.0,  // 左转圆弧半径
                                          8.0,  // 第二段直线长度
                                          8.0,  // S 弯纵向跨度，非弧长
                                          0.8,  // S 弯横向振幅
                                          0.02  // 相邻采样点最大间距
    );
    // 从离散坐标计算累计弧长和曲率。
    auto const geometry = compute_geometry(points2);

    // 每侧期望宽度 2 m，宽度变化率 0.05 m/m，内侧半径裕量 20%。
    auto const widths = compute_widths(geometry.curvature,
                                       geometry.s,
                                       0.05,
                                       2.0,
                                       0.8,
                                       geometry.sharp_corner);

    auto s = geometry.s;

    std::vector<rerun::Vector3D> points;
    std::vector<rerun::Vector3D> center;

    points.reserve(s.size());
    center.reserve(s.size());
    for(auto const si : s)
    {
      points.emplace_back(si, 0, 0);
    }
    std::vector<rerun::Vector3D> lboundary;
    lboundary.reserve(s.size());
    std::vector<rerun::Vector3D> rboundary;
    rboundary.reserve(s.size());
    for(int i = 0; i < s.size(); ++i)
    {
      lboundary.emplace_back(s[i], widths.first[i], 0);
      rboundary.emplace_back(s[i], -widths.second[i], 0);
      center.emplace_back(points2[i].x, points2[i].y, 0);
    }


    std::vector<rerun::Vector3D> left_boundary;
    std::vector<rerun::Vector3D> right_boundary;

    left_boundary.reserve(points2.size());
    right_boundary.reserve(points2.size());

    // 尖角采用偏移线接头，内侧交叉部分裁掉，保留中心线原有尖角。
    for(auto const& p : buildOffsetBoundary(points2, widths.first, 1.0))
    {
      left_boundary.emplace_back(p.x, p.y, 0);
    }
    for(auto const& p : buildOffsetBoundary(points2, widths.second, -1.0))
    {
      right_boundary.emplace_back(p.x, p.y, 0);
    }

    rec.log("/center_line",
            rerun::LineStrips3D{rerun::LineStrip3D(center)}.with_colors(
                {rerun::Color{255, 255, 255}}));
    rec.log("/left_boundary",
            rerun::LineStrips3D{rerun::LineStrip3D(left_boundary)});
    rec.log("/right_boundary",
            rerun::LineStrips3D{rerun::LineStrip3D(right_boundary)});
  }
  catch(std::exception const& e)
  {}



  return 0;
}

#endif
