#include <rerun.hpp>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>


struct Point
{
  double x, y;
};
struct Projection
{
  Point q;
  double yaw;
  double lateral;
  double distance;
};

// 投影到最近的非退化线段；方向由 path 点的排列顺序决定。
inline Projection
project(std::vector<Point> const& path, Point p)
{
  double best = std::numeric_limits<double>::infinity();
  Projection result{};
  for(std::size_t i = 1; i < path.size(); ++i)
  {
    auto const a = path[i - 1];
    auto const b = path[i];
    double const dx = b.x - a.x;
    double const dy = b.y - a.y;
    double const len2 = dx * dx + dy * dy;
    if(len2 < 1e-12)
    {
      continue;
    }
    double const u =
        std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
    Point const q{a.x + u * dx, a.y + u * dy};
    double const ex = p.x - q.x;
    double const ey = p.y - q.y;
    double const d2 = ex * ex + ey * ey;
    if(d2 < best)
    {
      best = d2;
      // 单位左法向量为 (-dy, dx) / length。
      result = {q,
                std::atan2(dy, dx),
                (-dy * ex + dx * ey) / std::sqrt(len2),
                std::sqrt(d2),};
    }
  }
  if(!std::isfinite(best))
  {
    throw std::invalid_argument("Path needs a nonzero-length segment");
  }
  return result;
}


rerun::Vec3D
xyz(Point p, float z = 0.0F)
{
  return {static_cast<float>(p.x), static_cast<float>(p.y), z};
}

void
log_frame(rerun::RecordingStream const& rec,
          double t,
          Point robot,
          double yaw,
          Projection const& pr,
          std::vector<rerun::Vec3D> const& history)
{
  constexpr double threshold = 0.15;
  rec.set_time_duration_secs("sim_time", t);
  auto const color = std::abs(pr.lateral) > threshold
                         ? rerun::Color(255, 70, 70)
                         : rerun::Color(255, 190, 50);

  rec.log("world/robot/center",
          rerun::Points3D({xyz(robot, 0.03F)})
              .with_colors(rerun::Color(70, 160, 255))
              .with_radii(0.06F));

  // 0.50 m × 0.38 m 的平面轮廓，转换到世界坐标后绘制。
  std::vector<rerun::Vec3D> outline;
  for(Point const c : {Point{0.25, 0.19},
                 Point{0.25, -0.19},
                 Point{-0.25, -0.19},
                 Point{-0.25, 0.19},
                 Point{0.25, 0.19},})
  {
    outline.push_back(xyz({robot.x + std::cos(yaw) * c.x - std::sin(yaw) * c.y,
                           robot.y + std::sin(yaw) * c.x + std::cos(yaw) * c.y,},
                          0.03F));
  }
  rec.log("world/robot/outline",
          rerun::LineStrips3D(rerun::LineStrip3D(outline))
              .with_colors(rerun::Color(70, 160, 255))
              .with_radii(0.012F));
  rec.log(
      "world/robot/heading",
      rerun::Arrows3D::from_vectors({{static_cast<float>(0.45 * std::cos(yaw)),
                                      static_cast<float>(0.45 * std::sin(yaw)),
                                      0.0F,},})
          .with_origins({xyz(robot, 0.03F)})
          .with_colors(rerun::Color(70, 160, 255))
          .with_radii(0.02F));

  rec.log("world/projection",
          rerun::Points3D({xyz(pr.q, 0.04F)})
              .with_colors(rerun::Color(80, 240, 120))
              .with_radii(0.045F));
  // 连线在折线顶点/端点处不一定垂直，因此单独命名为 nearest link。
  rec.log("world/nearest_link",
          rerun::LineStrips3D(
              rerun::LineStrip3D({xyz(pr.q, 0.04F), xyz(robot, 0.04F)}))
              .with_colors(rerun::Color(150, 150, 150))
              .with_radii(0.008F));

  // 只画法向误差：即便投影落在线段端点，箭头长度也是 |lateral|。
  double const nx = -std::sin(pr.yaw);
  double const ny = std::cos(pr.yaw);
  char label[80];
  std::snprintf(label, sizeof(label), "lateral = %+.3f m", pr.lateral);
  rec.log("world/lateral_error",
          rerun::Arrows3D::from_vectors({{static_cast<float>(nx * pr.lateral),
                                          static_cast<float>(ny * pr.lateral),
                                          0.0F,},})
              .with_origins({xyz(pr.q, 0.06F)})
              .with_colors(color)
              .with_radii(0.015F)
              .with_labels({label})
              .with_show_labels(true));

  if(history.size() >= 2)
  {
    rec.log("world/actual_path",
            rerun::LineStrips3D(rerun::LineStrip3D(history))
                .with_colors(rerun::Color(70, 160, 255))
                .with_radii(0.01F));
  }
  rec.log("metrics/lateral_error_m", rerun::Scalars(pr.lateral));
  rec.log("metrics/zero", rerun::Scalars(0.0));
  rec.log("metrics/upper_limit", rerun::Scalars(threshold));
  rec.log("metrics/lower_limit", rerun::Scalars(-threshold));
}

int
main(int argc, char* const* argv)
{
  bool const save = argc == 3 && std::string(argv[1]) == "--save";
  if(argc != 1 && !save)
  {
    std::cerr << "Usage: lateral_error_demo [--save output.rrd]\n";
    return 1;
  }
  rerun::RecordingStream const rec("lateral_error_demo");
  if(save)
  {
    rec.save(argv[2]).exit_on_failure();
  }
  else
  {
    rec.spawn().exit_on_failure();
  }
  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  // 参考曲线 y = 0.8 sin(0.5x)，离散为折线，单位为米。
  std::vector<Point> path;
  std::vector<rerun::Vec3D> reference;
  for(int i = 0; i <= 600; ++i)
  {
    double const x = i * 0.02;
    path.push_back({x, 0.8 * std::sin(0.5 * x)});
    reference.push_back(xyz(path.back()));
  }
  rec.log_static("world/reference_path",
                 rerun::LineStrips3D(rerun::LineStrip3D(reference))
                     .with_colors(rerun::Color(80, 240, 120))
                     .with_radii(0.018F));
  rec.log_static("metrics/lateral_error_m",
                 rerun::SeriesLines()
                     .with_names("lateral error (m)")
                     .with_colors(rerun::Color(255, 190, 50))
                     .with_widths(2.0F));
  rec.log_static("metrics/zero",
                 rerun::SeriesLines().with_names("zero").with_colors(
                     rerun::Color(150, 150, 150)));
  rec.log_static("metrics/upper_limit",
                 rerun::SeriesLines().with_names("+0.15 m").with_colors(
                     rerun::Color(255, 70, 70)));
  rec.log_static("metrics/lower_limit",
                 rerun::SeriesLines().with_names("-0.15 m").with_colors(
                     rerun::Color(255, 70, 70)));

  // 20 秒 / 50 Hz；构造振荡并逐渐收敛的横向偏移，不是控制器仿真。
  constexpr double dt = 0.02;
  std::vector<rerun::Vec3D> history;
  Point prev{};
  auto const start = std::chrono::steady_clock::now();
  for(int k = 0; k <= 1000; ++k)
  {
    double const t = k * dt;
    double const x = 0.5 + 0.5 * t;
    double const y = 0.8 * std::sin(0.5 * x);
    double const ref_yaw = std::atan2(0.4 * std::cos(0.5 * x), 1.0);
    double const offset = 0.35 * std::exp(-0.06 * t) * std::sin(1.4 * t);
    Point const robot{x - std::sin(ref_yaw) * offset,
                      y + std::cos(ref_yaw) * offset,};
    // 朝向来自实际运动方向；横向误差仍以参考路径方向为准。
    double const yaw =
        k == 0 ? ref_yaw : std::atan2(robot.y - prev.y, robot.x - prev.x);
    history.push_back(xyz(robot, 0.02F));
    log_frame(rec, t, robot, yaw, project(path, robot), history);
    prev = robot;
    if(!save) {
      std::this_thread::sleep_until(
          start + std::chrono::milliseconds(((k + 1) * 20)));
}
  }
  rec.flush_blocking();
  std::cout << (save ? "Recording saved.\n"
                     : "Demo finished; rewind sim_time to replay.\n");
}
