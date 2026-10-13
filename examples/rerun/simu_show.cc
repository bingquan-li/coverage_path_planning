#include <math.h>

#include <numbers>
#include <rerun.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr double kPi = std::numbers::pi;

struct Pose2D
{
  double x;
  double y;
  double yaw; // 弧度；机器人 +X 方向为车头
};

using Point3 = std::array<float, 3>;
using Path = std::vector<Pose2D>;

double
angleDiff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

double
distance(Pose2D const& a, Pose2D const& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// 将机器人局部坐标转换到世界坐标。
Point3
toWorld(Pose2D const& pose, double x, double y, float z)
{
  double const c = std::cos(pose.yaw);
  double const s = std::sin(pose.yaw);

  return {
      static_cast<float>(pose.x + c * x - s * y),
      static_cast<float>(pose.y + s * x + c * y),
      z,
  };
}

// 示例机器人：长 0.50 m，宽 0.38 m，中心为旋转参考点。
// 换成真实机器人时，替换这里的局部坐标顶点。
std::vector<Point3>
footprint(Pose2D const& pose, float z)
{
  return {
      toWorld(pose, 0.25, 0.19, z),
      toWorld(pose, 0.25, -0.19, z),
      toWorld(pose, -0.25, -0.19, z),
      toWorld(pose, -0.25, 0.19, z),
      toWorld(pose, 0.25, 0.19, z), // 首尾相连，闭合轮廓
  };
}

void
logPath(rerun::RecordingStream const& rec,
        std::string const& entity,
        Path const& path,
        rerun::Color color,
        float radius,
        float z)
{
  if(path.size() < 2)
  {
    rec.log(entity, rerun::Clear::RECURSIVE);
    return;
  }

  std::vector<Point3> points;
  points.reserve(path.size());

  for(auto const& pose : path)
  {
    points.push_back({
        static_cast<float>(pose.x),
        static_cast<float>(pose.y),
        z,
    });
  }

  rec.log(entity,
          rerun::LineStrips3D({rerun::LineStrip3D(points)})
              .with_colors({color})
              .with_radii({radius}));
}

// 按累计路程或航向变化采样，避免每个轨迹点都画箭头。
// 箭头使用 yaw，而不是相邻位置之差，因此也适用于倒车。
void
logHeadings(rerun::RecordingStream const& rec,
            std::string const& entity,
            Path const& path,
            rerun::Color color,
            float length,
            float z)
{
  if(path.empty())
  {
    rec.log(entity, rerun::Clear::RECURSIVE);
    return;
  }

  std::vector<rerun::Position3D> origins;
  std::vector<rerun::Vector3D> vectors;

  double accumulated_distance = 0.0;
  double last_yaw = path.front().yaw;

  for(std::size_t i = 0; i < path.size(); ++i)
  {
    auto const& pose = path[i];

    if(i > 0)
    {
      accumulated_distance += distance(path[i - 1], pose);
    }

    bool const should_draw =
        i == 0 || accumulated_distance >= 0.45 ||
        std::abs(angleDiff(pose.yaw, last_yaw)) >= kPi / 6.0;

    if(!should_draw)
    {
      continue;
    }

    origins.emplace_back(static_cast<float>(pose.x),
                         static_cast<float>(pose.y),
                         z);

    vectors.emplace_back(length * static_cast<float>(std::cos(pose.yaw)),
                         length * static_cast<float>(std::sin(pose.yaw)),
                         0.0F);

    accumulated_distance = 0.0;
    last_yaw = pose.yaw;
  }

  rec.log(entity,
          rerun::Arrows3D::from_vectors(vectors)
              .with_origins(origins)
              .with_colors({color})
              .with_radii({0.008F}));
}

void
logRobot(rerun::RecordingStream const& rec, Pose2D const& pose)
{
  rec.log("world/robot/footprint",
          rerun::LineStrips3D({
                                  rerun::LineStrip3D(footprint(pose, 0.05F)),
                              })
              .with_colors({rerun::Color(255, 220, 60)})
              .with_radii({0.015F}));

  logHeadings(rec,
              "world/robot/heading",
              Path{pose},
              rerun::Color(255, 220, 60),
              0.40F,
              0.06F);
}

void
logHistory(rerun::RecordingStream const& rec, std::deque<Pose2D> const& poses)
{
  std::vector<rerun::LineStrip3D> strips;

  strips.reserve(poses.size());
for(auto const& pose : poses)
  {
    strips.emplace_back(footprint(pose, 0.015F));
  }

  rec.log("world/history/footprints",
          rerun::LineStrips3D(strips)
              .with_colors({rerun::Color(160, 185, 170, 80)})
              .with_radii({0.005F}));
}

// 示例全局路径：y = 0.5 sin(x)。
// yaw 是该曲线切线方向。
Pose2D
referencePose(double x)
{
  return {
      x,
      0.5 * std::sin(x),
      std::atan2(0.5 * std::cos(x), 1.0),
  };
}

// 模拟实际运动相对参考路径存在小幅偏差。
Pose2D
actualPose(double x)
{
  return {
      x,
      0.5 * std::sin(x) + 0.06 * std::sin(4.0 * x),
      std::atan2(0.5 * std::cos(x) + 0.24 * std::cos(4.0 * x), 1.0),
  };
}

} // namespace

int
main()
{
  rerun::RecordingStream const rec("robot_trajectory_demo");
  rec.spawn().exit_on_failure();

  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  rerun::Color const blue(70, 140, 255);
  rerun::Color const orange(255, 150, 40);
  rerun::Color const green(60, 220, 130);

  // 1. 全局参考路径：在第 0 帧记录，之后保持显示。
  rec.set_time_sequence("frame", 0);

  Path global_path;
  for(int i = 0; i <= 200; ++i)
  {
    global_path.push_back(referencePose(i * 0.05));
  }

  logPath(rec, "world/path/global", global_path, blue, 0.006F, 0.005F);

  Path actual_history;
  std::deque<Pose2D> footprint_history;

  // 20 Hz，共 24 秒：
  // 前 12 秒行驶，中间 4 秒原地旋转一圈，最后继续行驶。
  for(std::int64_t frame = 0; frame <= 480; ++frame)
  {
    rec.set_time_sequence("frame", frame);

    bool const rotating = frame >= 240 && frame < 320;

    double x = NAN;
    if(frame < 240)
    {
      x = frame * 0.025;
    }
    else if(frame < 320)
    {
      x = 6.0;
    }
    else
    {
      x = 6.0 + (frame - 320) * 0.025;
    }

    Pose2D robot = actualPose(x);

    if(rotating)
    {
      // x、y 不变，仅改变 yaw。
      robot.yaw += 2.0 * kPi * static_cast<double>(frame - 240) / 80.0;
    }

    // 2. 当前局部规划轨迹：前方 1.5 m。
    // 原地旋转阶段清除局部行驶轨迹。
    Path local_path;

    if(!rotating)
    {
      for(int i = 0; i <= 30; ++i)
      {
        double const local_x = x + i * 0.05;
        if(local_x > 10.0)
        {
          break;
        }

        local_path.push_back(referencePose(local_x));
      }
    }

    logPath(rec, "world/path/local/line", local_path, orange, 0.014F, 0.02F);

    logHeadings(rec,
                "world/path/local/headings",
                local_path,
                orange,
                0.22F,
                0.03F);

    // 3. 实际轨迹：按位置或航向变化采样。
    // 原地旋转时仍记录姿态，使朝向箭头能表达旋转。
    if(actual_history.empty() ||
       distance(actual_history.back(), robot) >= 0.04 ||
       std::abs(angleDiff(robot.yaw, actual_history.back().yaw)) >= kPi / 12.0)
    {
      actual_history.push_back(robot);
    }

    logPath(rec,
            "world/path/actual/line",
            actual_history,
            green,
            0.010F,
            0.025F);

    logHeadings(rec,
                "world/path/actual/headings",
                actual_history,
                green,
                0.18F,
                0.035F);

    // 4. 当前机器人轮廓和车头箭头。
    logRobot(rec, robot);

    // 5. 稀疏历史轮廓：每隔 0.8 m 或旋转 30° 记录一次。
    if(footprint_history.empty() ||
       distance(footprint_history.back(), robot) >= 0.8 ||
       std::abs(angleDiff(robot.yaw, footprint_history.back().yaw)) >=
           kPi / 6.0)
    {
      footprint_history.push_back(robot);

      while(footprint_history.size() > 12)
      {
        footprint_history.pop_front();
      }
    }

    logHistory(rec, footprint_history);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}