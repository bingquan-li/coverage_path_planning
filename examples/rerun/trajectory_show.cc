#include <rerun.hpp>

#include <vector>

struct Pose2D
{
  double x;
  double y;
  double yaw; // 弧度，绕 Z 轴旋转，逆时针为正
};

void
log_trajectory(rerun::RecordingStream const& rec,
               std::vector<Pose2D> const& trajectory,
               float axis_length = 0.2F)
{
  if(trajectory.empty())
  {
    return;
  }

  std::vector<rerun::Vec3D> points;
  std::vector<rerun::Position3D> origins;
  std::vector<rerun::Vector3D> vectors;

  points.reserve(trajectory.size());
  origins.reserve(trajectory.size());
  vectors.reserve(trajectory.size());

  for(auto const& pose : trajectory)
  {
    auto const x = static_cast<float>(pose.x);
    auto const y = static_cast<float>(pose.y);
    auto const yaw = static_cast<float>(pose.yaw);

    points.emplace_back(x, y, 0.0F);
    origins.emplace_back(x, y, 0.0F);

    // 位姿的局部 +X 轴，在世界坐标系中的方向。
    vectors.emplace_back(axis_length * std::cos(yaw),
                         axis_length * std::sin(yaw),
                         0.0F);
  }

  // 所有朝向箭头，统一放在一个实体下。
  rec.log("world/trajectory/headings",
          rerun::Arrows3D::from_vectors(vectors)
              .with_origins(origins)
              .with_colors(rerun::Color(255, 0, 0))
              .with_radii(0.03F));

  // 轨迹折线。
  rec.log("world/trajectory/path",
          rerun::LineStrips3D(rerun::LineStrip3D(points))
              .with_colors(rerun::Color(0, 160, 255))
              .with_radii(0.01F));
  // std::vector<rerun::Vec3D> points;
  // points.reserve(trajectory.size());

  // for(std::size_t i = 0; i < trajectory.size(); ++i)
  // {
  //   auto const& pose = trajectory[i];

  //   auto const x = static_cast<float>(pose.x);
  //   auto const y = static_cast<float>(pose.y);
  //   auto const yaw = static_cast<float>(pose.yaw);

  //   points.emplace_back(x, y, 0.0F);

  //   // 每个位姿使用独立的实体路径，让所有位姿同时显示。
  //   std::string const entity_path =
  //       "world/trajectory/poses/" + std::to_string(i);

  //   rec.log(entity_path,
  //           rerun::Transform3D()
  //               .with_translation({x, y, 0.0F})
  //               .with_rotation_axis_angle(
  //                   rerun::RotationAxisAngle({0.0F, 0.0F, 1.0F},
  //                                            rerun::Angle::radians(yaw))),
  //           rerun::TransformAxes3D(axis_length));
  // }

  // // 用折线连接轨迹点。
  // rec.log("world/trajectory/path",
  //         rerun::LineStrips3D(rerun::LineStrip3D(points))
  //             .with_colors(rerun::Color(0, 160, 255))
  //             .with_radii(0.015F));
}

int
main()
{
  rerun::RecordingStream const rec("pose_trajectory_demo");
  // rec.spawn().exit_on_failure();
  rec.connect_grpc("rerun+http://192.168.171.203:9876/proxy").handle();
  // 世界坐标系：X 向右、Y 向前、Z 向上。
  rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  // 原点处绘制世界坐标轴。
  rec.log_static("world/origin",
                 rerun::Transform3D().with_translation({0.0F, 0.0F, 0.0F}),
                 rerun::TransformAxes3D(0.5F));

  // x、y 单位为米，yaw 单位为弧度。
  std::vector<Pose2D> const trajectory = {
      {0.0, 0.0, 0.0},
      {0.4, 0.0, 0.0},
      {0.8, 0.0, 0.0},
      {1.2, 0.0, 0.2},
      {1.6, 0.1, 0.4},
      {1.9, 0.3, 0.7},
      {2.1, 0.6, 1.0},
      {2.2, 1.0, 1.3},
      {2.2, 1.4, 1.57079632679},
      {2.2, 1.8, 1.57079632679},
  };

  log_trajectory(rec, trajectory, 0.2F);

  return 0;
}