#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rerun.hpp>

#include <cstdint>
#include <vector>

using Ring = std::vector<cv::Point>;

struct Polygon
{
  Ring outer;
  std::vector<Ring> holes;
};

void
log_gridmap(rerun::RecordingStream const& rec,
            cv::Mat const& map,
            float resolution,      // 米/像素
            float origin_x = 0.0F, // 地图左下角的世界坐标
            float origin_y = 0.0F)
{
  CV_Assert(!map.empty());
  CV_Assert(map.type() == CV_8UC1);
  CV_Assert(resolution > 0.0F);

  auto const width = static_cast<std::uint32_t>(map.cols);
  auto const height = static_cast<std::uint32_t>(map.rows);

  std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width) * height);

  // 逐行复制，也支持内存不连续的 Mat，例如 ROI。
  for(int row = 0; row < map.rows; ++row)
  {
    std::memcpy(pixels.data() + static_cast<std::size_t>(row) * width,
                map.ptr<std::uint8_t>(row),
                width);
  }

  rec.log_static("world/map",
                 rerun::archetypes::GridMap()
                     .with_data(rerun::ImageBuffer(pixels))
                     .with_format(rerun::components::ImageFormat(
                         {width, height},
                         rerun::ColorModel::L,
                         rerun::ChannelDatatype::U8))
                     .with_cell_size(resolution)
                     .with_translation({origin_x, origin_y, 0.0F}));
}

cv::Mat
make_gridmap()
{
  // 0：障碍物；127：未知；255：自由空间
  cv::Mat map(480, 720, CV_8UC1, cv::Scalar(127));

  cv::rectangle(map, {20, 20}, {699, 459}, cv::Scalar(0), cv::FILLED);

  // 凹多边形主房间
  Ring const main_room{{40, 40},
                       {440, 40},
                       {440, 180},
                       {510, 180},
                       {510, 435},
                       {40, 435},
                       {40, 300},
                       {100, 300},
                       {100, 200},
                       {40, 200},};

  cv::fillPoly(map, std::vector<Ring>{main_room}, cv::Scalar(255));

  // 右侧不连通的房间
  cv::rectangle(map, {550, 50}, {675, 425}, cv::Scalar(255), cv::FILLED);

  // 矩形障碍物
  cv::rectangle(map, {160, 110}, {240, 190}, cv::Scalar(0), cv::FILLED);

  // 圆形障碍物
  cv::circle(map, {350, 320}, 52, cv::Scalar(0), cv::FILLED);

  // 三角形障碍物
  Ring const triangle{{580, 175}, {650, 225}, {580, 275}};

  cv::fillPoly(map, std::vector<Ring>{triangle}, cv::Scalar(0));

  // 房间边缘的未知区域
  cv::rectangle(map, {285, 40}, {325, 85}, cv::Scalar(127), cv::FILLED);

  return map;
}

int
main()
{
  // 1. 创建记录流，启动本机 Rerun Viewer 并连接
  rerun::RecordingStream const rec("coverage_planner");
  rec.spawn().exit_on_failure();
  cv::Mat const map = make_gridmap();

  log_gridmap(rec, map, 0.05F); // 每格 5 cm
  // // 2. 创建地图
  // cv::Mat map = make_gridmap();

  // CV_Assert(map.type() == CV_8UC1);

  // // Rerun 此接口要求像素紧密排列，没有行间填充。
  // // 当前生成的 map 本身是连续的；这一步兼容未来传入 ROI 的情况。
  // if(!map.isContinuous())
  // {
  //   map = map.clone();
  // }

  // // 3. 记录灰度图
  // rec.log_static(
  //     "gridmap",
  //     rerun::Image::from_grayscale8(
  //         rerun::Collection<std::uint8_t>::borrow(map.ptr<std::uint8_t>(),
  //                                                 map.total()),
  //         {static_cast<std::uint32_t>(map.cols),
  //          static_cast<std::uint32_t>(map.rows)}));

  // // 4. 在图像的像素坐标系中添加标注
  // // 使用图像下面的子路径，方便在同一个 2D 视图中叠加显示。
  // rec.log_static(
  //     "gridmap/annotations",
  //     rerun::Points2D({{160.0F, 100.0F}, {300.0F, 250.0F}, {560.0F, 160.0F}})
  //         .with_labels({"rectangle", "circle", "triangle"})
  //         .with_colors({rerun::Color(0, 255, 0),
  //                       rerun::Color(0, 0, 255),
  //                       rerun::Color(255, 255, 0)})
  //         .with_radii({2.0F})
  //         .with_show_labels(true));



  // 5. 退出前等待缓冲数据发送完成
  auto const result = rec.flush_blocking();

  return 0;
}