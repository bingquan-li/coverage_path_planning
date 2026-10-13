#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>


#include <thread>
#include <vector>

using Ring = std::vector<cv::Point>;

struct Polygon
{
  Ring outer;
  std::vector<Ring> holes;
};

cv::Mat
make_gridmap()
{
  // 8U：8 位无符号整数
  // C1：单通道，也就是灰度图，灰度值 = 亮度值。数值越小 → 越暗、数值越大 → 越亮，想象成一个灯的亮度旋钮
  // 0   → 黑色
  // 127 → 灰色
  // 255 → 白色
  cv::Mat map(480, 720, CV_8UC1, cv::Scalar(127));
  cv::rectangle(map, {20, 20}, {699, 459}, cv::Scalar(0), cv::FILLED);
  // Concave main room and a disconnected room to the right.
  // 一个凹多边形主房间，以及右侧一个不连通的房间。
  // 每一个存放的都是一个点的 (x,y)
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
  //  会把这些点连接成一个多边形
  cv::fillPoly(map, std::vector<Ring>{main_room}, cv::Scalar(255));
  cv::rectangle(map, {550, 50}, {675, 425}, cv::Scalar(255), cv::FILLED);
  // Internal obstacles become holes of their respective free-space polygons.
  // → 内部障碍物会成为各自自由空间多边形中的孔洞。
  cv::rectangle(map, {160, 110}, {240, 190}, cv::Scalar(0), cv::FILLED);
  cv::circle(map, {350, 320}, 52, cv::Scalar(0), cv::FILLED);
  Ring const triangle{{580, 175}, {650, 225}, {580, 275}};
  cv::fillPoly(map, std::vector<Ring>{triangle}, cv::Scalar(0));
  // Unknown cells are excluded from free space too (a notch at the room edge).
  // → 未知区域的栅格同样不属于自由空间（这里在房间边缘形成了一个缺口）。
  cv::rectangle(map, {285, 40}, {325, 85}, cv::Scalar(127), cv::FILLED);

  return map;
}


int
main()
{
  // 1. 创建 Foxglove WebSocket Server
  foxglove::WebSocketServerOptions options;
  options.host = "0.0.0.0";
  options.port = 8765;

  auto const server = foxglove::WebSocketServer::create(std::move(options)).value();

  // 2. 创建图像 channel
  auto image_channel =
      foxglove::messages::RawImageChannel::create("/gridmap").value();

  // 3. 创建 OpenCV 地图
  cv::Mat const map = make_gridmap();

  // 4. OpenCV Mat -> Foxglove RawImage
  foxglove::messages::RawImage image;

  image.width = static_cast<uint32_t>(map.cols);
  image.height = static_cast<uint32_t>(map.rows);

  // CV_8UC1 对应单通道 8 bit 灰度图
  image.encoding = "mono8";

  // 每一行占多少字节
  image.step = static_cast<uint32_t>(map.step);
  image.data.resize(map.total() * map.elemSize());

  std::memcpy(image.data.data(), map.data, image.data.size());

  auto annotation_channel = foxglove::messages::ImageAnnotationsChannel::create(
                                "/gridmap_annotations")
                                .value();
  foxglove::messages::ImageAnnotations annotations;
  // rectangle
  {
    foxglove::messages::TextAnnotation text;
    text.position = foxglove::messages::Point2{160, 100};
    text.text = "rectangle";
    text.font_size = 16;
    text.text_color = foxglove::messages::Color{0.0, 1.0, 0.0, 1.0};
    annotations.texts.push_back(text);
  }

  // circle
  {
    foxglove::messages::TextAnnotation text;
    text.position = foxglove::messages::Point2{300, 250};
    text.text = "circle";
    text.font_size = 16;
    text.text_color = foxglove::messages::Color{0.0, 0.0, 1.0, 1.0};
    annotations.texts.push_back(text);
  }

  // triangle
  {
    foxglove::messages::TextAnnotation text;
    text.position = foxglove::messages::Point2{560, 160};
    text.text = "triangle";
    text.font_size = 16;
    text.text_color = foxglove::messages::Color{1.0, 1.0, 0.0, 1.0};
    annotations.texts.push_back(text);
  }


  // 保持程序运行
  while(true)
  {
    // 5. 发布
    annotation_channel.log(annotations);
    image_channel.log(image);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}