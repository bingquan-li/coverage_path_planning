#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Ring = std::vector<cv::Point>;

struct Polygon
{
  Ring outer;
  std::vector<Ring> holes;
};

// Image convention: 0 = occupied, 127 = unknown, 255 = free.
// The world origin is the lower-left corner of the image; each cell is 5 cm.
constexpr double resolution = 0.05;
constexpr double origin_x = -2.0;
constexpr double origin_y = -1.0;
constexpr double epsilon_pixels = 2.0;

cv::Mat
make_gridmap()
{
  cv::Mat map(480, 720, CV_8UC1, cv::Scalar(127));
  cv::rectangle(map, {20, 20}, {699, 459}, cv::Scalar(0), cv::FILLED);
  // Concave main room and a disconnected room to the right.
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
  cv::rectangle(map, {550, 50}, {675, 425}, cv::Scalar(255), cv::FILLED);
  // Internal obstacles become holes of their respective free-space polygons.
  cv::rectangle(map, {160, 110}, {240, 190}, cv::Scalar(0), cv::FILLED);
  cv::circle(map, {350, 320}, 52, cv::Scalar(0), cv::FILLED);
  Ring const triangle{{580, 175}, {650, 225}, {580, 275}};
  cv::fillPoly(map, std::vector<Ring>{triangle}, cv::Scalar(0));
  // Unknown cells are excluded from free space too (a notch at the room edge).
  cv::rectangle(map, {285, 40}, {325, 85}, cv::Scalar(127), cv::FILLED);
  return map;
}

Ring
simplify_ring(Ring const& contour, bool hole)
{
  Ring ring;
  cv::approxPolyDP(contour, ring, epsilon_pixels, true);
  if(ring.size() < 3 || cv::contourArea(ring) == 0.0)
  {
    throw std::runtime_error("Contour is too small to form a polygon");
  }
  // Flipping image Y to world Y reverses orientation. In world coordinates:
  // outer rings are counterclockwise, holes are clockwise.
  if((cv::contourArea(ring, true) > 0.0) != hole)
  {
    // std::reverse(ring.begin(), ring.end());
    std::ranges::reverse(ring);
  }
  return ring;
}

std::vector<Polygon>
extract_polygons(cv::Mat const& free_mask)
{
  std::vector<Ring> contours;
  std::vector<cv::Vec4i> hierarchy;
  // CCOMP groups each outer boundary with its immediate holes, retaining
  // disconnected regions (and free islands within obstacles) as outer rings.
  cv::findContours(free_mask.clone(),
                   contours,
                   hierarchy,
                   cv::RETR_CCOMP,
                   cv::CHAIN_APPROX_SIMPLE);
  std::vector<Polygon> polygons;
  for(std::size_t i = 0; i < contours.size(); ++i)
  {
    if(hierarchy[i][3] != -1)
    {
      continue;
    }
    Polygon polygon{simplify_ring(contours[i], false), {}};
    for(int child = hierarchy[i][2]; child != -1; child = hierarchy[child][0])
    {
      polygon.holes.push_back(simplify_ring(contours[child], true));
    }
    polygons.push_back(std::move(polygon));
  }
  // Stable labels: largest free-space region first.
  // std::sort(polygons.begin(),
  //           polygons.end(),
  //           [](auto const& a, auto const& b)
  //           { return cv::contourArea(a.outer) > cv::contourArea(b.outer); });
  std::ranges::stable_sort(
      polygons,
      [](auto const& a, auto const& b)
      { return cv::contourArea(a.outer) > cv::contourArea(b.outer); });
  return polygons;
}

void
write_ring(cv::FileStorage& file, Ring const& ring, int height)
{
  file << "{" << "pixels" << "[";
  for(auto const& p : ring)
  {
    file << "[:" << p.x << p.y << "]";
  }
  file << "]" << "world_m" << "[";
  for(auto const& p : ring)
  {
    // Contours lie on boundary-cell centers, not exact grid-cell edges.
    file << "[:" << origin_x + (p.x + 0.5) * resolution
         << origin_y + (height - p.y - 0.5) * resolution << "]";
  }
  file << "]" << "}";
}

void
save_polygons(std::filesystem::path const& path,
              std::vector<Polygon> const& polygons,
              cv::Size size)
{
  cv::FileStorage file(path.string(),
                       cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  if(!file.isOpened())
  {
    throw std::runtime_error("Cannot write " + path.string());
  }
  file << "width" << size.width << "height" << size.height << "resolution_m"
       << resolution << "origin_m" << "[:" << origin_x << origin_y << "]"
       << "epsilon_pixels" << epsilon_pixels << "rings_closed_implicitly" << 1
       << "polygons" << "[";
  for(auto const& polygon : polygons)
  {
    file << "{" << "outer";
    write_ring(file, polygon.outer, size.height);
    file << "holes" << "[";
    for(auto const& hole : polygon.holes)
    {
      write_ring(file, hole, size.height);
    }
    file << "]" << "}";
  }
  file << "]";
}

void
draw_ring(cv::Mat& image, Ring const& ring, cv::Scalar const& color)
{
  cv::polylines(image, std::vector<Ring>{ring}, true, color, 2, cv::LINE_AA);
  for(auto const& vertex : ring)
  {
    cv::circle(image, vertex, 4, color, cv::FILLED, cv::LINE_AA);
  }
}

cv::Mat
make_preview(cv::Mat const& map, std::vector<Polygon> const& polygons)
{
  cv::Mat original;
  cv::cvtColor(map, original, cv::COLOR_GRAY2BGR);
  cv::Mat overlay = original.clone();
  cv::Scalar const outer_color(160, 165, 0);
  cv::Scalar const hole_color(20, 115, 240);
  for(std::size_t i = 0; i < polygons.size(); ++i)
  {
    auto const& polygon = polygons[i];
    draw_ring(overlay, polygon.outer, outer_color);
    for(auto const& hole : polygon.holes)
    {
      draw_ring(overlay, hole, hole_color);
    }
    auto const box = cv::boundingRect(polygon.outer);
    cv::putText(overlay,
                "P" + std::to_string(i),
                {box.x + 12, box.y + 28},
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                outer_color,
                2,
                cv::LINE_AA);
  }
  cv::Mat preview(map.rows + 105,
                  map.cols * 2 + 30,
                  CV_8UC3,
                  cv::Scalar(248, 248, 248));
  original.copyTo(preview(cv::Rect(10, 50, map.cols, map.rows)));
  overlay.copyTo(preview(cv::Rect(map.cols + 20, 50, map.cols, map.rows)));
  auto const label =
      [&](std::string const& text, cv::Point position, cv::Scalar color)
  {
    cv::putText(preview,
                text,
                position,
                cv::FONT_HERSHEY_SIMPLEX,
                0.62,
                std::move(color),
                1,
                cv::LINE_AA);
  };
  label("Input gridmap", {20, 32}, cv::Scalar(40, 40, 40));
  label("Extracted free-space polygons",
        {map.cols + 30, 32},
        cv::Scalar(40, 40, 40));
  label("White: free   Black: occupied   Gray: unknown",
        {20, map.rows + 82},
        cv::Scalar(40, 40, 40));
  label("Teal: outer   Orange: holes   Dots: vertices",
        {map.cols + 30, map.rows + 82},
        cv::Scalar(40, 40, 40));
  return preview;
}

void
save_image(std::filesystem::path const& path, cv::Mat const& image)
{
  if(!cv::imwrite(path.string(), image))
  {
    throw std::runtime_error("Cannot write " + path.string());
  }
}
} // namespace

int
main(int argc, char* const* argv)
{
  try
  {
    if(argc > 2 || (argc == 2 && std::string(argv[1]) == "--help"))
    {
      std::cout << "Usage: " << argv[0] << " [output_directory]\n";
      return argc > 2 ? 1 : 0;
    }
    std::filesystem::path const output =
        argc == 2 ? argv[1] : "examples/gridmap_polygon_output";
    std::filesystem::create_directories(output);
    auto const map = make_gridmap();
    save_image(output / "gridmap.png", map);
    // Read the generated image back to demonstrate the full image-to-polygon path.
    auto const input =
        cv::imread((output / "gridmap.png").string(), cv::IMREAD_GRAYSCALE);
    if(input.empty())
    {
      throw std::runtime_error("Cannot read generated gridmap");
    }
    cv::Mat free_mask;
    cv::compare(input, 255, free_mask, cv::CMP_EQ);
    auto const polygons = extract_polygons(free_mask);
    save_image(output / "free_mask.png", free_mask);
    save_image(output / "preview.png", make_preview(input, polygons));
    save_polygons(output / "polygons.json", polygons, input.size());
    std::cout << "Extracted " << polygons.size() << " free-space polygons\n";
    for(std::size_t i = 0; i < polygons.size(); ++i)
    {
      auto const& polygon = polygons[i];
      std::cout << "  P" << i << ": " << polygon.outer.size()
                << " outer vertices, " << polygon.holes.size() << " holes\n";
    }
    std::cout
        << "Saved gridmap.png, free_mask.png, preview.png and polygons.json to "
        << std::filesystem::absolute(output) << '\n';
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << "gridmap_to_polygon: " << error.what() << '\n';
    return 1;
  }
}
