#include "coverage_path_planning/viz/converters.hh"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace planning_viz::detail
{
namespace msg = foxglove::messages;

msg::Timestamp
to_message_timestamp(std::uint64_t stamp_ns)
{
  constexpr std::uint64_t ns_per_second = 1'000'000'000;
  auto const seconds = stamp_ns / ns_per_second;
  if(seconds > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::out_of_range("timestamp exceeds Foxglove Timestamp range");
  }
  return msg::Timestamp{
      .sec = static_cast<std::uint32_t>(seconds),
      .nsec = static_cast<std::uint32_t>(stamp_ns % ns_per_second),};
}

msg::SceneUpdate
make_deletion(std::string_view entity_id, std::uint64_t stamp_ns)
{
  msg::SceneEntityDeletion deletion;
  deletion.timestamp = to_message_timestamp(stamp_ns);
  deletion.type =
      msg::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
  deletion.id = std::string(entity_id);

  msg::SceneUpdate update;
  update.deletions.push_back(std::move(deletion));
  return update;
}

msg::SceneUpdate
make_line_update(Points3 points,
                 std::string_view entity_id,
                 std::string_view frame_id,
                 std::uint64_t stamp_ns,
                 msg::Color color,
                 double width_m,
                 bool closed)
{
  auto const timestamp = to_message_timestamp(stamp_ns);
  if(points.empty())
  {
    return make_deletion(entity_id, stamp_ns);
  }
  if(points.size() < (closed ? 3U : 2U))
  {
    throw std::invalid_argument("too few points; clearing previous geometry");
  }
  if(!std::isfinite(width_m) || width_m <= 0.0)
  {
    throw std::invalid_argument(
        "invalid line width; clearing previous geometry");
  }
  for(auto const& point : points)
  {
    if(!point.allFinite())
    {
      throw std::invalid_argument(
          "non-finite point; clearing previous geometry");
    }
  }

  msg::LinePrimitive line;
  line.type = closed ? msg::LinePrimitive::LineType::LINE_LOOP
                     : msg::LinePrimitive::LineType::LINE_STRIP;
  line.thickness = width_m;
  line.scale_invariant = false;
  line.color = color;

  msg::Pose identity;
  identity.position = msg::Vector3{0.0, 0.0, 0.0};
  identity.orientation = msg::Quaternion{0.0, 0.0, 0.0, 1.0};
  line.pose = identity;

  line.points.reserve(points.size());
  for(auto const& point : points)
  {
    line.points.push_back(msg::Point3{point.x(), point.y(), point.z()});
  }

  msg::SceneEntity entity;
  entity.id = std::string(entity_id);
  entity.frame_id = std::string(frame_id);
  entity.timestamp = timestamp;
  entity.lifetime = msg::Duration{0, 0};
  entity.frame_locked = false;
  entity.lines.push_back(std::move(line));

  msg::SceneUpdate update;
  update.entities.push_back(std::move(entity));
  return update;
}

msg::SceneUpdate
make_water_leak_update(WaterLeakView const& view,
                       std::string_view frame_id,
                       std::uint64_t stamp_ns)
{
  msg::SceneUpdate update;
  // Delete even when the current mask is empty, including after replay resets.
  for(auto const* const id : {"washcloth", "squeegee", "wet_regions"})
  {
    update.deletions.push_back(make_deletion(id, stamp_ns).deletions.front());
  }
  if(view.cloth.empty())
  {
    return update;
  }
  if(view.cloth.size() != 4 || view.squeegee.size() < 2 ||
     view.wet_triangles.size() % 3 != 0 ||
     view.wet_boundary_lines.size() % 2 != 0)
  {
    throw std::invalid_argument("invalid water leak geometry");
  }
  for(auto const points : {view.wet_triangles, view.wet_boundary_lines})
  {
    for(auto const& p : points)
    {
      if(!p.allFinite())
      {
        throw std::invalid_argument("non-finite wet region vertex");
      }
    }
  }
  auto cloth = make_line_update(view.cloth,
                                "washcloth",
                                frame_id,
                                stamp_ns,
                                msg::Color{0.1, 0.9, 0.3, 1.0},
                                0.004,
                                true)
                   .entities.front();
  auto squeegee =
      make_line_update(view.squeegee,
                       "squeegee",
                       frame_id,
                       stamp_ns,
                       view.suction ? msg::Color{0.0, 0.9, 1.0, 1.0}
                                    : msg::Color{0.5, 0.5, 0.5, 1.0},
                       0.008,
                       false)
          .entities.front();
  // Lift tool overlays slightly above the water to avoid coplanar flicker.
  for(auto* entity : {&cloth, &squeegee})
  {
    for(auto& p : entity->lines.front().points)
    {
      p.z += 0.015;
    }
  }
  msg::Pose identity;
  identity.position = msg::Vector3{0.0, 0.0, 0.0};
  identity.orientation = msg::Quaternion{0.0, 0.0, 0.0, 1.0};
  msg::TriangleListPrimitive cloth_fill;
  cloth_fill.pose = identity;
  cloth_fill.color = msg::Color{0.1, 0.9, 0.3, 0.35};
  // The rectangle is clockwise; reverse triangle winding to face +Z.
  for(auto const i : {0, 2, 1, 0, 3, 2})
  {
    auto const& p = view.cloth[i];
    cloth_fill.points.push_back({p.x(), p.y(), p.z() + 0.012});
  }
  cloth.triangles.push_back(std::move(cloth_fill));
  update.entities.push_back(std::move(cloth));
  update.entities.push_back(std::move(squeegee));

  msg::SceneEntity water;
  water.id = "wet_regions";
  water.frame_id = std::string(frame_id);
  water.timestamp = to_message_timestamp(stamp_ns);
  water.lifetime = msg::Duration{0, 0};
  if(!view.wet_triangles.empty())
  {
    msg::TriangleListPrimitive mesh;
    mesh.pose = identity;
    mesh.color = msg::Color{1.0, 0.1, 0.05, 0.85};
    mesh.points.reserve(view.wet_triangles.size());
    for(auto const& p : view.wet_triangles)
    {
      mesh.points.push_back({p.x(), p.y(), p.z()});
    }
    water.triangles.push_back(std::move(mesh));
  }
  if(!view.wet_boundary_lines.empty())
  {
    msg::LinePrimitive boundary;
    boundary.pose = identity;
    boundary.type = msg::LinePrimitive::LineType::LINE_LIST;
    boundary.color = msg::Color{1.0, 0.0, 0.0, 1.0};
    boundary.thickness = 2.0;
    boundary.scale_invariant = true;
    boundary.points.reserve(view.wet_boundary_lines.size());
    for(auto const& p : view.wet_boundary_lines)
    {
      boundary.points.push_back({p.x(), p.y(), p.z()});
    }
    water.lines.push_back(std::move(boundary));
  }
  update.entities.push_back(std::move(water));
  return update;
}

} // namespace planning_viz::detail
