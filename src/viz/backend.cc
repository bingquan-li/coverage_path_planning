// Contract: init/shutdown never overlap any publishing call.

#include "coverage_path_planning/viz/backend.hh"
#include "coverage_path_planning/viz/converters.hh"

#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace planning_viz::detail {
namespace {

namespace fmsg = foxglove::messages;
using Error = foxglove::FoxgloveError;

// A minimal fallback logger. Replace with the project's logger as needed.
// Throttles ALL publishing errors together to at most one line per second.
void report_error(char const *operation, char const *message) noexcept {
  using namespace std::chrono;
  auto const now =
      duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
          .count();
  static std::atomic<std::int64_t> next_log{
      std::numeric_limits<std::int64_t>::min()};
  auto next = next_log.load(std::memory_order_relaxed);
  if (now < next || !next_log.compare_exchange_strong(
                        next, now + 1'000'000'000, std::memory_order_relaxed)) {
    return;
  }
  std::fprintf(stderr, "[planning_viz] %s: %s\n", operation, message);
}

void append_error(std::string &target, char const *operation,
                  char const *message) {
  if (!target.empty()) {
    target += "; ";
  }
  target += operation;
  target += ": ";
  target += message;
}

// Only initialization uses exceptions to unwind a series of SDK create calls.
// The public init() catches them and returns false with an error string.
template <typename T>
T take_or_throw(foxglove::FoxgloveResult<T> result, char const *operation) {
  if (!result.has_value()) {
    throw std::runtime_error(std::string(operation) + ": " +
                             foxglove::strerror(result.error()));
  }
  return std::move(result.value());
}

// Used both for initialization rollback and explicit shutdown.
// Continue to the writer even if stopping the server reports an error.
void close_outputs(std::optional<foxglove::WebSocketServer> &server,
                   std::optional<foxglove::McapWriter> &writer,
                   std::string &error) {
  auto const attempt = [&](char const *operation, auto &&action) {
    try {
      auto const code = action();
      if (code != Error::Ok) {
        append_error(error, operation, foxglove::strerror(code));
      }
    } catch (std::exception const &ex) {
      append_error(error, operation, ex.what());
    } catch (...) {
      append_error(error, operation, "unknown exception");
    }
  };

  if (server) {
    attempt("stop websocket", [&] { return server->stop(); });
    server.reset();
  }
  if (writer) {
    attempt("close mcap", [&] { return writer->close(); });
    writer.reset();
  }
}

std::uint64_t resolve_stamp(std::optional<std::uint64_t> stamp) {
  if (!stamp) {
    auto const now = duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    if (now < 0) {
      throw std::invalid_argument("negative timestamp");
    }
    stamp = static_cast<std::uint64_t>(now);
  }
  // Validate before comparing timestamps or deciding whether to clear geometry.
  (void)to_message_timestamp(*stamp);
  return *stamp;
}

} // namespace

// Synchronous: consumes points and ctx during this call; saves neither view.
void Backend::publish_line(SceneChannel &channel, PublicationState &state,
                           Config const &config, Points3 points,
                           DrawContext ctx, std::string_view entity_id,
                           fmsg::Color color, double width_m, bool closed,
                           char const *operation) noexcept {
  try {
    std::scoped_lock const lock(state.mutex);
    auto const stamp = resolve_stamp(ctx.stamp_ns);
    if (state.last_stamp && stamp <= *state.last_stamp) {
      report_error(operation,
                   "timestamp must strictly increase for this topic");
      return;
    }
    std::string_view const frame = ctx.frame_id.empty()
                                       ? std::string_view(config.default_frame)
                                       : ctx.frame_id;

    fmsg::SceneUpdate message;
    try {
      message = make_line_update(points, entity_id, frame, stamp, color,
                                 width_m, closed);
    } catch (std::invalid_argument const &ex) {
      report_error(operation, ex.what());
      message = make_deletion(entity_id, stamp);
    }

    auto const error = channel.log(message, stamp);
    if (error != Error::Ok) {
      report_error(operation, foxglove::strerror(error));
    } else {
      state.last_stamp = stamp;
    }
  } catch (std::exception const &ex) {
    report_error(operation, ex.what());
  } catch (...) {
    report_error(operation, "unknown exception");
  }
}

bool Backend::init(Config const &config, std::string &error) {
  error.clear();
  if (initialized) {
    error = "planning_viz is already initialized; call shutdown first";
    return false;
  }
  if (!config.enabled) {
    initialized = true;
    return true;
  }
  if (config.default_frame.empty()) {
    error = "default_frame must not be empty";
    return false;
  }
  if (!config.websocket_enabled && !config.mcap_path) {
    error = "enable websocket or provide an mcap_path";
    return false;
  }
  if (config.mcap_path && config.mcap_path->empty()) {
    error = "mcap_path must not be empty when configured";
    return false;
  }

  std::unique_ptr<Resources> candidate;
  try {
    candidate = std::make_unique<Resources>();
    candidate->config = config;
    auto &r = *candidate;

    r.global_path.emplace(
        take_or_throw(SceneChannel::create("/planning/global_path", r.context),
                      "create global_path"));
    r.local_path.emplace(
        take_or_throw(SceneChannel::create("/planning/local_path", r.context),
                      "create local_path"));
    r.history_path_channel.emplace(take_or_throw(
        fmsg::PosesInFrameChannel::create("/planning/history_path", r.context),
        "create path"));
    r.trajectory.emplace(
        take_or_throw(SceneChannel::create("/planning/trajectory", r.context),
                      "create trajectory"));
    r.footprint.emplace(
        take_or_throw(SceneChannel::create("/planning/footprint", r.context),
                      "create footprint"));
    r.water_leak.emplace(
        take_or_throw(SceneChannel::create("/planning/water_leak", r.context),
                      "create water_leak"));
    r.transform.emplace(
        take_or_throw(fmsg::FrameTransformChannel::create("/tf", r.context),
                      "create transform"));
    r.odometry.emplace(take_or_throw(
        fmsg::OdometryChannel::create("/planning/odometry", r.context),
        "create odometry"));

    if (config.mcap_path) {
      foxglove::McapWriterOptions options;
      options.context = r.context;
      options.path = *r.config.mcap_path;
      options.truncate = false;
      r.writer.emplace(take_or_throw(foxglove::McapWriter::create(options),
                                     "create mcap writer"));
    }
    if (config.websocket_enabled) {
      foxglove::WebSocketServerOptions options;
      options.context = r.context;
      options.host = r.config.host;
      options.port = r.config.port;
      r.server.emplace(
          take_or_throw(foxglove::WebSocketServer::create(std::move(options)),
                        "create websocket server"));
    }

    resources = std::move(candidate);
    initialized = true;
    return true;
  } catch (std::exception const &ex) {
    error = std::string("initialize planning_viz: ") + ex.what();
  } catch (...) {
    error = "initialize planning_viz: unknown exception";
  }

  if (candidate) {
    close_outputs(candidate->server, candidate->writer, error);
  }
  // candidate destruction releases channels, then their Context.
  // A newly created MCAP file is retained, not deleted on failure.
  return false;
}

bool Backend::shutdown(std::string &error) {
  error.clear();
  // Precondition: all publishing threads have stopped and been joined.
  auto old = std::move(resources);
  initialized = false;
  if (old) {
    close_outputs(old->server, old->writer, error);
  }
  return error.empty();
}

void Backend::global_path(Points3 points, DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_line(*r.global_path, r.global_path_state, r.config, points, ctx,
               "path", fmsg::Color{0.2, 0.8, 0.3, 1.0}, 0.04, false,
               "globalPath");
}

void Backend::local_path(Points3 points, DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_line(*r.local_path, r.local_path_state, r.config, points, ctx, "path",
               fmsg::Color{.r = 1.0, .g = 0.6, .b = 0.1, .a = 1.0}, 0.05, false,
               "localPath");
}

void Backend::history_path(Points3 segments, DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_path(*r.history_path_channel, r.path_state, r.config, segments, ctx,
               "path");
}

void Backend::trajectory(Points3 points, DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_line(*r.trajectory, r.trajectory_state, r.config, points, ctx,
               "trajectory", fmsg::Color{0.9, 0.7, 0.1, 1.0}, 0.025, false,
               "trajectory");
}

void Backend::trajectory_footprints(std::span<Points3 const> polygons,
                                    DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_lines(*r.trajectory, r.trajectory_state, r.config, polygons, ctx,
                "trajectory", fmsg::Color{0.9, 0.7, 0.1, 1.0}, 0.005, true,
                "trajectoryFootprints");
}

void Backend::publish_lines(SceneChannel &channel, PublicationState &state,
                            Config const &config,
                            std::span<Points3 const> lines, DrawContext ctx,
                            std::string_view entity_id, fmsg::Color color,
                            double width_m, bool closed,
                            char const *operation) noexcept {
  try {
    std::scoped_lock const lock(state.mutex);
    auto const stamp = resolve_stamp(ctx.stamp_ns);
    if (state.last_stamp && stamp <= *state.last_stamp) {
      report_error(operation, "timestamp must strictly increase");
      return;
    }
    auto const frame = ctx.frame_id.empty()
                           ? std::string_view(config.default_frame)
                           : ctx.frame_id;
    auto message = make_deletion(entity_id, stamp);
    try {
      for (auto const points : lines) {
        if (points.empty()) {
          throw std::invalid_argument("empty line; clearing previous geometry");
        }
        auto outline = make_line_update(points, entity_id, frame, stamp, color,
                                        width_m, closed);
        if (message.entities.empty()) {
          message = std::move(outline);
          message.entities.front().lines.reserve(lines.size());
        } else {
          message.entities.front().lines.push_back(
              std::move(outline.entities.front().lines.front()));
        }
      }
    } catch (std::invalid_argument const &ex) {
      report_error(operation, ex.what());
      message = make_deletion(entity_id, stamp);
    }
    auto const error = channel.log(message, stamp);
    if (error == Error::Ok) {
      state.last_stamp = stamp;
    } else {
      report_error(operation, foxglove::strerror(error));
    }
  } catch (std::exception const &ex) {
    report_error(operation, ex.what());
  } catch (...) {
    report_error(operation, "unknown exception");
  }
}

void Backend::footprint(Points3 vertices, DrawContext ctx) {
  if (!resources) {
    return;
  }
  auto &r = *resources;
  publish_line(*r.footprint, r.footprint_state, r.config, vertices, ctx,
               "footprint", fmsg::Color{0.2, 0.6, 1.0, 1.0}, 0.03, true,
               "footprint");
}

void Backend::water_leak(WaterLeakView const &view, DrawContext ctx) {
  if (!resources) {
    return;
  }
  try {
    auto &r = *resources;
    std::scoped_lock const lock(r.water_leak_state.mutex);
    auto const stamp = resolve_stamp(ctx.stamp_ns);
    if (r.water_leak_state.last_stamp &&
        stamp <= *r.water_leak_state.last_stamp) {
      report_error("water_leak", "timestamp must strictly increase");
      return;
    }
    auto const frame = ctx.frame_id.empty()
                           ? std::string_view(r.config.default_frame)
                           : ctx.frame_id;
    fmsg::SceneUpdate message;
    try {
      message = make_water_leak_update(view, frame, stamp);
    } catch (std::invalid_argument const &ex) {
      report_error("water_leak", ex.what());
      message = make_water_leak_update({} frame, stamp);
    }
    auto const error = r.water_leak->log(message, stamp);
    if (error == Error::Ok) {
      r.water_leak_state.last_stamp = stamp;
    } else {
      report_error("water_leak", foxglove::strerror(error));
    }
  } catch (std::exception const &ex) {
    report_error("water_leak", ex.what());
  }
}

void Backend::odometry(RobotOdometry const &state, DrawContext ctx) {
  if (!resources) {
    return;
  }
  try {
    auto &r = *resources;
    std::scoped_lock const lock(r.odometry_state.mutex);
    auto const stamp = resolve_stamp(ctx.stamp_ns);
    if (r.odometry_state.last_stamp && stamp <= *r.odometry_state.last_stamp) {
      report_error("odometry", "timestamp must strictly increase");
      return;
    }
    if (!state.position.allFinite() || !std::isfinite(state.yaw) ||
        (state.linear_velocity && !std::isfinite(*state.linear_velocity)) ||
        (state.angular_velocity && !std::isfinite(*state.angular_velocity))) {
      report_error("odometry", "non-finite robot state");
      return;
    }

    fmsg::Odometry message;
    message.timestamp = to_message_timestamp(stamp);
    message.frame_id = ctx.frame_id.empty() ? r.config.default_frame
                                            : std::string(ctx.frame_id);
    message.body_frame_id = "base_link";
    fmsg::Pose pose;
    pose.position = fmsg::Vector3{state.position.x(), state.position.y(),
                                  state.position.z(),};
    pose.orientation = fmsg::Quaternion{0.0, 0.0, std::sin(state.yaw / 2.0),
                                        std::cos(state.yaw / 2.0),};
    message.pose = pose;

    fmsg::FrameTransform transform;
    transform.timestamp = message.timestamp;
    transform.parent_frame_id = message.frame_id;
    transform.child_frame_id = message.body_frame_id;
    transform.translation = pose.position;
    transform.rotation = pose.orientation;
    auto const transform_error = r.transform->log(transform, stamp);
    if (transform_error != Error::Ok) {
      report_error("transform", foxglove::strerror(transform_error));
      return;
    }

    if (state.linear_velocity) {
      message.linear_velocity = fmsg::Vector3{*state.linear_velocity, 0.0, 0.0};
    }
    if (state.angular_velocity) {
      message.angular_velocity =
          fmsg::Vector3{0.0, 0.0, *state.angular_velocity};
    }

    auto const error = r.odometry->log(message, stamp);
    if (error != Error::Ok) {
      report_error("odometry", foxglove::strerror(error));
    } else {
      r.odometry_state.last_stamp = stamp;
    }
  } catch (std::exception const &ex) {
    report_error("odometry", ex.what());
  }
}

void Backend::publish_path(foxglove::messages::PosesInFrameChannel &channel,
                           PublicationState &state, Config const &config,
                           Points3 poses, DrawContext ctx,
                           char const *operation) {
  auto const stamp = resolve_stamp(ctx.stamp_ns);
  if (poses.empty()) {
    auto const error = channel.log(fmsg::PosesInFrame(), stamp);
    if (error == Error::Ok) {
      state.last_stamp = stamp;
      return;
    }

    report_error(operation, foxglove::strerror(error));
    return;
  }

  fmsg::PosesInFrame path;
  auto const frame = ctx.frame_id.empty()
                         ? std::string_view(config.default_frame)
                         : ctx.frame_id;
  path.frame_id = frame;
  path.timestamp = to_message_timestamp(stamp);
  for (auto const &point : poses) {
    if (!point.allFinite()) {
      throw std::invalid_argument("non-finite point");
    }
    fmsg::Pose frame_pose;
    // frame_pose.frame_id = frame;
    // frame_pose.timestamp =
    // ?? 为什么这里错了
    frame_pose.position = fmsg::Vector3{point.x(), point.y(), 0.0};
    double const yaw = point.z();
    // Foxglove 的字段顺序是 x、y、z、w。
    frame_pose.orientation =
        fmsg::Quaternion{0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
    path.poses.push_back(frame_pose);
  }
  auto const error = channel.log(path, stamp);
  if (error == Error::Ok) {
    state.last_stamp = stamp;
  } else {
    report_error(operation, foxglove::strerror(error));
  }
}

void Backend::clear_local_path(DrawContext ctx) {
  // Empty input is handled as MATCHING_ID deletion by publishLine().
  // Use a timestamp later than the entity being removed.
  local_path(Points3{}, ctx);
}

} // namespace planning_viz::detail
