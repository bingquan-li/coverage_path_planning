#include "log_parser.hh"

#include <cmath>
#include <stdexcept>
#include <string>

namespace show
{
using namespace std::chrono_literals;

LogParser::LogParser(std::filesystem::path const& path)
  : file_(path)
{
  if(!file_)
  {
    throw std::runtime_error("cannot open log file: " + path.string());
  }
}

bool
LogParser::next_frame(Frame& frame)
{
  Event event;
  if(pending_pose_)
  {
    frame = pending_pose_->frame;
    pending_pose_.reset();
  }
  else
  {
    do
    {
      if(!next(event))
      {
        return false;
      }
    }
    while(!event.pose_changed);
    frame = event.frame;
  }

  while(next(event))
  {
    if(event.pose_changed)
    {
      pending_pose_ = event;
      return true;
    }
    frame.state.linear_velocity = event.frame.state.linear_velocity;
    frame.state.angular_velocity = event.frame.state.angular_velocity;
    ++frame.velocity_count;
  }
  return true;
}

bool
LogParser::next(Event& event)
{
  std::string line;
  while(std::getline(file_, line))
  {
    std::smatch time_match;
    if(!std::regex_search(line, time_match, clock_regex_))
    {
      continue;
    }

    try
    {
      auto const hour = std::stoi(time_match[1]);
      auto const minute = std::stoi(time_match[2]);
      auto const second = std::stoi(time_match[3]);
      auto const millisecond = std::stoi(time_match[4]);
      if(hour > 23 || minute > 59 || second > 59 || millisecond > 999)
      {
        continue;
      }
      auto clock = std::chrono::milliseconds(
          ((hour * 60 + minute) * 60 + second) * 1000 + millisecond);
      if(last_time_ && clock + day_offset_ + 12h < *last_time_)
      {
        day_offset_ += 24h;
      }
      clock += day_offset_;
      if(last_time_ && clock < *last_time_)
      {
        clock = *last_time_;
      }

      std::smatch match;
      if(std::regex_search(line, match, pose_regex_))
      {
        RobotState next_state;
        next_state.position = Eigen::Vector3d{std::stod(match[1]),
                                              std::stod(match[2]),
                                              std::stod(match[3])};
        next_state.yaw = std::stod(match[4]);
        if(!next_state.position.allFinite() || !std::isfinite(next_state.yaw))
        {
          continue;
        }
        state_ = std::move(next_state);
        has_pose_ = true;
        event = Event{Frame{state_, clock}, true};
      }
      else if(has_pose_ && std::regex_search(line, match, velocity_regex_))
      {
        auto const linear = std::stod(match[1]);
        auto const angular = std::stod(match[2]);
        if(!std::isfinite(linear) || !std::isfinite(angular))
        {
          continue;
        }
        state_.linear_velocity = linear;
        state_.angular_velocity = angular;
        event = Event{Frame{state_, clock}, false};
      }
      else
      {
        continue;
      }
      last_time_ = clock;
      return true;
    }
    catch(std::exception const&)
    {
      // Ignore malformed records and keep the most recent valid state.
    }
  }
  return false;
}

} // namespace show
