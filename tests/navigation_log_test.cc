#include "navigation_log.hh"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void
require(bool condition, char const* message)
{
  if(!condition)
  {
    throw std::runtime_error(message);
  }
}

std::string
record(std::string const& payload)
{
  return "21:43:12 455(84642)D/move: navigation result speed:" + payload;
}
} // namespace

int
main()
{
  try
  {
    auto sample = pdlog::parseNavigationLine(
        record("-0.0677626 -0.0937527 status:Navigating"));
    require(sample && std::abs(sample->linear_speed + 0.0677626) < 1e-12 &&
                std::abs(sample->angular_speed + 0.0937527) < 1e-12 &&
                sample->status == "Navigating",
            "signed speeds and state must match the source");
    sample =
        pdlog::parseNavigationLine("21:43:11 688(84642)D/move: navigation "
                                   "result speed:-0.1 -0.05 status:Escaping");
    require(sample && sample->linear_speed == -0.1 &&
                sample->angular_speed == -0.05 && sample->status == "Escaping",
            "escaping sample");
    for(auto const* state : {"Stuck", "FailEscape", "NewState"})
    {
      sample = pdlog::parseNavigationLine(
          record(std::string("0 0 status:") + state + "\r\n"));
      require(sample && sample->status == state && sample->linear_speed == 0 &&
                  sample->angular_speed == 0,
              "zero speeds and unknown state");
    }
    sample =
        pdlog::parseNavigationLine(record("+1e-2\t-2.5E-3 status:Navigating"));
    require(sample && sample->linear_speed == 0.01 &&
                sample->angular_speed == -0.0025,
            "scientific notation and whitespace");
    for(auto const* line :
        {"ordinary text",
         "21:43:12 455(1)D/move: other message",
         "21:43:12 455(1)D/other: navigation result speed:0 0 status:Stuck",
         "21:43:12 455(1)D/debug_bridge: {\"text\":\"(1)D/move: navigation "
         "result speed:0 0 status:Stuck\"}"})
    {
      require(!pdlog::parseNavigationLine(line), "ignore unrelated records");
    }
    for(auto const* invalid : {"",
                               "0 status:Stuck",
                               "0 0",
                               "0 0 status:",
                               "x 0 status:Stuck",
                               "nan 0 status:Stuck",
                               "0 inf status:Stuck",
                               "1e999 0 status:Stuck",
                               "0 0 status:Stuck unexpected",
                               "0-1 status:Stuck",
                               "0 0status:Stuck",
                               "+-1 0 status:Stuck"})
    {
      bool failed = false;
      try
      {
        pdlog::parseNavigationLine(record(invalid));
      }
      catch(std::runtime_error const&)
      {
        failed = true;
      }
      require(failed, "malformed navigation sample must be rejected");
    }

    pdlog::LogClock clock;
    require(clock.observe(record("0 0 status:Stuck")) == 78192455,
            "millisecond timestamp");
    require(clock.observe("21:43:11 688(1)D/move:") == 78191688,
            "out of order is not next day");
    require(clock.observe("21:43:08 554(1)D/move:") == 78188554,
            "earlier source time");
    for(auto const* invalid : {"",
                               "no timestamp",
                               "24:00:00 000(1)",
                               "00:60:00 000(1)",
                               "00:00:60 000(1)",
                               "00:00:00 1000(1)",
                               "00:00:00 abc(1)",
                               "00:00:00 123",
                               "xx:00:00 000(1)"})
    {
      require(!clock.observe(invalid),
              "invalid timestamp must not reuse prior time");
    }
    require(clock.observe("23:59:59 999(1)") == 86399999, "before midnight");
    require(clock.observe("00:00:00 005(1)") == 86400005, "midnight rollover");
    require(clock.observe("23:59:59 998(1)") == 86399998,
            "late previous-day sample");
    require(clock.observe("00:00:00  7(1)") == 86400007,
            "unpadded milliseconds");
    require(clock.observe("00:00:01 123(1)") == 86401123,
            "clock remains on new day");
    pdlog::LogClock reverse_clock;
    require(reverse_clock.observe("00:00:00 001(1)") == 1,
            "start after midnight");
    require(reverse_clock.observe("23:59:59 999(1)") == -1,
            "previous day before initial sample");
    std::cout << "navigation_log_test passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
