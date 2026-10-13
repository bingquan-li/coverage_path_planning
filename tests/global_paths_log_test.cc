#include "local_map_log.hh"

#include <nlohmann/json.hpp>

#include <iostream>
#include <stdexcept>

namespace
{
void
require(bool condition, char const* message)
{
  if(!condition)
    throw std::runtime_error(message);
}
template <class F>
void
fails(F action)
{
  try
  {
    action();
  }
  catch(std::exception const&)
  {
    return;
  }
  throw std::runtime_error("expected invalid global_paths to fail");
}
nlohmann::json
sample()
{
  return {{"type", "global_paths"},
          {"number_of_segment", 0},
          {"total_number_of_segments", 1},
          {"data",
           {{-2.66, -1.47}, {-2.64, -1.42}, {-2.66, -1.47}, {-3.19, -2.11}}}};
}
} // namespace

int
main()
{
  try
  {
    using pdlog::GlobalPathSegment;
    auto const s = sample();
    auto segment = pdlog::parseGlobalPathsLine(
        "21:40:43 198(84602)D/debug_bridge: " + s.dump() + "\r\n");
    require(segment && segment->number == 0 && segment->total == 1,
            "segment metadata");
    require(segment->vertices.size() == 4 &&
                segment->vertices[0] == segment->vertices[2],
            "preserve repeated points");
    require(segment->vertices.front()[0] == -2.66F &&
                segment->vertices.back()[1] == -2.11F,
            "world coordinates and open endpoints");
    require(!pdlog::parseGlobalPathsLine("ordinary global_paths diagnostic"),
            "ignore plain diagnostics");
    auto other = s;
    other["type"] = "foot_print";
    require(!pdlog::parseGlobalPathsLine(other.dump()), "exact type matching");
    for(auto key : {"data", "number_of_segment", "total_number_of_segments"})
    {
      auto bad = s;
      bad.erase(key);
      fails([&] { pdlog::parseGlobalPathsLine(bad.dump()); });
    }
    for(auto bad_total : {nlohmann::json(0),
                          nlohmann::json(-1),
                          nlohmann::json(1.5),
                          nlohmann::json(true),
                          nlohmann::json(4294967296ULL)})
    {
      auto bad = s;
      bad["total_number_of_segments"] = bad_total;
      fails([&] { pdlog::parseGlobalPathsLine(bad.dump()); });
    }
    for(auto bad_index : {nlohmann::json(-1),
                          nlohmann::json(1),
                          nlohmann::json(0.5),
                          nlohmann::json(false)})
    {
      auto bad = s;
      bad["number_of_segment"] = bad_index;
      fails([&] { pdlog::parseGlobalPathsLine(bad.dump()); });
    }
    for(auto bad_data : {nlohmann::json("bad"),
                         nlohmann::json{{1, 2, 3}},
                         nlohmann::json{{true, 2}},
                         nlohmann::json{{1e100, 2}}})
    {
      auto bad = s;
      bad["data"] = bad_data;
      fails([&] { pdlog::parseGlobalPathsLine(bad.dump()); });
    }
    fails([] { pdlog::parseGlobalPathsLine("prefix {bad global_paths JSON"); });
    auto empty = s;
    empty["data"] = nlohmann::json::array();
    require(pdlog::parseGlobalPathsLine(empty.dump())->vertices.empty(),
            "empty path clears display");

    pdlog::GlobalPathsAssembler assembler;
    auto paths = assembler.push(*segment);
    require(paths && paths->size() == 1 && !assembler.pending(),
            "single segment complete immediately");
    require(!assembler.push(GlobalPathSegment{0, 3, {{0, 0}, {1, 1}}}),
            "wait for all segments");
    require(!assembler.push(GlobalPathSegment{2, 3, {{20, 20}}}),
            "accept nonzero segments out of order");
    require(!assembler.push(GlobalPathSegment{2, 3, {{21, 21}}}),
            "replace repeated segment within current batch");
    paths = assembler.push(GlobalPathSegment{1, 3, {}});
    require(paths && paths->size() == 3 && paths->at(1).empty() &&
                paths->at(2)[0][0] == 21,
            "ordered independent strips with empty and single-point segments");
    fails([&] { assembler.push(GlobalPathSegment{1, 2, {{1, 1}}}); });
    require(!assembler.pending(), "orphan segment cannot reuse previous batch");
    assembler.push(GlobalPathSegment{0, 2, {{0, 0}}});
    fails([&] { assembler.push(GlobalPathSegment{1, 3, {{1, 1}}}); });
    require(!assembler.pending(), "bad count invalidates partial batch");
    assembler.push(GlobalPathSegment{0, 2, {{0, 0}}});
    paths = assembler.push(GlobalPathSegment{0, 1, {{7, 7}, {8, 8}}});
    require(paths && paths->size() == 1 && paths->at(0)[0][0] == 7,
            "new segment 0 starts a new plan");
    paths = assembler.push(GlobalPathSegment{0, 1, {}});
    require(paths && paths->front().empty(), "complete empty update");
    std::cout << "global_paths_log_test passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
