#include <cmath>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fmsg = foxglove::messages;
using namespace std::chrono_literals;

// ---------- 1. 算法数据 ----------
struct Cell
{
  int row;
  int col;

  bool
  operator<(Cell const& other) const
  {
    return std::tie(row, col) < std::tie(other.row, other.col);
  }

  bool
  operator==(Cell const& other) const
  {
    return row == other.row && col == other.col;
  }
};

std::vector<Cell> const free_cells = {
    {.row = 0, .col = 1}, // 0
    {.row = 1, .col = 0}, // 1
    {.row = 1, .col = 1}, // 2
    {.row = 1, .col = 2}, // 3
    {.row = 2, .col = 1}, // 4
    {.row = 2, .col = 2}, // 5
    {.row = 3, .col = 2}, // 6
    {.row = 3, .col = 3}, // 7
    {.row = 3, .col = 1}, // 8
    {.row = 4, .col = 3}, // 9
};

struct CellId
{
  CellId() = default;
  explicit CellId(std::size_t _value)
    : value(_value)
  {}
  auto
  operator<=>(CellId const&) const = default;

  std::size_t value;
};
struct Edge
{
  Edge() = default;
  Edge(CellId _from, CellId _to)
    : from(_from)
    , to(_to)
  {}
  CellId from;
  CellId to;
};
std::vector<Edge> const tree_edges = {
    {0, 2},
    {1, 2},
    {2, 3},
    {2, 4},
    {4, 5},
    {5, 6},
    {6, 7},
    {6, 8},
    {7, 9},
};

enum class SubCellDirection : int8_t
{
  NW = 1,
  NE = 2,
  SW = 3,
  SE = 4,
};
enum class TreeDirection : int8_t
{
  N = 1,
  S = 2,
  W = 3,
  E = 4,
};

struct SubCell
{
  SubCell(SubCellDirection _direction, Cell _cell)
    : direction(_direction)
    , cell(_cell)
  {}
  SubCellDirection direction;
  Cell cell;
};

std::map<SubCellDirection, Cell>
subcells(Cell const& cell)
{
  return {
      {SubCellDirection::NW, Cell{.row = 2 * cell.row, .col = 2 * cell.col}},
      {SubCellDirection::NE,
       Cell{.row = 2 * cell.row, .col = 2 * cell.col + 1},},
      {SubCellDirection::SW,
       Cell{.row = 2 * cell.row + 1, .col = 2 * cell.col},},
      {SubCellDirection::SE,
       Cell{.row = 2 * cell.row + 1, .col = 2 * cell.col + 1},},
  };
}

TreeDirection
get_direction(Cell const& from, Cell const& to)
{
  int const dr = to.row - from.row;
  int const dc = to.col - from.col;

  if(dr == -1 && dc == 0)
  {
    return TreeDirection::N;
  }
  if(dr == 1 && dc == 0)
  {
    return TreeDirection::S;
  }
  if(dr == 0 && dc == -1)
  {
    return TreeDirection::W;
  }
  if(dr == 0 && dc == 1)
  {
    return TreeDirection::E;
  }

  throw std::runtime_error("Cells are not adjacent");
}

struct InnerEdge
{
  // 这里之所以用 wall 墙这个含义，是因为：
  // 如果某个宏单元在某个方向上存在树边，那么这个方向对应的子格内部连接要被“墙”阻断。
  TreeDirection wall;
  SubCellDirection a;
  SubCellDirection b;
};

// inner 代表的是：一个宏单元内部，4 个子格之间“可能存在的内部连接”。
/*
NW ─── NE
│       │
│       │
SW ─── SE

NW <-> NE   北侧内部边
SW <-> SE   南侧内部边
NW <-> SW   西侧内部边
NE <-> SE   东侧内部边

方向        对应的内部连接

N           NW <-> NE
S           SW <-> SE
W           NW <-> SW
E           NE <-> SE
// 这里自己一开始没有反应过来，相的是上下左右！实际上，这里的东南西北就是就是上下左右！这里描述的不上边的方向，而是边的位置！
// 边在北边，也就是上面。
*/
std::array<InnerEdge, 4> const inner_edges = {{
    {TreeDirection::N, SubCellDirection::NW, SubCellDirection::NE},
    {TreeDirection::S, SubCellDirection::SW, SubCellDirection::SE},
    {TreeDirection::W, SubCellDirection::NW, SubCellDirection::SW},
    {TreeDirection::E, SubCellDirection::NE, SubCellDirection::SE},
},};


// ---------- 2. 基础转换函数 ----------

fmsg::Point3
to_point(Cell cell)
{
  return fmsg::Point3{
      .x = static_cast<double>(cell.col),
      .y = -static_cast<double>(cell.row),
      .z = 0.0,
  };
}

fmsg::Point3
to_subcell_point(Cell const& cell, double z = 0.0)
{
  return {
      .x = cell.col * 0.5 - 0.25,
      .y = -cell.row * 0.5 + 0.25,
      .z = z,
  };
}

fmsg::Color
rgba(double r, double g, double b, double a = 1.0)
{
  fmsg::Color color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

fmsg::Pose
make_pose(double x, double y, double z)
{
  fmsg::Pose pose;

  pose.position = fmsg::Vector3{
      .x = x,
      .y = y,
      .z = z,
  };

  // 单位四元数：无旋转。
  pose.orientation = fmsg::Quaternion{
      .x = 0.0,
      .y = 0.0,
      .z = 0.0,
      .w = 1.0,
  };

  return pose;
}

fmsg::Timestamp
now()
{
  // elapsed 的意思通常是：已经过去的、已经消耗的时间。
  auto const elapsed = std::chrono::system_clock::now().time_since_epoch();

  auto const seconds =
      std::chrono::duration_cast<std::chrono::seconds>(elapsed);

  auto const nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed - seconds);

  fmsg::Timestamp timestamp;
  timestamp.sec = static_cast<std::uint32_t>(seconds.count());
  timestamp.nsec = static_cast<std::uint32_t>(nanos.count());
  return timestamp;
}

// ---------- 3. 创建一条线段 ----------
fmsg::LinePrimitive
make_line(fmsg::Point3 const& a,
          fmsg::Point3 const& b,
          fmsg::Color const& color,
          double width)
{
  fmsg::LinePrimitive line;
  line.type = fmsg::LinePrimitive::LineType::LINE_LIST;

  // 线的局部坐标与 map 坐标重合。
  line.pose = make_pose(0.0, 0.0, 0.0);

  line.points = {a, b};
  line.color = color;
  line.thickness = width;
  line.scale_invariant = true;

  return line;
}

fmsg::ArrowPrimitive
make_arrow(fmsg::Point3 const& a,
           fmsg::Point3 const& b,
           fmsg::Color const& color,
           double shaft_diameter = 0.03,
           double head_length = 0.15,
           double head_diameter = 0.08)
{
  double const dx = b.x - a.x;
  double const dy = b.y - a.y;

  double const length = std::hypot(dx, dy);
  double const yaw = std::atan2(dy, dx);

  fmsg::ArrowPrimitive arrow;

  arrow.pose = make_pose(a.x, a.y, a.z);
  // 绕 z 轴旋转 yaw
  arrow.pose->orientation->x = 0.0;
  arrow.pose->orientation->y = 0.0;
  arrow.pose->orientation->z = std::sin(yaw / 2.0);
  arrow.pose->orientation->w = std::cos(yaw / 2.0);

  arrow.shaft_length = std::max(0.0, length - head_length);
  arrow.shaft_diameter = shaft_diameter;

  arrow.head_length = std::min(head_length, length);
  arrow.head_diameter = head_diameter;

  arrow.color = color;

  return arrow;
}

// ---------- 4. 构造覆盖算法单元的矩形网格及其 2×2 子格 ----------

fmsg::SceneEntity
make_grid(std::vector<Cell> const& cells)
{
  fmsg::SceneEntity entity;
  entity.id = "grid";
  entity.frame_id = "map";
  entity.lifetime = fmsg::Duration{};

  if(cells.empty())
  {
    return entity;
  }

  auto const cell_color = rgba(0.45, 0.45, 0.45, 0.85);
  auto const subcell_color = rgba(0.65, 0.65, 0.65, 0.5);

  // 网格略低于节点和轨迹，减少重叠。
  constexpr double z = -0.03;

  auto const add_line = [&](double x1,
                            double y1,
                            double x2,
                            double y2,
                            fmsg::Color const& color,
                            double width)
  {
    entity.lines.push_back(make_line(fmsg::Point3{.x = x1, .y = y1, .z = z},
                                     fmsg::Point3{.x = x2, .y = y2, .z = z},
                                     color,
                                     width));
  };

  // 用 free_cells 的行列范围确定完整矩形，包括其中未占用的大格。
  int min_row = cells.front().row;
  int max_row = cells.front().row;
  int min_col = cells.front().col;
  int max_col = cells.front().col;
  for(auto const& cell : cells)
  {
    min_row = std::min(min_row, cell.row);
    max_row = std::max(max_row, cell.row);
    min_col = std::min(min_col, cell.col);
    max_col = std::max(max_col, cell.col);
  }

  auto const top_left = to_point(Cell{.row = min_row, .col = min_col});
  auto const bottom_right = to_point(Cell{.row = max_row, .col = max_col});
  double const left = top_left.x - 0.5;
  double const right = bottom_right.x + 0.5;
  double const top = top_left.y + 0.5;
  double const bottom = bottom_right.y - 0.5;

  // 整行、整列画线：粗线是大格边界，居中的细线将大格分成四个小格。
  for(double x = left; x < right; x += 1.0)
  {
    add_line(x, top, x, bottom, cell_color, 2.0);
    add_line(x + 0.5, top, x + 0.5, bottom, subcell_color, 1.0);
  }
  add_line(right, top, right, bottom, cell_color, 2.0);

  for(double y = top; y > bottom; y -= 1.0)
  {
    add_line(left, y, right, y, cell_color, 2.0);
    add_line(left, y - 0.5, right, y - 0.5, subcell_color, 1.0);
  }
  add_line(left, bottom, right, bottom, cell_color, 2.0);

  return entity;
}

// ---------- 5. 将闭环数据转换成可视化实体 ----------

fmsg::SceneEntity
make_cycle(std::vector<Cell> const& cells)
{
  fmsg::SceneEntity entity;
  entity.id = "cycle";
  entity.frame_id = "map";
  entity.lifetime = fmsg::Duration{};

  // palette 最常见的意思是：调色板、色板、颜色集合。
  std::array<fmsg::Color, 6> const palette = {
      rgba(0.12, 0.47, 0.71),
      rgba(1.00, 0.50, 0.05),
      rgba(0.17, 0.63, 0.17),
      rgba(0.84, 0.15, 0.16),
      rgba(0.58, 0.40, 0.74),
      rgba(0.09, 0.75, 0.81),
  };

  for(std::size_t i = 0; i < cells.size(); ++i)
  {
    auto const p = to_point(cells[i]);
    auto const color = palette[i % palette.size()];

    // A. 在当前位置放置节点标记。
    fmsg::SpherePrimitive sphere;
    sphere.pose = make_pose(p.x, p.y, 0.03);
    sphere.size = fmsg::Vector3{
        .x = 0.12,
        .y = 0.12,
        .z = 0.12,
    };
    sphere.color = color;
    entity.spheres.push_back(sphere);

    // B. 添加行列索引标签。
    fmsg::TextPrimitive text;
    text.pose = make_pose(p.x + 0.20, p.y + 0.12, 0.12);
    text.text = "(" + std::to_string(cells[i].row) + ", " +
                std::to_string(cells[i].col) + ")";

    text.billboard = true;
    text.scale_invariant = true;
    text.font_size = 16.0;
    text.color = rgba(0.25, 0.25, 0.25, 0.5);
    entity.texts.push_back(std::move(text));
  }

  return entity;
}

void
run(fmsg::SceneUpdate& update)
{
  std::map<CellId, std::set<TreeDirection>> tree_dir;
  fmsg::SceneEntity arrow_entity;
  arrow_entity.id = "arrow";
  arrow_entity.frame_id = "map";
  arrow_entity.lifetime = fmsg::Duration{};

  auto const red = rgba(0.8, 0., 0., 1.0);

  for(auto const& [u, v] : tree_edges)
  {
    auto const& from = free_cells[u.value];
    auto const& to = free_cells[v.value];
    tree_dir[u].insert(get_direction(from, to));
    tree_dir[v].insert(get_direction(to, from));
    auto const from_point = to_point(from);
    auto const to_position = to_point(to);
    arrow_entity.arrows.emplace_back(make_arrow(from_point, to_position, red));
  }
  update.entities.emplace_back(std::move(arrow_entity));


  fmsg::SceneEntity subs_cell_entity;
  subs_cell_entity.id = "subs_cell";
  subs_cell_entity.frame_id = "map";
  subs_cell_entity.lifetime = fmsg::Duration{};
  fmsg::Color const color = rgba(0.0, 0.0, 1.0, 0.2);

  fmsg::SceneEntity edge_entity;
  edge_entity.id = "edge_entity";
  edge_entity.frame_id = "map";
  edge_entity.lifetime = fmsg::Duration{};
  fmsg::Color const edge_entity_color = rgba(0.0, 1.0, 0.0, 1.0);


  std::map<Cell, std::vector<Cell>> adj;


  for(std::size_t cell_id = 0; cell_id < free_cells.size(); ++cell_id)
  {
    // 这里只是加入了子节点，但是子节点之间还未连通
    auto const subs = subcells(free_cells[cell_id]);

    for(auto const& [dir, node] : subs)
    {
      adj.try_emplace(node);
      // A. 在当前位置放置节点标记。
      fmsg::SpherePrimitive sphere;
      auto const point = to_subcell_point(node, 0.03);
      sphere.pose = make_pose(point.x, point.y, point.z);
      sphere.size = fmsg::Vector3{
          .x = 0.22,
          .y = 0.22,
          .z = 0.22,
      };
      sphere.color = color;

      // B. 添加行列索引标签。
      fmsg::TextPrimitive text;
      text.pose = make_pose(point.x + 0.20, point.y + 0.12, 0.12);
      text.text = "(" + std::to_string(node.row) + ", " +
                  std::to_string(node.col) + ")";
      text.billboard = true;
      text.scale_invariant = true;
      text.font_size = 8.0;
      text.color = rgba(0., 0., 0.5, 1.0);
      subs_cell_entity.texts.push_back(std::move(text));
      subs_cell_entity.spheres.emplace_back(sphere);
    }

    // 这里得到了里“每个子格节点可以走到哪些相邻子格”的连接关系。
    // 上面得到的只是四个节点，如果不考虑生成树，这里的连接方式是这样的：
    /*
      NW ─── NE
      │       │
      │       │
      SW ─── SE
    */
    for(auto const& edge : inner_edges)
    {
      // 如果这个方向上没有生成树边，那么这条内部连接就保留。
      if(!tree_dir.at(cell_id).contains(edge.wall))
      {
        Cell const& a = subs.at(edge.a);
        Cell const& b = subs.at(edge.b);

        adj[a].push_back(b);
        adj[b].push_back(a);


        edge_entity.lines.emplace_back(make_line(to_subcell_point(a, 0.03),
                                                 to_subcell_point(b, 0.03),
                                                 edge_entity_color,
                                                 2.0));
      }
    }
  }
  update.entities.emplace_back(std::move(subs_cell_entity));
  update.entities.emplace_back(std::move(edge_entity));


  for(auto const& edge : tree_edges)
  {
    Cell u{};
    Cell v{};
    Cell const& a_cell = free_cells.at(edge.from.value);
    Cell const& b_cell = free_cells.at(edge.to.value);

    std::array<std::pair<SubCellDirection, SubCellDirection>, 2> pairs;
    // 水平相邻
    if(a_cell.row == b_cell.row)
    {
      // 保证 u 在左，v 在右
      if(a_cell.col < b_cell.col)
      {
        u = a_cell;
        v = b_cell;
      }
      else
      {
        u = b_cell;
        v = a_cell;
      }

      pairs = {{
          {SubCellDirection::NE, SubCellDirection::NW},
          {SubCellDirection::SE, SubCellDirection::SW},
      },};
    }
    // 竖直相邻
    else
    {
      // 保证 u 在上，v 在下
      // 保证 u 在上，v 在下
      if(a_cell.row < b_cell.row)
      {
        u = a_cell;
        v = b_cell;
      }
      else
      {
        u = b_cell;
        v = a_cell;
      }

      pairs = {{
          {SubCellDirection::SW, SubCellDirection::NW},
          {SubCellDirection::SE, SubCellDirection::NE},
      },};
    }
    auto const su = subcells(u);
    auto const sv = subcells(v);

    for(auto const& [a, b] : pairs)
    {
      Cell const& ca = su.at(a);
      Cell const& cb = sv.at(b);

      adj[ca].push_back(cb);
      adj[cb].push_back(ca);
    }
  }

  for(auto const& [node, neighbors] : adj)
  {
    if(neighbors.size() != 2)
    {
      throw std::runtime_error(
          "invalid cycle degree at (" + std::to_string(node.row) + ", " +
          std::to_string(node.col) + "): " + std::to_string(neighbors.size()));
    }
  }

  fmsg::SceneEntity path_entity;
  path_entity.id = "path_entity";
  path_entity.frame_id = "map";
  path_entity.lifetime = fmsg::Duration{};
  fmsg::Color const path_entity_color = rgba(0.0, 0.0, 1.0, 1.0);

  Cell const head = adj.begin()->first;
  std::vector<Cell> path;
  path.push_back(head);

  std::optional<Cell> prev;
  Cell cur = head;

  // 这里本质上不是“寻找一个环”，因为前面已经通过 adj 构造出了一个度数为 2 的环图；这里仅仅沿着这个环顺序走一圈，把节点顺序提取出来。
  while(true)
  {
    // 这个 node 的度数为 2，每次都选择剩下的哪个
    Cell const& n0 = adj.at(cur)[0];
    Cell const& n1 = adj.at(cur)[1];

    Cell nxt{};
    // 排除刚刚走过来的 prev
    // 背后的前提非常重要：环中的每个节点度数都是 2。
    /*
        当前节点 cur
          /     \
        prev     nxt
    所以根本不需要做真正的路径搜索，只需要：排除来路，
                                     → 剩下的就是去路
    */
    if(!prev.has_value() || n0.row != prev->row || n0.col != prev->col)
    {
      nxt = n0;
    }
    else
    {
      nxt = n1;
    }

    // 回到起点，闭环完成
    if(nxt.row == head.row && nxt.col == head.col)
    {
      break;
    }

    path.push_back(nxt);

    prev = cur;
    cur = nxt;
    auto const from_point = to_subcell_point(prev.value());
    auto const to_position = to_subcell_point(nxt);
    path_entity.arrows.emplace_back(
        make_arrow(from_point, to_position, path_entity_color));
  }
  update.entities.emplace_back(std::move(path_entity));
}

int
main()
{
  foxglove::WebSocketServerOptions options;
  options.host = "0.0.0.0";
  options.port = 1977;

  auto server_result = foxglove::WebSocketServer::create(std::move(options));

  if(!server_result.has_value())
  {
    std::cerr << "Failed to create WebSocket server\n";
    return 1;
  }

  auto const server = std::move(server_result.value());

  auto channel_result = fmsg::SceneUpdateChannel::create("/coverage/debug");

  if(!channel_result.has_value())
  {
    std::cerr << "Failed to create SceneUpdate channel\n";
    return 1;
  }

  auto channel = std::move(channel_result.value());

  // 图形是静态的，构造一次即可。
  fmsg::SceneUpdate update;
  update.entities.push_back(make_grid(free_cells));
  update.entities.push_back(make_cycle(free_cells));
  run(update);

  std::cout << "Listening on port：" << options.port << '\n';

  // 教学示例每秒重发，便于客户端晚连接或重新连接。
  while(true)
  {
    auto const timestamp = now();

    for(auto& entity : update.entities)
    {
      entity.timestamp = timestamp;
    }

    auto const error = channel.log(update);

    if(error != foxglove::FoxgloveError::Ok)
    {
      std::cerr << "Failed to log scene: " << foxglove::strerror(error) << '\n';
    }

    std::this_thread::sleep_for(1s);
  }
}
