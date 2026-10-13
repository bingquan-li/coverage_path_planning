"""阶段 3 的练习文件：拓扑信息的跨帧传播（论文 III-C，文档 4.4）。

只需实现下面八个函数，不要改其他文件。推荐按声明顺序写：

    python globally_guided/check3.py --stage advance    # advance_world
    python globally_guided/check3.py --stage shift      # shift_state
    python globally_guided/check3.py --stage carry      # carry_connector
    python globally_guided/check3.py --stage reintro    # reintroduce_states
    python globally_guided/check3.py --stage warm       # build_prm_warm
    python globally_guided/check3.py --stage guardmap   # match_guards
    python globally_guided/check3.py --stage segid      # assign_segment_ids
    python globally_guided/check3.py --stage trajid     # assign_trajectory_ids
    python globally_guided/check3.py --stage select     # select_guidance
    python globally_guided/check3.py --stage closed     # 多帧闭环
    python globally_guided/check3.py                    # 全跑

=====================================================================
先读这段：阶段 3 的难点和前两阶段都不一样
=====================================================================

阶段 1 是**纯谓词**：错了就是数学错了，一测就现形。
阶段 2 是**增量维护一个数据结构**：错了程序照跑，少找到一个通道，
要靠四条不变量和统计量来抓。

阶段 3 是**维护一个跨帧的身份映射**：错了的典型症状是
「每一帧单独看全对，连着跑起来机器人在拖」。单帧断言抓不到它，
所以 check3.py 里出现了一类新的用例：**时序断言**——
连续跑 N 帧，断言某个编号全程不变。

你在前两阶段犯的错，在这里的同形版本：

  以前                            阶段 3 的同形错误
  ------------------------------  --------------------------------------------
  a - b 方向写反                   障碍物该 +h 写成了 -h（两者方向相反！）
  节点 id / 下标 / State 混用        节点 id 和 segment ID 混用
  空分支没先挡                     第一帧没有 prev，prev_graph 是 None
  上界 clamp 成 10 这类拍脑袋       过期判定写成 t < h 而不是 t <= h
  自己手算长度不用 Trajectory.length  自己重写 UVD 而不用 uvd_equivalent

---------------------------------------------------------------
一个必须先想清楚的符号问题
---------------------------------------------------------------

两帧之间过了 h 秒。这件事在两个地方同时体现，而且**符号相反**：

  障碍物（世界的状态）： p -> p + v*h      它真的往前走了
  节点（未来的计划）：   t -> t - h        它离「现在」更近了

两者说的是同一件事：**时间原点往前挪了 h**。每一帧都有自己的 t = 0。
把符号弄反了程序不会报错，只是节点越排越晚、最后全部溢出规划窗口。
"""

from __future__ import annotations

import dataclasses

import numpy as np
from graph import Graph
from tracking import IdAllocator, segment_trajectory
from world import State, Trajectory, World


def advance_world(world: World, h: float) -> World:
    """把世界往前推 h 秒，返回一个**新的** World。

    这是整个阶段最短的一个函数，但它定义了「一帧」到底是什么意思。

    要改的只有障碍物的 position：新帧的 t = 0 就是旧帧的 t = h，
    所以新的 t=0 圆心是旧坐标系下 h 时刻的圆心。velocity / radius 不变
    （匀速假设），机器人参数和规划口径全部不变。

    两个提示：
    - 新圆心不要自己写 p + v*h，直接用 `obs.at(h)`。这是「让错误只可能
      发生在一个地方」的老办法，和阶段 1 用 inflated_radius() 而不自己
      拼半径是同一回事。
    - World 和 Obstacle 都是 frozen 的，**改不了**，只能造新的。
      `dataclasses.replace(obs, position=...)` 最省事。
      别忘了 World.obstacles 是 tuple 而不是 list。

    h = 0 时应该原样返回（内容相等即可，不要求是同一个对象）。
    """
    # obstacles: list[Obstacle] = []
    # for obs in world.obstacles:
    #     obstacles.append(Obstacle(obs.at(h), obs.velocity, obs.radius))
    # return World(tuple(obstacles))
    obstacles = tuple(
        dataclasses.replace(obs, position=obs.at(h)) for obs in world.obstacles
    )
    return dataclasses.replace(world, obstacles=obstacles)


def shift_state(x: State, h: float) -> State | None:
    """把一个节点搬到下一帧的坐标系：(x, y, t) -> (x, y, t - h)。

    空间坐标不动，只有时间变。意思是「这个计划中的航点还在原地，
    但它距离现在只剩 t - h 秒了」。

    ⭐ **过期判定**：t - h <= 0 时返回 None。

    为什么是 <= 而不是 < ：t - h == 0 意味着这个航点恰好就是「现在」，
    而新帧的 t = 0 这个位置已经被机器人自己（start guard）占了。
    再放一个同时刻的节点进去，它和 start 之间的那条边时间差为 0，
    edge_feasible 一定拒绝（same_time 那个用例）——它已经没用了。

    写成 t - h < 0 不会报错，只会让你的图里常年挂着一批没用的节点。
    """
    if x.t - h <= 0:
        return None
    return State(x.x, x.y, x.t - h)


def carry_connector(prev_graph: Graph, connector_id: int, h: float) -> State | None:
    """把上一帧的一个 connector 搬到本帧，带「过期重采样」规则。

    这就是论文第 6 行括号里那句话：**「当时间坐标归零时，在轨迹中点处
    重采样一个新节点」**。它看上去像个边角料，实际上是整个机制能不能
    立住的关键——没有它，空场景里的唯一一条通道每几帧就会换一次编号。

    规则：
        1. 先照常规 shift_state。没过期就直接返回。
        2. 过期了的话不要直接丢：取它所在那个**段**的中点做替身。
           segment_trajectory(prev_graph, connector_id).at(0.5) 就是那个中点
           （注意 at() 的参数是弧长进度 s，0.5 是「走了一半路」）。
           把这个中点再 shift_state 一次。
        3. 中点也过期了（整个段都滑出窗口了），那才真的返回 None。

    为什么这条规则必须存在：connector 会被同类里更短的替换，而最短往往
    意味着它会滑向起点附近（贴着直线走最短）。一旦它的 t 小于 h，下一帧就
    没了，于是这条通道的身份也跟着没了——明明机器人走法没变，编号却跳了。

    ⭐ **这个函数有两个调用方，reintroduce_states 和 assign_segment_ids，
    两边必须用同一条规则。** 只在前者里重采样、后者里用 shift_state，
    结果是节点活下来了、身份没跟上：复用生效但编号照跳。这是本阶段最难查的一个 bug，
    因为图看起来完全正常。把它单独抽成一个函数就是为了堵这个洞。
    """
    connector_state = prev_graph.state(connector_id)
    shift_connect = shift_state(connector_state, h)
    if shift_connect is not None:
        return shift_connect

    seg_traj = segment_trajectory(prev_graph, connector_id)
    mid = seg_traj.at(0.5)

    shift_connect = shift_state(mid, h)
    if shift_connect is not None:
        return shift_connect
    return None


def reintroduce_states(prev_graph: Graph, world: World, h: float) -> list[State]:
    """把上一帧图里还能用的节点搬到本帧，返回一串待注入的 State。

    这就是论文 Algorithm 1 第 4-6 行的 **ReintroduceSample**。

    参数里的 world 是**本帧的**世界（已经过了 advance_world），
    prev_graph 是**上一帧的**图（坐标还在旧帧的时间系里）。两者差一个 h。

    要做四件事：

    1. **跳过节点 0 和 1**。它们是上一帧的 start 和 goal，本帧会重新播种：
       start 是机器人的新位置，goal 被推到新窗口的末端。
       ⭐ 这条必须显式写。start 的 t = 0，搬过来自然就过期了，你可能觉得
       不管也行；但 goal 的 t = horizon，搬过来是 horizon - h，**没过期**。
       不挡的话你会在终点旁边多长出一个流浪 guard，而且每帧多一个。
    2. **搬家**：guard 用 shift_state，⭐ connector 用 **carry_connector**
       （它多了过期重采样那一步）。两边都返回 None 的才丢掉。
    3. **在新世界里重新检一遍碰撞**，exercise2.state_collision_free 不过的丢掉。
       上一帧安全不代表这一帧安全：行人的预测被卡尔曼滤波器更新了，
       管子的位置和倾角都可能变了。
    4. ⭐ **顺序：所有 guard 必须排在所有 connector 前面。**

    第 4 条是这个函数唯一的真坑，而且它没有任何计算，纯粹是顺序问题。
    因为这些 State 接下来要被 build_prm_warm 按顺序嗂给 try_add_sample，
    而 try_add_sample 是按「能看见几个 guard」做决策的：

        先嗂 connector -> 图里还没有它的两个 guard -> |L| = 0 或 1
                        -> 它被立成 guard，或者被丢掉

    两种结果都是灾难：前者会把一座桥误认为一个区域，整张图的结构都变了；
    后者直接丢掉了一个拓扑类。而且两种都不报错。

    guard 内部的顺序、connector 内部的顺序都不要紧。

    Returns:
        待注入的 State 列表，guard 在前、connector 在后。
    """
    
    
    raise NotImplementedError("reintroduce_states 还没写")


# 优先尝试的采样点：上一帧的数据
def build_prm_warm(
    world: World,
    start: State,
    goal: State,
    num_samples: int,
    rng: np.random.Generator,
    reintroduced: list[State] = (),  # pyright: ignore[reportArgumentType]
    sampler=None,
) -> Graph:
    """带热启动的 build_prm，完整的 Algorithm 1 第 1-8 行。

    和阶段 2 的 build_prm 比，只多了中间一步：

        1. 建空图，start 先、goal 后都加为 guard（id 恒为 0 和 1）
        2. ⭐ 按顺序把 reintroduced 里的每个 State 嗂给 try_add_sample
        3. 再循环 num_samples 次，采新样本

    ⭐ **num_samples 只管第 3 步。** 复用的节点不占采样预算：它们不是采来的，
    是上一帧已经花过预算挖出来的。这也是热启动能提速的原因——
    论文里 N_PRM = 120，如果复用的节点要扣预算，热启动就变成净亏的了。

    为什么复用的节点也要走 try_add_sample，而不是直接 add_guard / add_connector：
    因为世界变了。上一帧互相看不见的两个 guard，行人走开之后可能就互相看得见了；
    直接塞回去会弄坏四条不变量中的第 3 条。走 try_add_sample 则等于说：
    「上一帧的节点只是一个关于『哪里值得采样』的先验，它的命运仍由本帧的规则决定。」
    四条不变量因此自动保持成立，你不用写任何额外的修补逻辑。

    sampler 参数和阶段 2 一样，是留给测试的口子（graph.FixedSampler）。
    """
    raise NotImplementedError("build_prm_warm 还没写")


def match_guards(prev_graph: Graph, graph: Graph, h: float) -> dict[int, int]:
    """对上两帧的 guard，返回「旧 guard id -> 新 guard id」的映射。

    为什么需要这个：segment 由「一个 connector + 它的两个 guard」构成。
    要问「新帧的这座桥和旧帧的那座桥是不是同一座」，得先问
    「它们跨的是不是同一条河」，也就是两端的 guard 对不对得上。

    规则只有两条：

    1. **0 -> 0、1 -> 1，硬编码。** start 和 goal 是「机器人」和「目的地」，
       它们跨帧就是同一个东西，不管坐标怎么变。
       ⭐ 这条必须特判，因为它们的坐标**讲定会变**：start 是机器人走了 h 秒
       之后的新位置，goal 被推回了 t = horizon。按坐标匹配永远对不上，
       结果就是所有段每帧都拿新 ID——这是本阶段最容易死在里面的一个坑。
    2. 其他旧 guard：把它 shift_state 一下，在新图的 guard 里找坐标对得上的
       （用 tracking.states_close，别用 ==）。找不到就**不要放进映射**，
       说明它这一帧没活下来（过期了、撞上了、或者被降成 connector 了）。

    Returns:
        {旧 guard id: 新 guard id}。没匹配上的旧 guard 不出现在键里。
    """
    raise NotImplementedError("match_guards 还没写")


def assign_segment_ids(
    graph: Graph,
    prev_graph: Graph | None,
    guard_map: dict[int, int],
    prev_segment_ids: dict[int, int],
    allocator: IdAllocator,
    world: World,
    h: float,
    num_samples: int = 20,
) -> dict[int, int]:
    """给本帧每个 connector 定一个 segment ID (alpha)：能继承就继承，否则发新号。

    这是本阶段的心脏，对应论文 Fig. 3。

    判定「新帧的 connector c' 和旧帧的 connector c 是同一座桥」，两个条件：

    **条件 A：两端的 guard 对得上。**
        c 的两个 guard 经 guard_map 映过来，要恰好是 c' 的两个 guard。
        注意是**集合相等**：新旧两帧的 (early, late) 顺序未必一致，
        比如一个 guard 的时间减到比另一个早了。用 set 比，别用 tuple。
        映射里缺了任一个 guard，这对就直接不匹配。

    **条件 B：是同一种绕法（UVD 等价）。**
        把旧 connector 搬到本帧——⭐ 用 **carry_connector** 而不是 shift_state，
        否则在重采样那一帧你会白白丢掉一个身份。返回 None 就不匹配。
        然后比这两条三点折线：

            segment_trajectory(graph, c_new, middle=旧connector搬过来的点)
            segment_trajectory(graph, c_new)

        ⭐ 注意两边用的都是 **graph**（新帧的图）。middle 参数就是专门为此留的：
        两端用新帧的 guard，只让中间点不同。这样 uvd_equivalent 的前提
        （起点相同、终点相同）天然成立，而且比的恰好是「这两座桥绕法一样吗」。
        如果你拿旧帧的 guard 做端点，UVD 的前提就碎了，结果没意义。

    两个条件都满足 -> c' 继承 c 的 alpha。否则 c' 拿 allocator.new_id()。

    ⭐ **一个旧 alpha 最多只能被一个新 connector 继承。**
    否则两座本帧明明不同类的桥会拿到同一个编号，身份就不是身份了。
    用一个已被领走的集合挡一下即可。

    prev_graph 为 None（第一帧）时，所有 connector 都发新号。空分支先挡。

    Returns:
        {connector 节点 id: alpha}。键是本帧节点 id，值是跨帧编号，不要混。
    """
    raise NotImplementedError("assign_segment_ids 还没写")


def assign_trajectory_ids(
    paths: list[list[int]],
    segment_ids: dict[int, int],
    prev_map: dict[frozenset[int], int],
    allocator: IdAllocator,
) -> list[int]:
    """给每条轨迹定一个 trajectory ID (beta)，返回与 paths 一一对应的列表。

    ⭐ 注意这个函数**不碰几何**：没有 world、没有 graph、没有碰撞检测。
    几何在上一步就已经被压成 alpha 了，这里只是查表。这和阶段 2 里
    enumerate_paths 不要 world 是同一个道理：责任分层。

    规则：
        一条轨迹的身份 = 它经过的**段集合**，path_segment_key() 帮你算。
        这个集合在 prev_map 里有 -> 继承那个 beta；没有 -> allocator.new_id()。

    论文的例子：轨迹 tau_1 的 ID 为 1、含段 {1, 4}，就保存映射 1 -> {1, 4}；
    下一帧中由段 1 和 4 组成的轨迹仍被赋予 ID 1。

    为什么是集合而不是序列：因为这是一个 DAG，段的先后顺序由时间唯一决定，
    存顺序是冗余信息。而且论文写的就是「由段 1 和 4 组成」。

    ⭐ **一个 beta 也不能被两条轨迹同时领走。** 与 segment ID 同理。
    （会冲突吗？会。两条路径完全可能有相同的段集合，比如都只由段 {3} 组成。
    这种时候谁先来谁拿，后来的拿新号。）

    Returns:
        与 paths 等长的 beta 列表。paths 为空时返回空列表。
    """
    raise NotImplementedError("assign_trajectory_ids 还没写")


def select_guidance(
    trajectories: list[Trajectory],
    trajectory_ids: list[int],
    previous_id: int | None,
    consistency_penalty: float = 0.0,
) -> int:
    """从候选轨迹里选一条做引导轨迹，返回它的**下标**。

    这是论文式 (9) J_select 的简化版。完整版还有速度项、加速度项和折扣因子，
    那些要等阶段 4 有了样条才算得了。本阶段只留两项：

        cost(i) = trajectories[i].length
                + (consistency_penalty  如果 trajectory_ids[i] != previous_id)

    取代价最小的那条；平局时取下标最小的（结果必须确定）。

    这一项 w_c * C 就是阶段 3 存在的理由。注意它的形式：惩罚加在
    **其他所有轨迹**上，不是奖励加在上一条上。两种写法数值上等价，
    但论文的措辞是「要取代上一周期选中的那条，它必须**显著**更优」——
    这就是一个滞环（hysteresis），和恒温器不在设定点上反复通断是同一个机制。

    previous_id 为 None（第一帧）时没有任何轨迹能免惩罚，等价于纯比长度。

    Returns:
        选中轨迹的下标。⭐ 输入为空时返回 **-1**，不要报错。
        （这一帧找不到任何通道是完全可能的，比如行人正好堵死了路口。
        这种时候上层该做的是减速，而不是崩溃。）
    """
    raise NotImplementedError("select_guidance 还没写")
