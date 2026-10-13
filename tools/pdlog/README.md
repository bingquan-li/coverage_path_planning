# pdlog 目录批量日志过滤

## 单个 pdlog 的 local_map 回放

`local_map` 复用 `pdlog` 的 PPMd8 读取器，流式解压单个文件，再解析每行
日志前缀后的 JSON 及 `move` 标签下的 `navigation result` 运行状态。
JSON 处理 `type` 严格等于 `local_map`、`foot_print` 和 `global_paths` 的对象；
`local_map.data` 再经过 Base64、gzip 和每格 2 bit 的解码。

```bash
cmake --build build/Debug --target local_map
./build/Debug/tools/pdlog_build/local_map /path/to/xx.pdlog
# 无界面导出，之后用 rerun 打开：
./build/Debug/tools/pdlog_build/local_map /path/to/xx.pdlog --save /tmp/local_map.rrd
rerun /tmp/local_map.rrd
```

`map_size` 是正方形地图的边长（格），`map_scale` 是米/格，`pos` 仅用于地图中心
定位。左下角由 `pos - map_size * map_scale / 2` 计算。沿用原示例的
行序约定：解码后第 0 行位于底部。日志未提供旋转，地图按 world 轴对齐。
机器人轮廓使用 `foot_print.data` 中的 `[[x,y], ...]` 顶点，单位为米，
直接作为 world 坐标，不叠加地图 `pos`，也不再使用固定八边形。
顶点必须至少包含三个不同点；已闭合的轮廓保持不变，未闭合的自动连接首尾。
显示路径仍为 `world/robot/footprint`。

按 `foot_print → local_map` 一一配对：先暂存轮廓，遇到下一张地图时一起
写入同一帧，随后消费该轮廓。中间可夹杂其他类型日志。若连续出现两个轮廓，
报告前一个未配对并使用最新的；若地图缺少轮廓，保留地图并显式清除旧轮廓。
损坏的地图或轮廓会使当前待配对轮廓失效，避免错误沿用；文件末尾多余的
轮廓也会报告为未配对。

地图使用单通道 `GridMap` 和 `Costmap` 配色，0 类透明。

### 全局路径

支持 `global_paths` 的 world 坐标折线，单位为米。例如：

```text
21:40:43 198(84602)D/debug_bridge: {"data":[[-2.66,-1.47],[-2.64,-1.42],[-2.62,-1.38]],"number_of_segment":0,"total_number_of_segments":1,"type":"global_paths"}
```

显示在 `world/global_paths`，使用绿色细线，Z=0.04 m，略高于地图。
保留点序、重复点和折返，不自动闭合、不简化，也不叠加地图或机器人位姿。
现有 Blueprint 的 `/world/**` 过滤器会自动包含路径，无需新建视图。
单点段额外用 `world/global_paths/singletons` 的点显示。

分段编号从 0 开始，必须满足 `0 <= number_of_segment < total_number_of_segments`：

- 0 号段开始一组新的路径，`total_number_of_segments=1` 时立即更新。
- 多段收齐后按编号一次更新；各段为独立折线，不跨段连线。收集过程中保留上一组完整路径。
- 收到 0 号段后，其余段可乱序到达，同编号重发替换该段。再次收到 0 号段则重新开始。
- 缺失 0 号段、段数不一致、损坏的顶点或文件结束时未收齐，都会报告错误。
  协议没有规划 ID，无法可靠识别多组路径交错发送；默认一组的 0 号段先到。
- 完整一组的所有 `data` 都为空时清除旧路径；多段中的空段仍保留其编号。

路径按收齐消息的 `robot_time` / `log_line` 更新，不依赖地图帧，不消费待配对轮廓。
新路径替换旧路径，不累加到当前画面；拖动时间游标仍可查看历史路径。

### 运行状态与速度

支持如下文本记录，两个速度分别按线速度 `m/s` 和角速度 `rad/s` 显示，
保留原始正负号，不做单位换算：

```text
21:43:12 455(84642)D/move: navigation result speed:-0.0677626 -0.0937527 status:Navigating
```

默认 Blueprint 左侧显示地图，右侧依次显示：

- **Robot State / State History**：切换查看时间游标处最近一次状态、两个速度、
  来源时间/行号，或全部原始状态日志。Navigating 蓝色，Escaping 黄色，
  Stuck/FailEscape 红色；未知状态保留名称并以灰色记录。颜色用于历史日志。
- **Linear Speed (m/s)**：线速度阶梯曲线，保持上一采样值直到下一次记录。
- **Angular Speed (rad/s)**：角速度曲线。两种单位分开显示，避免共用纵轴。

两张曲线图联动缩放时间轴，默认收起 Selection 面板以留出显示空间。

实体位于 `navigation/current`、`navigation/events`、`navigation/linear_speed`、
`navigation/angular_speed`，与 `world` 空间实体分开，方便自定义 Blueprint。
每条有效状态独立记录，不等待地图，不丢弃地图间的多条速度记录。
状态记录（包括坏记录）不会消费待配对的轮廓。

默认选择共用的 `robot_time` 时间线：从 `HH:mm:ss 毫秒` 前缀读取时间，
表示从首日午夜起的时长，保留真实时间间隔。地图和配对轮廓一起使用地图行时间；
状态和速度使用各自记录时间。拖动游标时，地图、轮廓和状态分别显示该时刻之前
最近一次有效数据；图表保留速度历史。因此状态来源时间可能早于当前地图时间。
支持跨午夜及少量乱序，以距已知最新时间最近的一天解释日期（日志没有日期，
无法区分超过 12 小时的间隔与跨天乱序）。

`log_line` 提供所有数据的原始行号时间线；`frame` 仅保留地图帧序号，
查看状态与地图联动时请选择 `robot_time` 或 `log_line`。同一毫秒多条记录
仍被保存，需精确按文件顺序定位时使用 `log_line`。缺少有效时间前缀的记录
会警告并仅参与 `log_line`（地图另有 `frame`），不会错误沿用上一行时间。

损坏的 local_map/foot_print/navigation/global_paths 记录会打印文件名、行号并跳过；PPMd8 流损坏则终止。
全部有效、配对完整且至少输出一帧地图、一条状态或一组路径时返回 0（支持仅路径日志），
存在坏记录、未配对、没有有效数据或处理失败时返回 1，
命令行错误返回 2。坏帧前后有效的数据仍可在已生成的记录中查看。
输入输出不能指向同一个文件。

构建需要 `nlohmann_json`、Zlib 和已有 Rerun SDK。默认 Blueprint 随源码提供，
修改 `local_map_blueprint.py` 后用项目 Python 环境运行它以重新生成 `.rbl`。

```bash
cmake --build build/Debug --target local_map_log_test navigation_log_test global_paths_log_test
ctest --test-dir build/Debug -R '^(local_map_log|navigation_log|global_paths_log)_test$' --output-on-failure
```

## 从 Rerun 当前时刻直接导出 JSON

现在的导出脚本直接查询 Rerun 数据，不再读取 `.pdlog`，也不调用 C++ 日志导出器。
命令只需要输出路径：

```bash
.venv/bin/python tools/pdlog/export_current_frame.py /tmp/frame.json
```

在 Viewer 中选择 **robot_time** 或 **log_line** 时间线，暂停到目标时刻后运行命令。
脚本通过 MCP 读取游标，然后通过 catalog 的单点 latest-at 查询取数，仅请求当前
记录、选定时刻及所需组件；不会把全部历史地图下载到 Python 再筛选。

**当前版本的实时流限制：** C++ `rec.spawn()` 送入 Viewer 的 SDK proxy 记录，
不在 Viewer 的可查询 catalog 中。对已经打开的这类记录，先 `Ctrl+S` 保存为
`.rrd`，再在 Viewer 中打开该 `.rrd` 并选择它，之后可以反复直接导出，不需要原日志。
脚本遇到 proxy 记录会给出明确提示，不会偷偷退回读取 `.pdlog`。
也可首次运行时就生成 Rerun 记录，再打开：

```bash
./build/Debug/tools/pdlog_build/local_map xx.pdlog --save /tmp/map.rrd
rerun /tmp/map.rrd
```

使用项目 `.venv` 中的 Rerun Python SDK、PyArrow、DataFusion。已验证 Python SDK
0.38.1 与 Viewer 0.39.0 的组合。`--viewer http://127.0.0.1:9876` 指定 Viewer 地址，
`--rerun /path/to/rerun` 可指定支持 `viewer-mcp` 的 CLI。

也可保持当前记录，显式指定查询时刻（不移动 Viewer 游标）：

```bash
# 21:43:12.455，单位为从首日午夜起的整数纳秒。
.venv/bin/python tools/pdlog/export_current_frame.py frame.json \
  --timeline robot_time --at 78192455000000
# 或按原始行号时间线查询。
.venv/bin/python tools/pdlog/export_current_frame.py frame.json \
  --timeline log_line --at 12345
```

JSON 包含：

- `selection` / `viewer_recording`：选定时间与记录；`data_source` 为 `rerun`。
- `local_map`：二维 `data[row][col]`、宽高、分辨率、地图中心、原点及旋转。
  将显示索引 `0/32/192/254` 无损映射回 **0～3**，恢复**第 0 行在底部**的行序。
  其他像素格式或未知色图索引会报错，避免误导出。
- `foot_print`：world 坐标 `data`（XY）以及 Viewer 实际存储的 `data_xyz`。
  后者包含绘图时添加的 Z 高度；轮廓缺失或已清除时为 `null`。
- `navigation`：两个 Scalars 组件中的速度，及当前状态文本中的状态名称；
  选定时刻没有对应数据时为 `null`。
- `global_paths`：当前完整路径的 `segments` 数组，按 `number_of_segment` 编号，
  每段包含 XY `data` 和 Rerun 中的 `data_xyz`；没有路径或已清除时为 `null`。

坐标与分辨率保留的是 **Rerun 存储的 float 精度**，不再追溯原日志 double 数值。
地图和状态各自取游标时刻之前最近的值，与 Viewer 一致，二者不必同一采样时刻。
状态文本只用于提取状态名称，速度从数值组件获取，不从格式化文本反推。
已有 JSON 不会被覆盖；没有有效地图时不生成文件。

原来的 `local_map_export` C++ 命令仍作为离线日志导出工具保留，但不参与上述流程。

## 目录批量过滤

编译并使用默认配置：

```bash
cmake --build build --target pdlog
./build/tools/pdlog_build/pdlog > selected.log
```

默认配置为编译时项目源码目录中的 `configs/pdlog.yaml`，运行时无需切换到项目目录。
目录、时间范围和关键词都在 YAML 中设置：

```yaml
input_dir: "../logs"
time_range:
  start: "2026-09-03 19:47"
  end: "2026-09-03 20:10"
keywords:
  - "ERROR"
  - "WARN"
```

`input_dir` 可以是绝对路径；相对路径以 **YAML 文件所在目录** 为基准。
只扫描目录直属的普通文件，不递归进入子目录；实际默认目录见 `configs/pdlog.yaml`。

识别的文件名格式为 `NavigationService.g3log.YYYYMMDD-HHMMSS.数字.pdlog`，
例如 `NavigationService.g3log.20260903-194700.10776.pdlog`。
其他名称的文件被忽略；前缀和扩展名匹配、但日期或字段无效的文件会报告并跳过。
选中的文件按文件名中的时间从早到晚处理；同一秒的文件按完整文件名排序，
不依赖文件修改时间和目录枚举顺序。

时间范围使用 `"YYYY-MM-DD HH:MM"`，**包含起止两分钟**。
上述示例选择 `2026-09-03 19:47:00` 至 `2026-09-03 20:10:59` 的文件。
起止相同可选择单独一分钟，也支持跨天、跨月、跨年；起始时间晚于结束时间会报错。
配置与文件名使用同一时间表示，不转换时区。
`start: null` 或 `end: null` 表示对应方向不设边界，两者均为 `null` 则不限制时间。
这两个字段都必须写出。

**时间筛选仅作用于文件名。** 选中的文件会完整解压，再按关键词逐行过滤；
不会按日志正文的时间截取，也不会自动纳入文件名时间早于起始范围、但正文可能
延续到范围内的文件。若要包含这些内容，应相应提前起始时间。

每行包含任意一个关键词就输出整行，区分大小写，按字面子串匹配，不使用正则。
`keywords: []` 输出全部日志。关键词必须是非空标量，建议始终加引号，
避免 `#`、冒号或 `null` 等被 YAML 解释为其他语法。

所有结果顺序合并到标准输出，进度和诊断写标准错误，不在正文中插入文件名。
前一个有输出的文件缺少末尾换行时，在下一个文件首次输出前补一个换行，
避免两条日志粘连；无匹配行的文件不会增加空行，最终末行不强制补换行。
文件间不做逐行时间归并或去重。

指定其他配置文件：

```bash
./build/tools/pdlog_build/pdlog --config custom.yaml > selected.log
```

`--config` 的相对路径基于当前工作目录。移动可执行文件或删除原源码目录后，
应使用该参数指定可访问的配置。命令行不再接受单个输入日志路径或 `--keyword`。

退出状态：

- `0`：全部选中文件处理成功。没有匹配关键词的行也算成功。
- `1`：目录无法读取、没有符合命名和时间范围的文件、解压或输出失败。
- `2`：命令行或 YAML 配置错误，包括无效日期、逆序范围、缺失字段等。

扫描和配置校验在解压前完成；任意选中文件解压失败时立即停止处理后续文件。
由于采用流式输出，失败时可能已经写出部分日志，脚本应检查退出状态。

## 普通文本输入

可选配置 `input_format` 默认为 `pdlog`，现有 YAML 无需修改。设置为 `text`
时不解压，读取 `NavigationService.g3log.YYYYMMDD-HHMMSS.数字.log` 文件；
时间筛选、排序和关键词匹配规则保持一致。两种模式均按二进制方式打开输入，
保留 LF、CRLF、内嵌 NUL 和没有换行符的末行。

```yaml
input_dir: "../logs"
input_format: text
time_range:
  start: null
  end: null
keywords: []
```

## 模块与对象生命周期

当前只实现任务组织和日志读取，输出继续使用原有的关键词过滤与文本合并。
尚未加入业务事件解析、事件时间过滤、时区转换或 MCAP 导出。

| 文件 | 职责 |
| --- | --- |
| `pdlog.cc` | 命令行入口、配置加载及退出码映射 |
| `config.hh / config.cc` | `ConversionConfig`、YAML 校验、目录扫描与排序 |
| `types.hh` | `LogFile`、`LogLine` 和输入格式 |
| `application.hh / application.cc` | `ConversionApplication` 组织读取、过滤、输出，返回 `ConversionReport` |
| `log_reader.hh / log_reader.cc` | `ILogLineReader`、`PdlogReader`、`TextLogReader`、`LogSequenceReader` |
| `ppmd_decoder.hh / ppmd_decoder.cc` | `PpmdDecoder` 管理解码模型和 C 回调适配器 |

`LogSequenceReader` 按给定顺序读取，一次只打开一个文件；目录入口在创建它之前
完成排序。每个压缩文件独立创建 `PpmdDecoder`。文件句柄、模型通过 RAII 释放，
构造失败和异常路径也会清理资源。解码器借用文件，析构顺序保证先释放模型再关文件。

读取器的 `nextLine()` 返回 `std::optional<LogLine>`：

- 有值：返回拥有正文的日志行，以及来源路径、从 1 开始的文件内行号。
- `nullopt`：当前文件或整个序列正常读完，并已完成结束检查。
- 异常：读取或解码失败，调用方应停止并丢弃该读取器，不能当作 EOF 跳过。

读取层不匹配关键词、不补跨文件换行、不写 stdout/stderr；进度通过
`LogSequenceReader::FileStarted` 回调交给应用层。直接使用 `TextLogReader`
可读取任意名称的文本文件；文件名协议只约束目录扫描入口。程序也可以显式
构造不同 `InputFormat` 的 `LogFile` 列表，读取压缩和文本文件组成的序列。

```cpp
pdlog::LogSequenceReader reader({
    {"first.pdlog", {}, pdlog::InputFormat::Pdlog},
    {"second.log", {}, pdlog::InputFormat::Text},
});
while(auto line = reader.nextLine())
{
  // line->source_file、line->line_number、line->text
  // 后续业务解析器可在此消费日志行。
}
```

`ConversionApplication` 借用输出 `FILE*` 和诊断 `std::ostream&`，结束时检查输出
刷新结果，但不关闭调用方的流。成功返回处理文件数、读取行数及输出行数；CLI
保持正文纯净，不额外输出统计信息。文件数包含成功读完的空文件。

兼容性边界：无换行末行仍先于最终解码检查交付；只有继续读到 `nullopt` 才表示
结束检查成功。PPMd 检查仍依据结束符、底层读取错误及范围解码状态，不额外检查
物理 EOF 或结束符后的剩余字节，也不提供校验和。单行长度没有上限。

## 验证

```bash
cmake --build build --target pdlog pdlog_test
./build/tools/pdlog_build/pdlog_test
```

测试覆盖行来源和生命周期、CRLF/NUL/无换行末行、空文件、跨文件切换、排序和
分钟边界、配置校验、关键词过滤和分隔符、统计、有效空压缩流、损坏输入以及
解码器初始化失败。在存在 `/dev/full` 的系统上还检查输出写入和刷新失败。
`pdlog_core` 是可复用静态库，不包含 CLI `main()`。
