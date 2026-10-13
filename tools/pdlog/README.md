# pdlog 目录批量日志过滤

## 单个 pdlog 的 local_map 回放

`local_map` 复用 `pdlog` 的 PPMd8 读取器，流式解压单个文件，再解析每行
日志前缀后的 JSON。只处理 `type` 严格等于 `local_map` 的对象；其 `data`
再经过 Base64、gzip 和每格 2 bit 的解码。

```bash
cmake --build build/Debug --target local_map
./build/Debug/tools/pdlog_build/local_map /path/to/xx.pdlog
# 无界面导出，之后用 rerun 打开：
./build/Debug/tools/pdlog_build/local_map /path/to/xx.pdlog --save /tmp/local_map.rrd
rerun /tmp/local_map.rrd
```

`map_size` 是正方形地图的边长（格），`map_scale` 是米/格，`pos` 是地图中心
及机器人位置。左下角由 `pos - map_size * map_scale / 2` 计算。沿用原示例的
行序约定：解码后第 0 行位于底部。日志未提供旋转，地图按 world 轴对齐。

地图使用单通道 `GridMap` 和 `Costmap` 配色，0 类透明。每条有效记录在
`frame` 时间线上保存一帧，地图和机器人位置同步更新；`log_line` 时间线
可定位原始文件的行号。默认 Blueprint 选择 `frame`，它表示记录顺序，
不表示日志前缀中的实际时间间隔。

损坏的 local_map 记录会打印文件名、行号并跳过；PPMd8 流损坏则终止。
全部有效且至少输出一帧时返回 0，存在坏帧、没有地图或处理失败时返回 1，
命令行错误返回 2。坏帧前后有效的数据仍可在已生成的记录中查看。
输入输出不能指向同一个文件。

构建需要 `nlohmann_json`、Zlib 和已有 Rerun SDK。默认 Blueprint 随源码提供，
修改 `local_map_blueprint.py` 后用项目 Python 环境运行它以重新生成 `.rbl`。

```bash
cmake --build build/Debug --target local_map_log_test
ctest --test-dir build/Debug -R '^local_map_log_test$' --output-on-failure
```

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
