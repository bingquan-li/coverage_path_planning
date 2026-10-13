#include "application.hh"
#include "log_reader.hh"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>

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

template <class F>
void
throwsContaining(F action, std::string_view expected)
{
  try
  {
    action();
  }
  catch(std::exception const& error)
  {
    require(std::string_view(error.what()).find(expected) !=
                std::string_view::npos,
            "unexpected exception message");
    return;
  }
  throw std::runtime_error("expected an exception");
}

struct TemporaryDirectory
{
  std::filesystem::path path;
  TemporaryDirectory()
  {
    auto const seed =
        std::chrono::steady_clock::now().time_since_epoch().count();
    for(unsigned attempt = 0; attempt < 100; ++attempt)
    {
      path = std::filesystem::temp_directory_path() /
             ("pdlog-test-" + std::to_string(seed) + "-" +
              std::to_string(attempt));
      if(std::filesystem::create_directory(path))
      {
        return;
      }
    }
    throw std::runtime_error("cannot create test directory");
  }
  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

void
put(std::filesystem::path const& path, std::string const& contents)
{
  std::ofstream file(path, std::ios::binary);
  file.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  require(bool(file), "cannot create fixture");
}

std::string
readOutput(FILE* file)
{
  std::rewind(file);
  std::string result;
  for(int byte = 0; (byte = std::fgetc(file)) != EOF;)
  {
    result.push_back(static_cast<char>(byte));
  }
  require(std::ferror(file) == 0, "cannot read test output");
  return result;
}

void
checkReaders(std::filesystem::path const& root)
{
  auto const first = root / "first.log";
  auto const empty = root / "empty.log";
  auto const last = root / "last.log";
  put(first, std::string("first\r\n\nA\0B", 11));
  put(empty, "");
  put(last, "last\n");
  std::vector<std::filesystem::path> started;
  pdlog::LogSequenceReader reader{{{first, {}, pdlog::InputFormat::Text},
                                   {empty, {}, pdlog::InputFormat::Text},
                                   {last, {}, pdlog::InputFormat::Text},},
                                  [&](pdlog::LogFile const& file)
                                  { started.push_back(file.path); }};
  auto line1 = reader.next_line();
  require(line1 && line1->text == "first\r\n" && line1->line_number == 1 &&
              line1->source_file == first,
          "first line metadata/content");
  auto line2 = reader.next_line();
  require(line2 && line2->text == "\n" && line2->line_number == 2,
          "blank line");
  auto line3 = reader.next_line();
  require(line3 && line3->text == std::string("A\0B", 3) &&
              line3->line_number == 3,
          "binary unterminated tail");
  auto line4 = reader.next_line();
  require(line4 && line4->text == "last\n" && line4->line_number == 1 &&
              line4->source_file == last,
          "file transition and empty file");
  require(line1->text == "first\r\n", "returned line lifetime");
  require(!reader.next_line() && !reader.next_line(), "stable end of sequence");
  require(reader.filesProcessed() == 3 && started.size() == 3, "file counts");
  pdlog::LogSequenceReader none{{}};
  require(!none.next_line() && none.filesProcessed() == 0, "empty sequence");

  auto const missing = root / "missing";
  pdlog::LogSequenceReader failed{{{missing, {}, pdlog::InputFormat::Text},
                                   {last, {}, pdlog::InputFormat::Text},}};
  throwsContaining([&] { failed.next_line(); }, "cannot open");
  require(failed.filesProcessed() == 0,
          "failed file must not be counted/skipped");

  auto const compressed = root / "bad.pdlog";
  // order=8、1 MiB、RESTART；初始均匀模型只编码结束符的空流。
  auto const empty_stream = std::string("\x07\x00\xff\x00\xff\x00\x00", 7);
  put(compressed, empty_stream);
  pdlog::PdlogReader empty_compressed{compressed};
  require(!empty_compressed.next_line() && !empty_compressed.next_line(),
          "valid compressed empty stream");
  pdlog::LogSequenceReader mixed{{{compressed, {}, pdlog::InputFormat::Pdlog},
                                  {last, {}, pdlog::InputFormat::Text},}};
  auto mixed_line = mixed.next_line();
  require(mixed_line && mixed_line->text == "last\n" && !mixed.next_line() &&
              mixed.filesProcessed() == 2,
          "mixed compressed/text sequence");

  // 结束符仍可解出，但范围解码器 Code 不为零，必须抛错而不是返回 EOF。
  auto bad_end = empty_stream;
  bad_end.back() = '\x01';
  put(compressed, bad_end);
  throwsContaining(
      [&]
      {
        pdlog::PdlogReader r{compressed};
        r.next_line();
      },
      "truncated or invalid stream");
  put(compressed, "");
  throwsContaining([&] { pdlog::PdlogReader const r{compressed}; }, "invalid header");
  put(compressed, std::string("\0\0", 2));
  throwsContaining([&] { pdlog::PdlogReader const r{compressed}; },
                   "unsupported PPMd8");
  put(compressed, std::string("\x07\0\xff\xff\xff\xff", 6));
  // 重复触发分配后的初始化失败，供 ASan/LSan 检查构造失败时的清理。
  for(int iteration = 0; iteration < 8; ++iteration)
  {
    throwsContaining([&] { pdlog::PdlogReader const r{compressed}; },
                     "invalid PPMd8 range");
  }
}

void
checkApplication(std::filesystem::path const& root)
{
  auto const logs = root / "logs";
  std::filesystem::create_directory(logs);
  auto const name = [&](std::string const& time, std::string const& id)
  { return logs / ("NavigationService.g3log." + time + "." + id + ".log"); };
  put(name("20270101-000059", "2"), "KEEP last");
  put(name("20270101-000000", "20"), "skip\n");
  put(name("20270101-000000", "10"), "KEEP middle\r\n");
  put(name("20261231-235959", "1"), "KEEP first");
  put(name("20261231-235958", "1"), "");
  put(name("20270101-000100", "1"), "outside\n");
  put(name("20260230-000000", "1"), "invalid name\n");
  put(logs / "unrelated.log", "ignored\n");
  std::filesystem::create_directory(logs / "nested");
  put(logs / "nested" / "NavigationService.g3log.20270101-000000.1.log",
      "nested\n");
  auto const config_path = root / "settings.yaml";
  std::string const config_text =
      "input_dir: logs\ninput_format: text\n"
      "time_range: {start: '2026-12-31 23:59', end: '2027-01-01 00:00'}\n"
      "keywords: [KEEP, MATCH]\n";
  put(config_path, config_text);
  auto config = pdlog::load_config(config_path);
  require(config.input_dir == logs &&
              config.input_format == pdlog::InputFormat::Text,
          "relative input directory / input format");
  std::ostringstream diagnostics;
  auto const files = pdlog::collect_files(config, diagnostics);
  require(files.size() == 5 && files[0].path == name("20261231-235958", "1") &&
              files[2].path == name("20270101-000000", "10") &&
              files[4].path == name("20270101-000059", "2"),
          "selection and ordering");
  require(diagnostics.str().find("skipping invalid filename") !=
              std::string::npos,
          "invalid filename diagnostic");
  pdlog::detail::InputFile output{std::tmpfile()};
  require(bool(output), "tmpfile");
  pdlog::ConversionApplication const app{config, output.get(), diagnostics};
  auto const report = app.run();
  require(readOutput(output.get()) == "KEEP first\nKEEP middle\r\nKEEP last",
          "cross-file separator / filtered files");
  require(report.files_processed == 5 && report.lines_read == 4 &&
              report.lines_written == 3,
          "conversion report");

  config.keywords = {"not present"};
  auto const unmatched =
      pdlog::ConversionApplication{config, output.get(), diagnostics}.run();
  require(unmatched.lines_written == 0 && unmatched.files_processed == 5,
          "no matching lines is success");
  config.keywords.clear();
  auto const unfiltered =
      pdlog::ConversionApplication{config, output.get(), diagnostics}.run();
  require(unfiltered.lines_written == 4, "empty keyword list accepts all");

  if(std::filesystem::exists("/dev/full"))
  {
    pdlog::detail::InputFile full{std::fopen("/dev/full", "wb")};
    require(bool(full), "open /dev/full");
    throwsContaining(
        [&]
        {
          pdlog::ConversionApplication{config, full.get(), diagnostics}.run();
        },
        "flush failed");
    pdlog::detail::InputFile unbuffered{std::fopen("/dev/full", "wb")};
    require(bool(unbuffered), "open unbuffered /dev/full");
    require(std::setvbuf(unbuffered.get(), nullptr, _IONBF, 0) == 0,
            "unbuffered output");
    throwsContaining(
        [&]
        {
          pdlog::ConversionApplication{config, unbuffered.get(), diagnostics}
              .run();
        },
        "write failed");
  }

  config.input_dir = root / "empty-directory";
  std::filesystem::create_directory(config.input_dir);
  throwsContaining(
      [&]
      {
        pdlog::ConversionApplication{config, output.get(), diagnostics}.run();
      },
      "no log files match");
  config.input_dir = root / "absent-directory";
  throwsContaining(
      [&]
      {
        pdlog::ConversionApplication{config, output.get(), diagnostics}.run();
      },
      "cannot scan");
  throwsContaining(
      [&] { pdlog::ConversionApplication{config, nullptr, diagnostics}; },
      "output must not be null");

  for(auto const& bad : {std::string("input_format: unknown\n"),
                         std::string("input_format: null\n"),})
  {
    put(config_path,
        "input_dir: logs\ntime_range: {start: null, end: null}\nkeywords: "
        "[]\n" +
            bad);
    throwsContaining([&] { pdlog::load_config(config_path); }, "input_format");
  }
  put(config_path,
      "input_dir: logs\ntime_range: {start: '2026-02-30 00:00', end: "
      "null}\nkeywords: []\n");
  throwsContaining([&] { pdlog::load_config(config_path); },
                   "invalid timestamp");
  put(config_path,
      "input_dir: logs\ntime_range: {start: null, end: null}\nkeywords: []\n");
  require(pdlog::load_config(config_path).input_format ==
              pdlog::InputFormat::Pdlog,
          "backward-compatible default format");
}
} // namespace

int
main()
{
  try
  {
    TemporaryDirectory const directory;
    checkReaders(directory.path);
    checkApplication(directory.path);
    std::cout << "pdlog reader/application checks passed\n";
    return 0;
  }
  catch(std::exception const& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
