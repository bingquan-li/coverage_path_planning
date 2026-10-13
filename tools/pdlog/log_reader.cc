#include "log_reader.hh"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace pdlog
{
namespace
{
detail::InputFile
open_input(std::filesystem::path const& path)
{
  detail::InputFile input{std::fopen(path.string().c_str(), "rb")};
  if(!input)
  {
    int const error = errno;
    throw std::runtime_error("cannot open " + path.string() + ": " +
                             std::strerror(error));
  }
  return input;
}

PpmdParameters
read_header(FILE* input, std::filesystem::path const& path)
{
  unsigned char header[2]{};
  if(std::fread(header, 1, sizeof(header), input) != sizeof(header))
  {
    throw std::runtime_error("invalid header in " + path.string());
  }
  unsigned const value = static_cast<unsigned>(header[0]) |
                         (static_cast<unsigned>(header[1]) << 8U);
  PpmdParameters parameters{(value & 0x0FU) + 1U,
                            ((value >> 4U) & 0xFFU) + 1U,
                            value >> 12U,};
  if(parameters.order < 2 || parameters.restore_method > 1)
  {
    throw std::runtime_error(
        "unsupported PPMd8 parameters in " + path.string() +
        " (order=" + std::to_string(parameters.order) +
        ", memory=" + std::to_string(parameters.memory_mib) +
        " MiB, restore=" + std::to_string(parameters.restore_method) + ")");
  }
  return parameters;
}
} // namespace

PdlogReader::PdlogReader(std::filesystem::path path)
  : path_(std::move(path))
  , input_(open_input(path_))
  , decoder_(std::make_unique<PpmdDecoder>(
        input_.get(), read_header(input_.get(), path_), path_))
{}

std::optional<LogLine>
PdlogReader::next_line()
{
  if(reached_end_)
  {
    decoder_->validate_end();
    return std::nullopt;
  }

  LogLine line{path_, line_number_ + 1, {}};
  while(auto const byte = decoder_->next_byte())
  {
    line.text.push_back(static_cast<char>(*byte));
    if(*byte == '\n')
    {
      ++line_number_;
      return line;
    }
  }
  reached_end_ = true;
  if(!line.text.empty())
  {
    // 沿用旧行为：先交付无换行末行，下一次读取再做最终检查。
    // 提前停止读取不代表整个流已通过验证，必须读到 nullopt。
    ++line_number_;
    return line;
  }
  decoder_->validate_end();
  return std::nullopt;
}

TextLogReader::TextLogReader(std::filesystem::path path)
  : path_(std::move(path))
  , input_(open_input(path_))
{}

std::optional<LogLine>
TextLogReader::next_line()
{
  LogLine line{path_, line_number_ + 1, {}};
  for(;;)
  {
    int const value = std::fgetc(input_.get());
    if(value == EOF)
    {
      if(std::ferror(input_.get()) != 0)
      {
        throw std::runtime_error("cannot read " + path_.string());
      }
      if(line.text.empty())
      {
        return std::nullopt;
      }
      ++line_number_;
      return line;
    }
    line.text.push_back(static_cast<char>(value));
    if(value == '\n')
    {
      ++line_number_;
      return line;
    }
  }
}

LogSequenceReader::LogSequenceReader(std::vector<LogFile> files,
                                     FileStarted on_file_started)
  : files_(std::move(files))
  , on_file_started_(std::move(on_file_started))
{}

std::optional<LogLine>
LogSequenceReader::next_line()
{
  while(files_processed_ < files_.size())
  {
    if(!current_reader_)
    {
      auto const& file = files_[files_processed_];
      if(on_file_started_)
      {
        on_file_started_(file);
      }
      switch(file.format)
      {
        case InputFormat::Pdlog:
          current_reader_ = std::make_unique<PdlogReader>(file.path);
          break;
        case InputFormat::Text:
          current_reader_ = std::make_unique<TextLogReader>(file.path);
          break;
        default:
          throw std::invalid_argument("unsupported input format");
      }
    }
    if(auto line = current_reader_->next_line())
    {
      return line;
    }
    current_reader_.reset(); // 先关闭当前文件，再打开下一个。
    ++files_processed_;
  }
  return std::nullopt;
}
} // namespace pdlog
