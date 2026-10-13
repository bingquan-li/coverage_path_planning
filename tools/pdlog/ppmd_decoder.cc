#include "ppmd_decoder.hh"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

extern "C"
{
#include "Alloc.h"
#include "Ppmd8.h"
}

namespace pdlog
{
namespace
{
// C 回调传入首成员 IByteIn 的地址，不能将其转为任意 C++ 对象。
struct FileReader
{
  IByteIn interface;
  FILE* file;
  bool eof = false;
  bool error = false;
};
static_assert(std::is_standard_layout_v<FileReader>);
static_assert(offsetof(FileReader, interface) == 0);

Byte
readByte(void* opaque)
{
  auto* reader = static_cast<FileReader*>(opaque);
  int const value = std::fgetc(reader->file);
  if(value != EOF)
  {
    return static_cast<Byte>(value);
  }
  reader->eof = true;
  reader->error = reader->error || std::ferror(reader->file) != 0;
  // IByteIn 无错误通道，沿用补零约定，错误留给结束检查。
  return 0;
}

struct Model
{
  CPpmd8 value{};
  Model()
  {
    Ppmd8_Construct(&value);
  }
  ~Model()
  {
    Ppmd8_Free(&value, &g_BigAlloc);
  }
  Model(Model const&) = delete;
  Model&
  operator=(Model const&) = delete;
};
} // namespace

struct PpmdDecoder::Impl
{
  FileReader reader;
  Model model;
  std::filesystem::path source;
  bool saw_end_marker = false;

  Impl(FILE* input, PpmdParameters parameters, std::filesystem::path path)
    : reader{{readByte}, input}
    , source(std::move(path))
  {
    if(input == nullptr || parameters.order < 2 || parameters.order > 16 ||
       parameters.memory_mib < 1 || parameters.memory_mib > 256 ||
       parameters.restore_method > 1)
    {
      throw std::invalid_argument("invalid PPMd8 decoder arguments");
    }

    model.value.Stream.In = &reader.interface;
    if(Ppmd8_Alloc(&model.value, parameters.memory_mib << 20U, &g_BigAlloc) ==
       0)
    {
      throw std::runtime_error("cannot allocate " +
                               std::to_string(parameters.memory_mib) +
                               " MiB for " + source.string());
    }
    // Model 已构造，后续初始化失败或 C++ 异常同样会释放模型内存。
    if(Ppmd8_RangeDec_Init(&model.value) == 0)
    {
      throw std::runtime_error("invalid PPMd8 range stream in " +
                               source.string());
    }
    Ppmd8_Init(&model.value, parameters.order, parameters.restore_method);
  }
};

PpmdDecoder::PpmdDecoder(FILE* input,
                         PpmdParameters parameters,
                         std::filesystem::path const& source)
  : impl_(std::make_unique<Impl>(input, parameters, source))
{}

PpmdDecoder::~PpmdDecoder() = default;

std::optional<std::uint8_t>
PpmdDecoder::next_byte()
{
  if(impl_->saw_end_marker)
  {
    return std::nullopt;
  }
  int const symbol = Ppmd8_DecodeSymbol(&impl_->model.value);
  if(symbol >= 0)
  {
    return static_cast<std::uint8_t>(symbol);
  }
  if(symbol != -1)
  {
    throw std::runtime_error("corrupt PPMd8 data in " + impl_->source.string());
  }
  impl_->saw_end_marker = true;
  return std::nullopt;
}

void
PpmdDecoder::validate_end() const
{
  // 保留原有验证范围：不检查物理 EOF 或结束符后的多余数据。
  // Code == 0 不是校验和，不能保证识别所有截断或篡改。
  if(!impl_->saw_end_marker || impl_->reader.error ||
     !Ppmd8_RangeDec_IsFinishedOK(&impl_->model.value))
  {
    throw std::runtime_error("truncated or invalid stream in " +
                             impl_->source.string());
  }
}
} // namespace pdlog
