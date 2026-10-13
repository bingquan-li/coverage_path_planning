#pragma once

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>

namespace pdlog
{
struct PpmdParameters
{
  unsigned order;
  unsigned memory_mib;
  unsigned restore_method;
};

class PpmdDecoder
{
public:
  // input 已越过参数头；借用输入，调用方必须保证它比解码器活得更久。
  PpmdDecoder(FILE* input,
              PpmdParameters parameters,
              std::filesystem::path const& source);
  ~PpmdDecoder();
  PpmdDecoder(PpmdDecoder const&) = delete;
  PpmdDecoder&
  operator=(PpmdDecoder const&) = delete;

  // nullopt 只表示读到格式结束符；还必须调用 validateEnd()。
  std::optional<std::uint8_t>
  next_byte();
  void
  validate_end() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace pdlog
