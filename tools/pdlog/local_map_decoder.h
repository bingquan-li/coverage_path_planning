#pragma once
#include <zlib.h>

#include <array>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace local_map_decoder
{
using Bytes = std::vector<unsigned char>;

inline Bytes
decodeBase64(std::string const& text)
{
  if(text.empty() || text.size() % 4 != 0)
  {
    throw std::runtime_error("Invalid Base64 length");
  }

  auto const value = [](char c) -> int
  {
    if(c >= 'A' && c <= 'Z')
    {
      return c - 'A';
    }
    if(c >= 'a' && c <= 'z')
    {
      return c - 'a' + 26;
    }
    if(c >= '0' && c <= '9')
    {
      return c - '0' + 52;
    }
    if(c == '+')
    {
      return 62;
    }
    if(c == '/')
    {
      return 63;
    }
    return -1;
  };

  Bytes out;
  out.reserve(text.size() / 4 * 3);
  for(std::size_t i = 0; i < text.size(); i += 4)
  {
    bool const pad2 = text[i + 2] == '=';
    bool const pad3 = text[i + 3] == '=';
    int const a = value(text[i]);
    int const b = value(text[i + 1]);
    int const c = pad2 ? 0 : value(text[i + 2]);
    int const d = pad3 ? 0 : value(text[i + 3]);

    if(a < 0 || b < 0 || c < 0 || d < 0 || (pad2 && !pad3) ||
       ((pad2 || pad3) && i + 4 != text.size()) || (pad2 && (b & 15) != 0) ||
       (pad3 && !pad2 && (c & 3) != 0))
    {
      throw std::runtime_error("Invalid Base64 data");
    }

    out.push_back(static_cast<unsigned char>((a << 2) | (b >> 4)));
    if(!pad2)
    {
      out.push_back(static_cast<unsigned char>((b << 4) | (c >> 2)));
    }
    if(!pad3)
    {
      out.push_back(static_cast<unsigned char>((c << 6) | d));
    }
  }
  return out;
}

inline Bytes
decompressGzip(Bytes const& compressed)
{
  if(compressed.empty() || compressed.size() > std::numeric_limits<uInt>::max())
  {
    throw std::runtime_error("Invalid gzip input size");
  }

  z_stream stream{};
  // 15 + 16: 解码 gzip 格式，而非普通 zlib 或裸 deflate。
  if(inflateInit2(&stream, 15 + 16) != Z_OK)
  {
    throw std::runtime_error("inflateInit2 failed");
  }

  struct Cleanup
  {
    z_stream* stream;
    ~Cleanup()
    {
      inflateEnd(stream);
    }
  } cleanup{&stream};

  stream.next_in = const_cast<Bytef*>(compressed.data());
  stream.avail_in = static_cast<uInt>(compressed.size());

  Bytes out;
  std::array<unsigned char, 4096> buffer{};
  auto const max_bytes = static_cast<std::size_t const>(64U * 1024U * 1024U);
  for(;;)
  {
    stream.next_out = buffer.data();
    stream.avail_out = static_cast<uInt>(buffer.size());
    int const ret = inflate(&stream, Z_NO_FLUSH);
    if(ret != Z_OK && ret != Z_STREAM_END)
    {
      throw std::runtime_error("Invalid or truncated gzip data");
    }

    std::size_t const count = buffer.size() - stream.avail_out;
    if(count > max_bytes - out.size())
    {
      throw std::runtime_error("Decompressed data exceeds 64 MiB");
    }
    out.insert(out.end(), buffer.begin(), buffer.begin() + count);

    if(ret == Z_STREAM_END)
    {
      if(stream.avail_in != 0)
      {
        throw std::runtime_error("Unexpected bytes after gzip stream");
      }
      return out;
    }
  }
}

struct LocalMapData
{
  Bytes packed; // gzip 解压后的原始字节
  Bytes cells;  // 每个元素对应一个栅格，取值 0~3
};

inline LocalMapData
decodeLocalMapData(std::string const& data)
{
  LocalMapData result;
  result.packed = decompressGzip(decodeBase64(data));
  result.cells.reserve(result.packed.size() * 4);
  for(unsigned char b : result.packed)
  {
    result.cells.push_back((b >> 6) & 3);
    result.cells.push_back((b >> 4) & 3);
    result.cells.push_back((b >> 2) & 3);
    result.cells.push_back(b & 3);
  }
  return result;
}
} // namespace local_map_decoder
