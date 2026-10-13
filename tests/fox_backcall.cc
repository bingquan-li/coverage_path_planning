#include <foxglove/server.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <utility>

int
main()
{
  // SDK 回调可能来自其他线程。
  std::atomic_bool start_requested{false};
  std::atomic_bool reset_requested{false};

  using Key = std::pair<uint32_t, uint32_t>;

  std::mutex channels_mutex;
  std::map<Key, std::string> topics;

  foxglove::WebSocketServerOptions options;
  options.name = "planner_debug";
  options.host = "127.0.0.1";
  options.port = 8765;

  // 允许 Foxglove 向本程序发送消息。
  options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;

  // 本例只解析 JSON。
  options.supported_encodings = {"json"};

  // 客户端声明：“我准备在这个 Topic 上发布消息”。
  options.callbacks.onClientAdvertise =
      [&](uint32_t client_id, foxglove::ClientChannel const& channel)
  {
    std::scoped_lock const lock(channels_mutex);

    std::cout << "Advertise: topic=" << channel.topic
              << ", encoding=" << channel.encoding
              << ", schema=" << channel.schema_name << '\n';

    if(channel.encoding != "json")
    {
      return;
    }

    // 必须复制字符串，不能长期保存 channel.topic 的 string_view。
    topics[{client_id, channel.id}] = std::string(channel.topic);
  };

  // 实际收到按钮消息。
  options.callbacks.onMessageData = [&](uint32_t client_id,
                                        uint32_t channel_id,
                                        std::byte const* data,
                                        size_t size)
  {
    std::string topic;
    {
      std::scoped_lock const lock(channels_mutex);

      auto const it = topics.find({client_id, channel_id});
      if(it == topics.end())
      {
        return;
      }
      topic = it->second;
    }

    if(size == 0)
    {
      return;
    }

    std::string const payload(reinterpret_cast<char const*>(data), size);

    std::cout << "Received: topic=" << topic << ", payload=" << payload << '\n';

    auto const* begin = reinterpret_cast<char const*>(data);
    auto const message =
        nlohmann::json::parse(begin, begin + size, nullptr, false);

    if(!message.is_object())
    {
      return;
    }

    auto const value = message.find("data");
    if(value == message.end() || !value->is_boolean())
    {
      return;
    }

    // Push 松开时发送 false，本例忽略它。
    if(!value->get<bool>())
    {
      return;
    }

    if(topic == "/control/start")
    {
      start_requested.store(true);
      std::cout << "Start requested\n";
    }
    else if(topic == "/control/reset")
    {
      reset_requested.store(true);
      std::cout << "Reset requested\n";
    }
  };

  options.callbacks.onClientUnadvertise =
      [&](uint32_t client_id, uint32_t channel_id)
  {
    std::scoped_lock const lock(channels_mutex);
    topics.erase({client_id, channel_id});
  };

  auto result = foxglove::WebSocketServer::create(std::move(options));
  if(!result.has_value())
  {
    std::cerr << "Create server failed: " << foxglove::strerror(result.error())
              << '\n';
    return 1;
  }

  // 保留 server 对象，让服务端持续运行。
  auto server = std::move(result.value());

  std::cout << "Connect to ws://127.0.0.1:8765\n";
  std::cout << "Press Enter to exit.\n";
  std::cin.get();

  server.stop();
  return 0;
}