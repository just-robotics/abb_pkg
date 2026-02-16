#include "abb_grip/udp_sender.hpp"
#include <iostream>

namespace asio = boost::asio;
namespace ip = boost::asio::ip;

namespace abb_grip
{

UdpSender::UdpSender(const std::string& host, int port)
  : io_context_(),
    socket_(io_context_)
{
  boost::system::error_code ec;
  socket_.open(ip::udp::v4(), ec);
  if (ec)
  {
    std::cerr << "[Gripper] UdpSender open: " << ec.message() << std::endl;
    return;
  }

  ip::udp::resolver resolver(io_context_);
  auto results = resolver.resolve(
      ip::udp::v4(),
      host,
      std::to_string(port),
      ec);
  if (ec)
  {
    std::cerr << "[Gripper] Resolve " << host << ":" << port
              << " failed: " << ec.message() << std::endl;
    return;
  }
  target_endpoint_ = *results.begin();
  std::cout << "[Gripper] Sending UDP to " << host << ":" << port << std::endl;
}

void UdpSender::send(const std::string& data)
{
  if (data.empty())
  {
    return;
  }
  boost::system::error_code ec;
  socket_.send_to(
      asio::buffer(data.data(), data.size()),
      target_endpoint_,
      0,
      ec);
  if (ec)
  {
    std::cerr << "[Gripper] Send failed: " << ec.message() << std::endl;
  }
}

}  // namespace abb_grip
