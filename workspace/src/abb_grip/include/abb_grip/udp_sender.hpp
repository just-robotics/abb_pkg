#ifndef ABB_GRIP__UDP_SENDER_HPP_
#define ABB_GRIP__UDP_SENDER_HPP_

#include <string>
#include <boost/asio.hpp>

namespace abb_grip
{

/**
 * UDP sender for gripper commands (e.g. to robot controller at 192.168.125.1:6516).
 * Sends string data over UDP; used by EGM/gripper integration from new_abb_pkg.
 */
class UdpSender
{
public:
  UdpSender(const std::string& host, int port);
  void send(const std::string& data);

  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;

private:
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint target_endpoint_;
};

}  // namespace abb_grip

#endif  // ABB_GRIP__UDP_SENDER_HPP_
