#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "abb_grip/udp_sender.hpp"

namespace abb_grip
{

class GripNode : public rclcpp::Node
{
public:
  GripNode()
    : Node("grip_node")
  {
    gripper_ip_ = declare_parameter<std::string>("gripper_ip", "192.168.125.1");
    gripper_port_ = declare_parameter<int>("gripper_port", 6516);

    udp_sender_ = std::make_shared<UdpSender>(gripper_ip_, gripper_port_);

    sub_ = create_subscription<std_msgs::msg::String>(
        "gripper/command",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          this->onCommand(msg->data);
        });

    RCLCPP_INFO(get_logger(), "Gripper node: %s:%d, subscribed to gripper/command",
        gripper_ip_.c_str(), gripper_port_);
  }

private:
  void onCommand(const std::string& data)
  {
    udp_sender_->send(data);
  }

  std::string gripper_ip_;
  int gripper_port_;
  std::shared_ptr<UdpSender> udp_sender_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace abb_grip

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<abb_grip::GripNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
