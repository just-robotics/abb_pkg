#ifndef VEL_CONTROL_VEL_CONTROL_NODE_HPP
#define VEL_CONTROL_VEL_CONTROL_NODE_HPP

#include <abb_libegm/egm_wrapper.pb.h>

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include "vel_control/controller.hpp"

namespace vel_control {

class VelControlNode : public rclcpp::Node {
 public:
  VelControlNode();

 private:
  void update();
  void publishState();
  void ping();

  /** Fill protobuf Output with Cartesian velocities (linear mm/s, angular deg/s). */
  void fillVelocityCommand(double vx_mm_s, double vy_mm_s, double vz_mm_s,
                           double vrx_deg_s, double vry_deg_s, double vrz_deg_s,
                           double lin_gain, double ang_gain,
                           abb::egm::wrapper::Output& outputs);

  /** Fill protobuf Output pose (position mm, euler deg). */
  void fillPoseCommand(double x_mm, double y_mm, double z_mm,
                      double rx_deg, double ry_deg, double rz_deg,
                      abb::egm::wrapper::Output& outputs);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::string marker_frame_{"base"};
  std::unique_ptr<Controller> controller_;
  Controller::Config config_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ping_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  rclcpp::TimerBase::SharedPtr ping_timer_;

  visualization_msgs::msg::Marker line_strip_;

  std::mutex feedback_mutex_;
  std::array<double, 6> last_feedback_pose_{};
  double cmd_vel_lin_gain_{1.0};
  double cmd_vel_ang_gain_{1.0};
  double cmd_vel_timeout_s_{0.15};

  /** cmd_vel cache: only update in callback; update() reads and sends. */
  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist last_cmd_vel_{};
  std::optional<rclcpp::Time> last_cmd_vel_time_;

  /** Pose (mm, deg) for default/target; used only if needed. */
  std::array<double, 6> default_pose_{750.0, 0.0, 1641.0, -180.0, 0.0, -90.0};

  void initLineStrip();
};

}  // namespace vel_control

#endif  // VEL_CONTROL_VEL_CONTROL_NODE_HPP
