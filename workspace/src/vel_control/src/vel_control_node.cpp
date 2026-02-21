#include "vel_control/vel_control_node.hpp"

#include <abb_libegm/egm_wrapper.pb.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace vel_control {

VelControlNode::VelControlNode() : Node("vel_control_node") {
  Controller::Config config;
  config.port = declare_parameter<int>("port", 6515);
  marker_frame_ = declare_parameter<std::string>("marker_frame", "base");
  cmd_vel_lin_gain_ = declare_parameter<double>("cmd_vel_lin_gain", 1.0);
  cmd_vel_ang_gain_ = declare_parameter<double>("cmd_vel_ang_gain", 1.0);
  cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.15);

  default_pose_[0] = declare_parameter<double>("pose_x", 750.0);
  default_pose_[1] = declare_parameter<double>("pose_y", 0.0);
  default_pose_[2] = declare_parameter<double>("pose_z", 1641.0);
  default_pose_[3] = declare_parameter<double>("pose_rx", -180.0);
  default_pose_[4] = declare_parameter<double>("pose_ry", 0.0);
  default_pose_[5] = declare_parameter<double>("pose_rz", -90.0);

  config_ = config;
  controller_ = std::make_unique<Controller>(config);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&VelControlNode::cmdVelCallback, this, std::placeholders::_1));

  ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  ping_pub_ = create_publisher<std_msgs::msg::Bool>("abb_ping", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "egm_markers", rclcpp::SensorDataQoS());

  if (!controller_->initialize()) {
    RCLCPP_FATAL(get_logger(), "EGM controller init failed");
    throw std::runtime_error("EGM controller init failed");
  }

  update_timer_ = create_wall_timer(std::chrono::milliseconds(4),
                                    std::bind(&VelControlNode::update, this));
  state_pub_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                       std::bind(&VelControlNode::publishState, this));
  ping_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                  std::bind(&VelControlNode::ping, this));

  initLineStrip();
  RCLCPP_INFO(get_logger(), "vel_control_node started (port=%d)", config.port);
}

void VelControlNode::fillVelocityCommand(double vx_mm_s, double vy_mm_s, double vz_mm_s,
                                         double vrx_deg_s, double vry_deg_s, double vrz_deg_s,
                                         double lin_gain, double ang_gain,
                                         abb::egm::wrapper::Output& outputs) {
  auto* vel = outputs.mutable_robot()->mutable_cartesian()->mutable_velocity();
  vel->mutable_linear()->set_x(vx_mm_s * lin_gain);
  vel->mutable_linear()->set_y(vy_mm_s * lin_gain);
  vel->mutable_linear()->set_z(vz_mm_s * lin_gain);
  vel->mutable_angular()->set_x(vrx_deg_s * ang_gain);
  vel->mutable_angular()->set_y(vry_deg_s * ang_gain);
  vel->mutable_angular()->set_z(vrz_deg_s * ang_gain);
}

void VelControlNode::fillPoseCommand(double x_mm, double y_mm, double z_mm,
                                     double rx_deg, double ry_deg, double rz_deg,
                                     abb::egm::wrapper::Output& outputs) {
  auto* pose = outputs.mutable_robot()->mutable_cartesian()->mutable_pose();
  pose->mutable_position()->set_x(x_mm);
  pose->mutable_position()->set_y(y_mm);
  pose->mutable_position()->set_z(z_mm);
  pose->mutable_euler()->set_x(rx_deg);
  pose->mutable_euler()->set_y(ry_deg);
  pose->mutable_euler()->set_z(rz_deg);
}

void VelControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  last_cmd_vel_ = *msg;
  last_cmd_vel_time_ = get_clock()->now();
}

void VelControlNode::update() {
  if (!controller_->update()) {
    return;
  }

  abb::egm::wrapper::Output outputs;
  fillPoseCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, outputs);

  bool use_cmd_vel = false;
  geometry_msgs::msg::Twist cmd_copy;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (last_cmd_vel_time_.has_value()) {
      const auto now = get_clock()->now();
      if ((now - *last_cmd_vel_time_).seconds() < cmd_vel_timeout_s_) {
        use_cmd_vel = true;
        cmd_copy = last_cmd_vel_;
      }
    }
  }
  if (use_cmd_vel) {
    double vx = cmd_copy.linear.x * 1000.0;
    double vy = cmd_copy.linear.y * 1000.0;
    double vz = cmd_copy.linear.z * 1000.0;
    double vrx = cmd_copy.angular.x * 180.0 / M_PI;
    double vry = cmd_copy.angular.y * 180.0 / M_PI;
    double vrz = cmd_copy.angular.z * 180.0 / M_PI;
    fillVelocityCommand(vx, vy, vz, vrx, vry, vrz,
                       cmd_vel_lin_gain_, cmd_vel_ang_gain_, outputs);
  } else {
    fillVelocityCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       1.0, 1.0, outputs);
  }
  controller_->write(outputs);
}

void VelControlNode::publishState() {
  const auto& inputs = controller_->getInputs();
  const auto& cart_fb = inputs.feedback().robot().cartesian().pose();
  const auto& joint_fb = inputs.feedback().robot().joints().position();

  geometry_msgs::msg::PoseStamped ee_pose;
  ee_pose.header.stamp = get_clock()->now();
  ee_pose.header.frame_id = marker_frame_;
  ee_pose.pose.position.x = cart_fb.position().x() * 1e-3;
  ee_pose.pose.position.y = cart_fb.position().y() * 1e-3;
  ee_pose.pose.position.z = cart_fb.position().z() * 1e-3;
  tf2::Quaternion q;
  q.setRPY(cart_fb.euler().x() * M_PI / 180.0,
           cart_fb.euler().y() * M_PI / 180.0,
           cart_fb.euler().z() * M_PI / 180.0);
  ee_pose.pose.orientation.x = q.x();
  ee_pose.pose.orientation.y = q.y();
  ee_pose.pose.orientation.z = q.z();
  ee_pose.pose.orientation.w = q.w();
  ee_pose_pub_->publish(ee_pose);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  joint_state.header.frame_id = marker_frame_;
  for (int i = 0; i < joint_fb.values_size(); ++i) {
    joint_state.name.push_back("joint_" + std::to_string(i + 1));
    joint_state.position.push_back(joint_fb.values(i) * M_PI / 180.0);
  }
  joint_state_pub_->publish(joint_state);
}

void VelControlNode::ping() {
  std_msgs::msg::Bool msg;
  msg.data = true;
  ping_pub_->publish(msg);
}

void VelControlNode::initLineStrip() {
  line_strip_.header.frame_id = marker_frame_;
  line_strip_.ns = "egm";
  line_strip_.id = 2;
  line_strip_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_.action = visualization_msgs::msg::Marker::ADD;
  line_strip_.scale.x = 0.01;
  line_strip_.color.b = 1.0;
  line_strip_.color.a = 1.0;
}

}  // namespace vel_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vel_control::VelControlNode>());
  rclcpp::shutdown();
  return 0;
}
