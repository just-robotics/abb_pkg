#include "abb_pkg/egm_node.hpp"

#include <abb_libegm/egm_wrapper.pb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdexcept>
#include <thread>

EgmNode::EgmNode() : Node("egm_node") {
  abb_pkg::Controller::Config config;
  config.port = declare_parameter<int>("port", 6515);
  config.dlin = declare_parameter<double>("dlin", 50.0);
  config.radius = declare_parameter<double>("radius", 300.0);
  config.dtheta = declare_parameter<double>("dtheta", 0.005);

  config.x0 = declare_parameter<double>("x0", 770.0);
  config.y0 = declare_parameter<double>("y0", 0.0);
  config.x1 = declare_parameter<double>("x1", 770.0);
  config.y1 = declare_parameter<double>("y1", 0.0);
  config.z0 = declare_parameter<double>("z0", 1000.0);

  config.rx0 = declare_parameter<double>("rx0", 180.0);
  config.ry0 = declare_parameter<double>("ry0", 0.0);
  config.rz0 = declare_parameter<double>("rz0", 90.0);

  marker_frame_ = declare_parameter<std::string>("marker_frame", "base");
  config.base_link = declare_parameter<std::string>("base_link", "base_link");
  config.tip_link = declare_parameter<std::string>("tip_link", "tool0");
  config.urdf_string = declare_parameter<std::string>("robot_description", "");
  config.ik_timeout = declare_parameter<double>("ik_timeout", 0.5);
  config.ik_eps = declare_parameter<double>("ik_eps", 1e-3);
  config.dq_limit = declare_parameter<double>("dq_limit", 0.2);
  config.joint_tolerance = declare_parameter<double>("joint_tolerance", 0.5);

  cmd_vel_lin_gain_ = declare_parameter<double>("cmd_vel_lin_gain", 1.0);
  cmd_vel_ang_gain_ = declare_parameter<double>("cmd_vel_ang_gain", 1.0);
  cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.15);

  if (config.urdf_string.empty()) {
    RCLCPP_FATAL(get_logger(), "robot_description parameter is EMPTY");
    throw std::runtime_error("Missing URDF");
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&EgmNode::cmdVelCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Creating publishers...");
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "egm_markers", rclcpp::SensorDataQoS());
  ee_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);
  joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  ping_pub_ = create_publisher<std_msgs::msg::Bool>("abb_ping", 10);

  RCLCPP_INFO(get_logger(), "Creating controller instance...");
  config_ = config;
  controller_ = std::make_unique<abb_pkg::Controller>(config);

  RCLCPP_INFO(get_logger(),
              "Initializing controller (this may take a moment)...");
  if (!controller_->initialize()) {
    RCLCPP_FATAL(get_logger(), "EGM controller init failed");
    throw std::runtime_error("EGM controller init failed");
  }
  RCLCPP_INFO(get_logger(), "Controller initialized successfully");

  RCLCPP_INFO(get_logger(), "Creating action server...");
  action_server_ = rclcpp_action::create_server<abb_pkg::action::MoveToPose>(
      this, "move_to_pose",
      std::bind(&EgmNode::handleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&EgmNode::handleCancel, this, std::placeholders::_1),
      std::bind(&EgmNode::executeAction, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Creating timers...");
  update_timer_ = create_wall_timer(std::chrono::milliseconds(4),
                                    std::bind(&EgmNode::update, this));
  RCLCPP_INFO(get_logger(), "Update timer created and started (4ms interval)");
  state_pub_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                       std::bind(&EgmNode::publishState, this));
  ping_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                  std::bind(&EgmNode::ping, this));

  RCLCPP_INFO(get_logger(), "Initializing line strip marker...");
  initLineStrip();

  RCLCPP_INFO(get_logger(), "EGM node started (port=%d)", config.port);
  RCLCPP_INFO(get_logger(), "Action server ready at: move_to_pose");
  RCLCPP_INFO(get_logger(), "Publishing state to: ee_pose, joint_states");
}

rclcpp_action::GoalResponse EgmNode::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const abb_pkg::action::MoveToPose::Goal> goal) {
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(get_logger(), "Received goal request");

  if (executing_action_.load()) {
    RCLCPP_WARN(get_logger(),
                "Already executing an action, rejecting new goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EgmNode::handleCancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>>
        goal_handle) {
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EgmNode::executeAction(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>>
        goal_handle) {
  // IMPORTANT: This callback MUST return quickly.
  // If we block here, the executor can't run timers, so update() won't send EGM
  // commands.
  RCLCPP_INFO(get_logger(), "Action accepted (non-blocking), setting target");

  cancel_requested_.store(false);
  executing_action_.store(true);
  current_goal_handle_ = goal_handle;

  const auto goal = goal_handle->get_goal();
  {
    std::lock_guard<std::mutex> lock(target_pose_mutex_);
    target_pose_ = poseToTarget(goal->position, goal->orientation);
  }

  RCLCPP_INFO(get_logger(),
              "Target pose: x=%.1f mm, y=%.1f mm, z=%.1f mm, "
              "rx=%.1f deg, ry=%.1f deg, rz=%.1f deg",
              target_pose_[0], target_pose_[1], target_pose_[2],
              target_pose_[3], target_pose_[4], target_pose_[5]);
}

void EgmNode::update() {
  static int update_call_count = 0;
  static int update_fail_count = 0;

  update_call_count++;
  // Log every 250 iterations (~1 second at 4ms rate) AND first 10 calls
  if (update_call_count % 250 == 0 || update_call_count <= 10) {
    size_t target_size;
    {
      std::lock_guard<std::mutex> lock(target_pose_mutex_);
      target_size = target_pose_.size();
    }
    RCLCPP_INFO(get_logger(),
                "[UPDATE] Called %d times, executing_action_=%s, "
                "target_pose_.size()=%zu",
                update_call_count, executing_action_.load() ? "true" : "false",
                target_size);
  }

  if (!controller_->update()) {
    update_fail_count++;
    if (update_fail_count % 1000 == 0) {
      RCLCPP_WARN(
          get_logger(),
          "Controller update failed %d times - robot may not be connected",
          update_fail_count);
    }
    return;
  }

  // If EGM state is not OK, the library may ignore external outputs (statesOk()
  // == false). Make this explicit in logs/results so it's not a silent "robot
  // doesn't move".
  const auto& inputs_now = controller_->getInputs();
  if (inputs_now.has_status()) {
    const auto& st = inputs_now.status();
    const bool status_ok =
        (st.egm_state() == abb::egm::wrapper::Status::EGM_RUNNING) &&
        (st.motor_state() == abb::egm::wrapper::Status::MOTORS_ON) &&
        (st.rapid_execution_state() ==
         abb::egm::wrapper::Status::RAPID_RUNNING);

    static int status_log_counter = 0;
    if (++status_log_counter % 250 == 0 ||
        (executing_action_.load() && !status_ok)) {
      RCLCPP_INFO(get_logger(),
                  "[STATUS] egm_state=%d motor_state=%d rapid_state=%d ok=%s",
                  static_cast<int>(st.egm_state()),
                  static_cast<int>(st.motor_state()),
                  static_cast<int>(st.rapid_execution_state()),
                  status_ok ? "true" : "false");
    }

    // Grace period: allow status to recover for a short time, otherwise fail
    // goal.
    constexpr double status_grace_sec = 2.0;
    if (!status_ok) {
      if (!status_not_ok_since_) {
        status_not_ok_since_ = get_clock()->now();
      }
    } else {
      status_not_ok_since_.reset();
    }

    if (executing_action_.load() && current_goal_handle_ && !status_ok) {
      const auto now = get_clock()->now();
      const double bad_for =
          status_not_ok_since_ ? (now - *status_not_ok_since_).seconds() : 0.0;

      if (bad_for >= status_grace_sec) {
        auto result = std::make_shared<abb_pkg::action::MoveToPose::Result>();
        result->success = false;
        result->message =
            "EGM status not OK for too long (need EGM_RUNNING + MOTORS_ON + "
            "RAPID_RUNNING). "
            "Check robot RAPID program (EGMActPose), motors, and RAPID state.";
        current_goal_handle_->abort(result);
        executing_action_.store(false);
        current_goal_handle_.reset();
        cancel_requested_.store(false);
        status_not_ok_since_.reset();
        return;
      }
      // Otherwise: keep goal alive and wait for status to recover.
    }
  } else {
    // If no status is available, still continue; but log once when executing.
    static bool warned_no_status = false;
    if (executing_action_.load() && !warned_no_status) {
      RCLCPP_WARN(get_logger(), "EGM wrapper Input has no status() field");
      warned_no_status = true;
    }
  }

  const auto& inputs_for_cache = controller_->getInputs();
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    const auto& fb = inputs_for_cache.feedback().robot().cartesian().pose();
    last_feedback_pose_[0] = fb.position().x();
    last_feedback_pose_[1] = fb.position().y();
    last_feedback_pose_[2] = fb.position().z();
    last_feedback_pose_[3] = fb.euler().x();
    last_feedback_pose_[4] = fb.euler().y();
    last_feedback_pose_[5] = fb.euler().z();
  }

  // If executing action, move toward target
  // Otherwise, maintain current position or apply cmd_vel velocity
  abb::egm::wrapper::Output outputs;

  constexpr double position_tolerance_mm = 5.0;
  constexpr double orientation_tolerance_deg = 2.0;

  if (executing_action_.load() && current_goal_handle_) {
    // Snapshot target pose
    std::vector<double> local_target_pose;
    {
      std::lock_guard<std::mutex> lock(target_pose_mutex_);
      local_target_pose = target_pose_;
    }
 
    if (local_target_pose.size() != 6) {
      // Abort goal if target is invalid
      auto result = std::make_shared<abb_pkg::action::MoveToPose::Result>();
      result->success = false;
      result->message = "Invalid target pose";
      current_goal_handle_->abort(result);
      executing_action_.store(false);
      current_goal_handle_.reset();
      cancel_requested_.store(false);
      return;
    }

    const auto& inputs = controller_->getInputs();
    const auto& fb = inputs.feedback().robot().cartesian().pose();

    const double cx = fb.position().x();
    const double cy = fb.position().y();
    const double cz = fb.position().z();
    const double crx = fb.euler().x();
    const double cry = fb.euler().y();
    const double crz = fb.euler().z();

    const double dx = local_target_pose[0] - cx;
    const double dy = local_target_pose[1] - cy;
    const double dz = local_target_pose[2] - cz;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double drx = std::abs(local_target_pose[3] - crx);
    const double dry = std::abs(local_target_pose[4] - cry);
    const double drz = std::abs(local_target_pose[5] - crz);
    const double ori_error = std::max({drx, dry, drz});

    // Feedback (throttled)
    static int fb_counter = 0;
    if (++fb_counter % 25 == 0) {  // ~100ms
      auto feedback = std::make_shared<abb_pkg::action::MoveToPose::Feedback>();
      feedback->current_position.x = cx * 1e-3;
      feedback->current_position.y = cy * 1e-3;
      feedback->current_position.z = cz * 1e-3;
      tf2::Quaternion q;
      q.setRPY(crx * M_PI / 180.0, cry * M_PI / 180.0, crz * M_PI / 180.0);
      feedback->current_orientation.x = q.x();
      feedback->current_orientation.y = q.y();
      feedback->current_orientation.z = q.z();
      feedback->current_orientation.w = q.w();
      feedback->distance_to_target = dist;
      current_goal_handle_->publish_feedback(feedback);
    }

    // Cancel
    if (cancel_requested_.load() || current_goal_handle_->is_canceling()) {
      auto result = std::make_shared<abb_pkg::action::MoveToPose::Result>();
      result->success = false;
      result->message = "Cancelled";
      current_goal_handle_->canceled(result);
      executing_action_.store(false);
      current_goal_handle_.reset();
      cancel_requested_.store(false);
      return;
    }

    // Success
    if (dist < position_tolerance_mm && ori_error < orientation_tolerance_deg) {
      auto result = std::make_shared<abb_pkg::action::MoveToPose::Result>();
      result->success = true;
      result->message = "Reached target";
      current_goal_handle_->succeed(result);
      executing_action_.store(false);
      current_goal_handle_.reset();
      cancel_requested_.store(false);
      return;
    }

    // Command motion
    controller_->stepToward(local_target_pose, outputs);
  } else {
    const auto& inputs = controller_->getInputs();
    const auto& fb = inputs.feedback().robot().cartesian().pose();

    auto* pose = outputs.mutable_robot()->mutable_cartesian()->mutable_pose();
    pose->mutable_position()->set_x(fb.position().x());
    pose->mutable_position()->set_y(fb.position().y());
    pose->mutable_position()->set_z(fb.position().z());
    pose->mutable_euler()->set_x(fb.euler().x());
    pose->mutable_euler()->set_y(fb.euler().y());
    pose->mutable_euler()->set_z(fb.euler().z());

    bool use_cmd_vel = false;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      if (last_cmd_vel_time_.has_value()) {
        const auto now = get_clock()->now();
        if ((now - *last_cmd_vel_time_).seconds() < cmd_vel_timeout_s_) {
          use_cmd_vel = true;
          fillVelocityOutput(
              last_cmd_vel_.linear.x * 1000.0,
              last_cmd_vel_.linear.y * 1000.0,
              last_cmd_vel_.linear.z * 1000.0,
              last_cmd_vel_.angular.x * 180.0 / M_PI,
              last_cmd_vel_.angular.y * 180.0 / M_PI,
              last_cmd_vel_.angular.z * 180.0 / M_PI,
              cmd_vel_lin_gain_, cmd_vel_ang_gain_, outputs);
        }
      }
    }
  }

  controller_->write(outputs);

  publishMarkers();
}

void EgmNode::publishState() {
  const auto& inputs = controller_->getInputs();
  const auto& cart_fb = inputs.feedback().robot().cartesian().pose();
  const auto& joint_fb = inputs.feedback().robot().joints().position();

  // Publish end effector pose
  geometry_msgs::msg::PoseStamped ee_pose;
  ee_pose.header.stamp = get_clock()->now();
  ee_pose.header.frame_id = marker_frame_;

  ee_pose.pose.position.x = cart_fb.position().x() * 1e-3;  // Convert to meters
  ee_pose.pose.position.y = cart_fb.position().y() * 1e-3;
  ee_pose.pose.position.z = cart_fb.position().z() * 1e-3;

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(cart_fb.euler().x() * M_PI / 180.0,
           cart_fb.euler().y() * M_PI / 180.0,
           cart_fb.euler().z() * M_PI / 180.0);
  ee_pose.pose.orientation.x = q.x();
  ee_pose.pose.orientation.y = q.y();
  ee_pose.pose.orientation.z = q.z();
  ee_pose.pose.orientation.w = q.w();

  ee_pose_pub_->publish(ee_pose);

  // Publish joint states
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  joint_state.header.frame_id = marker_frame_;

  // Joint names (adjust based on your robot)
  for (int i = 0; i < joint_fb.values_size(); ++i) {
    joint_state.name.push_back("joint_" + std::to_string(i + 1));
    joint_state.position.push_back(joint_fb.values(i) * M_PI /
                                   180.0);  // Convert to radians
  }

  joint_state_pub_->publish(joint_state);
}

void EgmNode::publishMarkers() {
  const auto& inputs = controller_->getInputs();
  const auto& fb = inputs.feedback().robot().cartesian().pose();

  geometry_msgs::msg::Point p;
  p.x = fb.position().x() * 1e-3;
  p.y = fb.position().y() * 1e-3;
  p.z = fb.position().z() * 1e-3;
  line_strip_.points.push_back(p);

  constexpr size_t max_points = 10000;
  if (line_strip_.points.size() > max_points) {
    line_strip_.points.erase(
        line_strip_.points.begin(),
        line_strip_.points.begin() + (line_strip_.points.size() - max_points));
  }

  line_strip_.header.stamp = get_clock()->now();
  marker_pub_->publish(line_strip_);
}

void EgmNode::publishSphere(const std::vector<double>& pos, int id, float r,
                            float g, float b) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = marker_frame_;
  m.header.stamp = get_clock()->now();
  m.ns = "egm";
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;

  m.pose.position.x = pos[0] * 1e-3;
  m.pose.position.y = pos[1] * 1e-3;
  m.pose.position.z = pos[2] * 1e-3;

  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;

  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;

  marker_pub_->publish(m);
}

void EgmNode::initLineStrip() {
  line_strip_.header.frame_id = marker_frame_;
  line_strip_.ns = "egm";
  line_strip_.id = 2;
  line_strip_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip_.action = visualization_msgs::msg::Marker::ADD;
  line_strip_.scale.x = 0.01;
  line_strip_.color.b = 1.0;
  line_strip_.color.a = 1.0;
}

void EgmNode::ping() {
  std_msgs::msg::Bool msg;
  msg.data = true;
  ping_pub_->publish(msg);
}

std::vector<double> EgmNode::quaternionToEuler(
    const geometry_msgs::msg::Quaternion& quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Convert to degrees
  return {roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
}

std::vector<double> EgmNode::poseToTarget(
    const geometry_msgs::msg::Point& position,
    const geometry_msgs::msg::Quaternion& orientation) {
  std::vector<double> euler = quaternionToEuler(orientation);

  // Convert position from meters to millimeters
  return {
      position.x * 1000.0,  // x in mm
      position.y * 1000.0,  // y in mm
      position.z * 1000.0,  // z in mm
      euler[0],             // rx in degrees
      euler[1],             // ry in degrees
      euler[2]              // rz in degrees
  };
}

void EgmNode::fillVelocityOutput(double dx, double dy, double dz,
                                 double drx, double dry, double drz,
                                 double lin_gain, double ang_gain,
                                 abb::egm::wrapper::Output& outputs) {
  auto* vel = outputs.mutable_robot()->mutable_cartesian()->mutable_velocity();
  vel->mutable_linear()->set_x(dx * lin_gain);   // mm/s
  vel->mutable_linear()->set_y(dy * lin_gain);
  vel->mutable_linear()->set_z(dz * lin_gain);
  vel->mutable_angular()->set_x(drx * ang_gain);  // deg/s
  vel->mutable_angular()->set_y(dry * ang_gain);
  vel->mutable_angular()->set_z(drz * ang_gain);
}

void EgmNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::array<double, 6> pose_copy;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    pose_copy = last_feedback_pose_;
    last_cmd_vel_ = *msg;
    last_cmd_vel_time_ = get_clock()->now();
  }

  abb::egm::wrapper::Output outputs;
  auto* pose = outputs.mutable_robot()->mutable_cartesian()->mutable_pose();
  pose->mutable_position()->set_x(pose_copy[0]);
  pose->mutable_position()->set_y(pose_copy[1]);
  pose->mutable_position()->set_z(pose_copy[2]);
  pose->mutable_euler()->set_x(pose_copy[3]);
  pose->mutable_euler()->set_y(pose_copy[4]);
  pose->mutable_euler()->set_z(pose_copy[5]);

  const double dx = msg->linear.x * 1000.0;   // m/s -> mm/s
  const double dy = msg->linear.y * 1000.0;
  const double dz = msg->linear.z * 1000.0;
  const double drx = msg->angular.x * 180.0 / M_PI;  // rad/s -> deg/s
  const double dry = msg->angular.y * 180.0 / M_PI;
  const double drz = msg->angular.z * 180.0 / M_PI;
  fillVelocityOutput(dx, dy, dz, drx, dry, drz,
                    cmd_vel_lin_gain_, cmd_vel_ang_gain_, outputs);

  controller_->write(outputs);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgmNode>());
  rclcpp::shutdown();
  return 0;
}
