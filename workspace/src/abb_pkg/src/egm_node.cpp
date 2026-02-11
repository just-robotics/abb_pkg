#include "abb_pkg/egm_node.hpp"

#include <abb_libegm/egm_wrapper.pb.h>

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <stdexcept>
#include <thread>

EgmNode::EgmNode() : Node("egm_node") {
  using std::placeholders::_1;

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

  if (config.urdf_string.empty()) {
    RCLCPP_FATAL(get_logger(), "robot_description parameter is EMPTY");
    throw std::runtime_error("Missing URDF");
  }

  RCLCPP_INFO(get_logger(), "Creating marker publisher...");
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "egm_markers", rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(), "Creating controller instance...");
  config_ = config;  // Store config for later use
  controller_ = std::make_unique<abb_pkg::Controller>(config);

  RCLCPP_INFO(get_logger(),
              "Initializing controller (this may take a moment)...");
  if (!controller_->initialize()) {
    RCLCPP_FATAL(get_logger(), "EGM controller init failed");
    throw std::runtime_error("EGM controller init failed");
  }
  RCLCPP_INFO(get_logger(), "Controller initialized successfully");

  RCLCPP_INFO(get_logger(), "Creating subscription for figure_type topic...");
  type_sub_ = this->create_subscription<std_msgs::msg::String>(
      "figure_type", 10, std::bind(&EgmNode::typeCallback, this, _1));

  RCLCPP_INFO(get_logger(), "Creating update timer...");
  timer_ = create_wall_timer(std::chrono::milliseconds(4),
                             std::bind(&EgmNode::update, this));

  RCLCPP_INFO(get_logger(),
              "Cancelling timer (will be started on trajectory command)...");
  timer_->cancel();

  RCLCPP_INFO(get_logger(), "Initializing line strip marker...");
  initLineStrip();

  RCLCPP_INFO(get_logger(), "EGM node started (port=%d)", config.port);
  RCLCPP_INFO(get_logger(),
              "Waiting for robot connection and trajectory commands...");
  RCLCPP_INFO(get_logger(),
              "Try publishing to /figure_type topic: circle, line, "
              "circle_joints, line_joints, or test_ik");
}

void EgmNode::typeCallback(const std_msgs::msg::String::SharedPtr msg) {
  mode_ = ControlMode::CARTESIAN;

  RCLCPP_INFO(get_logger(), "Received trajectory type: %s", msg->data.c_str());

  if (msg->data == "circle") {
    trajectory_ = controller_->generateCircle();
  }

  else if (msg->data == "line") {
    trajectory_ = controller_->generateLine();
  }

  else if (msg->data == "circle_joints") {
    trajectory_ = controller_->generateCircle();
    RCLCPP_INFO(get_logger(), "Generated circle trajectory with %zu points",
                trajectory_.size());
    if (!trajectory_.empty()) {
      RCLCPP_INFO(
          get_logger(),
          "First point: x=%.1f, y=%.1f, z=%.1f, rx=%.1f, ry=%.1f, rz=%.1f",
          trajectory_[0][0], trajectory_[0][1], trajectory_[0][2],
          trajectory_[0][3], trajectory_[0][4], trajectory_[0][5]);
      if (trajectory_.size() > 1) {
        RCLCPP_INFO(
            get_logger(),
            "Second point: x=%.1f, y=%.1f, z=%.1f, rx=%.1f, ry=%.1f, rz=%.1f",
            trajectory_[1][0], trajectory_[1][1], trajectory_[1][2],
            trajectory_[1][3], trajectory_[1][4], trajectory_[1][5]);
      }
    }
    // Try to update robot state before computing IK (multiple attempts)
    RCLCPP_INFO(get_logger(),
                "Attempting to update robot state for circle_joints...");
    bool robot_updated = false;
    for (int i = 0; i < 10; ++i) {
      RCLCPP_INFO(get_logger(), "Update attempt %d/10...", i + 1);

      // Run update() with timeout to avoid blocking
      std::atomic<bool> update_result{false};
      std::atomic<bool> update_complete{false};
      std::thread update_thread([&]() {
        update_result = controller_->update();
        update_complete = true;
      });

      // Wait up to 100ms for update to complete
      auto start = std::chrono::steady_clock::now();
      while (!update_complete && (std::chrono::steady_clock::now() - start) <
                                     std::chrono::milliseconds(100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (update_complete) {
        update_thread.join();
        if (update_result) {
          robot_updated = true;
          RCLCPP_INFO(get_logger(),
                      "Robot state updated successfully (attempt %d)", i + 1);
          break;
        }
        RCLCPP_INFO(get_logger(), "Update failed on attempt %d", i + 1);
      } else {
        RCLCPP_WARN(get_logger(),
                    "Update timed out on attempt %d (waiting >100ms)", i + 1);
        update_thread.detach();  // Detach thread that's still running
      }

      if (i < 9) {
        RCLCPP_INFO(get_logger(), "Waiting 10ms before retry...");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } else {
        RCLCPP_WARN(
            get_logger(),
            "All 10 update attempts failed - robot may not be connected");
      }
    }

    // Print current robot position
    if (robot_updated) {
      const auto& inputs = controller_->getInputs();
      const auto& cart_fb = inputs.feedback().robot().cartesian().pose();
      const auto& joint_fb = inputs.feedback().robot().joints().position();

      RCLCPP_INFO(get_logger(), "=== Current Robot Position ===");
      RCLCPP_INFO(get_logger(),
                  "Cartesian: x=%.1f mm, y=%.1f mm, z=%.1f mm, rx=%.1f deg, "
                  "ry=%.1f deg, rz=%.1f deg",
                  cart_fb.position().x(), cart_fb.position().y(),
                  cart_fb.position().z(), cart_fb.euler().x(),
                  cart_fb.euler().y(), cart_fb.euler().z());

      RCLCPP_INFO(get_logger(), "Joints:");
      for (int i = 0; i < joint_fb.values_size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Joint %d: %.2f deg", i + 1,
                    joint_fb.values(i));
      }
      RCLCPP_INFO(get_logger(), "==============================");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Failed to update robot state - using last known position");
    }

    joint_traj_ = controller_->trajectoryToJoints(trajectory_);

    std::vector<std::vector<double>> filtered_traj;
    std::vector<std::vector<double>> filtered_joint;

    for (size_t i = 0; i < joint_traj_.size(); ++i) {
      if (!joint_traj_[i].empty()) {
        filtered_traj.push_back(trajectory_[i]);
        filtered_joint.push_back(joint_traj_[i]);
      } else {
        RCLCPP_WARN(get_logger(), "IK failed at point %zu — skipping", i);
      }
    }

    RCLCPP_INFO(get_logger(), "IK status: %zu / %zu", filtered_joint.size(),
                joint_traj_.size());

    if (filtered_joint.empty()) {
      RCLCPP_ERROR(get_logger(), "All IK points failed");
      return;
    }

    trajectory_.swap(filtered_traj);
    joint_traj_.swap(filtered_joint);

    mode_ = ControlMode::JOINTS;
  }

  else if (msg->data == "line_joints") {
    RCLCPP_INFO(get_logger(), "Generating line trajectory...");
    trajectory_ = controller_->generateLine();
    RCLCPP_INFO(get_logger(), "Generated line trajectory with %zu points",
                trajectory_.size());

    // Try to update robot state before computing IK (multiple attempts)
    RCLCPP_INFO(get_logger(), "Attempting to update robot state...");
    bool robot_updated = false;
    for (int i = 0; i < 10; ++i) {
      RCLCPP_INFO(get_logger(), "Update attempt %d/10...", i + 1);

      // Run update() with timeout to avoid blocking
      std::atomic<bool> update_result{false};
      std::atomic<bool> update_complete{false};
      std::thread update_thread([&]() {
        update_result = controller_->update();
        update_complete = true;
      });

      // Wait up to 100ms for update to complete
      auto start = std::chrono::steady_clock::now();
      while (!update_complete && (std::chrono::steady_clock::now() - start) <
                                     std::chrono::milliseconds(100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (update_complete) {
        update_thread.join();
        if (update_result) {
          robot_updated = true;
          RCLCPP_INFO(get_logger(),
                      "Robot state updated successfully (attempt %d)", i + 1);
          break;
        }
        RCLCPP_INFO(get_logger(), "Update failed on attempt %d", i + 1);
      } else {
        RCLCPP_WARN(get_logger(),
                    "Update timed out on attempt %d (waiting >100ms)", i + 1);
        update_thread.detach();  // Detach thread that's still running
      }

      if (i < 9) {
        RCLCPP_INFO(get_logger(), "Waiting 10ms before retry...");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } else {
        RCLCPP_WARN(
            get_logger(),
            "All 10 update attempts failed - robot may not be connected");
      }
    }

    // Print current robot position
    if (robot_updated) {
      const auto& inputs = controller_->getInputs();
      const auto& cart_fb = inputs.feedback().robot().cartesian().pose();
      const auto& joint_fb = inputs.feedback().robot().joints().position();

      RCLCPP_INFO(get_logger(), "=== Current Robot Position ===");
      RCLCPP_INFO(get_logger(),
                  "Cartesian: x=%.1f mm, y=%.1f mm, z=%.1f mm, rx=%.1f deg, "
                  "ry=%.1f deg, rz=%.1f deg",
                  cart_fb.position().x(), cart_fb.position().y(),
                  cart_fb.position().z(), cart_fb.euler().x(),
                  cart_fb.euler().y(), cart_fb.euler().z());

      RCLCPP_INFO(get_logger(), "Joints:");
      for (int i = 0; i < joint_fb.values_size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Joint %d: %.2f deg", i + 1,
                    joint_fb.values(i));
      }
      RCLCPP_INFO(get_logger(), "==============================");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Failed to update robot state - using last known position");
    }

    RCLCPP_INFO(get_logger(),
                "Starting IK computation for %zu trajectory points (this may "
                "take a while)...",
                trajectory_.size());
    auto start_time = std::chrono::steady_clock::now();
    joint_traj_ = controller_->trajectoryToJoints(trajectory_);
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    RCLCPP_INFO(get_logger(), "IK computation completed in %ld ms",
                duration.count());
    RCLCPP_INFO(get_logger(), "Filtering IK results...");

    std::vector<std::vector<double>> filtered_traj;
    std::vector<std::vector<double>> filtered_joint;

    for (size_t i = 0; i < joint_traj_.size(); ++i) {
      if (!joint_traj_[i].empty()) {
        filtered_traj.push_back(trajectory_[i]);
        filtered_joint.push_back(joint_traj_[i]);
      } else {
        RCLCPP_WARN(get_logger(), "IK failed at point %zu — skipping", i);
      }
    }

    RCLCPP_INFO(get_logger(), "IK status: %zu / %zu", filtered_joint.size(),
                joint_traj_.size());

    if (filtered_joint.empty()) {
      RCLCPP_ERROR(get_logger(), "All IK points failed");
      return;
    }

    RCLCPP_INFO(get_logger(), "Swapping filtered trajectories...");
    trajectory_.swap(filtered_traj);
    joint_traj_.swap(filtered_joint);

    RCLCPP_INFO(get_logger(),
                "Setting mode to JOINTS and starting trajectory execution...");
    mode_ = ControlMode::JOINTS;
  } else if (msg->data == "test_ik") {
    // Test IK with a single point
    RCLCPP_INFO(get_logger(), "Running IK test...");
    std::vector<std::vector<double>> sanity{{750, 0, 1121.5, 180, 0, -90}};

    RCLCPP_INFO(get_logger(),
                "Test point: x=%.1f, y=%.1f, z=%.1f, rx=%.1f, ry=%.1f, rz=%.1f",
                sanity[0][0], sanity[0][1], sanity[0][2], sanity[0][3],
                sanity[0][4], sanity[0][5]);

    // Try to update robot state before computing IK
    RCLCPP_INFO(get_logger(), "Attempting to update robot state for test...");
    bool robot_updated = false;
    for (int i = 0; i < 10; ++i) {
      RCLCPP_INFO(get_logger(), "Update attempt %d/10...", i + 1);

      // Run update() with timeout to avoid blocking
      std::atomic<bool> update_result{false};
      std::atomic<bool> update_complete{false};
      std::thread update_thread([&]() {
        update_result = controller_->update();
        update_complete = true;
      });

      // Wait up to 100ms for update to complete
      auto start = std::chrono::steady_clock::now();
      while (!update_complete && (std::chrono::steady_clock::now() - start) <
                                     std::chrono::milliseconds(100)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (update_complete) {
        update_thread.join();
        if (update_result) {
          robot_updated = true;
          RCLCPP_INFO(get_logger(),
                      "Robot state updated successfully (attempt %d)", i + 1);
          break;
        }
      } else {
        update_thread.detach();
      }

      if (i < 9) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    if (robot_updated) {
      const auto& inputs = controller_->getInputs();
      const auto& cart_fb = inputs.feedback().robot().cartesian().pose();
      const auto& joint_fb = inputs.feedback().robot().joints().position();

      RCLCPP_INFO(get_logger(), "=== Current Robot Position ===");
      RCLCPP_INFO(get_logger(),
                  "Cartesian: x=%.1f mm, y=%.1f mm, z=%.1f mm, rx=%.1f deg, "
                  "ry=%.1f deg, rz=%.1f deg",
                  cart_fb.position().x(), cart_fb.position().y(),
                  cart_fb.position().z(), cart_fb.euler().x(),
                  cart_fb.euler().y(), cart_fb.euler().z());

      RCLCPP_INFO(get_logger(), "Joints:");
      for (int i = 0; i < joint_fb.values_size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Joint %d: %.2f deg", i + 1,
                    joint_fb.values(i));
      }
      RCLCPP_INFO(get_logger(), "==============================");

      // Run FK vs EGM test
      RCLCPP_INFO(get_logger(), "Running FK vs EGM comparison test...");
      controller_->testFKvsEGM();
    } else {
      RCLCPP_WARN(get_logger(), "Cannot run FK test - robot state not updated");
    }

    RCLCPP_INFO(get_logger(), "Computing IK for test point...");
    auto start_time = std::chrono::steady_clock::now();
    auto test_result = controller_->trajectoryToJoints(sanity);
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    RCLCPP_INFO(get_logger(), "IK computation completed in %ld ms",
                duration.count());

    if (!test_result.empty() && !test_result[0].empty()) {
      RCLCPP_INFO(get_logger(), "IK test SUCCESS! Joint angles:");
      for (size_t i = 0; i < test_result[0].size(); ++i) {
        RCLCPP_INFO(get_logger(), "  Joint %zu: %.2f deg", i + 1,
                    test_result[0][i]);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "IK test FAILED - no solution found");
    }
    return;
  } else {
    RCLCPP_WARN(get_logger(), "Wrong trajectory type");
    return;
  }

  if (trajectory_.empty()) {
    RCLCPP_WARN(get_logger(), "Empty trajectory");
    return;
  }

  RCLCPP_INFO(get_logger(), "Preparing trajectory execution...");
  line_strip_.points.clear();

  index_ = 0;
  target_ = trajectory_[0];

  RCLCPP_INFO(get_logger(), "Starting timer for trajectory execution...");
  timer_->reset();
  RCLCPP_INFO(get_logger(), "Trajectory execution started!");
}

void EgmNode::update() {
  static int update_call_count = 0;
  static int update_fail_count = 0;

  update_call_count++;
  if (update_call_count % 250 == 0) {
    RCLCPP_DEBUG(get_logger(), "Update called %d times (failures: %d)",
                 update_call_count, update_fail_count);
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

  abb::egm::wrapper::Output outputs;
  bool reached = false;

  outputs.mutable_robot()->mutable_joints()->mutable_position();

  if (mode_ == ControlMode::CARTESIAN) {
    reached = controller_->stepToward(target_, outputs);
  } else {
    reached = controller_->stepTowardJoints(joint_traj_[index_], outputs,
                                            config_.dq_limit,
                                            config_.joint_tolerance);
  }

  publishMarkers();

  if (reached) {
    ++index_;

    if (index_ >= trajectory_.size()) {
      RCLCPP_INFO(get_logger(), "Trajectory finished");
      timer_->cancel();
      return;
    }

    target_ = trajectory_[index_];
  }

  controller_->write(outputs);
}

void EgmNode::publishMarkers() {
  const auto& inputs = controller_->getInputs();
  const auto& fb = inputs.feedback().robot().cartesian().pose();

  // publishSphere(target_, 0, 1.f, 0.f, 0.f);

  // publishSphere({fb.position().x(), fb.position().y(), fb.position().z()}, 1,
  // 0.f, 1.f, 0.f);

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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgmNode>());
  rclcpp::shutdown();
  return 0;
}
