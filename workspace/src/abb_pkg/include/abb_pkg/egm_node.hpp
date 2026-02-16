#ifndef ABB_PKG_EGM_NODE_HPP
#define ABB_PKG_EGM_NODE_HPP


#include <abb_libegm/egm_wrapper.pb.h>

#include <atomic>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "abb_pkg/controller.hpp"
#include "abb_pkg/action/move_to_pose.hpp"


class EgmNode : public rclcpp::Node {
    std::string marker_frame_{"tool"};

    std::unique_ptr<abb_pkg::Controller> controller_;
    abb_pkg::Controller::Config config_;

    // Action server
    rclcpp_action::Server<abb_pkg::action::MoveToPose>::SharedPtr action_server_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ping_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr state_pub_timer_;
    rclcpp::TimerBase::SharedPtr ping_timer_;

    visualization_msgs::msg::Marker line_strip_;

    // Action execution state (thread-safe)
    std::mutex target_pose_mutex_;
    std::vector<double> target_pose_;  // [x, y, z, rx, ry, rz] in mm and degrees
    std::atomic<bool> executing_action_{false};
    std::atomic<bool> cancel_requested_{false};
    std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> current_goal_handle_;

    // If controller status is temporarily not OK, don't fail immediately.
    std::optional<rclcpp::Time> status_not_ok_since_;

public:
    EgmNode();

private:
    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const abb_pkg::action::MoveToPose::Goal> goal);
    
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> goal_handle);
    
    void executeAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> goal_handle);

    // Main update loop
    void update();

    // State publishing
    void publishState();

    // Utility functions
    void publishMarkers();
    void publishSphere(const std::vector<double>& pos, int id, float r, float g, float b);
    void initLineStrip();
    void ping();

    // Convert quaternion to Euler angles (in degrees)
    std::vector<double> quaternionToEuler(const geometry_msgs::msg::Quaternion& quat);
    
    // Convert position from meters to millimeters
    std::vector<double> poseToTarget(const geometry_msgs::msg::Point& position,
                                     const geometry_msgs::msg::Quaternion& orientation);
};


#endif  // ABB_PKG_EGM_NODE_HPP
