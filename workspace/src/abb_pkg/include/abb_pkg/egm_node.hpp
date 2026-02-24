#ifndef ABB_PKG_EGM_NODE_HPP
#define ABB_PKG_EGM_NODE_HPP


#include <abb_libegm/egm_wrapper.pb.h>

#include <array>
#include <atomic>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
    rclcpp_action::Server<abb_pkg::action::MoveToPose>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ping_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr state_pub_timer_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    visualization_msgs::msg::Marker line_strip_;
    std::mutex target_pose_mutex_;
    std::vector<double> target_pose_;
    std::atomic<bool> executing_action_{false};
    std::atomic<bool> cancel_requested_{false};
    std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> current_goal_handle_;
    std::optional<rclcpp::Time> status_not_ok_since_;
    std::mutex cmd_vel_mutex_;
    geometry_msgs::msg::Twist last_cmd_vel_;
    std::optional<rclcpp::Time> last_cmd_vel_time_;
    std::array<double, 6> last_feedback_pose_{};
    double cmd_vel_lin_gain_{1.0};
    double cmd_vel_ang_gain_{1.0};
    double cmd_vel_timeout_s_{0.15};

public:
    EgmNode();

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const abb_pkg::action::MoveToPose::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> goal_handle);
    void executeAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<abb_pkg::action::MoveToPose>> goal_handle);

    void update();
    void publishState();
    void publishMarkers();
    void initLineStrip();
    void ping();
    std::vector<double> quaternionToEuler(const geometry_msgs::msg::Quaternion& quat);
    std::vector<double> poseToTarget(const geometry_msgs::msg::Point& position,
                                     const geometry_msgs::msg::Quaternion& orientation);
    void fillVelocityOutput(double dx, double dy, double dz,
                            double drx, double dry, double drz,
                            double lin_gain, double ang_gain,
                            abb::egm::wrapper::Output& outputs);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};


#endif  // ABB_PKG_EGM_NODE_HPP
