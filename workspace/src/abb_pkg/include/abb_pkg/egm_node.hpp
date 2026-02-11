#ifndef ABB_PKG_EGM_NODE_HPP
#define ABB_PKG_EGM_NODE_HPP


#include <abb_libegm/egm_wrapper.pb.h>

#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

#include "abb_pkg/controller.hpp"


class EgmNode : public rclcpp::Node {
    std::string marker_frame_{"tool"};

    std::unique_ptr<abb_pkg::Controller> controller_;
    abb_pkg::Controller::Config config_;  // Store config for accessing parameters

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr type_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    visualization_msgs::msg::Marker line_strip_;

    std::vector<std::vector<double>> trajectory_;
    std::vector<double> target_;
    size_t index_{0};

    enum class ControlMode {
        CARTESIAN,
        JOINTS
    };

    ControlMode mode_{ControlMode::CARTESIAN};

    std::vector<std::vector<double>> joint_traj_;

public:
    EgmNode();

private:
    void update();
    void publishMarkers();
    void publishSphere(const std::vector<double>& pos, int id, float r, float g, float b);
    void initLineStrip();

    void typeCallback(const std_msgs::msg::String::SharedPtr msg);
};


#endif  // ABB_PKG_EGM_NODE_HPP
