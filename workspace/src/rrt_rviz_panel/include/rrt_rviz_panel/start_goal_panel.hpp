#ifndef RRT_RVIZ_PANEL__START_GOAL_PANEL_HPP_
#define RRT_RVIZ_PANEL__START_GOAL_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace rrt_rviz_panel {

class StartGoalPanel : public rviz_common::Panel
{
public:
  explicit StartGoalPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  void onSendBoth();
  void onTopicsEdited();

private:
  bool parsePoint(
    QLineEdit * x_edit,
    QLineEdit * y_edit,
    QLineEdit * z_edit,
    geometry_msgs::msg::PointStamped & out_msg,
    std::string & err) const;

  void updatePublishers();

  QLineEdit * frame_id_edit_;
  QLineEdit * start_topic_edit_;
  QLineEdit * goal_topic_edit_;

  QLineEdit * start_x_edit_;
  QLineEdit * start_y_edit_;
  QLineEdit * start_z_edit_;

  QLineEdit * goal_x_edit_;
  QLineEdit * goal_y_edit_;
  QLineEdit * goal_z_edit_;

  QPushButton * send_both_btn_;
  QLabel * status_label_;

  std::string current_start_topic_;
  std::string current_goal_topic_;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
};

}  // namespace rrt_rviz_panel

#endif  // RRT_RVIZ_PANEL__START_GOAL_PANEL_HPP_
