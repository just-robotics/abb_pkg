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
#include <std_msgs/msg/bool.hpp>

namespace rrt_rviz_panel {

class StartGoalPanel : public rviz_common::Panel
{
public:
  explicit StartGoalPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  void onGeneratePath();
  void onSendPath();
  void onMapLock();
  void onMapUnlock();
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
  QLineEdit * goal_topic_edit_;
  QLineEdit * map_lock_topic_edit_;
  QLineEdit * send_path_topic_edit_;

  QLineEdit * goal_x_edit_;
  QLineEdit * goal_y_edit_;
  QLineEdit * goal_z_edit_;

  QPushButton * generate_path_btn_;
  QPushButton * send_path_btn_;
  QPushButton * lock_map_btn_;
  QPushButton * unlock_map_btn_;
  QLabel * status_label_;

  std::string current_goal_topic_;
  std::string current_map_lock_topic_;
  std::string current_send_path_topic_;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr map_lock_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr send_path_pub_;
};

}  // namespace rrt_rviz_panel

#endif  // RRT_RVIZ_PANEL__START_GOAL_PANEL_HPP_
