#ifndef ABB_WORK_CELL_PANEL__WORK_CELL_PANEL_HPP_
#define ABB_WORK_CELL_PANEL__WORK_CELL_PANEL_HPP_

#include <map>
#include <string>

#include <QLabel>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace abb_work_cell_panel {

// Calls the Trigger services of the work_cell node and shows its status.
class WorkCellPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WorkCellPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private:
  QPushButton * addButton(const QString & text, const std::string & service);
  void call(const std::string & service);
  void showReply(const QString & text);
  void showStatus(const QString & text);

  QLabel * status_label_;
  QLabel * reply_label_;

  rclcpp::Node::SharedPtr raw_node_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

}  // namespace abb_work_cell_panel

#endif  // ABB_WORK_CELL_PANEL__WORK_CELL_PANEL_HPP_
