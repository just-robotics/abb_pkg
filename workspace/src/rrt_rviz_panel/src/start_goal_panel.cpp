#include "rrt_rviz_panel/start_goal_panel.hpp"

#include <string>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rrt_rviz_panel {

StartGoalPanel::StartGoalPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout();

  auto * cfg_group = new QGroupBox("Topics and Frame");
  auto * cfg_form = new QFormLayout();

  frame_id_edit_ = new QLineEdit("world");
  goal_topic_edit_ = new QLineEdit("/goal_point");
  map_lock_topic_edit_ = new QLineEdit("/lock_map");
  send_path_topic_edit_ = new QLineEdit("/send_path");

  cfg_form->addRow("Frame ID", frame_id_edit_);
  cfg_form->addRow("Goal Topic", goal_topic_edit_);
  cfg_form->addRow("Map Lock Topic", map_lock_topic_edit_);
  cfg_form->addRow("Send Path Topic", send_path_topic_edit_);
  cfg_group->setLayout(cfg_form);
  root_layout->addWidget(cfg_group);

  auto * goal_group = new QGroupBox("Goal Point (x,y,z)");
  auto * goal_grid = new QGridLayout();
  goal_x_edit_ = new QLineEdit("2.5");
  goal_y_edit_ = new QLineEdit("0.5");
  goal_z_edit_ = new QLineEdit("0.8");
  goal_grid->addWidget(new QLabel("x"), 0, 0);
  goal_grid->addWidget(goal_x_edit_, 0, 1);
  goal_grid->addWidget(new QLabel("y"), 0, 2);
  goal_grid->addWidget(goal_y_edit_, 0, 3);
  goal_grid->addWidget(new QLabel("z"), 0, 4);
  goal_grid->addWidget(goal_z_edit_, 0, 5);
  goal_group->setLayout(goal_grid);
  root_layout->addWidget(goal_group);

  generate_path_btn_ = new QPushButton("Generate Path");
  root_layout->addWidget(generate_path_btn_);

  send_path_btn_ = new QPushButton("Send Path");
  root_layout->addWidget(send_path_btn_);

  auto * map_lock_btn_layout = new QHBoxLayout();
  lock_map_btn_ = new QPushButton("Lock Map");
  unlock_map_btn_ = new QPushButton("Unlock Map");
  map_lock_btn_layout->addWidget(lock_map_btn_);
  map_lock_btn_layout->addWidget(unlock_map_btn_);
  root_layout->addLayout(map_lock_btn_layout);

  status_label_ = new QLabel("Status: ready");
  root_layout->addWidget(status_label_);

  setLayout(root_layout);

  connect(generate_path_btn_, &QPushButton::clicked, this, &StartGoalPanel::onGeneratePath);
  connect(send_path_btn_, &QPushButton::clicked, this, &StartGoalPanel::onSendPath);
  connect(lock_map_btn_, &QPushButton::clicked, this, &StartGoalPanel::onMapLock);
  connect(unlock_map_btn_, &QPushButton::clicked, this, &StartGoalPanel::onMapUnlock);
  connect(goal_topic_edit_, &QLineEdit::editingFinished, this, &StartGoalPanel::onTopicsEdited);
  connect(map_lock_topic_edit_, &QLineEdit::editingFinished, this, &StartGoalPanel::onTopicsEdited);
  connect(send_path_topic_edit_, &QLineEdit::editingFinished, this, &StartGoalPanel::onTopicsEdited);
}

void StartGoalPanel::onInitialize()
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    status_label_->setText("Status: ROS node abstraction unavailable");
    return;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  if (!raw_node_) {
    status_label_->setText("Status: RViz ROS node unavailable");
    return;
  }

  updatePublishers();
  status_label_->setText("Status: initialized");
}

void StartGoalPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  config.mapSetValue("frame_id", frame_id_edit_->text());
  config.mapSetValue("goal_topic", goal_topic_edit_->text());
  config.mapSetValue("map_lock_topic", map_lock_topic_edit_->text());
  config.mapSetValue("send_path_topic", send_path_topic_edit_->text());

  config.mapSetValue("goal_x", goal_x_edit_->text());
  config.mapSetValue("goal_y", goal_y_edit_->text());
  config.mapSetValue("goal_z", goal_z_edit_->text());
}

void StartGoalPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString value;
  if (config.mapGetString("frame_id", &value)) frame_id_edit_->setText(value);
  if (config.mapGetString("goal_topic", &value)) goal_topic_edit_->setText(value);
  if (config.mapGetString("map_lock_topic", &value)) map_lock_topic_edit_->setText(value);
  if (config.mapGetString("send_path_topic", &value)) send_path_topic_edit_->setText(value);

  if (config.mapGetString("goal_x", &value)) goal_x_edit_->setText(value);
  if (config.mapGetString("goal_y", &value)) goal_y_edit_->setText(value);
  if (config.mapGetString("goal_z", &value)) goal_z_edit_->setText(value);

  updatePublishers();
}

bool StartGoalPanel::parsePoint(
  QLineEdit * x_edit,
  QLineEdit * y_edit,
  QLineEdit * z_edit,
  geometry_msgs::msg::PointStamped & out_msg,
  std::string & err) const
{
  bool ok_x = false;
  bool ok_y = false;
  bool ok_z = false;

  const double x = x_edit->text().toDouble(&ok_x);
  const double y = y_edit->text().toDouble(&ok_y);
  const double z = z_edit->text().toDouble(&ok_z);

  if (!ok_x || !ok_y || !ok_z) {
    err = "invalid x/y/z";
    return false;
  }

  out_msg.header.frame_id = frame_id_edit_->text().toStdString();
  out_msg.header.stamp = raw_node_->now();
  out_msg.point.x = x;
  out_msg.point.y = y;
  out_msg.point.z = z;
  return true;
}

void StartGoalPanel::updatePublishers()
{
  if (!raw_node_) {
    return;
  }

  const std::string goal_topic = goal_topic_edit_->text().toStdString();
  const std::string map_lock_topic = map_lock_topic_edit_->text().toStdString();
  const std::string send_path_topic = send_path_topic_edit_->text().toStdString();

  if (goal_topic != current_goal_topic_) {
    goal_pub_ = raw_node_->create_publisher<geometry_msgs::msg::PointStamped>(goal_topic, 10);
    current_goal_topic_ = goal_topic;
  }

  if (map_lock_topic != current_map_lock_topic_) {
    map_lock_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(map_lock_topic, 10);
    current_map_lock_topic_ = map_lock_topic;
  }

  if (send_path_topic != current_send_path_topic_) {
    send_path_pub_ = raw_node_->create_publisher<std_msgs::msg::Bool>(send_path_topic, 10);
    current_send_path_topic_ = send_path_topic;
  }
}

void StartGoalPanel::onTopicsEdited()
{
  updatePublishers();
  status_label_->setText("Status: topics updated");
}

void StartGoalPanel::onGeneratePath()
{
  updatePublishers();

  if (!goal_pub_) {
    status_label_->setText("Status: goal publisher not ready");
    return;
  }

  geometry_msgs::msg::PointStamped goal_msg;
  std::string err;
  if (!parsePoint(goal_x_edit_, goal_y_edit_, goal_z_edit_, goal_msg, err)) {
    status_label_->setText(QString::fromStdString("Status: goal not sent, " + err));
    return;
  }

  goal_pub_->publish(goal_msg);
  status_label_->setText("Status: generate_path sent (goal published)");
}

void StartGoalPanel::onSendPath()
{
  updatePublishers();

  if (!send_path_pub_) {
    status_label_->setText("Status: send_path publisher not ready");
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  send_path_pub_->publish(msg);
  status_label_->setText("Status: send_path trigger sent");
}

void StartGoalPanel::onMapLock()
{
  updatePublishers();

  if (!map_lock_pub_) {
    status_label_->setText("Status: map lock publisher not ready");
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  map_lock_pub_->publish(msg);
  status_label_->setText("Status: map lock sent");
}

void StartGoalPanel::onMapUnlock()
{
  updatePublishers();

  if (!map_lock_pub_) {
    status_label_->setText("Status: map lock publisher not ready");
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = false;
  map_lock_pub_->publish(msg);
  status_label_->setText("Status: map unlock sent");
}

}  // namespace rrt_rviz_panel

PLUGINLIB_EXPORT_CLASS(rrt_rviz_panel::StartGoalPanel, rviz_common::Panel)
