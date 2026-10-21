#include "abb_work_cell_panel/work_cell_panel.hpp"

#include <memory>
#include <string>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace abb_work_cell_panel {

namespace {

const char * const kNamespace = "/work_cell/";

}  // namespace

WorkCellPanel::WorkCellPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout();

  auto * tool_group = new QGroupBox(QString::fromUtf8("Инструмент"));
  auto * tool_layout = new QVBoxLayout();
  auto * take_layout = new QHBoxLayout();
  take_layout->addWidget(addButton(QString::fromUtf8("Взять фрезу"), "take_mill"));
  take_layout->addWidget(addButton(QString::fromUtf8("Взять лазер"), "take_laser"));
  tool_layout->addLayout(take_layout);
  tool_layout->addWidget(addButton(QString::fromUtf8("Снять инструмент"), "release_tool"));
  tool_group->setLayout(tool_layout);
  root_layout->addWidget(tool_group);

  auto * work_group = new QGroupBox(QString::fromUtf8("Работа"));
  auto * work_layout = new QVBoxLayout();
  auto * start_stop_layout = new QHBoxLayout();
  start_stop_layout->addWidget(addButton(QString::fromUtf8("Начать работу"), "start_work"));
  start_stop_layout->addWidget(addButton(QString::fromUtf8("Прекратить работу"), "stop_work"));
  work_layout->addLayout(start_stop_layout);
  work_layout->addWidget(addButton(QString::fromUtf8("Домой"), "go_home"));
  work_group->setLayout(work_layout);
  root_layout->addWidget(work_group);

  status_label_ = new QLabel(QString::fromUtf8("Нет связи с work_cell"));
  reply_label_ = new QLabel();
  reply_label_->setWordWrap(true);
  root_layout->addWidget(status_label_);
  root_layout->addWidget(reply_label_);
  root_layout->addStretch();

  setLayout(root_layout);
}

void WorkCellPanel::onInitialize()
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    showReply(QString::fromUtf8("Нет ROS-ноды RViz"));
    return;
  }
  raw_node_ = ros_node_abstraction->get_raw_node();

  for (auto & [service, client] : clients_) {
    client = raw_node_->create_client<std_srvs::srv::Trigger>(kNamespace + service);
  }

  // The status is latched, so the panel gets it even when started later.
  rclcpp::QoS qos(1);
  qos.transient_local();
  status_sub_ = raw_node_->create_subscription<std_msgs::msg::String>(
    std::string(kNamespace) + "status", qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      showStatus(QString::fromStdString(msg->data));
    });
}

QPushButton * WorkCellPanel::addButton(const QString & text, const std::string & service)
{
  auto * button = new QPushButton(text);
  clients_[service] = nullptr;
  connect(button, &QPushButton::clicked, this, [this, service]() { call(service); });
  return button;
}

void WorkCellPanel::call(const std::string & service)
{
  const auto & client = clients_.at(service);
  if (!client || !client->service_is_ready()) {
    showReply(QString::fromUtf8("Сервис недоступен: ") + QString::fromStdString(kNamespace + service));
    return;
  }

  client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      showReply(QString::fromStdString(response->message));
    });
}

// ROS callbacks may come from another thread, Qt widgets must be touched in the GUI one.
void WorkCellPanel::showReply(const QString & text)
{
  QMetaObject::invokeMethod(reply_label_, [this, text]() { reply_label_->setText(text); },
    Qt::QueuedConnection);
}

void WorkCellPanel::showStatus(const QString & text)
{
  QMetaObject::invokeMethod(status_label_, [this, text]() { status_label_->setText(text); },
    Qt::QueuedConnection);
}

}  // namespace abb_work_cell_panel

PLUGINLIB_EXPORT_CLASS(abb_work_cell_panel::WorkCellPanel, rviz_common::Panel)
