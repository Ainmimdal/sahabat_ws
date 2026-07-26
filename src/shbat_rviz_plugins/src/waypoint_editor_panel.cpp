#include "shbat_rviz_plugins/waypoint_editor_panel.hpp"

#include <chrono>

#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace shbat_rviz_plugins
{

WaypointEditorPanel::WaypointEditorPanel(QWidget * parent)
: rviz_common::Panel(parent),
  load_button_(new QPushButton("Load")),
  save_button_(new QPushButton("Save")),
  undo_button_(new QPushButton("Undo")),
  redo_button_(new QPushButton("Redo")),
  clear_button_(new QPushButton("Clear")),
  status_label_(new QLabel("Select 'Sahabat Add Waypoint', then click-drag on the map."))
{
  status_label_->setWordWrap(true);
  auto * buttons = new QHBoxLayout;
  buttons->addWidget(load_button_);
  buttons->addWidget(save_button_);
  buttons->addWidget(undo_button_);
  buttons->addWidget(redo_button_);
  buttons->addWidget(clear_button_);

  auto * layout = new QVBoxLayout;
  layout->addLayout(buttons);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(load_button_, &QPushButton::clicked, this, &WaypointEditorPanel::loadWaypoints);
  connect(save_button_, &QPushButton::clicked, this, &WaypointEditorPanel::saveWaypoints);
  connect(undo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::undo);
  connect(redo_button_, &QPushButton::clicked, this, &WaypointEditorPanel::redo);
  connect(clear_button_, &QPushButton::clicked, this, &WaypointEditorPanel::clearAll);
}

void WaypointEditorPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  load_client_ = node_->create_client<Trigger>("/sahabat_waypoint_editor/load");
  save_client_ = node_->create_client<Trigger>("/sahabat_waypoint_editor/save");
  undo_client_ = node_->create_client<Trigger>("/sahabat_waypoint_editor/undo");
  redo_client_ = node_->create_client<Trigger>("/sahabat_waypoint_editor/redo");
  clear_client_ = node_->create_client<Trigger>("/sahabat_waypoint_editor/clear");
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/sahabat_waypoint_editor/status", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      const QString text = QString::fromStdString(message->data);
      QMetaObject::invokeMethod(this, [this, text]() {setStatus(text);}, Qt::QueuedConnection);
    });
}

void WaypointEditorPanel::loadWaypoints()
{
  callService(load_client_, "Waypoint editor tool is not loaded yet");
}

void WaypointEditorPanel::saveWaypoints()
{
  callService(save_client_, "Waypoint editor tool is not loaded yet");
}

void WaypointEditorPanel::undo()
{
  callService(undo_client_, "Waypoint editor tool is not loaded yet");
}

void WaypointEditorPanel::redo()
{
  callService(redo_client_, "Waypoint editor tool is not loaded yet");
}

void WaypointEditorPanel::clearAll()
{
  callService(clear_client_, "Waypoint editor tool is not loaded yet");
}

void WaypointEditorPanel::callService(
  const rclcpp::Client<Trigger>::SharedPtr & client,
  const QString & unavailable_message)
{
  if (!client || !client->wait_for_service(std::chrono::milliseconds(300))) {
    setStatus(unavailable_message);
    return;
  }
  client->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      auto response = future.get();
      const QString text = QString::fromStdString(response->message);
      QMetaObject::invokeMethod(this, [this, text]() {setStatus(text);}, Qt::QueuedConnection);
    });
}

void WaypointEditorPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

}  // namespace shbat_rviz_plugins

PLUGINLIB_EXPORT_CLASS(shbat_rviz_plugins::WaypointEditorPanel, rviz_common::Panel)
