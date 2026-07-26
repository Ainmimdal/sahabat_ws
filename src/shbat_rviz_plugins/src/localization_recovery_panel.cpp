#include "shbat_rviz_plugins/localization_recovery_panel.hpp"

#include <memory>

#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace shbat_rviz_plugins
{

LocalizationRecoveryPanel::LocalizationRecoveryPanel(QWidget * parent)
: rviz_common::Panel(parent),
  start_button_(new QPushButton("Global Relocalize + Rotate")),
  stop_button_(new QPushButton("Stop Rotation")),
  status_label_(new QLabel("Waiting for localization recovery node...")),
  availability_timer_(new QTimer(this))
{
  status_label_->setWordWrap(true);
  start_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  auto * layout = new QVBoxLayout;
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);
  connect(start_button_, &QPushButton::clicked, this,
    &LocalizationRecoveryPanel::startRecovery);
  connect(stop_button_, &QPushButton::clicked, this,
    &LocalizationRecoveryPanel::stopRecovery);
  connect(availability_timer_, &QTimer::timeout, this,
    &LocalizationRecoveryPanel::updateAvailability);
}

void LocalizationRecoveryPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node is unavailable.");
    start_button_->setEnabled(false);
    return;
  }
  node_ = abstraction->get_raw_node();
  start_client_ = node_->create_client<Trigger>("/localization/start_recovery");
  stop_client_ = node_->create_client<Trigger>("/localization/stop_recovery");
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  active_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/localization/recovery_active", state_qos,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      QMetaObject::invokeMethod(
        this, [this, active = msg->data]() {setActive(active);},
        Qt::QueuedConnection);
    });
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/localization/recovery_status", state_qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      const QString status = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(
        this, [this, status]() {setStatus(status);}, Qt::QueuedConnection);
    });
  availability_timer_->start(1000);
  updateAvailability();
}

void LocalizationRecoveryPanel::updateAvailability()
{
  if (!start_client_ || !start_client_->service_is_ready()) {
    start_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    setStatus(
      "AMCL recovery service unavailable. If this is the SLAM Toolbox "
      "localization test, use RViz 2D Pose Estimate instead."
    );
    return;
  }
  if (!stop_button_->isEnabled()) {
    start_button_->setEnabled(true);
  }
  if (status_label_->text().contains("unavailable")) {
    setStatus("AMCL recovery ready. Ensure the robot has room to rotate.");
  }
}

void LocalizationRecoveryPanel::startRecovery()
{
  if (!start_client_ || !start_client_->service_is_ready()) {
    setStatus("Recovery service unavailable. Start operations.launch.py.");
    return;
  }
  start_button_->setEnabled(false);
  setStatus("Requesting global relocalization...");
  start_client_->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString message = QString::fromStdString(response->message);
      const bool success = response->success;
      QMetaObject::invokeMethod(this, [this, success, message]() {
        setStatus(message);
        if (!success) {
          setActive(false);
        }
      }, Qt::QueuedConnection);
    });
}

void LocalizationRecoveryPanel::stopRecovery()
{
  if (!stop_client_ || !stop_client_->service_is_ready()) {
    setStatus("Recovery stop service unavailable.");
    return;
  }
  stop_client_->async_send_request(std::make_shared<Trigger::Request>());
}

void LocalizationRecoveryPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

void LocalizationRecoveryPanel::setActive(bool active)
{
  start_button_->setEnabled(!active);
  stop_button_->setEnabled(active);
}

}  // namespace shbat_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  shbat_rviz_plugins::LocalizationRecoveryPanel,
  rviz_common::Panel)
