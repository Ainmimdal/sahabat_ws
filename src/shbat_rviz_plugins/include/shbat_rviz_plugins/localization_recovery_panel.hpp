#ifndef SHBAT_RVIZ_PLUGINS__LOCALIZATION_RECOVERY_PANEL_HPP_
#define SHBAT_RVIZ_PLUGINS__LOCALIZATION_RECOVERY_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace shbat_rviz_plugins
{

class LocalizationRecoveryPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LocalizationRecoveryPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void startRecovery();
  void stopRecovery();
  void updateAvailability();

private:
  using Trigger = std_srvs::srv::Trigger;
  void setStatus(const QString & text);
  void setActive(bool active);

  QPushButton * start_button_;
  QPushButton * stop_button_;
  QLabel * status_label_;
  QTimer * availability_timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

}  // namespace shbat_rviz_plugins

#endif  // SHBAT_RVIZ_PLUGINS__LOCALIZATION_RECOVERY_PANEL_HPP_
