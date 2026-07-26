#ifndef SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_PANEL_HPP_
#define SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_PANEL_HPP_

#include <memory>

#include <QLabel>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace shbat_rviz_plugins
{

class WaypointEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointEditorPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void loadWaypoints();
  void saveWaypoints();
  void undo();
  void redo();
  void clearAll();

private:
  using Trigger = std_srvs::srv::Trigger;
  void callService(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const QString & unavailable_message);
  void setStatus(const QString & text);

  QPushButton * load_button_;
  QPushButton * save_button_;
  QPushButton * undo_button_;
  QPushButton * redo_button_;
  QPushButton * clear_button_;
  QLabel * status_label_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr load_client_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr undo_client_;
  rclcpp::Client<Trigger>::SharedPtr redo_client_;
  rclcpp::Client<Trigger>::SharedPtr clear_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

}  // namespace shbat_rviz_plugins

#endif  // SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_PANEL_HPP_
