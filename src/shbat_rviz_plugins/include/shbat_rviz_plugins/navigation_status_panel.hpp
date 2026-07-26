#ifndef SHBAT_RVIZ_PLUGINS__NAVIGATION_STATUS_PANEL_HPP_
#define SHBAT_RVIZ_PLUGINS__NAVIGATION_STATUS_PANEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <QLabel>
#include <QPushButton>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sahabat_interfaces/msg/operator_status.hpp"

namespace shbat_rviz_plugins
{

class NavigationStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavigationStatusPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void refreshParameters();

private:
  using GoalStatusArray = action_msgs::msg::GoalStatusArray;
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using OperatorStatus = sahabat_interfaces::msg::OperatorStatus;

  QLabel * addRow(const QString & name, QWidget * parent, int row);
  void setLabel(QLabel * label, const QString & text);
  void setParamLine(const QString & node_name, const QString & text);
  std::string statusText(int8_t status) const;
  std::string parameterValueText(
    const rcl_interfaces::msg::ParameterValue & value) const;
  double yawFromQuaternion(
    const geometry_msgs::msg::Quaternion & orientation) const;

  QLabel * mode_label_;
  QLabel * health_label_;
  QLabel * pose_label_;
  QLabel * velocity_label_;
  QLabel * target_label_;
  QLabel * nav_state_label_;
  QLabel * feedback_label_;
  QLabel * recovery_label_;
  QLabel * params_label_;
  QLabel * param_status_label_;
  QPushButton * refresh_params_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<OperatorStatus>::SharedPtr operator_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<NavigateToPose::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;
  std::vector<rclcpp::Client<GetParameters>::SharedPtr> param_clients_;
  QStringList param_lines_;
};

}  // namespace shbat_rviz_plugins

#endif  // SHBAT_RVIZ_PLUGINS__NAVIGATION_STATUS_PANEL_HPP_
