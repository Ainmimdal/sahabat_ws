#include "shbat_rviz_plugins/navigation_status_panel.hpp"

#include <cmath>
#include <sstream>
#include <utility>

#include <QGridLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QStringList>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rviz_common/display_context.hpp"

namespace shbat_rviz_plugins
{

NavigationStatusPanel::NavigationStatusPanel(QWidget * parent)
: rviz_common::Panel(parent),
  mode_label_(nullptr),
  health_label_(nullptr),
  pose_label_(nullptr),
  velocity_label_(nullptr),
  target_label_(nullptr),
  nav_state_label_(nullptr),
  feedback_label_(nullptr),
  recovery_label_(nullptr),
  params_label_(nullptr),
  param_status_label_(nullptr),
  refresh_params_button_(new QPushButton("Refresh Nav2 Parameters"))
{
  auto * root_layout = new QVBoxLayout;

  auto * status_box = new QGroupBox("Navigation Status");
  auto * status_layout = new QGridLayout;
  status_box->setLayout(status_layout);
  mode_label_ = addRow("Mode", status_box, 0);
  health_label_ = addRow("Health", status_box, 1);
  pose_label_ = addRow("Current pose", status_box, 2);
  velocity_label_ = addRow("Velocity", status_box, 3);
  target_label_ = addRow("Target", status_box, 4);
  nav_state_label_ = addRow("Nav2 state", status_box, 5);
  feedback_label_ = addRow("Feedback", status_box, 6);
  recovery_label_ = addRow("Recovery", status_box, 7);

  auto * params_box = new QGroupBox("Runtime Nav2 Parameters");
  auto * params_layout = new QVBoxLayout;
  params_label_ = new QLabel("Press refresh after Nav2 is active.");
  params_label_->setWordWrap(true);
  param_status_label_ = new QLabel("Not queried yet.");
  param_status_label_->setWordWrap(true);
  params_layout->addWidget(refresh_params_button_);
  params_layout->addWidget(param_status_label_);
  params_layout->addWidget(params_label_);
  params_box->setLayout(params_layout);

  root_layout->addWidget(status_box);
  root_layout->addWidget(params_box);
  root_layout->addStretch();
  setLayout(root_layout);

  setLabel(mode_label_, "Waiting for /operator/status");
  setLabel(health_label_, "map ?, scan ?, TF ?, localization ?");
  setLabel(pose_label_, "Waiting for /amcl_pose");
  setLabel(velocity_label_, "Waiting for /odom");
  setLabel(target_label_, "Waiting for RViz/Nav2 goal");
  setLabel(nav_state_label_, "Waiting for NavigateToPose status");
  setLabel(feedback_label_, "Waiting for NavigateToPose feedback");
  setLabel(recovery_label_, "Not active");

  connect(refresh_params_button_, &QPushButton::clicked, this,
    &NavigationStatusPanel::refreshParameters);
}

QLabel * NavigationStatusPanel::addRow(
  const QString & name, QWidget * parent, int row)
{
  auto * layout = qobject_cast<QGridLayout *>(parent->layout());
  auto * name_label = new QLabel(name + ":");
  auto * value_label = new QLabel("-");
  value_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  value_label->setWordWrap(true);
  layout->addWidget(name_label, row, 0, Qt::AlignTop);
  layout->addWidget(value_label, row, 1);
  return value_label;
}

void NavigationStatusPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setLabel(nav_state_label_, "RViz ROS node is unavailable.");
    refresh_params_button_->setEnabled(false);
    return;
  }
  node_ = abstraction->get_raw_node();

  const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
  operator_status_sub_ = node_->create_subscription<OperatorStatus>(
    "/operator/status", transient_qos,
    [this](OperatorStatus::ConstSharedPtr msg) {
      const QString mode = QString("%1 map=%2 op=%3").arg(msg->mode).arg(
        QString::fromStdString(msg->active_map)).arg(
        QString::fromStdString(msg->active_operation));
      const QString health = QString("map %1, scan %2, TF %3, localization %4, E-stop %5: %6").arg(
        msg->map_healthy ? "OK" : "BAD").arg(
        msg->scan_healthy ? "OK" : "BAD").arg(
        msg->tf_healthy ? "OK" : "BAD").arg(
        msg->localization_healthy ? "OK" : "BAD").arg(
        msg->emergency_stop ? "ACTIVE" : "clear").arg(
        QString::fromStdString(msg->diagnostic_message));
      const QString pose = QString("x=%1 y=%2 yaw=%3 rad").arg(
        msg->pose.x, 0, 'f', 3).arg(msg->pose.y, 0, 'f', 3).arg(
        msg->pose.theta, 0, 'f', 3);
      const QString velocity = QString("linear=%1 m/s angular=%2 rad/s").arg(
        msg->linear_velocity, 0, 'f', 3).arg(msg->angular_velocity, 0, 'f', 3);
      const QString recovery = QString("%1: %2").arg(
        msg->localization_recovery_active ? "active" : "inactive").arg(
        QString::fromStdString(msg->localization_recovery_status));
      const QString state = QString::fromStdString(msg->navigation_state);
      QMetaObject::invokeMethod(this, [this, mode, health, pose, velocity, recovery, state]() {
        setLabel(mode_label_, mode);
        setLabel(health_label_, health);
        setLabel(pose_label_, pose);
        setLabel(velocity_label_, velocity);
        setLabel(recovery_label_, recovery);
        if (!state.isEmpty()) {
          setLabel(nav_state_label_, state);
        }
      }, Qt::QueuedConnection);
    });

  amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
      const auto & pose = msg->pose.pose;
      const double yaw = yawFromQuaternion(pose.orientation);
      const QString text = QString("x=%1 y=%2 yaw=%3 rad").arg(
        pose.position.x, 0, 'f', 3).arg(pose.position.y, 0, 'f', 3).arg(yaw, 0, 'f', 3);
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(pose_label_, text);}, Qt::QueuedConnection);
    });

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      const QString text = QString("linear=%1 m/s angular=%2 rad/s").arg(
        msg->twist.twist.linear.x, 0, 'f', 3).arg(
        msg->twist.twist.angular.z, 0, 'f', 3);
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(velocity_label_, text);}, Qt::QueuedConnection);
    });

  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/plan", 10,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
      if (msg->poses.empty()) {
        return;
      }
      const auto & pose = msg->poses.back().pose;
      const double yaw = yawFromQuaternion(pose.orientation);
      const QString text = QString("plan goal x=%1 y=%2 yaw=%3 rad").arg(
        pose.position.x, 0, 'f', 3).arg(
        pose.position.y, 0, 'f', 3).arg(yaw, 0, 'f', 3);
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(target_label_, text);}, Qt::QueuedConnection);
    });

  goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      const double yaw = yawFromQuaternion(msg->pose.orientation);
      const QString text = QString("RViz /goal_pose x=%1 y=%2 yaw=%3 rad").arg(
        msg->pose.position.x, 0, 'f', 3).arg(
        msg->pose.position.y, 0, 'f', 3).arg(yaw, 0, 'f', 3);
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(target_label_, text);}, Qt::QueuedConnection);
    });

  nav_status_sub_ = node_->create_subscription<GoalStatusArray>(
    "/navigate_to_pose/_action/status", 10,
    [this](GoalStatusArray::ConstSharedPtr msg) {
      QString text = "No active NavigateToPose goals";
      if (!msg->status_list.empty()) {
        const auto & status = msg->status_list.back();
        text = QString::fromStdString(statusText(status.status));
      }
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(nav_state_label_, text);}, Qt::QueuedConnection);
    });

  nav_feedback_sub_ = node_->create_subscription<NavigateToPose::Impl::FeedbackMessage>(
    "/navigate_to_pose/_action/feedback", 10,
    [this](NavigateToPose::Impl::FeedbackMessage::ConstSharedPtr msg) {
      const auto & feedback = msg->feedback;
      const double eta = static_cast<double>(feedback.estimated_time_remaining.sec) +
        static_cast<double>(feedback.estimated_time_remaining.nanosec) / 1e9;
      const QString text = QString("remaining=%1 m ETA=%2 s recoveries=%3").arg(
        feedback.distance_remaining, 0, 'f', 2).arg(eta, 0, 'f', 1).arg(
        feedback.number_of_recoveries);
      QMetaObject::invokeMethod(this, [this, text]() {setLabel(feedback_label_, text);}, Qt::QueuedConnection);
    });

  refreshParameters();
}

void NavigationStatusPanel::refreshParameters()
{
  if (!node_) {
    return;
  }

  param_clients_.clear();
  param_lines_.clear();
  setLabel(param_status_label_, "Querying Nav2 parameter services...");
  setLabel(params_label_, "");

  const std::vector<std::pair<std::string, std::vector<std::string>>> requests = {
    {"controller_server", {
      "FollowPath.desired_linear_vel",
      "FollowPath.max_linear_vel",
      "FollowPath.max_angular_vel",
      "FollowPath.lookahead_dist",
      "FollowPath.rotate_to_heading_min_angle",
      "general_goal_checker.xy_goal_tolerance",
      "general_goal_checker.yaw_goal_tolerance"}},
    {"velocity_smoother", {
      "smoothing_frequency", "max_velocity", "min_velocity", "max_accel", "max_decel"}},
    {"planner_server", {
      "GridBased.plugin", "GridBased.tolerance", "GridBased.use_astar", "GridBased.allow_unknown"}},
    {"global_costmap/global_costmap", {
      "inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor", "track_unknown_space"}},
    {"local_costmap/local_costmap", {
      "inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor", "width", "height"}},
  };

  for (const auto & request_info : requests) {
    const auto & node_name = request_info.first;
    auto client = node_->create_client<GetParameters>("/" + node_name + "/get_parameters");
    if (!client->service_is_ready()) {
      setParamLine(QString::fromStdString(node_name), "parameter service unavailable");
      param_clients_.push_back(client);
      continue;
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names = request_info.second;
    client->async_send_request(
      request,
      [this, node_name, names = request_info.second](rclcpp::Client<GetParameters>::SharedFuture future) {
        std::ostringstream line;
        try {
          const auto response = future.get();
          for (size_t i = 0; i < response->values.size() && i < names.size(); ++i) {
            if (i > 0) {
              line << ", ";
            }
            line << names[i] << "=" << parameterValueText(response->values[i]);
          }
        } catch (const std::exception & error) {
          line << "query failed: " << error.what();
        }
        const QString node = QString::fromStdString(node_name);
        const QString text = QString::fromStdString(line.str());
        QMetaObject::invokeMethod(this, [this, node, text]() {
          setParamLine(node, text);
        }, Qt::QueuedConnection);
      });
    param_clients_.push_back(client);
  }
}

void NavigationStatusPanel::setLabel(QLabel * label, const QString & text)
{
  if (label) {
    label->setText(text);
  }
}

void NavigationStatusPanel::setParamLine(
  const QString & node_name, const QString & text)
{
  const QString line = QString("%1: %2").arg(node_name, text);
  for (int i = 0; i < param_lines_.size(); ++i) {
    if (param_lines_[i].startsWith(node_name + ":")) {
      param_lines_[i] = line;
      setLabel(params_label_, param_lines_.join("\n"));
      setLabel(param_status_label_, "Latest runtime query results:");
      return;
    }
  }
  param_lines_.append(line);
  setLabel(params_label_, param_lines_.join("\n"));
  setLabel(param_status_label_, "Latest runtime query results:");
}

std::string NavigationStatusPanel::statusText(int8_t status) const
{
  switch (status) {
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      return "accepted";
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      return "executing";
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      return "canceling";
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      return "succeeded";
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      return "canceled";
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      return "aborted";
    default:
      return "unknown";
  }
}

std::string NavigationStatusPanel::parameterValueText(
  const rcl_interfaces::msg::ParameterValue & value) const
{
  using Type = rcl_interfaces::msg::ParameterType;
  std::ostringstream stream;
  switch (value.type) {
    case Type::PARAMETER_BOOL:
      return value.bool_value ? "true" : "false";
    case Type::PARAMETER_INTEGER:
      return std::to_string(value.integer_value);
    case Type::PARAMETER_DOUBLE:
      stream << value.double_value;
      return stream.str();
    case Type::PARAMETER_STRING:
      return value.string_value;
    case Type::PARAMETER_BOOL_ARRAY:
      stream << "[";
      for (size_t i = 0; i < value.bool_array_value.size(); ++i) {
        stream << (i ? ", " : "") << (value.bool_array_value[i] ? "true" : "false");
      }
      stream << "]";
      return stream.str();
    case Type::PARAMETER_INTEGER_ARRAY:
      stream << "[";
      for (size_t i = 0; i < value.integer_array_value.size(); ++i) {
        stream << (i ? ", " : "") << value.integer_array_value[i];
      }
      stream << "]";
      return stream.str();
    case Type::PARAMETER_DOUBLE_ARRAY:
      stream << "[";
      for (size_t i = 0; i < value.double_array_value.size(); ++i) {
        stream << (i ? ", " : "") << value.double_array_value[i];
      }
      stream << "]";
      return stream.str();
    case Type::PARAMETER_STRING_ARRAY:
      stream << "[";
      for (size_t i = 0; i < value.string_array_value.size(); ++i) {
        stream << (i ? ", " : "") << value.string_array_value[i];
      }
      stream << "]";
      return stream.str();
    default:
      return "unset";
  }
}

double NavigationStatusPanel::yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & orientation) const
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
}

}  // namespace shbat_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  shbat_rviz_plugins::NavigationStatusPanel,
  rviz_common::Panel)
