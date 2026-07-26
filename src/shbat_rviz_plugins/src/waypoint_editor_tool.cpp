#include "shbat_rviz_plugins/waypoint_editor_tool.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "geometry_msgs/msg/point.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/menu_entry.hpp"

namespace shbat_rviz_plugins
{
namespace
{

std::string makeId()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return "wp-" + std::to_string(now);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

WaypointEditorTool::WaypointEditorTool()
: rviz_default_plugins::tools::PoseTool()
{
}

void WaypointEditorTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Sahabat Add Waypoint");

  auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  waypoint_file_ = node_->declare_parameter<std::string>(
    "waypoint_file", "~/sahabat_ws/maps/waypoints.yaml");
  if (!waypoint_file_.empty() && waypoint_file_[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      waypoint_file_.replace(0, 1, home);
    }
  }

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "sahabat_waypoint_editor", node_);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/sahabat_waypoint_editor/status", rclcpp::QoS(1).transient_local());
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/waypoint_markers", rclcpp::QoS(1).transient_local());

  load_service_ = node_->create_service<Trigger>(
    "/sahabat_waypoint_editor/load",
    std::bind(&WaypointEditorTool::handleLoad, this, std::placeholders::_1, std::placeholders::_2));
  save_service_ = node_->create_service<Trigger>(
    "/sahabat_waypoint_editor/save",
    std::bind(&WaypointEditorTool::handleSave, this, std::placeholders::_1, std::placeholders::_2));
  undo_service_ = node_->create_service<Trigger>(
    "/sahabat_waypoint_editor/undo",
    std::bind(&WaypointEditorTool::handleUndo, this, std::placeholders::_1, std::placeholders::_2));
  redo_service_ = node_->create_service<Trigger>(
    "/sahabat_waypoint_editor/redo",
    std::bind(&WaypointEditorTool::handleRedo, this, std::placeholders::_1, std::placeholders::_2));
  clear_service_ = node_->create_service<Trigger>(
    "/sahabat_waypoint_editor/clear",
    std::bind(&WaypointEditorTool::handleClear, this, std::placeholders::_1, std::placeholders::_2));

  loadFromDisk();
  snapshot();
  renderMarkers();
}

void WaypointEditorTool::activate()
{
  PoseTool::activate();
}

void WaypointEditorTool::deactivate()
{
  PoseTool::deactivate();
}

void WaypointEditorTool::onPoseSet(double x, double y, double theta)
{
  EditorWaypoint waypoint;
  waypoint.id = makeId();
  waypoint.name = "waypoint_" + std::to_string(waypoints_.size() + 1);
  waypoint.x = x;
  waypoint.y = y;
  waypoint.yaw = theta;
  waypoints_.push_back(waypoint);
  snapshot();
  renderMarkers();
  publishStatus("Added " + waypoint.name);
  deactivate();
}

void WaypointEditorTool::loadFromDisk()
{
  waypoints_.clear();
  preserved_segments_ = YAML::Node();
  preserved_settings_ = YAML::Node();
  set_name_ = "Default";
  revision_ = 0;

  if (!std::filesystem::exists(waypoint_file_)) {
    publishStatus("No waypoint file yet: " + waypoint_file_);
    return;
  }

  YAML::Node root = YAML::LoadFile(waypoint_file_);
  if (root["name"]) {
    set_name_ = root["name"].as<std::string>();
  }
  if (root["revision"]) {
    revision_ = root["revision"].as<int>();
  }
  preserved_segments_ = root["segments"];
  preserved_settings_ = root["settings"];

  const auto entries = root["waypoints"];
  if (!entries || !entries.IsSequence()) {
    publishStatus("Waypoint file has no waypoints list");
    return;
  }
  for (const auto & item : entries) {
    EditorWaypoint waypoint;
    waypoint.id = item["id"] ? item["id"].as<std::string>() : makeId();
    waypoint.name = item["name"] ? item["name"].as<std::string>() : "waypoint";
    waypoint.x = item["x"] ? item["x"].as<double>() : 0.0;
    waypoint.y = item["y"] ? item["y"].as<double>() : 0.0;
    waypoint.yaw = item["yaw"] ? item["yaw"].as<double>() : 0.0;
    waypoint.dwell_seconds = item["dwell_seconds"] ? item["dwell_seconds"].as<double>() : 0.0;
    waypoint.enabled = item["enabled"] ? item["enabled"].as<bool>() : true;
    waypoints_.push_back(waypoint);
  }
  history_.clear();
  history_index_ = 0;
  snapshot();
  publishStatus("Loaded " + std::to_string(waypoints_.size()) + " waypoint(s)");
}

bool WaypointEditorTool::saveToDisk(std::string & error)
{
  try {
    std::filesystem::create_directories(std::filesystem::path(waypoint_file_).parent_path());
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << set_name_;
    out << YAML::Key << "revision" << YAML::Value << revision_ + 1;
    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto & waypoint : waypoints_) {
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << waypoint.id;
      out << YAML::Key << "name" << YAML::Value << waypoint.name;
      out << YAML::Key << "x" << YAML::Value << waypoint.x;
      out << YAML::Key << "y" << YAML::Value << waypoint.y;
      out << YAML::Key << "yaw" << YAML::Value << waypoint.yaw;
      out << YAML::Key << "dwell_seconds" << YAML::Value << waypoint.dwell_seconds;
      out << YAML::Key << "enabled" << YAML::Value << waypoint.enabled;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    if (preserved_segments_ && !preserved_segments_.IsNull()) {
      out << YAML::Key << "segments" << YAML::Value << preserved_segments_;
    }
    if (preserved_settings_ && !preserved_settings_.IsNull()) {
      out << YAML::Key << "settings" << YAML::Value << preserved_settings_;
    }
    out << YAML::EndMap;

    const auto path = std::filesystem::path(waypoint_file_);
    const auto temporary = path.parent_path() / ("." + path.filename().string() + ".tmp");
    std::ofstream stream(temporary);
    stream << out.c_str() << "\n";
    stream.close();
    std::filesystem::rename(temporary, path);
    revision_ += 1;
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
}

void WaypointEditorTool::renderMarkers()
{
  server_->clear();
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    server_->insert(makeMarker(i), std::bind(&WaypointEditorTool::processFeedback, this, std::placeholders::_1));
  }
  server_->applyChanges();
  publishDisplayMarkers();
}

void WaypointEditorTool::publishDisplayMarkers()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto & waypoint = waypoints_[index];

    visualization_msgs::msg::Marker body;
    body.header.frame_id = "map";
    body.header.stamp = node_->now();
    body.ns = "sahabat_waypoint_editor_points";
    body.id = static_cast<int>(index);
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = waypoint.x;
    body.pose.position.y = waypoint.y;
    body.pose.position.z = 0.08;
    body.scale.x = 0.34;
    body.scale.y = 0.34;
    body.scale.z = 0.16;
    body.color.r = waypoint.enabled ? 0.0 : 0.45;
    body.color.g = waypoint.enabled ? 0.85 : 0.45;
    body.color.b = waypoint.enabled ? 0.75 : 0.45;
    body.color.a = waypoint.enabled ? 0.95 : 0.35;
    markers.markers.push_back(body);

    visualization_msgs::msg::Marker arrow;
    arrow.header = body.header;
    arrow.ns = "sahabat_waypoint_editor_arrows";
    arrow.id = static_cast<int>(index + 1000);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x = waypoint.x;
    arrow.pose.position.y = waypoint.y;
    arrow.pose.position.z = 0.1;
    arrow.pose.orientation = quaternionFromYaw(waypoint.yaw);
    arrow.scale.x = 0.55;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.08;
    arrow.color.r = 1.0;
    arrow.color.g = 0.56;
    arrow.color.b = 0.1;
    arrow.color.a = waypoint.enabled ? 0.95 : 0.35;
    markers.markers.push_back(arrow);

    visualization_msgs::msg::Marker label;
    label.header = body.header;
    label.ns = "sahabat_waypoint_editor_labels";
    label.id = static_cast<int>(index + 2000);
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = waypoint.x;
    label.pose.position.y = waypoint.y;
    label.pose.position.z = 0.45;
    label.scale.z = 0.22;
    label.color.r = 1.0;
    label.color.g = 1.0;
    label.color.b = 1.0;
    label.color.a = 1.0;
    label.text = std::to_string(index + 1) + "\n" + waypoint.name;
    markers.markers.push_back(label);
  }

  marker_pub_->publish(markers);
}

visualization_msgs::msg::InteractiveMarker WaypointEditorTool::makeMarker(std::size_t index) const
{
  const auto & waypoint = waypoints_.at(index);
  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = "map";
  marker.name = std::to_string(index);
  marker.description = std::to_string(index + 1) + "  " + waypoint.name;
  marker.scale = 1.0;
  marker.pose.position.x = waypoint.x;
  marker.pose.position.y = waypoint.y;
  marker.pose.orientation = quaternionFromYaw(waypoint.yaw);

  visualization_msgs::msg::InteractiveMarkerControl move;
  move.name = "move";
  move.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  move.always_visible = true;
  move.orientation.w = 0.7071;
  move.orientation.y = 0.7071;
  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.36;
  sphere.scale.y = 0.36;
  sphere.scale.z = 0.18;
  sphere.color.r = waypoint.enabled ? 0.0 : 0.45;
  sphere.color.g = waypoint.enabled ? 0.85 : 0.45;
  sphere.color.b = waypoint.enabled ? 0.75 : 0.45;
  sphere.color.a = waypoint.enabled ? 0.92 : 0.38;
  move.markers.push_back(sphere);
  marker.controls.push_back(move);

  visualization_msgs::msg::InteractiveMarkerControl rotate;
  rotate.name = "rotate";
  rotate.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  rotate.orientation.w = 0.7071;
  rotate.orientation.y = -0.7071;
  marker.controls.push_back(rotate);

  visualization_msgs::msg::InteractiveMarkerControl arrow_control;
  arrow_control.name = "heading";
  arrow_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  arrow_control.always_visible = true;
  visualization_msgs::msg::Marker arrow;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.scale.x = 0.58;
  arrow.scale.y = 0.09;
  arrow.scale.z = 0.09;
  arrow.color.r = 1.0;
  arrow.color.g = 0.56;
  arrow.color.b = 0.1;
  arrow.color.a = waypoint.enabled ? 0.95 : 0.35;
  arrow_control.markers.push_back(arrow);
  marker.controls.push_back(arrow_control);

  visualization_msgs::msg::InteractiveMarkerControl label_control;
  label_control.name = "label";
  label_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  label_control.always_visible = true;
  visualization_msgs::msg::Marker text;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.position.z = 0.45;
  text.scale.z = 0.22;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.color.a = 1.0;
  text.text = std::to_string(index + 1) + "\n" + waypoint.name;
  label_control.markers.push_back(text);
  marker.controls.push_back(label_control);

  visualization_msgs::msg::InteractiveMarkerControl menu;
  menu.name = "menu";
  menu.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  menu.always_visible = true;
  marker.controls.push_back(menu);

  visualization_msgs::msg::MenuEntry toggle;
  toggle.id = 1;
  toggle.parent_id = 0;
  toggle.title = waypoint.enabled ? "Disable Waypoint" : "Enable Waypoint";
  marker.menu_entries.push_back(toggle);
  visualization_msgs::msg::MenuEntry erase;
  erase.id = 2;
  erase.parent_id = 0;
  erase.title = "Delete Waypoint";
  marker.menu_entries.push_back(erase);
  return marker;
}

void WaypointEditorTool::processFeedback(Feedback::ConstSharedPtr feedback)
{
  const auto index = static_cast<std::size_t>(std::stoul(feedback->marker_name));
  if (!validIndex(index)) {
    return;
  }
  if (feedback->event_type == Feedback::POSE_UPDATE) {
    waypoints_[index].x = feedback->pose.position.x;
    waypoints_[index].y = feedback->pose.position.y;
    waypoints_[index].yaw = yawFromQuaternion(feedback->pose.orientation);
    pose_dirty_ = true;
    server_->setPose(feedback->marker_name, feedback->pose);
    server_->applyChanges();
  } else if (feedback->event_type == Feedback::MOUSE_UP && pose_dirty_) {
    pose_dirty_ = false;
    snapshot();
    publishDisplayMarkers();
  } else if (feedback->event_type == Feedback::MENU_SELECT) {
    if (feedback->menu_entry_id == 1) {
      waypoints_[index].enabled = !waypoints_[index].enabled;
      snapshot();
      renderMarkers();
    } else if (feedback->menu_entry_id == 2) {
      deleteWaypoint(index);
    }
  }
}

void WaypointEditorTool::deleteWaypoint(std::size_t index)
{
  if (!validIndex(index)) {
    return;
  }
  waypoints_.erase(waypoints_.begin() + static_cast<long>(index));
  snapshot();
  renderMarkers();
  publishStatus("Deleted waypoint");
}

void WaypointEditorTool::snapshot()
{
  if (history_index_ + 1 < history_.size()) {
    history_.erase(history_.begin() + static_cast<long>(history_index_ + 1), history_.end());
  }
  history_.push_back(waypoints_);
  history_index_ = history_.size() - 1;
}

void WaypointEditorTool::publishStatus(const std::string & message)
{
  std_msgs::msg::String status;
  status.data = message;
  status_pub_->publish(status);
  RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
}

bool WaypointEditorTool::validIndex(std::size_t index) const
{
  return index < waypoints_.size();
}

void WaypointEditorTool::handleLoad(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  loadFromDisk();
  renderMarkers();
  response->success = true;
  response->message = "Loaded waypoints";
}

void WaypointEditorTool::handleSave(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  std::string error;
  response->success = saveToDisk(error);
  response->message = response->success ? "Saved waypoints" : error;
  publishStatus(response->message);
}

void WaypointEditorTool::handleUndo(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (history_index_ == 0 || history_.empty()) {
    response->success = false;
    response->message = "Nothing to undo";
    return;
  }
  history_index_ -= 1;
  waypoints_ = history_[history_index_];
  renderMarkers();
  response->success = true;
  response->message = "Undid waypoint edit";
}

void WaypointEditorTool::handleRedo(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  if (history_index_ + 1 >= history_.size()) {
    response->success = false;
    response->message = "Nothing to redo";
    return;
  }
  history_index_ += 1;
  waypoints_ = history_[history_index_];
  renderMarkers();
  response->success = true;
  response->message = "Redid waypoint edit";
}

void WaypointEditorTool::handleClear(
  const std::shared_ptr<Trigger::Request> /*request*/,
  std::shared_ptr<Trigger::Response> response)
{
  waypoints_.clear();
  snapshot();
  renderMarkers();
  response->success = true;
  response->message = "Cleared waypoints";
}

}  // namespace shbat_rviz_plugins

PLUGINLIB_EXPORT_CLASS(shbat_rviz_plugins::WaypointEditorTool, rviz_common::Tool)
