#ifndef SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_TOOL_HPP_
#define SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

namespace shbat_rviz_plugins
{

struct EditorWaypoint
{
  std::string id;
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double dwell_seconds{0.0};
  bool enabled{true};
};

class WaypointEditorTool : public rviz_default_plugins::tools::PoseTool
{
public:
  WaypointEditorTool();
  void onInitialize() override;
  void onPoseSet(double x, double y, double theta) override;
  void activate() override;
  void deactivate() override;

private:
  using Trigger = std_srvs::srv::Trigger;
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;

  void loadFromDisk();
  bool saveToDisk(std::string & error);
  void renderMarkers();
  void publishDisplayMarkers();
  visualization_msgs::msg::InteractiveMarker makeMarker(std::size_t index) const;
  void processFeedback(Feedback::ConstSharedPtr feedback);
  void deleteWaypoint(std::size_t index);
  void snapshot();
  void publishStatus(const std::string & message);
  bool validIndex(std::size_t index) const;

  void handleLoad(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleSave(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleUndo(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleRedo(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleClear(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<Trigger>::SharedPtr load_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr undo_service_;
  rclcpp::Service<Trigger>::SharedPtr redo_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;

  std::string waypoint_file_;
  std::string set_name_{"Default"};
  int revision_{0};
  YAML::Node preserved_segments_;
  YAML::Node preserved_settings_;
  std::vector<EditorWaypoint> waypoints_;
  std::vector<std::vector<EditorWaypoint>> history_;
  std::size_t history_index_{0};
  bool pose_dirty_{false};
};

}  // namespace shbat_rviz_plugins

#endif  // SHBAT_RVIZ_PLUGINS__WAYPOINT_EDITOR_TOOL_HPP_
