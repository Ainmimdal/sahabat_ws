#ifndef SHBAT_RVIZ_PLUGINS__APRILTAG_LANDMARKS_PANEL_HPP_
#define SHBAT_RVIZ_PLUGINS__APRILTAG_LANDMARKS_PANEL_HPP_

#include <memory>
#include <string>

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sahabat_interfaces/msg/april_tag_landmark_array.hpp"
#include "sahabat_interfaces/srv/manage_april_tag_landmark.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace shbat_rviz_plugins
{

class AprilTagLandmarksPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AprilTagLandmarksPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void captureSelected();
  void deleteSelected();
  void localizeNow();
  void reloadMap();
  void selectedTagChanged();
  void syncMapSelection();

private:
  using Landmark = sahabat_interfaces::msg::AprilTagLandmark;
  using LandmarkArray = sahabat_interfaces::msg::AprilTagLandmarkArray;
  using ManageLandmark = sahabat_interfaces::srv::ManageAprilTagLandmark;

  const Landmark * selectedLandmark() const;
  void updateState(LandmarkArray::ConstSharedPtr message);
  void updateControls();
  void callManage(
    uint8_t action, int32_t tag_id = 0, const QString & name = {},
    const QString & map_id = {});
  void setStatus(const QString & text);

  QLabel * map_label_;
  QLabel * pose_label_;
  QLabel * detail_label_;
  QLabel * status_label_;
  QComboBox * tag_combo_;
  QLineEdit * name_edit_;
  QPushButton * capture_button_;
  QPushButton * delete_button_;
  QPushButton * localize_button_;
  QPushButton * reload_button_;
  QTimer * map_timer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<LandmarkArray>::SharedPtr state_sub_;
  rclcpp::Client<ManageLandmark>::SharedPtr manage_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localize_client_;
  LandmarkArray state_;
  std::string last_rviz_map_id_;
};

}  // namespace shbat_rviz_plugins

#endif  // SHBAT_RVIZ_PLUGINS__APRILTAG_LANDMARKS_PANEL_HPP_
