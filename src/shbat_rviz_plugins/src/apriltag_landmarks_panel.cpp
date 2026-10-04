#include "shbat_rviz_plugins/apriltag_landmarks_panel.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace shbat_rviz_plugins
{

AprilTagLandmarksPanel::AprilTagLandmarksPanel(QWidget * parent)
: rviz_common::Panel(parent),
  map_label_(new QLabel("Waiting for manager")),
  pose_label_(new QLabel("Map pose: unknown")),
  detail_label_(new QLabel("No tag selected")),
  status_label_(new QLabel("Waiting for AprilTag landmark state...")),
  tag_combo_(new QComboBox),
  name_edit_(new QLineEdit),
  capture_button_(new QPushButton("Capture")),
  delete_button_(new QPushButton("Delete")),
  localize_button_(new QPushButton("Localize Now")),
  reload_button_(new QPushButton("Reload")),
  map_timer_(new QTimer(this))
{
  auto * root = new QVBoxLayout;
  root->setContentsMargins(6, 6, 6, 6);

  auto * summary = new QGroupBox("AprilTag Localization");
  auto * summary_layout = new QVBoxLayout;
  auto * help = new QLabel(
    "First localize normally, face a tag, then capture it. "
    "Later the same tag can initialize AMCL at startup.");
  help->setWordWrap(true);
  map_label_->setWordWrap(true);
  pose_label_->setWordWrap(true);
  summary_layout->addWidget(help);
  summary_layout->addWidget(map_label_);
  summary_layout->addWidget(pose_label_);
  summary->setLayout(summary_layout);

  auto * tag_box = new QGroupBox("Tag");
  auto * tag_layout = new QFormLayout;
  tag_combo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  tag_combo_->setMinimumContentsLength(10);
  name_edit_->setPlaceholderText("e.g. lobby_north");
  tag_layout->addRow("Tag ID", tag_combo_);
  tag_layout->addRow("Name", name_edit_);
  detail_label_->setWordWrap(true);
  detail_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  tag_layout->addRow(detail_label_);

  auto * buttons = new QGridLayout;
  buttons->addWidget(capture_button_, 0, 0);
  buttons->addWidget(delete_button_, 0, 1);
  buttons->addWidget(localize_button_, 1, 0);
  buttons->addWidget(reload_button_, 1, 1);
  tag_layout->addRow(buttons);
  tag_box->setLayout(tag_layout);

  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  root->addWidget(summary);
  root->addWidget(tag_box);
  root->addWidget(status_label_);
  root->addStretch();
  setLayout(root);
  setMinimumWidth(250);

  connect(capture_button_, &QPushButton::clicked, this,
    &AprilTagLandmarksPanel::captureSelected);
  connect(delete_button_, &QPushButton::clicked, this,
    &AprilTagLandmarksPanel::deleteSelected);
  connect(localize_button_, &QPushButton::clicked, this,
    &AprilTagLandmarksPanel::localizeNow);
  connect(reload_button_, &QPushButton::clicked, this,
    &AprilTagLandmarksPanel::reloadMap);
  connect(tag_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    &AprilTagLandmarksPanel::selectedTagChanged);
  connect(map_timer_, &QTimer::timeout, this,
    &AprilTagLandmarksPanel::syncMapSelection);
  updateControls();
}

void AprilTagLandmarksPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    setStatus("RViz ROS node is unavailable.");
    return;
  }
  node_ = abstraction->get_raw_node();
  manage_client_ = node_->create_client<ManageLandmark>(
    "/apriltag_landmarks/manage");
  localize_client_ = node_->create_client<std_srvs::srv::Trigger>(
    "/localization/set_from_tags");
  state_sub_ = node_->create_subscription<LandmarkArray>(
    "/apriltag_landmarks/state",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](LandmarkArray::ConstSharedPtr message) {
      QMetaObject::invokeMethod(this, [this, message]() {
        updateState(message);
      }, Qt::QueuedConnection);
    });
  map_timer_->start(1000);
  syncMapSelection();
}

const AprilTagLandmarksPanel::Landmark *
AprilTagLandmarksPanel::selectedLandmark() const
{
  if (tag_combo_->currentIndex() < 0) {
    return nullptr;
  }
  const int32_t id = tag_combo_->currentData().toInt();
  const auto found = std::find_if(
    state_.landmarks.begin(), state_.landmarks.end(),
    [id](const Landmark & landmark) {return landmark.id == id;});
  return found == state_.landmarks.end() ? nullptr : &(*found);
}

void AprilTagLandmarksPanel::updateState(LandmarkArray::ConstSharedPtr message)
{
  const int previous_id = tag_combo_->currentData().toInt();
  state_ = *message;
  const int saved_count = static_cast<int>(std::count_if(
      state_.landmarks.begin(), state_.landmarks.end(),
      [](const Landmark & landmark) {return landmark.saved;}));
  map_label_->setText(QString("Map: %1 | saved tags: %2").arg(
    QString::fromStdString(state_.map_id)).arg(saved_count));
  pose_label_->setText(
    state_.map_pose_available ? "Map pose: available" : "Map pose: not established");
  setStatus(QString::fromStdString(state_.status));

  QSignalBlocker blocker(tag_combo_);
  tag_combo_->clear();
  int selected_index = -1;
  for (const auto & landmark : state_.landmarks) {
    const QString label = QString("ID %1 | %2 | %3").arg(landmark.id).arg(
      landmark.visible ? "visible" : "not visible").arg(
      landmark.saved ? "saved" : "new");
    tag_combo_->addItem(label, landmark.id);
    if (landmark.id == previous_id) {
      selected_index = tag_combo_->count() - 1;
    }
  }
  if (selected_index >= 0) {
    tag_combo_->setCurrentIndex(selected_index);
  } else if (tag_combo_->count() > 0) {
    tag_combo_->setCurrentIndex(0);
  }
  selectedTagChanged();
}

void AprilTagLandmarksPanel::selectedTagChanged()
{
  const auto * landmark = selectedLandmark();
  if (!landmark) {
    detail_label_->setText("No detected or saved tags for this map.");
    name_edit_->clear();
    updateControls();
    return;
  }
  name_edit_->setText(QString::fromStdString(landmark->name));
  detail_label_->setText(QString("Family %1 | margin %2 | distance %3 m").arg(
    QString::fromStdString(landmark->family)).arg(
    landmark->decision_margin, 0, 'f', 1).arg(landmark->distance_m, 0, 'f', 2));
  updateControls();
}

void AprilTagLandmarksPanel::updateControls()
{
  const auto * landmark = selectedLandmark();
  const bool manager_ready = manage_client_ && manage_client_->service_is_ready();
  capture_button_->setEnabled(
    manager_ready && landmark && landmark->visible && state_.map_pose_available);
  delete_button_->setEnabled(manager_ready && landmark && landmark->saved);
  capture_button_->setText(
    landmark && landmark->saved ? "Update Pose" : "Capture");
  const bool visible_saved = std::any_of(
    state_.landmarks.begin(), state_.landmarks.end(),
    [](const Landmark & item) {return item.saved && item.visible;});
  localize_button_->setEnabled(
    localize_client_ && localize_client_->service_is_ready() && visible_saved);
  reload_button_->setEnabled(manager_ready);
}

void AprilTagLandmarksPanel::captureSelected()
{
  const auto * landmark = selectedLandmark();
  if (landmark) {
    callManage(ManageLandmark::Request::CAPTURE, landmark->id, name_edit_->text());
  }
}

void AprilTagLandmarksPanel::deleteSelected()
{
  const auto * landmark = selectedLandmark();
  if (!landmark || !landmark->saved) {
    return;
  }
  if (QMessageBox::question(
      this, "Delete saved tag",
      QString("Delete saved pose for tag ID %1?").arg(landmark->id)) != QMessageBox::Yes)
  {
    return;
  }
  callManage(ManageLandmark::Request::DELETE, landmark->id);
}

void AprilTagLandmarksPanel::localizeNow()
{
  if (!localize_client_ || !localize_client_->service_is_ready()) {
    setStatus("AprilTag localization service is unavailable.");
    return;
  }
  setStatus("Requesting localization from visible saved tags...");
  localize_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      QString text;
      try {
        text = QString::fromStdString(future.get()->message);
      } catch (const std::exception & error) {
        text = QString("Localization request failed: %1").arg(error.what());
      }
      QMetaObject::invokeMethod(this, [this, text]() {setStatus(text);},
        Qt::QueuedConnection);
    });
}

void AprilTagLandmarksPanel::reloadMap()
{
  callManage(ManageLandmark::Request::RELOAD);
}

void AprilTagLandmarksPanel::syncMapSelection()
{
  updateControls();
  if (!node_ || !manage_client_ || !manage_client_->service_is_ready()) {
    return;
  }
  std::string map_id;
  if (!node_->get_parameter("map_id", map_id) || map_id.empty() ||
    map_id == last_rviz_map_id_ || map_id == state_.map_id)
  {
    return;
  }
  last_rviz_map_id_ = map_id;
  callManage(ManageLandmark::Request::SET_MAP, 0, {}, QString::fromStdString(map_id));
}

void AprilTagLandmarksPanel::callManage(
  uint8_t action, int32_t tag_id, const QString & name, const QString & map_id)
{
  if (!manage_client_ || !manage_client_->service_is_ready()) {
    setStatus("AprilTag landmark manager is unavailable.");
    return;
  }
  auto request = std::make_shared<ManageLandmark::Request>();
  request->action = action;
  request->tag_id = tag_id;
  request->name = name.toStdString();
  request->map_id = map_id.toStdString();
  manage_client_->async_send_request(
    request,
    [this](rclcpp::Client<ManageLandmark>::SharedFuture future) {
      QString text;
      try {
        text = QString::fromStdString(future.get()->message);
      } catch (const std::exception & error) {
        text = QString("Tag action failed: %1").arg(error.what());
      }
      QMetaObject::invokeMethod(this, [this, text]() {setStatus(text);},
        Qt::QueuedConnection);
    });
}

void AprilTagLandmarksPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

}  // namespace shbat_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  shbat_rviz_plugins::AprilTagLandmarksPanel, rviz_common::Panel)
