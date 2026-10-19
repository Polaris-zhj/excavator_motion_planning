#include "planning_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <QColor>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QSizePolicy>
#include <QString>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace excavator_visualization {
namespace {

const std::vector<std::string> kDefaultJointNames = {
    "swing_joint", "boom_joint", "stick_joint", "bucket_joint"};

QDoubleSpinBox* MakeSpinBox(double minimum, double maximum, double step,
                            double value, int decimals = 3) {
  auto* box = new QDoubleSpinBox();
  box->setRange(minimum, maximum);
  box->setDecimals(decimals);
  box->setSingleStep(step);
  box->setValue(value);
  box->setMinimumWidth(44);
  box->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  return box;
}

QPushButton* MakeActionButton(const QString& text, const QString& color) {
  auto* button = new QPushButton(text);
  button->setCursor(Qt::PointingHandCursor);
  button->setMinimumHeight(30);
  button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  button->setStyleSheet(
      QString(
          "QPushButton { background:%1; color:white; border:0; "
          "border-radius:4px; padding:6px 8px; font-weight:600; }"
          "QPushButton:hover { background:%2; }"
          "QPushButton:pressed { background:%3; }"
          "QPushButton:disabled { background:#4a4d52; color:#9aa0a6; }")
          .arg(color, QColor(color).lighter(115).name(),
               QColor(color).darker(115).name()));
  return button;
}

std_msgs::msg::ColorRGBA MakeColor(float red, float green, float blue,
                                   float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

visualization_msgs::msg::InteractiveMarkerControl MakeControl(
    const std::string& name, uint8_t mode, double x, double y, double z) {
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = name;
  control.interaction_mode = mode;
  control.orientation.w = 1.0;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  return control;
}

}  // namespace

ExcavatorPlanningPanel::ExcavatorPlanningPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
  BuildUi();
}

void ExcavatorPlanningPanel::BuildUi() {
  setMinimumWidth(240);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  // Keep the panel usable when the vertical dock splitter is moved; the
  // remaining content is reachable by scrolling instead of being clipped.
  scroll->setMinimumHeight(240);

  auto* content = new QWidget(scroll);
  auto* layout = new QVBoxLayout(content);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(8);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  // Fixed footer: primary actions, status and progress stay visible while
  // the configuration groups above scroll independently.
  auto* footer = new QWidget(this);
  auto* footer_layout = new QVBoxLayout(footer);
  footer_layout->setContentsMargins(8, 6, 8, 8);
  footer_layout->setSpacing(5);
  footer->setStyleSheet(
      "QLabel#status { background:palette(alternate-base); "
      "border:1px solid palette(mid); border-radius:4px; padding:4px; }");

  plan_status_label_ = new QLabel("Plan: idle", footer);
  plan_status_label_->setObjectName("status");
  plan_status_label_->setSizePolicy(QSizePolicy::Ignored,
                                    QSizePolicy::Preferred);
  plan_status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  execution_status_label_ = new QLabel("Execution: idle", footer);
  execution_status_label_->setObjectName("status");
  execution_status_label_->setSizePolicy(QSizePolicy::Ignored,
                                         QSizePolicy::Preferred);
  execution_status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  execution_progress_bar_ = new QProgressBar(footer);
  execution_progress_bar_->setRange(0, 100);
  execution_progress_bar_->setValue(0);
  execution_progress_bar_->setTextVisible(true);
  execution_progress_bar_->setFormat("%p%");
  execution_progress_bar_->setMaximumHeight(14);
  footer_layout->addWidget(plan_status_label_);
  footer_layout->addWidget(execution_status_label_);
  footer_layout->addWidget(execution_progress_bar_);

  auto* buttons = new QGridLayout();
  buttons->setHorizontalSpacing(6);
  buttons->setVerticalSpacing(6);
  plan_button_ = MakeActionButton("Plan / preview", "#2f6fb0");
  execute_button_ = MakeActionButton("Confirm && execute", "#2e7d32");
  cancel_button_ = MakeActionButton("Cancel", "#5f6368");
  auto* stop_button = MakeActionButton("Stop", "#b23b3b");
  execute_button_->setEnabled(false);
  plan_button_->setToolTip("Generate a trajectory and show the preview");
  execute_button_->setToolTip("Execute the previewed trajectory");
  cancel_button_->setToolTip("Cancel the active execution");
  stop_button->setToolTip("Immediate stop");
  buttons->addWidget(plan_button_, 0, 0);
  buttons->addWidget(execute_button_, 0, 1);
  buttons->addWidget(cancel_button_, 1, 0);
  buttons->addWidget(stop_button, 1, 1);
  buttons->setColumnStretch(0, 1);
  buttons->setColumnStretch(1, 1);
  footer_layout->addLayout(buttons);
  outer->addWidget(footer);

  content->setStyleSheet(
      "QGroupBox { font-weight:600; border:1px solid palette(mid); "
      "border-radius:5px; margin-top:12px; padding-top:4px; }"
      "QGroupBox::title { subcontrol-origin:margin; left:8px; "
      "padding:0 4px; }"
      "QLabel#fieldLabel { color:palette(mid); font-size:11px; }"
      "QLabel#status { background:palette(alternate-base); "
      "border:1px solid palette(mid); border-radius:4px; padding:5px; }"
      "QDoubleSpinBox, QLineEdit { padding:2px 4px; }");

  const auto make_field_label = [content](const QString& text) {
    auto* label = new QLabel(text, content);
    label->setObjectName("fieldLabel");
    return label;
  };
  const auto add_labelled_field = [&make_field_label](
                                      QGridLayout* grid, int label_row,
                                      int field_row, int column,
                                      const QString& text, QWidget* field) {
    grid->addWidget(make_field_label(text), label_row, column);
    grid->addWidget(field, field_row, column);
  };

  // Joint-space goal: one stretchable column per joint.
  auto* target_group = new QGroupBox("Goal joint positions (rad)", content);
  auto* target_grid = new QGridLayout(target_group);
  target_grid->setHorizontalSpacing(6);
  target_grid->setVerticalSpacing(2);
  for (std::size_t index = 0; index < kDefaultJointNames.size(); ++index) {
    const int column = static_cast<int>(index);
    target_grid->setColumnStretch(column, 1);
    const QString name =
        QString::fromStdString(kDefaultJointNames[index]);
    auto* label = make_field_label(name);
    label->setToolTip(name);
    if (name.endsWith("_joint")) {
      label->setText(name.left(name.size() - 6));
    }
    target_grid->addWidget(label, 0, column);
    auto* box = MakeSpinBox(-6.2832, 6.2832, 0.01, 0.0);
    box->setToolTip(name);
    target_spinboxes_.push_back(box);
    target_grid->addWidget(box, 1, column);
  }
  layout->addWidget(target_group);

  // Planning start.
  auto* start_group = new QGroupBox(
      "Planning start (base_link; default: current state)", content);
  auto* start_grid = new QGridLayout(start_group);
  start_grid->setHorizontalSpacing(6);
  start_grid->setVerticalSpacing(4);
  use_pose_start_ = new QCheckBox("Use pose start", start_group);
  use_pose_start_->setToolTip(
      "Use the x/y/z fields or the blue interactive marker as the start pose");
  auto* use_current_start = new QPushButton("Use current state", start_group);
  use_current_start->setCursor(Qt::PointingHandCursor);
  use_current_start->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  start_grid->addWidget(use_pose_start_, 0, 0, 1, 3);
  start_grid->addWidget(use_current_start, 0, 3, 1, 3);

  start_pose_x_ = MakeSpinBox(-10.0, 10.0, 0.05, 4.0);
  start_pose_y_ = MakeSpinBox(-10.0, 10.0, 0.05, 0.0);
  start_pose_z_ = MakeSpinBox(-5.0, 10.0, 0.05, 1.0);
  const QString start_labels[] = {"x", "y", "z"};
  QDoubleSpinBox* start_fields[] = {start_pose_x_, start_pose_y_,
                                    start_pose_z_};
  for (int index = 0; index < 3; ++index) {
    const int label_column = index * 2;
    start_grid->addWidget(make_field_label(start_labels[index]), 1,
                          label_column);
    start_grid->addWidget(start_fields[index], 1, label_column + 1);
    start_grid->setColumnStretch(label_column + 1, 1);
  }
  layout->addWidget(start_group);
  connect(use_current_start, &QPushButton::clicked, this, [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    use_pose_start_->setChecked(false);
    has_start_pose_ = false;
    plan_status_ = "planning start reset to the current /joint_states";
  });

  // Cartesian goal: position row and orientation row.
  auto* pose_group = new QGroupBox(
      "Cartesian goal (base_link; overrides joint goal)", content);
  auto* pose_grid = new QGridLayout(pose_group);
  pose_grid->setHorizontalSpacing(6);
  pose_grid->setVerticalSpacing(2);
  for (int column = 0; column < 3; ++column) {
    pose_grid->setColumnStretch(column, 1);
  }
  use_pose_goal_ = new QCheckBox("Use pose", pose_group);
  constrain_orientation_ = new QCheckBox("Constrain orientation", pose_group);
  constrain_orientation_->setChecked(false);
  pose_grid->addWidget(use_pose_goal_, 0, 0, 1, 2);
  pose_grid->addWidget(constrain_orientation_, 0, 2, 1, 1);
  pose_x_ = MakeSpinBox(-10.0, 10.0, 0.05, 4.0);
  pose_y_ = MakeSpinBox(-10.0, 10.0, 0.05, 0.0);
  pose_z_ = MakeSpinBox(-5.0, 10.0, 0.05, 1.0);
  pose_roll_ = MakeSpinBox(-3.14159, 3.14159, 0.05, 0.0);
  pose_pitch_ = MakeSpinBox(-3.14159, 3.14159, 0.05, 0.0);
  pose_yaw_ = MakeSpinBox(-3.14159, 3.14159, 0.05, 0.0);
  add_labelled_field(pose_grid, 1, 2, 0, "x", pose_x_);
  add_labelled_field(pose_grid, 1, 2, 1, "y", pose_y_);
  add_labelled_field(pose_grid, 1, 2, 2, "z", pose_z_);
  add_labelled_field(pose_grid, 3, 4, 0, "roll", pose_roll_);
  add_labelled_field(pose_grid, 3, 4, 1, "pitch", pose_pitch_);
  add_labelled_field(pose_grid, 3, 4, 2, "yaw", pose_yaw_);
  layout->addWidget(pose_group);

  // Box obstacle editor: 4 responsive columns.
  auto* obstacle_group = new QGroupBox("Box obstacle (world frame)", content);
  auto* grid = new QGridLayout(obstacle_group);
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(2);
  for (int column = 0; column < 4; ++column) {
    grid->setColumnStretch(column, 1);
  }
  obstacle_id_edit_ = new QLineEdit("box_1", obstacle_group);
  obstacle_id_edit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  obstacle_x_ = MakeSpinBox(-20.0, 20.0, 0.1, 0.0);
  obstacle_y_ = MakeSpinBox(-20.0, 20.0, 0.1, 0.0);
  obstacle_z_ = MakeSpinBox(-5.0, 20.0, 0.1, 1.0);
  obstacle_yaw_ = MakeSpinBox(-3.14159, 3.14159, 0.05, 0.0);
  obstacle_sx_ = MakeSpinBox(0.01, 20.0, 0.1, 1.0);
  obstacle_sy_ = MakeSpinBox(0.01, 20.0, 0.1, 1.0);
  obstacle_sz_ = MakeSpinBox(0.01, 20.0, 0.1, 1.0);
  grid->addWidget(make_field_label("id"), 0, 0);
  grid->addWidget(obstacle_id_edit_, 0, 1, 1, 3);
  add_labelled_field(grid, 1, 2, 0, "x", obstacle_x_);
  add_labelled_field(grid, 1, 2, 1, "y", obstacle_y_);
  add_labelled_field(grid, 1, 2, 2, "z", obstacle_z_);
  add_labelled_field(grid, 1, 2, 3, "yaw", obstacle_yaw_);
  add_labelled_field(grid, 3, 4, 0, "sx", obstacle_sx_);
  add_labelled_field(grid, 3, 4, 1, "sy", obstacle_sy_);
  add_labelled_field(grid, 3, 4, 2, "sz", obstacle_sz_);
  auto* apply_button = MakeActionButton("Add / update", "#3f4348");
  auto* remove_button = MakeActionButton("Remove", "#3f4348");
  auto* clear_button = MakeActionButton("Clear", "#3f4348");
  apply_button->setToolTip("Add or update the box obstacle");
  remove_button->setToolTip("Remove the obstacle with this id");
  clear_button->setToolTip("Remove all scene obstacles");
  grid->addWidget(apply_button, 5, 0, 1, 2);
  grid->addWidget(remove_button, 5, 2);
  grid->addWidget(clear_button, 5, 3);
  layout->addWidget(obstacle_group);

  scene_label_ = new QLabel("Scene: 0 obstacles", content);
  scene_label_->setObjectName("fieldLabel");
  layout->addWidget(scene_label_);
  layout->addStretch();

  connect(plan_button_, &QPushButton::clicked, this,
          [this]() { SendPlanRequest(); });
  connect(execute_button_, &QPushButton::clicked, this,
          [this]() { SendExecuteRequest(); });
  connect(cancel_button_, &QPushButton::clicked, this,
          [this]() { CancelExecution(); });
  connect(stop_button, &QPushButton::clicked, this,
          [this]() { StopExecution(); });
  connect(apply_button, &QPushButton::clicked, this,
          [this]() { ApplyObstacleChange(true); });
  connect(remove_button, &QPushButton::clicked, this,
          [this]() { RemoveObstacle(); });
  connect(clear_button, &QPushButton::clicked, this,
          [this]() { ClearObstacles(); });

  ui_timer_ = new QTimer(this);
  connect(ui_timer_, &QTimer::timeout, this, [this]() { RefreshUi(); });
  ui_timer_->start(100);
}

void ExcavatorPlanningPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    SetStatus("ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  plan_client_ = rclcpp_action::create_client<PlanAction>(
      node_, "/plan_trajectory");
  execute_client_ = rclcpp_action::create_client<ExecuteAction>(
      node_, "/excavator/execute_trajectory");
  joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
            HandleJointState(*message);
          });
  status_subscription_ =
      node_->create_subscription<excavator_interfaces::msg::ExecutionStatus>(
          "/excavator/execution_status", 10,
          [this](excavator_interfaces::msg::ExecutionStatus::ConstSharedPtr
                     message) { HandleExecutionStatus(*message); });
  stop_publisher_ =
      node_->create_publisher<std_msgs::msg::Empty>("/excavator/stop", 10);
  target_pose_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/excavator/target_pose", rclcpp::QoS(1).transient_local(),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_target_pose_ = message->pose;
            has_pending_target_pose_ = true;
          });
  start_pose_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/excavator/start_pose", rclcpp::QoS(1).transient_local(),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_start_pose_ = message->pose;
            has_pending_start_pose_ = true;
          });
  scene_publisher_ =
      node_->create_publisher<excavator_interfaces::msg::PlanningScene>(
          "/excavator/scene", rclcpp::QoS(1).transient_local());
  scene_marker_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/excavator/scene_markers", rclcpp::QoS(1).transient_local());
  planned_trajectory_publisher_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local());
  obstacle_server_ =
      std::make_unique<interactive_markers::InteractiveMarkerServer>(
          "excavator_obstacle", node_.get());
  PublishScene();
  RefreshObstacleMarkers();
  SetStatus("ready: waiting for a /joint_states source");
}

void ExcavatorPlanningPanel::HandleJointState(
    const sensor_msgs::msg::JointState& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<double> positions;
  positions.reserve(kDefaultJointNames.size());
  for (const std::string& name : kDefaultJointNames) {
    const auto found =
        std::find(message.name.begin(), message.name.end(), name);
    if (found == message.name.end()) {
      return;
    }
    const std::size_t index =
        static_cast<std::size_t>(found - message.name.begin());
    if (index >= message.position.size()) {
      return;
    }
    positions.push_back(message.position[index]);
  }
  joint_names_ = kDefaultJointNames;
  current_positions_ = std::move(positions);
  has_joint_state_ = true;
}

void ExcavatorPlanningPanel::HandleExecutionStatus(
    const excavator_interfaces::msg::ExecutionStatus& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  execution_status_ = message.message;
  execution_progress_ = message.progress;
}

void ExcavatorPlanningPanel::SetStatus(const std::string& text) {
  std::lock_guard<std::mutex> lock(mutex_);
  plan_status_ = text;
}

void ExcavatorPlanningPanel::RefreshUi() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!targets_initialized_ && has_joint_state_) {
    for (std::size_t index = 0;
         index < target_spinboxes_.size() && index < current_positions_.size();
         ++index) {
      target_spinboxes_[index]->setValue(current_positions_[index]);
    }
    targets_initialized_ = true;
  }
  if (has_pending_start_pose_) {
    start_pose_ = pending_start_pose_;
    has_start_pose_ = true;
    use_pose_start_->setChecked(true);
    start_pose_x_->setValue(start_pose_.position.x);
    start_pose_y_->setValue(start_pose_.position.y);
    start_pose_z_->setValue(start_pose_.position.z);
    plan_status_ =
        "start pose updated from the RViz interactive marker";
    has_pending_start_pose_ = false;
  }
  if (has_pending_target_pose_) {
    use_pose_goal_->setChecked(true);
    pose_x_->setValue(pending_target_pose_.position.x);
    pose_y_->setValue(pending_target_pose_.position.y);
    pose_z_->setValue(pending_target_pose_.position.z);
    const double x = pending_target_pose_.orientation.x;
    const double y = pending_target_pose_.orientation.y;
    const double z = pending_target_pose_.orientation.z;
    const double w = pending_target_pose_.orientation.w;
    const double roll = std::atan2(2.0 * (w * x + y * z),
                                   1.0 - 2.0 * (x * x + y * y));
    const double pitch_argument =
        std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch = std::asin(pitch_argument);
    const double yaw = std::atan2(2.0 * (w * z + x * y),
                                  1.0 - 2.0 * (y * y + z * z));
    pose_roll_->setValue(roll);
    pose_pitch_->setValue(pitch);
    pose_yaw_->setValue(yaw);
    plan_status_ = "target pose updated from the RViz interactive marker";
    has_pending_target_pose_ = false;
  }
  plan_status_label_->setText(QString::fromStdString("Plan: " + plan_status_));
  plan_status_label_->setToolTip(plan_status_label_->text());
  char progress[64];
  std::snprintf(progress, sizeof(progress), "Execution: %s (%.0f%%)",
                execution_status_.c_str(), execution_progress_ * 100.0F);
  execution_status_label_->setText(QString::fromUtf8(progress));
  execution_status_label_->setToolTip(execution_status_label_->text());
  execution_progress_bar_->setValue(static_cast<int>(
      std::lround(std::clamp(execution_progress_, 0.0F, 1.0F) * 100.0F)));
  scene_label_->setText(
      QString::fromStdString("Scene: " +
                             std::to_string(scene_manager_.Size()) +
                             " obstacle(s)"));
  plan_button_->setEnabled(has_joint_state_ && !plan_in_flight_);
  execute_button_->setEnabled(has_planned_trajectory_ && !execute_in_flight_);
}

void ExcavatorPlanningPanel::SendPlanRequest() {
  if (!node_ || !plan_client_) {
    return;
  }
  std::vector<double> start;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_joint_state_) {
      plan_status_ = "cannot plan: no /joint_states received";
      return;
    }
    start = current_positions_;
    plan_in_flight_ = true;
    plan_status_ = "sending planning request";
  }
  if (!plan_client_->wait_for_action_server(std::chrono::seconds(1))) {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_in_flight_ = false;
    plan_status_ = "planning action server unavailable";
    return;
  }

  PlanAction::Goal goal;
  goal.start_state.name = kDefaultJointNames;
  goal.start_state.position = start;
  goal.goal_state.name = kDefaultJointNames;
  goal.goal_state.position.clear();
  for (QDoubleSpinBox* box : target_spinboxes_) {
    goal.goal_state.position.push_back(box->value());
  }
  goal.target_link = "bucket_tip";
  goal.allowed_planning_time.sec = 5;
  goal.allowed_planning_time.nanosec = 0;
  goal.planning_scene = scene_manager_.Snapshot();
  goal.use_pose_goal = use_pose_goal_->isChecked();
  goal.constrain_orientation = constrain_orientation_->isChecked();
  goal.use_pose_start = use_pose_start_->isChecked();
  if (goal.use_pose_start) {
    geometry_msgs::msg::Pose start_pose;
    if (has_start_pose_) {
      start_pose = start_pose_;
    } else {
      start_pose.orientation.w = 1.0;
    }
    start_pose.position.x = start_pose_x_->value();
    start_pose.position.y = start_pose_y_->value();
    start_pose.position.z = start_pose_z_->value();
    goal.start_pose = start_pose;
  }
  if (goal.use_pose_goal) {
    goal.target_pose.position.x = pose_x_->value();
    goal.target_pose.position.y = pose_y_->value();
    goal.target_pose.position.z = pose_z_->value();
    const double roll = pose_roll_->value();
    const double pitch = pose_pitch_->value();
    const double yaw = pose_yaw_->value();
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    goal.target_pose.orientation.x = sr * cp * cy - cr * sp * sy;
    goal.target_pose.orientation.y = cr * sp * cy + sr * cp * sy;
    goal.target_pose.orientation.z = cr * cp * sy - sr * sp * cy;
    goal.target_pose.orientation.w = cr * cp * cy + sr * sp * sy;
  }

  rclcpp_action::Client<PlanAction>::SendGoalOptions options;
  options.goal_response_callback =
      [this](const PlanGoalHandle::SharedPtr& goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!goal_handle) {
          plan_in_flight_ = false;
          plan_status_ = "planning goal rejected";
        } else {
          plan_status_ = "planning...";
        }
      };
  options.feedback_callback =
      [this](PlanGoalHandle::SharedPtr,
             const std::shared_ptr<const PlanAction::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_status_ = feedback->stage + ": " + feedback->message;
      };
  options.result_callback =
      [this](const PlanGoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        plan_in_flight_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->success) {
          planned_trajectory_ = result.result->trajectory;
          has_planned_trajectory_ = true;
          plan_status_ =
              "success: " +
              std::to_string(planned_trajectory_.points.size()) +
              " points; preview only, execution requires confirmation";
          if (planned_trajectory_publisher_ &&
              planned_trajectory_.points.size() >= 2) {
            // Preview the result even when the planning server does not
            // publish it (other PlannerInterface implementations).
            planned_trajectory_publisher_->publish(planned_trajectory_);
          }
        } else {
          plan_status_ = "planning failed: " +
                         (result.result ? result.result->message
                                        : std::string("no result"));
          has_planned_trajectory_ = false;
        }
      };
  plan_client_->async_send_goal(goal, options);
}

void ExcavatorPlanningPanel::SendExecuteRequest() {
  if (!node_ || !execute_client_) {
    return;
  }
  trajectory_msgs::msg::JointTrajectory trajectory;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_planned_trajectory_) {
      plan_status_ = "cannot execute: no validated trajectory";
      return;
    }
    trajectory = planned_trajectory_;
    execute_in_flight_ = true;
    execution_status_ = "confirming execution";
  }
  if (!execute_client_->wait_for_action_server(std::chrono::seconds(1))) {
    std::lock_guard<std::mutex> lock(mutex_);
    execute_in_flight_ = false;
    execution_status_ = "execute action server unavailable";
    return;
  }

  ExecuteAction::Goal goal;
  goal.trajectory = trajectory;
  goal.require_confirmation = true;
  // A custom planning start is reached by re-initializing the kinematic
  // simulator; the controller rejects this when simulator initialization is
  // disabled (hardware mode).
  goal.initialize_to_start = use_pose_start_->isChecked();
  rclcpp_action::Client<ExecuteAction>::SendGoalOptions options;
  options.goal_response_callback =
      [this](const ExecuteGoalHandle::SharedPtr& goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!goal_handle) {
          execute_in_flight_ = false;
          execution_status_ = "execution goal rejected";
        }
      };
  options.feedback_callback =
      [this](ExecuteGoalHandle::SharedPtr,
             const std::shared_ptr<const ExecuteAction::Feedback> feedback) {
        std::lock_guard<std::mutex> lock(mutex_);
        execution_status_ = feedback->message;
        execution_progress_ = feedback->progress;
      };
  options.result_callback =
      [this](const ExecuteGoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        execute_in_flight_ = false;
        if (!result.result) {
          execution_status_ = "execution ended without a result";
          execution_progress_ = 0.0F;
          return;
        }
        execution_progress_ = result.result->success ? 1.0F : 0.0F;
        execution_status_ = result.result->message;
      };
  execute_client_->async_send_goal(goal, options);
}

void ExcavatorPlanningPanel::CancelExecution() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_status_ = "cancel requested";
  }
  if (plan_client_) {
    plan_client_->async_cancel_all_goals();
  }
  if (execute_client_) {
    execute_client_->async_cancel_all_goals();
  }
}

void ExcavatorPlanningPanel::StopExecution() {
  if (stop_publisher_) {
    stop_publisher_->publish(std_msgs::msg::Empty());
    std::lock_guard<std::mutex> lock(mutex_);
    execution_status_ = "immediate stop requested";
    execute_in_flight_ = false;
  }
}

void ExcavatorPlanningPanel::ApplyObstacleChange(bool update) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = obstacle_x_->value();
  pose.position.y = obstacle_y_->value();
  pose.position.z = obstacle_z_->value();
  const double yaw = obstacle_yaw_->value();
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  geometry_msgs::msg::Vector3 size;
  size.x = obstacle_sx_->value();
  size.y = obstacle_sy_->value();
  size.z = obstacle_sz_->value();
  const std::string id = obstacle_id_edit_->text().toStdString();
  tramcar::planning::ValidationResult result;
  if (update) {
    result = scene_manager_.UpdateBox(id, pose, size);
    if (!result.ok && result.message.find("does not exist") !=
                            std::string::npos) {
      result = scene_manager_.AddBox(id, pose, size);
    }
  } else {
    result = scene_manager_.AddBox(id, pose, size);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_status_ =
        result.ok ? "scene updated: " + id : "scene error: " + result.message;
  }
  PublishScene();
  RefreshObstacleMarkers();
}

void ExcavatorPlanningPanel::RefreshObstacleMarkers() {
  if (!obstacle_server_ || !node_) {
    return;
  }
  obstacle_server_->clear();
  const auto objects = scene_manager_.Objects();
  for (const tramcar::planning::visualization::BoxObstacle& box : objects) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = scene_manager_.frame_id();
    marker.header.stamp = node_->now();
    marker.name = box.id;
    marker.description = "Box obstacle " + box.id;
    marker.pose = box.pose;
    marker.scale =
        std::max({box.size.x, box.size.y, box.size.z}) * 1.4 + 0.2;

    visualization_msgs::msg::Marker visual;
    visual.type = visualization_msgs::msg::Marker::CUBE;
    visual.scale = box.size;
    visual.pose.orientation.w = 1.0;
    visual.color = MakeColor(0.9F, 0.15F, 0.1F, 0.35F);

    // The visible box belongs to the free-move control, so it can be dragged
    // in any direction rather than only along the fixed axis handles.
    visualization_msgs::msg::InteractiveMarkerControl free_move;
    free_move.name = "move_free";
    free_move.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    free_move.orientation.w = 1.0;
    free_move.always_visible = true;
    free_move.markers.push_back(visual);
    marker.controls.push_back(free_move);

    marker.controls.push_back(MakeControl(
        "rotate_3d",
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D, 0.0, 0.0,
        0.0));
    marker.controls.push_back(MakeControl(
        "move_x", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
        1.0, 0.0, 0.0));
    marker.controls.push_back(MakeControl(
        "move_y", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
        0.0, 1.0, 0.0));
    marker.controls.push_back(MakeControl(
        "move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
        0.0, 0.0, 1.0));
    marker.controls.push_back(MakeControl(
        "rotate_x",
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 1.0,
        0.0, 0.0));
    marker.controls.push_back(MakeControl(
        "rotate_y",
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0,
        1.0, 0.0));
    marker.controls.push_back(MakeControl(
        "rotate_z",
        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0,
        0.0, 1.0));

    const std::string id = box.id;
    obstacle_server_->insert(
        marker,
        [this, id](const visualization_msgs::msg::InteractiveMarkerFeedback::
                       ConstSharedPtr& feedback) {
          if (feedback->event_type !=
              visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            return;
          }
          const auto current_objects = scene_manager_.Objects();
          const auto found = std::find_if(
              current_objects.begin(), current_objects.end(),
              [&id](const tramcar::planning::visualization::BoxObstacle&
                        candidate) { return candidate.id == id; });
          if (found == current_objects.end()) {
            return;
          }
          const auto result =
              scene_manager_.UpdateBox(id, feedback->pose, found->size);
          if (result.ok) {
            std::lock_guard<std::mutex> lock(mutex_);
            plan_status_ = "obstacle '" + id + "' moved";
          }
          PublishScene();
        });
  }
  obstacle_server_->applyChanges();
}

void ExcavatorPlanningPanel::RemoveObstacle() {
  const auto result = scene_manager_.RemoveBox(
      obstacle_id_edit_->text().toStdString());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_status_ = result.ok ? "obstacle removed"
                             : "scene error: " + result.message;
  }
  PublishScene();
  RefreshObstacleMarkers();
}

void ExcavatorPlanningPanel::ClearObstacles() {
  scene_manager_.Clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_status_ = "scene cleared";
  }
  PublishScene();
  RefreshObstacleMarkers();
}

void ExcavatorPlanningPanel::PublishScene() {
  if (node_ && scene_publisher_) {
    scene_publisher_->publish(scene_manager_.Snapshot());
  }
  if (!node_ || !scene_marker_publisher_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = scene_manager_.frame_id();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  const auto objects = scene_manager_.Objects();
  for (std::size_t index = 0; index < objects.size(); ++index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = scene_manager_.frame_id();
    marker.header.stamp = node_->now();
    marker.ns = "excavator_obstacles";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = objects[index].pose;
    marker.scale = objects[index].size;
    marker.color = MakeColor(0.9F, 0.15F, 0.1F, 0.5F);
    markers.markers.push_back(std::move(marker));
  }
  scene_marker_publisher_->publish(markers);
}

}  // namespace excavator_visualization

PLUGINLIB_EXPORT_CLASS(excavator_visualization::ExcavatorPlanningPanel,
                       rviz_common::Panel)
