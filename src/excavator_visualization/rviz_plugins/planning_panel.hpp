#ifndef EXCAVATOR_VISUALIZATION__PLANNING_PANEL_HPP_
#define EXCAVATOR_VISUALIZATION__PLANNING_PANEL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include "excavator_interfaces/action/execute_trajectory.hpp"
#include "excavator_interfaces/action/plan_trajectory.hpp"
#include "excavator_interfaces/msg/execution_status.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "excavator_visualization/planning_scene_manager.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace excavator_visualization {

// RViz panel A: start/goal editing (joint-space targets), box obstacle
// editing, planning request/preview and confirmed execution.
class ExcavatorPlanningPanel : public rviz_common::Panel {
 public:
  explicit ExcavatorPlanningPanel(QWidget* parent = nullptr);

  void onInitialize() override;

 private:
  void BuildUi();
  void SendPlanRequest();
  void SendExecuteRequest();
  void CancelExecution();
  void StopExecution();
  void ApplyObstacleChange(bool update);
  void RemoveObstacle();
  void ClearObstacles();
  void PublishScene();
  void RefreshObstacleMarkers();
  void RefreshUi();
  void HandleJointState(const sensor_msgs::msg::JointState& message);
  void HandleExecutionStatus(
      const excavator_interfaces::msg::ExecutionStatus& message);
  void SetStatus(const std::string& text);

  using PlanAction = excavator_interfaces::action::PlanTrajectory;
  using ExecuteAction = excavator_interfaces::action::ExecuteTrajectory;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<PlanAction>;
  using ExecuteGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteAction>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PlanAction>::SharedPtr plan_client_;
  rclcpp_action::Client<ExecuteAction>::SharedPtr execute_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<excavator_interfaces::msg::ExecutionStatus>::SharedPtr
      status_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      start_pose_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
  rclcpp::Publisher<excavator_interfaces::msg::PlanningScene>::SharedPtr
      scene_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      scene_marker_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      planned_trajectory_publisher_;

  std::mutex mutex_;
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  bool has_joint_state_{false};
  bool targets_initialized_{false};
  trajectory_msgs::msg::JointTrajectory planned_trajectory_;
  bool has_planned_trajectory_{false};
  std::string plan_status_{"idle"};
  std::string execution_status_{"idle"};
  float execution_progress_{0.0F};
  bool plan_in_flight_{false};
  bool execute_in_flight_{false};
  bool has_pending_target_pose_{false};
  geometry_msgs::msg::Pose pending_target_pose_;
  bool has_start_pose_{false};
  bool has_pending_start_pose_{false};
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose pending_start_pose_;

  tramcar::planning::visualization::PlanningSceneManager scene_manager_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer>
      obstacle_server_;
  std::vector<QDoubleSpinBox*> target_spinboxes_;
  QCheckBox* use_pose_start_{nullptr};
  QDoubleSpinBox* start_pose_x_{nullptr};
  QDoubleSpinBox* start_pose_y_{nullptr};
  QDoubleSpinBox* start_pose_z_{nullptr};
  QCheckBox* use_pose_goal_{nullptr};
  QCheckBox* constrain_orientation_{nullptr};
  QDoubleSpinBox* pose_x_{nullptr};
  QDoubleSpinBox* pose_y_{nullptr};
  QDoubleSpinBox* pose_z_{nullptr};
  QDoubleSpinBox* pose_roll_{nullptr};
  QDoubleSpinBox* pose_pitch_{nullptr};
  QDoubleSpinBox* pose_yaw_{nullptr};
  QLineEdit* obstacle_id_edit_{nullptr};
  QDoubleSpinBox* obstacle_x_{nullptr};
  QDoubleSpinBox* obstacle_y_{nullptr};
  QDoubleSpinBox* obstacle_z_{nullptr};
  QDoubleSpinBox* obstacle_yaw_{nullptr};
  QDoubleSpinBox* obstacle_sx_{nullptr};
  QDoubleSpinBox* obstacle_sy_{nullptr};
  QDoubleSpinBox* obstacle_sz_{nullptr};
  QLabel* plan_status_label_{nullptr};
  QLabel* execution_status_label_{nullptr};
  QLabel* scene_label_{nullptr};
  QProgressBar* execution_progress_bar_{nullptr};
  QPushButton* plan_button_{nullptr};
  QPushButton* execute_button_{nullptr};
  QPushButton* cancel_button_{nullptr};
  QTimer* ui_timer_{nullptr};
};

}  // namespace excavator_visualization

#endif  // EXCAVATOR_VISUALIZATION__PLANNING_PANEL_HPP_
