#ifndef EXCAVATOR_VISUALIZATION__JOINT_PLOT_PANEL_HPP_
#define EXCAVATOR_VISUALIZATION__JOINT_PLOT_PANEL_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QColor>
#include <QLabel>
#include <QPoint>
#include <QRect>
#include <QTimer>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "excavator_interfaces/msg/execution_status.hpp"
#include "excavator_visualization/trajectory_plot_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_visualization {

struct PlotCurve {
  std::string name;
  QColor color;
  tramcar::planning::visualization::PlotSeries reference;
  tramcar::planning::visualization::PlotSeries actual;
};

// Small QPainter plot widget. QCustomPlot is deliberately not required.
// One widget shows all joint curves for one quantity, with a colored legend.
class JointPlotWidget : public QWidget {
 public:
  explicit JointPlotWidget(QWidget* parent = nullptr);

  void SetCurves(std::vector<PlotCurve> curves, const std::string& title,
                 const std::string& unit);

  // Return to the automatic data range after mouse zoom/pan.
  void ResetView();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;

 private:
  QRect PlotRect() const;
  bool DataBounds(double* max_t, double* min_v, double* max_v) const;

  std::vector<PlotCurve> curves_;
  std::string title_;
  std::string unit_;

  // View window. auto_view_ follows the data extents; mouse interaction
  // switches to a fixed window until ResetView()/double-click.
  bool auto_view_{true};
  bool has_data_{false};
  double view_t_lo_{0.0};
  double view_t_hi_{1.0};
  double view_v_lo_{-1.0};
  double view_v_hi_{1.0};
  // Effective bounds used by the last paint (also the anchor for zoom).
  double current_t_lo_{0.0};
  double current_t_hi_{1.0};
  double current_v_lo_{-1.0};
  double current_v_hi_{1.0};

  bool dragging_{false};
  QPoint drag_start_;
  double drag_t_lo_{0.0};
  double drag_t_hi_{1.0};
  double drag_v_lo_{-1.0};
  double drag_v_hi_{1.0};
};

// RViz panel B: simultaneous q(t), dq(t) and ddq(t) plots. Every plot draws
// all joints with stable per-joint colors and a colored legend; solid lines
// are reference values and dashed lines are measured values.
class ExcavatorJointPlotPanel : public rviz_common::Panel {
 public:
  explicit ExcavatorJointPlotPanel(QWidget* parent = nullptr);

  void onInitialize() override;

 private:
  void HandleTrajectory(
      const trajectory_msgs::msg::JointTrajectory& message);
  void HandleJointState(const sensor_msgs::msg::JointState& message);
  void HandleExecutionStatus(
      const excavator_interfaces::msg::ExecutionStatus& message);
  void RefreshPlot();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<excavator_interfaces::msg::ExecutionStatus>::SharedPtr
      status_subscription_;

  std::mutex mutex_;
  tramcar::planning::visualization::TrajectoryPlotData plot_data_;
  std::vector<std::string> joint_names_;
  bool execution_running_{false};
  bool has_execution_status_{false};
  bool reset_view_pending_{false};
  bool offline_started_{false};
  double offline_start_stamp_{0.0};
  double execution_elapsed_{0.0};
  double status_stamp_seconds_{0.0};
  std::chrono::steady_clock::time_point status_receipt_time_{};
  std::string last_status_{"waiting for a planned trajectory"};

  JointPlotWidget* position_plot_{nullptr};
  JointPlotWidget* velocity_plot_{nullptr};
  JointPlotWidget* acceleration_plot_{nullptr};
  QLabel* info_label_{nullptr};
  QTimer* timer_{nullptr};
};

}  // namespace excavator_visualization

#endif  // EXCAVATOR_VISUALIZATION__JOINT_PLOT_PANEL_HPP_
