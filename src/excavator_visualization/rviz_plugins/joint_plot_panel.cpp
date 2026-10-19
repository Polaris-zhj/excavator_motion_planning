#include "joint_plot_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QSizePolicy>
#include <QString>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace excavator_visualization {
namespace {

using tramcar::planning::visualization::PlotQuantity;
using tramcar::planning::visualization::PlotSeries;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

double ToSeconds(const builtin_interfaces::msg::Time& time) {
  return static_cast<double>(time.sec) +
         static_cast<double>(time.nanosec) * 1.0e-9;
}

tramcar::planning::JointTrajectory ToCoreTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message) {
  tramcar::planning::JointTrajectory trajectory;
  trajectory.joint_names = message.joint_names;
  trajectory.points.reserve(message.points.size());
  for (const auto& message_point : message.points) {
    tramcar::planning::JointTrajectoryPoint point;
    point.t = ToSeconds(message_point.time_from_start);
    point.positions = message_point.positions;
    point.velocities = message_point.velocities;
    point.accelerations = message_point.accelerations;
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

const std::vector<QColor>& JointColors() {
  static const std::vector<QColor> colors = {
      QColor(79, 195, 247),   // light blue
      QColor(255, 183, 77),   // orange
      QColor(129, 199, 132),  // green
      QColor(186, 104, 200),  // purple
      QColor(255, 241, 118),  // yellow
      QColor(240, 98, 146),   // pink
  };
  return colors;
}

}  // namespace

JointPlotWidget::JointPlotWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setToolTip(
      "Wheel: zoom (Ctrl = time only, Shift = value only)\n"
      "Left drag: pan\nDouble-click: reset to automatic range");
}

QRect JointPlotWidget::PlotRect() const {
  return rect().adjusted(52, 62, -12, -22);
}

bool JointPlotWidget::DataBounds(double* max_t, double* min_v,
                                 double* max_v) const {
  if (max_t == nullptr || min_v == nullptr || max_v == nullptr) {
    return false;
  }
  *max_t = 0.0;
  *min_v = std::numeric_limits<double>::infinity();
  *max_v = -std::numeric_limits<double>::infinity();
  bool has_data = false;
  for (const PlotCurve& curve : curves_) {
    for (const PlotSeries* series : {&curve.reference, &curve.actual}) {
      for (std::size_t index = 0; index < series->times.size(); ++index) {
        has_data = true;
        *max_t = std::max(*max_t, series->times[index]);
        *min_v = std::min(*min_v, series->values[index]);
        *max_v = std::max(*max_v, series->values[index]);
      }
    }
  }
  if (!has_data) {
    *max_t = 1.0;
    *min_v = -1.0;
    *max_v = 1.0;
  }
  return has_data;
}

void JointPlotWidget::ResetView() {
  auto_view_ = true;
  dragging_ = false;
  unsetCursor();
  update();
}

void JointPlotWidget::wheelEvent(QWheelEvent* event) {
  if (!has_data_) {
    event->ignore();
    return;
  }
  const int delta = event->angleDelta().y();
  if (delta == 0) {
    event->ignore();
    return;
  }
  const QRect plot = PlotRect();
  const double x_ratio = std::clamp(
      (event->position().x() - plot.left()) /
          std::max(1.0, static_cast<double>(plot.width())),
      0.0, 1.0);
  const double y_ratio = std::clamp(
      (plot.bottom() - event->position().y()) /
          std::max(1.0, static_cast<double>(plot.height())),
      0.0, 1.0);
  const double factor = delta > 0 ? 1.0 / 1.2 : 1.2;
  const bool zoom_time =
      (event->modifiers() & Qt::ShiftModifier) == 0;
  const bool zoom_value =
      (event->modifiers() & Qt::ControlModifier) == 0;

  double data_max_t = 0.0;
  double data_min_v = 0.0;
  double data_max_v = 0.0;
  DataBounds(&data_max_t, &data_min_v, &data_max_v);
  const double max_t_span = 3.0 * std::max(1.0e-6, data_max_t);
  const double max_v_span =
      3.0 * std::max(1.0e-6, data_max_v - data_min_v);

  if (zoom_time) {
    const double anchor = current_t_lo_ +
                          x_ratio * (current_t_hi_ - current_t_lo_);
    double lo = anchor - (anchor - current_t_lo_) * factor;
    double hi = anchor + (current_t_hi_ - anchor) * factor;
    constexpr double kMinSpan = 1.0e-4;
    if (hi - lo < kMinSpan) {
      const double center = 0.5 * (lo + hi);
      lo = center - 0.5 * kMinSpan;
      hi = center + 0.5 * kMinSpan;
    }
    // Do not let repeated wheel-out gestures shrink the curves to an
    // invisible dot far outside the data.
    if (hi - lo > max_t_span) {
      lo = anchor - x_ratio * max_t_span;
      hi = lo + max_t_span;
    }
    view_t_lo_ = lo;
    view_t_hi_ = hi;
  }
  if (zoom_value) {
    const double anchor = current_v_lo_ +
                          y_ratio * (current_v_hi_ - current_v_lo_);
    double lo = anchor - (anchor - current_v_lo_) * factor;
    double hi = anchor + (current_v_hi_ - anchor) * factor;
    constexpr double kMinSpan = 1.0e-5;
    if (hi - lo < kMinSpan) {
      const double center = 0.5 * (lo + hi);
      lo = center - 0.5 * kMinSpan;
      hi = center + 0.5 * kMinSpan;
    }
    if (hi - lo > max_v_span) {
      lo = anchor - y_ratio * max_v_span;
      hi = lo + max_v_span;
    }
    view_v_lo_ = lo;
    view_v_hi_ = hi;
  }
  auto_view_ = false;
  update();
  event->accept();
}

void JointPlotWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton && has_data_) {
    dragging_ = true;
    drag_start_ = event->pos();
    drag_t_lo_ = current_t_lo_;
    drag_t_hi_ = current_t_hi_;
    drag_v_lo_ = current_v_lo_;
    drag_v_hi_ = current_v_hi_;
    auto_view_ = false;
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }
  QWidget::mousePressEvent(event);
}

void JointPlotWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!dragging_) {
    QWidget::mouseMoveEvent(event);
    return;
  }
  const QRect plot = PlotRect();
  const double dx = event->pos().x() - drag_start_.x();
  const double dy = event->pos().y() - drag_start_.y();
  const double t_span = drag_t_hi_ - drag_t_lo_;
  const double v_span = drag_v_hi_ - drag_v_lo_;
  const double t_shift =
      -dx * t_span / std::max(1.0, static_cast<double>(plot.width()));
  const double v_shift =
      dy * v_span / std::max(1.0, static_cast<double>(plot.height()));
  view_t_lo_ = drag_t_lo_ + t_shift;
  view_t_hi_ = drag_t_hi_ + t_shift;
  view_v_lo_ = drag_v_lo_ + v_shift;
  view_v_hi_ = drag_v_hi_ + v_shift;
  update();
  event->accept();
}

void JointPlotWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (dragging_ && event->button() == Qt::LeftButton) {
    dragging_ = false;
    unsetCursor();
    event->accept();
    return;
  }
  QWidget::mouseReleaseEvent(event);
}

void JointPlotWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    ResetView();
    event->accept();
    return;
  }
  QWidget::mouseDoubleClickEvent(event);
}

void JointPlotWidget::SetCurves(std::vector<PlotCurve> curves,
                                const std::string& title,
                                const std::string& unit) {
  curves_ = std::move(curves);
  title_ = title;
  unit_ = unit;
  update();
}

void JointPlotWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(32, 34, 38));
  const QRect plot_rect = PlotRect();

  painter.setPen(QColor(210, 210, 210));
  painter.drawText(8, 16, QString::fromStdString(title_));
  painter.setPen(QColor(150, 150, 150));
  painter.drawText(8, 32, QString::fromStdString(unit_));

  double max_t = 1.0;
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  bool has_data = false;
  for (const PlotCurve& curve : curves_) {
    for (const PlotSeries* series : {&curve.reference, &curve.actual}) {
      for (std::size_t index = 0; index < series->times.size(); ++index) {
        has_data = true;
        max_t = std::max(max_t, series->times[index]);
        min_v = std::min(min_v, series->values[index]);
        max_v = std::max(max_v, series->values[index]);
      }
    }
  }
  has_data_ = has_data;
  if (!has_data) {
    painter.setPen(QColor(160, 160, 160));
    painter.drawText(plot_rect, Qt::AlignCenter,
                     "waiting for a planned trajectory");
    return;
  }
  if (max_v - min_v < 1.0e-6) {
    min_v -= 0.5;
    max_v += 0.5;
  } else {
    const double padding = 0.08 * (max_v - min_v);
    min_v -= padding;
    max_v += padding;
  }
  if (max_t <= 0.0) {
    max_t = 1.0;
  }

  double t_lo = 0.0;
  double t_hi = max_t;
  double v_lo = min_v;
  double v_hi = max_v;
  if (!auto_view_) {
    t_lo = view_t_lo_;
    t_hi = view_t_hi_;
    v_lo = view_v_lo_;
    v_hi = view_v_hi_;
  } else {
    // Keep the stored window in sync so the first mouse interaction starts
    // from exactly what is displayed.
    view_t_lo_ = t_lo;
    view_t_hi_ = t_hi;
    view_v_lo_ = v_lo;
    view_v_hi_ = v_hi;
  }
  if (t_hi - t_lo < 1.0e-9) {
    t_hi = t_lo + 1.0;
  }
  if (v_hi - v_lo < 1.0e-9) {
    v_hi = v_lo + 1.0;
  }
  current_t_lo_ = t_lo;
  current_t_hi_ = t_hi;
  current_v_lo_ = v_lo;
  current_v_hi_ = v_hi;

  auto to_x = [&](double t) {
    return plot_rect.left() +
           (t - t_lo) / (t_hi - t_lo) * plot_rect.width();
  };
  auto to_y = [&](double value) {
    return plot_rect.bottom() -
           (value - v_lo) / (v_hi - v_lo) * plot_rect.height();
  };
  const auto axis_decimals = [](double span) {
    if (!std::isfinite(span) || span <= 0.0) {
      return 2;
    }
    const int digits = static_cast<int>(std::ceil(-std::log10(span))) + 1;
    return std::clamp(digits, 2, 6);
  };
  const int x_decimals = axis_decimals(t_hi - t_lo);
  const int y_decimals = axis_decimals(v_hi - v_lo);

  painter.setPen(QPen(QColor(70, 72, 78), 1));
  for (int line = 0; line <= 4; ++line) {
    const int y = plot_rect.top() + line * plot_rect.height() / 4;
    painter.drawLine(plot_rect.left(), y, plot_rect.right(), y);
    const double value = v_hi - line * (v_hi - v_lo) / 4.0;
    painter.setPen(QColor(170, 170, 170));
    painter.drawText(4, y + 4,
                     QString::number(value, 'f', y_decimals));
    painter.setPen(QPen(QColor(70, 72, 78), 1));
  }
  for (int line = 0; line <= 4; ++line) {
    const int x = plot_rect.left() + line * plot_rect.width() / 4;
    painter.drawLine(x, plot_rect.top(), x, plot_rect.bottom());
    painter.setPen(QColor(170, 170, 170));
    painter.drawText(x - 12, plot_rect.bottom() + 15,
                     QString::number(t_lo + line * (t_hi - t_lo) / 4.0,
                                     'f', x_decimals));
    painter.setPen(QPen(QColor(70, 72, 78), 1));
  }

  // Colored legend for every joint, repeated on each plot.
  int legend_x = plot_rect.left();
  const int legend_y = 52;
  for (const PlotCurve& curve : curves_) {
    painter.setPen(QPen(curve.color, 3));
    painter.drawLine(legend_x, legend_y - 4, legend_x + 18, legend_y - 4);
    painter.setPen(QColor(200, 200, 200));
    const QString name = QString::fromStdString(curve.name);
    painter.drawText(legend_x + 23, legend_y, name);
    legend_x += 30 + painter.fontMetrics().horizontalAdvance(name);
  }
  painter.setPen(QColor(150, 150, 150));
  painter.drawText(QRect(rect().left(), 4, rect().width() - 8, 20),
                   Qt::AlignRight | Qt::AlignTop,
                   "solid=ref, dashed=measured");
  painter.setPen(QColor(120, 120, 120));
  painter.drawText(QRect(rect().left(), 20, rect().width() - 8, 20),
                   Qt::AlignRight | Qt::AlignTop,
                   "wheel=zoom, drag=pan, double-click=reset");

  auto draw_series = [&](const PlotSeries& series, const QColor& color,
                         Qt::PenStyle style, double width) {
    if (series.times.size() < 2) {
      return;
    }
    painter.setPen(QPen(color, width, style));
    QPainterPath path;
    path.moveTo(to_x(series.times.front()), to_y(series.values.front()));
    for (std::size_t index = 1; index < series.times.size(); ++index) {
      path.lineTo(to_x(series.times[index]), to_y(series.values[index]));
    }
    painter.drawPath(path);
  };
  painter.save();
  painter.setClipRect(plot_rect);
  for (const PlotCurve& curve : curves_) {
    draw_series(curve.reference, curve.color, Qt::SolidLine, 1.8);
    QColor measured = curve.color;
    measured.setAlpha(190);
    draw_series(curve.actual, measured, Qt::DashLine, 1.4);
  }
  painter.restore();

  // If zooming/panning moved every sample outside the visible window, make
  // that explicit instead of leaving the user with a blank chart.
  bool any_visible = false;
  for (const PlotCurve& curve : curves_) {
    for (const PlotSeries* series : {&curve.reference, &curve.actual}) {
      for (std::size_t index = 0; index < series->times.size(); ++index) {
        const double t = series->times[index];
        const double value = series->values[index];
        if (t >= t_lo && t <= t_hi && value >= v_lo && value <= v_hi) {
          any_visible = true;
          break;
        }
      }
      if (any_visible) {
        break;
      }
    }
    if (any_visible) {
      break;
    }
  }
  if (!any_visible) {
    painter.setPen(QColor(220, 190, 120));
    painter.drawText(plot_rect, Qt::AlignCenter,
                     "No data in this view\nDouble-click to reset");
  }
}

ExcavatorJointPlotPanel::ExcavatorJointPlotPanel(QWidget* parent)
    : rviz_common::Panel(parent) {
  auto* outer = new QVBoxLayout(this);
  position_plot_ = new JointPlotWidget(this);
  velocity_plot_ = new JointPlotWidget(this);
  acceleration_plot_ = new JointPlotWidget(this);
  outer->addWidget(position_plot_);
  outer->addWidget(velocity_plot_);
  outer->addWidget(acceleration_plot_);
  info_label_ = new QLabel("waiting for a planned trajectory", this);
  outer->addWidget(info_label_);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { RefreshPlot(); });
  timer_->start(100);
}

void ExcavatorJointPlotPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  trajectory_subscription_ =
      node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local(),
          [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
            HandleTrajectory(*message);
          });
  joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", rclcpp::QoS(100),
          [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
            HandleJointState(*message);
          });
  status_subscription_ =
      node_->create_subscription<excavator_interfaces::msg::ExecutionStatus>(
          "/excavator/execution_status", 10,
          [this](excavator_interfaces::msg::ExecutionStatus::ConstSharedPtr
                     message) { HandleExecutionStatus(*message); });
}

void ExcavatorJointPlotPanel::HandleTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message) {
  const tramcar::planning::JointTrajectory trajectory =
      ToCoreTrajectory(message);
  std::lock_guard<std::mutex> lock(mutex_);
  plot_data_.SetReference(trajectory);
  joint_names_ = trajectory.joint_names;
  offline_started_ = false;
  offline_start_stamp_ = 0.0;
  reset_view_pending_ = true;
  last_status_ = "reference loaded: " +
                 std::to_string(trajectory.points.size()) + " points";
}

void ExcavatorJointPlotPanel::HandleJointState(
    const sensor_msgs::msg::JointState& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!plot_data_.HasReference()) {
    return;
  }
  double sample_time = execution_elapsed_;
  if (!has_execution_status_) {
    // Offline replay has no execution-status publisher. Use the replay's
    // joint-state timestamps relative to the first sample and stop at the end
    // of the first loop.
    const double stamp = static_cast<double>(message.header.stamp.sec) +
                         static_cast<double>(message.header.stamp.nanosec) *
                             1.0e-9;
    if (!offline_started_) {
      offline_start_stamp_ = stamp;
      offline_started_ = true;
    }
    sample_time = stamp - offline_start_stamp_;
    if (sample_time < 0.0 ||
        sample_time > plot_data_.Duration() + 0.1) {
      return;
    }
  } else if (!execution_running_) {
    return;
  } else {
    // ExecutionStatus is published at 10 Hz while joint states arrive at the
    // control rate (100 Hz). Map the joint-state header stamp onto the
    // status clock instead of reusing the last elapsed_time: this gives every
    // sample a distinct, jitter-free timestamp so the measured derivative is
    // not corrupted by callback latency.
    const double stamp = ToSeconds(message.header.stamp);
    const double since_receipt =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - status_receipt_time_)
            .count();
    const double offset = (stamp > 0.0 && status_stamp_seconds_ > 0.0)
                              ? stamp - status_stamp_seconds_
                              : std::clamp(since_receipt, 0.0, 0.2);
    sample_time = execution_elapsed_ + std::clamp(offset, -0.05, 0.5);
    if (sample_time > plot_data_.Duration() + 0.15) {
      return;
    }
  }
  tramcar::planning::RobotState state;
  state.positions.reserve(joint_names_.size());
  state.velocities.reserve(joint_names_.size());
  for (const std::string& name : joint_names_) {
    const auto found = std::find(message.name.begin(), message.name.end(), name);
    if (found == message.name.end()) {
      return;
    }
    const std::size_t index =
        static_cast<std::size_t>(found - message.name.begin());
    if (index >= message.position.size() || index >= message.velocity.size()) {
      return;
    }
    state.positions.push_back(message.position[index]);
    state.velocities.push_back(message.velocity[index]);
  }
  plot_data_.AddActual(sample_time, state);
}

void ExcavatorJointPlotPanel::HandleExecutionStatus(
    const excavator_interfaces::msg::ExecutionStatus& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_execution_status_) {
    // Drop any fallback samples that were anchored to an arbitrary first
    // stamp before the true trajectory clock became available.
    if (plot_data_.ActualSampleCount() > 0) {
      plot_data_.ClearActual();
    }
    has_execution_status_ = true;
  } else if (message.elapsed_time + 0.2 < execution_elapsed_) {
    // The clock jumped backwards: a new execution started or an offline
    // replay loop wrapped. Start a fresh measured curve instead of mixing
    // two runs (AddActual would otherwise reject all new samples).
    plot_data_.ClearActual();
  }
  execution_running_ =
      message.state == excavator_interfaces::msg::ExecutionStatus::RUNNING;
  execution_elapsed_ = message.elapsed_time;
  status_receipt_time_ = std::chrono::steady_clock::now();
  status_stamp_seconds_ = ToSeconds(message.header.stamp);
  last_status_ = message.message;
}

void ExcavatorJointPlotPanel::RefreshPlot() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (reset_view_pending_) {
    position_plot_->ResetView();
    velocity_plot_->ResetView();
    acceleration_plot_->ResetView();
    reset_view_pending_ = false;
  }
  if (!plot_data_.HasReference()) {
    position_plot_->SetCurves({}, "Position q(t)", "rad");
    velocity_plot_->SetCurves({}, "Velocity dq(t)", "rad/s");
    acceleration_plot_->SetCurves({}, "Acceleration ddq(t)",
                                  "rad/s^2 (measured: dv/dt)");
    info_label_->setText(QString::fromStdString(last_status_));
    return;
  }

  const std::vector<std::string> names = plot_data_.JointNames();
  const double duration = plot_data_.Duration();
  const std::size_t actual_samples = plot_data_.ActualSampleCount();
  const QColor fallback(160, 160, 160);

  const auto build_curves = [&](PlotQuantity quantity) {
    std::vector<PlotCurve> curves;
    curves.reserve(names.size());
    for (std::size_t joint = 0; joint < names.size(); ++joint) {
      PlotCurve curve;
      curve.name = names[joint];
      curve.color = joint < JointColors().size()
                        ? JointColors()[joint]
                        : fallback;
      curve.reference =
          plot_data_.ReferenceSeries(joint, quantity, 400);
      curve.actual = plot_data_.ActualSeries(joint, quantity);
      if (!curve.actual.times.empty()) {
        curve.reference = plot_data_.AlignedReferenceSeries(
            curve.actual, joint, quantity);
      }
      curves.push_back(std::move(curve));
    }
    return curves;
  };

  position_plot_->SetCurves(build_curves(PlotQuantity::kPosition),
                            "Position q(t)", "rad");
  velocity_plot_->SetCurves(build_curves(PlotQuantity::kVelocity),
                            "Velocity dq(t)", "rad/s");
  acceleration_plot_->SetCurves(
      build_curves(PlotQuantity::kAcceleration), "Acceleration ddq(t)",
      "rad/s^2 (measured: dv/dt)");

  char info[256];
  std::snprintf(info, sizeof(info),
                "duration %.2f s | measured samples %zu | %s", duration,
                actual_samples, last_status_.c_str());
  info_label_->setText(QString::fromUtf8(info));
}

}  // namespace excavator_visualization

PLUGINLIB_EXPORT_CLASS(excavator_visualization::ExcavatorJointPlotPanel,
                       rviz_common::Panel)
