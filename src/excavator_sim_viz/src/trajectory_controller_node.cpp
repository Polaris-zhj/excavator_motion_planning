#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_collision/collision_checker.hpp"
#include "excavator_common/joint_trajectory.hpp"
#include "excavator_common/robot_state.hpp"
#include "excavator_interfaces/action/execute_trajectory.hpp"
#include "excavator_interfaces/msg/execution_status.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "excavator_interfaces/msg/trajectory_update.hpp"
#include "excavator_sim_viz/kinematic_simulator.hpp"
#include "excavator_sim_viz/trajectory_controller.hpp"
#include "excavator_sim_viz/velocity_tracking_controller.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_sim_viz {
namespace {

using tramcar::planning::ControlCommand;
using tramcar::planning::ExecutionState;
using tramcar::planning::ExecutionStatusSnapshot;
using tramcar::planning::JointPositions;
using tramcar::planning::JointTrajectory;
using tramcar::planning::JointTrajectoryPoint;
using tramcar::planning::KinematicSimulator;
using tramcar::planning::RobotState;
using tramcar::planning::SimulationState;
using tramcar::planning::TrajectoryController;
using tramcar::planning::TrajectoryControllerConfig;
using tramcar::planning::TrajectoryStepResult;
using tramcar::planning::ValidationResult;
using tramcar::planning::VelocityControllerConfig;
using tramcar::planning::VelocityTrackingController;

using Clock = std::chrono::steady_clock;
using ExecuteAction = excavator_interfaces::action::ExecuteTrajectory;
using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteAction>;
using ExecutionStatusMessage = excavator_interfaces::msg::ExecutionStatus;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

JointTrajectory ToCoreTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points) {
  JointTrajectory trajectory;
  trajectory.joint_names = joint_names;
  trajectory.points.reserve(points.size());
  for (const auto& message_point : points) {
    JointTrajectoryPoint point;
    point.t = ToSeconds(message_point.time_from_start);
    point.positions = message_point.positions;
    point.velocities = message_point.velocities;
    point.accelerations = message_point.accelerations;
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

std::uint8_t ToMessageState(ExecutionState state) {
  return static_cast<std::uint8_t>(state);
}

}  // namespace

class TrajectoryControllerNode final : public rclcpp::Node {
 public:
  TrajectoryControllerNode()
      : Node("excavator_trajectory_controller"),
        joint_names_(declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"})),
        initial_positions_(declare_parameter<std::vector<double>>(
            "initial_positions", {0.0, -0.35, 0.45, -0.4})),
        kinematics_(joint_names_) {
    const double control_rate_hz =
        declare_parameter<double>("control_rate_hz", 100.0);
    const double position_gain =
        declare_parameter<double>("position_gain", 4.0);
    max_tracking_error_ = declare_parameter<double>("max_tracking_error", 0.5);
    completion_tolerance_ =
        declare_parameter<double>("completion_position_tolerance", 0.02);
    execution_timeout_margin_ =
        declare_parameter<double>("execution_timeout_margin", 3.0);
    start_continuity_tolerance_ =
        declare_parameter<double>("start_continuity_tolerance", 0.05);
    update_position_tolerance_ =
        declare_parameter<double>("update_position_continuity_tolerance", 0.05);
    update_velocity_tolerance_ =
        declare_parameter<double>("update_velocity_continuity_tolerance", 1.0);
    max_collision_joint_step_ =
        declare_parameter<double>("max_collision_joint_step", 0.03);
    require_collision_check_ =
        declare_parameter<bool>("require_collision_check", true);
    allow_simulator_initialization_ =
        declare_parameter<bool>("allow_simulator_initialization", true);
    std::vector<double> default_acceleration =
        declare_parameter<std::vector<double>>(
            "max_acceleration", {1.0, 0.6, 0.8, 1.0});
    // Non-planning URDF joints are published with fixed display values so
    // robot_state_publisher has a complete TF tree for every link.
    passive_joint_names_ = declare_parameter<std::vector<std::string>>(
        "passive_joint_names",
        {"boom_cyl_l_body_joint", "boom_cyl_l_rod_joint",
         "boom_cyl_r_body_joint", "boom_cyl_r_rod_joint",
         "stick_cyl_body_joint", "stick_cyl_rod_joint",
         "bucket_cyl_body_joint", "bucket_cyl_rod_joint",
         "bucket_linkage_joint", "stick_rocker_l_joint",
         "stick_rocker_r_joint"});
    const std::vector<std::string> allowed_collision_pairs =
        declare_parameter<std::vector<std::string>>(
            "allowed_collision_pairs",
            {"base_link:boom_link", "base_link:stick_link",
             "base_link:bucket_link", "swing_link:stick_link",
             "swing_link:bucket_link"});
    passive_joint_positions_ = declare_parameter<std::vector<double>>(
        "passive_joint_positions",
        {-0.30301, -0.23228, -0.30301, -0.23228, 0.04501, -0.36905,
         -0.08259, 0.11620, -0.40, -0.40, -0.40});
    const std::string robot_description =
        declare_parameter<std::string>("robot_description", "");

    if (!std::isfinite(control_rate_hz) || control_rate_hz <= 0.0) {
      throw std::invalid_argument(
          "control_rate_hz must be finite and positive");
    }
    if (passive_joint_positions_.size() != passive_joint_names_.size()) {
      throw std::invalid_argument(
          "passive_joint_positions must match passive_joint_names");
    }
    if (!std::isfinite(max_collision_joint_step_) ||
        max_collision_joint_step_ <= 0.0) {
      throw std::invalid_argument(
          "max_collision_joint_step must be finite and positive");
    }

    std::vector<std::pair<double, double>> position_limits;
    JointPositions max_velocity(joint_names_.size(), 1.0);
    if (!robot_description.empty()) {
      const ValidationResult model_loaded =
          kinematics_.InitializeFromString(robot_description);
      if (!model_loaded.ok) {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model: %s",
                     model_loaded.message.c_str());
      } else {
        position_limits = kinematics_.GetPositionLimits();
        max_velocity = kinematics_.GetVelocityLimits();
        const ValidationResult collision_initialized =
            collision_checker_.Initialize(robot_description, joint_names_);
        if (!collision_initialized.ok) {
          RCLCPP_ERROR(get_logger(), "Failed to initialize collision checker: %s",
                       collision_initialized.message.c_str());
        } else {
          std::vector<std::pair<std::string, std::string>> allowed_pairs;
          for (const std::string& entry : allowed_collision_pairs) {
            const std::size_t separator = entry.find(':');
            if (separator == std::string::npos || separator == 0 ||
                separator + 1 == entry.size()) {
              RCLCPP_WARN(get_logger(),
                          "Ignoring malformed allowed_collision_pairs entry: %s",
                          entry.c_str());
              continue;
            }
            allowed_pairs.emplace_back(entry.substr(0, separator),
                                       entry.substr(separator + 1));
          }
          collision_checker_.SetAllowedCollisionPairs(allowed_pairs);
          collision_ready_ = true;
        }
      }
    }
    if (position_limits.size() != joint_names_.size() ||
        max_velocity.size() != joint_names_.size()) {
      RCLCPP_WARN(get_logger(),
                  "robot_description limits unavailable; using parameter "
                  "fallback limits");
      position_limits.assign(joint_names_.size(),
                             {-3.141592653589793, 3.141592653589793});
      max_velocity.assign(joint_names_.size(), 0.5);
    }
    if (default_acceleration.size() != joint_names_.size()) {
      RCLCPP_WARN(get_logger(),
                  "max_acceleration size mismatch; using defaults");
      default_acceleration.assign(joint_names_.size(), 1.0);
    }
    if (initial_positions_.size() != joint_names_.size()) {
      throw std::invalid_argument(
          "initial_positions must match the configured joint count");
    }

    VelocityControllerConfig velocity_config;
    velocity_config.max_velocity = max_velocity;
    velocity_config.max_acceleration = default_acceleration;
    for (const auto& limit : position_limits) {
      velocity_config.lower_position_limits.push_back(limit.first);
      velocity_config.upper_position_limits.push_back(limit.second);
    }
    velocity_config.position_gain = position_gain;

    TrajectoryControllerConfig controller_config;
    controller_config.joint_names = joint_names_;
    controller_config.max_velocity = velocity_config.max_velocity;
    controller_config.max_acceleration = velocity_config.max_acceleration;
    controller_config.lower_position_limits =
        velocity_config.lower_position_limits;
    controller_config.upper_position_limits =
        velocity_config.upper_position_limits;
    controller_config.max_tracking_error = max_tracking_error_;
    controller_config.completion_position_tolerance = completion_tolerance_;
    controller_config.execution_timeout_margin = execution_timeout_margin_;
    controller_config.start_continuity_tolerance = start_continuity_tolerance_;
    controller_config.update_position_continuity_tolerance =
        update_position_tolerance_;
    controller_config.update_velocity_continuity_tolerance =
        update_velocity_tolerance_;
    controller_ = std::make_unique<TrajectoryController>(
        std::move(controller_config),
        std::make_unique<VelocityTrackingController>(velocity_config));

    tramcar::planning::SimulatorLimits simulator_limits;
    simulator_limits.max_velocity = velocity_config.max_velocity;
    simulator_limits.max_acceleration = velocity_config.max_acceleration;
    simulator_limits.lower_position_limits =
        velocity_config.lower_position_limits;
    simulator_limits.upper_position_limits =
        velocity_config.upper_position_limits;
    simulator_ = std::make_unique<KinematicSimulator>(
        joint_names_, initial_positions_, std::move(simulator_limits));

    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    status_publisher_ = create_publisher<ExecutionStatusMessage>(
        "/excavator/execution_status", 10);
    legacy_status_publisher_ =
        create_publisher<ExecutionStatusMessage>("/execution_status", 10);

    trajectory_subscription_ =
        create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory", 10,
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
              HandleFullTrajectory(std::move(msg));
            });
    update_subscription_ =
        create_subscription<excavator_interfaces::msg::TrajectoryUpdate>(
            "/excavator/trajectory_update", 10,
            [this](
                excavator_interfaces::msg::TrajectoryUpdate::ConstSharedPtr msg) {
              HandleTrajectoryUpdate(std::move(msg));
            });
    scene_subscription_ =
        create_subscription<excavator_interfaces::msg::PlanningScene>(
            "/excavator/scene", 10,
            [this](
                excavator_interfaces::msg::PlanningScene::ConstSharedPtr msg) {
              HandleScene(std::move(msg));
            });
    cancel_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/cancel_trajectory", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) { HandleCancelTopic(); });
    pause_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/excavator/pause", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          const ValidationResult result = controller_->Pause();
          PublishStatus(true);
          RCLCPP_INFO(get_logger(), "Pause request: %s", result.message.c_str());
        });
    resume_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/excavator/resume", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          const ValidationResult result = controller_->Resume();
          PublishStatus(true);
          RCLCPP_INFO(get_logger(), "Resume request: %s", result.message.c_str());
        });
    stop_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/excavator/stop", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          const ValidationResult result = controller_->Stop();
          PublishStatus(true);
          RCLCPP_INFO(get_logger(), "Stop request: %s", result.message.c_str());
        });

    action_server_ = rclcpp_action::create_server<ExecuteAction>(
        this, "/excavator/execute_trajectory",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const ExecuteAction::Goal> goal) {
          return HandleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandleExecute> goal_handle) {
          return HandleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleExecute> goal_handle) {
          HandleAccepted(goal_handle);
        });
    legacy_action_server_ = rclcpp_action::create_server<ExecuteAction>(
        this, "/execute_trajectory",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const ExecuteAction::Goal> goal) {
          return HandleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandleExecute> goal_handle) {
          return HandleCancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleExecute> goal_handle) {
          HandleAccepted(goal_handle);
        });

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { ControlLoop(); });
    last_cycle_ = Clock::now();
    PublishStatus(true);
    RCLCPP_INFO(get_logger(),
                "Trajectory controller ready at %.1f Hz; executing through "
                "/excavator/execute_trajectory",
                control_rate_hz);
  }

 private:
  RobotState CurrentState() const {
    RobotState state;
    state.positions = simulator_->positions();
    state.velocities = simulator_->velocities();
    return state;
  }

  void HandleFullTrajectory(
      trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
    const JointTrajectory trajectory =
        ToCoreTrajectory(message->joint_names, message->points);
    const ValidationResult result =
        controller_->SubmitTrajectory(trajectory, CurrentState());
    if (!result.ok) {
      RCLCPP_WARN(get_logger(), "Trajectory submission rejected: %s",
                  result.message.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
                  "Trajectory accepted as candidate; waiting for execution "
                  "confirmation");
    }
    PublishStatus(true);
  }

  void HandleTrajectoryUpdate(
      excavator_interfaces::msg::TrajectoryUpdate::ConstSharedPtr message) {
    if (message->update_mode !=
        excavator_interfaces::msg::TrajectoryUpdate::REPLACE) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting TrajectoryUpdate: unknown update_mode %u",
                  static_cast<unsigned int>(message->update_mode));
      return;
    }
    const JointTrajectory trajectory =
        ToCoreTrajectory(message->joint_names, message->points);
    if (collision_ready_) {
      excavator_interfaces::msg::PlanningScene scene;
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        if (has_scene_) {
          scene = scene_;
        } else {
          scene.header.frame_id.clear();
        }
      }
      const auto collision = collision_checker_.CheckTrajectory(
          trajectory, scene, max_collision_joint_step_);
      if (!collision.valid || collision.collision) {
        RCLCPP_WARN(get_logger(), "Trajectory update rejected by collision "
                                  "validation: %s",
                    collision.message.c_str());
        PublishStatus(true);
        return;
      }
    }
    const ValidationResult result =
        controller_->UpdateTrajectory(trajectory, CurrentState());
    if (!result.ok) {
      RCLCPP_WARN(get_logger(), "Trajectory update rejected: %s",
                  result.message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Trajectory update accepted: %s",
                  result.message.c_str());
    }
    PublishStatus(true);
  }

  void HandleScene(
      excavator_interfaces::msg::PlanningScene::ConstSharedPtr message) {
    if (message->is_diff) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting incremental planning scene; full snapshots only");
      return;
    }
    std::lock_guard<std::mutex> lock(scene_mutex_);
    scene_ = *message;
    has_scene_ = true;
  }

  void HandleCancelTopic() {
    const ValidationResult result = controller_->Cancel();
    PublishStatus(true);
    RCLCPP_INFO(get_logger(), "Cancel request: %s", result.message.c_str());
  }

  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const ExecuteAction::Goal> goal) {
    if (!goal->require_confirmation) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting execution: require_confirmation must be true");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const JointTrajectory trajectory =
        ToCoreTrajectory(goal->trajectory.joint_names, goal->trajectory.points);
    const ValidationResult validation =
        tramcar::planning::ValidateJointTrajectory(trajectory);
    if (!validation.ok || trajectory.joint_names != joint_names_) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid execution trajectory: %s",
                  validation.ok ? "joint order mismatch"
                                : validation.message.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting execution: another execution goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleExecute>) {
    const ValidationResult result = controller_->Cancel();
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      cancel_requested_ = true;
    }
    RCLCPP_INFO(get_logger(), "Action cancel request: %s", result.message.c_str());
    PublishStatus(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleExecute> goal_handle) {
    const auto goal = goal_handle->get_goal();
    const JointTrajectory trajectory =
        ToCoreTrajectory(goal->trajectory.joint_names, goal->trajectory.points);

    std::string failure;
    if (goal->initialize_to_start) {
      if (!allow_simulator_initialization_) {
        failure =
            "simulator initialization is disabled on this controller";
      } else if (trajectory.points.empty()) {
        failure = "cannot initialize the simulator from an empty trajectory";
      } else {
        RobotState initial_state;
        initial_state.positions = trajectory.points.front().positions;
        initial_state.velocities = trajectory.points.front().velocities;
        const ValidationResult reset =
            simulator_->SetState(initial_state);
        if (!reset.ok) {
          failure =
              "failed to initialize the simulator to the trajectory start: " +
              reset.message;
        } else {
          RCLCPP_WARN(get_logger(),
                      "Simulation-only initialization: moved the kinematic "
                      "simulator to the trajectory start");
        }
      }
    }
    if (!failure.empty()) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = failure;
      RCLCPP_ERROR(get_logger(), "%s", failure.c_str());
      goal_handle->abort(result);
      PublishStatus(true);
      return;
    }
    if (!collision_ready_) {
      if (require_collision_check_) {
        failure =
            "collision checker is not ready; execution requires final "
            "collision validation";
      }
    } else {
      excavator_interfaces::msg::PlanningScene scene;
      {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        if (has_scene_) {
          scene = scene_;
        } else {
          scene.header.frame_id.clear();
        }
      }
      const auto collision = collision_checker_.CheckTrajectory(
          trajectory, scene, max_collision_joint_step_);
      if (!collision.valid || collision.collision) {
        failure = "final trajectory collision validation failed: " +
                  collision.message;
      }
    }

    if (!failure.empty()) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = failure;
      RCLCPP_ERROR(get_logger(), "%s", failure.c_str());
      goal_handle->abort(result);
      return;
    }

    const ValidationResult execution =
        controller_->ExecuteTrajectory(trajectory, CurrentState());
    if (!execution.ok) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = "execution rejected: " + execution.message;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      PublishStatus(true);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_ = goal_handle;
      cancel_requested_ = false;
      last_feedback_state_ = ExecutionState::kIdle;
    }
    last_cycle_ = Clock::now();
    RCLCPP_INFO(get_logger(),
                "Execution started after final validation; %zu points",
                trajectory.points.size());
    PublishStatus(true);
  }

  void ControlLoop() {
    const auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last_cycle_).count();
    last_cycle_ = now;
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 0.001;
    }
    dt = std::clamp(dt, 1.0e-4, 0.05);

    const RobotState current = CurrentState();
    const TrajectoryStepResult step = controller_->Update(current, dt);
    if (step.has_command) {
      SimulationState simulation_state;
      const ValidationResult applied =
          simulator_->ApplyCommand(step.command, dt, &simulation_state);
      if (!applied.ok) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "Simulator rejected command: %s",
                              applied.message.c_str());
        if (controller_->GetStatus().state == ExecutionState::kRunning) {
          controller_->Cancel();
        }
      }
    }
    PublishJointState();
    PublishStatus(false);
    UpdateActionGoal();
  }

  void PublishJointState() {
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = joint_names_;
    state.position = simulator_->positions();
    state.velocity = simulator_->velocities();
    for (std::size_t index = 0; index < passive_joint_names_.size();
         ++index) {
      const std::string& joint_name = passive_joint_names_[index];
      if (std::find(state.name.begin(), state.name.end(), joint_name) ==
          state.name.end()) {
        state.name.push_back(joint_name);
        state.position.push_back(passive_joint_positions_[index]);
        state.velocity.push_back(0.0);
      }
    }
    joint_state_publisher_->publish(state);
  }

  void PublishStatus(bool force) {
    const ExecutionStatusSnapshot snapshot = controller_->GetStatus();
    const auto now_time = Clock::now();
    const double since_status =
        std::chrono::duration<double>(now_time - last_status_time_).count();
    const bool active_state = snapshot.state == ExecutionState::kRunning ||
                              snapshot.state == ExecutionState::kPaused;
    const double min_period = active_state ? 0.1 : 1.0;
    if (!force && snapshot.state == last_status_state_ &&
        since_status < min_period) {
      return;
    }
    last_status_time_ = now_time;
    last_status_state_ = snapshot.state;
    ExecutionStatusMessage message;
    message.header.stamp = now();
    message.state = ToMessageState(snapshot.state);
    message.message = snapshot.message;
    message.elapsed_time = snapshot.elapsed_time;
    message.duration = snapshot.duration;
    message.progress = snapshot.progress;
    message.tracking_error_norm = snapshot.tracking_error_norm;
    status_publisher_->publish(message);
    legacy_status_publisher_->publish(message);
  }

  void UpdateActionGoal() {
    std::shared_ptr<GoalHandleExecute> goal_handle;
    bool cancel_requested = false;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_handle = active_goal_;
      cancel_requested = cancel_requested_;
    }
    if (!goal_handle) {
      return;
    }

    const ExecutionStatusSnapshot snapshot = controller_->GetStatus();
    if (snapshot.state == ExecutionState::kRunning ||
        snapshot.state == ExecutionState::kPaused ||
        snapshot.state == ExecutionState::kReady ||
        snapshot.state == ExecutionState::kValidating) {
      const auto now_time = Clock::now();
      const double since_feedback =
          std::chrono::duration<double>(now_time - last_feedback_time_)
              .count();
      if (snapshot.state != last_feedback_state_ || since_feedback >= 0.1) {
        last_feedback_state_ = snapshot.state;
        last_feedback_time_ = now_time;
        auto feedback = std::make_shared<ExecuteAction::Feedback>();
        feedback->state = ToMessageState(snapshot.state);
        feedback->progress = static_cast<float>(snapshot.progress);
        feedback->message = snapshot.message;
        goal_handle->publish_feedback(feedback);
      }
      return;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_.reset();
      cancel_requested_ = cancel_requested_;
    }

    if (snapshot.state == ExecutionState::kCompleted) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = true;
      result->message = snapshot.message;
      goal_handle->succeed(result);
    } else if (snapshot.state == ExecutionState::kCancelled) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = snapshot.message;
      if (cancel_requested) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } else if (snapshot.state == ExecutionState::kSafetyStop ||
               snapshot.state == ExecutionState::kError ||
               snapshot.state == ExecutionState::kIdle) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = snapshot.state == ExecutionState::kIdle
                            ? "execution stopped immediately"
                            : snapshot.message;
      goal_handle->abort(result);
    }
  }

  std::vector<std::string> joint_names_;
  std::vector<double> initial_positions_;
  tramcar::planning::UrdfKinematicsModel kinematics_;
  tramcar::planning::UrdfFclCollisionChecker collision_checker_;
  std::unique_ptr<TrajectoryController> controller_;
  std::unique_ptr<KinematicSimulator> simulator_;
  bool collision_ready_{false};
  bool require_collision_check_{true};
  bool allow_simulator_initialization_{true};
  double max_collision_joint_step_{0.03};
  double max_tracking_error_{0.5};
  double completion_tolerance_{0.02};
  double execution_timeout_margin_{3.0};
  double start_continuity_tolerance_{0.05};
  double update_position_tolerance_{0.05};
  double update_velocity_tolerance_{1.0};
  std::vector<std::string> passive_joint_names_;
  std::vector<double> passive_joint_positions_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<ExecutionStatusMessage>::SharedPtr status_publisher_;
  rclcpp::Publisher<ExecutionStatusMessage>::SharedPtr legacy_status_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<excavator_interfaces::msg::TrajectoryUpdate>::SharedPtr
      update_subscription_;
  rclcpp::Subscription<excavator_interfaces::msg::PlanningScene>::SharedPtr
      scene_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pause_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
  rclcpp_action::Server<ExecuteAction>::SharedPtr action_server_;
  rclcpp_action::Server<ExecuteAction>::SharedPtr legacy_action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  Clock::time_point last_cycle_{};
  Clock::time_point last_status_time_{};
  ExecutionState last_status_state_{ExecutionState::kIdle};

  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleExecute> active_goal_;
  bool cancel_requested_{false};
  ExecutionState last_feedback_state_{ExecutionState::kIdle};
  Clock::time_point last_feedback_time_{};

  std::mutex scene_mutex_;
  excavator_interfaces::msg::PlanningScene scene_;
  bool has_scene_{false};
};

}  // namespace excavator_sim_viz

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_sim_viz::TrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
