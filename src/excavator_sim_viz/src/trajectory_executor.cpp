#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/action/execute_trajectory.hpp"
#include "excavator_interfaces/msg/execution_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace excavator_sim_viz {
namespace {

using ExecuteAction = excavator_interfaces::action::ExecuteTrajectory;
using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteAction>;
using ExecutionStatusMessage = excavator_interfaces::msg::ExecutionStatus;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
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

}  // namespace

class TrajectoryExecutor final : public rclcpp::Node {
 public:
  TrajectoryExecutor() : Node("excavator_trajectory_executor") {
    trajectory_publisher_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory", 10);
    cancel_publisher_ =
        create_publisher<std_msgs::msg::Empty>("/cancel_trajectory", 10);
    status_subscription_ = create_subscription<ExecutionStatusMessage>(
        "/execution_status", 10,
        [this](ExecutionStatusMessage::ConstSharedPtr message) {
          HandleStatus(std::move(message));
        });
    action_server_ = rclcpp_action::create_server<ExecuteAction>(
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
  }

 private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const ExecuteAction::Goal> goal) {
    if (!goal->require_confirmation) {
      RCLCPP_WARN(get_logger(),
                  "Rejecting execution: require_confirmation must be true");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto trajectory = ToCoreTrajectory(goal->trajectory);
    const auto validation =
        tramcar::planning::ValidateJointTrajectory(trajectory);
    if (!validation.ok ||
        trajectory.joint_names !=
            std::vector<std::string>{"swing_joint", "boom_joint", "stick_joint",
                                     "bucket_joint"}) {
      RCLCPP_WARN(get_logger(), "Rejecting invalid execution trajectory: %s",
                  validation.message.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return active_goal_ ? rclcpp_action::GoalResponse::REJECT
                        : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleExecute>) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_requested_ = true;
    }
    cancel_publisher_->publish(std_msgs::msg::Empty());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(const std::shared_ptr<GoalHandleExecute> goal_handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_ = goal_handle;
      cancel_requested_ = false;
    }
    trajectory_publisher_->publish(goal_handle->get_goal()->trajectory);
  }

  void HandleStatus(ExecutionStatusMessage::ConstSharedPtr message) {
    std::shared_ptr<GoalHandleExecute> goal_handle;
    bool cancel_requested = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle = active_goal_;
      cancel_requested = cancel_requested_;
      if (!goal_handle) {
        return;
      }
      if (message->state == ExecutionStatusMessage::SUCCEEDED ||
          message->state == ExecutionStatusMessage::CANCELED ||
          message->state == ExecutionStatusMessage::REJECTED ||
          message->state == ExecutionStatusMessage::FAILED) {
        active_goal_.reset();
        cancel_requested_ = false;
      }
    }

    if (message->state == ExecutionStatusMessage::RUNNING) {
      auto feedback = std::make_shared<ExecuteAction::Feedback>();
      feedback->state = message->state;
      const double final_time =
          ToSeconds(goal_handle->get_goal()->trajectory.points.back()
                        .time_from_start);
      feedback->progress =
          final_time > 0.0
              ? static_cast<float>(std::clamp(message->elapsed_time / final_time,
                                              0.0, 1.0))
              : 1.0F;
      feedback->message = message->message;
      goal_handle->publish_feedback(feedback);
      return;
    }

    if (message->state == ExecutionStatusMessage::SUCCEEDED) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = true;
      result->message = message->message;
      goal_handle->succeed(result);
    } else if (message->state == ExecutionStatusMessage::CANCELED &&
               cancel_requested) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = message->message;
      goal_handle->canceled(result);
    } else if (message->state == ExecutionStatusMessage::REJECTED ||
               message->state == ExecutionStatusMessage::FAILED ||
               message->state == ExecutionStatusMessage::CANCELED) {
      auto result = std::make_shared<ExecuteAction::Result>();
      result->success = false;
      result->message = message->message;
      goal_handle->abort(result);
    }
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cancel_publisher_;
  rclcpp::Subscription<ExecutionStatusMessage>::SharedPtr status_subscription_;
  rclcpp_action::Server<ExecuteAction>::SharedPtr action_server_;
  std::mutex mutex_;
  std::shared_ptr<GoalHandleExecute> active_goal_;
  bool cancel_requested_{false};
};

}  // namespace excavator_sim_viz

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_sim_viz::TrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
