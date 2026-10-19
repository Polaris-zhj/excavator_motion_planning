#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace excavator_visualization {
namespace {

const char kTargetMarkerName[] = "bucket_tip_target";
const char kStartMarkerName[] = "bucket_tip_start";
// The start handle is displayed beside the current tip until the user drags
// it, so the two interactive balls do not overlap and the handle is not hidden
// inside the bucket/stick geometry.
constexpr double kStartHandleOffsetY = 1.0;

geometry_msgs::msg::Pose ToRosPose(const tramcar::planning::Pose3& pose) {
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x;
  message.position.y = pose.position.y;
  message.position.z = pose.position.z;
  message.orientation.x = pose.orientation.x;
  message.orientation.y = pose.orientation.y;
  message.orientation.z = pose.orientation.z;
  message.orientation.w = pose.orientation.w;
  return message;
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

// Publishes two 6-DoF RViz Interactive Markers:
//  - a green target ball at the bucket tip;
//  - a blue start handle above it.
// Dragging either ball freely publishes /excavator/target_pose or
// /excavator/start_pose. The node never drives the controller.
class TargetMarkerNode final : public rclcpp::Node {
 public:
  TargetMarkerNode()
      : Node("excavator_target_marker"),
        joint_names_(declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"})),
        kinematics_(joint_names_) {
    const std::string robot_description =
        declare_parameter<std::string>("robot_description", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "bucket_tip");

    if (!robot_description.empty()) {
      const auto loaded = kinematics_.InitializeFromString(robot_description);
      if (loaded.ok) {
        kinematics_ready_ = true;
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model: %s",
                     loaded.message.c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(),
                   "robot_description is empty; target marker needs FK");
    }

    target_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
            "/excavator/target_pose", rclcpp::QoS(1).transient_local());
    start_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
            "/excavator/start_pose", rclcpp::QoS(1).transient_local());
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "excavator_target", this);

    tramcar::planning::Pose3 target_initial;
    target_initial.position.x = 3.5;
    target_initial.position.z = 1.0;
    CreateMarker(kTargetMarkerName,
                 "Drag to set the bucket_tip target pose", target_initial,
                 0.1F, 0.9F, 0.4F, false);
    tramcar::planning::Pose3 start_initial = target_initial;
    start_initial.position.y += kStartHandleOffsetY;
    CreateMarker(kStartMarkerName,
                 "Drag to set the planning start pose", start_initial, 0.2F,
                 0.5F, 1.0F, true);
    server_->applyChanges();

    joint_state_subscription_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
              HandleJointState(std::move(message));
            });
    RCLCPP_INFO(get_logger(),
                "Interactive start/target markers ready on "
                "/excavator_target/update");
  }

 private:
  void CreateMarker(const std::string& name, const std::string& description,
                    const tramcar::planning::Pose3& initial_pose, float red,
                    float green, float blue, bool is_start) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.name = name;
    marker.description = description;
    marker.pose = ToRosPose(initial_pose);
    marker.scale = 0.60;

    visualization_msgs::msg::Marker visual;
    visual.type = visualization_msgs::msg::Marker::SPHERE;
    visual.scale.x = 0.16;
    visual.scale.y = 0.16;
    visual.scale.z = 0.16;
    visual.color.r = red;
    visual.color.g = green;
    visual.color.b = blue;
    visual.color.a = 0.90F;

    // The visible ball belongs to the free-move control so the user can drag
    // it in any direction instead of only along the fixed axes.
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

    server_->insert(
        marker,
        [this, is_start](
            const visualization_msgs::msg::InteractiveMarkerFeedback::
                ConstSharedPtr& feedback) { HandleFeedback(feedback, is_start); });
  }

  void HandleFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback,
      bool is_start) {
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      return;
    }
    if (is_start) {
      user_moved_start_ = true;
    } else {
      user_moved_target_ = true;
    }
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = frame_id_;
    message.header.stamp = now();
    message.pose = feedback->pose;
    if (is_start) {
      start_publisher_->publish(message);
    } else {
      target_publisher_->publish(message);
    }
  }

  void HandleJointState(sensor_msgs::msg::JointState::ConstSharedPtr message) {
    if (!kinematics_ready_ ||
        (user_moved_start_ && user_moved_target_)) {
      return;
    }
    tramcar::planning::JointPositions positions;
    positions.reserve(joint_names_.size());
    for (const std::string& name : joint_names_) {
      const auto found =
          std::find(message->name.begin(), message->name.end(), name);
      if (found == message->name.end()) {
        return;
      }
      const std::size_t index =
          static_cast<std::size_t>(found - message->name.begin());
      if (index >= message->position.size()) {
        return;
      }
      positions.push_back(message->position[index]);
    }
    tramcar::planning::Pose3 pose;
    if (!kinematics_.GetLinkPose(positions, tip_link_, &pose).ok) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    const double since_update =
        std::chrono::duration<double>(now_time - last_marker_update_).count();
    const double dx = pose.position.x - last_pose_.position.x;
    const double dy = pose.position.y - last_pose_.position.y;
    const double dz = pose.position.z - last_pose_.position.z;
    const double position_delta = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double quaternion_dot =
        pose.orientation.w * last_pose_.orientation.w +
        pose.orientation.x * last_pose_.orientation.x +
        pose.orientation.y * last_pose_.orientation.y +
        pose.orientation.z * last_pose_.orientation.z;
    const double orientation_delta =
        2.0 * std::acos(std::min(1.0, std::abs(quaternion_dot)));
    // Update at most at 10 Hz while the tip moves, and resend once per second
    // so a late-joining RViz display receives the markers. Flooding the update
    // topic makes RViz report out-of-order updates.
    const bool moved =
        position_delta > 1.0e-3 || orientation_delta > 1.0e-3;
    if ((moved && since_update < 0.1) || (!moved && since_update < 1.0)) {
      return;
    }
    last_pose_ = pose;
    last_marker_update_ = now_time;
    if (!user_moved_target_) {
      server_->setPose(kTargetMarkerName, ToRosPose(pose));
    }
    if (!user_moved_start_) {
      tramcar::planning::Pose3 handle = pose;
      handle.position.y += kStartHandleOffsetY;
      server_->setPose(kStartMarkerName, ToRosPose(handle));
    }
    server_->applyChanges();
  }

  std::vector<std::string> joint_names_;
  tramcar::planning::UrdfKinematicsModel kinematics_;
  std::string frame_id_;
  std::string tip_link_;
  bool kinematics_ready_{false};
  bool user_moved_target_{false};
  bool user_moved_start_{false};
  tramcar::planning::Pose3 last_pose_;
  std::chrono::steady_clock::time_point last_marker_update_{};
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
};

}  // namespace excavator_visualization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<excavator_visualization::TargetMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
