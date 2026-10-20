#include <urdf/model.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_visualization/trajectory_occupancy_visualizer.hpp"
#include "excavator_visualization/trajectory_visualizer.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace excavator_visualization {
namespace {

using tramcar::planning::JointPositions;
using tramcar::planning::JointTrajectory;
using tramcar::planning::JointTrajectoryPoint;
using tramcar::planning::Pose3;
using tramcar::planning::ValidationResult;

double ToSeconds(const builtin_interfaces::msg::Duration& duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool BuildOccupancyVisuals(
    const std::string& urdf_xml, const std::vector<std::string>& link_names,
    std::vector<tramcar::planning::visualization::LinkVisual>* visuals,
    rclcpp::Logger logger) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    RCLCPP_ERROR(logger, "Failed to parse URDF for occupancy visuals");
    return false;
  }
  visuals->clear();
  for (const std::string& link_name : link_names) {
    const auto link = model.getLink(link_name);
    if (!link) {
      RCLCPP_WARN(logger, "Occupancy link '%s' is not in the URDF",
                  link_name.c_str());
      continue;
    }
    urdf::VisualSharedPtr visual = link->visual;
    if (!visual && !link->visual_array.empty()) {
      visual = link->visual_array.front();
    }
    if (!visual || !visual->geometry) {
      RCLCPP_WARN(logger, "Occupancy link '%s' has no visual; skipping",
                  link_name.c_str());
      continue;
    }
    tramcar::planning::visualization::LinkVisual link_visual;
    if (visual->geometry->type == urdf::Geometry::MESH) {
      const auto mesh =
          std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
      if (!mesh || mesh->filename.empty()) {
        RCLCPP_WARN(logger, "Occupancy mesh for '%s' is invalid; skipping",
                    link_name.c_str());
        continue;
      }
      link_visual.type =
          tramcar::planning::visualization::LinkVisualType::kMesh;
      link_visual.mesh_resource = mesh->filename;
      link_visual.scale.x = mesh->scale.x;
      link_visual.scale.y = mesh->scale.y;
      link_visual.scale.z = mesh->scale.z;
    } else if (visual->geometry->type == urdf::Geometry::BOX) {
      const auto box =
          std::dynamic_pointer_cast<urdf::Box>(visual->geometry);
      if (!box || box->dim.x <= 0.0 || box->dim.y <= 0.0 ||
          box->dim.z <= 0.0) {
        RCLCPP_WARN(logger, "Occupancy box for '%s' is invalid; skipping",
                    link_name.c_str());
        continue;
      }
      link_visual.type =
          tramcar::planning::visualization::LinkVisualType::kBox;
      link_visual.scale.x = box->dim.x;
      link_visual.scale.y = box->dim.y;
      link_visual.scale.z = box->dim.z;
    } else {
      RCLCPP_WARN(logger,
                  "Occupancy link '%s' uses an unsupported visual type; "
                  "skipping", link_name.c_str());
      continue;
    }
    link_visual.link_name = link_name;
    link_visual.origin.position.x = visual->origin.position.x;
    link_visual.origin.position.y = visual->origin.position.y;
    link_visual.origin.position.z = visual->origin.position.z;
    link_visual.origin.orientation.x = visual->origin.rotation.x;
    link_visual.origin.orientation.y = visual->origin.rotation.y;
    link_visual.origin.orientation.z = visual->origin.rotation.z;
    link_visual.origin.orientation.w = visual->origin.rotation.w;
    if (visual->material && (visual->material->color.r != 0.0F ||
                             visual->material->color.g != 0.0F ||
                             visual->material->color.b != 0.0F)) {
      link_visual.red = visual->material->color.r;
      link_visual.green = visual->material->color.g;
      link_visual.blue = visual->material->color.b;
    }
    link_visual.alpha = 0.30F;
    visuals->push_back(std::move(link_visual));
  }
  return !visuals->empty();
}

JointTrajectory ToCoreTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message) {
  JointTrajectory trajectory;
  trajectory.joint_names = message.joint_names;
  trajectory.points.reserve(message.points.size());
  for (const auto& message_point : message.points) {
    JointTrajectoryPoint point;
    point.t = ToSeconds(message_point.time_from_start);
    point.positions = message_point.positions;
    point.velocities = message_point.velocities;
    point.accelerations = message_point.accelerations;
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

}  // namespace

class TrajectoryVisualizerNode final : public rclcpp::Node {
 public:
  TrajectoryVisualizerNode()
      : Node("excavator_trajectory_visualizer"),
        joint_names_(declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"})),
        kinematics_(joint_names_) {
    const std::string robot_description =
        declare_parameter<std::string>("robot_description", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "bucket_tip");
    occupancy_enabled_ = declare_parameter<bool>("occupancy_enabled", true);
    const int occupancy_samples =
        declare_parameter<int>("occupancy_samples", 8);
    const std::vector<std::string> occupancy_links =
        declare_parameter<std::vector<std::string>>(
            "occupancy_links",
            {"swing_link", "boom_link", "stick_link", "bucket_link"});

    if (!robot_description.empty()) {
      const ValidationResult loaded =
          kinematics_.InitializeFromString(robot_description);
      if (loaded.ok) {
        kinematics_ready_ = true;
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model: %s",
                     loaded.message.c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(),
                   "robot_description is empty; visualization cannot run FK");
    }

    visualizer_ = std::make_unique<
        tramcar::planning::visualization::TrajectoryVisualizer>(
        kinematics_, frame_id_, tip_link_);
    if (occupancy_enabled_ && kinematics_ready_ && !robot_description.empty()) {
      std::vector<tramcar::planning::visualization::LinkVisual>
          occupancy_visuals;
      if (BuildOccupancyVisuals(robot_description, occupancy_links,
                                &occupancy_visuals, get_logger())) {
        occupancy_visualizer_ = std::make_unique<
            tramcar::planning::visualization::TrajectoryOccupancyVisualizer>(
            kinematics_, std::move(occupancy_visuals), frame_id_,
            static_cast<std::size_t>(std::max(1, occupancy_samples)));
      } else {
        occupancy_enabled_ = false;
        RCLCPP_ERROR(get_logger(),
                     "No mesh or box visuals found for occupancy; "
                     "preview disabled");
      }
    } else {
      occupancy_enabled_ = false;
    }

    planned_marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/excavator/planned_path", rclcpp::QoS(1).transient_local());
    occupancy_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/excavator/trajectory_occupancy",
            rclcpp::QoS(1).transient_local());
    current_tip_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/excavator/current_tip", rclcpp::QoS(1));
    end_effector_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "/excavator/end_effector_path", rclcpp::QoS(1).transient_local());

    trajectory_subscription_ =
        create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/excavator/planned_trajectory", rclcpp::QoS(1).transient_local(),
            [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
              HandleTrajectory(std::move(msg));
            });
    joint_state_subscription_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
              HandleJointState(std::move(msg));
            });
    RCLCPP_INFO(get_logger(),
                "Visualization ready: frame=%s tip=%s occupancy=%s",
                frame_id_.c_str(), tip_link_.c_str(),
                occupancy_enabled_ ? "on" : "off");
  }

 private:
  void HandleTrajectory(
      trajectory_msgs::msg::JointTrajectory::ConstSharedPtr message) {
    if (!kinematics_ready_) {
      return;
    }
    const JointTrajectory trajectory = ToCoreTrajectory(*message);
    visualization_msgs::msg::MarkerArray markers;
    const ValidationResult marker_result =
        visualizer_->BuildMarkers(trajectory, &markers);
    if (!marker_result.ok) {
      RCLCPP_WARN(get_logger(), "Marker generation failed: %s",
                  marker_result.message.c_str());
      return;
    }
    for (auto& marker : markers.markers) {
      marker.header.stamp = now();
    }
    planned_marker_publisher_->publish(markers);

    nav_msgs::msg::Path path;
    const ValidationResult path_result =
        visualizer_->BuildEndEffectorPath(trajectory, &path);
    if (path_result.ok) {
      path.header.stamp = now();
      for (auto& pose : path.poses) {
        pose.header.stamp = path.header.stamp;
      }
      end_effector_path_publisher_->publish(path);
    }

    if (occupancy_enabled_) {
      visualization_msgs::msg::MarkerArray occupancy;
      const ValidationResult occupancy_result =
          occupancy_visualizer_->Build(trajectory, &occupancy);
      if (occupancy_result.ok) {
        for (auto& marker : occupancy.markers) {
          marker.header.stamp = now();
        }
        occupancy_publisher_->publish(occupancy);
      } else {
        RCLCPP_WARN(get_logger(), "Occupancy preview failed: %s",
                    occupancy_result.message.c_str());
      }
    }
  }

  void HandleJointState(sensor_msgs::msg::JointState::ConstSharedPtr message) {
    if (!kinematics_ready_ || message->position.size() < joint_names_.size()) {
      return;
    }
    JointPositions positions;
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
    visualization_msgs::msg::MarkerArray markers;
    Pose3 tip_pose;
    const ValidationResult fk =
        kinematics_.GetLinkPose(positions, tip_link_, &tip_pose);
    if (!fk.ok) {
      return;
    }
    markers.markers.clear();
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    visualization_msgs::msg::Marker tip;
    tip.header.frame_id = frame_id_;
    tip.header.stamp = now();
    tip.ns = "excavator_current_tip";
    tip.id = 0;
    tip.type = visualization_msgs::msg::Marker::SPHERE;
    tip.action = visualization_msgs::msg::Marker::ADD;
    tip.pose.position.x = tip_pose.position.x;
    tip.pose.position.y = tip_pose.position.y;
    tip.pose.position.z = tip_pose.position.z;
    tip.pose.orientation.x = tip_pose.orientation.x;
    tip.pose.orientation.y = tip_pose.orientation.y;
    tip.pose.orientation.z = tip_pose.orientation.z;
    tip.pose.orientation.w = tip_pose.orientation.w;
    tip.scale.x = 0.08;
    tip.scale.y = 0.08;
    tip.scale.z = 0.08;
    tip.color.r = 0.1F;
    tip.color.g = 0.9F;
    tip.color.b = 0.9F;
    tip.color.a = 1.0F;
    markers.markers.push_back(tip);
    current_tip_publisher_->publish(markers);
  }

  std::vector<std::string> joint_names_;
  tramcar::planning::UrdfKinematicsModel kinematics_;
  std::unique_ptr<tramcar::planning::visualization::TrajectoryVisualizer>
      visualizer_;
  std::unique_ptr<
      tramcar::planning::visualization::TrajectoryOccupancyVisualizer>
      occupancy_visualizer_;
  std::string frame_id_;
  std::string tip_link_;
  bool occupancy_enabled_{true};
  bool kinematics_ready_{false};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      planned_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      occupancy_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      current_tip_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      end_effector_path_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
};

}  // namespace excavator_visualization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<excavator_visualization::TrajectoryVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
