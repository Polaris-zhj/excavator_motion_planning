#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "excavator_collision/collision_checker_factory.hpp"
#include "excavator_collision/obstacle_adapter.hpp"
#include "excavator_collision/robot_collision_geometry_model.hpp"
#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "kinematics/dh_kinematics_model.hpp"
#include "kinematics/urdf_kinematics_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/empty.hpp"
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace excavator_visualization {
namespace {

namespace data = excavator::planning::data;

using excavator::planning::CollisionCheckResult;
using excavator::planning::CollisionChecker;
using excavator::planning::CollisionCheckerFactory;
using excavator::planning::CollisionGeometry;
using excavator::planning::CollisionGeometryInstance;
using excavator::planning::CollisionGeometryType;
using excavator::planning::CollisionModelConfig;
using excavator::planning::CollisionQueryOptions;
using excavator::planning::DhKinematicsModel;
using excavator::planning::InverseKinematicsOptions;
using excavator::planning::InverseKinematicsResult;
using excavator::planning::JointPositions;
using excavator::planning::KinematicsModel;
using excavator::planning::LinkCollisionGeometry;
using excavator::planning::Obstacle;
using excavator::planning::ObstacleAdapter;
using excavator::planning::ObstacleList;
using excavator::planning::ObstacleType;
using excavator::planning::Pose3;
using excavator::planning::RobotCollisionGeometryModel;
using excavator::planning::UrdfKinematicsModel;
using excavator::planning::ValidationResult;

const char kTipMarker[] = "bucket_tip_drag";
const char kCollisionNamespace[] = "excavator_collision_geometry";
// Obstacle id used for the point cloud built from RViz "Publish Point" clicks.
constexpr int kClickedPointCloudId = 9001;

std_msgs::msg::ColorRGBA MakeColor(float red, float green, float blue,
                                   float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

geometry_msgs::msg::Pose ToRosPose(const Pose3& pose) {
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

geometry_msgs::msg::Pose ToRosPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose message;
  message.position.x = transform.translation().x();
  message.position.y = transform.translation().y();
  message.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.linear());
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();
  return message;
}

Eigen::Isometry3d ToTransform(const Pose3& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();
  return transform;
}

Pose3 ToPose3(const geometry_msgs::msg::Pose& message) {
  Pose3 pose;
  pose.position.x = message.position.x;
  pose.position.y = message.position.y;
  pose.position.z = message.position.z;
  pose.orientation.x = message.orientation.x;
  pose.orientation.y = message.orientation.y;
  pose.orientation.z = message.orientation.z;
  pose.orientation.w = message.orientation.w;
  return pose;
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

std::string ReadFile(const std::string& path, std::string* base_directory) {
  std::ifstream file(path);
  if (!file) {
    return {};
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::size_t separator = path.find_last_of('/');
  if (base_directory != nullptr) {
    *base_directory = separator == std::string::npos
                          ? std::string()
                          : path.substr(0, separator);
  }
  return buffer.str();
}

bool IsFinite(const geometry_msgs::msg::Pose& pose) {
  const Eigen::Quaterniond quaternion(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && quaternion.coeffs().allFinite() &&
         quaternion.norm() > 1.0e-9;
}

}  // namespace

// Interactive collision test helper for simulation:
//  - displays the YAML collision geometry of every link;
//  - continuously checks the current /joint_states against /excavator/scene
//    (the RViz planning panel publishes this when an obstacle is added,
//    edited or dragged);
//  - paints the colliding link's collision body and the offending obstacle
//    bright red on collision;
//  - offers a draggable bucket-tip marker that solves IK and publishes
//    /excavator/manual_joint_state so the simulated robot can be jogged.
//
// Obstacle creation/dragging itself is provided by the existing RViz planning
// panel (interactive marker namespace /excavator_obstacle).
class CollisionVisualTestNode final : public rclcpp::Node {
 public:
  CollisionVisualTestNode()
      : Node("excavator_collision_visual_test"),
        joint_names_(declare_parameter<std::vector<std::string>>(
            "joint_names",
            {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"})) {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    tip_link_ = declare_parameter<std::string>("tip_link", "bucket_tip");
    const std::string kinematics_backend =
        declare_parameter<std::string>("kinematics_backend", "dh");
    const std::string dh_parameters_file =
        declare_parameter<std::string>("dh_parameters_file", "");
    const std::string robot_description =
        declare_parameter<std::string>("robot_description", "");
    const std::string collision_model_file =
        declare_parameter<std::string>("collision_model_file", "");
    const double check_rate_hz =
        declare_parameter<double>("check_rate_hz", 20.0);
    point_cloud_resolution_ =
        declare_parameter<double>("point_cloud_resolution", 0.05);
    point_marker_scale_ =
        declare_parameter<double>("point_marker_scale", 0.06);
    if (!std::isfinite(point_cloud_resolution_) ||
        point_cloud_resolution_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "point_cloud_resolution must be positive; using 0.05 m");
      point_cloud_resolution_ = 0.05;
    }
    if (!std::isfinite(point_marker_scale_) || point_marker_scale_ <= 0.0) {
      point_marker_scale_ = 0.06;
    }
    // Empty by default: the visual test must show every non-adjacent
    // self-collision, including bucket hitting the crawler chassis. Callers
    // can still opt into specific allowed pairs through the parameter.
    const std::vector<std::string> allowed_pairs =
        declare_parameter<std::vector<std::string>>(
            "allowed_collision_pairs", std::vector<std::string>{});

    if (!LoadKinematics(kinematics_backend, dh_parameters_file,
                        robot_description)) {
      RCLCPP_ERROR(get_logger(), "Kinematics unavailable; collision test node "
                                 "will not move or check the robot");
    }

    std::string collision_base_directory;
    const std::string collision_yaml =
        ReadFile(collision_model_file, &collision_base_directory);
    if (collision_yaml.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read collision model YAML: %s",
                   collision_model_file.c_str());
    } else {
      InitializeCollisionChecker(collision_yaml, collision_base_directory,
                                 allowed_pairs);
    }

    collision_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/excavator/collision_test_markers",
            rclcpp::QoS(1).transient_local());
    manual_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>(
            "/excavator/manual_joint_state", 10);

    joint_state_subscription_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
              HandleJointState(std::move(message));
            });
    scene_subscription_ =
        create_subscription<excavator_interfaces::msg::PlanningScene>(
            "/excavator/scene", rclcpp::QoS(1).transient_local(),
            [this](
                excavator_interfaces::msg::PlanningScene::ConstSharedPtr
                    message) { scene_ = *message; });
    manual_target_subscription_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "/excavator/manual_target_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
              if (!message->header.frame_id.empty() &&
                  message->header.frame_id != frame_id_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "manual_target_pose frame '%s' ignored; expected '%s'",
                    message->header.frame_id.c_str(), frame_id_.c_str());
                return;
              }
              HandleManualTarget(ToPose3(message->pose));
            });
    clicked_point_subscription_ =
        create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
              HandleClickedPoint(std::move(message));
            });
    clear_cloud_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "/excavator/clear_point_cloud", 10,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          const std::size_t count = clicked_points_.size();
          clicked_points_.clear();
          RCLCPP_INFO(get_logger(), "Cleared %zu point-cloud test points",
                      count);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "excavator_collision_test", this);
    CreateTipMarker();
    server_->applyChanges();

    const double period = check_rate_hz > 0.0 ? 1.0 / check_rate_hz : 0.05;
    check_timer_ = create_wall_timer(
        std::chrono::duration<double>(period),
        [this]() { UpdateCollisionMarkers(); });

    RCLCPP_INFO(get_logger(),
                "Collision visual test ready: drag '%s' to jog, add/drag box "
                "obstacles in the planning panel, or use RViz 'Publish Point' "
                "to add point-cloud points; collisions turn red",
                kTipMarker);
  }

 private:
  bool LoadKinematics(const std::string& backend,
                      const std::string& dh_parameters_file,
                      const std::string& robot_description) {
    if (backend == "dh") {
      auto model = std::make_unique<DhKinematicsModel>(joint_names_);
      const ValidationResult loaded =
          model->InitializeFromFile(dh_parameters_file);
      if (!loaded.ok) {
        RCLCPP_ERROR(get_logger(), "Failed to load DH parameters: %s",
                     loaded.message.c_str());
        return false;
      }
      kinematics_ = std::move(model);
      kinematics_ready_ = true;
      return true;
    }
    if (backend == "urdf") {
      auto model = std::make_unique<UrdfKinematicsModel>(joint_names_);
      const ValidationResult loaded =
          model->InitializeFromString(robot_description);
      if (!loaded.ok) {
        RCLCPP_ERROR(get_logger(), "Failed to load URDF kinematics: %s",
                     loaded.message.c_str());
        return false;
      }
      kinematics_ = std::move(model);
      kinematics_ready_ = true;
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Unknown kinematics_backend: %s",
                 backend.c_str());
    return false;
  }

  void InitializeCollisionChecker(
      const std::string& collision_yaml,
      const std::string& collision_base_directory,
      const std::vector<std::string>& allowed_pairs) {
    checker_ = CollisionCheckerFactory::Instance().Create("fcl");
    if (!checker_) {
      RCLCPP_ERROR(get_logger(), "Collision backend 'fcl' is not registered");
      return;
    }
    CollisionModelConfig config;
    config.joint_names = joint_names_;
    config.model_format = "yaml";
    config.model_source = collision_yaml;
    config.model_base_directory = collision_base_directory;
    config.kinematics_reference_frame = frame_id_;
    const ValidationResult initialized =
        checker_->Initialize(config, kinematics_.get());
    if (!initialized.ok) {
      RCLCPP_ERROR(get_logger(), "Collision checker initialization failed: %s",
                   initialized.message.c_str());
      checker_.reset();
      return;
    }
    for (const auto& pair : ParsePairs(allowed_pairs)) {
      checker_->SetAllowedCollision(pair.first, pair.second, true);
    }

    const ValidationResult model_loaded = robot_model_.InitializeFromYaml(
        collision_yaml, joint_names_, frame_id_, collision_base_directory);
    if (!model_loaded.ok) {
      RCLCPP_WARN(get_logger(),
                  "Collision geometry visuals unavailable: %s",
                  model_loaded.message.c_str());
      robot_model_ready_ = false;
    } else {
      robot_model_ready_ = true;
    }
  }

  static std::vector<std::pair<std::string, std::string>> ParsePairs(
      const std::vector<std::string>& entries) {
    std::vector<std::pair<std::string, std::string>> pairs;
    for (const std::string& entry : entries) {
      const std::size_t separator = entry.find(':');
      if (separator == std::string::npos || separator == 0 ||
          separator + 1 == entry.size()) {
        continue;
      }
      pairs.emplace_back(entry.substr(0, separator),
                         entry.substr(separator + 1));
    }
    return pairs;
  }

  void CreateTipMarker() {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.name = kTipMarker;
    marker.description = "Drag to jog the excavator (IK, position only)";
    marker.pose.position.x = 3.5;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.scale = 0.7;

    visualization_msgs::msg::Marker visual;
    visual.type = visualization_msgs::msg::Marker::SPHERE;
    visual.scale.x = 0.18;
    visual.scale.y = 0.18;
    visual.scale.z = 0.18;
    visual.color = MakeColor(0.1F, 0.9F, 1.0F, 0.9F);

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

    server_->insert(
        marker,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr& feedback) { HandleTipFeedback(feedback); });
  }

  void HandleTipFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
          feedback) {
    if (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
      dragging_tip_ = true;
      return;
    }
    if (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      dragging_tip_ = false;
      return;
    }
    if (feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      return;
    }
    HandleManualTarget(ToPose3(feedback->pose));
  }

  void HandleManualTarget(const Pose3& target) {
    if (!kinematics_ || !kinematics_ready_) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    const double since_last =
        std::chrono::duration<double>(now_time - last_ik_time_).count();
    if (since_last < 0.02) {
      return;
    }
    last_ik_time_ = now_time;

    InverseKinematicsOptions options;
    // The excavator has four joints; allow arbitrary tip translation and keep
    // orientation free instead of demanding an unreachable 6-DoF pose.
    options.orientation_weight = 0.0;
    options.position_tolerance = 0.005;
    options.max_iterations = 80;
    options.max_joint_step = 0.25;
    options.time_budget_seconds = 0.03;

    JointPositions seed = latest_joint_positions_;
    if (seed.size() != joint_names_.size()) {
      seed.clear();
      for (const auto& limit : kinematics_->GetPositionLimits()) {
        seed.push_back(0.5 * (limit.first + limit.second));
      }
    }
    const InverseKinematicsResult result =
        kinematics_->SolveIK(target, tip_link_, seed, options);
    if (!result.success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                           "Tip IK failed: %s", result.message.c_str());
      return;
    }

    latest_joint_positions_ = result.joint_positions;
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = joint_names_;
    message.position = result.joint_positions;
    message.velocity.assign(joint_names_.size(), 0.0);
    manual_publisher_->publish(message);
  }

  void HandleJointState(
      sensor_msgs::msg::JointState::ConstSharedPtr message) {
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
    latest_joint_positions_ = std::move(positions);
    kinematics_ready_ = kinematics_ != nullptr;

    if (dragging_tip_ || !kinematics_ready_ || !server_) {
      return;
    }
    const auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_time - last_marker_sync_).count() <
        0.05) {
      return;
    }
    last_marker_sync_ = now_time;
    Pose3 tip_pose;
    if (!kinematics_->GetLinkPose(latest_joint_positions_, tip_link_, &tip_pose)
             .ok) {
      return;
    }
    geometry_msgs::msg::Pose marker_pose = ToRosPose(tip_pose);
    server_->setPose(kTipMarker, marker_pose);
    server_->applyChanges();
  }

  // RViz "Publish Point" publishes /clicked_point in the fixed frame. Points
  // are transformed to the collision frame and accumulated into one point
  // cloud obstacle, so point-cloud collision checking is usable like a box.
  void HandleClickedPoint(
      const geometry_msgs::msg::PointStamped::ConstSharedPtr& message) {
    if (!message) {
      return;
    }
    geometry_msgs::msg::PointStamped point = *message;
    if (point.header.frame_id.empty()) {
      point.header.frame_id = frame_id_;
    }
    if (point.header.frame_id != frame_id_) {
      try {
        point = tf_buffer_->transform(*message, frame_id_,
                                      tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException& exception) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot transform clicked_point from '%s' to '%s': %s",
            message->header.frame_id.c_str(), frame_id_.c_str(),
            exception.what());
        return;
      }
    }
    if (!std::isfinite(point.point.x) || !std::isfinite(point.point.y) ||
        !std::isfinite(point.point.z)) {
      return;
    }

    data::Point stored;
    stored.x = point.point.x;
    stored.y = point.point.y;
    stored.z = point.point.z;
    for (const data::Point& existing : clicked_points_) {
      if (std::abs(existing.x - stored.x) < 1.0e-4 &&
          std::abs(existing.y - stored.y) < 1.0e-4 &&
          std::abs(existing.z - stored.z) < 1.0e-4) {
        RCLCPP_DEBUG(get_logger(), "Ignoring duplicate point (%.3f, %.3f, %.3f)",
                     stored.x, stored.y, stored.z);
        return;
      }
    }
    clicked_points_.push_back(stored);
    RCLCPP_INFO(get_logger(), "Added point (%.3f, %.3f, %.3f); %zu total",
                stored.x, stored.y, stored.z, clicked_points_.size());
  }

  void UpdateCollisionMarkers() {
    if (!checker_ || !kinematics_ready_ ||
        latest_joint_positions_.size() != joint_names_.size()) {
      return;
    }
    // A second /joint_states source makes the collision pose (latest message)
    // disagree with the robot the user sees in RViz; report it explicitly.
    const auto now_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_time - last_publisher_check_).count() >
        2.0) {
      last_publisher_check_ = now_time;
      const auto joint_state_publishers =
          get_publishers_info_by_topic("/joint_states");
      if (joint_state_publishers.size() > 1) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Multiple /joint_states publishers (%zu) detected; collision may "
            "use a different robot pose than the RViz display",
            joint_state_publishers.size());
      }
    }
    std::vector<Obstacle> converted;
    CollisionCheckResult result;
    std::vector<std::string> colliding_links;
    const ValidationResult obstacle_result =
        ObstacleAdapter::Convert(scene_.objects, &converted);
    if (!obstacle_result.ok) {
      result.message = obstacle_result.message;
    } else {
      ObstacleList geometry_obstacles;
      ObstacleList message_clouds;
      for (const Obstacle& obstacle : converted) {
        if (obstacle.cloud_points.empty()) {
          geometry_obstacles.push_back(obstacle);
        } else {
          message_clouds.push_back(obstacle);
        }
      }

      const bool has_clicked_cloud = !clicked_points_.empty();
      Obstacle clicked_cloud;
      if (has_clicked_cloud) {
        clicked_cloud.id = kClickedPointCloudId;
        // Points are already expressed in frame_id_, so the cloud obstacle
        // itself sits at the origin.
        clicked_cloud.center.x = 0.0;
        clicked_cloud.center.y = 0.0;
        clicked_cloud.center.z = 0.0;
        clicked_cloud.cloud_points = clicked_points_;
      }

      // The interface selects one obstacle representation per query, so
      // geometry obstacles and point clouds are checked separately and the
      // first collision is reported. Self-collision runs once.
      const CollisionCheckResult self_result =
          checker_->CheckSelfCollision(latest_joint_positions_);
      CollisionQueryOptions geometry_options;
      geometry_options.collect_all_collisions = true;
      CollisionCheckResult geometry_result;
      geometry_result.valid = true;
      if (!geometry_obstacles.empty()) {
        geometry_result = checker_->CheckEnvironmentCollision(
            latest_joint_positions_, geometry_obstacles, geometry_options);
      }
      CollisionCheckResult cloud_result;
      cloud_result.valid = true;
      if (has_clicked_cloud || !message_clouds.empty()) {
        ObstacleList clouds = std::move(message_clouds);
        if (has_clicked_cloud) {
          clouds.push_back(clicked_cloud);
        }
        CollisionQueryOptions cloud_options;
        cloud_options.obstacle_type = ObstacleType::kPointCloud;
        cloud_options.point_cloud_resolution = point_cloud_resolution_;
        cloud_options.collect_all_collisions = true;
        cloud_result = checker_->CheckEnvironmentCollision(
            latest_joint_positions_, clouds, cloud_options);
      }

      // Together the queries cover every colliding link, so adding a point
      // that touches a new link highlights that link as well.
      const auto append_colliding_links =
          [&colliding_links](const CollisionCheckResult& source) {
            if (!source.valid || !source.collision) {
              return;
            }
            const auto append = [&colliding_links](const std::string& link) {
              if (!link.empty() &&
                  std::find(colliding_links.begin(), colliding_links.end(),
                            link) == colliding_links.end()) {
                colliding_links.push_back(link);
              }
            };
            append(source.first_link);
            for (const std::string& link : source.colliding_links) {
              append(link);
            }
          };
      append_colliding_links(self_result);
      append_colliding_links(geometry_result);
      append_colliding_links(cloud_result);

      const bool invalid = !self_result.valid || !geometry_result.valid ||
                           !cloud_result.valid;
      if (self_result.valid && self_result.collision) {
        result = self_result;
      } else if (geometry_result.valid && geometry_result.collision) {
        result = geometry_result;
      } else if (cloud_result.valid && cloud_result.collision) {
        result = cloud_result;
      } else if (!invalid) {
        result.valid = true;
        result.collision = false;
        result.message = "no collision detected";
      } else {
        result.message =
            !self_result.valid
                ? self_result.message
                : (!geometry_result.valid ? geometry_result.message
                                          : cloud_result.message);
      }
    }

    if (!result.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Collision check failed (no red markers shown): %s",
                           result.message.c_str());
    }

    const bool cloud_collision =
        result.valid && result.collision &&
        result.second_object == std::to_string(kClickedPointCloudId);
    const bool has_points = !clicked_points_.empty();
    const bool collision_now = result.valid && result.collision;

    std::vector<std::string> sorted_links = colliding_links;
    std::sort(sorted_links.begin(), sorted_links.end());
    std::string link_list;
    for (const std::string& link : sorted_links) {
      link_list += " " + link;
    }
    if (collision_now != collision_active_) {
      if (collision_now) {
        RCLCPP_INFO(get_logger(),
                    "Collision detected: %s (links:%s points=%zu)",
                    result.message.c_str(), link_list.c_str(),
                    clicked_points_.size());
      } else {
        RCLCPP_INFO(get_logger(), "Collision cleared");
      }
    } else if (collision_now && sorted_links != last_colliding_links_) {
      RCLCPP_INFO(get_logger(), "Collision links updated:%s (points=%zu)",
                  link_list.c_str(), clicked_points_.size());
    }
    last_colliding_links_ = collision_now ? sorted_links
                                          : std::vector<std::string>{};

    visualization_msgs::msg::MarkerArray markers;
    if (result.valid && result.collision) {
      // Highlight every colliding robot link. Self collisions add both links;
      // environment collisions add every link that intersects an obstacle.
      if (colliding_links.empty() && !result.first_link.empty()) {
        colliding_links.push_back(result.first_link);
      }
      AppendRobotGeometryMarkers(colliding_links, &markers);
      AppendCollisionText(result, &markers);
      AppendObstacleHighlight(result, &markers);
    }
    if (has_points) {
      AppendPointCloudMarkers(cloud_collision, &markers);
    }

    if (markers.markers.empty()) {
      // Clear only when the previous cycle showed something (or once at
      // startup, to remove stale transient-local markers).
      if (!cleared_once_ || collision_active_) {
        visualization_msgs::msg::MarkerArray clear;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = now();
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear.markers.push_back(marker);
        collision_publisher_->publish(clear);
        cleared_once_ = true;
      }
      collision_active_ = false;
      return;
    }

    collision_active_ = result.valid && result.collision;
    cleared_once_ = true;
    collision_publisher_->publish(markers);
  }

  void AppendPointCloudMarkers(bool colliding,
                               visualization_msgs::msg::MarkerArray* markers) {
    if (clicked_points_.empty()) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "excavator_collision_test_points";
    marker.id = 9200;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = point_marker_scale_;
    marker.scale.y = point_marker_scale_;
    marker.scale.z = point_marker_scale_;
    // Orange while clear, bright red when this point cloud is the colliding
    // obstacle.
    marker.color = colliding ? MakeColor(1.0F, 0.0F, 0.0F, 0.95F)
                             : MakeColor(1.0F, 0.55F, 0.0F, 0.85F);
    marker.points.reserve(clicked_points_.size());
    for (const data::Point& point : clicked_points_) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = point.z;
      marker.points.push_back(ros_point);
    }
    markers->markers.push_back(std::move(marker));
  }

  void AppendRobotGeometryMarkers(
      const std::vector<std::string>& colliding_links,
      visualization_msgs::msg::MarkerArray* markers) {
    if (!robot_model_ready_ || !kinematics_) {
      return;
    }
    int32_t id = 0;
    for (const LinkCollisionGeometry& link : robot_model_.links()) {
      if (std::find(colliding_links.begin(), colliding_links.end(),
                    link.link_name) == colliding_links.end()) {
        continue;
      }
      Pose3 link_pose;
      if (!kinematics_
               ->GetLinkPose(latest_joint_positions_, link.link_name, &link_pose)
               .ok) {
        continue;
      }
      const Eigen::Isometry3d world_link = ToTransform(link_pose);
      for (const CollisionGeometryInstance& instance : link.geometries) {
        const Eigen::Isometry3d world_geometry =
            world_link * instance.link_to_collision;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = now();
        marker.ns = kCollisionNamespace;
        marker.id = id++;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = ToRosPose(world_geometry);
        // Only the colliding link is drawn, in bright red.
        marker.color = MakeColor(1.0F, 0.02F, 0.0F, 0.85F);
        if (!AppendGeometryShape(instance.geometry, &marker)) {
          continue;
        }
        markers->markers.push_back(std::move(marker));
      }
    }
  }

  static bool AppendGeometryShape(const CollisionGeometry& geometry,
                                  visualization_msgs::msg::Marker* marker) {
    const Eigen::Vector3d dimensions = geometry.dimensions;
    switch (geometry.type) {
      case CollisionGeometryType::kBox:
        marker->type = visualization_msgs::msg::Marker::CUBE;
        marker->scale.x = dimensions.x();
        marker->scale.y = dimensions.y();
        marker->scale.z = dimensions.z();
        return true;
      case CollisionGeometryType::kSphere:
        marker->type = visualization_msgs::msg::Marker::SPHERE;
        marker->scale.x = 2.0 * dimensions.x();
        marker->scale.y = marker->scale.x;
        marker->scale.z = marker->scale.x;
        return true;
      case CollisionGeometryType::kCylinder:
        marker->type = visualization_msgs::msg::Marker::CYLINDER;
        marker->scale.x = 2.0 * dimensions.x();
        marker->scale.y = marker->scale.x;
        marker->scale.z = dimensions.y();
        return true;
      case CollisionGeometryType::kCapsule:
        marker->type = visualization_msgs::msg::Marker::CYLINDER;
        marker->scale.x = 2.0 * dimensions.x();
        marker->scale.y = marker->scale.x;
        marker->scale.z = dimensions.y() + 2.0 * dimensions.x();
        return true;
      case CollisionGeometryType::kTriangleMesh: {
        if (geometry.mesh_vertices.empty()) {
          return false;
        }
        Eigen::Vector3d minimum =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d maximum =
            Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
        for (const Eigen::Vector3d& vertex : geometry.mesh_vertices) {
          minimum = minimum.cwiseMin(vertex);
          maximum = maximum.cwiseMax(vertex);
        }
        const Eigen::Vector3d center = 0.5 * (minimum + maximum);
        // Translate the marker, which currently sits at the geometry origin,
        // to the mesh bounding-box center in world coordinates.
        const Eigen::Quaterniond orientation(
            marker->pose.orientation.w, marker->pose.orientation.x,
            marker->pose.orientation.y, marker->pose.orientation.z);
        const Eigen::Vector3d world_center =
            orientation * center +
            Eigen::Vector3d(marker->pose.position.x, marker->pose.position.y,
                            marker->pose.position.z);
        marker->pose.position.x = world_center.x();
        marker->pose.position.y = world_center.y();
        marker->pose.position.z = world_center.z();
        marker->type = visualization_msgs::msg::Marker::CUBE;
        marker->scale.x = std::max(1.0e-3, maximum.x() - minimum.x());
        marker->scale.y = std::max(1.0e-3, maximum.y() - minimum.y());
        marker->scale.z = std::max(1.0e-3, maximum.z() - minimum.z());
        return true;
      }
    }
    return false;
  }

  void AppendCollisionText(const CollisionCheckResult& result,
                           visualization_msgs::msg::MarkerArray* markers) {
    Pose3 pose;
    if (!kinematics_
             ->GetLinkPose(latest_joint_positions_, result.first_link, &pose)
             .ok) {
      pose = Pose3{};
    }
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id_;
    text.header.stamp = now();
    text.ns = "excavator_collision_text";
    text.id = 9000;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = ToRosPose(pose);
    text.pose.position.z += 0.8;
    text.scale.z = 0.22;
    text.color = MakeColor(1.0F, 0.05F, 0.02F, 1.0F);
    text.text = "COLLISION: " + result.message;
    markers->markers.push_back(std::move(text));
  }

  void AppendObstacleHighlight(
      const CollisionCheckResult& result,
      visualization_msgs::msg::MarkerArray* markers) {
    for (const auto& object : scene_.objects) {
      if (!object.enabled ||
          std::to_string(ObstacleAdapter::ObjectId(object.id)) !=
              result.second_object) {
        continue;
      }
      if (!IsFinite(object.pose)) {
        return;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "excavator_collision_obstacle";
      marker.id = 9100;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = object.pose;
      marker.color = MakeColor(1.0F, 0.0F, 0.0F, 0.65F);
      const auto& dimensions = object.primitive.dimensions;
      if (object.primitive.type ==
          shape_msgs::msg::SolidPrimitive::SPHERE) {
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        const double diameter =
            dimensions.empty() ? 0.2 : 2.0 * dimensions[0];
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;
      } else if (object.primitive.type ==
                 shape_msgs::msg::SolidPrimitive::CYLINDER) {
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        const double height = dimensions.empty() ? 0.2 : dimensions[0];
        const double radius = dimensions.size() > 1 ? dimensions[1] : 0.1;
        marker.scale.x = 2.0 * radius;
        marker.scale.y = marker.scale.x;
        marker.scale.z = height;
      } else {
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = dimensions.size() > 0 ? dimensions[0] : 0.2;
        marker.scale.y = dimensions.size() > 1 ? dimensions[1] : 0.2;
        marker.scale.z = dimensions.size() > 2 ? dimensions[2] : 0.2;
      }
      markers->markers.push_back(std::move(marker));
      return;
    }
  }

  std::vector<std::string> joint_names_;
  std::string frame_id_;
  std::string tip_link_;
  std::unique_ptr<KinematicsModel> kinematics_;
  std::unique_ptr<CollisionChecker> checker_;
  RobotCollisionGeometryModel robot_model_;
  bool robot_model_ready_{false};
  bool kinematics_ready_{false};
  bool dragging_tip_{false};
  bool collision_active_{false};
  bool cleared_once_{false};
  std::vector<std::string> last_colliding_links_;
  double point_cloud_resolution_{0.05};
  double point_marker_scale_{0.06};
  std::vector<data::Point> clicked_points_;

  JointPositions latest_joint_positions_;
  excavator_interfaces::msg::PlanningScene scene_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      collision_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr manual_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
  rclcpp::Subscription<excavator_interfaces::msg::PlanningScene>::SharedPtr
      scene_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      manual_target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_point_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr
      clear_cloud_subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::chrono::steady_clock::time_point last_ik_time_{};
  std::chrono::steady_clock::time_point last_marker_sync_{};
  std::chrono::steady_clock::time_point last_publisher_check_{};
};

}  // namespace excavator_visualization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<excavator_visualization::CollisionVisualTestNode>());
  rclcpp::shutdown();
  return 0;
}
