#include "excavator_visualization/planning_scene_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace tramcar {
namespace planning {
namespace visualization {
namespace {

bool IsFinitePose(const geometry_msgs::msg::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool IsFiniteSize(const geometry_msgs::msg::Vector3& size) {
  return std::isfinite(size.x) && std::isfinite(size.y) &&
         std::isfinite(size.z);
}

}  // namespace

PlanningSceneManager::PlanningSceneManager(std::string frame_id)
    : frame_id_(std::move(frame_id)) {}

ValidationResult PlanningSceneManager::ValidateBox(
    const std::string& id, const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Vector3& size) {
  if (id.empty()) {
    return {false, "obstacle id must not be empty"};
  }
  if (!IsFinitePose(pose)) {
    return {false, "obstacle pose contains non-finite values"};
  }
  if (!IsFiniteSize(size) || size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0) {
    return {false, "obstacle size must be finite and positive"};
  }
  const double quaternion_norm =
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
  if (quaternion_norm < 1.0e-9) {
    return {false, "obstacle orientation quaternion is invalid"};
  }
  return {true, ""};
}

ValidationResult PlanningSceneManager::AddBox(
    const std::string& id, const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Vector3& size) {
  const ValidationResult validation = ValidateBox(id, pose, size);
  if (!validation.ok) {
    return validation;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::any_of(objects_.begin(), objects_.end(),
                  [&id](const BoxObstacle& box) { return box.id == id; })) {
    return {false, "obstacle id already exists"};
  }
  objects_.push_back(BoxObstacle{id, pose, size});
  return {true, ""};
}

ValidationResult PlanningSceneManager::UpdateBox(
    const std::string& id, const geometry_msgs::msg::Pose& pose,
    const geometry_msgs::msg::Vector3& size) {
  const ValidationResult validation = ValidateBox(id, pose, size);
  if (!validation.ok) {
    return validation;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found =
      std::find_if(objects_.begin(), objects_.end(),
                   [&id](const BoxObstacle& box) { return box.id == id; });
  if (found == objects_.end()) {
    return {false, "obstacle id does not exist"};
  }
  found->pose = pose;
  found->size = size;
  return {true, ""};
}

ValidationResult PlanningSceneManager::RemoveBox(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found =
      std::find_if(objects_.begin(), objects_.end(),
                   [&id](const BoxObstacle& box) { return box.id == id; });
  if (found == objects_.end()) {
    return {false, "obstacle id does not exist"};
  }
  objects_.erase(found);
  return {true, ""};
}

void PlanningSceneManager::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  objects_.clear();
}

std::vector<BoxObstacle> PlanningSceneManager::Objects() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return objects_;
}

std::size_t PlanningSceneManager::Size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return objects_.size();
}

std::string PlanningSceneManager::frame_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_id_;
}

void PlanningSceneManager::SetFrameId(const std::string& frame_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  frame_id_ = frame_id;
}

excavator_interfaces::msg::PlanningScene
PlanningSceneManager::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = frame_id_;
  scene.is_diff = false;
  scene.objects.reserve(objects_.size());
  for (const BoxObstacle& box : objects_) {
    excavator_interfaces::msg::CollisionObject object;
    object.id = box.id;
    object.pose = box.pose;
    object.enabled = true;
    object.primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitive.dimensions = {box.size.x, box.size.y, box.size.z};
    scene.objects.push_back(std::move(object));
  }
  return scene;
}

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar
