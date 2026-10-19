#ifndef EXCAVATOR_VISUALIZATION__PLANNING_SCENE_MANAGER_HPP_
#define EXCAVATOR_VISUALIZATION__PLANNING_SCENE_MANAGER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "excavator_common/joint_trajectory.hpp"
#include "excavator_interfaces/msg/planning_scene.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace tramcar {
namespace planning {
namespace visualization {

struct BoxObstacle {
  std::string id;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 size;
};

// Owns the editable box obstacle collection and produces complete scene
// snapshots. The visualization never performs collision checking itself.
class PlanningSceneManager {
 public:
  explicit PlanningSceneManager(std::string frame_id = "world");

  ValidationResult AddBox(const std::string& id,
                          const geometry_msgs::msg::Pose& pose,
                          const geometry_msgs::msg::Vector3& size);
  ValidationResult UpdateBox(const std::string& id,
                             const geometry_msgs::msg::Pose& pose,
                             const geometry_msgs::msg::Vector3& size);
  ValidationResult RemoveBox(const std::string& id);
  void Clear();
  std::vector<BoxObstacle> Objects() const;
  std::size_t Size() const;
  std::string frame_id() const;
  void SetFrameId(const std::string& frame_id);

  excavator_interfaces::msg::PlanningScene Snapshot() const;

 private:
  static ValidationResult ValidateBox(const std::string& id,
                                      const geometry_msgs::msg::Pose& pose,
                                      const geometry_msgs::msg::Vector3& size);

  mutable std::mutex mutex_;
  std::string frame_id_;
  std::vector<BoxObstacle> objects_;
};

}  // namespace visualization
}  // namespace planning
}  // namespace tramcar

#endif  // EXCAVATOR_VISUALIZATION__PLANNING_SCENE_MANAGER_HPP_
