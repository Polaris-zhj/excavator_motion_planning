#include "excavator_collision/collision_checker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "kinematics/dh_kinematics_model.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace tramcar {
namespace planning {
namespace {

// The collision backend is intentionally independent of URDF. This small
// analytical model plays the role of the DH kinematics model in unit tests:
// base_link -> arm_link (revolute) -> tool_link (fixed offset).
class SingleJointKinematics final : public KinematicsModel {
 public:
  explicit SingleJointKinematics(std::vector<std::string> names = {
                                     "arm_joint"})
      : joint_names_(std::move(names)) {}

  const std::vector<std::string>& joint_names() const override {
    return joint_names_;
  }
  std::vector<std::pair<double, double>> GetPositionLimits() const override {
    return {{-3.141592653589793, 3.141592653589793}};
  }
  std::vector<double> GetVelocityLimits() const override { return {1.0}; }

  ValidationResult GetLinkPose(const JointPositions& joint_positions,
                               const std::string& link_name,
                               Pose3* pose) const override {
    if (pose == nullptr) {
      return {false, "pose output pointer is null"};
    }
    if (joint_positions.size() != joint_names_.size()) {
      return {false, "joint position count mismatch"};
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (link_name == "base_link") {
      transform.setIdentity();
    } else if (link_name == "arm_link" || link_name == "tool_link") {
      Eigen::Isometry3d joint = Eigen::Isometry3d::Identity();
      joint.translation() = Eigen::Vector3d(0.4, 0.0, 0.0);
      joint.linear() =
          Eigen::AngleAxisd(joint_positions[0], Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      transform = joint;
      if (link_name == "tool_link") {
        transform = transform * Eigen::Translation3d(1.0, 0.0, 0.0);
      }
    } else {
      return {false, "unknown link: " + link_name};
    }

    pose->position.x = transform.translation().x();
    pose->position.y = transform.translation().y();
    pose->position.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.linear());
    pose->orientation.x = quaternion.x();
    pose->orientation.y = quaternion.y();
    pose->orientation.z = quaternion.z();
    pose->orientation.w = quaternion.w();
    return {true, ""};
  }

  ValidationResult GetJacobian(const JointPositions&,
                               const std::string&,
                               Jacobian*) const override {
    return {false, "not implemented in test kinematics"};
  }

  InverseKinematicsResult SolveIK(
      const Pose3&, const std::string&, const JointPositions&,
      const InverseKinematicsOptions&) const override {
    InverseKinematicsResult result;
    result.message = "not implemented in test kinematics";
    return result;
  }

 private:
  std::vector<std::string> joint_names_;
};

const char kCollisionYaml[] = R"(
robot:
  name: collision_test_robot
  root_frame: base_link
links:
  - link_name: base_link
    geometries:
      - name: base_box
        type: box
        size: [1.0, 1.0, 1.0]
        origin: {xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}
  - link_name: arm_link
    parent_link_name: base_link
    geometries:
      - name: arm_sphere
        type: sphere
        radius: 0.2
  - link_name: tool_link
    parent_link_name: arm_link
    geometries:
      - name: tool_sphere
        type: sphere
        radius: 0.2
)";

std::string MeshCollisionYaml(const std::string& mesh_path) {
  return R"(
robot:
  root_frame: base_link
links:
  - link_name: base_link
    geometries:
      - name: base_mesh
        type: mesh
        resource: ")" +
         mesh_path + R"("
        scale: [1.0, 1.0, 1.0]
  - link_name: arm_link
    parent_link_name: base_link
    geometries:
      - name: arm_sphere
        type: sphere
        radius: 0.2
  - link_name: tool_link
    parent_link_name: arm_link
    geometries:
      - name: tool_sphere
        type: sphere
        radius: 0.2
)";
}

excavator_interfaces::msg::CollisionObject MakeSphere(
    const std::string& id, double x, double y, double z, double radius) {
  excavator_interfaces::msg::CollisionObject object;
  object.id = id;
  object.enabled = true;
  object.pose.position.x = x;
  object.pose.position.y = y;
  object.pose.position.z = z;
  object.pose.orientation.w = 1.0;
  object.primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  object.primitive.dimensions = {radius};
  return object;
}

std::string ReadFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

ValidationResult InitializeChecker(FclCollisionChecker* checker,
                                   const std::string& yaml,
                                   SingleJointKinematics* kinematics) {
  CollisionModelConfig config;
  config.joint_names = kinematics->joint_names();
  config.model_source = yaml;
  config.model_format = "yaml";
  config.kinematics_reference_frame = "base_link";
  return checker->Initialize(config, kinematics);
}

TEST(FclCollisionCheckerTest, DetectsEnvironmentCollision) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere("obstacle", 0.4, 0.0, 0.0, 0.15));

  const CollisionCheckResult result = checker.CheckState({0.0}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_EQ(result.second_object, "obstacle");
}

TEST(FclCollisionCheckerTest, AcceptsClearEnvironment) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere("far_obstacle", 10.0, 10.0, 10.0, 0.1));

  const CollisionCheckResult result = checker.CheckState({0.0}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.collision);
}

TEST(FclCollisionCheckerTest, DetectsNonAdjacentSelfCollision) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const CollisionCheckResult result =
      checker.CheckState({3.141592653589793}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_NE(result.message.find("self collision"), std::string::npos);
}

TEST(FclCollisionCheckerTest, HonoursAllowedCollisionPairs) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;
  checker.SetAllowedCollisionPairs({{"base_link", "tool_link"}});

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const CollisionCheckResult result =
      checker.CheckState({3.141592653589793}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.collision);
}

TEST(FclCollisionCheckerTest, LoadsYamlStlCollisionMesh) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(
      &checker, MeshCollisionYaml(TEST_MESH_PATH), &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere("mesh_surface", 0.2, 0.2, 0.0, 0.1));
  const CollisionCheckResult result = checker.CheckState({0.0}, scene);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
}

TEST(FclCollisionCheckerTest, RejectsDuplicateJointNames) {
  SingleJointKinematics kinematics({"arm_joint", "arm_joint"});
  FclCollisionChecker checker;
  const auto result = InitializeChecker(&checker, kCollisionYaml, &kinematics);
  EXPECT_FALSE(result.ok);
}

TEST(FclCollisionCheckerTest, RequiresKinematicsModel) {
  FclCollisionChecker checker;
  CollisionModelConfig config;
  config.joint_names = {"arm_joint"};
  config.model_source = kCollisionYaml;
  config.model_format = "yaml";
  const auto result = checker.Initialize(config, nullptr);
  EXPECT_FALSE(result.ok);
}

TEST(FclCollisionCheckerTest, SamplesInteriorPathStates) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  // The obstacle is clear at both endpoints but intersects the tool's swept
  // region at an intermediate joint value.
  scene.objects.push_back(MakeSphere("swept_obstacle", 1.1, 0.0, 0.0, 0.18));

  JointPath path;
  path.joint_names = {"arm_joint"};
  path.points = {{-1.0}, {1.0}};
  const CollisionCheckResult result = checker.CheckPath(path, scene, 0.05);
  EXPECT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.checked_states, 1.0);

  const CollisionCheckResult obstacle_result =
      checker.CheckPathCollisionWithObstacle(
          path, scene, "swept_obstacle", 0.05);
  EXPECT_TRUE(obstacle_result.valid) << obstacle_result.message;
  EXPECT_TRUE(obstacle_result.collision);
}

TEST(FclCollisionCheckerTest, RejectsRootFrameMismatch) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  CollisionModelConfig config;
  config.joint_names = kinematics.joint_names();
  config.model_source = kCollisionYaml;
  config.model_format = "yaml";
  config.kinematics_reference_frame = "world";
  const auto result = checker.Initialize(config, &kinematics);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("does not match"), std::string::npos);
}

TEST(FclCollisionCheckerTest, SupportsScopedStateAndDistanceQueries) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  const CollisionCheckResult self_result = checker.CheckSelfCollision({0.0});
  ASSERT_TRUE(self_result.valid) << self_result.message;
  EXPECT_FALSE(self_result.collision);

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(
      MakeSphere("distance_obstacle", 10.0, 10.0, 10.0, 0.1));

  const CollisionCheckResult environment_result =
      checker.CheckEnvironmentCollision({0.0}, scene);
  ASSERT_TRUE(environment_result.valid) << environment_result.message;
  EXPECT_FALSE(environment_result.collision);

  const CollisionDistanceResult distance =
      checker.ComputeDistanceToObstacle({0.0}, scene, "distance_obstacle");
  ASSERT_TRUE(distance.valid) << distance.message;
  EXPECT_GT(distance.distance, 0.0);
  EXPECT_EQ(distance.obstacle_id, "distance_obstacle");
  EXPECT_FALSE(distance.robot_link.empty());
}

TEST(FclCollisionCheckerTest, RejectsUnknownDistanceObstacle) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const CollisionDistanceResult distance =
      checker.ComputeDistanceToObstacle({0.0}, scene, "missing_obstacle");
  EXPECT_FALSE(distance.valid);
  EXPECT_NE(distance.message.find("not found"), std::string::npos);
}

TEST(FclCollisionCheckerTest, RunsWithDhKinematicsAndExcavatorCollisionYaml) {
  const std::string dh_yaml = ReadFile(TEST_DH_PATH);
  const std::string collision_yaml = ReadFile(TEST_COLLISION_YAML_PATH);
  ASSERT_FALSE(dh_yaml.empty()) << TEST_DH_PATH;
  ASSERT_FALSE(collision_yaml.empty()) << TEST_COLLISION_YAML_PATH;

  DhKinematicsModel kinematics(
      {"swing_joint", "boom_joint", "stick_joint", "bucket_joint"});
  const auto kinematics_initialized =
      kinematics.InitializeFromString(dh_yaml);
  ASSERT_TRUE(kinematics_initialized.ok) << kinematics_initialized.message;

  FclCollisionChecker checker;
  CollisionModelConfig config;
  config.joint_names = kinematics.joint_names();
  config.model_source = collision_yaml;
  config.model_format = "yaml";
  config.kinematics_reference_frame = "base_link";
  const auto initialized = checker.Initialize(config, &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;
  checker.SetAllowedCollisionPairs({{"base_link", "boom_link"},
                                    {"base_link", "stick_link"},
                                    {"base_link", "bucket_link"},
                                    {"swing_link", "stick_link"},
                                    {"swing_link", "bucket_link"}});

  const JointPositions home = {0.0, -0.35, 0.45, -0.40};
  const CollisionCheckResult self_result = checker.CheckSelfCollision(home);
  ASSERT_TRUE(self_result.valid) << self_result.message;
  EXPECT_FALSE(self_result.collision) << self_result.message;

  Pose3 bucket_pose;
  ASSERT_TRUE(
      kinematics.GetLinkPose(home, "bucket_link", &bucket_pose).ok);
  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakeSphere(
      "bucket_obstacle", bucket_pose.position.x, bucket_pose.position.y,
      bucket_pose.position.z, 0.6));
  const CollisionCheckResult environment_result =
      checker.CheckEnvironmentCollision(home, scene);
  ASSERT_TRUE(environment_result.valid) << environment_result.message;
  EXPECT_TRUE(environment_result.collision);

  excavator_interfaces::msg::PlanningScene far_scene;
  far_scene.header.frame_id = "base_link";
  far_scene.objects.push_back(
      MakeSphere("far_obstacle", 50.0, 50.0, 50.0, 0.5));
  const CollisionDistanceResult distance =
      checker.ComputeDistanceToObstacle(home, far_scene, "far_obstacle");
  ASSERT_TRUE(distance.valid) << distance.message;
  EXPECT_GT(distance.distance, 0.0);
}

}  // namespace
}  // namespace planning
}  // namespace tramcar
