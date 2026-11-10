#include "excavator_collision/fcl_collision_checker.hpp"
#include "excavator_collision/obstacle_adapter.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "excavator_interfaces/msg/planning_scene.hpp"
#include "kinematics/dh_kinematics_model.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace excavator {
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

excavator_interfaces::msg::CollisionObject MakePointCloudObject(
    const std::string& id, const std::vector<Eigen::Vector3d>& points) {
  excavator_interfaces::msg::CollisionObject object;
  object.id = id;
  object.enabled = true;
  object.pose.orientation.w = 1.0;

  auto& cloud = object.point_cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  cloud.fields.resize(3);
  for (std::size_t index = 0; index < 3; ++index) {
    cloud.fields[index].name =
        index == 0 ? "x" : (index == 1 ? "y" : "z");
    cloud.fields[index].offset = static_cast<std::uint32_t>(index * 4);
    cloud.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[index].count = 1;
  }

  cloud.data.resize(cloud.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const std::array<float, 3> coordinates = {
        static_cast<float>(points[index].x()),
        static_cast<float>(points[index].y()),
        static_cast<float>(points[index].z())};
    std::memcpy(cloud.data.data() + index * cloud.point_step,
                coordinates.data(), sizeof(coordinates));
  }
  return object;
}

std::string AdaptedId(const std::string& id) {
  return std::to_string(ObstacleAdapter::ObjectId(id));
}

Obstacle ConvertObstacle(
    const excavator_interfaces::msg::CollisionObject& source) {
  Obstacle obstacle;
  const ValidationResult result = ObstacleAdapter::Convert(source, &obstacle);
  EXPECT_TRUE(result.ok) << result.message;
  return obstacle;
}

std::vector<Obstacle> ConvertObstacles(
    const excavator_interfaces::msg::PlanningScene& scene) {
  std::vector<Obstacle> obstacles;
  const ValidationResult result =
      ObstacleAdapter::Convert(scene.objects, &obstacles);
  EXPECT_TRUE(result.ok) << result.message;
  return obstacles;
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

  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckCollision({0.0}, obstacles);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_EQ(result.second_object, AdaptedId("obstacle"));
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

  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckCollision({0.0}, obstacles);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_FALSE(result.collision);
}

TEST(FclCollisionCheckerTest, DetectsPointCloudEnvironmentCollision) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakePointCloudObject(
      "cloud", {Eigen::Vector3d(0.45, 0.0, 0.0),
                Eigen::Vector3d(0.40, 0.05, 0.0)}));

  CollisionQueryOptions options;
  options.obstacle_type = ObstacleType::kPointCloud;
  options.point_cloud_resolution = 0.02;

  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckEnvironmentCollision({0.0}, obstacles, options);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_EQ(result.second_object, AdaptedId("cloud"));
}

TEST(FclCollisionCheckerTest, RejectsEmptyPointCloud) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  scene.objects.push_back(MakePointCloudObject("empty_cloud", {}));

  CollisionQueryOptions options;
  options.obstacle_type = ObstacleType::kPointCloud;

  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckEnvironmentCollision({0.0}, obstacles, options);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.message.find("point cloud is empty"), std::string::npos);
}

TEST(FclCollisionCheckerTest, ComputesDistanceToPointCloud) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  const auto obstacle = MakePointCloudObject(
      "far_cloud", {Eigen::Vector3d(10.0, 10.0, 10.0)});
  CollisionQueryOptions options;
  options.obstacle_type = ObstacleType::kPointCloud;

  const CollisionDistanceResult distance =
      checker.ComputeDistance({0.0}, ConvertObstacle(obstacle), options);
  ASSERT_TRUE(distance.valid) << distance.message;
  EXPECT_GT(distance.distance, 1.0);
  EXPECT_EQ(distance.obstacle_id, AdaptedId("far_cloud"));
}

TEST(FclCollisionCheckerTest, ChecksPointCloudPathWithSingleObstacle) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  // A point on the tool sphere's swept arc, clear at both path endpoints.
  const double angle = 0.5;
  const auto obstacle = MakePointCloudObject(
      "swept_cloud",
      {Eigen::Vector3d(1.4 * std::cos(angle), 1.4 * std::sin(angle), 0.0)});
  CollisionQueryOptions options;
  options.obstacle_type = ObstacleType::kPointCloud;

  JointPath path;
  path.joint_names = {"arm_joint"};
  path.points = {{-1.0}, {1.0}};
  const CollisionCheckResult result = checker.CheckPathCollision(
      path, ConvertObstacle(obstacle), 0.05, options);
  ASSERT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.checked_states, 1.0);
}

TEST(FclCollisionCheckerTest, DetectsNonAdjacentSelfCollision) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckCollision({3.141592653589793}, obstacles);
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
  checker.SetAllowedCollision("base_link", "tool_link", true);

  excavator_interfaces::msg::PlanningScene scene;
  scene.header.frame_id = "base_link";
  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckCollision({3.141592653589793}, obstacles);
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
  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult result =
      checker.CheckCollision({0.0}, obstacles);
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
  const auto obstacles = ConvertObstacles(scene);

  JointPath path;
  path.joint_names = {"arm_joint"};
  path.points = {{-1.0}, {1.0}};
  const CollisionCheckResult result =
      checker.CheckPathCollision(path, obstacles, 0.05);
  EXPECT_TRUE(result.valid) << result.message;
  EXPECT_TRUE(result.collision);
  EXPECT_GT(result.checked_states, 1.0);

  const CollisionCheckResult obstacle_result =
      checker.CheckPathCollision(path, obstacles.front(), 0.05);
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
  const auto obstacles = ConvertObstacles(scene);

  const CollisionCheckResult environment_result =
      checker.CheckEnvironmentCollision({0.0}, obstacles);
  ASSERT_TRUE(environment_result.valid) << environment_result.message;
  EXPECT_FALSE(environment_result.collision);

  const CollisionDistanceResult distance =
      checker.ComputeDistance({0.0}, obstacles.front());
  ASSERT_TRUE(distance.valid) << distance.message;
  EXPECT_GT(distance.distance, 0.0);
  EXPECT_EQ(distance.obstacle_id, AdaptedId("distance_obstacle"));
  EXPECT_FALSE(distance.robot_link.empty());
}

TEST(FclCollisionCheckerTest, RejectsEmptyDistanceObstacleId) {
  SingleJointKinematics kinematics;
  FclCollisionChecker checker;
  const auto initialized = InitializeChecker(&checker, kCollisionYaml,
                                             &kinematics);
  ASSERT_TRUE(initialized.ok) << initialized.message;

  const CollisionDistanceResult distance =
      checker.ComputeDistance({0.0}, Obstacle{});
  EXPECT_FALSE(distance.valid);
  EXPECT_NE(distance.message.find("must be non-negative"),
            std::string::npos);
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
  for (const auto& pair :
       std::vector<std::pair<std::string, std::string>>{
           {"base_link", "boom_link"},
           {"base_link", "stick_link"},
           {"base_link", "bucket_link"},
           {"swing_link", "stick_link"},
           {"swing_link", "bucket_link"}}) {
    checker.SetAllowedCollision(pair.first, pair.second, true);
  }

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
  const auto obstacles = ConvertObstacles(scene);
  const CollisionCheckResult environment_result =
      checker.CheckEnvironmentCollision(home, obstacles);
  ASSERT_TRUE(environment_result.valid) << environment_result.message;
  EXPECT_TRUE(environment_result.collision);

  excavator_interfaces::msg::PlanningScene far_scene;
  far_scene.header.frame_id = "base_link";
  far_scene.objects.push_back(
      MakeSphere("far_obstacle", 50.0, 50.0, 50.0, 0.5));
  const auto far_obstacles = ConvertObstacles(far_scene);
  const CollisionDistanceResult distance =
      checker.ComputeDistance(home, far_obstacles.front());
  ASSERT_TRUE(distance.valid) << distance.message;
  EXPECT_GT(distance.distance, 0.0);
}

}  // namespace
}  // namespace planning
}  // namespace excavator
