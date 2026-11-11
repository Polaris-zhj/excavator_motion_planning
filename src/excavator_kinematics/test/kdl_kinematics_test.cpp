#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "excavator_kinematics/kinematics_solver_factory.hpp"

namespace tramcar::planning {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<KinematicsModel> MakePlanarArm() {
  ModelDescription description;
  description.name = "planar_3r";
  description.root_link = "base";
  description.links = {{"base"}, {"link1"}, {"link2"}, {"link3"}, {"tip"}};

  JointModel joint1;
  joint1.name = "joint1";
  joint1.type = JointType::kRevolute;
  joint1.parent_link = "base";
  joint1.child_link = "link1";
  joint1.axis = Eigen::Vector3d::UnitZ();

  JointModel joint2;
  joint2.name = "joint2";
  joint2.type = JointType::kRevolute;
  joint2.parent_link = "link1";
  joint2.child_link = "link2";
  joint2.origin.translation = Eigen::Vector3d(3.0, 0.0, 0.0);
  joint2.axis = Eigen::Vector3d::UnitZ();

  JointModel joint3;
  joint3.name = "joint3";
  joint3.type = JointType::kRevolute;
  joint3.parent_link = "link2";
  joint3.child_link = "link3";
  joint3.origin.translation = Eigen::Vector3d(2.5, 0.0, 0.0);
  joint3.axis = Eigen::Vector3d::UnitZ();

  JointModel tool;
  tool.name = "tool_fixed";
  tool.type = JointType::kFixed;
  tool.parent_link = "link3";
  tool.child_link = "tip";
  tool.origin.translation = Eigen::Vector3d(1.0, 0.0, 0.0);

  description.joints = {joint1, joint2, joint3, tool};
  description.joint_order = {"joint1", "joint2", "joint3"};

  auto model = std::make_shared<KinematicsModel>();
  const KinematicsResult result = model->Initialize(description);
  if (!result.ok()) return nullptr;
  return model;
}

}  // namespace

TEST(KdlKinematicsTest, ComputesRootRelativeForwardKinematics) {
  auto model = MakePlanarArm();
  ASSERT_NE(model, nullptr);
  auto solver = KinematicsSolverFactory::Instance().Create();
  ASSERT_NE(solver, nullptr);
  ASSERT_TRUE(solver->Initialize(model).ok());

  JointVector q(3);
  q << kPi / 6.0, -kPi / 3.0, kPi / 9.0;
  Pose pose;
  const KinematicsResult result = solver->GetLinkPose(q, "tip", &pose);
  ASSERT_TRUE(result.ok()) << result.message;

  EXPECT_NEAR(pose.position.x(), 5.748, 2.0e-3);
  EXPECT_NEAR(pose.position.y(), 0.0764, 2.0e-3);
  const Eigen::Quaterniond expected(Eigen::AngleAxisd(
      -kPi / 18.0, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(pose.orientation.angularDistance(expected), 0.0, 1.0e-6);
}

TEST(KdlKinematicsTest, ComputesJacobianInRootFrameAndModelOrder) {
  auto model = MakePlanarArm();
  ASSERT_NE(model, nullptr);
  auto solver = KinematicsSolverFactory::Instance().Create();
  ASSERT_NE(solver, nullptr);
  ASSERT_TRUE(solver->Initialize(model).ok());

  JointVector q = JointVector::Zero(3);
  JacobianMatrix jacobian;
  const KinematicsResult result = solver->GetJacobian(q, "tip", &jacobian);
  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_EQ(jacobian.rows(), 6);
  ASSERT_EQ(jacobian.cols(), 3);

  EXPECT_NEAR(jacobian(0, 0), 0.0, 1.0e-9);
  EXPECT_NEAR(jacobian(1, 0), 6.5, 1.0e-9);
  EXPECT_NEAR(jacobian(1, 1), 3.5, 1.0e-9);
  EXPECT_NEAR(jacobian(1, 2), 1.0, 1.0e-9);
  EXPECT_NEAR(jacobian(5, 0), 1.0, 1.0e-9);
  EXPECT_NEAR(jacobian(5, 1), 1.0, 1.0e-9);
  EXPECT_NEAR(jacobian(5, 2), 1.0, 1.0e-9);
}

TEST(KdlKinematicsTest, ComputesRelativePoseBetweenLinks) {
  auto model = MakePlanarArm();
  ASSERT_NE(model, nullptr);
  auto solver = KinematicsSolverFactory::Instance().Create();
  ASSERT_NE(solver, nullptr);
  ASSERT_TRUE(solver->Initialize(model).ok());

  JointVector q = JointVector::Zero(3);
  Pose relative_pose;
  const KinematicsResult result =
      solver->GetRelativePose(q, "link2", "tip", &relative_pose);
  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_NEAR(relative_pose.position.x(), 3.5, 1.0e-9);
  EXPECT_NEAR(relative_pose.position.y(), 0.0, 1.0e-9);
}

TEST(KdlKinematicsTest, SolvesIkAndVerifiesFk) {
  auto model = MakePlanarArm();
  ASSERT_NE(model, nullptr);
  auto solver = KinematicsSolverFactory::Instance().Create();
  ASSERT_NE(solver, nullptr);
  ASSERT_TRUE(solver->Initialize(model).ok());

  JointVector target_q(3);
  target_q << 0.2, -0.5, 0.3;
  Pose target;
  ASSERT_TRUE(solver->GetLinkPose(target_q, "tip", &target).ok());

  IkRequest request;
  request.target_link = "tip";
  request.target_pose = target;
  request.initial_q = target_q;
  request.initial_q(0) += 0.02;
  request.initial_q(1) -= 0.02;
  request.initial_q(2) += 0.01;
  request.options.position_tolerance = 1.0e-6;
  request.options.orientation_tolerance = 1.0e-6;
  const IkResult result = solver->InverseKinematics(request);
  ASSERT_TRUE(result.result.ok()) << result.result.message;
  EXPECT_NEAR(result.position_error, 0.0, 1.0e-6);
  EXPECT_NEAR(result.orientation_error, 0.0, 1.0e-6);
}

TEST(KdlKinematicsTest, FactoryCreatesKdlBackend) {
  KinematicsSolverConfig config;
  auto solver = KinematicsSolverFactory::Instance().Create(config);
  ASSERT_NE(solver, nullptr);
  EXPECT_FALSE(KinematicsSolverFactory::Instance().AvailableBackends().empty());
}

}  // namespace tramcar::planning
