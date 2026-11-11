#include <gtest/gtest.h>

#include "excavator_kinematics/kinematics_model.hpp"

namespace tramcar::planning {
namespace {

ModelDescription MakeModel() {
  ModelDescription description;
  description.name = "test";
  description.root_link = "base";
  description.links = {{"base"}, {"link1"}, {"link2"}};

  JointModel first;
  first.name = "joint1";
  first.type = JointType::kRevolute;
  first.parent_link = "base";
  first.child_link = "link1";
  first.axis = Eigen::Vector3d::UnitZ();

  JointModel second;
  second.name = "joint2";
  second.type = JointType::kPrismatic;
  second.parent_link = "link1";
  second.child_link = "link2";
  second.axis = Eigen::Vector3d::UnitX();
  second.limit.has_position_limits = true;
  second.limit.lower = 0.0;
  second.limit.upper = 1.0;

  description.joints = {first, second};
  description.joint_order = {"joint1", "joint2"};
  return description;
}

}  // namespace

TEST(KinematicsModelTest, InitializesAndPreservesJointOrder) {
  KinematicsModel model;
  const KinematicsResult result = model.Initialize(MakeModel());
  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_EQ(model.root_link(), "base");
  ASSERT_EQ(model.GetActiveJointNames().size(), 2U);
  EXPECT_EQ(model.GetActiveJointNames()[0], "joint1");
  EXPECT_EQ(model.GetActiveJointNames()[1], "joint2");

  std::size_t index = 0;
  ASSERT_TRUE(model.GetActiveJointIndex("joint2", &index));
  EXPECT_EQ(index, 1U);
  EXPECT_NE(model.GetParentJoint("link2"), nullptr);
}

TEST(KinematicsModelTest, RejectsDisconnectedTopology) {
  ModelDescription description = MakeModel();
  description.links.push_back({"orphan"});
  KinematicsModel model;
  const KinematicsResult result = model.Initialize(description);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, KinematicsStatus::kInvalidArgument);
}

TEST(KinematicsModelTest, RejectsIncompleteJointOrder) {
  ModelDescription description = MakeModel();
  description.joint_order = {"joint1"};
  KinematicsModel model;
  const KinematicsResult result = model.Initialize(description);
  EXPECT_FALSE(result.ok());
}

}  // namespace tramcar::planning
