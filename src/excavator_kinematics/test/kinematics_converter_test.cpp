#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "excavator_kinematics/kinematics_model.hpp"
#include "excavator_kinematics/model_converter/dh_model_converter.hpp"
#include "excavator_kinematics/model_converter/model_config_parser.hpp"
#include "excavator_kinematics/model_converter/urdf_model_converter.hpp"

namespace excavator {
namespace planning {

TEST(KinematicsConverterTest, ParsesCustomModelYaml) {
  const YAML::Node yaml = YAML::Load(R"(
kinematics_model:
  name: small_arm
  root_link: base
  joint_order: [joint1]
  links:
    - name: base
    - name: tip
  joints:
    - name: joint1
      type: revolute
      parent_link: base
      child_link: tip
      origin:
        xyz: [0.0, 0.0, 0.5]
        rpy: [0.0, 0.0, 0.0]
      axis: [0.0, 0.0, 1.0]
      limit:
        lower: -1.0
        upper: 1.0
        velocity: 0.5
)");
  ModelDescription description;
  const KinematicsResult parsed =
      ModelConfigParser::Parse(yaml, &description);
  ASSERT_TRUE(parsed.ok()) << parsed.message;
  KinematicsModel model;
  const KinematicsResult initialized = model.Initialize(description);
  ASSERT_TRUE(initialized.ok()) << initialized.message;
  EXPECT_EQ(model.GetActiveJointNames().size(), 1U);
}

TEST(KinematicsConverterTest, ConvertsModifiedDhToProjectModel) {
  const YAML::Node yaml = YAML::Load(R"(
convention: modified_dh
root_link: base_link
base:
  xyz: [0.0, 0.0, 1.0]
  rpy: [0.0, 0.0, 0.0]
joints:
  - name: swing_joint
    link: swing_link
    theta_sign: -1.0
    alpha_prev: 0.0
    a_prev: 0.0
    d: 0.0
    theta_offset: 0.0
    limits: [-3.0, 3.0]
    velocity: 0.5
links:
  swing_link:
    xyz: [0.0, 0.0, 0.0]
    rpy: [0.0, 0.0, 0.0]
  bucket_tip:
    xyz: [1.0, 0.0, 0.0]
    rpy: [0.0, 0.0, 0.0]
)");
  ModelDescription description;
  const KinematicsResult converted =
      DhModelConverter::FromYaml(yaml, &description);
  ASSERT_TRUE(converted.ok()) << converted.message;
  KinematicsModel model;
  const KinematicsResult initialized = model.Initialize(description);
  ASSERT_TRUE(initialized.ok()) << initialized.message;
  EXPECT_NE(model.GetLink("bucket_tip"), nullptr);
  EXPECT_EQ(model.GetActiveJointNames().front(), "swing_joint");
}

TEST(KinematicsConverterTest, ConvertsUrdfRootAndJointFrames) {
  const std::string urdf = R"(
<robot name="test">
  <link name="base"/>
  <link name="tip"/>
  <link name="hydraulic_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="0.5"/>
  </joint>
  <joint name="hydraulic_joint" type="prismatic">
    <parent link="base"/>
    <child link="hydraulic_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1" effort="10" velocity="0.2"/>
  </joint>
</robot>)";
  ModelDescription description;
  const KinematicsResult converted =
      UrdfModelConverter::FromString(urdf, &description, {"joint1"});
  ASSERT_TRUE(converted.ok()) << converted.message;
  EXPECT_EQ(description.root_link, "base");
  ASSERT_EQ(description.joint_order.size(), 1U);
  EXPECT_EQ(description.joint_order.front(), "joint1");
  EXPECT_EQ(description.joints.size(), 2U);
  KinematicsModel model;
  const KinematicsResult initialized = model.Initialize(description);
  ASSERT_TRUE(initialized.ok()) << initialized.message;
}

}  // namespace planning
}  // namespace excavator
