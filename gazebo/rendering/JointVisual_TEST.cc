/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <ignition/math/Pose3.hh>

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class JointVisual_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(JointVisual_TEST, JointVisualTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // get scene visual child count before we create any visuals
  EXPECT_TRUE(scene->WorldVisual() != NULL);
  unsigned int count = scene->WorldVisual()->GetChildCount();

  // create a fake child visual where the joint visual will be attached to
  gazebo::rendering::VisualPtr childVis;
  childVis.reset(
      new gazebo::rendering::Visual("child", scene->WorldVisual()));

  // create a joint message for testing
  gazebo::msgs::JointPtr jointMsg;
  jointMsg.reset(new gazebo::msgs::Joint);
  jointMsg->set_parent(scene->WorldVisual()->Name());
  jointMsg->set_parent_id(scene->WorldVisual()->GetId());
  jointMsg->set_child(childVis->Name());
  jointMsg->set_child_id(childVis->GetId());
  jointMsg->set_name("test_joint");
  jointMsg->set_id(11111);
  msgs::Set(jointMsg->mutable_pose(),
      ignition::math::Pose3d(1, 2, 3, 1.57, 1.57, 0));
  jointMsg->set_type(msgs::Joint::REVOLUTE2);
  jointMsg->add_angle(1.2);
  {
    msgs::Axis *axis1 = jointMsg->mutable_axis1();
    msgs::Set(axis1->mutable_xyz(), ignition::math::Vector3d(0, 1, 0));
    axis1->set_limit_lower(-1.2);
    axis1->set_limit_upper(2.3);
    axis1->set_limit_effort(6);
    axis1->set_limit_velocity(1);
    axis1->set_damping(true);
    axis1->set_friction(true);
    axis1->set_use_parent_model_frame(true);
  }
  jointMsg->add_angle(-1.2);
  {
    msgs::Axis *axis2 = jointMsg->mutable_axis2();
    msgs::Set(axis2->mutable_xyz(), ignition::math::Vector3d(0, 0, 1));
    axis2->set_limit_lower(-1.2);
    axis2->set_limit_upper(-0.3);
    axis2->set_limit_effort(3);
    axis2->set_limit_velocity(2);
    axis2->set_damping(false);
    axis2->set_friction(false);
    axis2->set_use_parent_model_frame(false);
  }

  // test calling constructor and Load functions and make sure
  // there are no segfaults
  gazebo::rendering::JointVisualPtr jointVis(
      new gazebo::rendering::JointVisual(
      "model_GUIONLY_joint_vis", childVis));
  jointVis->Load(jointMsg);

  // pose matches the message's pose
  EXPECT_EQ(jointVis->Pose(),
      ignition::math::Pose3d(1, 2, 3, 1.57, 1.57, 0));

  // has axis 1 and it is visible
  EXPECT_TRUE(jointVis->GetArrowVisual() != NULL);
  EXPECT_TRUE(jointVis->GetArrowVisual()->GetVisible());

  // has axis 2 and it is visible
  EXPECT_TRUE(jointVis->GetParentAxisVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual()->GetVisible());

  // update pose from a message
  jointMsg.reset(new gazebo::msgs::Joint);
  jointMsg->set_name("test_joint");
  msgs::Set(jointMsg->mutable_pose(),
      ignition::math::Pose3d(3, 2, 1, 0, 1.57, 0));
  jointVis->UpdateFromMsg(jointMsg);

  // pose properly updated
  EXPECT_EQ(jointVis->Pose(),
      ignition::math::Pose3d(3, 2, 1, 0, 1.57, 0));

  // axis 1 still visible
  EXPECT_TRUE(jointVis->GetArrowVisual() != NULL);
  EXPECT_TRUE(jointVis->GetArrowVisual()->GetVisible());

  // axis 2 still visible
  EXPECT_TRUE(jointVis->GetParentAxisVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual()->GetVisible());

  // update joint type and axis from a message
  jointMsg.reset(new gazebo::msgs::Joint);
  jointMsg->set_name("test_joint");
  jointMsg->set_type(msgs::Joint::REVOLUTE);
  jointMsg->add_angle(2.5);
  {
    msgs::Axis *axis1 = jointMsg->mutable_axis1();
    msgs::Set(axis1->mutable_xyz(), ignition::math::Vector3d(1, 0, 0));
    axis1->set_limit_lower(-1.2);
    axis1->set_limit_upper(2.3);
    axis1->set_limit_effort(6);
    axis1->set_limit_velocity(1);
    axis1->set_damping(true);
    axis1->set_friction(true);
    axis1->set_use_parent_model_frame(false);
  }
  jointVis->UpdateFromMsg(jointMsg);

  // pose hasn't changed
  EXPECT_EQ(jointVis->Pose(),
      ignition::math::Pose3d(3, 2, 1, 0, 1.57, 0));

  // axis 1 still visible
  EXPECT_TRUE(jointVis->GetArrowVisual() != NULL);
  EXPECT_TRUE(jointVis->GetArrowVisual()->GetVisible());

  // axis 2 still there but not visible
  EXPECT_TRUE(jointVis->GetParentAxisVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual() != NULL);
  EXPECT_FALSE(jointVis->GetParentAxisVisual()->GetArrowVisual()->GetVisible());

  // update joint type and pose from a message
  jointMsg.reset(new gazebo::msgs::Joint);
  jointMsg->set_name("test_joint");
  msgs::Set(jointMsg->mutable_pose(),
      ignition::math::Pose3d(0, -2, 1, -1.57, 1.57, 0));
  jointMsg->set_type(msgs::Joint::BALL);
  jointVis->UpdateFromMsg(jointMsg);

  // new pose
  EXPECT_EQ(jointVis->Pose(),
      ignition::math::Pose3d(0, -2, 1, -1.57, 1.57, 0));

  // axis 1 still there but not visible
  EXPECT_TRUE(jointVis->GetArrowVisual() != NULL);
  EXPECT_FALSE(jointVis->GetArrowVisual()->GetVisible());

  // axis 2 still there but not visible
  EXPECT_TRUE(jointVis->GetParentAxisVisual() != NULL);
  EXPECT_TRUE(jointVis->GetParentAxisVisual()->GetArrowVisual() != NULL);
  EXPECT_FALSE(jointVis->GetParentAxisVisual()->GetArrowVisual()->GetVisible());

  // test destroying the visuals
  childVis->Fini();
  EXPECT_EQ(childVis->GetChildCount(), 0u);
  EXPECT_EQ(jointVis->GetChildCount(), 0u);

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
