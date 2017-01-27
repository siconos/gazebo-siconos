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
#include <ignition/math/Pose3.hh>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue1124Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test for issue #1124 by directly setting a model pose
TEST_F(Issue1124Test, SetModelPose)
{
  Load("worlds/box_plane_low_friction_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  physics::CollisionPtr coll = link->GetCollision("collision");
  ASSERT_TRUE(coll != NULL);

  // The start pose should be centered at the origin
  EXPECT_EQ(coll->WorldPose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  // The world->steps are not necessary. There are included to
  // err on the side of caution.
  world->Step(1);
  model->SetWorldPose(ignition::math::Pose3d(2, 2, 0.5, 0, 0, 0));
  world->Step(1);

  EXPECT_EQ(model->WorldPose(), ignition::math::Pose3d(2, 2, 0.5, 0, 0, 0));
  world->Step(1);

  EXPECT_EQ(link->WorldPose(), ignition::math::Pose3d(2, 2, 0.5, 0, 0, 0));
  world->Step(1);

  // The new pose should be centered a the new model location
  EXPECT_EQ(coll->WorldPose(), ignition::math::Pose3d(2, 2, 0.5, 0, 0, 0));
  world->Step(1);
}

/////////////////////////////////////////////////
// \brief Test for issue #1124 by directly setting a link pose
TEST_F(Issue1124Test, SetLinkPose)
{
  Load("worlds/box_plane_low_friction_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  physics::CollisionPtr coll = link->GetCollision("collision");
  ASSERT_TRUE(coll != NULL);

  // The start pose should be centered at the origin
  EXPECT_EQ(coll->WorldPose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  link->SetWorldPose(ignition::math::Pose3d(-4, 5, 0.5, 0, 0, 0));

  // The new pose should be centered a the new model location
  EXPECT_EQ(coll->WorldPose(), ignition::math::Pose3d(-4, 5, 0.5, 0, 0, 0));
}

/////////////////////////////////////////////////
// \brief Test for issue #1124 using the physics engine
TEST_F(Issue1124Test, MovingPose)
{
  Load("worlds/box_plane_low_friction_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  physics::CollisionPtr coll = link->GetCollision("collision");
  ASSERT_TRUE(coll != NULL);

  // The start pose should be centered at the origin
  EXPECT_EQ(coll->WorldPose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  // Move the model by adding a force. This will exercise the physics engine
  // pose change callbacks.
  link->AddForce(ignition::math::Vector3d(100, 0, 0.0));
  world->Step(5000);

  EXPECT_GT(model->WorldPose().Pos().X(), 1.0);
  EXPECT_EQ(model->WorldPose(), coll->WorldPose());
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
