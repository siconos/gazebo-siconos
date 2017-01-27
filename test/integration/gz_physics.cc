/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class GzPhysics : public ServerFixture,
                  public testing::WithParamInterface<const char*>

{
};

/////////////////////////////////////////////////
// \brief Test setting the gravity.
TEST_F(GzPhysics, Gravity)
{
  Load("worlds/empty.world");

  // Spawn a box that will eventually float up.
  SpawnBox("box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector3d(0, 0, .5),
      ignition::math::Vector3d::Zero);
  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Get a pointer to the model
  physics::ModelPtr model = world->ModelByName("box");
  ASSERT_TRUE(model != NULL);

  EXPECT_EQ(model->WorldPose(), ignition::math::Pose3d(0, 0, .5, 0, 0, 0));

  SetPause(true);

  // Change gravity
  custom_exec("gz physics -g 0,0,9.8");

  world->Step(100);

  EXPECT_GT(model->WorldPose().Pos().Z(), 0.5);
}

/////////////////////////////////////////////////
// \brief Test setting the step size.
TEST_F(GzPhysics, StepSize)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  ASSERT_TRUE(world->Physics() != NULL);

  // Change step size
  custom_exec("gz physics -s 0.002");
  EXPECT_NEAR(world->Physics()->GetMaxStepSize(), 0.002, 1e-5);

  // Change step size
  custom_exec("gz physics -s 0.001");
  EXPECT_NEAR(world->Physics()->GetMaxStepSize(), 0.001, 1e-5);
}

/////////////////////////////////////////////////
// \brief Test setting the iterations.
TEST_F(GzPhysics, Iters)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  ASSERT_TRUE(world->Physics() != NULL);

  // Change iterations
  {
    custom_exec("gz physics -i 35");
    boost::any iters = world->Physics()->GetParam("iters");
    EXPECT_EQ(boost::any_cast<int>(iters), 35);
  }

  // Change iterations
  {
    custom_exec("gz physics -i 200");
    boost::any iters = world->Physics()->GetParam("iters");
    EXPECT_EQ(boost::any_cast<int>(iters), 200);
  }
}

/////////////////////////////////////////////////
// \brief Test setting the physics profile for each physics engine.
TEST_P(GzPhysics, Profile)
{
  Load("test/worlds/presets.world", false, GetParam());

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  ASSERT_TRUE(world->Physics() != NULL);

  // Check initial profile and step size
  EXPECT_EQ(world->PresetMgr()->CurrentProfile(), "preset_1");
  EXPECT_FLOAT_EQ(world->Physics()->GetMaxStepSize(), 0.01);

  // Change profile
  custom_exec("gz physics -o preset_2");
  EXPECT_EQ(world->PresetMgr()->CurrentProfile(), "preset_2");
  EXPECT_FLOAT_EQ(world->Physics()->GetMaxStepSize(), 0.02);

  // Change profile and override step size
  custom_exec("gz physics -o preset_1 -s 0.001");
  EXPECT_EQ(world->PresetMgr()->CurrentProfile(), "preset_1");
  EXPECT_FLOAT_EQ(world->Physics()->GetMaxStepSize(), 0.001);
}

/////////////////////////////////////////////////
// \brief Test setting the update rate.
TEST_F(GzPhysics, UpdateRate)
{
  Load("worlds/empty.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  ASSERT_TRUE(world->Physics() != NULL);

  // Change update rate
  custom_exec("gz physics -u 2.0");
  EXPECT_NEAR(world->Physics()->GetRealTimeUpdateRate(), 2.0, 1e-3);

  // Change update rate
  custom_exec("gz physics -u 0.5");
  EXPECT_NEAR(world->Physics()->GetRealTimeUpdateRate(), 0.5, 1e-3);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, GzPhysics, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
