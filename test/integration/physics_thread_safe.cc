/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsThreadSafeTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Load a blank world and try to change gravity.
  /// The test passes if it doesn't seg-fault.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void BlankWorld(const std::string &_physicsEngine);

  /// \brief Load the revolute joint test world, unthrottle the update rate,
  /// and repeately call Link::Get* functions to verify thread safety.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void LinkGet(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsThreadSafeTest::BlankWorld(const std::string &_physicsEngine)
{
  Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // The following lines cause a seg-fault on revision 031749b
  // This test passes if it doesn't seg-fault.
  auto g = world->Gravity();
  physics->SetGravity(g);
}

/////////////////////////////////////////////////
void PhysicsThreadSafeTest::LinkGet(const std::string &_physicsEngine)
{
  Load("worlds/revolute_joint_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Unthrottle the update rate
  physics->SetRealTimeUpdateRate(0);

  std::string modelName = "pendulum_0deg";
  std::string linkName = "lower_link";

  physics::ModelPtr model = world->ModelByName(modelName);
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink(linkName);
  ASSERT_TRUE(link != NULL);

  // Start the simulation
  world->SetPaused(false);

  // Run for 5 seconds of sim time
  while (world->SimTime().sec < 5)
  {
    // Call these functions repeatedly
    // Test passes if it doesn't abort early
    ignition::math::Vector3d vel = link->WorldLinearVel();
    vel += link->WorldLinearVel(ignition::math::Vector3d::Zero);
    vel += link->WorldLinearVel(ignition::math::Vector3d::Zero,
        ignition::math::Quaterniond::Identity);
    vel += link->WorldCoGLinearVel();
    vel += link->WorldAngularVel();
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsThreadSafeTest, BlankWorld)
{
  BlankWorld(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsThreadSafeTest, LinkGet)
{
  LinkGet(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsThreadSafeTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
