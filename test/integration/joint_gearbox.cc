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

#include <gtest/gtest.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"

#define TOL 1e-6
using namespace gazebo;

class ODEGearboxJoint_TEST : public ServerFixture
{
  public: void GearboxTest(const std::string &_physicsEngine);
  public: void SetGearboxRatio(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// GearboxTest:
// start gearbox.world, apply balancing forces across geared members,
// check for equilibrium.
////////////////////////////////////////////////////////////////////////
void ODEGearboxJoint_TEST::GearboxTest(const std::string &_physicsEngine)
{
  // load gearbox world
  Load("worlds/gearbox.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("model_1");
  physics::JointPtr joint0 = model->GetJoint("joint_02");
  physics::JointPtr joint1 = model->GetJoint("joint_12");
  physics::JointPtr joint3 = model->GetJoint("joint_23");

  physics::JointPtr gearboxJoint = model->GetJoint("joint_13");
  ASSERT_TRUE(gearboxJoint != NULL);
  ASSERT_TRUE(gearboxJoint->HasType(physics::Base::GEARBOX_JOINT));
  double gearboxRatio = gearboxJoint->GetParam("gearbox_ratio", 0);
  EXPECT_NEAR(gearboxRatio, -1.5, TOL);
  double force3 = 1.0;
  double force1 = force3 * gearboxRatio;

  // repeat the same test for various joint0 angles (thanks to issue 1703)
  int directions = 20;
  double increments = M_PI/4.0;
  for (int j = -directions; j < directions; j+=2)
  {
    // reset world
    world->Reset();
    // set joint0 angle
    double angle = static_cast<double>(j) * increments;
    joint0->SetPosition(0, angle);
    gzdbg << "j [" << j
          << "] angle [" << angle
          << "]\n";

    int steps = 10000;
    for (int i = 0; i < steps; ++i)
    {
      joint1->SetForce(0, force1);
      joint3->SetForce(0, force3);
      world->Step(1);
      if (i%1000 == 0)
        gzdbg << "gearbox time [" << world->SimTime().Double()
              << "] vel [" << joint1->GetVelocity(0)
              << "] pose [" << joint1->Position(0)
              << "] vel [" << joint3->GetVelocity(0)
              << "] pose [" << joint3->Position(0)
              << "]\n";
      EXPECT_NEAR(joint1->GetVelocity(0), 0, TOL);
      EXPECT_NEAR(joint3->GetVelocity(0), 0, TOL);
      EXPECT_NEAR(joint1->Position(0), 0, TOL);
      EXPECT_NEAR(joint3->Position(0), 0, TOL);
    }

    // slight imbalance
    for (int i = 0; i < steps; ++i)
    {
      joint1->SetForce(0, -force3);
      joint3->SetForce(0,  force3);
      world->Step(1);
      if (i%1000 == 0)
        gzdbg << "gearbox time [" << world->SimTime().Double()
              << "] vel [" << joint1->GetVelocity(0)
              << "] pose [" << joint1->Position(0)
              << "] vel [" << joint3->GetVelocity(0)
              << "] pose [" << joint3->Position(0)
              << "]\n";
      EXPECT_GT(joint1->GetVelocity(0), 0);
      EXPECT_GT(joint3->GetVelocity(0), 0);
      EXPECT_GT(joint1->Position(0), 0);
      EXPECT_GT(joint3->Position(0), 0);
      EXPECT_NEAR(joint1->GetVelocity(0)*gearboxRatio, -joint3->GetVelocity(0),
        TOL);
      EXPECT_NEAR(joint1->Position(0)*gearboxRatio,
                 -joint3->Position(0), TOL);
    }
  }
}

TEST_F(ODEGearboxJoint_TEST, GearboxTestODE)
{
  GearboxTest("ode");
}

////////////////////////////////////////////////////////////////////////
// SetGearboxRatio:
// start gearbox.world, set a new gear ratio,
// apply balancing forces across geared members, and check for equilibrium.
////////////////////////////////////////////////////////////////////////
void ODEGearboxJoint_TEST::SetGearboxRatio(const std::string &_physicsEngine)
{
  // load gearbox world
  Load("worlds/gearbox.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("model_1");
  physics::JointPtr joint1 = model->GetJoint("joint_12");
  physics::JointPtr joint3 = model->GetJoint("joint_23");

  physics::JointPtr gearboxJoint = model->GetJoint("joint_13");
  ASSERT_TRUE(gearboxJoint != NULL);
  ASSERT_TRUE(gearboxJoint->HasType(physics::Base::GEARBOX_JOINT));
  double gearboxRatio = -2.5;
  gearboxJoint->SetParam("gearbox_ratio", 0, gearboxRatio);
  EXPECT_NEAR(gearboxRatio,
    gearboxJoint->GetParam("gearbox_ratio", 0), TOL);
  double force3 = 1.0;
  double force1 = force3 * gearboxRatio;

  int steps = 10000;
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, force1);
    joint3->SetForce(0, force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->SimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->Position(0)
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->Position(0)
            << "]\n";
    EXPECT_NEAR(joint1->GetVelocity(0), 0, TOL);
    EXPECT_NEAR(joint3->GetVelocity(0), 0, TOL);
    EXPECT_NEAR(joint1->Position(0), 0, TOL);
    EXPECT_NEAR(joint3->Position(0), 0, TOL);
  }

  // slight imbalance
  for (int i = 0; i < steps; ++i)
  {
    joint1->SetForce(0, -force3);
    joint3->SetForce(0,  force3);
    world->Step(1);
    if (i%1000 == 0)
      gzdbg << "gearbox time [" << world->SimTime().Double()
            << "] vel [" << joint1->GetVelocity(0)
            << "] pose [" << joint1->Position(0)
            << "] vel [" << joint3->GetVelocity(0)
            << "] pose [" << joint3->Position(0)
            << "]\n";
    EXPECT_GT(joint1->GetVelocity(0), 0);
    EXPECT_GT(joint3->GetVelocity(0), 0);
    EXPECT_GT(joint1->Position(0), 0);
    EXPECT_GT(joint3->Position(0), 0);
    EXPECT_NEAR(joint1->GetVelocity(0)*gearboxRatio, -joint3->GetVelocity(0),
      TOL);
    EXPECT_NEAR(joint1->Position(0)*gearboxRatio,
               -joint3->Position(0), TOL);
  }
}

TEST_F(ODEGearboxJoint_TEST, SetGearboxRatioODE)
{
  SetGearboxRatio("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
