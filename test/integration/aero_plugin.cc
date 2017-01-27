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
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL 1e-6
#define TOL_CONT 2.0

using namespace gazebo;

class JointLiftDragPluginTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Load example world with a lifting surface plugin
  /// Measure / verify force torques against analytical answers.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void LiftDragPlugin1(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void JointLiftDragPluginTest::LiftDragPlugin1(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzlog << "this test works for ode only for now (Link::AddForce)"
          << " missing for other engines.\n";
    return;
  }

  // Load our force torque test world
  Load("worlds/lift_drag_plugin.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d::Zero);

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();

  // get time step size
  double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get joint and get force torque
  physics::ModelPtr model_1 = world->ModelByName("lift_drag_demo_model");
  physics::LinkPtr body = model_1->GetLink("body");
  physics::LinkPtr wing_1 = model_1->GetLink("wing_1");
  physics::LinkPtr wing_2 = model_1->GetLink("wing_2");
  physics::JointPtr body_joint = model_1->GetJoint("body_joint");
  physics::JointPtr wing_1_joint = model_1->GetJoint("wing_1_joint");
  physics::JointPtr wing_2_joint = model_1->GetJoint("wing_2_joint");

  // some aero coeffs
  double cla = 4.0;
  double cda = 20.0;
  double dihedral = 0.1;
  double rho = 1.2041;
  double area = 10;
  // double stall_alpha = 10.0;
  double a0 = 0.1;

  // run for 100 seconds
  for (unsigned int i = 0; i < 2400; ++i)
  {
    world->Step(1);
    body->AddForce(ignition::math::Vector3d(-1, 0, 0));

    if (i > 2385)
    {
      double v = body->WorldLinearVel().X();
      double q = 0.5 * rho * v * v;
      double cl = cla * a0 * q * area;
      double cd = cda * a0 * q * area;

      physics::JointWrench body_wrench = body_joint->GetForceTorque(0);
      physics::JointWrench wing_1_wrench = wing_1_joint->GetForceTorque(0);
      physics::JointWrench wing_2_wrench = wing_2_joint->GetForceTorque(0);
      auto wing_1_pose = wing_1->WorldPose();
      auto wing_1_force =
        wing_1_pose.Rot().RotateVector(wing_1_wrench.body2Force);
      auto wing_1_torque =
        wing_1_pose.Rot().RotateVector(wing_1_wrench.body2Torque);

      auto wing_2_pose = wing_2->WorldPose();
      auto wing_2_force =
        wing_2_pose.Rot().RotateVector(wing_2_wrench.body2Force);
      auto wing_2_torque =
        wing_2_pose.Rot().RotateVector(wing_2_wrench.body2Torque);
      gzdbg << "body velocity [" << body->WorldLinearVel()
            << "] cl [" << cl
            << "] cd [" << cd
            << "] body force [" << body_wrench.body2Force
            << "] body torque [" << body_wrench.body2Torque
            << "] wing_1 force [" << wing_1_force
            << "] wing_1 torque [" << wing_1_torque
            << "] wing_2 force [" << wing_2_force
            << "] wing_2 torque [" << wing_2_torque
            << "]\n";

      EXPECT_NEAR(wing_1_force.Z(), cl * cos(dihedral), TOL);
    }
  }
}

TEST_P(JointLiftDragPluginTest, LiftDragPlugin1)
{
  LiftDragPlugin1(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointLiftDragPluginTest,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
