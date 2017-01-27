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
#include <ignition/math/Rand.hh>
#include <ignition/math/SignalStats.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;
const double g_tolerance = 1e-2;

class JointTestRevolute : public JointTest
{
  /// \brief Spawn single pendulum and test GetWorldEnergy functions.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void PendulumEnergy(const std::string &_physicsEngine);

  /// \brief Spin joints several rotations and verify that the angles
  /// wrap properly.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void WrapAngle(const std::string &_physicsEngine);

  /// \brief Load 8 double pendulums arranged in a circle.
  /// Measure angular velocity of links, and verify proper axis orientation.
  /// Then set joint limits and verify that links remain within limits.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _solverType Type of solver to use.
  public: void RevoluteJoint(const std::string &_physicsEngine,
                             const std::string &_solverType="quick");

  /// \brief Load a simple pendulum, simulate and compare to numerical
  /// results from SimplePendulumIntegrator.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SimplePendulum(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////
void JointTestRevolute::PendulumEnergy(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  {
    SpawnJointOptions opt;
    opt.type = "revolute";
    opt.worldParent = true;
    double Am = IGN_PI / 4;
    opt.modelPose.Rot().Euler(0, 0, Am);
    opt.childLinkPose.Pos().Z() = 3.0;
    opt.jointPose.Pos().Y() = 1.5;
    opt.axis.Set(1, 0, 0);

    gzdbg << "SpawnJoint " << opt.type << " child world" << std::endl;
    physics::JointPtr joint = SpawnJoint(opt);
    ASSERT_TRUE(joint != NULL);

    // Get initial energy
    physics::LinkPtr link = joint->GetChild();
    ASSERT_TRUE(link != NULL);
    physics::ModelPtr model = link->GetModel();
    ASSERT_TRUE(model != NULL);

    double energy0 = model->GetWorldEnergy();
    EXPECT_NEAR(model->GetWorldEnergyKinetic(), 0.0, g_tolerance);

    unsigned int stepSize = 5;
    unsigned int stepCount = 500;
    for (unsigned int i = 0; i < stepCount; ++i)
    {
      world->Step(stepSize);
      double energy = model->GetWorldEnergy();
      EXPECT_NEAR(energy / energy0, 1.0, g_tolerance);
    }
  }
}

////////////////////////////////////////////////////////////
void JointTestRevolute::WrapAngle(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // disable gravity
  physics->SetGravity(ignition::math::Vector3d::Zero);

  {
    std::string jointType = "revolute";
    gzdbg << "SpawnJoint " << jointType << " child world" << std::endl;
    physics::JointPtr joint = SpawnJoint(jointType, false, true);
    ASSERT_TRUE(joint != NULL);

    // set velocity to 2 pi rad/s and step forward 1.5 seconds.
    // angle should reach 3 pi rad.
    double vel = 2*IGN_PI;
    unsigned int stepSize = 50;
    unsigned int stepCount = 30;
    double dt = physics->GetMaxStepSize();

    // Verify that the joint should make more than 1 revolution
    EXPECT_GT(vel * stepSize * stepCount * dt, 1.25 * 2 * IGN_PI);

    joint->SetVelocity(0, vel);

    ignition::math::SignalMaxAbsoluteValue angleErrorMax;
    // expect that joint velocity is constant
    // and that joint angle is unwrapped
    for (unsigned int i = 0; i < stepCount; ++i)
    {
      world->Step(stepSize);
      EXPECT_NEAR(joint->GetVelocity(0), vel, g_tolerance);
      double time = world->SimTime().Double();
      angleErrorMax.InsertData(joint->Position(0) - time*vel);
    }
#ifndef LIBBULLET_VERSION_GT_282
    if (_physicsEngine == "bullet")
    {
      gzerr << "Skipping portion of test, angle wrapping requires bullet 2.83"
            << std::endl;
    }
    else
#endif
    {
      EXPECT_NEAR(angleErrorMax.Value(), 0.0, g_tolerance);
    }
  }
}


////////////////////////////////////////////////////////////////////////
void JointTestRevolute::RevoluteJoint(const std::string &_physicsEngine,
                                      const std::string &_solverType)
{
  ignition::math::Rand::Seed(0);
  // Load world
  Load("worlds/revolute_joint_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Set solver type
  physics->SetParam("solver_type", _solverType);
  if (_solverType == "world")
  {
    physics->SetParam("ode_quiet", true);
  }

  // Model names
  std::vector<std::string> modelNames;
  modelNames.push_back("pendulum_0deg");
  modelNames.push_back("pendulum_45deg");
  modelNames.push_back("pendulum_90deg");
  modelNames.push_back("pendulum_135deg");
  modelNames.push_back("pendulum_180deg");
  modelNames.push_back("pendulum_225deg");
  modelNames.push_back("pendulum_270deg");
  modelNames.push_back("pendulum_315deg");

  // Global axis
  double sqrt1_2 = sqrt(2.0) / 2.0;
  std::vector<ignition::math::Vector3d> globalAxes;
  globalAxes.push_back(ignition::math::Vector3d(1, 0, 0));
  globalAxes.push_back(ignition::math::Vector3d(sqrt1_2, sqrt1_2, 0));
  globalAxes.push_back(ignition::math::Vector3d(0, 1, 0));
  globalAxes.push_back(ignition::math::Vector3d(-sqrt1_2, sqrt1_2, 0));
  globalAxes.push_back(ignition::math::Vector3d(-1, 0, 0));
  globalAxes.push_back(ignition::math::Vector3d(-sqrt1_2, -sqrt1_2, 0));
  globalAxes.push_back(ignition::math::Vector3d(0, -1, 0));
  globalAxes.push_back(ignition::math::Vector3d(sqrt1_2, -sqrt1_2, 0));

  // Link names
  std::vector<std::string> linkNames;
  linkNames.push_back("lower_link");
  linkNames.push_back("upper_link");

  // Joint names
  std::vector<std::string> jointNames;
  jointNames.push_back("lower_joint");
  jointNames.push_back("upper_joint");

  physics::ModelPtr model;
  physics::LinkPtr link;
  std::vector<std::string>::iterator modelIter;
  physics::JointPtr joint;

  double energy0 = 1.0;
  // Check global axes before simulation starts
  std::vector<ignition::math::Vector3d>::iterator axisIter;
  axisIter = globalAxes.begin();
  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      energy0 = model->GetWorldEnergy();
      gzdbg << "Check global axes of model " << *modelIter << '\n';
      std::vector<std::string>::iterator jointIter;
      for (jointIter  = jointNames.begin();
           jointIter != jointNames.end(); ++jointIter)
      {
        joint = model->GetJoint(*jointIter);
        if (joint)
        {
          auto axis = joint->GlobalAxis(0);
          EXPECT_NEAR(axis.X(), (*axisIter).X(), g_tolerance);
          EXPECT_NEAR(axis.Y(), (*axisIter).Y(), g_tolerance);
          EXPECT_NEAR(axis.Z(), (*axisIter).Z(), g_tolerance);
        }
        else
        {
          gzerr << "Error loading joint " << *jointIter
                << " of model " << *modelIter << '\n';
          EXPECT_TRUE(joint != NULL);
        }
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
    ++axisIter;
  }

  // Step forward 0.75 seconds
  double dt = physics->GetMaxStepSize();
  ASSERT_GT(dt, 0);
  int steps = ceil(0.75 / dt);
  world->Step(steps);

  // Get global angular velocity of each link
  ignition::math::Vector3d angVel;
  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      EXPECT_NEAR(model->GetWorldEnergy() / energy0, 1.0, g_tolerance);
      gzdbg << "Check angular velocity of model " << *modelIter << '\n';
      link = model->GetLink("base");
      if (link)
      {
        // Expect stationary base
        angVel = link->WorldAngularVel();
        EXPECT_NEAR(angVel.X(), 0, g_tolerance*10);
        EXPECT_NEAR(angVel.Y(), 0, g_tolerance*10);
        EXPECT_NEAR(angVel.Z(), 0, g_tolerance*10);
      }
      else
      {
        gzerr << "Error loading base link of model " << *modelIter << '\n';
        EXPECT_TRUE(link != NULL);
      }

      std::vector<std::string>::iterator linkIter;
      for (linkIter  = linkNames.begin();
           linkIter != linkNames.end(); ++linkIter)
      {
        link = model->GetLink(*linkIter);
        if (link)
        {
          // Expect relative angular velocity of pendulum links to be negative
          // and along x axis.
          angVel = link->RelativeAngularVel().Normalize();
          EXPECT_NEAR(angVel.X(), -1, g_tolerance);
          EXPECT_NEAR(angVel.Y(),  0, 2*g_tolerance);
          EXPECT_NEAR(angVel.Z(),  0, 2*g_tolerance);
        }
        else
        {
          gzerr << "Error loading link " << *linkIter
                << " of model " << *modelIter << '\n';
          EXPECT_TRUE(link != NULL);
        }
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Keep stepping forward, verifying that joint angles move in the direction
  // implied by the joint velocity
  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      EXPECT_NEAR(model->GetWorldEnergy() / energy0, 1.0, g_tolerance);
      double jointVel1, jointVel2;
      double angle1, angle2, angle3;

      gzdbg << "Check angle measurement for " << *modelIter << '\n';
      std::vector<std::string>::iterator jointIter;
      for (jointIter  = jointNames.begin();
           jointIter != jointNames.end(); ++jointIter)
      {
        joint = model->GetJoint(*jointIter);
        if (joint)
        {
          // Get first joint angle
          angle1 = joint->Position(0);

          // Get joint velocity and assume it is not too small
          jointVel1 = joint->GetVelocity(0);
          EXPECT_GT(fabs(jointVel1), 1e-1);

          // Take 1 step and measure again
          world->Step(1);

          // Expect angle change in direction of joint velocity
          angle2 = joint->Position(0);
          EXPECT_GT(
            (angle2 - angle1) * ignition::math::clamp(jointVel1*1e4, -1.0, 1.0)
            , 0);

          jointVel2 = joint->GetVelocity(0);
          EXPECT_GT(fabs(jointVel2), 1e-1);

          // Take 1 step and measure the last angle, expect decrease
          world->Step(1);
          angle3 = joint->Position(0);
          EXPECT_GT(
            (angle3 - angle2) * ignition::math::clamp(jointVel2*1e4, -1.0, 1.0)
            , 0);
        }
        else
        {
          gzerr << "Error loading joint " << *jointIter
                << " of model " << *modelIter << '\n';
          EXPECT_TRUE(joint != NULL);
        }
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }


  // Reset the world, and impose joint limits
  world->Reset();

  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      std::vector<std::string>::iterator jointIter;
      for (jointIter  = jointNames.begin();
           jointIter != jointNames.end(); ++jointIter)
      {
        joint = model->GetJoint(*jointIter);
        if (joint)
        {
          joint->SetLowerLimit(0, -0.1);
          joint->SetUpperLimit(0, 0.1);
        }
        else
        {
          gzerr << "Error loading joint " << *jointIter
                << " of model " << *modelIter << '\n';
          EXPECT_TRUE(joint != NULL);
        }
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Step forward again for 0.75 seconds and check that joint angles
  // are within limits
  world->Step(steps);
  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      gzdbg << "Check angle limits and velocity of joints of model "
            << *modelIter << '\n';
      std::vector<std::string>::iterator jointIter;
      for (jointIter  = jointNames.begin();
           jointIter != jointNames.end(); ++jointIter)
      {
        joint = model->GetJoint(*jointIter);
        if (joint)
        {
          EXPECT_NEAR(joint->Position(0), 0, 0.11);
        }
        else
        {
          gzerr << "Error loading joint " << *jointIter
                << " of model " << *modelIter << '\n';
          EXPECT_TRUE(joint != NULL);
        }
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Reset world again, disable gravity, detach upper_joint
  // Then apply torque at lower_joint and verify motion
  world->Reset();
  physics->SetGravity(ignition::math::Vector3d::Zero);
  for (modelIter  = modelNames.begin();
       modelIter != modelNames.end(); ++modelIter)
  {
    model = world->ModelByName(*modelIter);
    if (model)
    {
      gzdbg << "Check SetForce for model " << *modelIter << '\n';

      joint = model->GetJoint("upper_joint");
      if (joint)
      {
        if (_physicsEngine == "simbody" ||
            _physicsEngine == "dart")
        {
          gzerr << "Skipping joint detachment per #862 / #903" << std::endl;
          continue;
        }
        // Detach upper_joint.
        joint->Detach();
        // Simbody and DART will not easily detach joints,
        // consider freezing joint limit instead
        // auto curAngle = joint->Position(0u);
        // joint->SetLowerLimit(0, curAngle);
        // joint->SetUpperLimit(0, curAngle);
      }
      else
      {
        gzerr << "Error loading upper_joint "
              << " of model " << *modelIter << '\n';
        EXPECT_TRUE(joint != NULL);
      }

      // Step forward and let things settle a bit.
      world->Step(100);

      joint = model->GetJoint("lower_joint");
      if (joint)
      {
        double oldVel, newVel, force;
        oldVel = joint->GetVelocity(0);
        // Apply positive torque to the lower_joint and step forward.
        force = 1;

        for (int i = 0; i < 10; ++i)
        {
          joint->SetForce(0, force);
          world->Step(1);
          joint->SetForce(0, force);
          world->Step(1);
          joint->SetForce(0, force);
          world->Step(1);
          newVel = joint->GetVelocity(0);

          // gzdbg << "model " << *modelIter
          //       << "  i " << i
          //       << "  oldVel " << oldVel
          //       << "  newVel " << newVel
          //       << "  upper joint v "
          //       << model->GetJoint("upper_joint")->GetVelocity(0)
          //       << " joint " << joint->GetName() <<  "\n";

          // Expect increasing velocities
          EXPECT_GT(newVel, oldVel);
          oldVel = newVel;

          // Check that GetForce returns what we set
          EXPECT_NEAR(joint->GetForce(0u), force, g_tolerance);

          // Expect joint velocity to be near angular velocity difference
          // of child and parent, along global axis
          // jointVel == DOT(angVelChild - angVelParent, axis)
          double jointVel = joint->GetVelocity(0);
          ignition::math::Vector3d axis = joint->GlobalAxis(0);
          angVel  = joint->GetChild()->WorldAngularVel();
          angVel -= joint->GetParent()->WorldAngularVel();
          EXPECT_NEAR(jointVel, axis.Dot(angVel), g_tolerance);
        }
        // Apply negative torque to lower_joint
        force = -3;
        for (int i = 0; i < 10; ++i)
        {
          joint->SetForce(0, force);
          world->Step(1);
          joint->SetForce(0, force);
          world->Step(1);
          joint->SetForce(0, force);
          world->Step(1);
          newVel = joint->GetVelocity(0);

          // gzdbg << "model " << *modelIter
          //       << "  i " << i
          //       << "  oldVel " << oldVel
          //       << "  newVel " << newVel
          //       << "  upper joint v "
          //       << model->GetJoint("upper_joint")->GetVelocity(0)
          //       << " joint " << joint->GetName() <<  "\n";

          // Expect decreasing velocities
          EXPECT_LT(newVel, oldVel);

          // Check that GetForce returns what we set
          EXPECT_NEAR(joint->GetForce(0u), force, g_tolerance);

          // Expect joint velocity to be near angular velocity difference
          // of child and parent, along global axis
          // jointVel == DOT(angVelChild - angVelParent, axis)
          double jointVel = joint->GetVelocity(0);
          ignition::math::Vector3d axis = joint->GlobalAxis(0);
          angVel  = joint->GetChild()->WorldAngularVel();
          angVel -= joint->GetParent()->WorldAngularVel();
          EXPECT_NEAR(jointVel, axis.Dot(angVel), g_tolerance);
        }
      }
      else
      {
        gzerr << "Error loading lower_joint "
              << " of model " << *modelIter << '\n';
        EXPECT_TRUE(joint != NULL);
      }
    }
    else
    {
      gzerr << "Error loading model " << *modelIter << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}

////////////////////////////////////////////////////////////////////////
void JointTestRevolute::SimplePendulum(const std::string &_physicsEngine)
{
  Load("worlds/simple_pendulums.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  int i = 0;
  while (!this->HasEntity("model_1") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_1");

  physics::PhysicsEnginePtr physicsEngine = world->Physics();
  EXPECT_TRUE(physicsEngine != NULL);
  physics::ModelPtr model = world->ModelByName("model_1");
  EXPECT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink("link_2");  // sphere link at end
  EXPECT_TRUE(link != NULL);

  double g = 9.81;
  double l = 10.0;
  double m = 10.0;

  double e_start;

  {
    // check velocity / energy
    ignition::math::Vector3d vel = link->WorldLinearVel();
    ignition::math::Pose3d pos = link->WorldPose();
    double pe = g * m * pos.Pos().Z();
    double ke = 0.5 * m * (vel.X()*vel.X() + vel.Y()*vel.Y() + vel.Z()*vel.Z());
    e_start = pe + ke;
    // gzdbg << "total energy [" << e_start
    //       << "] pe[" << pe
    //       << "] ke[" << ke
    //       << "] p[" << pos.pos.z
    //       << "] v[" << vel
    //       << "]\n";
  }
  physicsEngine->SetMaxStepSize(0.0001);
  physicsEngine->SetParam("iters", 1000);

  {
    // test with global contact_max_correcting_vel at 0 as set by world file
    //   here we expect significant energy loss as the velocity correction
    //   is set to 0
    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->Step(2000);
      {
        // check velocity / energy
        ignition::math::Vector3d vel = link->WorldLinearVel();
        ignition::math::Pose3d pos = link->WorldPose();
        double pe = g * m * pos.Pos().Z();
        double ke = 0.5 * m * (vel.X()*vel.X() + vel.Y()*vel.Y() +
            vel.Z()*vel.Z());
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
          PendulumAngle(g, l, 1.57079633, 0.0, world->SimTime().Double(),
          0.000001) - 1.5707963);
        double actual_theta = joint->Position(0);
        // gzdbg << "time [" << world->SimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->WorldPose()
        //       << "]\n";
         EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }



  {
    // test with global contact_max_correcting_vel at 100
    // here we expect much lower energy loss
    world->Reset();
    if (_physicsEngine == "ode")
      EXPECT_TRUE(physicsEngine->SetParam("contact_max_correcting_vel", 100.0));

    int steps = 10;  // @todo: make this more general
    for (int i = 0; i < steps; i ++)
    {
      world->Step(2000);
      {
        // check velocity / energy
        ignition::math::Vector3d vel = link->WorldLinearVel();
        ignition::math::Pose3d pos = link->WorldPose();
        double pe = g * m * pos.Pos().Z();
        double ke = 0.5 * m * (vel.X()*vel.X() + vel.Y()*vel.Y() +
            vel.Z()*vel.Z());
        double e = pe + ke;
        double e_tol = 3.0*static_cast<double>(i+1)
          / static_cast<double>(steps);
        // gzdbg << "total energy [" << e
        //       << "] pe[" << pe
        //       << "] ke[" << ke
        //       << "] p[" << pos.pos.z
        //       << "] v[" << vel
        //       << "] error[" << e - e_start
        //       << "] tol[" << e_tol
        //       << "]\n";

        EXPECT_LT(fabs(e - e_start), e_tol);
      }

      physics::JointPtr joint = model->GetJoint("joint_0");
      if (joint)
      {
        double integ_theta = (
          PendulumAngle(g, l, 1.57079633, 0.0, world->SimTime().Double(),
          0.000001) - 1.5707963);
        double actual_theta = joint->Position(0);
        // gzdbg << "time [" << world->SimTime().Double()
        //       << "] exact [" << integ_theta
        //       << "] actual [" << actual_theta
        //       << "] pose [" << model->WorldPose()
        //       << "]\n";
         EXPECT_LT(fabs(integ_theta - actual_theta) , 0.01);
      }
    }
  }
}

TEST_P(JointTestRevolute, PendulumEnergy)
{
  PendulumEnergy(this->physicsEngine);
}

TEST_P(JointTestRevolute, RevoluteJoint)
{
  RevoluteJoint(this->physicsEngine);
}

TEST_F(JointTestRevolute, RevoluteJointWorldStep)
{
  RevoluteJoint("ode", "world");
}

TEST_P(JointTestRevolute, SimplePendulum)
{
  SimplePendulum(this->physicsEngine);
}

TEST_P(JointTestRevolute, WrapAngle)
{
  WrapAngle(this->physicsEngine);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestRevolute,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
