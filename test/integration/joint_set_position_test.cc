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
// #include "gazebo/physics/Joint.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL 0.001
using namespace gazebo;

class JointKinematicTest : public ServerFixture,
                           public testing::WithParamInterface<const char*>
{
  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in series with World::Step(1) with physics paused to avoid race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionTest(const std::string &_physicsEngine);

  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in parallel with World::Step(1) with physics running to check for race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionThreadedTest(const std::string &_physicsEngine);

  /// \brief Test setting joint position.  Joint::SetPosition is called
  /// in series with World::Step(1) with physics paused to avoid race
  /// conditions between physics updating link poses and Joint::Angle setting
  /// link poses.
  /// This test tries to set joint angles of loop joints, the model state
  /// should not change.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void SetJointPositionLoopJointTest(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionTest(const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  unsigned int seed = time(NULL);

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    sleep(1);
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double test_wall_duration = 2.0;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_1::joint_01"));
  joints.push_back(model->GetJoint("model_1::joint_12"));
  // joints.push_back(model->GetJoint("model_1::joint_23"));
  // joints.push_back(model->GetJoint("model_1::joint_34"));
  // joints.push_back(model->GetJoint("model_1::joint_45"));
  // joints.push_back(model->GetJoint("model_1::joint_52"));
  joints.push_back(model->GetJoint("model_1::joint_22a"));
  joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  // joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  // joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  // joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  // joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  world->SetPaused(true);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << test_wall_duration
        << " secs, see how well Joint::SetPosition deals with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition in series"
        << " with World::Step, so there's no physics engine update collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  while (common::Time::GetWallTime().Double() <
         start_wall_time + test_wall_duration)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        (*ji)->SetPosition(0,
            static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      }

      // gzdbg << "debug: running ["
      //       << common::Time::GetWallTime().Double()
      //       << " / " << start_wall_time + test_wall_duration
      //       << "]\n";
    }

    // step simulation
    world->Step(1);

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li)
    {
      ignition::math::Vector3d linVel = (*li)->WorldLinearVel();
      ignition::math::Vector3d angVel = (*li)->WorldAngularVel();
      EXPECT_NEAR(linVel.X(), 0, TOL);
      EXPECT_NEAR(linVel.Y(), 0, TOL);
      EXPECT_NEAR(linVel.Z(), 0, TOL);
      EXPECT_NEAR(angVel.X(), 0, TOL);
      EXPECT_NEAR(angVel.Y(), 0, TOL);
      EXPECT_NEAR(angVel.Z(), 0, TOL);
    }
  }
  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

TEST_P(JointKinematicTest, SetJointPositionTest)
{
  SetJointPositionTest(GetParam());
}

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionThreadedTest(
  const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  unsigned int seed = time(NULL);

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    sleep(1);
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double test_wall_duration = 2.0;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_1::joint_01"));
  joints.push_back(model->GetJoint("model_1::joint_12"));
  // joints.push_back(model->GetJoint("model_1::joint_23"));
  // joints.push_back(model->GetJoint("model_1::joint_34"));
  // joints.push_back(model->GetJoint("model_1::joint_45"));
  // joints.push_back(model->GetJoint("model_1::joint_52"));
  joints.push_back(model->GetJoint("model_1::joint_22a"));
  joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  // joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  // joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  // joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  // joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  world->SetPaused(false);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << test_wall_duration
        << " secs, see how well Joint::SetPosition delas with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition happens"
        << " in parallel with phsics update, leading to potential collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  while (common::Time::GetWallTime().Double() <
         start_wall_time + test_wall_duration)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        (*ji)->SetPosition(0,
            static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      }

      // gzdbg << "debug: running ["
      //       << common::Time::GetWallTime().Double()
      //       << " / " << start_wall_time + test_wall_duration
      //       << "]\n";
    }

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li)
    {
      ignition::math::Vector3d linVel = (*li)->WorldLinearVel();
      ignition::math::Vector3d angVel = (*li)->WorldAngularVel();
      EXPECT_NEAR(linVel.X(), 0, TOL);
      EXPECT_NEAR(linVel.Y(), 0, TOL);
      EXPECT_NEAR(linVel.Z(), 0, TOL);
      EXPECT_NEAR(angVel.X(), 0, TOL);
      EXPECT_NEAR(angVel.Y(), 0, TOL);
      EXPECT_NEAR(angVel.Z(), 0, TOL);
    }
  }
  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

// This test fails on OSX (see issue #1219)
// https://bitbucket.org/osrf/gazebo/issues/1219
#ifndef __APPLE__
TEST_P(JointKinematicTest, SetJointPositionThreadedTest)
{
  SetJointPositionThreadedTest(GetParam());
}
#endif

//////////////////////////////////////////////////
void JointKinematicTest::SetJointPositionLoopJointTest(
    const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  unsigned int seed = time(NULL);

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->ModelByName("model_1");
  while (!model)
  {
    model = world->ModelByName("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    sleep(1);
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  const double iterations = 3;
  double elapsed_wall_time;
  const double pub_rate = 10000.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  // joints.push_back(model->GetJoint("model_1::joint_01"));
  // joints.push_back(model->GetJoint("model_1::joint_12"));
  joints.push_back(model->GetJoint("model_1::joint_23"));
  joints.push_back(model->GetJoint("model_1::joint_34"));
  joints.push_back(model->GetJoint("model_1::joint_45"));
  joints.push_back(model->GetJoint("model_1::joint_52"));
  // joints.push_back(model->GetJoint("model_1::joint_22a"));
  // joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  std::vector<ignition::math::Pose3d> linkPoses;
  for (physics::Link_V::iterator li = links.begin();
                                 li != links.end(); ++li)
    linkPoses.push_back((*li)->WorldPose());

  world->SetPaused(true);
  start_time = world->SimTime().Double();
  start_wall_time = common::Time::GetWallTime().Double();
  double last_update_wall_time = -1e16;

  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands for " << iterations
        << " steps, see how well Joint::SetPosition delas with random inputs.\n"
        << " The test is run such that we call Joint::SetPosition in series"
        << " with World::Step, so there's no physics engine update collision"
        << " with Link::SetWorldPose from ODEJoint::SetPosition.\n";
  gzdbg << " Calling Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  for (int i = 0; i < iterations; ++i)
  {
    // limit setting pose to some rate in wall time
    if (common::Time::GetWallTime().Double() - last_update_wall_time
        >= (1.0/pub_rate))
    {
      last_update_wall_time = common::Time::GetWallTime().Double();
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        (*ji)->SetPosition(0,
            static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      }

      // gzdbg << "debug: running ["
      //       << i << " / " << iterations
      //       << "]\n";
    }

    // step simulation
    world->Step(1);

    // gzdbg << "debug: " << common::Time::GetWallTime().Double()
    //       << " - " << last_update_wall_time
    //       << " >= " << (1.0/pub_rate) << "\n";

    auto pi = linkPoses.begin();
    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li, ++pi)
    {
      auto pose = (*li)->WorldPose();
      EXPECT_NEAR(pose.Pos().X(), pi->Pos().X(), TOL);
      EXPECT_NEAR(pose.Pos().Y(), pi->Pos().Y(), TOL);
      EXPECT_NEAR(pose.Pos().Z(), pi->Pos().Z(), TOL);
      EXPECT_NEAR(pose.Rot().W(), pi->Rot().W(), TOL);
      EXPECT_NEAR(pose.Rot().X(), pi->Rot().X(), TOL);
      EXPECT_NEAR(pose.Rot().Y(), pi->Rot().Y(), TOL);
      EXPECT_NEAR(pose.Rot().Z(), pi->Rot().Z(), TOL);
    }
  }

  double test_duration = world->SimTime().Double() - start_time;
  elapsed_wall_time = common::Time::GetWallTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

TEST_P(JointKinematicTest, SetJointPositionLoopJointTest)
{
  SetJointPositionLoopJointTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointKinematicTest,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
