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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "test/integration/joint_test.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;
class Issue494Test : public JointTest
{
  /// \brief Test for issue #494, using proper joint axis frame.
  /// Also test basic joint properties.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to test.
  public: void CheckAxisFrame(const std::string &_physicsEngine,
                              const std::string &_jointType);

  /// \brief Check joint properties.
  /// \param[in] _joint Joint to check.
  /// \param[in] _axis Expected axis vector in global frame.
  public: void CheckJointProperties(physics::JointPtr _joint,
                                    const ignition::math::Vector3d &_axis);
};


/////////////////////////////////////////////////
void Issue494Test::CheckAxisFrame(const std::string &_physicsEngine,
                                  const std::string &_jointType)
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

  SpawnJointOptions opt;
  opt.type = _jointType;
  double Am = M_PI / 11;
  double Al = M_PI / 12;
  double Aj = M_PI / 13;
  opt.modelPose.Rot().Euler(0, 0, Am);
  opt.childLinkPose.Rot().Euler(0, 0, Al);
  opt.jointPose.Rot().Euler(0, 0, Aj);
  opt.axis.Set(1, 0, 0);

  // i = 0: joint between child link and parent link
  // i = 1: joint between child link and world
  // i = 2: joint between world and parent link
  for (int i = 0; i < 3; ++i)
  {
    gzdbg << "SpawnJoint " << _jointType;
    if (i / 2)
    {
      opt.worldChild = true;
      std::cout << " world";
    }
    else
    {
      opt.worldChild = false;
      std::cout << " child";
    }
    if (i % 2)
    {
      opt.worldParent = true;
      std::cout << " world";
    }
    else
    {
      opt.worldParent = false;
      std::cout << " parent";
    }
    std::cout << std::endl;

    if (opt.worldChild && _physicsEngine == "dart")
    {
      gzerr << "dart seg-faults without a child link, skipping sub-test"
            << std::endl;
      break;
    }

    // spawn joint using using parent model frame to define joint axis
    {
      gzdbg << "test case with joint axis specified in parent model frame.\n";
      opt.useParentModelFrame = true;
      physics::JointPtr jointUseParentModelFrame = SpawnJoint(opt);
      ASSERT_TRUE(jointUseParentModelFrame != NULL);

      if (opt.worldParent)
      {
        gzdbg << "  where parent is world.\n";
        this->CheckJointProperties(jointUseParentModelFrame, opt.axis);
      }
      else
      {
        gzdbg << "  where parent is another link (not world).\n";
        this->CheckJointProperties(jointUseParentModelFrame,
          ignition::math::Vector3d(cos(Am), sin(Am), 0));
      }
    }

    // spawn joint using using child link frame to define joint axis
    {
      gzdbg << "test case with joint axis specified in child link frame.\n";
      opt.useParentModelFrame = false;
      physics::JointPtr joint = SpawnJoint(opt);
      ASSERT_TRUE(joint != NULL);

      if (opt.worldChild)
      {
        gzdbg << "  where parent is world.\n";
        this->CheckJointProperties(joint,
          ignition::math::Vector3d(cos(Aj), sin(Aj), 0));
      }
      else
      {
        gzdbg << "  where parent is another link (not world).\n";
        this->CheckJointProperties(joint,
          ignition::math::Vector3d(cos(Am+Al+Aj), sin(Am+Al+Aj), 0));
      }
    }
  }
}

/////////////////////////////////////////////////
void Issue494Test::CheckJointProperties(physics::JointPtr _joint,
                                        const ignition::math::Vector3d &_axis)
{
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world != NULL);
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);

  // Check that Joint::GlobalAxis matches _axis
  EXPECT_EQ(_axis, _joint->GlobalAxis(0));

  // test LocalAxis, AxisFrame, and AxisFrameOffset
  // get axis specified locally (in joint frame or in parent model frame)
  ignition::math::Vector3d axisLocalFrame = _joint->LocalAxis(0);
  {
    // rotate axis into global frame
    ignition::math::Vector3d axisGlobalFrame =
      _joint->AxisFrame(0).RotateVector(axisLocalFrame);
    // Test AxisFrame: check that axis in global frame is
    // computed correctly.
    EXPECT_EQ(axisGlobalFrame, _axis);
  }
  {
    // rotate axis into joint frame
    ignition::math::Vector3d axisJointFrame =
      _joint->AxisFrameOffset(0).RotateVector(axisLocalFrame);
    // roate axis specified in global frame into joint frame
    ignition::math::Vector3d axisJointFrame2 =
      _joint->WorldPose().Rot().RotateVectorReverse(_axis);
    EXPECT_EQ(axisJointFrame, axisJointFrame2);
  }

  if (!_joint->GetChild())
  {
    gzerr << "The rest of this test fails without a child link" << std::endl;
    return;
  }

  double velocityMagnitude = 1.0;
  std::vector<double> velocities;
  velocities.push_back(velocityMagnitude);
  velocities.push_back(0.0);
  velocities.push_back(-velocityMagnitude);
  for (std::vector<double>::iterator iter = velocities.begin();
       iter != velocities.end(); ++iter)
  {
    // Use Joint::SetVelocity with different values
    double vel = *iter;
    _joint->SetVelocity(0, vel);

    // Verify that Joint::GetVelocity returns the same value
    EXPECT_NEAR(_joint->GetVelocity(0), vel, g_tolerance);

    // Also verify that relative body motions match expected joint behavior
    ignition::math::Vector3d childVelocity, parentVelocity;
    {
      physics::LinkPtr child = _joint->GetChild();
      if (child)
      {
        if (_joint->HasType(physics::Base::HINGE_JOINT)
              || _joint->HasType(physics::Base::UNIVERSAL_JOINT))
        {
          childVelocity = child->WorldAngularVel();
        }
        else if (_joint->HasType(physics::Base::SLIDER_JOINT)
              || _joint->HasType(physics::Base::SCREW_JOINT))
        {
          childVelocity = child->WorldLinearVel();
        }
      }
    }
    {
      physics::LinkPtr parent = _joint->GetParent();
      if (parent)
      {
        if (_joint->HasType(physics::Base::HINGE_JOINT)
            || _joint->HasType(physics::Base::UNIVERSAL_JOINT))
        {
          parentVelocity = parent->WorldAngularVel();
        }
        else if (_joint->HasType(physics::Base::SLIDER_JOINT)
              || _joint->HasType(physics::Base::SCREW_JOINT))
        {
          parentVelocity = parent->WorldLinearVel();
        }
      }
    }
    std::cout << "    joint pose:        " << _joint->WorldPose()
              << std::endl;
    std::cout << "    global axis:       " << _axis << std::endl;
    std::cout << "    axis frame:        " << _joint->AxisFrame(0)
              << std::endl;
    std::cout << "    axis frame offset: " << _joint->AxisFrameOffset(0)
              << std::endl;
    std::cout << "    desired velocity:  " << vel << std::endl;
    std::cout << "    joint velocity:    " << _joint->GetVelocity(0)
              << std::endl;
    std::cout << "    child velocity:    " << childVelocity << std::endl;
    std::cout << "    parent velocity:   " << parentVelocity << std::endl;
    std::cout << std::endl;
    EXPECT_NEAR(vel, _axis.Dot(childVelocity - parentVelocity), g_tolerance);
  }
}

TEST_P(Issue494Test, CheckAxisFrame)
{
  CheckAxisFrame(this->physicsEngine, this->jointType);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue494Test,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "prismatic"
                  , "universal")));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
