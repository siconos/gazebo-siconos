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
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "test_config.h"
#include "test/util.hh"

#define NEAR_TOL 2e-5

using namespace gazebo;

class BulletTypes : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// Check Vector3 conversions
TEST_F(BulletTypes, ConvertVector3)
{
  {
    ignition::math::Vector3d vec, vec2;
    btVector3 bt = physics::BulletTypes::ConvertVector3(vec);
    EXPECT_NEAR(bt.getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), 0, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector3Ign(bt);
    EXPECT_LT((vec-vec2).SquaredLength(), NEAR_TOL*NEAR_TOL);
  }

  {
    ignition::math::Vector3d vec(100.5, -2.314, 42), vec2;
    btVector3 bt = physics::BulletTypes::ConvertVector3(vec);
    EXPECT_NEAR(bt.getX(), vec.X(), NEAR_TOL);
    EXPECT_NEAR(bt.getY(), vec.Y(), NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), vec.Z(), NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector3Ign(bt);
    EXPECT_LT((vec-vec2).SquaredLength(), NEAR_TOL*NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Check Vector4 conversions
TEST_F(BulletTypes, ConvertVector4)
{
  {
    ignition::math::Vector4d vec, vec2;
    btVector4 bt = physics::BulletTypes::ConvertVector4dIgn(vec);
    EXPECT_NEAR(bt.getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getW(), 0, NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector4dIgn(bt);
    EXPECT_LT((vec-vec2).SquaredLength(), NEAR_TOL*NEAR_TOL);
  }

  {
    ignition::math::Vector4d vec(100.5, -2.314, 42, 848.8), vec2;
    btVector4 bt = physics::BulletTypes::ConvertVector4dIgn(vec);
    EXPECT_NEAR(bt.getX(), vec.X(), NEAR_TOL);
    EXPECT_NEAR(bt.getY(), vec.Y(), NEAR_TOL);
    EXPECT_NEAR(bt.getZ(), vec.Z(), NEAR_TOL);
    EXPECT_NEAR(bt.getW(), vec.W(), NEAR_TOL);
    vec2 = physics::BulletTypes::ConvertVector4dIgn(bt);
    EXPECT_LT((vec-vec2).SquaredLength(), NEAR_TOL*NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Check Pose conversions
TEST_F(BulletTypes, ConvertPose)
{
  {
    ignition::math::Pose3d pose, pose2;
    btTransform bt = physics::BulletTypes::ConvertPose(pose);
    EXPECT_NEAR(bt.getOrigin().getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getX(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getY(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getZ(), 0, NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getW(), 1, NEAR_TOL);
    pose2 = physics::BulletTypes::ConvertPoseIgn(bt);
    EXPECT_LT((pose.Pos()-pose2.Pos()).SquaredLength(), NEAR_TOL*NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).X(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).Y(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).Z(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).W(), NEAR_TOL);
  }

  {
    ignition::math::Pose3d pose(105.4, 3.1, -0.34, 3.14/8, 3.14/16, -3.14/2);
    ignition::math::Pose3d pose2;
    btTransform bt = physics::BulletTypes::ConvertPose(pose);
    EXPECT_NEAR(bt.getOrigin().getX(), pose.Pos().X(), NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getY(), pose.Pos().Y(), NEAR_TOL);
    EXPECT_NEAR(bt.getOrigin().getZ(), pose.Pos().Z(), NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getX(), pose.Rot().X(), NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getY(), pose.Rot().Y(), NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getZ(), pose.Rot().Z(), NEAR_TOL);
    EXPECT_NEAR(bt.getRotation().getW(), pose.Rot().W(), NEAR_TOL);
    pose2 = physics::BulletTypes::ConvertPoseIgn(bt);
    EXPECT_LT((pose.Pos()-pose2.Pos()).SquaredLength(), NEAR_TOL*NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).X(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).Y(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).Z(), NEAR_TOL);
    EXPECT_LT((pose.Rot()-pose2.Rot()).W(), NEAR_TOL);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
