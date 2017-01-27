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

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Angle.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class AngleTest : public ::testing::Test { };

TEST_F(AngleTest, Angle)
{
  math::Angle angle1;
  EXPECT_TRUE(math::equal(0.0, angle1.Radian()));

  angle1.SetFromDegree(180.0);
  EXPECT_TRUE(angle1 == M_PI);

  angle1 = math::Angle(0.1) - math::Angle(0.3);
  EXPECT_TRUE(angle1 == -0.2);

  math::Angle angle(0.5);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle.SetFromRadian(M_PI);
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.Degree()));

  angle.Normalize();
  EXPECT_TRUE(math::equal(GZ_RTOD(M_PI), angle.Degree()));

  angle = math::Angle(0.1) + math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle = math::Angle(0.1) * math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.02, angle.Radian()));

  angle = math::Angle(0.1) / math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle -= math::Angle(0.1);
  EXPECT_TRUE(math::equal(0.4, angle.Radian()));

  angle += math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.6, angle.Radian()));

  angle *= math::Angle(0.5);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle /= math::Angle(0.1);
  EXPECT_TRUE(math::equal(3.0, angle.Radian()));
  EXPECT_TRUE(angle == math::Angle(3));
  EXPECT_TRUE(angle != math::Angle(2));
  EXPECT_TRUE(angle < math::Angle(4));
  EXPECT_TRUE(angle <= math::Angle(3));
  EXPECT_TRUE(angle > math::Angle(2));
  EXPECT_TRUE(angle >= math::Angle(3));
}
