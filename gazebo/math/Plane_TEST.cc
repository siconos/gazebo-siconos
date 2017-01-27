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
#include "gazebo/math/Plane.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class PlaneTest : public ::testing::Test { };

TEST_F(PlaneTest, Plane)
{
  {
    math::Plane plane;
    EXPECT_TRUE(math::equal(plane.d, 0.0));
    EXPECT_TRUE(plane.normal == math::Vector3());
    EXPECT_TRUE(plane.size == math::Vector2d(0, 0));
  }

  {
    math::Plane plane(math::Vector3(0, 0, 1), math::Vector2d(2, 3), 2.0);
    EXPECT_TRUE(math::equal(plane.d, 2.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 0, 1));
    EXPECT_TRUE(plane.size == math::Vector2d(2, 3));

    EXPECT_DOUBLE_EQ(-1, plane.Distance(math::Vector3(0, 0, 1),
          math::Vector3(0, 0, -1)));

    plane.Set(math::Vector3(1, 0, 0), math::Vector2d(1, 1), 1.0);
    EXPECT_TRUE(math::equal(plane.d, 1.0));
    EXPECT_TRUE(plane.normal == math::Vector3(1, 0, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(1, 1));

    plane = math::Plane(math::Vector3(0, 1, 0), math::Vector2d(4, 4), 5.0);
    EXPECT_TRUE(math::equal(plane.d, 5.0));
    EXPECT_TRUE(plane.normal == math::Vector3(0, 1, 0));
    EXPECT_TRUE(plane.size == math::Vector2d(4, 4));
  }
}
