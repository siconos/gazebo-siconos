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

#include "gazebo/math/Vector2i.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class Vector2iTest : public ::testing::Test { };

TEST_F(Vector2iTest, Vector2i)
{
  {
    math::Vector2i v;
    EXPECT_EQ(0, v.x);
    EXPECT_EQ(0, v.y);
  }

  // Constructor
  math::Vector2i v(1, 2);
  EXPECT_EQ(1, v.x);
  EXPECT_EQ(2, v.y);

  // ::Distance
  EXPECT_EQ(2, v.Distance(math::Vector2i(0, 0)));

  // ::Normalize
  v.Normalize();
  EXPECT_TRUE(v == math::Vector2i(0, 1));

  // ::Set
  v.Set(4, 5);
  EXPECT_TRUE(v == math::Vector2i(4, 5));

  // ::operator=
  v = math::Vector2i(6, 7);
  EXPECT_TRUE(v == math::Vector2i(6, 7));

  // ::operator= int
  v = 5;
  EXPECT_TRUE(v == math::Vector2i(5, 5));

  // ::operator+
  v = v + math::Vector2i(1, 2);
  EXPECT_TRUE(v == math::Vector2i(6, 7));

  // ::operator +=
  v += math::Vector2i(5, 6);
  EXPECT_TRUE(v == math::Vector2i(11, 13));

  // ::operator -
  v = v - math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(9, 9));

  // ::operator -=
  v.Set(2, 4);
  v -= math::Vector2i(1, 6);
  EXPECT_TRUE(v == math::Vector2i(1, -2));

  // ::operator /
  v.Set(10, 6);
  v = v / math::Vector2i(2, 3);
  EXPECT_TRUE(v == math::Vector2i(5, 2));

  // ::operator /=
  v.Set(10, 6);
  v /= math::Vector2i(2, 3);
  EXPECT_TRUE(v == math::Vector2i(5, 2));

  // ::operator / int
  v.Set(10, 6);
  v = v / 2;
  EXPECT_TRUE(v == math::Vector2i(5, 3));

  // ::operator /= int
  v.Set(10, 6);
  v /= 2;
  EXPECT_TRUE(v == math::Vector2i(5, 3));

  // ::operator * int
  v.Set(10, 6);
  v = v * 2;
  EXPECT_TRUE(v == math::Vector2i(20, 12));

  // ::operator * int
  v.Set(10, 6);
  v *= 2;
  EXPECT_TRUE(v == math::Vector2i(20, 12));

  // ::operator * vector2i
  v.Set(10, 6);
  v = v * math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(20, 24));

  // ::operator *= vector2i
  v.Set(10, 6);
  v *= math::Vector2i(2, 4);
  EXPECT_TRUE(v == math::Vector2i(20, 24));


  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v.Set(6, 7);
  EXPECT_EQ(6, v[0]);
  EXPECT_EQ(7, v[1]);
}


