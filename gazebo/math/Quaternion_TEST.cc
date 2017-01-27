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
#include "gazebo/math/Quaternion.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class QuaternionTest : public ::testing::Test { };

//////////////////////////////////////////////////
TEST_F(QuaternionTest, Quaternion)
{
  {
    math::Quaternion q;
    EXPECT_TRUE(math::equal(q.w, 1.0));
    EXPECT_TRUE(math::equal(q.x, 0.0));
    EXPECT_TRUE(math::equal(q.y, 0.0));
    EXPECT_TRUE(math::equal(q.z, 0.0));
  }

  {
    math::Quaternion q(1, 2, 3, 4);
    EXPECT_TRUE(math::equal(q.w, 1.0));
    EXPECT_TRUE(math::equal(q.x, 2.0));
    EXPECT_TRUE(math::equal(q.y, 3.0));
    EXPECT_TRUE(math::equal(q.z, 4.0));
  }

  {
    math::Quaternion q(0, 1, 2);
    EXPECT_TRUE(q == math::Quaternion(math::Vector3(0, 1, 2)));
  }

  math::Quaternion q1(math::Vector3(0, 0, 1), M_PI);
  EXPECT_TRUE(math::equal(q1.x, 0.0));
  EXPECT_TRUE(math::equal(q1.y, 0.0));
  EXPECT_TRUE(math::equal(q1.z, 1.0));
  EXPECT_TRUE(math::equal(q1.w, 0.0));

  math::Quaternion q(q1);
  EXPECT_TRUE(q == q1);

  q.SetToIdentity();
  EXPECT_TRUE(math::equal(q.w, 1.0));
  EXPECT_TRUE(math::equal(q.x, 0.0));
  EXPECT_TRUE(math::equal(q.y, 0.0));
  EXPECT_TRUE(math::equal(q.z, 0.0));

  q = math::Quaternion(M_PI*0.1, M_PI*0.5, M_PI);
  EXPECT_TRUE(q == math::Quaternion(0.110616, -0.698401, 0.110616, 0.698401));

  EXPECT_TRUE(q.GetLog() ==
      math::Quaternion(0, -1.02593, 0.162491, 1.02593));

  EXPECT_TRUE(q.GetExp() ==
      math::Quaternion(0.545456, -0.588972, 0.093284, 0.588972));

  q1 = q;
  q1.w = 2.0;
  EXPECT_TRUE(q1.GetLog() ==
      math::Quaternion(0, -0.698401, 0.110616, 0.698401));

  q1.x = 0.000000001;
  q1.y = 0.0;
  q1.z = 0.0;
  q1.w = 0.0;
  EXPECT_TRUE(q1.GetExp() == math::Quaternion(1, 0, 0, 0));

  q.Invert();
  EXPECT_TRUE(q == math::Quaternion(0.110616, 0.698401, -0.110616, -0.698401));

  q.SetFromAxis(0, 1, 0, M_PI);
  EXPECT_TRUE(q == math::Quaternion(6.12303e-17, 0, 1, 0));

  q.SetFromAxis(math::Vector3(1, 0, 0), M_PI);
  EXPECT_TRUE(q == math::Quaternion(0, 1, 0, 0));

  q.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(q.w, 1.0));
  EXPECT_TRUE(math::equal(q.x, 2.0));
  EXPECT_TRUE(math::equal(q.y, 3.0));
  EXPECT_TRUE(math::equal(q.z, 4.0));

  q.Normalize();
  EXPECT_TRUE(q == math::Quaternion(0.182574, 0.365148, 0.547723, 0.730297));


  EXPECT_TRUE(math::equal(q.GetRoll(), 1.4289, 1e-3));
  EXPECT_TRUE(math::equal(q.GetPitch(), -0.339837, 1e-3));
  EXPECT_TRUE(math::equal(q.GetYaw(), 2.35619, 1e-3));

  math::Vector3 axis;
  double angle;
  q.GetAsAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3(0.371391, 0.557086, 0.742781));
  EXPECT_TRUE(math::equal(angle, 2.77438, 1e-3));

  q.Scale(0.1);
  EXPECT_TRUE(q == math::Quaternion(0.990394, 0.051354, 0.0770309, 0.102708));

  q = q + math::Quaternion(0, 1, 2);
  EXPECT_TRUE(q == math::Quaternion(1.46455, -0.352069, 0.336066, 0.841168));

  q += q;
  EXPECT_TRUE(q == math::Quaternion(2.92911, -0.704137, 0.672131, 1.68234));

  q -= math::Quaternion(.4, .2, .1);
  EXPECT_TRUE(q == math::Quaternion(1.95416, -0.896677, 0.56453, 1.65341));

  q = q - math::Quaternion(0, 1, 2);
  EXPECT_TRUE(q == math::Quaternion(1.48, -0.493254, 0.305496, 0.914947));

  q *= math::Quaternion(.4, .1, .01);
  EXPECT_TRUE(q == math::Quaternion(1.53584, -0.236801, 0.551841, 0.802979));

  q = q * 5.0;
  EXPECT_TRUE(q == math::Quaternion(7.67918, -1.184, 2.7592, 4.0149));

  std::cerr << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
  std::cerr << q.RotateVectorReverse(math::Vector3(1, 2, 3)) << "\n";

  EXPECT_TRUE(q.RotateVectorReverse(math::Vector3(1, 2, 3)) ==
      math::Vector3(-0.104115, 0.4975, 3.70697));

  EXPECT_TRUE(math::equal(q.Dot(math::Quaternion(.4, .2, .1)), 7.67183, 1e-3));

  EXPECT_TRUE(math::Quaternion::Squad(1.1, math::Quaternion(.1, 0, .2),
        math::Quaternion(0, .3, .4), math::Quaternion(.5, .2, 1),
        math::Quaternion(0, 0, 2), true) ==
      math::Quaternion(0.346807, -0.0511734, -0.0494723, 0.935232));

  EXPECT_TRUE(math::Quaternion::EulerToQuaternion(math::Vector3(.1, .2, .3)) ==
      math::Quaternion(0.983347, 0.0342708, 0.106021, 0.143572));

  q.Round(2);
  EXPECT_TRUE(math::equal(-1.18, q.x));
  EXPECT_TRUE(math::equal(2.76, q.y));
  EXPECT_TRUE(math::equal(4.01, q.z));
  EXPECT_TRUE(math::equal(7.68, q.w));

  q.x = q.y = q.z = q.w = 0.0;
  q.Normalize();
  EXPECT_TRUE(q == math::Quaternion());

  q.SetFromAxis(0, 0, 0, 0);
  EXPECT_TRUE(q == math::Quaternion());

  EXPECT_TRUE(math::Quaternion::EulerToQuaternion(0.1, 0.2, 0.3) ==
      math::Quaternion(0.983347, 0.0342708, 0.106021, 0.143572));

  q.x = q.y = q.z = q.w = 0.0;
  q.GetAsAxis(axis, angle);
  EXPECT_TRUE(axis == math::Vector3(1, 0, 0));
  EXPECT_TRUE(math::equal(angle, 0.0, 1e-3));
  {
    // simple 180 rotation about yaw, should result in x and y flipping signs
    q = math::Quaternion(0, 0, M_PI);
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.w << ", " << q.x << ", "
      << q.y << ", " << q.z << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    EXPECT_TRUE(r1 == math::Vector3(-1, -2, 3));
    EXPECT_TRUE(r2 == math::Vector3(-1, -2, 3));
  }

  {
    // simple  90 rotation about yaw, should map x to y, y to -x
    // simple -90 rotation about yaw, should map x to -y, y to x
    q = math::Quaternion(0, 0, 0.5*M_PI);
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    std::cout << "[" << q.w << ", " << q.x << ", "
      << q.y << ", " << q.z << "]\n";
    std::cout << " forward turns [" << v << "] to [" << r1 << "]\n";
    std::cout << " reverse turns [" << v << "] to [" << r2 << "]\n";
    std::cout << " x axis [" << q.GetXAxis() << "]\n";
    std::cout << " y axis [" << q.GetYAxis() << "]\n";
    std::cout << " z axis [" << q.GetZAxis() << "]\n";
    EXPECT_TRUE(r1 == math::Vector3(-2, 1, 3));
    EXPECT_TRUE(r2 == math::Vector3(2, -1, 3));
    EXPECT_TRUE(q.GetInverse().GetXAxis() == math::Vector3(0, -1, 0));
    EXPECT_TRUE(q.GetInverse().GetYAxis() == math::Vector3(1, 0, 0));
    EXPECT_TRUE(q.GetInverse().GetZAxis() == math::Vector3(0, 0, 1));
  }

  // Test RPY fixed-body-frame convention:
  // Rotate each unit vector in roll and pitch
  {
    q = math::Quaternion(M_PI/2.0, M_PI/2.0, 0);
    math::Vector3 v1(1, 0, 0);
    math::Vector3 r1 = q.RotateVector(v1);
    // 90 degrees about X does nothing,
    // 90 degrees about Y sends point down to -Z
    EXPECT_EQ(r1, math::Vector3(0, 0, -1));

    math::Vector3 v2(0, 1, 0);
    math::Vector3 r2 = q.RotateVector(v2);
    // 90 degrees about X sends point to +Z
    // 90 degrees about Y sends point to +X
    EXPECT_EQ(r2, math::Vector3(1, 0, 0));

    math::Vector3 v3(0, 0, 1);
    math::Vector3 r3 = q.RotateVector(v3);
    // 90 degrees about X sends point to -Y
    // 90 degrees about Y does nothing
    EXPECT_EQ(r3, math::Vector3(0, -1, 0));
  }

  {
    // now try a harder case (axis[1,2,3], rotation[0.3*pi])
    // verified with octave
    q.SetFromAxis(math::Vector3(1, 2, 3), 0.3*M_PI);
    std::cout << "[" << q.w << ", " << q.x << ", "
      << q.y << ", " << q.z << "]\n";
    std::cout << " x [" << q.GetInverse().GetXAxis() << "]\n";
    std::cout << " y [" << q.GetInverse().GetYAxis() << "]\n";
    std::cout << " z [" << q.GetInverse().GetZAxis() << "]\n";
    EXPECT_TRUE(q.GetInverse().GetXAxis() ==
                math::Vector3(0.617229, -0.589769, 0.520770));
    EXPECT_TRUE(q.GetInverse().GetYAxis() ==
                math::Vector3(0.707544, 0.705561, -0.039555));
    EXPECT_TRUE(q.GetInverse().GetZAxis() ==
                math::Vector3(-0.344106, 0.392882, 0.852780));

    // rotate about the axis of rotation should not change axis
    math::Vector3 v = math::Vector3(1, 2, 3);
    math::Vector3 r1 = q.RotateVector(v);
    math::Vector3 r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(1, 2, 3));
    EXPECT_TRUE(r2 == math::Vector3(1, 2, 3));

    // rotate unit vectors
    v = math::Vector3(0, 0, 1);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(0.520770, -0.039555, 0.852780));
    EXPECT_TRUE(r2 == math::Vector3(-0.34411, 0.39288, 0.85278));
    v = math::Vector3(0, 1, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(-0.58977, 0.70556, 0.39288));
    EXPECT_TRUE(r2 == math::Vector3(0.707544, 0.705561, -0.039555));
    v = math::Vector3(1, 0, 0);
    r1 = q.RotateVector(v);
    r2 = q.RotateVectorReverse(v);
    EXPECT_TRUE(r1 == math::Vector3(0.61723, 0.70754, -0.34411));
    EXPECT_TRUE(r2 == math::Vector3(0.61723, -0.58977, 0.52077));

    EXPECT_TRUE(-q == math::Quaternion(-0.891007, -0.121334,
                                       -0.242668, -0.364002));

    EXPECT_TRUE(q.GetAsMatrix3() == math::Matrix3(
                0.617229, -0.589769, 0.52077,
                0.707544, 0.705561, -0.0395554,
                -0.344106, 0.392882, 0.85278));

    EXPECT_TRUE(q.GetAsMatrix4() == math::Matrix4(
                0.617229, -0.589769, 0.52077, 0,
                0.707544, 0.705561, -0.0395554, 0,
                -0.344106, 0.392882, 0.85278, 0,
                0, 0, 0, 1));
  }

  // Test quaternion multiplication (rotation) order of application
  // if qa rotates frame o to p
  //    qb rotates frame p to q
  //    qc rotates frame q to r
  //    qd rotates frame r to s
  // then qd * qc * qb * qa rotates frame o to s
  EXPECT_EQ(math::Quaternion(0, 0, 0),
            math::Quaternion(0, -0.5*M_PI, 0)*
            math::Quaternion(-0.5*M_PI, 0, 0)*
            math::Quaternion(0,  0.5*M_PI, 0)*
            math::Quaternion(0, 0,  0.5*M_PI));
  EXPECT_EQ(math::Quaternion(0, 0, M_PI),
            math::Quaternion(0, 0,  0.5*M_PI)*
            math::Quaternion(0,  0.5*M_PI, 0)*
            math::Quaternion(-0.5*M_PI, 0, 0)*
            math::Quaternion(0, -0.5*M_PI, 0));
}

//////////////////////////////////////////////////
TEST_F(QuaternionTest, Integrate)
{
  // Integrate by zero, expect no change
  {
    const math::Quaternion q(0.5, 0.5, 0.5, 0.5);
    EXPECT_EQ(q, q.Integrate(math::Vector3::Zero, 1.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3::UnitX, 0.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3::UnitY, 0.0));
    EXPECT_EQ(q, q.Integrate(math::Vector3::UnitZ, 0.0));
  }

  // Integrate along single axes,
  // expect linear change in roll, pitch, yaw
  {
    const math::Quaternion q(1, 0, 0, 0);
    math::Quaternion qRoll  = q.Integrate(math::Vector3::UnitX, 1.0);
    math::Quaternion qPitch = q.Integrate(math::Vector3::UnitY, 1.0);
    math::Quaternion qYaw   = q.Integrate(math::Vector3::UnitZ, 1.0);
    EXPECT_EQ(qRoll.GetAsEuler(),  math::Vector3::UnitX);
    EXPECT_EQ(qPitch.GetAsEuler(), math::Vector3::UnitY);
    EXPECT_EQ(qYaw.GetAsEuler(),   math::Vector3::UnitZ);
  }

  // Integrate sequentially along single axes in order XYZ,
  // expect rotations to match Euler Angles
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qX   = q.Integrate(math::Vector3::UnitX, angle);
    math::Quaternion qXY  = qX.Integrate(math::Vector3::UnitY, angle);
    EXPECT_EQ(qXY.GetAsEuler(), angle*math::Vector3(1, 1, 0));
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qX   = q.Integrate(math::Vector3::UnitX, angle);
    math::Quaternion qXZ  = qX.Integrate(math::Vector3::UnitZ, angle);
    EXPECT_EQ(qXZ.GetAsEuler(), angle*math::Vector3(1, 0, 1));
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qY   = q.Integrate(math::Vector3::UnitY, angle);
    math::Quaternion qYZ  = qY.Integrate(math::Vector3::UnitZ, angle);
    EXPECT_EQ(qYZ.GetAsEuler(), angle*math::Vector3(0, 1, 1));
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qX   = q.Integrate(math::Vector3::UnitX, angle);
    math::Quaternion qXY  = qX.Integrate(math::Vector3::UnitY, angle);
    math::Quaternion qXYZ = qXY.Integrate(math::Vector3::UnitZ, angle);
    EXPECT_EQ(qXYZ.GetAsEuler(), angle*math::Vector3::One);
  }

  // Integrate sequentially along single axes in order ZYX,
  // expect rotations to not match Euler Angles
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qZ   = q.Integrate(math::Vector3::UnitZ, angle);
    math::Quaternion qZY  = qZ.Integrate(math::Vector3::UnitY, angle);
    EXPECT_NE(qZY.GetAsEuler().Ign(), (angle*math::Vector3(0, 1, 1)).Ign());
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qZ   = q.Integrate(math::Vector3::UnitZ, angle);
    math::Quaternion qZX  = qZ.Integrate(math::Vector3::UnitX, angle);
    EXPECT_NE(qZX.GetAsEuler().Ign(), angle*math::Vector3(1, 0, 1).Ign());
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qZ   = q.Integrate(math::Vector3::UnitZ, angle);
    math::Quaternion qZY  = qZ.Integrate(math::Vector3::UnitY, angle);
    math::Quaternion qZYX = qZY.Integrate(math::Vector3::UnitX, angle);
    EXPECT_NE(qZYX.GetAsEuler().Ign(), angle*math::Vector3(1, 1, 1).Ign());
  }
  {
    const math::Quaternion q(1, 0, 0, 0);
    const double angle = 0.5;
    math::Quaternion qY   = q.Integrate(math::Vector3::UnitY, angle);
    math::Quaternion qYX  = qY.Integrate(math::Vector3::UnitX, angle);
    EXPECT_NE(qYX.GetAsEuler().Ign(), angle*math::Vector3(1, 1, 0).Ign());
  }

  // Integrate a full rotation about different axes,
  // expect no change.
  {
    const math::Quaternion q(0.5, 0.5, 0.5, 0.5);
    const double fourPi = 4 * M_PI;
    math::Quaternion qX = q.Integrate(math::Vector3::UnitX, fourPi);
    math::Quaternion qY = q.Integrate(math::Vector3::UnitY, fourPi);
    math::Quaternion qZ = q.Integrate(math::Vector3::UnitZ, fourPi);
    EXPECT_EQ(q, qX);
    EXPECT_EQ(q, qY);
    EXPECT_EQ(q, qZ);
  }
}

