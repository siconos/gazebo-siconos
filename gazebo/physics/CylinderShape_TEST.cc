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

#include "gazebo/physics/CylinderShape.hh"
#include "test/util.hh"

using namespace gazebo;

class CylinderShapeTest : public gazebo::testing::AutoLogFixture { };

TEST_F(CylinderShapeTest, Scale)
{
  std::ostringstream cylinderStr;
  cylinderStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<cylinder>"
    <<         "<radius>0.5</radius>"
    <<         "<length>1.0</length>"
    <<       "</cylinder>"
    <<     "</geometry>"
    <<   "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr cylinderSDF(new sdf::SDF);
  cylinderSDF->SetFromString(cylinderStr.str());

  physics::CylinderShapePtr cylinder(
      new physics::CylinderShape(physics::CollisionPtr()));
  sdf::ElementPtr elem = cylinderSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("collision");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("geometry");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("cylinder");
  ASSERT_TRUE(elem != NULL);
  cylinder->Load(elem);

  // Test scaling with unit size
  double radius = cylinder->GetRadius();
  double length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.5);
  EXPECT_DOUBLE_EQ(length, 1.0);

  cylinder->SetScale(ignition::math::Vector3d(1.5, 1.5, 1.5));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.75);
  EXPECT_DOUBLE_EQ(length, 1.5);

  cylinder->SetScale(ignition::math::Vector3d(2.0, 2.0, 2.0));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 1.0);
  EXPECT_DOUBLE_EQ(length, 2.0);

  // reset scale
  cylinder->SetScale(ignition::math::Vector3d(1.0, 1.0, 1.0));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.5);
  EXPECT_DOUBLE_EQ(length, 1.0);

  // Test scaling with non-unit size
  cylinder->SetRadius(2.5);
  cylinder->SetLength(3.5);
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 2.5);
  EXPECT_DOUBLE_EQ(length, 3.5);

  cylinder->SetScale(ignition::math::Vector3d(2.0, 2.0, 2.0));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 5.0);
  EXPECT_DOUBLE_EQ(length, 7.0);

  cylinder->SetScale(ignition::math::Vector3d(100.0, 100.0, 100.0));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 250.0);
  EXPECT_DOUBLE_EQ(length, 350.0);

  cylinder->SetScale(ignition::math::Vector3d(0.1, 0.1, 0.1));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.25);
  EXPECT_DOUBLE_EQ(length, 0.35);

  // reset scale
  cylinder->SetScale(ignition::math::Vector3d(1.0, 1.0, 1.0));
  cylinder->SetRadius(0.5);
  cylinder->SetLength(1.0);
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.5);
  EXPECT_DOUBLE_EQ(length, 1.0);

  // Test scaling with different x, y and z components
  cylinder->SetScale(ignition::math::Vector3d(0.5, 1.0, 2.5));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  // radius should be multiplied by max of (0.5, 1.0)
  EXPECT_DOUBLE_EQ(radius, 0.5);
  EXPECT_DOUBLE_EQ(length, 2.5);

  // Test scaling with negative components
  // This should fail and radius and length should remain the same as before
  cylinder->SetScale(ignition::math::Vector3d(-1.0, -2.0, -3.0));
  radius = cylinder->GetRadius();
  length = cylinder->GetLength();
  EXPECT_DOUBLE_EQ(radius, 0.5);
  EXPECT_DOUBLE_EQ(length, 2.5);
}

TEST_F(CylinderShapeTest, Volume)
{
  std::ostringstream cylinderStr;
  cylinderStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<cylinder>"
    <<         "<radius>1.0</radius>"
    <<         "<length>1.0</length>"
    <<       "</cylinder>"
    <<     "</geometry>"
    <<   "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr cylinderSDF(new sdf::SDF);
  cylinderSDF->SetFromString(cylinderStr.str());

  physics::CylinderShapePtr cylinder(
      new physics::CylinderShape(physics::CollisionPtr()));
  sdf::ElementPtr elem = cylinderSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("collision");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("geometry");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("cylinder");
  ASSERT_TRUE(elem != NULL);
  cylinder->Load(elem);

  EXPECT_FLOAT_EQ(cylinder->ComputeVolume(), M_PI);
  cylinder->SetLength(3);
  cylinder->SetRadius(2);
  EXPECT_FLOAT_EQ(cylinder->ComputeVolume(), M_PI*4*3);

  // The bounding box approximation should be 0 because the Shape has no
  // Collision parent
  EXPECT_DOUBLE_EQ(cylinder->Shape::ComputeVolume(), 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
