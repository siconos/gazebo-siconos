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
#include <ignition/math/Helpers.hh>
#include <sdf/sdf.hh>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class RaySensor_TEST : public ServerFixture
{
};

static std::string raySensorString =
"<sdf version='1.3'>"
"  <sensor name='laser' type='ray'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <ray>"
"      <scan>"
"        <horizontal>"
"          <samples>640</samples>"
"          <resolution>1.000000</resolution>"
"          <min_angle>-2.2689</min_angle>"
"          <max_angle>2.2689</max_angle>"
"        </horizontal>"
"      </scan>"
"      <range>"
"        <min>0.08</min>"
"        <max>10.0</max>"
"        <resolution>0.01</resolution>"
"      </range>"
"    </ray>"
"  </sensor>"
"</sdf>";

static std::string raySensorScanResString =
"<sdf version='1.3'>"
"  <sensor name='laser' type='ray'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <ray>"
"      <scan>"
"        <horizontal>"
"          <samples>120</samples>"
"          <resolution>2</resolution>"
"          <min_angle>-2.2689</min_angle>"
"          <max_angle>2.2689</max_angle>"
"        </horizontal>"
"        <vertical>"
"          <samples>2</samples>"
"          <resolution>3</resolution>"
"          <min_angle>-2.2689</min_angle>"
"          <max_angle>2.2689</max_angle>"
"        </vertical>"
"      </scan>"
"      <range>"
"        <min>0.08</min>"
"        <max>10.0</max>"
"        <resolution>0.01</resolution>"
"      </range>"
"    </ray>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST_F(RaySensor_TEST, CreateLaser)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(raySensorString, sdf);

  // Create the Ray sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::laser"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the Ray sensor
  sensors::RaySensorPtr sensor = std::dynamic_pointer_cast<sensors::RaySensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  double angleRes = (sensor->AngleMax() - sensor->AngleMin()).Radian() /
                     sensor->RayCount();
  EXPECT_EQ(sensor->AngleMin(), ignition::math::Angle(-2.2689));
  EXPECT_EQ(sensor->AngleMax(), ignition::math::Angle(2.2689));
  EXPECT_NEAR(sensor->RangeMin(), 0.08, 1e-6);
  EXPECT_NEAR(sensor->RangeMax(), 10.0, 1e-6);
  EXPECT_NEAR(sensor->AngleResolution(), angleRes, 1e-3);
  EXPECT_NEAR(sensor->RangeResolution(), 0.01, 1e-3);
  EXPECT_EQ(sensor->RayCount(), 640);
  EXPECT_EQ(sensor->RangeCount(), 640);

  EXPECT_EQ(sensor->VerticalRayCount(), 1);
  EXPECT_EQ(sensor->VerticalRangeCount(), 1);
  EXPECT_EQ(sensor->VerticalAngleMin(), 0);
  EXPECT_EQ(sensor->VerticalAngleMax(), 0);

  EXPECT_TRUE(sensor->IsActive());

  // Update the sensor
  sensor->Update(true);

  // Get all the range values
  std::vector<double> ranges;
  sensor->Ranges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(640));

  // Check that all the range values
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(ranges[i], ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
    EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
    EXPECT_EQ(sensor->Fiducial(i), -1);
  }
}

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor with a scan resolution higher than 1
TEST_F(RaySensor_TEST, LaserScanResolution)
{
  Load("worlds/empty.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(raySensorScanResString, sdf);

  // Create the Ray sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::laser"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the Ray sensor
  sensors::RaySensorPtr sensor = std::dynamic_pointer_cast<sensors::RaySensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  // range count = ray count * resolution
  EXPECT_EQ(sensor->RayCount(), 120);
  EXPECT_EQ(sensor->RangeCount(), 240);

  EXPECT_EQ(sensor->VerticalRayCount(), 2);
  EXPECT_EQ(sensor->VerticalRangeCount(), 6);

  EXPECT_TRUE(sensor->IsActive());

  // Update the sensor
  sensor->Update(true);

  // Get all the range values
  std::vector<double> ranges;
  sensor->Ranges(ranges);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(240 * 6));

  // Check that all the range values
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(ranges[i], ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(sensor->Range(i), ranges[i]);
    EXPECT_NEAR(sensor->Retro(i), 0, 1e-6);
    EXPECT_EQ(sensor->Fiducial(i), -1);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
