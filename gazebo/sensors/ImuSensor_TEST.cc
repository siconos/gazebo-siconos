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

#include <sys/time.h>
#include <gtest/gtest.h>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/sensors/ImuSensor.hh"

#define TOL 1e-4

using namespace gazebo;
class ImuSensor_TEST : public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  public: void BasicImuSensorCheck(const std::string &_physicsEngine);
  public: void LinearAccelerationTest(const std::string &_physicsEngine);
};

static std::string imuSensorString =
"<sdf version='1.3'>"
"  <sensor name='imu' type='imu'>"
"    <always_on>1</always_on>"
"    <update_rate>20.000000</update_rate>"
"    <imu>"
"      <topic>/test_imu</topic>"
"    </imu>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
void ImuSensor_TEST::BasicImuSensorCheck(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(imuSensorString, sdf);

  // Create the IMU sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "ground_plane::link", 0);

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName, std::string("default::ground_plane::link::imu"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the IMU sensor
  sensors::ImuSensorPtr sensor = std::dynamic_pointer_cast<sensors::ImuSensor>
    (mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(sensor->AngularVelocity(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(sensor->LinearAcceleration(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(sensor->Orientation(), ignition::math::Quaterniond::Identity);
}

/////////////////////////////////////////////////
// Drop a model with imu sensor and measure its linear acceleration
void ImuSensor_TEST::LinearAccelerationTest(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  if (_physicsEngine == "simbody")
  {
    // default accuracy flunks this test, increase accuracy setting
    physics->SetParam("accuracy", 0.0001);
  }

  double z = 3;
  double gravityZ = world->Gravity().Z();
  double stepSize = physics->GetMaxStepSize();

  std::string modelName = "imuModel";
  std::string imuSensorName = "imuSensor";
  ignition::math::Pose3d modelPose(0, 0, z, 0, 0, 0);

  std::string topic = "~/" + imuSensorName + "_" + _physicsEngine;
  // spawn imu sensor
  SpawnUnitImuSensor(modelName, imuSensorName,
      "box", topic, modelPose.Pos(), modelPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(imuSensorName);
  sensors::ImuSensorPtr imuSensor =
      std::dynamic_pointer_cast<sensors::ImuSensor>(sensor);

  ASSERT_TRUE(imuSensor != nullptr);

  sensors::SensorManager::Instance()->Init();
  imuSensor->SetActive(true);

  EXPECT_EQ(imuSensor->AngularVelocity(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(imuSensor->LinearAcceleration(), ignition::math::Vector3d::Zero);
  EXPECT_EQ(imuSensor->Orientation(), ignition::math::Quaterniond::Identity);

  // step world and verify imu's linear acceleration is zero on free fall
  world->Step(200);
  EXPECT_NEAR(imuSensor->LinearAcceleration().X(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Y(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Z(), 0, TOL);
  world->Step(1);
  EXPECT_NEAR(imuSensor->LinearAcceleration().X(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Y(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Z(), 0, TOL);

  // Predict time of contact with ground plane.
  double tHit = sqrt((z-0.5) / (-gravityZ));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->SimTime().Double();
  double steps = ceil(dtHit / stepSize);
  EXPECT_GT(steps, 0);
  world->Step(steps);

  EXPECT_NEAR(imuSensor->LinearAcceleration().X(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Y(), 0, TOL);
  EXPECT_NEAR(imuSensor->LinearAcceleration().Z(), -gravityZ, 0.4);
}

/////////////////////////////////////////////////
TEST_P(ImuSensor_TEST, BasicImuSensorCheck)
{
  BasicImuSensorCheck(GetParam());
}

/////////////////////////////////////////////////
TEST_P(ImuSensor_TEST, LinearAccelerationTest)
{
  LinearAccelerationTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ImuSensor_TEST,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
