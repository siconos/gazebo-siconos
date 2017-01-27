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
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"


using namespace gazebo;
class ForceTorqueSensor_TEST : public ServerFixture,
                               public testing::WithParamInterface<const char*>
{
    public: void ForceTorqueTest(const std::string &_physicsEngine);
};

static std::string forceTorqueSensorString =
"<sdf version='1.4'>"
"  <sensor name='force_torque' type='force_torque'>"
"    <update_rate>30</update_rate>"
"    <always_on>true</always_on>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a ForceTorque sensor
void ForceTorqueSensor_TEST::ForceTorqueTest(const std::string &_physicsEngine)
{
  Load("worlds/pioneer2dx.world", true, _physicsEngine);

  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("sensor.sdf", sdf);
  sdf::readString(forceTorqueSensorString, sdf);

  physics::WorldPtr world = physics::get_world("default");
  physics::ModelPtr model = world->ModelByName("pioneer2dx");
  physics::JointPtr joint = model->GetJoint("left_wheel_hinge");

  // Create the force torque sensor
  std::string sensorName = mgr->CreateSensor(sdf, "default",
      "pioneer2dx::left_wheel_hinge", joint->GetId());

  // Make sure the returned sensor name is correct
  EXPECT_EQ(sensorName,
      std::string("default::pioneer2dx::left_wheel_hinge::force_torque"));

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the force torque sensor
  sensors::ForceTorqueSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        mgr->GetSensor(sensorName));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != nullptr);

  EXPECT_EQ(sensor->Torque(), ignition::math::Vector3d(0, 0, 0));
  EXPECT_EQ(sensor->Force(), ignition::math::Vector3d(0, 0, 0));

  EXPECT_TRUE(sensor->IsActive());
}

/////////////////////////////////////////////////
TEST_P(ForceTorqueSensor_TEST, ForceTorqueTest)
{
  ForceTorqueTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines,
                        ForceTorqueSensor_TEST,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
