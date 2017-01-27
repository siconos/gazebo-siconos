/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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

#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class GzCamera : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test the at camera follow (gz camera -f <model> -c <camera>) moves
// the camera
TEST_F(GzCamera, Follow)
{
  Load("worlds/empty_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Spawn a box to follow.
  SpawnBox("box", ignition::math::Vector3d(1, 1, 1),
                  ignition::math::Vector3d(10, 10, 1),
                  ignition::math::Vector3d::Zero);
  ignition::math::Pose3d cameraStartPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraStartPose.Pos(), cameraStartPose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor("test_camera");
  ASSERT_TRUE(sensor != NULL);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor != NULL);
  rendering::CameraPtr camera = camSensor->Camera();
  ASSERT_TRUE(camera != NULL);

  // Make sure the sensor is at the correct initial pose
  EXPECT_EQ(camera->WorldPose(), cameraStartPose);

  SetPause(true);

  // Tell the camera to follow the box. The camera should move toward the
  // box.
  custom_exec("gz camera -c test_camera_model::body::test_camera -f box");
  world->Step(5000);

  // Make sure the camera is not at the initial pose.
  EXPECT_TRUE(camera->WorldPose() != cameraStartPose);

  EXPECT_NEAR(camera->WorldPose().Pos().X(), 9.9, 0.1);
  EXPECT_NEAR(camera->WorldPose().Pos().Y(), 9.9, 0.1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
