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

#include <ignition/math/Helpers.hh>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/sensors/sensors.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class GPURaySensorTest : public ServerFixture
{
};

void OnNewLaserFrame(int *_scanCounter, float *_scanDest,
                  const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_scanDest, _scan, _width * _height * _depth);
  *_scanCounter += 1;
}

/////////////////////////////////////////////////
/// \brief Test GPU ray sensor range values,
/// Adapted from LaserUnitBox test in laser.cc
TEST_F(GPURaySensorTest, LaserUnitBox)
{
  // Test GPU ray sensors with 3 boxes in the world.
  // First GPU ray sensor at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  std::string modelName = "gpu_ray_model";
  std::string raySensorName = "gpu_ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  // Spawn another gpu ray sensor at 90 degree roll
  std::string modelName2 = "gpu_ray_model_roll";
  std::string raySensorName2 = "gpu_ray_sensor_roll";
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(M_PI/2.0, 0, 0));

  SpawnGpuRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  SpawnGpuRaySensor(modelName2, raySensorName2, testPose2.Pos(),
      testPose2.Rot().Euler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Physics()->SetGravity(ignition::math::Vector3d::Zero);

  // box in front of ray sensor 1 and 2
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the right of ray sensor 1
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -1, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the left of the ray sensor 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);

  SpawnBox(box01, ignition::math::Vector3d(1, 1, 1), box01Pose.Pos(),
      box01Pose.Rot().Euler());

  SpawnBox(box02, ignition::math::Vector3d(1, 1, 1), box02Pose.Pos(),
      box02Pose.Rot().Euler());

  SpawnBox(box03, ignition::math::Vector3d(1, 1, 1), box03Pose.Pos(),
      box03Pose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::GpuRaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  sensors::SensorPtr sensor2 = sensors::get_sensor(raySensorName2);
  sensors::GpuRaySensorPtr raySensor2 =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor2);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(raySensor != NULL);
  EXPECT_TRUE(raySensor2 != NULL);

  raySensor->SetActive(true);
  raySensor2->SetActive(true);

  // Verify ray sensor 1 range readings
  // listen to new laser frames
  float *scan = new float[raySensor->RayCount()
      * raySensor->VerticalRayCount() * 3];
  int scanCount = 0;
  event::ConnectionPtr c =
    raySensor->ConnectNewLaserFrame(
        std::bind(&::OnNewLaserFrame, &scanCount, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // wait for a few laser scans
  int i = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  int mid = samples / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2;

  // ray sensor 1 should see box01 and box02
  EXPECT_NEAR(raySensor->Range(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_NEAR(raySensor->Range(0), expectedRangeAtMidPoint, LASER_TOL);

  EXPECT_DOUBLE_EQ(raySensor->Range(samples-1), ignition::math::INF_D);

  // Verify ray sensor 2 range readings
  // listen to new laser frames
  float *scan2 = new float[raySensor2->RayCount()
      * raySensor2->VerticalRayCount() * 3];
  int scanCount2 = 0;
  event::ConnectionPtr c2 =
    raySensor->ConnectNewLaserFrame(
        std::bind(&::OnNewLaserFrame, &scanCount2, scan2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // wait for a few laser scans
  i = 0;
  scanCount2 = 0;
  while (scanCount2 < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // Only box01 should be visible to ray sensor 2
  EXPECT_NEAR(raySensor2->Range(mid), expectedRangeAtMidPoint, LASER_TOL);
  EXPECT_DOUBLE_EQ(raySensor2->Range(0), ignition::math::INF_D);
  EXPECT_DOUBLE_EQ(raySensor->Range(samples-1), ignition::math::INF_D);

  // Move all boxes out of range
  world->ModelByName(box01)->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(maxRange + 1, 0, 0),
      ignition::math::Quaterniond::Identity));
  world->ModelByName(box02)->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(0, -(maxRange + 1), 0),
      ignition::math::Quaterniond::Identity));

  // wait for a few more laser scans
  i = 0;
  scanCount = 0;
  scanCount2 = 0;
  while ((scanCount < 10 ||scanCount2 < 10) && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (int i = 0; i < raySensor->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(raySensor->Range(i), ignition::math::INF_D);

  for (int i = 0; i < raySensor->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(raySensor2->Range(i), ignition::math::INF_D);

  c.reset();
  c2.reset();

  delete [] scan;
  delete [] scan2;
}

/////////////////////////////////////////////////
/// \brief Spawn multiple GPU ray sensors with same name.
/// Verify that it doesn't crash.
TEST_F(GPURaySensorTest, NameCollision)
{
  // Test GPU ray sensors with 3 boxes in the world.
  // First GPU ray sensor at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values
  Load("worlds/empty_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  std::string modelName = "gpu_ray_model";
  std::string raySensorName = "gpu_ray_sensor";
  double hMinAngle = -M_PI/2.0;
  double hMaxAngle = M_PI/2.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  double rangeResolution = 0.02;
  unsigned int samples = 320;
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  // Spawn another gpu ray sensor at 90 degree roll
  std::string modelName2 = "gpu_ray_model_roll";
  std::string raySensorName2 = "gpu_ray_sensor";
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(M_PI/2.0, 0, 0));

  SpawnGpuRaySensor(modelName, raySensorName, testPose.Pos(),
      testPose.Rot().Euler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  SpawnGpuRaySensor(modelName2, raySensorName2, testPose2.Pos(),
      testPose2.Rot().Euler(), hMinAngle, hMaxAngle, minRange, maxRange,
      rangeResolution, samples);

  std::string box01 = "box_01";
  std::string box02 = "box_02";
  std::string box03 = "box_03";

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->Physics()->SetGravity(ignition::math::Vector3d::Zero);

  // box in front of ray sensor 1 and 2
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the right of ray sensor 1
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -1, 0.5),
                                   ignition::math::Quaterniond::Identity);
  // box on the left of the ray sensor 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);

  SpawnBox(box01, ignition::math::Vector3d(1, 1, 1), box01Pose.Pos(),
      box01Pose.Rot().Euler());

  SpawnBox(box02, ignition::math::Vector3d(1, 1, 1), box02Pose.Pos(),
      box02Pose.Rot().Euler());

  SpawnBox(box03, ignition::math::Vector3d(1, 1, 1), box03Pose.Pos(),
      box03Pose.Rot().Euler());

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::GpuRaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  sensors::SensorPtr sensor2 = sensors::get_sensor(raySensorName2);
  sensors::GpuRaySensorPtr raySensor2 =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor2);

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(raySensor != NULL);
  EXPECT_TRUE(raySensor2 != NULL);
}

/////////////////////////////////////////////////
/// \brief Test GPU ray sensor interaction with terrain
TEST_F(GPURaySensorTest, Heightmap)
{
  Load("worlds/gpu_laser_heightmap.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run gpu laser test\n";
    return;
  }

  // Get a pointer to the gpu laser sensor
  std::string gpuLaserName = "gpu_laser_sensor";
  int t = 0;
  while (sensors::get_sensor(gpuLaserName) == NULL && t < 100)
  {
    common::Time::MSleep(100);
    ++t;
  }
  ASSERT_LT(t, 100);
  sensors::SensorPtr sensor = sensors::get_sensor(gpuLaserName);
  sensors::GpuRaySensorPtr raySensor =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

  EXPECT_TRUE(raySensor != NULL);

  // listen to new laser frames
  float *scan = new float[raySensor->RayCount()
      * raySensor->VerticalRayCount() * 3];
  int scanCount = 0;
  event::ConnectionPtr c =
    raySensor->ConnectNewLaserFrame(
        std::bind(&::OnNewLaserFrame, &scanCount, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  // wait for a few laser scans
  int i = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  // Verify initial laser range readings. Nothing should be intersecting
  double maxRange = 10;
  EXPECT_NEAR(raySensor->RangeMax(), maxRange, LASER_TOL);

  for (int i = 0; i < raySensor->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(raySensor->Range(i), ignition::math::INF_D);

  // Move laser model very close to terrain, it should now returns range values
  // that are less than half the max range
  std::string gpuLaserModelName = "gpu_laser";
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->ModelByName(gpuLaserModelName)->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(13.2, 0, 0.035),
      ignition::math::Quaterniond::Identity));

  // wait for a few laser scans
  i = 0;
  scanCount = 0;
  while (scanCount < 10 && i < 300)
  {
    common::Time::MSleep(10);
    i++;
  }
  EXPECT_LT(i, 300);

  for (int i = 0; i < raySensor->RayCount(); ++i)
    EXPECT_TRUE(raySensor->Range(i) < maxRange / 2.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
