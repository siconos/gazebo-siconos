/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class SpeedTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void BallTest(const std::string &_physicsEngine);
  public: void ShapesWorld(const std::string &_physicsEngine);
};

void SpeedTest::BallTest(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  double emptySpeed;
  while ((emptySpeed = GetPercentRealTime()) == 0)
    common::Time::MSleep(100);
  common::Time::MSleep(2000);
  emptySpeed = GetPercentRealTime();

  // Load 500 spheres into the world
  std::string name = "sphere";
  math::Vector3 pos(0, 0, 5);

  for (int i = 0; i < 500; ++i)
  {
    pos.z += i;
    SpawnSphere(name + boost::lexical_cast<std::string>(i),
        pos, math::Vector3(0, 0, 0), i == 499);
  }

  common::Time::MSleep(2000);
  double loadedSpeed = GetPercentRealTime();

  double speedRatio = loadedSpeed / emptySpeed;

  std::cout << "Speed: Empty[" << emptySpeed << "] Loaded["
            << loadedSpeed << "] Ratio[" << speedRatio << "]\n";

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speedRatio, 0.02);
#else
  EXPECT_GT(speedRatio, 0.01);
#endif
}

TEST_P(SpeedTest, BallTest)
{
  BallTest(GetParam());
}

void SpeedTest::ShapesWorld(const std::string &_physicsEngine)
{
  Load("worlds/shapes.world", false, _physicsEngine);
  double emptySpeed;
  while ((emptySpeed = GetPercentRealTime()) == 0)
    common::Time::MSleep(100);
  common::Time::MSleep(2000);
  emptySpeed = GetPercentRealTime();

  // Load 500 spheres into the world
  std::string name = "sphere";
  math::Vector3 pos(0, 0, 5);
  for (int i = 0; i < 500; ++i)
  {
    pos.z += i;
    SpawnSphere(name + boost::lexical_cast<std::string>(i),
        pos, math::Vector3(0, 0, 0), i == 499);
  }
  common::Time::MSleep(2000);

  double loadedSpeed = GetPercentRealTime();
  double speedRatio = loadedSpeed / emptySpeed;

  std::cout << "Speed: Empty[" << emptySpeed << "] Loaded["
            << loadedSpeed << "] Ratio[" << speedRatio << "]\n";

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speedRatio, 0.08);
#else
  EXPECT_GT(speedRatio, 0.01);
#endif
}

TEST_P(SpeedTest, ShapesWorld)
{
  ShapesWorld(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, SpeedTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
