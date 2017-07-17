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

#include "gazebo/util/LogRecord.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class GzLog : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test log recording from gzserver
TEST_F(GzLog, Record)
{
  util::LogRecord *recorder = util::LogRecord::Instance();
  recorder->Init("test");
  Load("worlds/single_revolute_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  world->SetPaused(true);

  ASSERT_TRUE(recorder != NULL);

  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(recorder->Running());

  // Start log recording
  custom_exec("gz log -w default -d 1");
  world->Step(100);

  std::string filename = recorder->Filename();

  EXPECT_TRUE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(filename.empty());
  EXPECT_GT(recorder->FileSize(), 0u);

  // Stop log recording
  custom_exec("gz log -w default -d 0");

  EXPECT_FALSE(recorder->Running());

  std::string cmd = "gz log -i -f " + filename;
  std::string info = custom_exec(cmd);
  std::string logVersionStr = info.substr(
      info.find("Log Version:    ")+16, info.find("\n")-16);

  EXPECT_EQ(logVersionStr, GZ_LOG_VERSION);
}

/////////////////////////////////////////////////
/// Record a log file with filter
TEST_F(GzLog, RecordFilter)
{
  util::LogRecord *recorder = util::LogRecord::Instance();
  recorder->SetFilter("pendulum_0deg*");
  double period = 0.02;
  recorder->SetPeriod(period);
  recorder->Init("test");
  Load("worlds/revolute_joint_test.world");

  // Get a pointer to the world
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  ASSERT_TRUE(recorder != nullptr);

  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(recorder->Running());

  // Start log recording
  custom_exec("gz log -w default -d 1");

  world->Step(100);

  std::string filename = recorder->Filename();
  EXPECT_TRUE(recorder->Running());
  EXPECT_FALSE(recorder->Paused());
  EXPECT_FALSE(filename.empty());
  EXPECT_GT(recorder->FileSize(), 0u);

  // Stop log recording
  custom_exec("gz log -w default -d 0");

  EXPECT_FALSE(recorder->Running());

  // Store parsed output in a string
  std::string cmd = "gz log -e -f " + filename;
  std::string state = custom_exec(cmd);

  // Get sdf string containing world state
  auto sdfBeginIdx = state.find("<sdf version");
  auto sdfEndIdx = state.find("</sdf>");
  EXPECT_NE(sdfBeginIdx, std::string::npos);
  EXPECT_NE(sdfEndIdx, std::string::npos);
  std::string worldState = state.substr(sdfBeginIdx, sdfEndIdx+2);
  // Convert string to sdf element
  sdf::SDF worldSdf;
  worldSdf.SetFromString(worldState);
  EXPECT_TRUE(worldSdf.Root()->HasElement("world"));
  auto worldElem = worldSdf.Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("model"));
  auto modelElem = worldElem->GetElement("model");

  // check that all pendulum models are not filtered in the initial world state
  unsigned int pendulumCount = 0;
  while (modelElem)
  {
    std::string name = modelElem->Get<std::string>("name");
    if (name.find("pendulum") != std::string::npos)
      pendulumCount++;
    modelElem = modelElem->GetNextElement("model");
  }
  // there should be 8 pendulums in the world
  EXPECT_EQ(pendulumCount, 8u);

  // Parse state string and verify logging period and model name filter
  unsigned int filteredStateCount = 0;
  auto nextIdx = sdfEndIdx;
  std::string simTimeStr = "<sim_time>";
  double prevSimTime = -1;
  auto simTimeIdx = state.find(simTimeStr, nextIdx);
  auto nameIdx = state.find("<model name='", nextIdx);
  while (nameIdx != std::string::npos && simTimeIdx != std::string::npos)
  {
    // verify log period
    auto simTimeEndIdx = state.find("</sim_time>", simTimeIdx);
    std::string ns = state.substr(simTimeIdx + simTimeStr.size() + 2,
      simTimeEndIdx - simTimeIdx - simTimeStr.size() - 2);
    double simTime = std::stod(ns) / 1e9;
    EXPECT_GT(simTime, 0);
    if (prevSimTime >= 0)
      EXPECT_FLOAT_EQ(simTime - prevSimTime, period);
    prevSimTime = simTime;

    // verify model name - only models matching the filter string should exist
    auto nameBeginIdx = state.find("\'", nameIdx);
    auto nameEndIdx = state.find("\'", nameBeginIdx+1);
    EXPECT_NE(nameBeginIdx, std::string::npos);
    EXPECT_NE(nameEndIdx, std::string::npos);
    std::string modelName = state.substr(nameBeginIdx+1,
        nameEndIdx-nameBeginIdx-1);
    EXPECT_EQ(modelName, "pendulum_0deg");
    filteredStateCount++;
    nameIdx = state.find("<model name='", nameEndIdx+1);
    simTimeIdx = state.find("<sim_time>", nameEndIdx+1);
  }
  EXPECT_GT(filteredStateCount, 0u);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
