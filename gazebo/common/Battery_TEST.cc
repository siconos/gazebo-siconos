/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Battery.hh"
#include "test/util.hh"

using namespace gazebo;

class BatteryTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(BatteryTest, Construction)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != nullptr);

  EXPECT_DOUBLE_EQ(battery->Voltage(), 0.0);
  EXPECT_EQ(battery->PowerLoads().size(), 0u);
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, AddConsumer)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != nullptr);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, 0u);
  EXPECT_EQ(battery->PowerLoads().size(), 1u);

  battery->SetPowerLoad(consumerId, 5.0);

  double powerLoad = 0;
  EXPECT_TRUE(battery->PowerLoad(consumerId, powerLoad));
  EXPECT_DOUBLE_EQ(powerLoad, 5.0);

  // Resetting the voltage has no effect on the power load
  battery->ResetVoltage();
  EXPECT_TRUE(battery->PowerLoad(consumerId, powerLoad));
  EXPECT_DOUBLE_EQ(powerLoad, 5.0);

  // Reinitializing the battery discard any power load
  battery->Init();
  EXPECT_EQ(battery->PowerLoads().size(), 0u);
  EXPECT_FALSE(battery->PowerLoad(consumerId, powerLoad));
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, RemoveConsumer)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != nullptr);

  uint32_t consumerId = battery->AddConsumer();
  EXPECT_EQ(consumerId, 0u);
  EXPECT_EQ(battery->PowerLoads().size(), 1u);

  double powerLoad = 1.0;
  EXPECT_TRUE(battery->SetPowerLoad(consumerId, powerLoad));
  EXPECT_TRUE(battery->PowerLoad(consumerId, powerLoad));
  EXPECT_DOUBLE_EQ(powerLoad, 1.0);

  uint32_t consumerId2 = battery->AddConsumer();

  EXPECT_TRUE(battery->RemoveConsumer(consumerId));
  EXPECT_EQ(battery->PowerLoads().size(), 1u);

  uint32_t consumerId3 = battery->AddConsumer();
  EXPECT_TRUE(battery->RemoveConsumer(consumerId3));
  uint32_t consumerId4 = battery->AddConsumer();

  EXPECT_FALSE(consumerId == consumerId2);
  EXPECT_FALSE(consumerId == consumerId3);
  EXPECT_FALSE(consumerId == consumerId4);

  EXPECT_FALSE(consumerId2 == consumerId3);
  EXPECT_FALSE(consumerId2 == consumerId4);

  EXPECT_FALSE(consumerId3 == consumerId4);

  EXPECT_FALSE(battery->RemoveConsumer(25));
}

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetPowerLoad)
{
  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != nullptr);

  // Add two consumers
  uint32_t consumerId1 = battery->AddConsumer();
  uint32_t consumerId2 = battery->AddConsumer();
  EXPECT_EQ(battery->PowerLoads().size(), 2u);

  // Set consumers power load
  double powerLoad1 = 1.0;
  double powerLoad2 = 2.0;
  EXPECT_TRUE(battery->SetPowerLoad(consumerId1, powerLoad1));
  EXPECT_TRUE(battery->SetPowerLoad(consumerId2, powerLoad2));

  // Check consumers power load
  EXPECT_TRUE(battery->PowerLoad(consumerId1, powerLoad1));
  EXPECT_DOUBLE_EQ(powerLoad1, 1.0);
  EXPECT_TRUE(battery->PowerLoad(consumerId2, powerLoad2));
  EXPECT_DOUBLE_EQ(powerLoad2, 2.0);
}

/// \brief A fixture class to help with updating the battery voltage.
class BatteryUpdateFixture
{
  /// \brief Update voltage by incrementing it.
  public: double Update(const common::BatteryPtr _battery)
          {
            return _battery->Voltage() + this->step;
          }

  /// \brief Voltage amount to increment by.
  public: double step;
};

/////////////////////////////////////////////////
TEST_F(BatteryTest, SetUpdateFunc)
{
  int N = 10;
  const double initVoltage = 12.0;

  std::ostringstream batteryStr;
  batteryStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<battery name='battery'>"
    <<     "<voltage>" << initVoltage << "</voltage>"
    <<   "</battery>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr batterySDF(new sdf::SDF);
  batterySDF->SetFromString(batteryStr.str());

  // Create the battery
  common::BatteryPtr battery(new common::Battery());
  EXPECT_TRUE(battery != nullptr);

  sdf::ElementPtr elem = batterySDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("battery");
  ASSERT_TRUE(elem != nullptr);
  battery->Load(elem);

  battery->Init();
  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage);

  BatteryUpdateFixture fixture;
  fixture.step = -0.1;
  battery->SetUpdateFunc(std::bind(&BatteryUpdateFixture::Update,
        &fixture, std::placeholders::_1));

  for (int i = 0; i < N; ++i)
    battery->Update();

  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage + N * fixture.step);

  // Reinitialize the battery, and expect the same result
  battery->Init();
  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage);

  for (int i = 0; i < N; ++i)
    battery->Update();

  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage + N * fixture.step);

  // Reset the voltage to its initial value, and expect the same result
  battery->ResetVoltage();
  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage);

  for (int i = 0; i < N; ++i)
    battery->Update();

  EXPECT_DOUBLE_EQ(battery->Voltage(), initVoltage + N * fixture.step);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
