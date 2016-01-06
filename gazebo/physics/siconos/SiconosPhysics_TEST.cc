/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
using namespace physics;

class SiconosPhysics_TEST : public ServerFixture
{
  public: void PhysicsMsgParam();
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: static msgs::Physics physicsPubMsg;
  public: static msgs::Physics physicsResponseMsg;
};

msgs::Physics SiconosPhysics_TEST::physicsPubMsg;
msgs::Physics SiconosPhysics_TEST::physicsResponseMsg;

/////////////////////////////////////////////////
/// Test setting and getting siconos physics params
TEST_F(SiconosPhysics_TEST, PhysicsParam)
{
  std::string physicsEngineStr = "siconos";
  Load("worlds/empty.world", true, physicsEngineStr);
  WorldPtr world = get_world("default");
  ASSERT_TRUE(world != NULL);

  PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), physicsEngineStr);

  SiconosPhysicsPtr siconosPhysics
      = boost::static_pointer_cast<SiconosPhysics>(physics);
  ASSERT_TRUE(siconosPhysics != NULL);

  std::string type = "quick";
  int preconIters = 5;
  int iters = 45;
  double sor = 1.2;
  double cfm = 0.3;
  double erp = 0.12;
  double contactMaxCorrectingVel = 50;
  double contactSurfaceLayer = 0.02;
  double contactResidualSmoothing = 0.1;
  double contactSorScale = 1.0;
  bool threadPositionCorrection = true;
  bool experimentalRowReordering = true;
  double warmStartFactor = 1.0;
  int extraFrictionIterations = 15;

  // test setting/getting physics engine params
  // TODO

  // verify against equivalent functions
  // TODO

  // Set params to different values and verify the old values are correctly
  // replaced by the new ones.
  type = "world";
  preconIters = 10;
  iters = 55;
  sor = 1.4;
  cfm = 0.1;
  erp = 0.22;
  contactMaxCorrectingVel = 40;
  contactSurfaceLayer = 0.03;
  contactResidualSmoothing = 0.09;
  contactSorScale = 0.9;
  threadPositionCorrection = false;
  experimentalRowReordering = false;
  warmStartFactor = 0.9;
  extraFrictionIterations = 14;

  // TODO
}

/////////////////////////////////////////////////
void SiconosPhysics_TEST::OnPhysicsMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == physicsPubMsg.GetTypeName())
    physicsResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void SiconosPhysics_TEST::PhysicsMsgParam()
{
  physicsPubMsg.Clear();
  physicsResponseMsg.Clear();

  std::string physicsEngineStr = "siconos";
  Load("worlds/empty.world", false, physicsEngineStr);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
  ASSERT_TRUE(engine != NULL);

  transport::NodePtr phyNode;
  phyNode = transport::NodePtr(new transport::Node());
  phyNode->Init();

  transport::PublisherPtr physicsPub
       = phyNode->Advertise<msgs::Physics>("~/physics");
  transport::PublisherPtr requestPub
      = phyNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responseSub = phyNode->Subscribe("~/response",
      &SiconosPhysics_TEST::OnPhysicsMsgResponse, this);

  // TODO
  physicsPubMsg.set_type(msgs::Physics::SICONOS);

  physicsPub->Publish(physicsPubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("physics_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (physicsResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);
  ASSERT_LT(waitCount, maxWaitCount);

  // TODO

  phyNode->Fini();
}

/////////////////////////////////////////////////
TEST_F(SiconosPhysics_TEST, PhysicsMsgParam)
{
  PhysicsMsgParam();
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
