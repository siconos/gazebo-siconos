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
#include <string>

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
using namespace physics;

class BulletPhysics_TEST : public ServerFixture
{
  public: void PhysicsMsgParam();
  public: void OnPhysicsMsgResponse(ConstResponsePtr &_msg);
  public: static msgs::Physics physicsPubMsg;
  public: static msgs::Physics physicsResponseMsg;
};

msgs::Physics BulletPhysics_TEST::physicsPubMsg;
msgs::Physics BulletPhysics_TEST::physicsResponseMsg;

/////////////////////////////////////////////////
/// Test setting and getting bullet physics params
TEST_F(BulletPhysics_TEST, PhysicsParam)
{
  std::string physicsEngineStr = "bullet";
  Load("worlds/empty.world", true, physicsEngineStr);
  WorldPtr world = get_world("default");
  ASSERT_TRUE(world != nullptr);

  PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != nullptr);
  EXPECT_EQ(physics->GetType(), physicsEngineStr);

  BulletPhysicsPtr bulletPhysics
      = boost::static_pointer_cast<BulletPhysics>(physics);
  ASSERT_TRUE(bulletPhysics != nullptr);

  std::string type = "sequential_impulse";
  int iters = 45;
  double sor = 1.2;
  double cfm = 0.3;
  double erp = 0.12;
  double contactSurfaceLayer = 0.02;
  bool splitImpulse = true;
  double splitImpulsePenetrationThreshold = 0.02;

  // test setting/getting physics engine params
  bulletPhysics->SetParam("solver_type", type);
  bulletPhysics->SetParam("iters", iters);
  bulletPhysics->SetParam("sor", sor);
  bulletPhysics->SetParam("cfm", cfm);
  bulletPhysics->SetParam("erp", erp);
  bulletPhysics->SetParam("contact_surface_layer",
      contactSurfaceLayer);
  bulletPhysics->SetParam("split_impulse",
      splitImpulse);
  bulletPhysics->SetParam("split_impulse_penetration_threshold",
      splitImpulsePenetrationThreshold);

  boost::any value;
  value = bulletPhysics->GetParam("solver_type");
  std::string typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = bulletPhysics->GetParam("iters");
  int itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = bulletPhysics->GetParam("sor");
  double sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = bulletPhysics->GetParam("cfm");
  double cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = bulletPhysics->GetParam("erp");
  double erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);

  value = bulletPhysics->GetParam("contact_surface_layer");
  double contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);

  value = bulletPhysics->GetParam("split_impulse");
  double splitImpulseRet = boost::any_cast<bool>(value);
  EXPECT_DOUBLE_EQ(splitImpulse, splitImpulseRet);

  value = bulletPhysics->GetParam("split_impulse_penetration_threshold");
  double splitImpulsePenetrationThresholdRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(splitImpulsePenetrationThreshold,
    splitImpulsePenetrationThresholdRet);

  // Set params to different values and verify the old values are correctly
  // replaced by the new ones.
  iters = 55;
  sor = 1.4;
  cfm = 0.1;
  erp = 0.22;
  contactSurfaceLayer = 0.03;
  splitImpulse = true;
  splitImpulsePenetrationThreshold = 0.0;

  int maxContacts = 32;
  double minStepSize = 32.32;
  double maxStepSize = 3232.32;

  bulletPhysics->SetParam("solver_type", type);
  bulletPhysics->SetParam("iters", iters);
  bulletPhysics->SetParam("sor", sor);
  bulletPhysics->SetParam("cfm", cfm);
  bulletPhysics->SetParam("erp", erp);
  bulletPhysics->SetParam("contact_surface_layer",
      contactSurfaceLayer);
  bulletPhysics->SetParam("split_impulse",
      splitImpulse);
  bulletPhysics->SetParam("split_impulse_penetration_threshold",
      splitImpulsePenetrationThreshold);

  value = bulletPhysics->GetParam("solver_type");
  typeRet = boost::any_cast<std::string>(value);
  EXPECT_EQ(type, typeRet);
  value = bulletPhysics->GetParam("iters");
  itersRet = boost::any_cast<int>(value);
  EXPECT_EQ(iters, itersRet);
  value = bulletPhysics->GetParam("sor");
  sorRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(sor, sorRet);
  value = bulletPhysics->GetParam("cfm");
  cfmRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(cfm, cfmRet);
  value = bulletPhysics->GetParam("erp");
  erpRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(erp, erpRet);

  value = bulletPhysics->GetParam("contact_surface_layer");
  contactSurfaceLayerRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(contactSurfaceLayer, contactSurfaceLayerRet);

  value = bulletPhysics->GetParam("split_impulse");
  splitImpulseRet = boost::any_cast<bool>(value);
  EXPECT_DOUBLE_EQ(splitImpulse, splitImpulseRet);

  value = bulletPhysics->GetParam("split_impulse_penetration_threshold");
  splitImpulsePenetrationThresholdRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(splitImpulsePenetrationThreshold,
    splitImpulsePenetrationThresholdRet);

  int maxContactsRet;
  double minStepSizeRet;
  double maxStepSizeRet;
  bulletPhysics->SetParam("max_contacts", maxContacts);
  bulletPhysics->SetParam("min_step_size", minStepSize);
  bulletPhysics->SetParam("max_step_size", maxStepSize);
  value = bulletPhysics->GetParam("max_contacts");
  maxContactsRet = boost::any_cast<int>(value);
  EXPECT_DOUBLE_EQ(maxContacts, maxContactsRet);
  value = bulletPhysics->GetParam("min_step_size");
  minStepSizeRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(minStepSize, minStepSizeRet);
  value = bulletPhysics->GetParam("max_step_size");
  maxStepSizeRet = boost::any_cast<double>(value);
  EXPECT_DOUBLE_EQ(maxStepSize, maxStepSizeRet);
}

/////////////////////////////////////////////////
void BulletPhysics_TEST::OnPhysicsMsgResponse(ConstResponsePtr &_msg)
{
  if (_msg->type() == physicsPubMsg.GetTypeName())
    physicsResponseMsg.ParseFromString(_msg->serialized_data());
}

/////////////////////////////////////////////////
void BulletPhysics_TEST::PhysicsMsgParam()
{
  physicsPubMsg.Clear();
  physicsResponseMsg.Clear();

  std::string physicsEngineStr = "bullet";
  Load("worlds/empty.world", false, physicsEngineStr);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  physics::PhysicsEnginePtr engine = world->Physics();
  ASSERT_TRUE(engine != nullptr);

  transport::NodePtr phyNode;
  phyNode = transport::NodePtr(new transport::Node());
  phyNode->Init();

  transport::PublisherPtr physicsPub
       = phyNode->Advertise<msgs::Physics>("~/physics");
  transport::PublisherPtr requestPub
      = phyNode->Advertise<msgs::Request>("~/request");
  transport::SubscriberPtr responsePub = phyNode->Subscribe("~/response",
      &BulletPhysics_TEST::OnPhysicsMsgResponse, this);

  physicsPubMsg.set_enable_physics(true);
  physicsPubMsg.set_max_step_size(0.002);
  physicsPubMsg.set_real_time_update_rate(700);
  physicsPubMsg.set_real_time_factor(1.3);
  physicsPubMsg.set_iters(555);
  physicsPubMsg.set_sor(1.4);
  physicsPubMsg.set_cfm(0.12);
  physicsPubMsg.set_erp(0.23);
  physicsPubMsg.set_contact_surface_layer(0.01);
  physicsPubMsg.set_type(msgs::Physics::BULLET);
  physicsPubMsg.set_solver_type("sequential_impulse");
  physicsPub->Publish(physicsPubMsg);

  msgs::Request *requestMsg = msgs::CreateRequest("physics_info", "");
  requestPub->Publish(*requestMsg);

  int waitCount = 0, maxWaitCount = 3000;
  while (physicsResponseMsg.ByteSize() == 0 && ++waitCount < maxWaitCount)
    common::Time::MSleep(10);
  ASSERT_LT(waitCount, maxWaitCount);

  EXPECT_DOUBLE_EQ(physicsResponseMsg.max_step_size(),
      physicsPubMsg.max_step_size());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.real_time_update_rate(),
      physicsPubMsg.real_time_update_rate());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.real_time_factor(),
      physicsPubMsg.real_time_factor());
  EXPECT_EQ(physicsResponseMsg.solver_type(),
      physicsPubMsg.solver_type());
  EXPECT_EQ(physicsResponseMsg.enable_physics(),
      physicsPubMsg.enable_physics());
  EXPECT_EQ(physicsResponseMsg.iters(),
      physicsPubMsg.iters());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.sor(),
      physicsPubMsg.sor());
  EXPECT_DOUBLE_EQ(physicsResponseMsg.cfm(),
      physicsPubMsg.cfm());

  phyNode->Fini();
}

/////////////////////////////////////////////////
TEST_F(BulletPhysics_TEST, PhysicsMsgParam)
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
