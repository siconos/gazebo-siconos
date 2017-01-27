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
#include <map>
#include <string>
#include <ignition/math/Helpers.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

const double g_big = 1e17;
const double g_physics_tol = 1e-2;

class PhysicsCollisionTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Test Collision::GetBoundingBox.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetBoundingBox(const std::string &_physicsEngine);

  /// \brief Spawn identical models with different collision pose offsets
  /// and verify that they have matching behavior.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void PoseOffsets(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsCollisionTest::GetBoundingBox(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody" ||
      _physicsEngine == "dart")
  {
    gzerr << "Bounding boxes not yet working with "
          << _physicsEngine
          << ", see issue #1148"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check bounding box of ground plane
  {
    physics::ModelPtr model = world->ModelByName("ground_plane");
    ignition::math::Box box = model->BoundingBox();
    EXPECT_LT(box.Min().X(), -g_big);
    EXPECT_LT(box.Min().Y(), -g_big);
    EXPECT_LT(box.Min().Z(), -g_big);
    EXPECT_GT(box.Max().X(), g_big);
    EXPECT_GT(box.Max().Y(), g_big);
    EXPECT_DOUBLE_EQ(box.Max().Z(), 0.0);
  }
}

/////////////////////////////////////////////////
void PhysicsCollisionTest::PoseOffsets(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Box size
  const double dx = 0.9;
  const double dy = 0.4;
  const double dz = 0.9;
  const double mass = 10.0;
  const double angle = IGN_PI / 2.0;

  const unsigned int testCases = 4;
  for (unsigned int i = 0; i < testCases; ++i)
  {
    // Use msgs::AddBoxLink
    msgs::Model msgModel;
    ignition::math::Pose3d modelPose, linkPose, collisionPose;

    msgModel.set_name(this->GetUniqueString("model"));
    msgs::AddBoxLink(msgModel, mass, ignition::math::Vector3d(dx, dy, dz));
    modelPose.Pos().X() = i * dz * 5;
    modelPose.Pos().Z() = dz;
    double z0 = dz - dy/2;

    // i=0: rotated model pose
    //  expect collision pose to match model pose
    if (i == 0)
    {
      modelPose.Rot().Euler(angle, 0.0, 0.0);
    }
    // i=1: rotated link pose
    //  expect collision pose to match link pose
    else if (i == 1)
    {
      linkPose.Rot().Euler(angle, 0.0, 0.0);
    }
    // i=2: rotated collision pose
    //  expect collision pose to differ from link pose
    else if (i == 2)
    {
      collisionPose.Rot().Euler(angle, 0.0, 0.0);
    }
    // i=3: offset collision pose
    //  expect collision pose to differ from link pose
    else if (i == 3)
    {
      collisionPose.Pos().Set(0, 0, dz);
      z0 = 1.5 * dz;
    }

    {
      auto msgLink = msgModel.mutable_link(0);
      auto msgCollision = msgLink->mutable_collision(0);

      msgs::Set(msgModel.mutable_pose(), modelPose);
      msgs::Set(msgLink->mutable_pose(), linkPose);
      msgs::Set(msgCollision->mutable_pose(), collisionPose);
    }

    auto model = this->SpawnModel(msgModel);
    ASSERT_TRUE(model != nullptr);

    auto link = model->GetLink();
    ASSERT_TRUE(link != nullptr);

    const unsigned int index = 0;
    auto collision = link->GetCollision(index);
    ASSERT_TRUE(collision != nullptr);

    EXPECT_EQ(model->WorldPose(), modelPose);
    EXPECT_EQ(link->WorldPose(), linkPose + modelPose);
    EXPECT_EQ(collision->WorldPose(), collisionPose + linkPose + modelPose);

    // i=0: rotated model pose
    //  expect collision pose to match model pose
    if (i == 0)
    {
      EXPECT_EQ(model->WorldPose(), collision->WorldPose());
    }
    // i=1: rotated link pose
    //  expect collision pose to match link pose
    else if (i == 1)
    {
      EXPECT_EQ(link->WorldPose(), collision->WorldPose());
    }
    // i=2: rotated collision pose
    //  expect collision position to match link position
    else if (i == 2)
    {
      EXPECT_EQ(link->WorldPose().Pos(), collision->WorldPose().Pos());
    }
    // i=3: offset collision pose
    //  expect collision postion to match link position plus offset
    else if (i == 3)
    {
      EXPECT_EQ(link->WorldPose().Pos() + collisionPose.Pos(),
                collision->WorldPose().Pos());
    }

    auto physics = world->Physics();
    ASSERT_TRUE(physics != nullptr);
    physics->SetRealTimeUpdateRate(0);
    const double dt = physics->GetMaxStepSize();
    const double g = world->Gravity().Z();
    EXPECT_DOUBLE_EQ(dt, 1e-3);
    ASSERT_DOUBLE_EQ(g, -9.8);
    const double t0 = 1 + sqrt(2*z0 / (-g));
    const int steps = floor(t0 / dt);
    world->Step(steps);

    // For 0-2, drop and expect box to rest at specific height
    if (i <= 2)
    {
      EXPECT_NEAR(collision->WorldPose().Pos().Z(), dy/2, g_physics_tol);
    }
    else
    {
      EXPECT_NEAR(collision->WorldPose().Pos().Z(), dz/2, g_physics_tol);
    }
  }
}

/////////////////////////////////////////////////
TEST_F(PhysicsCollisionTest, ModelSelfCollide)
{
  // self_collide is only implemented in ODE
  Load("worlds/model_self_collide.world", true, "ode");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);

  auto g = world->Gravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.X(), 0);
  EXPECT_EQ(g.Y(), 0);
  EXPECT_LE(g.Z(), -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // 4 models: all_collide, some_collide, no_collide, and explicit_no_collide
  std::map<std::string, physics::ModelPtr> models;
  models["all_collide"]         = physics::ModelPtr();
  models["some_collide"]        = physics::ModelPtr();
  models["no_collide"]          = physics::ModelPtr();
  models["explicit_no_collide"] = physics::ModelPtr();
  for (auto &iter : models)
  {
    gzdbg << "Getting model " << iter.first << std::endl;
    iter.second = world->ModelByName(iter.first);
    ASSERT_TRUE(iter.second != NULL);
  }

  // Step forward 0.2 s
  double stepTime = 0.2;
  unsigned int steps = floor(stepTime / dt);
  world->Step(steps);

  // Expect boxes to be falling
  double fallVelocity = g.Z() * stepTime;
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check falling: " << link->GetScopedName() << std::endl;
      EXPECT_LT(link->WorldLinearVel().Z(), fallVelocity*(1-g_physics_tol));
    }
  }

  // Another 3000 steps should put the boxes at rest
  world->Step(3000);

  // Expect 3 boxes to be stationary
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check resting: " << link->GetScopedName() << std::endl;
      EXPECT_NEAR(link->WorldLinearVel().Z(), 0, g_physics_tol);
    }
  }

  gzdbg << "Check resting positions" << std::endl;

  // link2 of all_collide should have the highest z-coordinate (around 3)
  EXPECT_NEAR(models["all_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              2.5, g_physics_tol);

  // link2 of some_collide should have a middling z-coordinate (around 2)
  EXPECT_NEAR(models["some_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              1.5, g_physics_tol);

  // link2 of no_collide should have a low z-coordinate (around 1)
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              0.5, g_physics_tol);

  // link2 of explicit_no_collide should have the same z-coordinate as above
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
     models["explicit_no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
     g_physics_tol);

  Unload();
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, GetBoundingBox)
{
  GetBoundingBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, PoseOffsets)
{
  PoseOffsets(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsCollisionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
