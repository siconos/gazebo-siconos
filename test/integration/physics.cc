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

#include <map>
#include <string>
#include <vector>

#include <ignition/math/Rand.hh>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "SimplePendulumIntegrator.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/test/helper_physics_generator.hh"

#ifdef HAVE_BULLET
#include "gazebo/physics/bullet/bullet_math_inc.h"
#endif

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void InelasticCollision(const std::string &_physicsEngine);
  public: void EmptyWorld(const std::string &_physicsEngine);
  public: void SpawnDrop(const std::string &_physicsEngine);
  public: void SpawnDropCoGOffset(const std::string &_physicsEngine);
  public: void SphereAtlasLargeError(const std::string &_physicsEngine);
  public: void CollisionFiltering(const std::string &_physicsEngine);
  public: void JointDampingTest(const std::string &_physicsEngine);
  public: void DropStuff(const std::string &_physicsEngine);
};

////////////////////////////////////////////////////////////////////////
// EmptyWorld:
// Load a world, take a few steps, and verify that time is increasing.
// This is the most basic physics engine test.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::EmptyWorld(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // simulate 1 step
  world->Step(1);
  double t = world->SimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->Step(steps);
  double dt = world->Physics()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  t = world->SimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

TEST_P(PhysicsTest, EmptyWorld)
{
  EmptyWorld(GetParam());
}

////////////////////////////////////////////////////////////////////////
// SpawnDrop:
// Load a world, check that gravity points along z axis, spawn simple
// shapes (box, sphere, cylinder), verify that they fall and hit the
// ground plane. The test currently assumes inelastic collisions.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SpawnDrop(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  auto g = world->Gravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.X(), 0);
  EXPECT_EQ(g.Y(), 0);
  EXPECT_LE(g.Z(), -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // spawn some simple shapes and check to see that they start falling
  double z0 = 3;
  std::map<std::string, ignition::math::Vector3d> modelPos;
  modelPos["test_box"] = ignition::math::Vector3d(0, 0, z0);
  modelPos["test_sphere"] = ignition::math::Vector3d(4, 0, z0);
  modelPos["test_cylinder"] = ignition::math::Vector3d(8, 0, z0);
  modelPos["test_empty"] = ignition::math::Vector3d(12, 0, z0);
  modelPos["link_offset_box"] = ignition::math::Vector3d(0, 0, z0);

  // FIXME Trimesh drop test passes in bullet but fails in ode because
  // the mesh bounces to the side when it hits the ground.
  // See issue #513. Uncomment test when issue is resolved.
  // modelPos["test_trimesh"] = ignition::math::Vector3d(16, 0, z0);

  SpawnBox("test_box", ignition::math::Vector3d(1, 1, 1),
      modelPos["test_box"], ignition::math::Vector3d::Zero);
  SpawnSphere("test_sphere", modelPos["test_sphere"],
      ignition::math::Vector3d::Zero);
  SpawnCylinder("test_cylinder", modelPos["test_cylinder"],
      ignition::math::Vector3d::Zero);
  SpawnEmptyLink("test_empty", modelPos["test_empty"],
      ignition::math::Vector3d::Zero);

  std::ostringstream linkOffsetStream;
  ignition::math::Pose3d linkOffsetPose1(0, 0, z0, 0, 0, 0);
  ignition::math::Pose3d linkOffsetPose2(1000, 1000, 0, 0, 0, 0);
  ignition::math::Vector3d linkOffsetSize(1, 1, 1);
  linkOffsetStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='link_offset_box'>"
    << "<pose>" << linkOffsetPose1 << "</pose>"
    << "<allow_auto_disable>false</allow_auto_disable>"
    << "<link name ='body'>"
    << "  <pose>" << linkOffsetPose2 << "</pose>"
    << "  <inertial>"
    << "    <mass>4.0</mass>"
    << "    <inertia>"
    << "      <ixx>0.1667</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>"
    << "      <iyy>0.1667</iyy> <iyz>0.0</iyz>"
    << "      <izz>0.1667</izz>"
    << "    </inertia>"
    << "  </inertial>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>" << linkOffsetSize << "</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <box><size>" << linkOffsetSize << "</size></box>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(linkOffsetStream.str());

  /// \TODO: bullet needs this to pass
  if (physics->GetType()  == "bullet")
    physics->SetParam("iters", 300);

  // std::string trimeshPath =
  //    "file://media/models/cube_20k/meshes/cube_20k.stl";
  // SpawnTrimesh("test_trimesh", trimeshPath,
  // ignition::math::Vector3d(0.5, 0.5, 0.5),
  //    modelPos["test_trimesh"], ignition::math::Vector3d::Zero);

  int steps = 2;
  physics::ModelPtr model;
  ignition::math::Pose3d pose1, pose2;
  ignition::math::Vector3d vel1, vel2;

  double t, x0 = 0;
  // This loop steps the world forward and makes sure that each model falls,
  // expecting downward z velocity and decreasing z position.
  for (std::map<std::string, ignition::math::Vector3d>::iterator iter =
      modelPos.begin(); iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->ModelByName(name);
    if (model != NULL)
    {
      gzdbg << "Check freefall of model " << name << '\n';
      // Step once and check downward z velocity
      world->Step(1);
      vel1 = model->WorldLinearVel();
      t = world->SimTime().Double();
      EXPECT_EQ(vel1.X(), 0);
      EXPECT_EQ(vel1.Y(), 0);
      EXPECT_NEAR(vel1.Z(), g.Z()*t, -g.Z()*t*PHYSICS_TOL);
      // Need to step at least twice to check decreasing z position
      world->Step(steps - 1);
      pose1 = model->WorldPose();
      x0 = modelPos[name].X();
      EXPECT_EQ(pose1.Pos().X(), x0);
      EXPECT_EQ(pose1.Pos().Y(), 0);
      EXPECT_NEAR(pose1.Pos().Z(),
          z0 + g.Z()/2*t*t, (z0+g.Z()/2*t*t)*PHYSICS_TOL);
      // Check once more and just make sure they keep falling
      world->Step(steps);
      vel2 = model->WorldLinearVel();
      pose2 = model->WorldPose();
      EXPECT_LT(vel2.Z(), vel1.Z());
      EXPECT_LT(pose2.Pos().Z(), pose1.Pos().Z());

      // if (physics->GetType()  == "bullet")
      // {
      //   gzerr << "m[" << model->GetName()
      //         << "] p[" << model->WorldPose()
      //         << "] v[" << model->GetWorldLinearVel()
      //         << "]\n";

      //   gzerr << "wait: ";
      //   getchar();
      // }
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Predict time of contact with ground plane.
  double tHit = sqrt(2*(z0-0.5) / (-g.Z()));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->SimTime().Double();
  steps = ceil(dtHit / dt);
  EXPECT_GT(steps, 0);

  world->Step(steps);

  // debug
  // for (int i = 0; i < steps; ++i)
  // {
  //   world->Step(1);
  //   if (physics->GetType()  == "bullet")
  //   {
  //     model = world->ModelByName("link_offset_box");
  //     gzerr << "m[" << model->GetName()
  //           << "] i[" << i << "/" << steps
  //           << "] pm[" << model->WorldPose()
  //           << "] pb[" << model->GetLink("body")->WorldPose()
  //           << "] v[" << model->GetWorldLinearVel()
  //           << "]\n";

  //     if (model->WorldPose().Pos().Z() < 0.6)
  //     {
  //       gzerr << "wait: ";
  //       getchar();
  //     }
  //   }
  // }

  // This loop checks the velocity and pose of each model 0.5 seconds
  // after the time of predicted ground contact. The velocity is expected
  // to be small, and the pose is expected to be underneath the initial pose.
  for (std::map<std::string, ignition::math::Vector3d>::iterator iter =
      modelPos.begin(); iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->ModelByName(name);
    if (model != NULL)
    {
      gzdbg << "Check ground contact of model " << name << '\n';
      // Check that velocity is small
      vel1 = model->WorldLinearVel();
      t = world->SimTime().Double();
      EXPECT_NEAR(vel1.X(), 0, PHYSICS_TOL);
      EXPECT_NEAR(vel1.Y(), 0, PHYSICS_TOL);
      if (name == "test_empty")
        EXPECT_NEAR(vel1.Z(), g.Z()*t, -g.Z()*t*PHYSICS_TOL);
      else
        EXPECT_NEAR(vel1.Z(), 0, PHYSICS_TOL);

      // Check that model is resting on ground
      pose1 = model->WorldPose();
      x0 = modelPos[name].X();
      double posTolerance = PHYSICS_TOL;
#ifdef HAVE_BULLET
      if (_physicsEngine == "bullet" && sizeof(btScalar) == 4)
      {
        posTolerance *= 1400;
      }
#endif
      EXPECT_NEAR(pose1.Pos().X(), x0, posTolerance);
      EXPECT_NEAR(pose1.Pos().Y(), 0, posTolerance);

      // debug
      // if (physics->GetType()  == "bullet")
      // {
      //   gzerr << "m[" << model->GetName()
      //         << "] p[" << model->WorldPose()
      //         << "] v[" << model->GetWorldLinearVel()
      //         << "]\n";

      //   gzerr << "wait: ";
      //   getchar();
      // }

      if (name == "test_empty")
      {
        EXPECT_NEAR(pose1.Pos().Z(), z0+g.Z()/2*t*t,
            fabs((z0+g.Z()/2*t*t)*PHYSICS_TOL));
      }
      else
        EXPECT_NEAR(pose1.Pos().Z(), 0.5, PHYSICS_TOL);
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Compute and check link pose of link_offset_box
  gzdbg << "Check link pose of link_offset_box\n";
  model = world->ModelByName("link_offset_box");
  ASSERT_TRUE(model != NULL);
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  // relative pose of link in linkOffsetPose2
  for (int i = 0; i < 20; ++i)
  {
    pose1 = model->WorldPose();
    pose2 = linkOffsetPose2 + pose1;
    EXPECT_NEAR(pose2.Pos().X(), linkOffsetPose2.Pos().X(), PHYSICS_TOL);
    EXPECT_NEAR(pose2.Pos().Y(), linkOffsetPose2.Pos().Y(), PHYSICS_TOL);
    EXPECT_NEAR(pose2.Pos().Z(), 0.5, PHYSICS_TOL);
    world->Step(1);
  }
}

TEST_P(PhysicsTest, SpawnDrop)
{
  SpawnDrop(GetParam());
}

////////////////////////////////////////////////////////////////////////
// SpawnDropCoGOffset:
// Load a world, check that gravity points along z axis, spawn several
// spheres of varying radii and center of gravity (cg) location.
//  sphere1: smaller radius, centered cg
//  sphere2: larger radius, centered cg
//  sphere3: larger radius, lowered cg
//  sphere4: larger radius, raised cg
//  sphere5: larger radius, y offset cg
//  sphere6: larger radius, x offset cg
//  sphere7: larger radius, 45 deg offset cg
//  sphere8: larger radius, -30 deg offset cg
// The bottom of each sphere is at the same height, and it is verified
// that they hit the ground at the same time. Also, sphere5 should start
// rolling to the side when it hits the ground.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SpawnDropCoGOffset(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Skipping SpawnDropCoGOffset for physics engine ["
          << _physicsEngine
          << "] due to issue #1209.\n";
    return;
  }

  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  auto g = world->Gravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.X(), 0);
  EXPECT_EQ(g.Y(), 0);
  EXPECT_LT(g.Z(), 0);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // spawn some spheres and check to see that they start falling
  double z0 = 3;
  double r1 = 0.5, r2 = 1.5;
  ignition::math::Vector3d v30(ignition::math::Vector3d::Zero);
  ignition::math::Vector3d cog;
  ignition::math::Angle angle;

  std::vector<std::string> modelNames;
  std::vector<double> x0s;
  std::vector<double> y0s;
  std::vector<double> radii;
  std::vector<ignition::math::Vector3d> cogs;

  // sphere1 and sphere2 have c.g. at center of sphere, different sizes
  modelNames.push_back("small_centered_sphere");
  x0s.push_back(0);
  y0s.push_back(0);
  radii.push_back(r1);
  cogs.push_back(v30);

  modelNames.push_back("large_centered_sphere");
  x0s.push_back(4);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(v30);

  // sphere3 has c.g. below the center
  modelNames.push_back("lowered_cog_sphere");
  x0s.push_back(8);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(ignition::math::Vector3d(0, 0, -r1));

  // sphere4 has c.g. above the center
  modelNames.push_back("raised_cog_sphere");
  x0s.push_back(-4);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(ignition::math::Vector3d(0, 0, r1));

  // sphere5 has c.g. to the side along y axis; it will roll
  modelNames.push_back("cog_y_offset_sphere");
  x0s.push_back(-8);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(ignition::math::Vector3d(0, r1, 0));

  // sphere6 has c.g. to the side along x axis; it will roll
  modelNames.push_back("cog_x_offset_sphere");
  x0s.push_back(15);
  y0s.push_back(0);
  radii.push_back(r2);
  cogs.push_back(ignition::math::Vector3d(r1, 0, 0));

  // sphere7 has c.g. to the side diagonally; it will roll
  modelNames.push_back("cog_xy_45deg_offset_sphere");
  x0s.push_back(0);
  y0s.push_back(8);
  radii.push_back(r2);
  angle.Degree(45);
  cogs.push_back(ignition::math::Vector3d(r1*cos(angle.Radian()),
        r1*sin(angle.Radian()), 0));

  // sphere8 has c.g. to the side diagonally; it will roll
  modelNames.push_back("cog_xy_-30deg_offset_sphere");
  x0s.push_back(0);
  y0s.push_back(-8);
  radii.push_back(r2);
  angle.Degree(-30);
  cogs.push_back(ignition::math::Vector3d(r1*cos(angle.Radian()),
        r1*sin(angle.Radian()), 0));

  unsigned int i;
  for (i = 0; i < modelNames.size(); ++i)
  {
    SpawnSphere(modelNames[i],
                ignition::math::Vector3d(x0s[i], y0s[i], z0+radii[i]),
                v30, cogs[i], radii[i]);
  }

  int steps = 2;
  physics::ModelPtr model;
  ignition::math::Pose3d pose1, pose2;
  ignition::math::Vector3d vel1, vel2;

  double t, x0 = 0, y0 = 0, radius;
  // This loop steps the world forward and makes sure that each model falls,
  // expecting downward z velocity and decreasing z position.
  for (i = 0; i < modelNames.size(); ++i)
  {
    // Make sure the model is loaded
    model = world->ModelByName(modelNames[i]);
    x0 = x0s[i];
    y0 = y0s[i];
    radius = radii[i];
    if (model != NULL)
    {
      gzdbg << "Check freefall of model " << modelNames[i] << '\n';
      // Step once and check downward z velocity
      world->Step(1);
      vel1 = model->WorldLinearVel();
      t = world->SimTime().Double();
      EXPECT_NEAR(vel1.X(), 0, 1e-16);
      EXPECT_NEAR(vel1.Y(), 0, 1e-16);
      EXPECT_NEAR(vel1.Z(), g.Z()*t, -g.Z()*t*PHYSICS_TOL);
      // Need to step at least twice to check decreasing z position
      world->Step(steps - 1);
      pose1 = model->WorldPose();
      EXPECT_NEAR(pose1.Pos().X(), x0, PHYSICS_TOL*PHYSICS_TOL);
      EXPECT_NEAR(pose1.Pos().Y(), y0, PHYSICS_TOL*PHYSICS_TOL);
      EXPECT_NEAR(pose1.Pos().Z(), z0+radius + g.Z()/2*t*t,
                  (z0+radius+g.Z()/2*t*t)*PHYSICS_TOL);

      // Check once more and just make sure they keep falling
      world->Step(steps);
      vel2 = model->WorldLinearVel();
      pose2 = model->WorldPose();
      EXPECT_LT(vel2.Z(), vel1.Z());
      EXPECT_LT(pose2.Pos().Z(), pose1.Pos().Z());
    }
    else
    {
      gzerr << "Error loading model " << modelNames[i] << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }

  // Predict time of contact with ground plane.
  double tHit = sqrt(2*(z0-0.5) / (-g.Z()));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->SimTime().Double();
  steps = ceil(dtHit / dt);
  EXPECT_GT(steps, 0);
  world->Step(steps);

  // This loop checks the velocity and pose of each model 0.5 seconds
  // after the time of predicted ground contact. Except for sphere5,
  // the velocity is expected to be small, and the pose is expected
  // to be underneath the initial pose. sphere5 is expected to be rolling.
  for (i = 0; i < modelNames.size(); ++i)
  {
    // Make sure the model is loaded
    model = world->ModelByName(modelNames[i]);
    x0 = x0s[i];
    y0 = y0s[i];
    radius = radii[i];
    cog = cogs[i];
    if (model != NULL)
    {
      gzdbg << "Check ground contact and roll without slip of model "
            << modelNames[i] << '\n';

      // Check that velocity is small
      vel1 = model->WorldLinearVel();
      vel2 = model->WorldAngularVel();

      // vertical component of linear and angular velocity should be small
      EXPECT_NEAR(vel1.Z(), 0, PHYSICS_TOL);
      EXPECT_NEAR(vel2.Z(), 0, PHYSICS_TOL);

      // expect small values for directions with no offset
      if (cog.X() == 0)
      {
        EXPECT_NEAR(vel1.X(), 0, PHYSICS_TOL);
        EXPECT_NEAR(vel2.Y(), 0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT(vel1.X()*cog.X(), 0.2*cog.X()*cog.X());
        EXPECT_GT(vel2.Y()*cog.X(), 0.2*cog.X()*cog.X());
      }

      if (cog.Y() == 0)
      {
        EXPECT_NEAR(vel1.Y(), 0, PHYSICS_TOL);
        EXPECT_NEAR(vel2.X(), 0, PHYSICS_TOL);
      }
      else
      {
        EXPECT_GT(vel1.Y()*cog.Y(),  0.2*cog.Y()*cog.Y());
        EXPECT_LT(vel2.X()*cog.Y(), -0.2*cog.Y()*cog.Y());
      }

      // Expect roll without slip
      EXPECT_NEAR(vel1.X(),  vel2.Y()*radius, PHYSICS_TOL);
      EXPECT_NEAR(vel1.Y(), -vel2.X()*radius, PHYSICS_TOL);

      // Use GetWorldLinearVel with global offset to check roll without slip
      // Expect small linear velocity at contact point
      ignition::math::Vector3d vel3 = model->GetLink()->WorldLinearVel(
          ignition::math::Vector3d(0, 0, -radius),
          ignition::math::Quaterniond::Identity);
      EXPECT_NEAR(vel3.X(), 0, PHYSICS_TOL);
      EXPECT_NEAR(vel3.Y(), 0, PHYSICS_TOL);
      EXPECT_NEAR(vel3.Z(), 0, PHYSICS_TOL);
      // Expect speed at top of sphere to be double the speed at center
      ignition::math::Vector3d vel4 = model->GetLink()->WorldLinearVel(
          ignition::math::Vector3d(0, 0, radius),
          ignition::math::Quaterniond::Identity);
      EXPECT_NEAR(vel4.Y(), 2*vel1.Y(), PHYSICS_TOL);
      EXPECT_NEAR(vel4.X(), 2*vel1.X(), PHYSICS_TOL);
      EXPECT_NEAR(vel4.Z(), 0, PHYSICS_TOL);

      // Check that model is resting on ground
      pose1 = model->WorldPose();
      EXPECT_NEAR(pose1.Pos().Z(), radius, PHYSICS_TOL);

      // expect no pose change for directions with no offset
      if (cog.X() == 0)
      {
        EXPECT_NEAR(pose1.Pos().X(), x0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT((pose1.Pos().X()-x0) * cog.X(), cog.X() * cog.X());
      }

      // expect no pose change for directions with no offset
      if (cog.Y() == 0)
      {
        EXPECT_NEAR(pose1.Pos().Y(), y0, PHYSICS_TOL);
      }
      // expect rolling in direction of cog offset
      else
      {
        EXPECT_GT((pose1.Pos().Y()-y0) * cog.Y(), cog.Y() * cog.Y());
      }
    }
    else
    {
      gzerr << "Error loading model " << modelNames[i] << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}

TEST_P(PhysicsTest, SpawnDropCoGOffset)
{
  SpawnDropCoGOffset(GetParam());
}

//////////////////////////////////////////////////
TEST_F(PhysicsTest, EmptyStates)
{
  this->Load("worlds/empty.world");
  auto world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // Current world state
  physics::WorldState worldState(world);

  auto modelStates = worldState.GetModelStates();
  EXPECT_EQ(1u, worldState.GetModelStateCount());
  EXPECT_EQ(1u, modelStates.size());

  // Model state
  auto modelState = modelStates["box"];
  EXPECT_EQ(ignition::math::Pose3d::Zero, modelState.Pose());
}

//////////////////////////////////////////////////
TEST_F(PhysicsTest, StateChange)
{
  this->Load("worlds/shapes.world");
  auto world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  // Check initial world state
  physics::WorldState oldWorldState(world);
  EXPECT_EQ(4u, oldWorldState.GetModelStateCount());

  auto oldModelState = oldWorldState.GetModelState("box");
  ignition::math::Pose3d oldPose(0, 0, 0.5, 0, 0, 0);
  EXPECT_EQ(oldPose, oldModelState.Pose());

  // Move the box
  world->SetPaused(true);

  ignition::math::Pose3d newPose(1, 2, 0.5, 0, 0, 0);
  world->ModelByName("box")->SetWorldPose(newPose);

  world->Step(1);

  // Make sure the box state reflects the move
  physics::WorldState newWorldState(world);
  EXPECT_EQ(4u, newWorldState.GetModelStateCount());

  auto newModelState = newWorldState.GetModelState("box");
  EXPECT_EQ(newPose, newModelState.Pose());

  // Reset world state, and check for correctness
  world->SetState(oldWorldState);

  physics::WorldState resetWorldState(world);
  EXPECT_EQ(4u, resetWorldState.GetModelStateCount());

  auto resetModelState = resetWorldState.GetModelState("box");
  EXPECT_EQ(oldPose, resetModelState.Pose());
}

//////////////////////////////////////////////////
TEST_F(PhysicsTest, StateInsertion)
{
  this->Load("worlds/empty.world");
  auto world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  std::string newModelName("new_model");
  std::string newLightName("new_light");

  // Check initial count
  EXPECT_EQ(1u, world->ModelCount());
  EXPECT_EQ(1u, world->LightCount());
  EXPECT_TRUE(world->ModelByName(newModelName) == NULL);
  EXPECT_TRUE(world->LightByName(newLightName) == NULL);

  // Current world state
  physics::WorldState worldState(world);
  EXPECT_EQ(1u, worldState.GetModelStateCount());
  EXPECT_EQ(1u, worldState.LightStateCount());

  // Insertions
  std::stringstream newModelStr;
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << newModelName << "'>"
              << "<link name ='link'>"
              << "  <collision name ='collision'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </collision>"
              << "  <visual name ='visual'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </visual>"
              << "</link>"
              << "</model>"
              << "</sdf>";

  std::stringstream newLightStr;
  newLightStr << "<sdf version='" << SDF_VERSION << "'>"
              << "<light name ='" << newLightName << "' type='spot'>"
              << "<diffuse>0 1 0 1</diffuse>"
              << "</light>"
              << "</sdf>";

  std::vector<std::string> insertions;
  insertions.push_back(newModelStr.str());
  insertions.push_back(newLightStr.str());

  worldState.SetInsertions(insertions);

  // Set state which includes insertion
  world->SetPaused(true);
  world->SetState(worldState);

  world->Step(1);

  // Check entities were inserted
  EXPECT_EQ(2u, world->ModelCount());
  EXPECT_EQ(2u, world->LightCount());
  EXPECT_FALSE(world->ModelByName(newModelName) == NULL);
  EXPECT_FALSE(world->LightByName(newLightName) == NULL);

  // New world state
  physics::WorldState newWorldState(world);
  EXPECT_EQ(2u, newWorldState.GetModelStateCount());
  EXPECT_EQ(2u, newWorldState.LightStateCount());
}

//////////////////////////////////////////////////
TEST_F(PhysicsTest, StateDeletion)
{
  this->Load("worlds/shapes.world");
  auto world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  std::string deleteModelName("box");
  std::string deleteLightName("sun");

  // Check initial count
  EXPECT_EQ(4u, world->ModelCount());
  EXPECT_EQ(1u, world->LightCount());
  EXPECT_FALSE(world->ModelByName(deleteModelName) == NULL);
  EXPECT_FALSE(world->LightByName(deleteLightName) == NULL);

  // Current world state
  physics::WorldState worldState(world);
  EXPECT_EQ(4u, worldState.GetModelStateCount());
  EXPECT_EQ(1u, worldState.LightStateCount());

  // Deletions
  std::vector<std::string> deletions;
  deletions.push_back(deleteModelName);
  deletions.push_back(deleteLightName);

  worldState.SetDeletions(deletions);

  // Set state which includes deletion
  world->SetState(worldState);

  gazebo::common::Time::MSleep(10);

  // Check entities were deleted
  EXPECT_EQ(3u, world->ModelCount());
  EXPECT_EQ(0u, world->LightCount());
  EXPECT_TRUE(world->ModelByName(deleteModelName) == NULL);
  EXPECT_TRUE(world->LightByName(deleteLightName) == NULL);

  // New world state
  physics::WorldState newWorldState(world);
  EXPECT_EQ(3u, newWorldState.GetModelStateCount());
  EXPECT_EQ(0u, newWorldState.LightStateCount());
}

////////////////////////////////////////////////////////////////////////
void PhysicsTest::JointDampingTest(const std::string &_physicsEngine)
{
  // Random seed is set to prevent brittle failures (gazebo issue #479)
  ignition::math::Rand::Seed(18420503);
  Load("worlds/damp_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("model_4_mass_1_ixx_1_damping_10") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get model_4_mass_1_ixx_1_damping_10");

  physics::ModelPtr model = world->ModelByName(
      "model_4_mass_1_ixx_1_damping_10");
  EXPECT_TRUE(model != NULL);

  {
    // compare against recorded data only
    double test_duration = 1.5;
    double dt = world->Physics()->GetMaxStepSize();
    int steps = test_duration/dt;

    for (int i = 0; i < steps; ++i)
    {
      world->Step(1);  // theoretical contact, but
      // gzdbg << "box time [" << world->SimTime().Double()
      //       << "] vel [" << model->GetWorldLinearVel()
      //       << "] pose [" << model->WorldPose()
      //       << "]\n";
    }

    EXPECT_EQ(world->SimTime().Double(), 1.5);

    // This test expects a linear velocity at the CoG
    ignition::math::Vector3d vel =
      model->GetLink()->WorldCoGLinearVel();
    ignition::math::Pose3d pose = model->WorldPose();

    EXPECT_EQ(vel.X(), 0.0);

    if (_physicsEngine == "dart")
    {
      // DART needs greater tolerance. The reason is not sure yet.
      // Please see issue #904
      EXPECT_NEAR(vel.Y(), -10.2009, 0.012);
      EXPECT_NEAR(vel.Z(), -6.51755, 0.012);
    }
    else
    {
      EXPECT_NEAR(vel.Y(), -10.2009, PHYSICS_TOL);
      EXPECT_NEAR(vel.Z(), -6.51755, PHYSICS_TOL);
    }

    EXPECT_DOUBLE_EQ(pose.Pos().X(), 3.0);
    EXPECT_NEAR(pose.Pos().Y(), 0.0, PHYSICS_TOL);
    EXPECT_NEAR(pose.Pos().Z(), 10.099, PHYSICS_TOL);
    EXPECT_NEAR(pose.Rot().Euler().X(), 0.567334, PHYSICS_TOL);
    EXPECT_DOUBLE_EQ(pose.Rot().Euler().Y(), 0.0);
    EXPECT_DOUBLE_EQ(pose.Rot().Euler().Z(), 0.0);
  }
}

TEST_P(PhysicsTest, JointDampingTest)
{
  JointDampingTest(GetParam());
}

////////////////////////////////////////////////////////////////////////
void PhysicsTest::DropStuff(const std::string &_physicsEngine)
{
  Load("worlds/drop_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("cylinder") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get cylinder");

  {
    // todo: get parameters from drop_test.world
    double test_duration = 1.5;
    double z = 10.5;
    double v = 0.0;
    double g = -10.0;
    double dt = world->Physics()->GetMaxStepSize();

    // world->Step(1428);  // theoretical contact, but
    // world->Step(100);  // integration error requires few more steps

    int steps = test_duration/dt;
    bool postContactCorrection = false;

    for (int i = 0; i < steps; ++i)
    {
      // integrate here to see when the collision should happen
      v = v + dt * g;
      z = z + dt * v;

      // theoretical contact, but
      world->Step(1);
      {
        physics::ModelPtr boxModel = world->ModelByName("box");
        if (boxModel)
        {
          ignition::math::Vector3d vel = boxModel->WorldLinearVel();
          ignition::math::Pose3d pose = boxModel->WorldPose();
          // gzdbg << "box time [" << world->SimTime().Double()
          //      << "] sim z [" << pose.Pos().Z()
          //      << "] exact z [" << z
          //      << "] sim vz [" << vel.Z()
          //      << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !postContactCorrection)
          {
            EXPECT_LT(fabs(vel.Z() - v) , 0.0001);
            EXPECT_LT(fabs(pose.Pos().Z() - z) , 0.0001);
          }
          else
          {
            // sometimes -0.01, why?
            EXPECT_LT(fabs(vel.Z()), 0.0101);
            if (_physicsEngine == "dart")
            {
              // DART needs more tolerance until supports 'correction for
              // penetration' feature.
              // Please see issue #902
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.00410);
            }
            else
            {
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.00001);
            }
          }
        }

        physics::ModelPtr sphereModel = world->ModelByName("sphere");
        if (sphereModel)
        {
          ignition::math::Vector3d vel = sphereModel->WorldLinearVel();
          ignition::math::Pose3d pose = sphereModel->WorldPose();
          // gzdbg << "sphere time [" << world->SimTime().Double()
          //       << "] sim z [" << pose.Pos().Z()
          //       << "] exact z [" << z
          //       << "] sim vz [" << vel.Z()
          //       << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !postContactCorrection)
          {
            EXPECT_LT(fabs(vel.Z() - v), 0.0001);
            EXPECT_LT(fabs(pose.Pos().Z() - z), 0.0001);
          }
          else
          {
            if (_physicsEngine == "dart")
            {
              // DART needs more tolerance until supports 'correction for
              // penetration' feature.
              // Please see issue #902
              EXPECT_LT(fabs(vel.Z()), 0.015);
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.00410);
            }
            else
            {
              EXPECT_LT(fabs(vel.Z()), 3e-5);
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.00001);
            }
          }
        }

        physics::ModelPtr cylinderModel = world->ModelByName("cylinder");
        if (cylinderModel)
        {
          ignition::math::Vector3d vel = cylinderModel->WorldLinearVel();
          ignition::math::Pose3d pose = cylinderModel->WorldPose();
          // gzdbg << "cylinder time [" << world->SimTime().Double()
          //       << "] sim z [" << pose.Pos().Z()
          //       << "] exact z [" << z
          //       << "] sim vz [" << vel.Z()
          //       << "] exact vz [" << v << "]\n";
          if (z > 0.5 || !postContactCorrection)
          {
            EXPECT_LT(fabs(vel.Z() - v), 0.0001);
            EXPECT_LT(fabs(pose.Pos().Z() - z), 0.0001);
          }
          else
          {
            EXPECT_LT(fabs(vel.Z()), 0.011);
            if (_physicsEngine == "dart")
            {
              // DART needs more tolerance until supports 'correction for
              // penetration' feature.
              // Please see issue #902
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.0041);
            }
            else
            {
              EXPECT_LT(fabs(pose.Pos().Z() - 0.5), 0.0001);
            }
          }
        }
      }
      if (z < 0.5) postContactCorrection = true;
    }
  }
}

// This test doesn't pass yet in Bullet or Simbody
TEST_F(PhysicsTest, DropStuffODE)
{
  DropStuff("ode");
}

#ifdef HAVE_DART
TEST_F(PhysicsTest, DropStuffDART)
{
  DropStuff("dart");
}
#endif

////////////////////////////////////////////////////////////////////////
void PhysicsTest::InelasticCollision(const std::string &_physicsEngine)
{
  // check conservation of mementum for linear inelastic collision
  Load("worlds/collision_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  int i = 0;
  while (!this->HasEntity("sphere") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get sphere");

  {
    // todo: get parameters from drop_test.world
    double test_duration = 1.1;
    double dt = world->Physics()->GetMaxStepSize();

    physics::ModelPtr boxModel = world->ModelByName("box");
    physics::LinkPtr boxLink = boxModel->GetLink("link");
    double f = 1000.0;
    double v = 0;
    double x = 0;
    double m = boxLink->GetInertial()->Mass();

    int steps = test_duration/dt;

    for (int i = 0; i < steps; ++i)
    {
      double t = world->SimTime().Double();
      double velTolerance = PHYSICS_TOL;
#ifdef HAVE_BULLET
      if (_physicsEngine == "bullet" && sizeof(btScalar) == 4)
      {
        velTolerance *= 11;
      }
#endif

      // theoretical contact, but
      world->Step(1);

      if (boxModel)
      {
        ignition::math::Vector3d vel = boxModel->WorldLinearVel();
        ignition::math::Pose3d pose = boxModel->WorldPose();

        // gzdbg << "box time [" << t
        //      << "] sim x [" << pose.Pos().X()
        //      << "] ideal x [" << x
        //      << "] sim vx [" << vel.X()
        //      << "] ideal vx [" << v
        //      << "]\n";

        if (i == 0)
        {
          boxModel->GetLink("link")->SetForce(
              ignition::math::Vector3d(f, 0, 0));
          // The following has been failing since pull request #1284,
          // so it has been disabled.
          // See bitbucket.org/osrf/gazebo/issue/1394
          // EXPECT_EQ(boxModel->GetLink("link")->GetWorldForce(),
          //  ignition:: ignition::math::Vector3d(f, 0, 0));
        }

        if (t > 1.000 && t < 1.01)
        {
          // collision transition, do nothing
        }
        else
        {
          // collision happened
          EXPECT_NEAR(pose.Pos().X(), x, PHYSICS_TOL);
          EXPECT_NEAR(vel.X(), v, velTolerance);
        }
      }

      physics::ModelPtr sphereModel = world->ModelByName("sphere");
      if (sphereModel)
      {
        ignition::math::Vector3d vel = sphereModel->WorldLinearVel();
        ignition::math::Pose3d pose = sphereModel->WorldPose();
        // gzdbg << "sphere time [" << world->SimTime().Double()
        //      << "] sim x [" << pose.Pos().X()
        //      << "] ideal x [" << x
        //      << "] sim vx [" << vel.X()
        //      << "] ideal vx [" << v
        //      << "]\n";
        if (t > 1.000 && t < 1.01)
        {
          // collision transition, do nothing
        }
        else if (t <= 1.00)
        {
          // no collision
          EXPECT_EQ(pose.Pos().X(), 2);
          EXPECT_EQ(vel.X(), 0);
        }
        else
        {
          // collision happened
          EXPECT_NEAR(pose.Pos().X(), x + 1.0, PHYSICS_TOL);
          EXPECT_NEAR(vel.X(), v, velTolerance);
        }
      }

      // integrate here to see when the collision should happen
      // double impulse = dt*f;
      // if (i == 0)
      //    v = v + impulse;
      // inelastic col. w/ eqal mass.
      // else if (t >= 1.0)
      //   v = dt*f/ 2.0;
      // x = x + dt * v;

      // integrate here to see when the collision should happen
      double vold = v;
      if (i == 0)
      {
        v = vold + dt* (f / m);
      }
      else if (t >= 1.0)
      {
        // inelastic col. w/ eqal mass.
        v = dt*f/ 2.0;
      }
      x = x + dt * (v + vold) / 2.0;
    }
  }
}

TEST_P(PhysicsTest, InelasticCollision)
{
  InelasticCollision(GetParam());
}

////////////////////////////////////////////////////////////////////////
// SphereAtlasLargeError:
// Check algorithm's ability to re-converge after a large LCP error is
// introduced.
// In this test, a model with similar dynamics properties to Atlas V3
// is pinned to the world by both feet.  Robot is moved by a large
// distance, violating the joints between world and feet temporarily.
// Robot is then allowed to settle.  Check to see that the LCP solution
// does not become unstable.
////////////////////////////////////////////////////////////////////////
void PhysicsTest::SphereAtlasLargeError(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Skipping SphereAtlasLargeError for physics engine ["
          << _physicsEngine
          << "] as this test only works for ODE for now.\n";
    return;
  }

  Load("worlds/sphere_atlas_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(ignition::math::Vector3d::Zero);

  int i = 0;
  while (!this->HasEntity("sphere_atlas") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get sphere_atlas");

  physics::ModelPtr model = world->ModelByName("sphere_atlas");
  EXPECT_TRUE(model != NULL);
  physics::LinkPtr head = model->GetLink("head");
  EXPECT_TRUE(head != NULL);

  {
    gzdbg << "Testing large perturbation with PID controller active.\n";
    // Test:  With Robot PID controller active, introduce a large
    //        constraint error by breaking some model joints to the world
    model->SetWorldPose(ignition::math::Pose3d(1000, 0, 0, 0, 0, 0));

    // let model settle
    world->Step(2000);

    for (unsigned int n = 0; n < 10; ++n)
    {
      world->Step(1);
      // manually check joint constraint violation for each joint
      physics::Link_V links = model->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        ignition::math::Pose3d childInWorld = links[i]->WorldPose();

        physics::Joint_V parentJoints = links[i]->GetParentJoints();
        for (unsigned int j = 0; j < parentJoints.size(); ++j)
        {
          // anchor position in world frame
          ignition::math::Vector3d anchorPos =
            parentJoints[j]->Anchor(0);

          // anchor pose in child link frame
          ignition::math::Pose3d anchorInChild =
            ignition::math::Pose3d(anchorPos,
                ignition::math::Quaterniond::Identity) - childInWorld;

          // initial anchor pose in child link frame
          ignition::math::Pose3d anchorInitialInChild =
            parentJoints[j]->InitialAnchorPose();

          physics::LinkPtr parent = parentJoints[j]->GetParent();
          if (parent)
          {
            // compare everything in the parent frame
            // rel to model - rel to model
            ignition::math::Pose3d childInitialInParent =
              links[i]->InitialRelativePose() - parent->InitialRelativePose();

            ignition::math::Pose3d parentInWorld = parent->WorldPose();

            ignition::math::Pose3d childInParent = childInWorld -
              parentInWorld;

            ignition::math::Pose3d anchorInParent = anchorInChild +
              childInParent;

            ignition::math::Pose3d anchorInitialInParent =
              anchorInitialInChild + childInitialInParent;

            ignition::math::Pose3d jointError = anchorInParent -
              anchorInitialInParent;

            // joint constraint violation must be less than...
            EXPECT_LT(jointError.Pos().SquaredLength(), PHYSICS_TOL);

            // debug
            if (jointError.Pos().SquaredLength() >= PHYSICS_TOL)
              gzdbg << "i [" << n
                    << "] link [" << links[i]->GetName()
                    // << "] parent[" << parent->GetName()
                    << "] error[" << jointError.Pos().SquaredLength()
                    // << "] pose[" << childInWorld
                    << "] anchor[" << anchorInChild
                    << "] cinp[" << childInParent
                    << "] ainp0[" << anchorInitialInParent
                    << "] ainp[" << anchorInParent
                    << "] diff[" << jointError
                    << "]\n";
          }
        }
      }
    }
  }

  {
    gzdbg << "Testing large perturbation with PID controller disabled.\n";
    // Test:  Turn off Robot PID controller, then introduce a large
    //        constraint error by breaking some model joints to the world

    // special hook in SphereAtlasTestPlugin disconnects
    // PID controller on Reset.
    world->Reset();
    world->Step(1);

    model->SetWorldPose(ignition::math::Pose3d(1000, 0, 0, 0, 0, 0));

    // let model settle
    world->Step(2000);

    for (unsigned int n = 0; n < 10; ++n)
    {
      world->Step(1);
      // manually check joint constraint violation for each joint
      physics::Link_V links = model->GetLinks();
      for (unsigned int i = 0; i < links.size(); ++i)
      {
        ignition::math::Pose3d childInWorld = links[i]->WorldPose();

        physics::Joint_V parentJoints = links[i]->GetParentJoints();
        for (unsigned int j = 0; j < parentJoints.size(); ++j)
        {
          // anchor position in world frame
          ignition::math::Vector3d anchorPos =
            parentJoints[j]->Anchor(0);

          // anchor pose in child link frame
          ignition::math::Pose3d anchorInChild =
            ignition::math::Pose3d(anchorPos,
                ignition::math::Quaterniond::Identity) - childInWorld;

          // initial anchor pose in child link frame
          ignition::math::Pose3d anchorInitialInChild =
            parentJoints[j]->InitialAnchorPose();

          physics::LinkPtr parent = parentJoints[j]->GetParent();
          if (parent)
          {
            // compare everything in the parent frame
            // rel to model - rel to model
            ignition::math::Pose3d childInitialInParent =
              links[i]->InitialRelativePose() - parent->InitialRelativePose();

            ignition::math::Pose3d parentInWorld = parent->WorldPose();
            ignition::math::Pose3d childInParent = childInWorld - parentInWorld;
            ignition::math::Pose3d anchorInParent =
              anchorInChild + childInParent;
            ignition::math::Pose3d anchorInitialInParent =
              anchorInitialInChild + childInitialInParent;
            ignition::math::Pose3d jointError = anchorInParent -
              anchorInitialInParent;

            // joint constraint violation must be less than...
            EXPECT_LT(jointError.Pos().SquaredLength(), PHYSICS_TOL);

            // debug
            if (jointError.Pos().SquaredLength() >= PHYSICS_TOL)
              gzdbg << "i [" << n
                    << "] link [" << links[i]->GetName()
                    // << "] parent[" << parent->GetName()
                    << "] error[" << jointError.Pos().SquaredLength()
                    // << "] pose[" << childInWorld
                    << "] anchor[" << anchorInChild
                    << "] cinp[" << childInParent
                    << "] ainp0[" << anchorInitialInParent
                    << "] ainp[" << anchorInParent
                    << "] diff[" << jointError
                    << "]\n";
          }
        }
      }
    }
  }
}

TEST_P(PhysicsTest, SphereAtlasLargeError)
{
  SphereAtlasLargeError(GetParam());
}

////////////////////////////////////////////////////////////////////////
// CollisionFiltering:
// Load a world, spawn a model with two overlapping links. By default,
// the links should not collide with each other as they have the same
// parent model. Check the x and y velocities to see if they are 0
////////////////////////////////////////////////////////////////////////
void PhysicsTest::CollisionFiltering(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  std::stringstream newModelStr;

  std::string modelName = "multiLinkModel";
  ignition::math::Pose3d modelPose(0, 0, 2, 0, 0, 0);
  ignition::math::Pose3d link01Pose(0, 0.1, 0, 0, 0, 0);
  ignition::math::Pose3d link02Pose(0, -0.1, 0, 0, 0, 0);

  // A model composed of two overlapping links at fixed y offset from origin
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << modelName << "'>"
              << "<pose>" << modelPose.Pos().X() << " "
                         << modelPose.Pos().Y() << " "
                         << modelPose.Pos().Z() << " "
                         << modelPose.Rot().Euler().X() << " "
                         << modelPose.Rot().Euler().Y() << " "
                         << modelPose.Rot().Euler().Z() << "</pose>"
              << "<link name ='link01'>"
              << "  <pose>" << link01Pose.Pos().X() << " "
                         << link01Pose.Pos().Y() << " "
                         << link01Pose.Pos().Z() << " "
                         << link01Pose.Rot().Euler().X() << " "
                         << link01Pose.Rot().Euler().Y() << " "
                         << link01Pose.Rot().Euler().Z() << "</pose>"
              << "  <collision name ='geom'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </collision>"
              << "  <visual name ='visual'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </visual>"
              << "</link>"
              << "<link name ='link02'>"
              << "  <pose>" << link02Pose.Pos().X() << " "
                         << link02Pose.Pos().Y() << " "
                         << link02Pose.Pos().Z() << " "
                         << link02Pose.Rot().Euler().X() << " "
                         << link02Pose.Rot().Euler().Y() << " "
                         << link02Pose.Rot().Euler().Z() << "</pose>"
              << "  <collision name ='geom'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </collision>"
              << "  <visual name ='visual'>"
              << "    <geometry>"
              << "      <box><size>1 1 1</size></box>"
              << "    </geometry>"
              << "  </visual>"
              << "</link>"
              << "</model>"
              << "</sdf>";

  SpawnSDF(newModelStr.str());

  // Wait for the entity to spawn
  int i = 0;
  while (!this->HasEntity(modelName) && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }
  if (i > 20)
    gzthrow("Unable to spawn model");

  world->Step(5);
  physics::ModelPtr model = world->ModelByName(modelName);

  ignition::math::Vector3d vel;

  physics::Link_V links = model->GetLinks();
  EXPECT_EQ(links.size(), 2u);
  for (physics::Link_V::const_iterator iter = links.begin();
      iter != links.end(); ++iter)
  {
    std::cout << "LinkName[" << (*iter)->GetScopedName() << "]\n";
    // Links should not repel each other hence expecting zero x, y vel
    vel = (*iter)->WorldLinearVel();
    EXPECT_EQ(vel.X(), 0);
    EXPECT_EQ(vel.Y(), 0);

    // Model should be falling
    EXPECT_LT(vel.Z(), 0);
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsTest, CollisionFiltering)
{
  CollisionFiltering(GetParam());
}

/////////////////////////////////////////////////
// This test verifies that gazebo doesn't crash when collisions occur
// and the <world><physics><ode><max_contacts> value is zero.
// The crash was reported in issue #593 on bitbucket
TEST_F(PhysicsTest, ZeroMaxContactsODE)
{
  // Load an empty world
  Load("worlds/zero_max_contacts.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("ground_plane");
  ASSERT_TRUE(model != NULL);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
