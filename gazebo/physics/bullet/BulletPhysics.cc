/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: The Bullet physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#include "physics/bullet/BulletTypes.hh"
#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletCollision.hh"

#include "physics/bullet/BulletPlaneShape.hh"
#include "physics/bullet/BulletSphereShape.hh"
#include "physics/bullet/BulletHeightmapShape.hh"
#include "physics/bullet/BulletMultiRayShape.hh"
#include "physics/bullet/BulletBoxShape.hh"
#include "physics/bullet/BulletCylinderShape.hh"
#include "physics/bullet/BulletTrimeshShape.hh"
#include "physics/bullet/BulletRayShape.hh"

#include "physics/bullet/BulletHingeJoint.hh"
#include "physics/bullet/BulletUniversalJoint.hh"
#include "physics/bullet/BulletBallJoint.hh"
#include "physics/bullet/BulletSliderJoint.hh"
#include "physics/bullet/BulletHinge2Joint.hh"
#include "physics/bullet/BulletScrewJoint.hh"

#include "transport/Publisher.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Collision.hh"
#include "physics/MapShape.hh"

#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"

#include "BulletPhysics.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("bullet", BulletPhysics)

extern ContactAddedCallback gContactAddedCallback;
extern ContactProcessedCallback gContactProcessedCallback;

//////////////////////////////////////////////////
bool ContactCallback(btManifoldPoint &/*_cp*/,
    const btCollisionObjectWrapper * /*_obj0*/, int /*_partId0*/,
    int /*_index0*/, const btCollisionObjectWrapper * /*_obj1*/,
    int /*_partId1*/, int /*_index1*/)
{
  return true;
}

//////////////////////////////////////////////////
bool ContactProcessed(btManifoldPoint &/*_cp*/, void * /*_body0*/,
                      void * /*_body1*/)
{
  return true;
}

//////////////////////////////////////////////////
BulletPhysics::BulletPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  // This function currently follows the pattern of bullet/Demos/HelloWorld

  // Default setup for memory and collisions
  this->collisionConfig = new btDefaultCollisionConfiguration();

  // Default collision dispatcher, a multi-threaded dispatcher may be available
  this->dispatcher = new btCollisionDispatcher(this->collisionConfig);

  // Broadphase collision detection uses axis-aligned bounding boxes (AABB)
  // to detect pairs of objects that may be in contact.
  // The narrow-phase collision detection evaluates each pair generated by the
  // broadphase.
  // "btDbvtBroadphase uses a fast dynamic bounding volume hierarchy based on
  // AABB tree" according to Bullet_User_Manual.pdf
  // "btAxis3Sweep and bt32BitAxisSweep3 implement incremental 3d sweep and
  // prune" also according to the user manual.
  // btCudaBroadphase can be used if GPU hardware is available
  // Here we are using btDbvtBroadphase.
  this->broadPhase = new btDbvtBroadphase();

  // Create btSequentialImpulseConstraintSolver, the default constraint solver.
  // Note that a multi-threaded solver may be available.
  this->solver = new btSequentialImpulseConstraintSolver;

  // Create a btDiscreteDynamicsWorld, which is used for discrete rigid bodies.
  // An alternative is btSoftRigidDynamicsWorld, which handles both soft and
  // rigid bodies.
  this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher,
      this->broadPhase, this->solver, this->collisionConfig);

  // TODO: Enable this to do custom contact setting
  gContactAddedCallback = ContactCallback;
  gContactProcessedCallback = ContactProcessed;
}

//////////////////////////////////////////////////
BulletPhysics::~BulletPhysics()
{
  // Delete in reverse-order of creation
  delete this->dynamicsWorld;
  delete this->solver;
  delete this->broadPhase;
  delete this->dispatcher;
  delete this->collisionConfig;

  this->dynamicsWorld = NULL;
  this->solver = NULL;
  this->broadPhase = NULL;
  this->dispatcher = NULL;
  this->collisionConfig = NULL;
}

//////////////////////////////////////////////////
void BulletPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr bulletElem = this->sdf->GetElement("bullet");

  math::Vector3 g = this->sdf->GetValueVector3("gravity");
  // ODEPhysics checks this, so we will too.
  if (g == math::Vector3(0, 0, 0))
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->dynamicsWorld->setGravity(btVector3(g.x, g.y, g.z));

  btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();

  // Split impulse feature. This reduces large bounces from deep penetrations,
  // but can lead to improper stacking of objects, see
  // http://bulletphysics.org/mediawiki-1.5.8/index.php/BtContactSolverInfo ...
  // ... #Split_Impulse
  info.m_splitImpulse = 1;
  info.m_splitImpulsePenetrationThreshold = -0.02;

  if (bulletElem->HasElement("constraints"))
  {
    // the following are undocumented members of btContactSolverInfo, see
    // bulletphysics.org/mediawiki-1.5.8/index.php/BtContactSolverInfo ...
    // ... #Undocumented_members_of_btContactSolverInfo
    // m_globalCfm: constraint force mixing
    info.m_globalCfm =
      bulletElem->GetElement("constraints")->GetValueDouble("cfm");
    // m_erp: Baumgarte factor
    info.m_erp = bulletElem->GetElement("constraints")->GetValueDouble("erp");
  }
}

//////////////////////////////////////////////////
void BulletPhysics::Init()
{
}

//////////////////////////////////////////////////
void BulletPhysics::InitForThread()
{
}

/////////////////////////////////////////////////
void BulletPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::BULLET);
    physicsMsg.set_update_rate(this->GetUpdateRate());
    // This function was copied from ODEPhysics with portions commented out.
    // TODO: determine which of these should be implemented.
    // physicsMsg.set_solver_type(this->stepType);
    physicsMsg.set_dt(this->GetStepTime());
    // physicsMsg.set_iters(this->GetSORPGSIters());
    // physicsMsg.set_sor(this->GetSORPGSW());
    // physicsMsg.set_cfm(this->GetWorldCFM());
    // physicsMsg.set_erp(this->GetWorldERP());
    // physicsMsg.set_contact_max_correcting_vel(
    //     this->GetContactMaxCorrectingVel());
    // physicsMsg.set_contact_surface_layer(this->GetContactSurfaceLayer());
    physicsMsg.mutable_gravity()->CopyFrom(msgs::Convert(this->GetGravity()));

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void BulletPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  if (_msg->has_dt())
    this->SetStepTime(_msg->dt());

  if (_msg->has_update_rate())
    this->SetUpdateRate(_msg->update_rate());

  // Like OnRequest, this function was copied from ODEPhysics.
  // TODO: change this when changing OnRequest.
  // if (_msg->has_solver_type())
  // {
  //   sdf::ElementPtr solverElem =
  //     this->sdf->GetElement("ode")->GetElement("solver");
  //   if (_msg->solver_type() == "quick")
  //   {
  //     solverElem->GetAttribute("type")->Set("quick");
  //     this->physicsStepFunc = &dWorldQuickStep;
  //   }
  //   else if (_msg->solver_type() == "world")
  //   {
  //     solverElem->GetAttribute("type")->Set("world");
  //     this->physicsStepFunc = &dWorldStep;
  //   }
  // }

  // if (_msg->has_iters())
  //   this->SetSORPGSIters(_msg->iters());

  // if (_msg->has_sor())
  //   this->SetSORPGSW(_msg->sor());

  // if (_msg->has_cfm())
  //   this->SetWorldCFM(_msg->cfm());

  // if (_msg->has_erp())
  //   this->SetWorldERP(_msg->erp());

  // if (_msg->has_contact_max_correcting_vel())
  //   this->SetContactMaxCorrectingVel(_msg->contact_max_correcting_vel());

  // if (_msg->has_contact_surface_layer())
  //   this->SetContactSurfaceLayer(_msg->contact_surface_layer());

  if (_msg->has_gravity())
    this->SetGravity(msgs::Convert(_msg->gravity()));

  /// Make sure all models get at least one update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
void BulletPhysics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void BulletPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  this->physicsUpdateMutex->lock();

  // common::Time currTime =  this->world->GetRealTime();

  this->dynamicsWorld->stepSimulation(
      this->GetStepTime(), 1, this->GetStepTime());
  // this->lastUpdateTime = currTime;

  this->physicsUpdateMutex->unlock();
}

//////////////////////////////////////////////////
void BulletPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void BulletPhysics::Reset()
{
  // See DemoApplication::clientResetScene() in
  // bullet/Demos/OpenGL/DemoApplication.cpp
  // this->physicsUpdateMutex->lock();
  // this->physicsUpdateMutex->unlock();
}

//////////////////////////////////////////////////

// //////////////////////////////////////////////////
// void BulletPhysics::SetSORPGSIters(unsigned int _iters)
// {
//   // TODO: set SDF parameter
//   btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();
//   // Line below commented out because it wasn't helping pendulum test.
//   // info.m_numIterations = _iters;
// }

LinkPtr BulletPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  BulletLinkPtr link(new BulletLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr BulletPhysics::CreateCollision(const std::string &_type,
                                            LinkPtr _parent)
{
  BulletCollisionPtr collision(new BulletCollision(_parent));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_parent->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr BulletPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  BulletCollisionPtr collision =
    boost::shared_dynamic_cast<BulletCollision>(_collision);

  if (_type == "plane")
    shape.reset(new BulletPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new BulletSphereShape(collision));
  else if (_type == "box")
    shape.reset(new BulletBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new BulletCylinderShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new BulletTrimeshShape(collision));
  else if (_type == "heightmap")
    shape.reset(new BulletHeightmapShape(collision));
  else if (_type == "multiray")
    shape.reset(new BulletMultiRayShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new BulletRayShape(_collision));
    else
      shape.reset(new BulletRayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  /*
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
    */
  return shape;
}

//////////////////////////////////////////////////
JointPtr BulletPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "revolute")
    joint.reset(new BulletHingeJoint(this->dynamicsWorld, _parent));
  else if (_type == "universal")
    joint.reset(new BulletUniversalJoint(this->dynamicsWorld, _parent));
  else if (_type == "ball")
    joint.reset(new BulletBallJoint(this->dynamicsWorld, _parent));
  else if (_type == "prismatic")
    joint.reset(new BulletSliderJoint(this->dynamicsWorld, _parent));
  else if (_type == "revolute2")
    joint.reset(new BulletHinge2Joint(this->dynamicsWorld, _parent));
  else if (_type == "screw")
    joint.reset(new BulletScrewJoint(this->dynamicsWorld, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void BulletPhysics::ConvertMass(InertialPtr /*_inertial*/,
                                void * /*_engineMass*/)
{
}

//////////////////////////////////////////////////
void BulletPhysics::ConvertMass(void * /*_engineMass*/,
                                const InertialPtr /*_inertial*/)
{
}

//////////////////////////////////////////////////
double BulletPhysics::GetWorldCFM()
{
  sdf::ElementPtr elem = this->sdf->GetElement("bullet");
  elem = elem->GetElement("constraints");
  return elem->GetValueDouble("cfm");
}

//////////////////////////////////////////////////
void BulletPhysics::SetWorldCFM(double _cfm)
{
  sdf::ElementPtr elem = this->sdf->GetElement("bullet");
  elem = elem->GetElement("constraints");
  elem->GetElement("cfm")->Set(_cfm);

  btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();
  info.m_globalCfm = _cfm;
}

//////////////////////////////////////////////////
void BulletPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->Set(_gravity);
  this->dynamicsWorld->setGravity(
    BulletTypes::ConvertVector3(_gravity));
}

//////////////////////////////////////////////////
void BulletPhysics::DebugPrint() const
{
}
