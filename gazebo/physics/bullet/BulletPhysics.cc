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

#include <algorithm>
#include <string>

#include <ignition/math/Rand.hh>

#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"

#include "gazebo/physics/bullet/BulletPlaneShape.hh"
#include "gazebo/physics/bullet/BulletSphereShape.hh"
#include "gazebo/physics/bullet/BulletHeightmapShape.hh"
#include "gazebo/physics/bullet/BulletMultiRayShape.hh"
#include "gazebo/physics/bullet/BulletBoxShape.hh"
#include "gazebo/physics/bullet/BulletCylinderShape.hh"
#include "gazebo/physics/bullet/BulletMeshShape.hh"
#include "gazebo/physics/bullet/BulletPolylineShape.hh"
#include "gazebo/physics/bullet/BulletRayShape.hh"

#include "gazebo/physics/bullet/BulletHingeJoint.hh"
#include "gazebo/physics/bullet/BulletUniversalJoint.hh"
#include "gazebo/physics/bullet/BulletBallJoint.hh"
#include "gazebo/physics/bullet/BulletSliderJoint.hh"
#include "gazebo/physics/bullet/BulletHinge2Joint.hh"
#include "gazebo/physics/bullet/BulletScrewJoint.hh"
#include "gazebo/physics/bullet/BulletFixedJoint.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/ContactManager.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("bullet", BulletPhysics)

extern ContactAddedCallback gContactAddedCallback;
extern ContactProcessedCallback gContactProcessedCallback;

//////////////////////////////////////////////////
struct CollisionFilter : public btOverlapFilterCallback
{
  // return true when pairs need collision
  virtual bool needBroadphaseCollision(btBroadphaseProxy *_proxy0,
      btBroadphaseProxy *_proxy1) const
    {
      GZ_ASSERT(_proxy0 != nullptr && _proxy1 != nullptr,
          "Bullet broadphase overlapping pair proxies are null");

      bool collide = (_proxy0->m_collisionFilterGroup
          & _proxy1->m_collisionFilterMask) != 0;
      collide = collide && (_proxy1->m_collisionFilterGroup
          & _proxy0->m_collisionFilterMask);

      btRigidBody *rb0 = btRigidBody::upcast(
              static_cast<btCollisionObject *>(_proxy0->m_clientObject));
      if (!rb0)
        return collide;

      btRigidBody *rb1 = btRigidBody::upcast(
              static_cast<btCollisionObject *>(_proxy1->m_clientObject));
      if (!rb1)
         return collide;

      BulletLink *link0 = static_cast<BulletLink *>(
          rb0->getUserPointer());
      GZ_ASSERT(link0 != nullptr, "Link0 in collision pair is null");

      BulletLink *link1 = static_cast<BulletLink *>(
          rb1->getUserPointer());
      GZ_ASSERT(link1 != nullptr, "Link1 in collision pair is null");

      if (!link0->GetSelfCollide() || !link1->GetSelfCollide())
      {
        if (link0->GetModel() == link1->GetModel())
          collide = false;
      }
      return collide;
    }
};

//////////////////////////////////////////////////
void InternalTickCallback(btDynamicsWorld *_world, btScalar _timeStep)
{
  int numManifolds = _world->getDispatcher()->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i)
  {
    btPersistentManifold *contactManifold =
        _world->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject *obA =
        static_cast<const btCollisionObject *>(contactManifold->getBody0());
    const btCollisionObject *obB =
        static_cast<const btCollisionObject *>(contactManifold->getBody1());

    const btRigidBody *rbA = btRigidBody::upcast(obA);
    const btRigidBody *rbB = btRigidBody::upcast(obB);

    BulletLink *link1 = static_cast<BulletLink *>(
        obA->getUserPointer());
    GZ_ASSERT(link1 != nullptr, "Link1 in collision pair is null");

    BulletLink *link2 = static_cast<BulletLink *>(
        obB->getUserPointer());
    GZ_ASSERT(link2 != nullptr, "Link2 in collision pair is null");

    unsigned int colIndex = 0;
    CollisionPtr collisionPtr1 = link1->GetCollision(colIndex);
    CollisionPtr collisionPtr2 = link2->GetCollision(colIndex);

    if (!collisionPtr1 || !collisionPtr2)
      continue;

    PhysicsEnginePtr engine = collisionPtr1->GetWorld()->Physics();
    BulletPhysicsPtr bulletPhysics =
          boost::static_pointer_cast<BulletPhysics>(engine);

    // Add a new contact to the manager. This will return nullptr if no one is
    // listening for contact information.
    Contact *contactFeedback = bulletPhysics->GetContactManager()->NewContact(
        collisionPtr1.get(), collisionPtr2.get(),
        collisionPtr1->GetWorld()->SimTime());

    if (!contactFeedback)
      continue;

    auto body1Pose = link1->WorldPose();
    auto body2Pose = link2->WorldPose();
    auto cg1Pos = link1->GetInertial()->Pose().Pos();
    auto cg2Pos = link2->GetInertial()->Pose().Pos();
    ignition::math::Vector3d localForce1;
    ignition::math::Vector3d localForce2;
    ignition::math::Vector3d localTorque1;
    ignition::math::Vector3d localTorque2;

    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; ++j)
    {
      btManifoldPoint &pt = contactManifold->getContactPoint(j);
      if (pt.getDistance() < 0.f)
      {
        const btVector3 &ptB = pt.getPositionWorldOnB();
        const btVector3 &normalOnB = pt.m_normalWorldOnB;
        btVector3 impulse = pt.m_appliedImpulse * normalOnB;

        // calculate force in world frame
        btVector3 force = impulse/_timeStep;

        // calculate torque in world frame
        btVector3 torqueA = (ptB-rbA->getCenterOfMassPosition()).cross(force);
        btVector3 torqueB = (ptB-rbB->getCenterOfMassPosition()).cross(-force);

        // Convert from world to link frame
        localForce1 = body1Pose.Rot().RotateVectorReverse(
            BulletTypes::ConvertVector3Ign(force));
        localForce2 = body2Pose.Rot().RotateVectorReverse(
            BulletTypes::ConvertVector3Ign(-force));
        localTorque1 = body1Pose.Rot().RotateVectorReverse(
            BulletTypes::ConvertVector3Ign(torqueA));
        localTorque2 = body2Pose.Rot().RotateVectorReverse(
            BulletTypes::ConvertVector3Ign(torqueB));

        contactFeedback->positions[j] = BulletTypes::ConvertVector3Ign(ptB);
        contactFeedback->normals[j] = BulletTypes::ConvertVector3Ign(normalOnB);
        contactFeedback->depths[j] = -pt.getDistance();
        if (!link1->IsStatic())
        {
          contactFeedback->wrench[j].body1Force = localForce1;
          contactFeedback->wrench[j].body1Torque = localTorque1;
        }
        if (!link2->IsStatic())
        {
          contactFeedback->wrench[j].body2Force = localForce2;
          contactFeedback->wrench[j].body2Torque = localTorque2;
        }
        contactFeedback->count++;
      }
    }
  }
}

//////////////////////////////////////////////////
bool ContactCallback(btManifoldPoint &_cp,
    const btCollisionObjectWrapper *_obj0, int /*_partId0*/, int /*_index0*/,
    const btCollisionObjectWrapper *_obj1, int /*_partId1*/, int /*_index1*/)
{
  _cp.m_combinedFriction = std::min(_obj1->m_collisionObject->getFriction(),
    _obj0->m_collisionObject->getFriction());

  // this return value is currently ignored, but to be on the safe side:
  //  return false if you don't calculate friction
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

  btOverlapFilterCallback *filterCallback = new CollisionFilter();
  btOverlappingPairCache* pairCache = this->dynamicsWorld->getPairCache();
  GZ_ASSERT(pairCache != nullptr,
      "Bullet broadphase overlapping pair cache is null");
  pairCache->setOverlapFilterCallback(filterCallback);

  // TODO: Enable this to do custom contact setting
  gContactAddedCallback = ContactCallback;
  gContactProcessedCallback = ContactProcessed;

  this->dynamicsWorld->setInternalTickCallback(
      InternalTickCallback, static_cast<void *>(this));

  // Set random seed for physics engine based on gazebo's random seed.
  // Note: this was moved from physics::PhysicsEngine constructor.
  this->SetSeed(ignition::math::Rand::Seed());

  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
}

//////////////////////////////////////////////////
BulletPhysics::~BulletPhysics()
{
  this->Fini();
}

//////////////////////////////////////////////////
void BulletPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr bulletElem = this->sdf->GetElement("bullet");

  auto g = this->world->Gravity();
  // ODEPhysics checks this, so we will too.
  if (g == ignition::math::Vector3d::Zero)
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->dynamicsWorld->setGravity(btVector3(g.X(), g.Y(), g.Z()));

  btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();

  // Split impulse feature. This reduces large bounces from deep penetrations,
  // but can lead to improper stacking of objects, see
  // http://web.archive.org/web/20120430155635/http://bulletphysics.org/
  //     mediawiki-1.5.8/index.php/BtContactSolverInfo#Split_Impulse
  info.m_splitImpulse =
      boost::any_cast<bool>(this->GetParam("split_impulse"));
  info.m_splitImpulsePenetrationThreshold =
    boost::any_cast<double>(
    this->GetParam("split_impulse_penetration_threshold"));

  // Use multiple friction directions.
  // This is important for rolling without slip (see issue #480)
  info.m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS;

  // the following are undocumented members of btContactSolverInfo
  // m_globalCfm: constraint force mixing
  info.m_globalCfm =
    bulletElem->GetElement("constraints")->Get<double>("cfm");
  // m_erp: Baumgarte factor
  info.m_erp = bulletElem->GetElement("constraints")->Get<double>("erp");

  info.m_numIterations =
      boost::any_cast<int>(this->GetParam("iters"));
  info.m_sor =
      boost::any_cast<double>(this->GetParam("sor"));

  gzlog << " debug physics: "
        << " iters[" << info.m_numIterations
        << "] sor[" << info.m_sor
        << "] erp[" << info.m_erp
        << "] cfm[" << info.m_globalCfm
        << "] split[" << info.m_splitImpulse
        << "] split tol[" << info.m_splitImpulsePenetrationThreshold
        << "]\n";

  // debugging
  // info.m_numIterations = 1000;
  // info.m_sor = 1.0;
  // info.m_erp = 0.2;
  // info.m_globalCfm = 0.0;
  // info.m_splitImpulse = 0;
  // info.m_splitImpulsePenetrationThreshold = 0.0;
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
    physicsMsg.set_solver_type(this->solverType);
    // min_step_size is defined but not yet used
    physicsMsg.set_min_step_size(
      boost::any_cast<double>(this->GetParam("min_step_size")));
    physicsMsg.set_iters(
      boost::any_cast<int>(this->GetParam("iters")));
    physicsMsg.set_enable_physics(this->world->PhysicsEnabled());
    physicsMsg.set_sor(
      boost::any_cast<double>(this->GetParam("sor")));
    physicsMsg.set_cfm(
      boost::any_cast<double>(this->GetParam("cfm")));
    physicsMsg.set_erp(
      boost::any_cast<double>(this->GetParam("erp")));

    physicsMsg.set_contact_surface_layer(
      boost::any_cast<double>(this->GetParam("contact_surface_layer")));

    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
        msgs::Convert(this->world->MagneticField()));
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void BulletPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_min_step_size())
    this->SetParam("min_step_size", _msg->min_step_size());

  if (_msg->has_solver_type())
    this->SetParam("solver_type", _msg->solver_type());

  if (_msg->has_iters())
    this->SetParam("iters", _msg->iters());

  if (_msg->has_sor())
    this->SetParam("sor", _msg->sor());

  if (_msg->has_cfm())
    this->SetParam("cfm", _msg->cfm());

  if (_msg->has_erp())
    this->SetParam("erp", _msg->erp());

  if (_msg->has_enable_physics())
    this->world->SetPhysicsEnabled(_msg->enable_physics());

  if (_msg->has_contact_surface_layer())
    this->SetParam("contact_surface_layer", _msg->contact_surface_layer());

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
  {
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());
  }

  if (_msg->has_max_step_size())
  {
    this->SetMaxStepSize(_msg->max_step_size());
  }

  /// Make sure all models get at least one update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
void BulletPhysics::UpdateCollision()
{
  this->contactManager->ResetCount();
}

//////////////////////////////////////////////////
void BulletPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  this->dynamicsWorld->stepSimulation(
    this->maxStepSize, 1, this->maxStepSize);
}

//////////////////////////////////////////////////
void BulletPhysics::Fini()
{
  // Delete in reverse-order of creation
  if (this->dynamicsWorld)
    delete this->dynamicsWorld;
  this->dynamicsWorld = nullptr;

  if (this->solver)
    delete this->solver;
  this->solver = nullptr;

  if (this->broadPhase)
    delete this->broadPhase;
  this->broadPhase = nullptr;

  if (this->dispatcher)
    delete this->dispatcher;
  this->dispatcher = nullptr;

  if (this->collisionConfig)
    delete this->collisionConfig;
  this->collisionConfig = nullptr;

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

//////////////////////////////////////////////////
void BulletPhysics::SetSORPGSIters(unsigned int _iters)
{
  // TODO: set SDF parameter
  btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();
  // Line below commented out because it wasn't helping pendulum test.
  info.m_numIterations = _iters;

  this->sdf->GetElement("bullet")->GetElement(
      "solver")->GetElement("iters")->Set(_iters);
}

//////////////////////////////////////////////////
bool BulletPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  sdf::ElementPtr bulletElem = this->sdf->GetElement("bullet");
  GZ_ASSERT(bulletElem != nullptr, "Bullet SDF element does not exist");

  btContactSolverInfo& info = this->dynamicsWorld->getSolverInfo();

  try
  {
    if (_key == "solver_type")
    {
      std::string value = boost::any_cast<std::string>(_value);
      if (value == "sequential_impulse")
      {
        bulletElem->GetElement("solver")->GetElement("type")->Set(value);
        this->solverType = value;
      }
      else
      {
        gzwarn << "Currently only 'sequential_impulse' solver is supported"
               << std::endl;
        return false;
      }
    }
    else if (_key == "cfm")
    {
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("constraints")->GetElement("cfm")->Set(value);
      info.m_globalCfm = value;
    }
    else if (_key == "erp")
    {
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("constraints")->GetElement("erp")->Set(value);
      info.m_erp = value;
    }
    else if (_key == "iters")
    {
      int value = boost::any_cast<int>(_value);
      bulletElem->GetElement("solver")->GetElement("iters")->Set(value);
      info.m_numIterations = value;
    }
    else if (_key == "sor")
    {
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("solver")->GetElement("sor")->Set(value);
      info.m_sor = value;
    }
    else if (_key == "contact_surface_layer")
    {
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("constraints")->GetElement(
          "contact_surface_layer")->Set(value);
    }
    else if (_key == "split_impulse")
    {
      bool value = boost::any_cast<bool>(_value);
      bulletElem->GetElement("constraints")->GetElement(
          "split_impulse")->Set(value);
    }
    else if (_key == "split_impulse_penetration_threshold")
    {
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("constraints")->GetElement(
          "split_impulse_penetration_threshold")->Set(value);
    }
    else if (_key == "max_contacts")
    {
      /// TODO: Implement max contacts param
      int value = boost::any_cast<int>(_value);
      this->sdf->GetElement("max_contacts")->GetValue()->Set(value);
    }
    else if (_key == "min_step_size")
    {
      /// TODO: Implement min step size param
      double value = boost::any_cast<double>(_value);
      bulletElem->GetElement("solver")->GetElement("min_step_size")->Set(value);
    }
    else
    {
      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "BulletPhysics::SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
boost::any BulletPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool BulletPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  sdf::ElementPtr bulletElem = this->sdf->GetElement("bullet");
  GZ_ASSERT(bulletElem != nullptr, "Bullet SDF element does not exist");

  if (_key == "solver_type")
    _value = bulletElem->GetElement("solver")->Get<std::string>("type");
  else if (_key == "cfm")
    _value = bulletElem->GetElement("constraints")->Get<double>("cfm");
  else if (_key == "erp")
    _value = bulletElem->GetElement("constraints")->Get<double>("erp");
  else if (_key == "iters")
    _value = bulletElem->GetElement("solver")->Get<int>("iters");
  else if (_key == "sor")
    _value = bulletElem->GetElement("solver")->Get<double>("sor");
  else if (_key == "contact_surface_layer")
    _value = bulletElem->GetElement("constraints")->Get<double>(
        "contact_surface_layer");
  else if (_key == "split_impulse")
  {
    _value = bulletElem->GetElement("constraints")->Get<bool>(
      "split_impulse");
  }
  else if (_key == "split_impulse_penetration_threshold")
  {
    _value = bulletElem->GetElement("constraints")->Get<double>(
      "split_impulse_penetration_threshold");
  }
  else if (_key == "max_contacts")
    _value = this->sdf->GetElement("max_contacts")->Get<int>();
  else if (_key == "min_step_size")
    _value = bulletElem->GetElement("solver")->Get<double>("min_step_size");
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  return true;
}

//////////////////////////////////////////////////
LinkPtr BulletPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == nullptr)
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
    boost::dynamic_pointer_cast<BulletCollision>(_collision);

  if (_type == "plane")
    shape.reset(new BulletPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new BulletSphereShape(collision));
  else if (_type == "box")
    shape.reset(new BulletBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new BulletCylinderShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new BulletMeshShape(collision));
  else if (_type == "polyline")
    shape.reset(new BulletPolylineShape(collision));
  else if (_type == "heightmap")
    shape.reset(new BulletHeightmapShape(collision));
  else if (_type == "multiray")
    shape.reset(new BulletMultiRayShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new BulletRayShape(_collision));
    else
      shape.reset(new BulletRayShape(this->world->Physics()));
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
  else if (_type == "fixed")
    joint.reset(new BulletFixedJoint(this->dynamicsWorld, _parent));
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
  return elem->Get<double>("cfm");
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
void BulletPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->world->SetGravitySDF(_gravity);
  this->dynamicsWorld->setGravity(
    BulletTypes::ConvertVector3(_gravity));
}

//////////////////////////////////////////////////
void BulletPhysics::DebugPrint() const
{
}

/////////////////////////////////////////////////
void BulletPhysics::SetSeed(uint32_t /*_seed*/)
{
  // GEN_srand is defined in btRandom.h, but nothing in bullet uses it
  // GEN_srand(_seed);

  // The best bet is probably btSequentialImpulseConstraintSolver::setRandSeed,
  // but it's not a static function.
  // There's 2 other instances of random number generation in bullet classes:
  //  btSoftBody.cpp:1160
  //  btConvexHullComputer.cpp:2188
  // It's going to be blank for now.
  /// \todo Implement this function.
}
