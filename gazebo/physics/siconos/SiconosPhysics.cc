/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/physics/siconos/SiconosPhysics.hh"

#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"

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

#include "SiconosWorld.hh"
#include "gazebo/physics/siconos/siconos_inc.h"

#include "gazebo/physics/siconos/SiconosPlaneShape.hh"
#include "gazebo/physics/siconos/SiconosSphereShape.hh"
#include "gazebo/physics/siconos/SiconosBoxShape.hh"
#include "gazebo/physics/siconos/SiconosCylinderShape.hh"
#include "gazebo/physics/siconos/SiconosRayShape.hh"
#include "gazebo/physics/siconos/SiconosMultiRayShape.hh"
#include "gazebo/physics/siconos/SiconosMeshShape.hh"

#include "gazebo/physics/siconos/SiconosHingeJoint.hh"
#include "gazebo/physics/siconos/SiconosHinge2Joint.hh"
#include "gazebo/physics/siconos/SiconosSliderJoint.hh"
#include "gazebo/physics/siconos/SiconosFixedJoint.hh"
#include "gazebo/physics/siconos/SiconosBallJoint.hh"

#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("siconos", SiconosPhysics)

/// \brief Constructor
SiconosPhysics::SiconosPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  // Set random seed for physics engine based on gazebo's random seed.
  // Note: this was moved from physics::PhysicsEngine constructor.
  this->SetSeed(ignition::math::Rand::Seed());

  // In the future, we could set this to various simulation method and
  // integration method combinations.  For now, the default is
  // TimeStepping with the MoreauJean OSI.
  this->solverType = "TimeStepping.MoreauJean";
}

/// \brief Destructor
SiconosPhysics::~SiconosPhysics()
{
}

//////////////////////////////////////////////////
void SiconosPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr siconosElem = this->sdf->GetElement("siconos");

  this->siconosWorld.reset(new SiconosWorld(this));

  auto g = this->world->Gravity();
  // ODEPhysics checks this, so we will too.
  if (g == ignition::math::Vector3d::Zero)
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->SetGravity(g);

  // Need to initialize the model which will receive created DSs.
  // Note: Doing this in Init() is too late!
  this->siconosWorld->setup();
}


/////////////////////////////////////////////////
void SiconosPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_min_step_size())
    this->SetParam("min_step_size", _msg->min_step_size());

  if (_msg->has_enable_physics())
    this->world->SetPhysicsEnabled(_msg->enable_physics());

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());

  if (_msg->has_max_step_size())
    this->SetMaxStepSize(_msg->max_step_size());

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

/////////////////////////////////////////////////
void SiconosPhysics::Init()
{
}

/////////////////////////////////////////////////
void SiconosPhysics::InitForThread()
{
}

/////////////////////////////////////////////////
void SiconosPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::SICONOS);
    physicsMsg.set_solver_type(this->solverType);
    // min_step_size is defined but not yet used
    physicsMsg.set_min_step_size(
      boost::any_cast<double>(this->GetParam("min_step_size")));

    // Interpret "iters" as Newton iterations
    physicsMsg.set_iters(
      boost::any_cast<int>(this->GetParam("newton_iters")));

    physicsMsg.set_enable_physics(this->world->PhysicsEnabled());

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

/// \brief Update the physics engine collision.
void SiconosPhysics::UpdateCollision()
{
  this->contactManager->ResetCount();

  // The collision detector in Siconos is called during
  // Simulation::computeOneStep.  Seeing as it's difficult to separate
  // it out here, and that there is no way to tell if UpdatePhysics()
  // is about to be called, nothing more to do.  (If we knew that
  // UpdatePhysics would be skipped and we just need to update contact
  // information, we could call the collision engine by itself, but
  // otherwise it will be called twice if we call it here.)
}

//////////////////////////////////////////////////
void SiconosPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  this->siconosWorld->compute();

  // For communicating position changes to Gazebo, check
  // ODELink::MoveCallback, use the dirtyPose mechanism.
  for (const auto &m : this->world->Models())
  {
      for (auto &lk : m->GetLinks()) {
          static_cast<SiconosLink*>(&*lk)->UpdatePoseFromBody();
          static_cast<SiconosLink*>(&*lk)->SetForce(ignition::math::Vector3d(0,0,0));
          static_cast<SiconosLink*>(&*lk)->SetTorque(ignition::math::Vector3d(0,0,0));
      }
  }
}

//////////////////////////////////////////////////
boost::any SiconosPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool SiconosPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  /* We steal some parameters from Bullet for now */
  sdf::ElementPtr bulletElem = this->sdf->GetElement("bullet");
  GZ_ASSERT(bulletElem != nullptr, "Bullet SDF element does not exist");

  sdf::ElementPtr siconosElem = this->sdf->GetElement("siconos");
  //GZ_ASSERT(siconosElem != nullptr, "Siconos SDF element does not exist");

  printf("SiconosPhysics::GetParam(%s)\n", _key.c_str());

  if (_key == "newton_iters")
    // Borrow Bullet's "iters" parameter for now
    _value = bulletElem->GetElement("solver")->Get<int>("iters");
  else if (_key == "min_step_size")
    _value = bulletElem->GetElement("solver")->Get<double>("min_step_size");
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  return true;
}

//////////////////////////////////////////////////
void SiconosPhysics::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
  //TODO
}

/// \brief Create a new body.
/// \param[in] _parent Parent model for the link.
LinkPtr SiconosPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  SiconosLinkPtr link(new SiconosLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

/// \brief Create a collision.
/// \param[in] _shapeType Type of collision to create.
/// \param[in] _link Parent link.
CollisionPtr SiconosPhysics::CreateCollision(const std::string &_type,
											 LinkPtr _parent)
{
  SiconosCollisionPtr collision(new SiconosCollision(_parent));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_parent->GetWorld());
  return collision;
}

/// \brief Create a physics::Shape object.
/// \param[in] _shapeType Type of shape to create.
/// \param[in] _collision Collision parent.
ShapePtr SiconosPhysics::CreateShape(const std::string &_type,
									 CollisionPtr _collision)
{
  ShapePtr shape;
  SiconosCollisionPtr collision =
    boost::dynamic_pointer_cast<SiconosCollision>(_collision);

  if (_type == "plane")
    shape.reset(new SiconosPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new SiconosSphereShape(collision));
  else if (_type == "box")
    shape.reset(new SiconosBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new SiconosCylinderShape(collision));
  else if (_type == "mesh")
    shape.reset(new SiconosMeshShape(collision));
  else if (_type == "multiray")
    shape.reset(new SiconosMultiRayShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new SiconosRayShape(_collision));
    else
      shape.reset(new SiconosRayShape(this->world->Physics()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

/// \brief Create a new joint.
/// \param[in] _type Type of joint to create.
/// \param[in] _parent Model parent.
JointPtr SiconosPhysics::CreateJoint(const std::string &_type,
									 ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "revolute")
    joint.reset(new SiconosHingeJoint(_parent, this->siconosWorld));
  else if (_type == "revolute2")
    joint.reset(new SiconosHinge2Joint(_parent, this->siconosWorld));
  else if (_type == "prismatic")
    joint.reset(new SiconosSliderJoint(_parent, this->siconosWorld));
  else if (_type == "fixed")
    joint.reset(new SiconosFixedJoint(_parent, this->siconosWorld));
  else if (_type == "ball")
    joint.reset(new SiconosBallJoint(_parent, this->siconosWorld));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

/// \brief Set the gravity vector.
/// \param[in] _gravity New gravity vector.
void SiconosPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->siconosWorld->SetGravity(_gravity);
}

/// \brief Look-up or add a collision group based on surface properties
long unsigned int SiconosPhysics::GetCollisionGroup(SiconosSurfaceParamsPtr surface)
{
  FrictionPyramidPtr friction = surface->FrictionPyramid();

  // TODO: how is restitution stored in SurfaceParams?
  double restitution = surface->normal_restitution;
  double mu1 = friction ? friction->MuPrimary() : 0.0;

  // Collision groups are the index into a list of relevant
  // parameters.  This look-up happens any time surface parameters are
  // changed that affect the non-smooth law, ie. friction and normal
  // restitution coefficients.  NewtonImpactFrictionNSL also supports
  // tangent restitution but it is not used in Gazebo for now.
  for ( std::vector<SiconosSurfaceParamsPtr>::size_type i = 0;
        i < this->collisionGroups.size(); i++ )
  {
    SiconosSurfaceParamsPtr surface2 = this->collisionGroups[i];
    FrictionPyramidPtr friction2 = surface2->FrictionPyramid();

    // Either both have no FrictionPyramid, or FrictionPyramid
    // MuPrimary() must match.
    bool frictionMatch = ((friction && friction2
                           && (std::abs(friction2->MuPrimary() - mu1)
                               < std::numeric_limits<double>::epsilon()))
                          || (!friction && !friction2));

    // Also check that restitution (bounce) matches
    if (frictionMatch && (std::abs(surface2->normal_restitution - restitution)
                          < std::numeric_limits<double>::epsilon()))
    {
      return i;
    }
  }

  // Must copy the surface params, because they can change by ProcessMsg.
  this->collisionGroups.push_back( surface->Copy() );
  return this->collisionGroups.size()-1;
}

/// \brief Look-up surface properties for a collision group
SiconosSurfaceParamsPtr
SiconosPhysics::GetCollisionGroupSurfaceParams(long unsigned int group)
{
  if (group < collisionGroups.size())
    return collisionGroups[group];
  else
    return nullptr;
}

/// \brief Debug print out of the physic engine state.
void SiconosPhysics::DebugPrint() const
{
}

//////////////////////////////////////////////////
void SiconosPhysics::SetSeed(uint32_t _seed)
{
  srand(_seed);
}
