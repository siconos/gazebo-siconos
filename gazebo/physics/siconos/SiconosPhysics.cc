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
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"

#include "SiconosWorld.hh"
#include "gazebo/physics/siconos/siconos_inc.h"

#include "gazebo/physics/siconos/SiconosPlaneShape.hh"
#include "gazebo/physics/siconos/SiconosSphereShape.hh"
#include "gazebo/physics/siconos/SiconosBoxShape.hh"
#include "gazebo/physics/siconos/SiconosRayShape.hh"

#include "gazebo/physics/siconos/SiconosHingeJoint.hh"
#include "gazebo/physics/siconos/SiconosFixedJoint.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("siconos", SiconosPhysics)

/// \brief Constructor
SiconosPhysics::SiconosPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  this->siconosWorld.reset(new ::SiconosWorld());

  // Set random seed for physics engine based on gazebo's random seed.
  // Note: this was moved from physics::PhysicsEngine constructor.
  this->SetSeed(ignition::math::Rand::Seed());
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

  auto g = this->world->Gravity();
  // ODEPhysics checks this, so we will too.
  if (g == ignition::math::Vector3d::Zero)
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->SetGravity(g);

  // Need to initialize the model which will receive created DSs.
  // Note: Doing this in Init() is too late!
  this->siconosWorld->init();
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
    physicsMsg.set_iters(
      boost::any_cast<int>(this->GetParam("iters")));
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
  sdf::ElementPtr siconosElem = this->sdf->GetElement("siconos");
  GZ_ASSERT(siconosElem != nullptr, "Siconos SDF element does not exist");
  return PhysicsEngine::GetParam(_key, _value);
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
    joint.reset(new SiconosHingeJoint(this->siconosWorld, _parent));
  else if (_type == "fixed")
    joint.reset(new SiconosFixedJoint(this->siconosWorld, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

/// \brief Set the gravity vector.
/// \param[in] _gravity New gravity vector.
void SiconosPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->siconosWorld->SetGravity(_gravity.X(), _gravity.Y(), _gravity.Z());
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
