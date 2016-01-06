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

#include "gazebo/physics/siconos/SiconosPlaneShape.hh"
#include "gazebo/physics/siconos/SiconosSphereShape.hh"

#include "gazebo/physics/siconos/SiconosHingeJoint.hh"
#include "gazebo/physics/siconos/SiconosFixedJoint.hh"

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

#include "gazebo/physics/siconos/siconos_inc.h"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("siconos", SiconosPhysics)

/// \brief Constructor
SiconosPhysics::SiconosPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  double t0 = 0; // initial computation time
  double T = std::numeric_limits<double>::infinity();
  dynamicsWorld.reset(new ::Model(t0, T));

  // Set random seed for physics engine based on gazebo's random seed.
  // Note: this was moved from physics::PhysicsEngine constructor.
  this->SetSeed(math::Rand::GetSeed());
}

/// \brief Destructor
SiconosPhysics::~SiconosPhysics()
{
}

//////////////////////////////////////////////////
void SiconosPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  math::Vector3 g = this->sdf->Get<math::Vector3>("gravity");
  // ODEPhysics checks this, so we will too.
  if (g == math::Vector3(0, 0, 0))
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  // this->dynamicsWorld->setGravity(btVector3(g.x, g.y, g.z));
}

/// \brief Initialize the physics engine.
void SiconosPhysics::Init()
{
}

/// \brief Init the engine for threads.
void SiconosPhysics::InitForThread()
{
}

/// \brief Update the physics engine collision.
void SiconosPhysics::UpdateCollision()
{
}

/// \brief Set the random number seed for the physics engine.
/// \param[in] _seed The random number seed.
void SiconosPhysics::SetSeed(uint32_t _seed)
{
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
    joint.reset(new SiconosHingeJoint(this->dynamicsWorld, _parent));
  else if (_type == "fixed")
    joint.reset(new SiconosFixedJoint(this->dynamicsWorld, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

/// \brief Set the gravity vector.
/// \param[in] _gravity New gravity vector.
void SiconosPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
}

/// \brief Debug print out of the physic engine state.
void SiconosPhysics::DebugPrint() const
{
}
