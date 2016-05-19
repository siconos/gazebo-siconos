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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
//#include "gazebo/physics/siconos/SiconosMotionState.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"

#include <BodyDS.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosLink::SiconosLink(EntityPtr _parent)
    : Link(_parent)
{
}

//////////////////////////////////////////////////
SiconosLink::~SiconosLink()
{
}

//////////////////////////////////////////////////
void SiconosLink::Load(sdf::ElementPtr _sdf)
{
  this->siconosPhysics = boost::dynamic_pointer_cast<SiconosPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->siconosPhysics == NULL)
    gzthrow("Not using the siconos physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosLink::Init()
{
  // Set the initial pose of the body
  // this->motionState.reset(new SiconosMotionState(
  //   boost::dynamic_pointer_cast<Link>(shared_from_this())));

  Link::Init();

  GZ_ASSERT(this->sdf != NULL, "Unable to initialize link, SDF is NULL");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));

  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  // btScalar mass = this->inertial->GetMass();
  // The siconos dynamics solver checks for zero mass to identify static and
  // kinematic bodies.
  if (this->IsStatic() || this->GetKinematic())
  {
    // mass = 0;
    this->inertial->SetInertiaMatrix(0, 0, 0, 0, 0, 0);
  }
  // btVector3 fallInertia(0, 0, 0);
  math::Vector3 cogVec = this->inertial->GetCoG();

  /// \todo FIXME:  Friction Parameters

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SiconosCollisionPtr collision;
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      SP::SiconosShape shape = collision->GetCollisionShape();

      /* Siconos TODO
      SurfaceParamsPtr surface = collision->GetSurface();
      GZ_ASSERT(surface, "Surface pointer for is invalid");
      FrictionPyramidPtr friction = surface->GetFrictionPyramid();
      GZ_ASSERT(friction, "Friction pointer is invalid");

      hackMu1 = friction->GetMuPrimary();
      hackMu2 = friction->GetMuSecondary();
      // gzerr << "link[" << this->GetName()
      //       << "] mu[" << hackMu1
      //       << "] mu2[" << hackMu2 << "]\n";

      math::Pose relativePose = collision->GetRelativePose();
      relativePose.pos -= cogVec;
      // if (!this->compoundShape)
      //   this->compoundShape = new btCompoundShape();
      // dynamic_cast<btCompoundShape *>(this->compoundShape)->addChildShape(
      //     SiconosTypes::ConvertPose(relativePose), shape);
      */
    }
  }

  // Create the new rigid body
  SP::SiconosVector q(new SiconosVector(7));
  SP::SiconosVector v(new SiconosVector(6));
  q->zero();
  v->zero();
  (*q)(3) = 1.0;
  this->body.reset(new BodyDS(q,v,1.0));

  // /// \TODO: get friction from collision object
  // this->body->setAnisotropicFriction(btVector3(1, 1, 1),
  //   btCollisionObject::CF_ANISOTROPIC_FRICTION);
  // this->body->setFriction(0.5*(hackMu1 + hackMu2));  // Hack

  SP::SiconosWorld siconosWorld = this->siconosPhysics->GetSiconosWorld();
  GZ_ASSERT(siconosWorld != NULL, "Siconos dynamics world is NULL");

  // siconos supports setting bits to a rigid body but not individual
  // shapes/collisions so find the first child collision and set rigid body to
  // use its category and collision bits.
  unsigned int categortyBits = GZ_ALL_COLLIDE;
  unsigned int collideBits = GZ_ALL_COLLIDE;
  SiconosCollisionPtr collision;
  for (Base_V::iterator iter = this->children.begin();
         iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      categortyBits = collision->GetCategoryBits();
      collideBits = collision->GetCollideBits();
      break;
    }
  }
  // siconosWorld->addRigidBody(this->body, categortyBits, collideBits);

  // Only use auto disable if no joints and no sensors are present
  // this->body->setActivationState(DISABLE_DEACTIVATION);
  // if (this->GetModel()->GetAutoDisable() &&
  //     this->GetModel()->GetJointCount() == 0 &&
  //     this->GetSensorCount() == 0)
  // {
  //   this->body->setActivationState(ACTIVE_TAG);
  //   this->body->setSleepingThresholds(0.1, 0.1);
  //   this->body->setDeactivationTime(1.0);
  // }

  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::Fini()
{
  Link::Fini();
  SP::SiconosWorld world = this->siconosPhysics->GetSiconosWorld();
  GZ_ASSERT(world != NULL, "SiconosWorld is NULL");
  // siconosWorld->removeRigidBody(this->body);
}

/////////////////////////////////////////////////////////////////////
void SiconosLink::UpdateMass()
{
  if (this->body && this->inertial)
  {
    // TODO Siconos
    // this->body->setMassProps(this->inertial->GetMass(),
    //     SiconosTypes::ConvertVector3(this->inertial->GetPrincipalMoments()));
  }
}

//////////////////////////////////////////////////
void SiconosLink::SetGravityMode(bool _mode)
{
  // TODO Siconos
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
    return;
  }

  if (_mode == false)
    // this->body->setGravity(btVector3(0, 0, 0));
    // this->body->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
    ;
  else
  {
    math::Vector3 g = this->siconosPhysics->GetGravity();
    // Siconos TODO
    // this->body->setGravity(btVector3(g.x, g.y, g.z));
    /*btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->body->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool SiconosLink::GetGravityMode() const
{
  bool result = false;
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetGravityMode returns "
          << result << " by default." << std::endl;
    return result;
  }
  // btVector3 g = this->body->getGravity();
  // result = !math::equal(static_cast<double>(g.length()), 0.0);

  return result;
}

//////////////////////////////////////////////////
void SiconosLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
}

//////////////////////////////////////////////////
/// Adapted from ODELink::OnPoseChange
void SiconosLink::OnPoseChange()
{
  Link::OnPoseChange();
  printf("SiconosLink::OnPoseChange()\n");

  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to respond to OnPoseChange"
          << std::endl;
    return;
  }

  // this->SetEnabled(true);

  const math::Pose myPose = this->GetWorldCoGPose();

  // TODO Siconos
  // this->body->setCenterOfMassTransform(
  //   SiconosTypes::ConvertPose(myPose));
}

//////////////////////////////////////////////////
bool SiconosLink::GetEnabled() const
{
  // This function and its counterpart SiconosLink::SetEnabled
  // don't do anything yet.
  return true;
}

//////////////////////////////////////////////////
void SiconosLink::SetEnabled(bool /*_enable*/) const
{
  // Siconos TODO
  /*
  if (!this->body)
    return;

  if (_enable)
    this->body->activate(true);
  else
    this->body->setActivationState(WANTS_DEACTIVATION);
    */
}

//////////////////////////////////////////////////
void SiconosLink::SetLinearVel(const math::Vector3 &_vel)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  printf("SiconosLink::SetLinearVel()\n");
  // TODO Siconos
  // this->body->setLinearVelocity(SiconosTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldCoGLinearVel() const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  // btVector3 vel = this->body->getLinearVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  math::Pose wPose = this->GetWorldPose();
  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  math::Vector3 offsetFromCoG = wPose.rot*(_offset - this->inertial->GetCoG());
  // Siconos TODO
  // btVector3 vel = this->body->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldLinearVel(const math::Vector3 &_offset,
                                            const math::Quaternion &_q) const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  math::Pose wPose = this->GetWorldPose();
  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  math::Vector3 offsetFromCoG = _q*_offset
        - wPose.rot*this->inertial->GetCoG();
  // Siconos TODO
  // btVector3 vel = this->body->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularVel(const math::Vector3 &_vel)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // Siconos TODO
  printf("SiconosLink::SetAngularVel()\n");
  // this->body->setAngularVelocity(SiconosTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldAngularVel() const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldAngularVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  // Siconos TODO
  // btVector3 vel = this->body->getAngularVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetForce(const math::Vector3 &_force)
{
  if (!this->body)
    return;

  // Siconos TODO
  printf("SiconosLink::SetForce()\n");
  // this->body->applyCentralForce(
  //   btVector3(_force.x, _force.y, _force.z));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldForce() const
{
  if (!this->body)
    return math::Vector3(0, 0, 0);

  // btVector3 btVec;

  // Siconos TODO
  // btVec = this->body->getTotalForce();

  // return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetTorque(const math::Vector3 &_torque)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // Siconos TODO
  printf("SiconosLink::SetTorque()\n");
  // this->body->applyTorque(SiconosTypes::ConvertVector3(_torque));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldTorque() const
{
  /*
  if (!this->body)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  // Siconos TODO
  btVec = this->body->getTotalTorque();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
SP::BodyDS SiconosLink::GetSiconosBodyDS() const
{
  return this->body;
}

//////////////////////////////////////////////////
void SiconosLink::ClearCollisionCache()
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to ClearCollisionCache" << std::endl;
    return;
  }

  SP::SiconosWorld world = this->siconosPhysics->GetSiconosWorld();
  GZ_ASSERT(world != NULL, "SiconosWorld is NULL");

  // Siconos TODO
  // siconosWorld->updateSingleAabb(this->body);
  // siconosWorld->getBroadphase()->getOverlappingPairCache()->
  //     cleanProxyFromPairs(this->body->getBroadphaseHandle(),
  //     siconosWorld->getDispatcher());
}

//////////////////////////////////////////////////
void SiconosLink::SetLinearDamping(double _damping)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearDamping"
          << std::endl;
    return;
  }
  // Siconos TODO
  printf("SiconosLink::SetLinearDamping()\n");
  // this->body->setDamping((btScalar)_damping,
  //     (btScalar)this->body->getAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularDamping(double _damping)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularDamping"
          << std::endl;
    return;
  }
  // Siconos TODO
  printf("SiconosLink::SetAngularDamping()\n");
  // this->body->setDamping(
  //     (btScalar)this->body->getLinearDamping(), (btScalar)_damping);
}

/////////////////////////////////////////////////
void SiconosLink::AddForce(const math::Vector3 &/*_force*/)
{
  gzlog << "SiconosLink::AddForce not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeForce(const math::Vector3 &/*_force*/)
{
  gzlog << "SiconosLink::AddRelativeForce not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtWorldPosition(const math::Vector3 &/*_force*/,
                                         const math::Vector3 &/*_pos*/)
{
  gzlog << "SiconosLink::AddForceAtWorldPosition not yet implemented."
        << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtRelativePosition(const math::Vector3 &/*_force*/,
                  const math::Vector3 &/*_relpos*/)
{
  gzlog << "SiconosLink::AddForceAtRelativePosition not yet implemented."
        << std::endl;
}

//////////////////////////////////////////////////
void SiconosLink::AddLinkForce(const math::Vector3 &/*_force*/,
    const math::Vector3 &/*_offset*/)
{
  gzlog << "SiconosLink::AddLinkForce not yet implemented (#1476)."
        << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddTorque(const math::Vector3 &/*_torque*/)
{
  gzlog << "SiconosLink::AddTorque not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeTorque(const math::Vector3 &/*_torque*/)
{
  gzlog << "SiconosLink::AddRelativeTorque not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::SetAutoDisable(bool /*_disable*/)
{
  gzlog << "SiconosLink::SetAutoDisable not yet implemented." << std::endl;
}

//////////////////////////////////////////////////
void SiconosLink::SetLinkStatic(bool /*_static*/)
{
  gzlog << "To be implemented\n";
}
