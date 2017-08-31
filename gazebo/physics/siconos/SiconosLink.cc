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
#include "gazebo/physics/siconos/SiconosWorld.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"

#include <BodyDS.hpp>
#include <SiconosContactor.hpp>
#include <SiconosCollisionManager.hpp>
#include <SiconosBulletCollisionManager.hpp>

using namespace gazebo;
using namespace physics;

namespace gazebo { namespace physics {
// A private class to attach a weak_ptr back to the SiconosLink for sensors.
class GzBodyDS : public BodyDS
{
protected:
  GzBodyDS() : BodyDS() {};
public:
  GzBodyDS(SP::SiconosVector position, SP::SiconosVector velocity,
           double mass, SP::SimpleMatrix inertia = SP::SimpleMatrix())
    : BodyDS(position, velocity, mass, inertia) {}
  SiconosLinkWeakPtr link;
};
}}

//////////////////////////////////////////////////
SiconosLink::SiconosLink(EntityPtr _parent)
    : Link(_parent)
    , contactorSet(new SiconosContactorSet())
    , gravityMode(true)
{
  this->siconosPhysics = boost::dynamic_pointer_cast<SiconosPhysics>(
      this->GetWorld()->Physics());
  if (this->siconosPhysics == nullptr)
    gzerr << "Not using the Siconos physics engine\n";
}

//////////////////////////////////////////////////
SiconosLink::~SiconosLink()
{
}

//////////////////////////////////////////////////
void SiconosLink::Load(sdf::ElementPtr _sdf)
{
  if (this->siconosPhysics)
    Link::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosLink::Init()
{
  Link::Init();

  GZ_ASSERT(this->sdf != NULL, "Unable to initialize link, SDF is NULL");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));

  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  double mass = this->inertial->Mass();

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SiconosCollisionPtr collision;
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      SP::SiconosContactor c(collision->GetSiconosContactor());
      if (c) {
        this->contactorSet->append(c);
      } else
        GZ_ASSERT(c, "Contactor is invalid");
    }
  }

  // Initial position and velocity
  SP::SiconosVector q(SiconosTypes::ConvertPose(this->WorldCoGPose()));
  SP::SiconosVector v(std11::make_shared<SiconosVector>(6));
  SiconosTypes::ConvertVector3(this->WorldCoGLinearVel(), *v, 0);
  SiconosTypes::ConvertVector3(this->RelativeAngularVel(), *v, 3);

  if (this->IsStatic() || this->GetKinematic())
  {
      // If the object is static or kinematic, do not create a dynamical
      // system for it.
  }
  else
  {
      // Otherwise, create the dynamic system with given inertia and
      // initial state vectors
      std11::shared_ptr<GzBodyDS> gzbody;
      this->body = gzbody = std11::make_shared<GzBodyDS>(q,v,mass);

      // We need a weak pointer back to the Link for sensors to find
      // it based on collision world queries.
      LinkPtr self = this->GetModel()->GetLinkById(this->GetId());
      gzbody->link = boost::static_pointer_cast<SiconosLink>(self);

      // give Siconos un-rotated inertia
      if (this->inertial)
      {
        auto moi = this->inertial->MOI(
          ignition::math::Pose3d(this->inertial->CoG(),
                                 ignition::math::Quaterniond::Identity));
        SP::SimpleMatrix inertia = SiconosTypes::ConvertMatrix3(moi);
        this->body->setInertia(inertia);
        this->body->setUseContactorInertia(false);
      }

      // Set up external force/torque vectors for this body
      this->force = std11::make_shared<SiconosVector>(3);
      this->force->zero();
      this->body->setFExtPtr(this->force);

      this->torque = std11::make_shared<SiconosVector>(3);
      this->torque->zero();
      this->body->setMExtPtr(this->torque);

      // Assign contactor shapes to this body
      this->body->setContactors(this->contactorSet);
  }

  // /// \TODO: get friction from collision object
  // this->body->setAnisotropicFriction(btVector3(1, 1, 1),
  //   btCollisionObject::CF_ANISOTROPIC_FRICTION);
  // this->body->setFriction(0.5*(hackMu1 + hackMu2));  // Hack

  SP::SiconosWorld siconosWorld = this->siconosPhysics->GetSiconosWorld();
  GZ_ASSERT(siconosWorld != NULL, "Siconos dynamics world is NULL");

  // TODO collision bits

  if (this->body)
  {
    // Add DS to the model
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->insertDynamicalSystem(this->body);

    /* Prepare the simulation integrator for the new DS */
    siconosWorld->GetSimulation()->prepareIntegratorForDS(
      siconosWorld->GetOneStepIntegrator(), this->body,
      siconosWorld->GetModel(), siconosWorld->GetSimulation()->nextTime());
  }
  else {
    // Add contactor to collision world
    siconosWorld->GetManager()->insertStaticContactorSet(this->contactorSet, q);
  }

  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::Fini()
{
  Link::Fini();

  SP::SiconosWorld siconosWorld = this->siconosPhysics->GetSiconosWorld();
  GZ_ASSERT(siconosWorld != NULL, "SiconosWorld is NULL");

  if (this->body) {
    siconosWorld->GetManager()->removeBody(this->body);
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->removeDynamicalSystem(this->body);
  }

  this->body.reset();
}

/////////////////////////////////////////////////////////////////////
void SiconosLink::UpdateMass()
{
  if (this->body && this->inertial)
  {
    auto moi = this->inertial->MOI(
      ignition::math::Pose3d(this->inertial->CoG(),
                             ignition::math::Quaterniond::Identity));
    SP::SimpleMatrix inertia = SiconosTypes::ConvertMatrix3(moi);
    this->body->setInertia(inertia);
    this->body->setUseContactorInertia(false);

    // In case the center of mass changed:
    this->OnPoseChange();
  }
}

//////////////////////////////////////////////////
void SiconosLink::SetGravityMode(bool _mode)
{
  this->gravityMode = _mode;
}

//////////////////////////////////////////////////
bool SiconosLink::GetGravityMode() const
{
  return this->gravityMode;
}

//////////////////////////////////////////////////
void SiconosLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
}

//////////////////////////////////////////////////
void SiconosLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to respond to OnPoseChange"
          << std::endl;
    return;
  }

  this->SetEnabled(true);

  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");

  SiconosTypes::ConvertPoseToVector7(this->WorldCoGPose(), this->body->q());
  this->body->swapInMemory();
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
void SiconosLink::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  // TODO Siconos
  (*this->body->velocity())(0) = _vel.X();
  (*this->body->velocity())(1) = _vel.Y();
  (*this->body->velocity())(2) = _vel.Z();
  this->body->swapInMemory();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldCoGLinearVel() const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  return SiconosTypes::ConvertVector3( this->body->linearVelocity(true) );
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset) const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");

  auto offsetFromCoG = this->WorldPose().Rot()
    .RotateVector(_offset - this->inertial->CoG());

  auto worldAngularVel =
    SiconosTypes::ConvertVector3( this->body->angularVelocity(true) );

  auto worldLinearVel =
    SiconosTypes::ConvertVector3( this->body->linearVelocity(true) );

  auto worldPointVel =
    worldLinearVel + worldAngularVel.Cross(offsetFromCoG);

  return worldPointVel;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldLinearVel(
  const ignition::math::Vector3d &_offset,
  const ignition::math::Quaterniond &_q) const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");

  auto wQ = this->WorldPose().Rot();
  auto offsetFromCoG = wQ.RotateVectorReverse(
    wQ.RotateVector(_q * _offset) - this->inertial->CoG());

  auto worldAngularVel =
    SiconosTypes::ConvertVector3( this->body->angularVelocity(true) );

  auto worldLinearVel =
    SiconosTypes::ConvertVector3( this->body->linearVelocity(true) );

  auto worldPointVel =
    worldLinearVel + worldAngularVel.Cross(offsetFromCoG);

  return worldPointVel;
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // Siconos TODO
  (*this->body->velocity())(3) = _vel.X();
  (*this->body->velocity())(4) = _vel.Y();
  (*this->body->velocity())(5) = _vel.Z();
  this->body->swapInMemory();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldAngularVel() const
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldAngularVel returns "
          << ignition::math::Vector3d(0, 0, 0) << " by default." << std::endl;
    return ignition::math::Vector3d(0, 0, 0);
  }

  return SiconosTypes::ConvertVector3(this->body->angularVelocity(true));
}

//////////////////////////////////////////////////
void SiconosLink::SetForce(const ignition::math::Vector3d &_force)
{
  if (!this->body)
    return;

  SiconosTypes::ConvertVector3(_force, this->force);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldForce() const
{
  if (!this->body)
    return ignition::math::Vector3d(0, 0, 0);

  return SiconosTypes::ConvertVector3(this->force);
}

//////////////////////////////////////////////////
void SiconosLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetTorque" << std::endl;
    return;
  }

  auto linkFrameTorque = this->WorldPose().Rot().RotateVectorReverse(_torque);
  SiconosTypes::ConvertVector3(linkFrameTorque, this->torque);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldTorque() const
{
  if (this->body)
    return this->WorldPose().Rot().RotateVector(
      SiconosTypes::ConvertVector3(this->torque));
  else
    return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
SP::BodyDS SiconosLink::GetSiconosBodyDS() const
{
  return this->body;
}

//////////////////////////////////////////////////
SP::SiconosContactorSet SiconosLink::GetSiconosContactorSet() const
{
  return this->contactorSet;
}

//////////////////////////////////////////////////
void SiconosLink::SetLinearDamping(double /*_damping*/)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearDamping"
          << std::endl;
    return;
  }
  // Siconos TODO
  // this->body->setDamping((btScalar)_damping,
  //     (btScalar)this->body->getAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularDamping(double /*_damping*/)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularDamping"
          << std::endl;
    return;
  }
  // Siconos TODO
  // this->body->setDamping(
  //     (btScalar)this->body->getLinearDamping(), (btScalar)_damping);
}

/////////////////////////////////////////////////
void SiconosLink::AddForce(const ignition::math::Vector3d &_force)
{
  if (this->body)
  {
    (*this->force)(0) += _force.X();
    (*this->force)(1) += _force.Y();
    (*this->force)(2) += _force.Z();

    this->SetEnabled(true);
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddForce"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeForce(const ignition::math::Vector3d &_force)
{
  if (this->body)
  {
    // Force vector represents a direction only, so it should be
    // rotated but not translated
    ignition::math::Vector3d forceWorld =
      this->WorldPose().Rot().RotateVector(_force);

    (*this->force)(0) += forceWorld.X();
    (*this->force)(1) += forceWorld.Y();
    (*this->force)(2) += forceWorld.Z();

    this->SetEnabled(true);
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddRelativeForce"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtWorldPosition(const ignition::math::Vector3d &_force,
                                          const ignition::math::Vector3d &_pos)
{
  if (this->body)
  {
    auto relpos = this->WorldPose().Rot().RotateVectorReverse(
      _pos - this->WorldPose().Pos());

    this->AddForceAtRelativePosition(_force, relpos);
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddForceAtWorldPosition"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtRelativePosition(const ignition::math::Vector3d &_force,
                                             const ignition::math::Vector3d &_relpos)
{
  if (this->body)
  {
    // In Siconos, force vector fExt is in world frame, torque moment
    // vector mExt is in body frame, since we don't set
    // NewtonEulerDS::isMextExpressedInInertialFrame.

    ignition::math::Vector3d worldFrameForce = _force;

    (*this->force)(0) += worldFrameForce.X();
    (*this->force)(1) += worldFrameForce.Y();
    (*this->force)(2) += worldFrameForce.Z();

    ignition::math::Vector3d linkFrameForce =
      this->WorldPose().Rot().RotateVectorReverse(_force);

    auto linkFrameMoment = _relpos - this->inertial->CoG();
    auto linkFrameTorque = linkFrameMoment.Cross(linkFrameForce);

    (*this->torque)(0) += linkFrameTorque.X();
    (*this->torque)(1) += linkFrameTorque.Y();
    (*this->torque)(2) += linkFrameTorque.Z();

    this->SetEnabled(true);
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddRelativeForce"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void SiconosLink::AddLinkForce(const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &_offset)
{
  if (this->body)
  {
    // Force vector represents a direction only, so it should be
    // rotated but not translated
    ignition::math::Vector3d worldFrameForce =
      this->WorldPose().Rot().RotateVector(_force);

    this->AddForceAtRelativePosition(worldFrameForce, _offset);
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddLinkForce"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void SiconosLink::AddTorque(const ignition::math::Vector3d &_torque)
{
  if (this->body)
  {
    auto linkFrameTorque = this->WorldCoGPose().Rot().RotateVectorReverse(_torque);
    (*this->torque)(0) += linkFrameTorque.X();
    (*this->torque)(1) += linkFrameTorque.Y();
    (*this->torque)(2) += linkFrameTorque.Z();
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddTorque"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeTorque(const ignition::math::Vector3d &_torque)
{
  if (this->body)
  {
    (*this->torque)(0) += _torque.X();
    (*this->torque)(1) += _torque.Y();
    (*this->torque)(2) += _torque.Z();
  }
  else if (!this->IsStatic())
  {
    gzlog << "Siconos body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddRelativeTorque"
          << std::endl;
  }
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

//////////////////////////////////////////////////
void SiconosLink::UpdatePoseFromBody()
{
  if (this->body) {
    this->dirtyPose = SiconosTypes::ConvertPose(this->body->q());

    ignition::math::Vector3d cog = this->dirtyPose.Rot().RotateVector(
      this->inertial->CoG());

    this->dirtyPose.Pos() -= cog;

    world->_AddDirty(this);
  }
}

//////////////////////////////////////////////////
void SiconosLink::UpdateSurface()
{
  // Update collision group for all collisions, since we don't know if
  // any have changed.

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SiconosCollisionPtr collision;
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      collision->UpdateCollisionGroup();
    }
  }
}

//////////////////////////////////////////////////
SiconosLinkWeakPtr SiconosLink::GetLinkForBody(const SP::BodyDS& ds)
{
  // This function exists to avoid exposing the private GzBodyDS
  // class defined above.
  GZ_ASSERT(std11::dynamic_pointer_cast<GzBodyDS>(ds), "BodyDS was not a GzBodyDS!");
  std11::shared_ptr<GzBodyDS> gzbody = std11::static_pointer_cast<GzBodyDS>(ds);
  return gzbody->link;
}
