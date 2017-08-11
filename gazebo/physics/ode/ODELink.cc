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
#include <math.h>
#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODELink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODELink::ODELink(EntityPtr _parent)
    : Link(_parent)
{
  this->linkId = nullptr;
}

//////////////////////////////////////////////////
ODELink::~ODELink()
{
  this->Fini();
}

//////////////////////////////////////////////////
void ODELink::Load(sdf::ElementPtr _sdf)
{
  this->odePhysics = boost::dynamic_pointer_cast<ODEPhysics>(
      this->GetWorld()->Physics());

  if (this->odePhysics == nullptr)
    gzthrow("Not using the ode physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void ODELink::Init()
{
  if (!this->IsStatic())
  {
    this->linkId = dBodyCreate(this->odePhysics->GetWorldId());
    dBodySetData(this->linkId, this);

    // Only use auto disable if no joints and no sensors are present
    if (this->GetModel()->GetAutoDisable() &&
        this->GetModel()->GetJointCount() == 0 &&
        this->GetSensorCount() == 0)
    {
      dBodySetAutoDisableDefaults(this->linkId);
      dBodySetAutoDisableFlag(this->linkId, 1);
    }
    else
    {
      dBodySetAutoDisableFlag(this->linkId, 0);
    }
  }

  GZ_ASSERT(this->sdf != nullptr, "Unable to initialize link, SDF is null");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());

  Link::Init();

  if (this->linkId)
  {
    GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
    ignition::math::Vector3d cogVec = this->inertial->CoG();
    for (auto const &child : this->children)
    {
      if (child->HasType(Base::COLLISION))
      {
        ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
        if (g->IsPlaceable() && g->GetCollisionId())
        {
          dGeomSetBody(g->GetCollisionId(), this->linkId);
        }
      }
    }
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to place collision bodies"
          << " in ODELink::Init" << std::endl;
  }

  // Update the Collision Offsets, Surface, and Center of Mass.
  this->UpdateCollisionOffsets();
  this->UpdateSurface();
  // Only update the Center of Mass if object is dynamic
  if (!this->GetKinematic())
    this->UpdateMass();

  if (this->linkId)
  {
    dBodySetMovedCallback(this->linkId, MoveCallback);
    dBodySetDisabledCallback(this->linkId, DisabledCallback);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to set callbacks in ODELink::Init"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void ODELink::DisabledCallback(dBodyID /*_id*/)
{
}

//////////////////////////////////////////////////
void ODELink::MoveCallback(dBodyID _id)
{
  const dReal *p;
  const dReal *r;
  ODELink *self = static_cast<ODELink*>(dBodyGetData(_id));
  // self->poseMutex->lock();

  p = dBodyGetPosition(_id);
  r = dBodyGetQuaternion(_id);

  self->dirtyPose.Pos().Set(p[0], p[1], p[2]);
  self->dirtyPose.Rot().Set(r[0], r[1], r[2], r[3]);

  // subtracting cog location from ode pose
  GZ_ASSERT(self->inertial != nullptr, "Inertial pointer is null");
  ignition::math::Vector3d cog = self->dirtyPose.Rot().RotateVector(
      self->inertial->CoG());

  self->dirtyPose.Pos() -= cog;

  // Tell the world that our pose has changed.
  self->world->_AddDirty(self);

  // self->poseMutex->unlock();

  // get force and applied to this body
  if (_id)
  {
    const dReal *dforce = dBodyGetForce(_id);
    self->force.Set(dforce[0], dforce[1], dforce[2]);

    const dReal *dtorque = dBodyGetTorque(_id);
    self->torque.Set(dtorque[0], dtorque[1], dtorque[2]);
  }
}

//////////////////////////////////////////////////
void ODELink::Fini()
{
  if (this->linkId)
    dBodyDestroy(this->linkId);
  this->linkId = nullptr;

  this->odePhysics.reset();

  Link::Fini();
}

//////////////////////////////////////////////////
void ODELink::SetGravityMode(bool _mode)
{
  if (this->sdf->HasElement("gravity"))
    this->sdf->GetElement("gravity")->Set(_mode);
  if (this->linkId)
  {
    dBodySetGravityMode(this->linkId, _mode ? 1: 0);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
  }
}

//////////////////////////////////////////////////
bool ODELink::GetGravityMode() const
{
  int mode = 0;
  if (this->linkId)
  {
    mode = dBodyGetGravityMode(this->linkId);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetGravityMode returns default of "
          << mode << std::endl;
  }

  return mode;
}

//////////////////////////////////////////////////
void ODELink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
  if (_collide)
    this->spaceId = dSimpleSpaceCreate(this->odePhysics->GetSpaceId());
}

//////////////////////////////////////////////////
void ODELink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->linkId)
  {
    if (!this->IsStatic() && this->initialized)
      gzlog << "ODE body for link [" << this->GetScopedName() << "]"
            << " does not exist, unable to respond to OnPoseChange"
            << std::endl;
    return;
  }

  this->SetEnabled(true);

  const ignition::math::Pose3d myPose = this->WorldPose();

  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
  ignition::math::Vector3d cog =
    myPose.Rot().RotateVector(this->inertial->CoG());

  // adding cog location for ode pose
  dBodySetPosition(this->linkId,
      myPose.Pos().X() + cog.X(),
      myPose.Pos().Y() + cog.Y(),
      myPose.Pos().Z() + cog.Z());

  dQuaternion q;
  q[0] = myPose.Rot().W();
  q[1] = myPose.Rot().X();
  q[2] = myPose.Rot().Y();
  q[3] = myPose.Rot().Z();

  // Set the rotation of the ODE link
  dBodySetQuaternion(this->linkId, q);
}

//////////////////////////////////////////////////
dBodyID ODELink::GetODEId() const
{
  return this->linkId;
}


//////////////////////////////////////////////////
void ODELink::SetEnabled(bool _enable) const
{
  if (!this->linkId)
  {
    if (!this->IsStatic() && this->initialized)
      gzlog << "ODE body for link [" << this->GetScopedName() << "]"
            << " does not exist, unable to SetEnabled" << std::endl;
    return;
  }

  if (_enable)
    dBodyEnable(this->linkId);
  else
    dBodyDisable(this->linkId);
}

/////////////////////////////////////////////////////////////////////
bool ODELink::GetEnabled() const
{
  bool result = true;

  if (this->linkId)
    result = dBodyIsEnabled(this->linkId);
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetEnabled returns default of "
          << result << std::endl;
  }

  return result;
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateCollisionOffsets()
{
  if (this->linkId)
  {
    GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
    ignition::math::Vector3d cogVec = this->inertial->CoG();
    for (auto const &child : this->children)
    {
      if (child->HasType(Base::COLLISION))
      {
        ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
        if (g->IsPlaceable() && g->GetCollisionId())
        {
          // update pose immediately
          ignition::math::Pose3d localPose = g->RelativePose();
          localPose.Pos() -= cogVec;

          dQuaternion q;
          q[0] = localPose.Rot().W();
          q[1] = localPose.Rot().X();
          q[2] = localPose.Rot().Y();
          q[3] = localPose.Rot().Z();

          // Set the pose of the encapsulated collision; this is always relative
          // to the CoM
          dGeomSetOffsetPosition(g->GetCollisionId(),
              localPose.Pos().X(), localPose.Pos().Y(), localPose.Pos().Z());
          dGeomSetOffsetQuaternion(g->GetCollisionId(), q);
        }
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateSurface()
{
  if (!this->linkId)
  {
    return;
  }

  for (auto const &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      ODECollisionPtr g = boost::static_pointer_cast<ODECollision>(child);
      if (g->IsPlaceable() && g->GetCollisionId())
      {
        // Set max_vel and min_depth
        boost::any value;
        if (g->GetODESurface()->maxVel < 0 && this->GetWorld()->
            Physics()->GetParam("contact_max_correcting_vel", value))
        {
          try
          {
            g->GetODESurface()->maxVel = boost::any_cast<double>(value);
          }
          catch(boost::bad_any_cast &_e)
          {
            gzerr << "Failed boost::any_cast in ODELink.cc: " << _e.what();
          }
        }
        // Set surface properties max_vel and min_depth
        dBodySetMaxVel(this->linkId, g->GetODESurface()->maxVel);
        dBodySetMinDepth(this->linkId, g->GetODESurface()->minDepth);
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////
void ODELink::UpdateMass()
{
  if (!this->linkId)
  {
    if (!this->IsStatic() && this->initialized)
      gzlog << "ODE body for link [" << this->GetScopedName() << "]"
            << " does not exist, unable to UpdateMass" << std::endl;
    return;
  }

  dMass odeMass;
  dMassSetZero(&odeMass);

  // The CoG must always be (0, 0, 0)
  ignition::math::Vector3d cog(0, 0, 0);

  GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
  // give ODE un-rotated inertia
  auto moi = this->inertial->MOI(
    ignition::math::Pose3d(this->inertial->CoG(),
                           ignition::math::Quaterniond::Identity));
  ignition::math::Vector3d principals(moi(0, 0), moi(1, 1), moi(2, 2));
  ignition::math::Vector3d products(moi(0, 1), moi(0, 2), moi(1, 2));

  dMassSetParameters(&odeMass, this->inertial->Mass(),
      cog.X(), cog.Y(), cog.Z(),
      principals.X(), principals.Y(), principals.Z(),
      products.X(), products.Y(), products.Z());

  if (this->inertial->Mass() > 0)
    dBodySetMass(this->linkId, &odeMass);
  else
    gzthrow("Setting custom link " + this->GetScopedName() + "mass to zero!");

  // In case the center of mass changed:
  this->UpdateCollisionOffsets();
  this->OnPoseChange();
}

//////////////////////////////////////////////////
void ODELink::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  if (this->linkId)
  {
    dBodySetLinearVel(this->linkId, _vel.X(), _vel.Y(), _vel.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldLinearVel(
    const ignition::math::Vector3d &_offset) const
{
  ignition::math::Vector3d vel;

  if (this->linkId)
  {
    dVector3 dvel;
    GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
    ignition::math::Vector3d offsetFromCoG = _offset -
      this->inertial->CoG();
    dBodyGetRelPointVel(this->linkId, offsetFromCoG.X(), offsetFromCoG.Y(),
        offsetFromCoG.Z(), dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetWorldLinearVel returns default of "
          << vel << std::endl;
  }
  return vel;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldLinearVel(
    const ignition::math::Vector3d &_offset,
    const ignition::math::Quaterniond &_q) const
{
  ignition::math::Vector3d vel;

  if (this->linkId)
  {
    dVector3 dvel;
    ignition::math::Pose3d wPose = this->WorldPose();
    GZ_ASSERT(this->inertial != nullptr, "Inertial pointer is null");
    ignition::math::Vector3d offsetFromCoG =
        wPose.Rot().RotateVectorReverse(_q * _offset)
        - this->inertial->CoG();
    dBodyGetRelPointVel(this->linkId, offsetFromCoG.X(), offsetFromCoG.Y(),
        offsetFromCoG.Z(), dvel);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetWorldLinearVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldCoGLinearVel() const
{
  ignition::math::Vector3d vel;

  if (this->linkId)
  {
    const dReal *dvel;
    dvel = dBodyGetLinearVel(this->linkId);
    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetWorldCoGLinearVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  if (this->linkId)
  {
    dBodySetAngularVel(this->linkId, _vel.X(), _vel.Y(), _vel.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldAngularVel() const
{
  ignition::math::Vector3d vel;

  if (this->linkId)
  {
    const dReal *dvel;

    dvel = dBodyGetAngularVel(this->linkId);

    vel.Set(dvel[0], dvel[1], dvel[2]);
  }
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetWorldAngularVel returns default of "
          << vel << std::endl;
  }

  return vel;
}

//////////////////////////////////////////////////
void ODELink::SetForce(const ignition::math::Vector3d &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodySetForce(this->linkId, _force.X(), _force.Y(), _force.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetForce" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetTorque(const ignition::math::Vector3d &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodySetTorque(this->linkId, _torque.X(), _torque.Y(), _torque.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetTorque" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::AddForce(const ignition::math::Vector3d &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForce(this->linkId, _force.X(), _force.Y(), _force.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddForce" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddRelativeForce(const ignition::math::Vector3d &_force)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelForce(this->linkId, _force.X(), _force.Y(), _force.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddRelativeForce" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddForceAtRelativePosition(
    const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &_relpos)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->linkId, _force.X(), _force.Y(), _force.Z(),
                          _relpos.X(), _relpos.Y(), _relpos.Z());
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddForceAtRelativePosition"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void ODELink::AddForceAtWorldPosition(
    const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &_pos)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddForceAtPos(this->linkId, _force.X(), _force.Y(), _force.Z(),
                          _pos.X(), _pos.Y(), _pos.Z());
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddForceAtWorldPosition"
          << std::endl;
  }
}

//////////////////////////////////////////////////
void ODELink::AddLinkForce(
    const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &_offset)
{
  if (this->linkId)
  {
    // Force vector represents a direction only, so it should be rotated but
    // not translated
    ignition::math::Vector3d forceWorld =
      this->WorldPose().Rot().RotateVector(_force);

    // Does this need to be rotated?
    ignition::math::Vector3d offsetCoG = _offset -
      this->inertial->CoG();

    this->SetEnabled(true);
    dBodyAddForceAtRelPos(this->linkId,
        forceWorld.X(), forceWorld.Y(), forceWorld.Z(),
        offsetCoG.X(), offsetCoG.Y(), offsetCoG.Z());
  }
  else if (!this->IsStatic())
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddLinkForce"
          << std::endl;
  }
}

/////////////////////////////////////////////////
void ODELink::AddTorque(const ignition::math::Vector3d &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddTorque(this->linkId, _torque.X(), _torque.Y(), _torque.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddTorque" << std::endl;
}

/////////////////////////////////////////////////
void ODELink::AddRelativeTorque(const ignition::math::Vector3d &_torque)
{
  if (this->linkId)
  {
    this->SetEnabled(true);
    dBodyAddRelTorque(this->linkId, _torque.X(), _torque.Y(), _torque.Z());
  }
  else if (!this->IsStatic())
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to AddRelativeTorque" << std::endl;
}

/////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldForce() const
{
  return this->force;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODELink::WorldTorque() const
{
  return this->torque;
}

//////////////////////////////////////////////////
dSpaceID ODELink::GetSpaceId() const
{
  return this->spaceId;
}

//////////////////////////////////////////////////
void ODELink::SetSpaceId(dSpaceID _spaceid)
{
  this->spaceId = _spaceid;
}

//////////////////////////////////////////////////
void ODELink::SetLinearDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetLinearDamping(this->GetODEId(), _damping);
  else if (!this->IsStatic() && this->initialized)
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetLinearDamping" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetAngularDamping(double _damping)
{
  if (this->GetODEId())
    dBodySetAngularDamping(this->GetODEId(), _damping);
  else if (!this->IsStatic() && this->initialized)
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetAngularDamping" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetKinematic(const bool &_state)
{
  this->sdf->GetElement("kinematic")->Set(_state);
  if (this->linkId)
  {
    if (_state && !dBodyIsKinematic(this->linkId))
      dBodySetKinematic(this->linkId);
    else if (dBodyIsKinematic(this->linkId))
      dBodySetDynamic(this->linkId);
  }
  else if (!this->IsStatic() && this->initialized)
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetKinematic" << std::endl;
}

//////////////////////////////////////////////////
bool ODELink::GetKinematic() const
{
  bool result = false;

  if (this->linkId)
    result = dBodyIsKinematic(this->linkId);
  else if (!this->IsStatic() && this->initialized)
  {
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, GetKinematic returns default of "
          << result << std::endl;
  }

  return result;
}

//////////////////////////////////////////////////
void ODELink::SetAutoDisable(bool _disable)
{
  if (this->GetModel()->GetJointCount() == 0 && this->linkId)
  {
    dBodySetAutoDisableFlag(this->linkId, _disable);
  }
  else if (!this->linkId)
    gzlog << "ODE body for link [" << this->GetScopedName() << "]"
          << " does not exist, unable to SetAutoDisable" << std::endl;
  else
    gzlog << "ODE model has joints, unable to SetAutoDisable" << std::endl;
}

//////////////////////////////////////////////////
void ODELink::SetLinkStatic(bool /*_static*/)
{
  gzlog << "To be implemented\n";
}
