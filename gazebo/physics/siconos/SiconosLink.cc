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

//////////////////////////////////////////////////
SiconosLink::SiconosLink(EntityPtr _parent)
    : Link(_parent)
    , contactorSet(new SiconosContactorSet())
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
  // Set the initial pose of the body
  // this->motionState.reset(new SiconosMotionState(
  //   boost::dynamic_pointer_cast<Link>(shared_from_this())));

  Link::Init();

  GZ_ASSERT(this->sdf != NULL, "Unable to initialize link, SDF is NULL");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));

  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  double mass = this->inertial->Mass();
  ignition::math::Vector3d cogVec = this->inertial->CoG();

  /// \todo FIXME:  Friction Parameters

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SiconosCollisionPtr collision;
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      SP::SiconosShape shape(collision->GetCollisionShape());
      if (shape) {
        SP::SiconosContactor c(new SiconosContactor(shape));
        this->contactorSet->append(c);
      } else
          GZ_ASSERT(shape, "Shape is invalid");

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

  // Initial position and velocity
  SP::SiconosVector q(new SiconosVector(7));
  SP::SiconosVector v(new SiconosVector(6));
  q->zero();
  v->zero();
  (*q)(3) = 1.0;

  ignition::math::Pose3d pose = this->WorldInertialPose();
  (*q)(0) = pose.Pos().X();
  (*q)(1) = pose.Pos().Y();
  (*q)(2) = pose.Pos().Z();
  (*q)(3) = pose.Rot().W();
  (*q)(4) = pose.Rot().X();
  (*q)(5) = pose.Rot().Y();
  (*q)(6) = pose.Rot().Z();

  if (this->IsStatic() || this->GetKinematic())
  {
      // If the object is static or kinematic, do not create a dynamical
      // system for it.
  }
  else
  {
      // Otherwise, create the dynamic system with given inertia and
      // initial state vectors
      this->body.reset(new BodyDS(q,v,mass));

      this->force.reset(new SiconosVector(3));
      this->weight.reset(new SiconosVector(3));
      this->weight->zero();
      this->weight->setValue(2, -mass * 9.81);

      *this->force = *this->weight;
      this->body->setFExtPtr(force);

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

    // Add DS to the integrator
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->topology()->setOSI(this->body, siconosWorld->GetOneStepIntegrator());

    // Initialize DS worksapce
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->topology()->initW(siconosWorld->GetSimulation()->nextTime(),
                          this->body, siconosWorld->GetOneStepIntegrator());

    /* Initialize the DS at the current time */
    this->body->initialize(siconosWorld->GetSimulation()->nextTime(),
                           siconosWorld->GetOneStepIntegrator()->getSizeMem());

    /* Simulation partial re-initialization */
    siconosWorld->GetSimulation()->initialize(siconosWorld->GetModel(), false);
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
    ignition::math::Vector3d g = this->siconosPhysics->World()->Gravity();
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

  // this->SetEnabled(true);

  const ignition::math::Pose3d myPose = this->WorldCoGPose();

  SiconosVector &q = *this->body->q();
  q(0) = myPose.Pos().X();
  q(1) = myPose.Pos().Y();
  q(2) = myPose.Pos().Z();
  q(3) = myPose.Rot().W();
  q(4) = myPose.Rot().X();
  q(5) = myPose.Rot().Y();
  q(6) = myPose.Rot().Z();
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
void SiconosLink::SetLinearVel(const ignition::math::Vector3d &/*_vel*/)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  // TODO Siconos
  // this->body->setLinearVelocity(SiconosTypes::ConvertVector3(_vel));
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

  // btVector3 vel = this->body->getLinearVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return ignition::math::Vector3d(0,0,0);
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

  ignition::math::Pose3d wPose = this->WorldPose();
  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  ignition::math::Vector3d offsetFromCoG = wPose.Rot()*(_offset - this->inertial->CoG());
  // Siconos TODO
  // btVector3 vel = this->body->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return ignition::math::Vector3d(0,0,0);
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

  ignition::math::Pose3d wPose = this->WorldPose();
  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  ignition::math::Vector3d offsetFromCoG = _q*_offset
        - wPose.Rot()*this->inertial->CoG();
  // Siconos TODO
  // btVector3 vel = this->body->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularVel(const ignition::math::Vector3d &/*_vel*/)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // Siconos TODO
  // this->body->setAngularVelocity(SiconosTypes::ConvertVector3(_vel));
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

  // Siconos TODO
  // btVector3 vel = this->body->getAngularVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetForce(const ignition::math::Vector3d &_force)
{
  if (!this->body)
    return;

  // Siconos TODO
  (*this->force)(0) = (*this->weight)(0) + _force.X();
  (*this->force)(1) = (*this->weight)(1) + _force.Y();
  (*this->force)(2) = (*this->weight)(2) + _force.Z();

  // this->body->applyCentralForce(
  //   btVector3(_force.x, _force.y, _force.z));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldForce() const
{
  if (!this->body)
    return ignition::math::Vector3d(0, 0, 0);

  // btVector3 btVec;

  // Siconos TODO
  // btVec = this->body->getTotalForce();

  // return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());
  return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetTorque(const ignition::math::Vector3d &/*_torque*/)
{
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // Siconos TODO
  // this->body->applyTorque(SiconosTypes::ConvertVector3(_torque));
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldTorque() const
{
  /*
  if (!this->body)
    return ignition::math::Vector3d(0, 0, 0);

  btignition::math::Vector3d btVec;

  // Siconos TODO
  btVec = this->body->getTotalTorque();

  return ignition::math::Vector3d(btVec.x(), btVec.y(), btVec.z());
  */
  return ignition::math::Vector3d();
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
  // Siconos TODO
  (*this->force)(0) = (*this->weight)(0) + _force.X();
  (*this->force)(1) = (*this->weight)(1) + _force.Y();
  (*this->force)(2) = (*this->weight)(2) + _force.Z();
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeForce(const ignition::math::Vector3d &/*_force*/)
{
  gzlog << "SiconosLink::AddRelativeForce not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtWorldPosition(const ignition::math::Vector3d &/*_force*/,
                                         const ignition::math::Vector3d &/*_pos*/)
{
  gzlog << "SiconosLink::AddForceAtWorldPosition not yet implemented."
        << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddForceAtRelativePosition(const ignition::math::Vector3d &/*_force*/,
                  const ignition::math::Vector3d &/*_relpos*/)
{
  gzlog << "SiconosLink::AddForceAtRelativePosition not yet implemented."
        << std::endl;
}

//////////////////////////////////////////////////
void SiconosLink::AddLinkForce(const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &/*_offset*/)
{
  // Siconos TODO
  AddForce(_force);
  gzlog << "SiconosLink::AddLinkForce not yet implemented correctly."
        << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddTorque(const ignition::math::Vector3d &/*_torque*/)
{
  gzlog << "SiconosLink::AddTorque not yet implemented." << std::endl;
}

/////////////////////////////////////////////////
void SiconosLink::AddRelativeTorque(const ignition::math::Vector3d &/*_torque*/)
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

//////////////////////////////////////////////////
void SiconosLink::UpdatePoseFromBody()
{
  // TODO static links have no body, position is only in contactor
  if (this->body) {
    SiconosVector &q = *this->body->q();
    this->dirtyPose.Pos().Set(q(0), q(1), q(2));
    this->dirtyPose.Rot().Set(q(3), q(4), q(5), q(6));
    world->_AddDirty(this);
  }
}
