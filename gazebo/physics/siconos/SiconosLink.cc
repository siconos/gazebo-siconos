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
      SP::SiconosContactor c(collision->GetSiconosContactor());
      if (c) {
        this->contactorSet->append(c);
      } else
        GZ_ASSERT(c, "Contactor is invalid");

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
  SP::SiconosVector q(SiconosTypes::ConvertPose(this->WorldInertialPose()));
  SP::SiconosVector v(std11::make_shared<SiconosVector>(6));
  SiconosTypes::ConvertVector3(this->WorldCoGLinearVel(), *v, 0);
  SiconosTypes::ConvertVector3(this->WorldAngularVel(), *v, 3);

  if (this->IsStatic() || this->GetKinematic())
  {
      // If the object is static or kinematic, do not create a dynamical
      // system for it.
  }
  else
  {
      // Otherwise, create the dynamic system with given inertia and
      // initial state vectors
      this->body = std11::make_shared<BodyDS>(q,v,mass);

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

    // Add DS to the integrator
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->topology()->setOSI(this->body, siconosWorld->GetOneStepIntegrator());

    // Initialize DS worksapce
    siconosWorld->GetModel()->nonSmoothDynamicalSystem()
      ->topology()->initDS(siconosWorld->GetModel(),
                           siconosWorld->GetSimulation()->nextTime(),
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

  SiconosTypes::ConvertPoseToVector7(myPose, this->body->q());
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

  return SiconosTypes::ConvertVector3( this->body->velocity() );
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

  const SiconosVector& v( *this->body->velocity() );
  return ignition::math::Vector3d(v(0),v(1),v(2));
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

  const SiconosVector& v( *this->body->velocity() );
  return ignition::math::Vector3d(v(3),v(4),v(5));
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
  if(_torque.SquaredLength() > 0.001)
    printf("SetTorque()\n");
  if (!this->body)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetTorque" << std::endl;
    return;
  }

  SiconosTypes::ConvertVector3(_torque, this->torque);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosLink::WorldTorque() const
{
  if (this->body)
    return SiconosTypes::ConvertVector3(this->torque);
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
  (*this->force)(0) += _force.X();
  (*this->force)(1) += _force.Y();
  (*this->force)(2) += _force.Z();
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
    const ignition::math::Vector3d &_offset)
{
  if (this->body)
  {
    // Force vector represents a direction only, so it should be
    // rotated but not translated
    ignition::math::Vector3d forceWorld =
      this->WorldPose().Rot().RotateVector(_force);

    // Siconos TODO
    ignition::math::Vector3d offsetCoG = _offset -
      this->inertial->CoG();

    (*this->force)(0) += forceWorld.X();
    (*this->force)(1) += forceWorld.Y();
    (*this->force)(2) += forceWorld.Z();

    this->SetEnabled(true);
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
    auto torque = this->WorldCoGPose().Rot().RotateVector(_torque);
    (*this->torque)(0) += torque.X();
    (*this->torque)(1) += torque.Y();
    (*this->torque)(2) += torque.Z();
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
    auto torque = this->WorldCoGPose().Rot().RotateVector(_torque);
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
    world->_AddDirty(this);
  }
}
