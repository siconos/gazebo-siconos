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

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosLink::SiconosLink(EntityPtr _parent)
    : Link(_parent)
{
  this->rigidLink = NULL;
  this->compoundDS = NULL;
}

//////////////////////////////////////////////////
SiconosLink::~SiconosLink()
{
  delete this->compoundDS;
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
  /// Currently, gazebo uses btCompoundShape to store multiple
  /// <collision> shapes in siconos.  Each child shape could have a
  /// different mu1 and mu2.  This is not ideal as friction is set
  /// per SiconosLink::rigidLink (btRigidBody : btCollisionObject).
  /// Right now, the friction coefficients for the last SiconosCollision
  /// processed in this link below is stored in hackMu1, hackMu2.
  /// The average is stored in this->rigidLink.
  /// Final friction coefficient is applied in ContactCallback
  /// by taking the lower of the 2 colliding rigidLink's.
  double hackMu1 = 0;
  double hackMu2 = 0;

  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      SiconosCollisionPtr collision;
      collision = boost::static_pointer_cast<SiconosCollision>(*iter);
      Interaction *interaction = collision->GetCollisionInteraction();

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
    }
  }

  // if there are no collisions in the link then use an empty shape
  // if (!this->compoundShape)
  //   this->compoundShape = new btEmptyShape();

  // this->compoundShape->calculateLocalInertia(mass, fallInertia);
  // fallInertia = SiconosTypes::ConvertVector3(
  //   this->inertial->GetPrincipalMoments());
  // TODO: inertia products not currently used
  // this->inertial->SetInertiaMatrix(fallInertia.x(), fallInertia.y(),
  //                                  fallInertia.z(), 0, 0, 0);

  // Create a construction info object
  // btRigidBody::btRigidBodyConstructionInfo
  //   rigidLinkCI(mass, this->motionState.get(), this->compoundShape,
  //   fallInertia);

  // rigidLinkCI.m_linearDamping = this->GetLinearDamping();
  // rigidLinkCI.m_angularDamping = this->GetAngularDamping();

  // // Create the new rigid body
  // this->rigidLink = new btRigidBody(rigidLinkCI);
  // this->rigidLink->setUserPointer(this);
  // this->rigidLink->setCollisionFlags(this->rigidLink->getCollisionFlags() |
  //     btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
  // this->rigidLink->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);

  // /// \TODO: get friction from collision object
  // this->rigidLink->setAnisotropicFriction(btVector3(1, 1, 1),
  //   btCollisionObject::CF_ANISOTROPIC_FRICTION);
  // this->rigidLink->setFriction(0.5*(hackMu1 + hackMu2));  // Hack

  // Setup motion clamping to prevent objects from moving too fast.
  // this->rigidLink->setCcdMotionThreshold(1);
  // math::Vector3 size = this->GetBoundingBox().GetSize();
  // this->rigidLink->setCcdSweptSphereRadius(size.GetMax()*0.8);

  // if (mass <= 0.0)
  //   this->rigidLink->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);

  SP::Model siconosWorld = this->siconosPhysics->GetDynamicsWorld();
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
  // siconosWorld->addRigidBody(this->rigidLink, categortyBits, collideBits);

  // Only use auto disable if no joints and no sensors are present
  // this->rigidLink->setActivationState(DISABLE_DEACTIVATION);
  // if (this->GetModel()->GetAutoDisable() &&
  //     this->GetModel()->GetJointCount() == 0 &&
  //     this->GetSensorCount() == 0)
  // {
  //   this->rigidLink->setActivationState(ACTIVE_TAG);
  //   this->rigidLink->setSleepingThresholds(0.1, 0.1);
  //   this->rigidLink->setDeactivationTime(1.0);
  // }

  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::Fini()
{
  Link::Fini();
  SP::Model siconosWorld = this->siconosPhysics->GetDynamicsWorld();
  GZ_ASSERT(siconosWorld != NULL, "Siconos dynamics world is NULL");
  // siconosWorld->removeRigidBody(this->rigidLink);
}

/////////////////////////////////////////////////////////////////////
void SiconosLink::UpdateMass()
{
  if (this->rigidLink && this->inertial)
  {
    // this->rigidLink->setMassProps(this->inertial->GetMass(),
    //     SiconosTypes::ConvertVector3(this->inertial->GetPrincipalMoments()));
  }
}

//////////////////////////////////////////////////
void SiconosLink::SetGravityMode(bool _mode)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetGravityMode" << std::endl;
    return;
  }

  if (_mode == false)
    // this->rigidLink->setGravity(btVector3(0, 0, 0));
    // this->rigidLink->setMassProps(btScalar(0), btmath::Vector3(0, 0, 0));
    ;
  else
  {
    math::Vector3 g = this->siconosPhysics->GetGravity();
    // this->rigidLink->setGravity(btVector3(g.x, g.y, g.z));
    /*btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0, 0, 0);

    this->compoundShape->calculateLocalInertia(btMass, fallInertia);
    this->rigidLink->setMassProps(btMass, fallInertia);
    */
  }
}

//////////////////////////////////////////////////
bool SiconosLink::GetGravityMode() const
{
  bool result = false;
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetGravityMode returns "
          << result << " by default." << std::endl;
    return result;
  }
  // btVector3 g = this->rigidLink->getGravity();
  // result = !math::equal(static_cast<double>(g.length()), 0.0);

  return result;
}

//////////////////////////////////////////////////
void SiconosLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);
}

//////////////////////////////////////////////////
/*void SiconosLink::AttachCollision(Collision *_collision)
{
  Link::AttachCollision(_collision);

  SiconosCollision *bcollision = dynamic_cast<SiconosCollision*>(_collision);

  if (_collision == NULL)
    gzthrow("requires SiconosCollision");

  btTransform trans;
  math::Pose relativePose = _collision->GetRelativePose();
  trans = SiconosTypes::ConvertPose(relativePose);

  bcollision->SetCompoundShapeIndex(this->compoundShape->getNumChildShapes());
  this->compoundShape->addChildShape(trans, bcollision->GetCollisionShape());
}
  */

//////////////////////////////////////////////////
/// Adapted from ODELink::OnPoseChange
void SiconosLink::OnPoseChange()
{
  Link::OnPoseChange();

  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to respond to OnPoseChange"
          << std::endl;
    return;
  }

  // this->SetEnabled(true);

  const math::Pose myPose = this->GetWorldCoGPose();

  // this->rigidLink->setCenterOfMassTransform(
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
  /*
  if (!this->rigidLink)
    return;

  if (_enable)
    this->rigidLink->activate(true);
  else
    this->rigidLink->setActivationState(WANTS_DEACTIVATION);
    */
}

//////////////////////////////////////////////////
void SiconosLink::SetLinearVel(const math::Vector3 &_vel)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearVel" << std::endl;
    return;
  }

  // this->rigidLink->setLinearVelocity(SiconosTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldCoGLinearVel() const
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  // btVector3 vel = this->rigidLink->getLinearVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldLinearVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  math::Pose wPose = this->GetWorldPose();
  GZ_ASSERT(this->inertial != NULL, "Inertial pointer is NULL");
  math::Vector3 offsetFromCoG = wPose.rot*(_offset - this->inertial->GetCoG());
  // btVector3 vel = this->rigidLink->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldLinearVel(const math::Vector3 &_offset,
                                            const math::Quaternion &_q) const
{
  if (!this->rigidLink)
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
  // btVector3 vel = this->rigidLink->getVelocityInLocalPoint(
  //     SiconosTypes::ConvertVector3(offsetFromCoG));

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularVel(const math::Vector3 &_vel)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // this->rigidLink->setAngularVelocity(SiconosTypes::ConvertVector3(_vel));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldAngularVel() const
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, GetWorldAngularVel returns "
          << math::Vector3(0, 0, 0) << " by default." << std::endl;
    return math::Vector3(0, 0, 0);
  }

  // btVector3 vel = this->rigidLink->getAngularVelocity();

  // return SiconosTypes::ConvertVector3(vel);
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetForce(const math::Vector3 &_force)
{
  if (!this->rigidLink)
    return;

  // this->rigidLink->applyCentralForce(
  //   btVector3(_force.x, _force.y, _force.z));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldForce() const
{
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  // btVector3 btVec;

  // btVec = this->rigidLink->getTotalForce();

  // return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  return math::Vector3(0,0,0);
}

//////////////////////////////////////////////////
void SiconosLink::SetTorque(const math::Vector3 &_torque)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularVel" << std::endl;
    return;
  }

  // this->rigidLink->applyTorque(SiconosTypes::ConvertVector3(_torque));
}

//////////////////////////////////////////////////
math::Vector3 SiconosLink::GetWorldTorque() const
{
  /*
  if (!this->rigidLink)
    return math::Vector3(0, 0, 0);

  btmath::Vector3 btVec;

  btVec = this->rigidLink->getTotalTorque();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
  */
  return math::Vector3();
}

//////////////////////////////////////////////////
DynamicalSystem *SiconosLink::GetSiconosLink() const
{
  return this->rigidLink;
}

//////////////////////////////////////////////////
void SiconosLink::ClearCollisionCache()
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to ClearCollisionCache" << std::endl;
    return;
  }

  SP::Model siconosWorld = this->siconosPhysics->GetDynamicsWorld();
  GZ_ASSERT(siconosWorld != NULL, "Siconos dynamics world is NULL");

  // siconosWorld->updateSingleAabb(this->rigidLink);
  // siconosWorld->getBroadphase()->getOverlappingPairCache()->
  //     cleanProxyFromPairs(this->rigidLink->getBroadphaseHandle(),
  //     siconosWorld->getDispatcher());
}

//////////////////////////////////////////////////
void SiconosLink::SetLinearDamping(double _damping)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetLinearDamping"
          << std::endl;
    return;
  }
  // this->rigidLink->setDamping((btScalar)_damping,
  //     (btScalar)this->rigidLink->getAngularDamping());
}

//////////////////////////////////////////////////
void SiconosLink::SetAngularDamping(double _damping)
{
  if (!this->rigidLink)
  {
    gzlog << "Siconos rigid body for link [" << this->GetName() << "]"
          << " does not exist, unable to SetAngularDamping"
          << std::endl;
    return;
  }
  // this->rigidLink->setDamping(
  //     (btScalar)this->rigidLink->getLinearDamping(), (btScalar)_damping);
}

//////////////////////////////////////////////////
/*void SiconosLink::SetCollisionRelativePose(SiconosCollision *_collision,
    const math::Pose &_newPose)
{
  std::map<std::string, Collision*>::iterator iter;
  unsigned int i;

  for (iter = this->collisions.begin(), i = 0; iter != this->collisions.end();
       ++iter, ++i)
  {
    if (iter->second == _collision)
      break;
  }

  if (i < this->collisions.size())
  {
    // Set the pose of the _collision in Siconos
    this->compoundShape->updateChildTransform(i,
        SiconosTypes::ConvertPose(_newPose));
  }
}*/

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
