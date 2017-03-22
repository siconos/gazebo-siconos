/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <functional>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/physics/dart/DARTSurfaceParams.hh"

#include "gazebo/physics/dart/DARTLinkPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTLink::DARTLink(EntityPtr _parent)
  : Link(_parent), dataPtr(new DARTLinkPrivate())
{
}

//////////////////////////////////////////////////
DARTLink::~DARTLink()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTLink::Load(sdf::ElementPtr _sdf)
{
  Link::Load(_sdf);

  // Name
  std::string bodyName = this->GetName();

  this->dataPtr->dartPhysics = boost::dynamic_pointer_cast<DARTPhysics>(
      this->GetWorld()->Physics());

  if (this->dataPtr->dartPhysics == nullptr)
    gzthrow("Not using the dart physics engine");

  // Check if soft_contact element is contained in this link. If so,
  // SoftBodyNode will be created. Otherwise, BodyNode will be created.
  sdf::ElementPtr dartElem;
  sdf::ElementPtr softCollElem;
  sdf::ElementPtr softGeomElem;

  if (_sdf->HasElement("collision"))
  {
    sdf::ElementPtr collElem = _sdf->GetElement("collision");
    while (collElem)
    {
      // Geometry
      sdf::ElementPtr geomElem = collElem->GetElement("geometry");

      // Surface
      if (collElem->HasElement("surface"))
      {
        sdf::ElementPtr surfaceElem = collElem->GetElement("surface");

        // Soft contact
        if (surfaceElem->HasElement("soft_contact"))
        {
          sdf::ElementPtr softContactElem
              = surfaceElem->GetElement("soft_contact");

          if (softContactElem->HasElement("dart"))
          {
            if (dartElem != nullptr)
            {
              gzerr << "DART supports only one deformable body in a link.\n";
              break;
            }

            dartElem = softContactElem->GetElement("dart");
            softCollElem = collElem;
            softGeomElem = geomElem;
          }
        }
      }

      collElem = collElem->GetNextElement("collision");
    }
  }

  if (dartElem != nullptr)
  {
    dart::dynamics::SoftBodyNode::UniqueProperties softProperties;

    // Mass
    double fleshMassFraction = dartElem->Get<double>("flesh_mass_fraction");

    // bone_attachment (Kv)
    double boneAttachment = DART_DEFAULT_VERTEX_STIFFNESS;
    if (dartElem->HasElement("bone_attachment"))
      boneAttachment = dartElem->Get<double>("bone_attachment");

    // stiffness (Ke)
    double stiffness = DART_DEFAULT_EDGE_STIFNESS;
    if (dartElem->HasElement("stiffness"))
      stiffness = dartElem->Get<double>("stiffness");

    // damping
    double damping = DART_DEFAULT_DAMPING_COEFF;
    if (dartElem->HasElement("damping"))
      damping = dartElem->Get<double>("damping");

    // pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    gzdbg << "pose" << T.matrix() << std::endl;
    if (softCollElem->HasElement("pose"))
    {
      T = DARTTypes::ConvPose(
          softCollElem->Get<ignition::math::Pose3d>("pose"));
    }

    // geometry
    if (softGeomElem->HasElement("box"))
    {
      sdf::ElementPtr boxEle = softGeomElem->GetElement("box");
      Eigen::Vector3d size =
          DARTTypes::ConvVec3(boxEle->Get<ignition::math::Vector3d>("size"));
      softProperties = dart::dynamics::SoftBodyNodeHelper::makeBoxProperties(
            size, T, fleshMassFraction, boneAttachment, stiffness, damping);
    }
//    else if (geomElem->HasElement("ellipsoid"))
//    {
//      sdf::ElementPtr ellipsoidEle = geomElem->GetElement("ellipsoid");
//      Eigen::Vector3d size
//          = DARTTypes::ConvVec3(ellipsoidEle->Get<
//          ignition::math::Vector3d>("size"));
//      double nSlices = ellipsoidEle->Get<double>("num_slices");
//      double nStacks = ellipsoidEle->Get<double>("num_stacks");
//      dart::dynamics::SoftBodyNodeHelper::setEllipsoid(
//            dtSoftBodyNode, size, nSlices, nStacks, fleshMassFraction);
//      dtSoftBodyNode->addCollisionShape(
//            new dart::dynamics::SoftMeshShape(dtSoftBodyNode));
//    }
    else
    {
      gzerr << "Unknown soft shape" << std::endl;
    }

    // Create DART SoftBodyNode properties
    dart::dynamics::BodyNode::Properties properties(bodyName);
    this->dataPtr->dtProperties.reset(
          new dart::dynamics::SoftBodyNode::Properties(
            properties, softProperties));

    this->dataPtr->isSoftBody;
  }
  else
  {
    // Create DART BodyNode properties
    this->dataPtr->dtProperties.reset(
          new dart::dynamics::BodyNode::Properties(bodyName));
  }

  for (auto child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      DARTCollisionPtr dartCollision =
          boost::static_pointer_cast<DARTCollision>(child);
      this->dataPtr->dtProperties->mColShapes.push_back(
            dartCollision->DARTCollisionShape());
    }
  }
}

//////////////////////////////////////////////////
void DARTLink::Init()
{
  Link::Init();

  this->dataPtr->Initialize();

  // DARTModel::Load() should be called first
  GZ_ASSERT(this->dataPtr->dtBodyNode != nullptr,
            "DART BodyNode is not initialized.");

  // Name
  std::string bodyName = this->GetName();
  this->dataPtr->dtBodyNode->setName(bodyName);

  // Mass
  double mass = this->inertial->Mass();
  this->dataPtr->dtBodyNode->setMass(mass);

  // Inertia
  double Ixx = this->inertial->IXX();
  double Iyy = this->inertial->IYY();
  double Izz = this->inertial->IZZ();
  double Ixy = this->inertial->IXY();
  double Ixz = this->inertial->IXZ();
  double Iyz = this->inertial->IYZ();
  this->dataPtr->dtBodyNode->setMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Visual
  this->visuals;

  // COG offset
  ignition::math::Vector3d cog = this->inertial->CoG();
  this->dataPtr->dtBodyNode->setLocalCOM(DARTTypes::ConvVec3(cog));

  // Gravity mode
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  // Friction coefficient

  /// \todo FIXME: Friction Parameters
  /// Gazebo allows different friction parameters per collision objects,
  /// while DART stores the friction parameter per link (BodyNode in DART). For
  /// now, the average friction parameter of all the child collision objects is
  /// stored in this->dataPtr->dtBodyNode.
  /// Final friction coefficient is applied in DART's constraint solver by
  /// taking the lower of the 2 colliding rigidLink's.
  /// See also:
  /// - https://github.com/dartsim/dart/issues/141
  /// - https://github.com/dartsim/dart/issues/266

  double hackAvgMu1 = 0;
  double hackAvgMu2 = 0;
  int numCollisions = 0;

  for (auto const &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      CollisionPtr collision =
          boost::static_pointer_cast<Collision>(child);

      SurfaceParamsPtr surface = collision->GetSurface();
      GZ_ASSERT(surface, "Surface pointer for is invalid");
      FrictionPyramidPtr friction = surface->FrictionPyramid();
      GZ_ASSERT(friction, "Friction pointer for is invalid");

      numCollisions++;
      hackAvgMu1 += friction->MuPrimary();
      hackAvgMu2 += friction->MuSecondary();
    }
  }

  hackAvgMu1 /= static_cast<double>(numCollisions);
  hackAvgMu2 /= static_cast<double>(numCollisions);

  this->dataPtr->dtBodyNode->setFrictionCoeff(0.5 * (hackAvgMu1 + hackAvgMu2));

  // We don't add dart body node to the skeleton here because dart body node
  // should be set its parent joint before being added. This body node will be
  // added to the skeleton in DARTModel::Init().
}

//////////////////////////////////////////////////
void DARTLink::Fini()
{
  Link::Fini();
}

/////////////////////////////////////////////////////////////////////
void DARTLink::UpdateMass()
{
  if (this->dataPtr->dtBodyNode && this->inertial)
  {
    this->dataPtr->dtBodyNode->setMass(this->inertial->Mass());
    auto Ixxyyzz = this->inertial->PrincipalMoments();
    auto Ixyxzyz = this->inertial->ProductsOfInertia();
    this->dataPtr->dtBodyNode->setMomentOfInertia(
        Ixxyyzz[0], Ixxyyzz[1], Ixxyyzz[2],
        Ixyxzyz[0], Ixyxzyz[1], Ixyxzyz[2]);
    auto cog = DARTTypes::ConvVec3(this->inertial->CoG());
    this->dataPtr->dtBodyNode->setLocalCOM(cog);
  }
}

//////////////////////////////////////////////////
void DARTLink::OnPoseChange()
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache("DARTLink::OnPoseChange",
                         std::bind(&DARTLink::OnPoseChange, this));
    return;
  }

  Link::OnPoseChange();

  // DART body node always have its parent joint.
  dart::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  dart::dynamics::FreeJoint *freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(joint);
  if (freeJoint)
  {
    // If the parent joint is free joint, set the 6 dof to fit the target pose.
    const Eigen::Isometry3d &W = DARTTypes::ConvPose(this->WorldPose());
    const Eigen::Isometry3d &T1 = joint->getTransformFromParentBodyNode();
    const Eigen::Isometry3d &InvT2 = joint->getTransformFromChildBodyNode();
    Eigen::Isometry3d P = Eigen::Isometry3d::Identity();

    if (this->dataPtr->dtBodyNode->getParentBodyNode())
      P = this->dataPtr->dtBodyNode->getParentBodyNode()->getTransform();

    Eigen::Isometry3d Q = T1.inverse() * P.inverse() * W * InvT2;

    // Convert homogeneous transformation matrix to 6-dimensional generalized
    // coordinates. There are several ways of conversions. Here is the way of
    // DART. The orientation part is converted by using logarithm map, which
    // maps SO(3) to so(3), and it takes the first three components of the
    // generalized coordinates. On the other hand, the position part just takes
    // the last three components of the generalized coordinates without any
    // conversion.
    freeJoint->setPositions(dart::dynamics::FreeJoint::convertToPositions(Q));
  }
  else
  {
    gzdbg << "OnPoseChange() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void DARTLink::SetEnabled(bool /*_enable*/) const
{
  // TODO: DART does not support this functionality.
}

//////////////////////////////////////////////////
bool DARTLink::GetEnabled() const
{
  // TODO: DART does not support this functionality.
  return true;
}

//////////////////////////////////////////////////
void DARTLink::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache("WorldLinearVel",
                         std::bind(&DARTLink::SetLinearVel, this, _vel),
                         _vel);
    return;
  }

  // DART body node always have its parent joint.
  dart::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  // Check if the parent joint is free joint
  dart::dynamics::FreeJoint *freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(joint);

  // If the parent joint is free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    // Generalized velocities
    Eigen::Vector3d genVel = DARTTypes::ConvVec3(_vel);

    dart::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getLocalTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentLinVel =
          dtBodyNode->getParentBodyNode()->getLinearVelocity();
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedLinVel =
          T.linear().transpose() *
          (parentAngVel.cross(T.translation()) + parentLinVel);

      // Subtract the effect
      genVel -= propagatedLinVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(3, genVel[0]);
    freeJoint->setVelocity(4, genVel[1]);
    freeJoint->setVelocity(5, genVel[2]);
  }
  else
  {
    gzdbg << "DARTLink::SetLinearVel() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache("WorldAngularVel",
                         std::bind(&DARTLink::SetAngularVel, this, _vel),
                         _vel);
    return;
  }

  // DART body node always have its parent joint.
  dart::dynamics::Joint *joint = this->dataPtr->dtBodyNode->getParentJoint();

  // This is for the case this function called before DARTModel::Init() is
  // called.
  if (joint == nullptr)
  {
    gzerr << "DARTModel::Init() should be called first.\n";
    return;
  }

  // Check if the parent joint is free joint
  dart::dynamics::FreeJoint *freeJoint =
      dynamic_cast<dart::dynamics::FreeJoint*>(joint);

  // If the parent joint is free joint, set the proper generalized velocity to
  // fit the linear velocity of the link
  if (freeJoint)
  {
    // Generalized velocities
    Eigen::Vector3d genVel = DARTTypes::ConvVec3(_vel);

    dart::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

    // If this link has parent link then subtract the effect of parent link's
    // linear and angular velocities
    if (dtBodyNode->getParentBodyNode())
    {
      // Local transformation from the parent link frame to this link frame
      Eigen::Isometry3d T = freeJoint->getLocalTransform();

      // Parent link's linear and angular velocities
      Eigen::Vector3d parentAngVel =
          dtBodyNode->getParentBodyNode()->getAngularVelocity();

      // The effect of the parent link's velocities
      Eigen::Vector3d propagatedAngVel = T.linear().transpose() * parentAngVel;

      // Subtract the effect
      genVel -= propagatedAngVel;
    }

    // Rotation matrix from world frame to this link frame
    Eigen::Matrix3d R = dtBodyNode->getTransform().linear();

    // Change the reference frame to world
    genVel = R * genVel;

    // Set the generalized velocities
    freeJoint->setVelocity(0, genVel[0]);
    freeJoint->setVelocity(1, genVel[1]);
    freeJoint->setVelocity(2, genVel[2]);
  }
  else
  {
    gzdbg << "DARTLink::SetLinearVel() doesn't make sense if the parent joint "
          << "is not free joint (6-dof).\n";
  }
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const ignition::math::Vector3d &_force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Force", std::bind(&DARTLink::SetForce, this, _force));
    return;
  }

  // DART assume that _force is external force.
  this->dataPtr->dtBodyNode->setExtForce(DARTTypes::ConvVec3(_force));
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Torque", std::bind(&DARTLink::SetTorque, this, _torque));
    return;
  }

  // DART assume that _torque is external torque.
  this->dataPtr->dtBodyNode->setExtTorque(DARTTypes::ConvVec3(_torque));
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const ignition::math::Vector3d &_force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Force", std::bind(&DARTLink::AddForce, this, _force));
    return;
  }

  this->dataPtr->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_force));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const ignition::math::Vector3d &_force)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "RelativeForce",
          std::bind(&DARTLink::AddRelativeForce, this, _force));
    return;
  }

  this->dataPtr->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_force),
                                Eigen::Vector3d::Zero(),
                                true, true);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const ignition::math::Vector3d &_force,
                                        const ignition::math::Vector3d &_pos)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "ForceAtWorldPosition",
          std::bind(&DARTLink::AddForceAtWorldPosition, this, _force, _pos));
    return;
  }

  this->dataPtr->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_pos),
                                DARTTypes::ConvVec3(_force),
                                false, false);
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(
    const ignition::math::Vector3d &_force,
    const ignition::math::Vector3d &_relpos)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "ForceAtRelativePosition",
          std::bind(
            &DARTLink::AddForceAtRelativePosition, this, _force, _relpos));
    return;
  }

  this->dataPtr->dtBodyNode->addExtForce(DARTTypes::ConvVec3(_force),
                                DARTTypes::ConvVec3(_relpos),
                                true, true);
}

//////////////////////////////////////////////////
void DARTLink::AddLinkForce(const ignition::math::Vector3d &/*_force*/,
    const ignition::math::Vector3d &/*_offset*/)
{
  gzlog << "DARTLink::AddLinkForce not yet implemented (issue #1477)."
        << std::endl;
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "Torque", std::bind(&DARTLink::AddTorque, this, _torque));
    return;
  }

  this->dataPtr->dtBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque));
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const ignition::math::Vector3d &_torque)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "RelativeTorque",
          std::bind(&DARTLink::AddRelativeTorque, this, _torque));
    return;
  }

  this->dataPtr->dtBodyNode->addExtTorque(DARTTypes::ConvVec3(_torque), true);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset) const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<ignition::math::Vector3d>("WorldLinearVel");

  Eigen::Vector3d linVel = this->dataPtr->dtBodyNode->getLinearVelocity(
        DARTTypes::ConvVec3(_offset));

  return DARTTypes::ConvVec3Ign(linVel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldLinearVel(
    const ignition::math::Vector3d &_offset,
    const ignition::math::Quaterniond &_q) const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<ignition::math::Vector3d>("WorldLinearVel");

  Eigen::Matrix3d R1 = Eigen::Matrix3d(DARTTypes::ConvQuat(_q));
  Eigen::Vector3d worldOffset = R1 * DARTTypes::ConvVec3(_offset);
  Eigen::Vector3d bodyOffset =
      this->dataPtr->dtBodyNode->getTransform().linear().transpose() *
      worldOffset;
  Eigen::Vector3d linVel =
      this->dataPtr->dtBodyNode->getLinearVelocity(bodyOffset);

  return DARTTypes::ConvVec3Ign(linVel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldCoGLinearVel() const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
        "WorldCoGLinearVel");
  }

  Eigen::Vector3d linVel = this->dataPtr->dtBodyNode->getCOMLinearVelocity();

  return DARTTypes::ConvVec3Ign(linVel);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldAngularVel() const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
        "WorldAngularVel");
  }

  Eigen::Vector3d angVel = this->dataPtr->dtBodyNode->getAngularVelocity();

  return DARTTypes::ConvVec3Ign(angVel);
}

/////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldForce() const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<ignition::math::Vector3d>("WorldForce");

  Eigen::Vector6d F = this->dataPtr->dtBodyNode->getExternalForceGlobal();
  return DARTTypes::ConvVec3Ign(F.tail<3>());
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTLink::WorldTorque() const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<ignition::math::Vector3d>("WorldTorque");

  // TODO: Need verification
  ignition::math::Vector3d torque;

  Eigen::Isometry3d W = this->dataPtr->dtBodyNode->getTransform();
  Eigen::Matrix6d G   = this->dataPtr->dtBodyNode->getSpatialInertia();
  Eigen::VectorXd V   = this->dataPtr->dtBodyNode->getSpatialVelocity();
  Eigen::VectorXd dV  = this->dataPtr->dtBodyNode->getSpatialAcceleration();
  Eigen::Vector6d F   = G * dV - dart::math::dad(V, G * V);

  torque = DARTTypes::ConvVec3Ign(W.linear() * F.head<3>());

  return torque;
}

//////////////////////////////////////////////////
void DARTLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);

  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "GravityMode", std::bind(&DARTLink::SetGravityMode, this, _mode));
    return;
  }

  this->dataPtr->dtBodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool DARTLink::GetGravityMode() const
{
  GZ_ASSERT(!this->dataPtr->IsInitialized() ||
            (this->dataPtr->dtBodyNode->getGravityMode() ==
             this->sdf->Get<bool>("gravity")),
            "Gazebo and DART disagree in gravity mode of the link.");

  return this->sdf->Get<bool>("gravity");
}

//////////////////////////////////////////////////
void DARTLink::SetSelfCollide(bool _collide)
{
  this->sdf->GetElement("self_collide")->Set(_collide);

  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "SelfCollide",
          std::bind(&DARTLink::SetSelfCollide, this, _collide));
    return;
  }

  dart::dynamics::BodyNode *dtBodyNode = this->dataPtr->dtBodyNode;

  // If this function is called before the body node is not added to a skeleton,
  // the body node does not have parent skeleton. So we just return here. Self
  // collision setting will be done later in DARTModel::Init().
  if (dtBodyNode->getSkeleton() == nullptr)
    return;

  dart::simulation::WorldPtr dtWorld = this->dataPtr->dartPhysics->DARTWorld();
  dart::dynamics::SkeletonPtr dtSkeleton =
      dtBodyNode->getSkeleton();
  dart::collision::CollisionDetector *dtCollDet =
      dtWorld->getConstraintSolver()->getCollisionDetector();

  Link_V links = this->GetModel()->GetLinks();

  bool isSkeletonSelfCollidable =
      dtBodyNode->getSkeleton()->isEnabledSelfCollisionCheck();

  if (_collide)
  {
    // If the skeleton is already self collidable, then we enable self
    // collidable pairs those are associated with this link. The links in the
    // pairs should be all and not itself each other.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this && links[i]->GetSelfCollide())
        {
          dart::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();

          // If this->dataPtr->dtBodyNode and itdtBodyNode are connected then
          // don't enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((dtBodyNode->getParentBodyNode() == itdtBodyNode) ||
              itdtBodyNode->getParentBodyNode() == this->dataPtr->dtBodyNode)
            continue;

          dtCollDet->enablePair(dtBodyNode, itdtBodyNode);
        }
      }
    }
    // If the skeleton is not self collidable, we first set the skeleton as
    // self collidable. If the skeleton is self collidable, then DART regards
    // that all the links in the skeleton is self collidable. So, we disable all
    // the pairs of which both of the links in the pair is not self collidable.
    else
    {
      dtSkeleton->enableSelfCollision();

      for (size_t i = 0; i < links.size() - 1; ++i)
      {
        for (size_t j = i + 1; j < links.size(); ++j)
        {
          dart::dynamics::BodyNode *itdtBodyNode1 =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();
          dart::dynamics::BodyNode *itdtBodyNode2 =
            boost::dynamic_pointer_cast<DARTLink>(links[j])->GetDARTBodyNode();

          // If this->dataPtr->dtBodyNode and itdtBodyNode are connected then
          // don't enable the pair.
          // Please see: https://bitbucket.org/osrf/gazebo/issue/899
          if ((itdtBodyNode1->getParentBodyNode() == itdtBodyNode2) ||
              itdtBodyNode2->getParentBodyNode() == itdtBodyNode1)
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);

          if (!links[i]->GetSelfCollide() || !links[j]->GetSelfCollide())
            dtCollDet->disablePair(itdtBodyNode1, itdtBodyNode2);
        }
      }
    }
  }
  else
  {
    // If the skeleton is self collidable, then we disable all the pairs
    // associated with this link.
    if (isSkeletonSelfCollidable)
    {
      for (size_t i = 0; i < links.size(); ++i)
      {
        if (links[i].get() != this)
        {
          dart::dynamics::BodyNode *itdtBodyNode =
            boost::dynamic_pointer_cast<DARTLink>(links[i])->GetDARTBodyNode();
          dtCollDet->disablePair(dtBodyNode, itdtBodyNode);
        }
      }
    }

    // If now all the links are not self collidable, then we set the skeleton
    // as not self collidable.
    bool isAllLinksNotCollidable = true;
    for (size_t i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetSelfCollide())
      {
        isAllLinksNotCollidable = false;
        break;
      }
    }
    if (isAllLinksNotCollidable)
      dtSkeleton->disableSelfCollision();
  }
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzwarn << "DART does not support DARTLink::SetLinearDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  // see: https://github.com/dartsim/dart/issues/85
  gzwarn << "DART does not support DARTLink::SetAngularDamping() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);

  gzwarn << "DART does not support DARTLink::SetKinematic() yet.\n";
}

//////////////////////////////////////////////////
bool DARTLink::GetKinematic() const
{
  // DART does not support kinematic mode for link.";
  return false;
}

//////////////////////////////////////////////////
void DARTLink::SetAutoDisable(bool /*_disable*/)
{
  gzwarn << "DART does not support DARTLink::SetAutoDisable() yet.\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinkStatic(bool _static)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "LinkStatic", std::bind(&DARTLink::SetLinkStatic, this, _static));
    return;
  }

  if (_static == this->dataPtr->staticLink)
    return;

  if (_static == true)
  {
    // Add weld joint constraint to DART
    this->dataPtr->dtWeldJointConst =
        new dart::constraint::WeldJointConstraint(this->dataPtr->dtBodyNode);
    this->DARTWorld()->getConstraintSolver()->addConstraint(
        this->dataPtr->dtWeldJointConst);
  }
  else
  {
    // Remove ball and revolute joint constraints from DART
    this->DARTWorld()->getConstraintSolver()->removeConstraint(
        this->dataPtr->dtWeldJointConst);
    delete this->dataPtr->dtWeldJointConst;
    this->dataPtr->dtWeldJointConst = nullptr;
  }

  this->dataPtr->staticLink = _static;
}

//////////////////////////////////////////////////
void DARTLink::updateDirtyPoseFromDARTTransformation()
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          "DirtyPoseFromDARTTransformation",
          std::bind(&DARTLink::updateDirtyPoseFromDARTTransformation, this));
    return;
  }

  // Step 1: get dart body's transformation
  // Step 2: set gazebo link's pose using the transformation
  ignition::math::Pose3d newPose = DARTTypes::ConvPoseIgn(
      this->dataPtr->dtBodyNode->getTransform());

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dataPtr->dirtyPoses.push_back(this);
}

//////////////////////////////////////////////////
DARTPhysicsPtr DARTLink::GetDARTPhysics(void) const
{
  return boost::dynamic_pointer_cast<DARTPhysics>(
        this->GetWorld()->Physics());
}

//////////////////////////////////////////////////
dart::simulation::WorldPtr DARTLink::DARTWorld(void) const
{
  return this->GetDARTPhysics()->DARTWorld();
}

//////////////////////////////////////////////////
DARTModelPtr DARTLink::GetDARTModel() const
{
  return boost::dynamic_pointer_cast<DARTModel>(this->GetModel());
}

//////////////////////////////////////////////////
DARTBodyNodePropPtr DARTLink::DARTProperties() const
{
  return this->dataPtr->dtProperties;
}

//////////////////////////////////////////////////
void DARTLink::SetDARTBodyNode(dart::dynamics::BodyNode *_dtBodyNode)
{
  this->dataPtr->dtBodyNode = _dtBodyNode;
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode *DARTLink::GetDARTBodyNode() const
{
  return this->dataPtr->dtBodyNode;
}

//////////////////////////////////////////////////
void DARTLink::SetDARTParentJoint(DARTJointPtr _dartParentJoint)
{
  this->dataPtr->dartParentJoint = _dartParentJoint;
}

//////////////////////////////////////////////////
void DARTLink::AddDARTChildJoint(DARTJointPtr _dartChildJoint)
{
  this->dataPtr->dartChildJoints.push_back(_dartChildJoint);
}

//////////////////////////////////////////////////
bool DARTLink::IsSoftBody() const
{
  return this->dataPtr->isSoftBody;
}

