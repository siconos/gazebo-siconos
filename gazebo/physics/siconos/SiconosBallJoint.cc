/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosBallJoint.hh"

// Use Bullet math to calculate reference frames
#include "gazebo/physics/bullet/BulletTypes.hh"

#include "SiconosWorld.hh"
#include <siconos/KneeJointR.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>

#include <siconos/Model.hpp>
#include <siconos/BodyDS.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosBallJoint::SiconosBallJoint(BasePtr _parent, SP::SiconosWorld _world)
    : BallJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
  this->siconosWorld = _world;

  this->siconosKneeJointR = std11::make_shared<KneeJointR>();
  this->siconosKneeJointR->setAbsolute(false);

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosKneeJointR, SP::Interaction()});
}

//////////////////////////////////////////////////
SiconosBallJoint::~SiconosBallJoint()
{
}

//////////////////////////////////////////////////
void SiconosBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosBallJoint::Init()
{
  BallJoint<SiconosJoint>::Init();

  // Joint anchor already set up by SiconosJoint::SetAnchor()

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != NULL, "Joint axis sdf member is NULL");
  this->SetupJointLimits();

  // Set Joint friction here in Init, since the siconos data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();

  // Connect the dynamical systems in the Siconos graph
  this->SiconosConnect();
}

//////////////////////////////////////////////////
double SiconosBallJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (this->siconosKneeJointR
      && _index < this->siconosKneeJointR->numberOfDoF())
  {
    ignition::math::Vector3d globalAxis = this->GlobalAxis(_index);
    if (this->parentLink)
      result = globalAxis.Dot(this->parentLink->WorldCoGLinearVel());
    if (this->parentLink && this->childLink)
      result -= globalAxis.Dot(this->childLink->WorldCoGLinearVel());
    else if (this->childLink) {
      result = globalAxis.Dot(this->childLink->WorldCoGLinearVel());
    }
  }
  return result;
}

//////////////////////////////////////////////////
void SiconosBallJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosBallJoint::Anchor(unsigned int /*_index*/) const
{
  gzerr << "Anchor(): No relation for link " << GetName() << ".";

  // Since SetAxis() takes the axis in world frame, we return it in
  // world frame here.
  ignition::math::Vector3d anchor =
    SiconosTypes::ConvertVector3(this->siconosKneeJointR->axes()[0]);

  SiconosLinkPtr link = boost::static_pointer_cast<SiconosLink>(this->parentLink);
  if (!link) link = boost::static_pointer_cast<SiconosLink>(this->childLink);
  if (!link) return ignition::math::Vector3d::Zero;

  return link->WorldPose().CoordPositionAdd(anchor);
}

//////////////////////////////////////////////////
void SiconosBallJoint::SetDamping(unsigned int /*index*/,
                                    const double /*_damping*/)
{
  /// \TODO: special case siconos specific linear damping, this needs testing.
  // if (this->siconosKneeJointR)
  //   this->siconosKneeJointR->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void SiconosBallJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosKneeJointR
      && _index < this->siconosKneeJointR->numberOfDoF())
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector v(3);
    this->siconosKneeJointR->normalDoF(v, bv, _index, false);
    ignition::math::Vector3d axis(SiconosTypes::ConvertVector3(v));

    if (link0 && link1) {
      link0->AddRelativeForce(axis * _effort/2);
      link1->AddRelativeForce(axis * -_effort/2);
    }
    else if (link0) {
      link0->AddRelativeForce(axis * _effort);
    }
    else if (link1) {
      link1->AddRelativeForce(axis * _effort);
    }
  }
}

//////////////////////////////////////////////////
double SiconosBallJoint::PositionImpl(unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid axis index [" << _index << "]" << std::endl;
    return 0.0;
  }

  SiconosLinkPtr link0, link1;
  double result = 0.0;

  if (this->siconosKneeJointR)
  {
    if (this->parentLink) {
      link0 = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    }
    if (this->parentLink && this->childLink) {
      link1 = boost::static_pointer_cast<SiconosLink>(this->childLink);
    }
    else if (this->childLink) {
      link0 = boost::static_pointer_cast<SiconosLink>(this->childLink);
    }
    else
      return 0.0;

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector y(1);
    this->siconosKneeJointR->computehDoF(0.0, bv, y, _index);
    result = y(0);
  }

  return result;
}
