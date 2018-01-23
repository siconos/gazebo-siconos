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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosScrewJoint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include <siconos/Model.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/CylindricalJointR.hpp>
#include <siconos/CouplerJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosScrewJoint::SiconosScrewJoint(BasePtr _parent, SP::SiconosWorld _world)
  : ScrewJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
  siconosWorld = _world;

  this->threadPitch = 0.01;

  this->siconosCylindricalJointR = std11::make_shared<CylindricalJointR>();
  this->siconosCylindricalJointR->setAbsolute(false);

  // Will be initialized in SiconosConnectJoint(), below.
  this->siconosCouplerJointR = std11::make_shared<CouplerJointR>();
  this->siconosCouplerJointR->setAbsolute(false);

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosCylindricalJointR, SP::Interaction()});
}

//////////////////////////////////////////////////
SiconosScrewJoint::~SiconosScrewJoint()
{
}

//////////////////////////////////////////////////
void SiconosScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosScrewJoint::Init()
{
  ScrewJoint<SiconosJoint>::Init();

  // Joint cylindrical axis already set up by SiconosJoint::SetAnchor() and
  // SiconosJoint::SetAxis()

  // Apply joint angle limits here.
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
double SiconosScrewJoint::PositionImpl(const unsigned int _index) const
{
  double result = 0.0;
  SiconosLinkPtr link0, link1;

  if (this->siconosCylindricalJointR
      && _index < this->siconosCylindricalJointR->numberOfDoF())
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

    // Can be called between Load and Init, so check if body exists
    if (link0 && !link0->GetSiconosBodyDS()) return 0.0;
    if (link1 && !link1->GetSiconosBodyDS()) return 0.0;

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector y(1);
    /* Note: 1-_index because opposite to Siconos, 0=angular, 1=linear in Gazebo */
    this->siconosCylindricalJointR->computehDoF(0.0, bv, y, 1-_index);
    result = y(0);
  }

  return result;
}

//////////////////////////////////////////////////
void SiconosScrewJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double SiconosScrewJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (this->siconosCylindricalJointR && _index == 0)
  {
    ignition::math::Vector3d globalAxis = this->GlobalAxis(0);
    if (this->parentLink)
      result = globalAxis.Dot(this->parentLink->WorldLinearVel());
    if (this->parentLink && this->childLink)
      result -= globalAxis.Dot(this->childLink->WorldLinearVel());
    else if (this->childLink)
      result = globalAxis.Dot(this->childLink->WorldLinearVel());
  }
  else if (this->siconosCylindricalJointR && _index == 1)
  {
    ignition::math::Vector3d globalAxis = this->GlobalAxis(0);
    if (this->parentLink)
      result = globalAxis.Dot(this->parentLink->WorldAngularVel());
    if (this->parentLink && this->childLink)
      result -= globalAxis.Dot(this->childLink->WorldAngularVel());
    else if (this->childLink)
      result = globalAxis.Dot(this->childLink->WorldAngularVel());
  }
  return result;
}

//////////////////////////////////////////////////
void SiconosScrewJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosCylindricalJointR
      && _index < this->siconosCylindricalJointR->numberOfDoF())
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    // Can be called between Load and Init, so check if body exists
    if (link0 && !link0->GetSiconosBodyDS()) return;
    if (link1 && !link1->GetSiconosBodyDS()) return;

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector v(3);
    this->siconosCylindricalJointR->normalDoF(v, bv, 0, true);
    ignition::math::Vector3d axis(SiconosTypes::ConvertVector3(v));

    /* Note: linear/angular indexes opposite to Siconos */

    if (_index==1)
    {
      if (link0 && link1) {
        link0->AddRelativeForce(axis * -_effort);
        link1->AddRelativeForce(axis * _effort);
      }
      else if (link0) {
        link0->AddRelativeForce(axis * _effort);
      }
      else if (link1) {
        link1->AddRelativeForce(axis * _effort);
      }
    }
    else if (_index==0)
    {
      if (link0 && link1) {
        link0->AddTorque(-_effort * axis);
        link1->AddTorque(_effort * axis);
      }
      else if (link0) {
        link0->AddTorque(_effort * axis);
      }
      else if (link1) {
        link1->AddTorque(_effort * axis);
      }
    }
  }
}

//////////////////////////////////////////////////
void SiconosScrewJoint::SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2)
{
  ScrewJoint<SiconosJoint>::SiconosConnectJoint(ds1, ds2);

  // Set up the coupler relation.
  // The screw is defined by a coupling between DoF 0 (prismatic) and
  // DoF 1 (pivot) of the cylindrical joint.
  this->siconosCouplerJointR->setReferences(
    this->siconosCylindricalJointR, 0, // angular
    this->siconosCylindricalJointR, 1, // linear
    // no 3rd-party refs
    SP::SiconosVector(), 0, SP::SiconosVector(), 0);

  // Set the ratio
  this->siconosCouplerJointR->setRatio(this->threadPitch);

  // Make the interaction
  int nc = this->siconosCouplerJointR->numberOfConstraints();
  this->siconosCouplerInteraction = std11::make_shared<::Interaction>(
    std11::make_shared<EqualityConditionNSL>(nc), this->siconosCouplerJointR);

  // Add the interaction to the NSDS
  this->siconosWorld->GetSimulation()->link(this->siconosCouplerInteraction,
                                            ds1, ds2);
}

//////////////////////////////////////////////////
void SiconosScrewJoint::SiconosDisconnectJoint()
{
  ScrewJoint<SiconosJoint>::SiconosDisconnectJoint();
  this->siconosWorld->GetSimulation()->unlink(this->siconosCouplerInteraction);
}

//////////////////////////////////////////////////
void SiconosScrewJoint::SetThreadPitch(double _threadPitch)
{
  this->threadPitch = _threadPitch;
  this->siconosCouplerJointR->setRatio(this->threadPitch);
}

//////////////////////////////////////////////////
double SiconosScrewJoint::GetThreadPitch()
{
  return threadPitch;
}

