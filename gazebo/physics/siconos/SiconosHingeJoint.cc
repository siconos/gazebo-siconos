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
#include "gazebo/physics/siconos/SiconosHingeJoint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include <siconos/Model.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/PivotJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosHingeJoint::SiconosHingeJoint(BasePtr _parent, SP::SiconosWorld _world)
  : HingeJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
  siconosWorld = _world;

  this->siconosPivotJointR = std11::make_shared<PivotJointR>();
  this->siconosPivotJointR->setAbsolute(false);

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosPivotJointR, SP::Interaction()});
}

//////////////////////////////////////////////////
SiconosHingeJoint::~SiconosHingeJoint()
{
}

//////////////////////////////////////////////////
void SiconosHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosHingeJoint::Init()
{
  HingeJoint<SiconosJoint>::Init();

  // Joint pivot axis already set up by SiconosJoint::SetAnchor() and
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

  // Allows access to impulse
  // this->siconosHinge->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();

  // Connect the dynamical systems in the Siconos graph
  this->SiconosConnect();
}

//////////////////////////////////////////////////
double SiconosHingeJoint::PositionImpl(const unsigned int _index) const
{
  double result = 0.0;
  SiconosLinkPtr link0, link1;

  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
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
    this->siconosPivotJointR->computehDoF(0.0, bv, y, _index);
    result = y(0);
  }

  return result;
}

//////////////////////////////////////////////////
void SiconosHingeJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double SiconosHingeJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
  {
    ignition::math::Vector3d globalAxis = this->GlobalAxis(_index);
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
void SiconosHingeJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector v(3);
    this->siconosPivotJointR->normalDoF(v, bv, _index, true);
    ignition::math::Vector3d axis(SiconosTypes::ConvertVector3(v));

    if (this->parentLink && this->childLink) {
      this->parentLink->AddTorque(-_effort * axis);
      this->childLink->AddTorque(_effort * axis);
    }

    else if (this->parentLink) {
      this->parentLink->AddTorque(_effort * axis);
    }

    else if (this->childLink) {
      this->childLink->AddTorque(_effort * axis);
    }
  }
}
