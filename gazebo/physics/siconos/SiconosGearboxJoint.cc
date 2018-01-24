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
#include "gazebo/physics/siconos/SiconosGearboxJoint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include <siconos/Model.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/PivotJointR.hpp>
#include <siconos/CouplerJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosGearboxJoint::SiconosGearboxJoint(BasePtr _parent, SP::SiconosWorld _world)
  : GearboxJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
  siconosWorld = _world;

  // Will be initialized in SiconosConnectJoint(), below.
  this->siconosCouplerJointR = std11::make_shared<CouplerJointR>();
  this->siconosCouplerJointR->setAbsolute(false);

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosCouplerJointR, SP::Interaction()});
}

//////////////////////////////////////////////////
SiconosGearboxJoint::~SiconosGearboxJoint()
{
}

//////////////////////////////////////////////////
bool SiconosGearboxJoint::IsInitialized() const
{
  return this->referenceLink != nullptr;
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<SiconosJoint>::Load(_sdf);

  this->SetGearboxRatio(this->gearRatio);
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::Init()
{
  GearboxJoint<SiconosJoint>::Init();

  LinkPtr link = this->model->GetLink(this->referenceBody);
  this->SetReferenceBody(link);

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
  // this->siconosGearbox->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();

  // Connect the dynamical systems in the Siconos graph
  this->SiconosConnect();
}

//////////////////////////////////////////////////
double SiconosGearboxJoint::PositionImpl(const unsigned int _index) const
{
  double result = 0.0;
  SiconosLinkPtr link0, link1;

  if (this->IsConnected() && _index < 1)
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
    this->siconosCouplerJointR->computehDoF(0.0, bv, y, _index);
    result = y(0);
  }

  return result;
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double SiconosGearboxJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0.0;

  // TODO
  return result;
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  // TOOD (requires normalDoF)
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2)
{
  // Get joints associated with the links and the reference link
  SiconosJointPtr j0, j1;

  // 0 or 1 depending if ref is child or parent
  unsigned int j0refidx=0, j1refidx=0;

  for (JointPtr j : this->model->GetJoints())
  {
    if (j->GetParent() == this->parentLink
        && j->GetChild() == this->referenceLink)
    {
      if (j0)
        gzwarn << "more than one joint found for same gearbox links\n";
      j0 = boost::dynamic_pointer_cast<SiconosJoint>(j);
      j0refidx = 1; // reference child
    }
    else if (j->GetChild() == this->parentLink
             && j->GetParent() == this->referenceLink)
    {
      if (j0)
        gzwarn << "more than one joint found for same gearbox links\n";
      j0 = boost::dynamic_pointer_cast<SiconosJoint>(j);
      j0refidx = 0; // reference parent
    }
    else if (j->GetParent() == this->childLink
             && j->GetChild() == this->referenceLink)
    {
      if (j1)
        gzwarn << "more than one joint found for same gearbox links\n";
      j1 = boost::dynamic_pointer_cast<SiconosJoint>(j);
      j1refidx = 1; // reference child
    }
    else if (j->GetChild() == this->childLink
             && j->GetParent() == this->referenceLink)
    {
      if (j1)
        gzwarn << "more than one joint found for same gearbox links\n";
      j1 = boost::dynamic_pointer_cast<SiconosJoint>(j);
      j1refidx = 0; // reference parent
    }
  }

  if (!j0 || !j1) {
    gzwarn << "reference joints not found, cannot connect gearbox\n";
    return;
  }

  // Get reference body
  SP::BodyDS refds;
  if (this->referenceLink)
    refds = this->referenceLink->GetSiconosBodyDS();

  // Set up the gear-ratio coupler
  SP::CouplerJointR cr = this->siconosCouplerJointR;
  cr->setRatio(this->gearRatio);
  cr->setReferences(j0->Relation(0), 0, j1->Relation(0), 0,
                    refds, j0refidx, refds, j1refidx);

  // Default implementation does the rest
  GearboxJoint<SiconosJoint>::SiconosConnectJoint(ds1, ds2);
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::SetGearboxRatio(double _gearRatio)
{
  this->gearRatio = _gearRatio;
  if (this->siconosCouplerJointR)
    this->siconosCouplerJointR->setRatio(this->gearRatio);
}

//////////////////////////////////////////////////
void SiconosGearboxJoint::SetReferenceBody(LinkPtr _body)
{
  SiconosLinkPtr link = boost::dynamic_pointer_cast<SiconosLink>(_body);

  if (link == nullptr && _body != nullptr)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
  }

  this->referenceLink = link;
}
