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
#include "gazebo/physics/siconos/SiconosHinge2Joint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include <siconos/Model.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/PivotJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosHinge2Joint::SiconosHinge2Joint(BasePtr _parent, SP::SiconosWorld _world)
  : Hinge2Joint<SiconosJoint>(_parent)
{
  siconosWorld = _world;
  GZ_ASSERT(siconosWorld, "SiconosWorld pointer is NULL");

  this->siconosPivotJointR1 = std11::make_shared<PivotJointR>();
  this->siconosPivotJointR1->setAbsolute(false);

  this->siconosPivotJointR2 = std11::make_shared<PivotJointR>();
  this->siconosPivotJointR2->setAbsolute(false);

  // Second pivot is relative to coupler, so always 0,0,0
  auto z(std11::make_shared<SiconosVector>(3));
  z->zero();
  this->siconosPivotJointR2->setPoint(0, z);

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosPivotJointR1, SP::Interaction()});
  this->relInterPairs.push_back({this->siconosPivotJointR2, SP::Interaction()});
}

//////////////////////////////////////////////////
SiconosHinge2Joint::~SiconosHinge2Joint()
{
}

//////////////////////////////////////////////////
void SiconosHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosHinge2Joint::Init()
{
  Hinge2Joint<SiconosJoint>::Init();

  // Joint pivot axes already set up by SiconosJoint::SetAnchor() and
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
double SiconosHinge2Joint::PositionImpl(const unsigned int _index) const
{
  double result = 0.0;
  SiconosLinkPtr link0, link1;

  if (this->siconosPivotJointR1
      && _index < this->siconosPivotJointR1->numberOfDoF())
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
    this->siconosPivotJointR1->computehDoF(0.0, bv, y, _index);
    result = y(0);
  }

  return result;
}

//////////////////////////////////////////////////
void SiconosHinge2Joint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double SiconosHinge2Joint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (this->siconosPivotJointR1
      && _index < this->siconosPivotJointR1->numberOfDoF())
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
void SiconosHinge2Joint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosPivotJointR1
      && _index < this->siconosPivotJointR1->numberOfDoF())
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector v(3);
    this->siconosPivotJointR1->normalDoF(v, bv, _index, true);
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

//////////////////////////////////////////////////
SP::NewtonEulerJointR SiconosHinge2Joint::Relation(unsigned int _index) const
{
  if (_index == 0) return siconosPivotJointR1;
  if (_index == 1) return siconosPivotJointR2;
  return SP::NewtonEulerJointR();
}

//////////////////////////////////////////////////
void SiconosHinge2Joint::SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2)
{
  GZ_ASSERT(this->relInterPairs.size()==2,
            "SiconosHinge2Joint has wrong number of relations");

  JointInteractionPair& ri1 = relInterPairs[0];
  JointInteractionPair& ri2 = relInterPairs[1];
  SP::NewtonEulerDS& dsc = this->siconosCouplerDS;

  GZ_ASSERT(!ri1.interaction, "joint already connected");
  GZ_ASSERT(!ri2.interaction, "joint already connected");
  GZ_ASSERT(!dsc, "joint already has a coupler");

  // Check that all required points and anchors have been provided.
  GZ_ASSERT(ri1.relation->points()[0], "pivot 1 point missing");
  GZ_ASSERT(ri1.relation->axes()[0], "pivot 1 axis missing");
  GZ_ASSERT(ri2.relation->points()[0], "pivot 2 point missing");
  GZ_ASSERT(ri2.relation->axes()[0], "pivot 2 axis missing");

  // Create coupler in ds1 frame plus anchor.  Note: Since the coupler
  // has a very small mass, it *will* affect the dynamical system!
  // One reason why this implementation is a stand-in for a proper
  // double-hinge joint implementation in Siconos.

  ignition::math::Pose3d q1(SiconosTypes::ConvertPose(*ds1->q()));
  ignition::math::Pose3d q2;
  if (ds2)
    q2 = SiconosTypes::ConvertPose(*ds2->q());

  // Apply ds orientations to axes
  ignition::math::Vector3d a1(
    q1.Rot().RotateVector(SiconosTypes::ConvertVector3(ri1.relation->axes()[0])));
  ignition::math::Vector3d a2(
    q2.Rot().RotateVector(SiconosTypes::ConvertVector3(ri2.relation->axes()[0])));

  // Initial orientation of coupler must align axis 1 rel ds1 and axis 2 rel ds2
  ignition::math::Quaterniond rot1(
        ignition::math::Quaterniond(0, 1, 0, 0)
      - ignition::math::Quaterniond(0, a1.X(), a1.Y(), a1.Z()));
  ignition::math::Quaterniond rot2(
        ignition::math::Quaterniond(0, 0, 1, 0)
      - ignition::math::Quaterniond(0, a2.X(), a2.Y(), a2.Z()));

  // The two vectors represent the plane in which the coupler should lie
  ignition::math::Quaterniond r(rot1 + rot2);

  // Initial position of coupler = ds1.q + pivot 1 position
  ignition::math::Vector3d p(
    q1.Pos() + SiconosTypes::ConvertVector3(*ri1.relation->points()[0]));

  SP::SiconosVector pos(SiconosTypes::ConvertPose(
                          ignition::math::Pose3d(p, r)));

  SP::SiconosVector vel(std11::make_shared<SiconosVector>(*ds1->velocity()));
  SP::SimpleMatrix I(new SimpleMatrix(3, 3));
  I->eye();
  dsc = std11::make_shared<BodyDS>(pos, vel, 0.00001, I);

  // Set up the coupler chain
  ri1.relation->setBasePositions(ds1->q(), dsc->q());
  ri2.relation->setBasePositions(dsc->q(), ds2 ? ds2->q() : SP::SiconosVector());

  // Create a Siconos Interacton with an EqualityConditionNSL
  int nc = ri1.relation->numberOfConstraints();
  ri1.interaction = std11::make_shared<::Interaction>(
    std11::make_shared<EqualityConditionNSL>(nc), ri1.relation);

  // Create a Siconos Interacton with an EqualityConditionNSL
  nc = ri2.relation->numberOfConstraints();
  ri2.interaction = std11::make_shared<::Interaction>(
    std11::make_shared<EqualityConditionNSL>(nc), ri2.relation);

  // Add the coupler to the NSDS
  this->siconosWorld->GetNonSmoothDynamicalSystem()->insertDynamicalSystem(dsc);

  // Initialize its integrator
  this->siconosWorld->GetSimulation()->associate(
    this->siconosWorld->GetOneStepIntegrator(), dsc);

  // Add the interactions to the NSDS
  this->siconosWorld->GetSimulation()->link(ri1.interaction, ds1, dsc);
  this->siconosWorld->GetSimulation()->link(ri2.interaction, dsc, ds2);
}

//////////////////////////////////////////////////
void SiconosHinge2Joint::SiconosDisconnectJoint()
{
  // Remove the coupler from the NSDS
  this->siconosWorld->GetNonSmoothDynamicalSystem()
    ->removeDynamicalSystem(this->siconosCouplerDS);

  this->siconosCouplerDS.reset();
}

//////////////////////////////////////////////////
unsigned int SiconosHinge2Joint::RelationPointIndex(unsigned int _index) const
{
  GZ_ASSERT(relInterPairs.size() == 0 || relInterPairs.size() == 2,
            "wrong number of relations for Hinge2 joint");

  GZ_ASSERT(_index < 2, "invalid joint anchor index");

  // Both relations use anchor point 0.
  return 0;
}

//////////////////////////////////////////////////
unsigned int SiconosHinge2Joint::RelationAxisIndex(unsigned int _index) const
{
  GZ_ASSERT(relInterPairs.size() == 0 || relInterPairs.size() == 2,
            "wrong number of relations for Hinge2 joint");

  GZ_ASSERT(_index < 2, "invalid joint axis index");

  // Both relations use axis 0.
  return 0;
}
