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
/* Desc: The base Siconos joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include <boost/bind.hpp>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosJoint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include "siconos/NewtonEulerJointR.hpp"
#include "siconos/JointStopR.hpp"
#include "siconos/JointFrictionR.hpp"
#include "siconos/BodyDS.hpp"
#include "siconos/Interaction.hpp"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
std::map< double, SP::RelayNSL > SiconosJoint::frictionNSL;

//////////////////////////////////////////////////
SiconosJoint::SiconosJoint(BasePtr _parent)
  : Joint(_parent)
{
  //this->feedback = NULL;
  this->stiffnessDampingInitialized = false;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
SiconosJoint::~SiconosJoint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void SiconosJoint::Fini()
{
  //this->feedback = NULL;

  this->Detach();
}

//////////////////////////////////////////////////
void SiconosJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }
}

//////////////////////////////////////////////////
void SiconosJoint::Init()
{
  Joint::Init();
}

//////////////////////////////////////////////////
bool SiconosJoint::IsInitialized()
{
  // Joints should have at least one relation, otherwise we assume we
  // are uninitialized.
  return relInterPairs.size() > 0;
}

//////////////////////////////////////////////////
void SiconosJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr SiconosJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (_index < this->relInterPairs.size()
      || !this->relInterPairs[_index].interaction)
    gzthrow("Attach bodies to the joint first");

  if (_index == 0 || _index == 1)
  {
    SiconosLinkPtr siconosLink1 =
      boost::static_pointer_cast<SiconosLink>(this->childLink);

    SiconosLinkPtr siconosLink2 =
      boost::static_pointer_cast<SiconosLink>(this->parentLink);

    // btRigidBody rigidLink = this->constraint->getRigidBodyA();

    // if (siconosLink1 && rigidLink.getUserPointer() == siconosLink1.get())
    //   result = this->childLink;
    // else if (siconosLink2)
      result = this->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool SiconosJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  if (relInterPairs.size() > 0 && this->relInterPairs[0].interaction)
    if ((this->childLink.get() == _one.get() &&
         this->parentLink.get() == _two.get()) ||
        (this->childLink.get() == _two.get() &&
         this->parentLink.get() == _one.get()))
      return true;
  return false;
}

//////////////////////////////////////////////////
bool SiconosJoint::IsConnected() const
{
  SiconosLinkPtr siconosLink1 =
    boost::dynamic_pointer_cast<SiconosLink>(this->childLink);
  SiconosLinkPtr siconosLink2 =
    boost::dynamic_pointer_cast<SiconosLink>(this->parentLink);
  return IsInitialized() && (siconosLink1 || siconosLink2);
}

//////////////////////////////////////////////////
void SiconosJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  // Connect if we are already initialized,
  // otherwise defered until Init().
  if (this->IsInitialized() && !this->IsConnected())
    this->SiconosConnect();
}

//////////////////////////////////////////////////
void SiconosJoint::Detach()
{
  this->SiconosDisconnect();

  Joint::Detach();
}

//////////////////////////////////////////////////
void SiconosJoint::SetProvideFeedback(bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void SiconosJoint::CacheForceTorque()
{
}

//////////////////////////////////////////////////
JointWrench SiconosJoint::GetForceTorque(unsigned int /*_index*/)
{
  GZ_ASSERT(this->relInterPairs.size() > 0 && this->relInterPairs[0].interaction,
            "interaction should be valid");
  return this->wrench;
}

//////////////////////////////////////////////////
void SiconosJoint::SetupJointFeedback()
{
  if (this->provideFeedback)
  {
    // if (this->feedback == NULL)
    // {
    //   this->feedback = new btJointFeedback;
    //   this->feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
    //   this->feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
    //   this->feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
    //   this->feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
    // }

    // if (this->constraint)
    //   this->constraint->setJointFeedback(this->feedback);
    // else
    //   gzerr << "Siconos Joint [" << this->GetName() << "] ID is invalid\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "SiconosJoint::SetDamping: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetStiffness(unsigned int _index, double _stiffness)
{
  if (_index < this->DOF())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "SiconosJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->DOF() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->DOF())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// \TODO: this check might not be needed?  attaching an object to a static
    /// body should not affect damping application.
    bool parentStatic =
      this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
      this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&SiconosJoint::ApplyStiffnessDamping, this));
        this->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->GetName()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
void SiconosJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void SiconosJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->DOF())
  {
    if (this->forceAppliedTime < this->GetWorld()->SimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->SimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double SiconosJoint::GetForce(unsigned int _index)
{
  if (_index < this->DOF())
  {
    return this->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void SiconosJoint::ApplyStiffnessDamping()
{
  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->PositionImpl(i));

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetAnchor(unsigned int /*_index*/,
    const ignition::math::Vector3d & /*_anchor*/)
{
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosJoint::Anchor(
          unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return ignition::math::Vector3d();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosJoint::LinkForce(
          unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return ignition::math::Vector3d();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosJoint::LinkTorque(
          unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return ignition::math::Vector3d();
}

//////////////////////////////////////////////////
void SiconosJoint::SetUpperLimit(unsigned int _index,
                                 const double _limit)
{
  Joint::SetUpperLimit(0, _limit);

  // TODO: Better way to detect no limit?
  if (_limit > 1e15)
    return;

  if (this->Relation(_index) && _index < this->DOF())
  {
    // Make room for this axis index
    if (_index >= this->upperStops.size())
      this->upperStops.resize(_index+1);

    // Check that we don't already have a stop at the same position
    if (this->upperStops[_index].relation
        && this->upperStops[_index].relation->position(0) > _limit-1e-15
        && this->upperStops[_index].relation->position(0) < _limit+1e-15)
      return;

    // Unlink existing Interaction if necessary
    if (this->IsConnected() && this->upperStops[_index].interaction)
      this->siconosWorld->GetSimulation()->unlink(
        this->upperStops[_index].interaction);

    // Create and store the stop Relation and Interaction
    SP::JointStopR rel = std11::make_shared<JointStopR>(
      this->Relation(_index), _limit, true /* stop direction */,
      _index /* dof number */);

    // Use the global joint stop NSL -- TODO: create per-stop
    // NewtonImpactNSL when restitution parameter available in SDF.
    SP::Interaction inter = std11::make_shared<::Interaction>(
      this->siconosWorld->JointStopNSL(), rel);

    this->upperStops[_index] = {rel, inter};

    // If we are connected, link the interaction immediately,
    // otherwise it is defered until Attach() calls SiconosConnect().
    if (IsConnected())
    {
      // Get dynamical systems
      SiconosLinkPtr par = boost::static_pointer_cast<SiconosLink>(this->parentLink);
      SiconosLinkPtr chld  = boost::static_pointer_cast<SiconosLink>(this->childLink);
      SP::BodyDS ds1( par ? par->GetSiconosBodyDS() : nullptr );
      SP::BodyDS ds2( chld  ? chld->GetSiconosBodyDS()  : nullptr );

      if (ds2 && !ds1)
        std::swap(ds1, ds2);

      // Link the stop Interaction
      this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);
    }
  }
  else
  {
    gzlog << "Joint relation not yet created.\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetLowerLimit(unsigned int _index,
                                 const double _limit)
{
  Joint::SetLowerLimit(0, _limit);

  // TODO: Better way to detect no limit?
  if (_limit < -1e15)
    return;

  if (this->Relation(_index) && _index < this->DOF())
  {
    // Make room for this axis index
    if (_index >= this->lowerStops.size())
      this->lowerStops.resize(_index+1);

    // Check that we don't already have a stop at the same position
    if (this->lowerStops[_index].relation
        && this->lowerStops[_index].relation->position(0) > _limit-1e-15
        && this->lowerStops[_index].relation->position(0) < _limit+1e-15)
      return;

    // Unlink existing Interaction if necessary
    if (this->IsConnected() && this->lowerStops[_index].interaction)
      this->siconosWorld->GetSimulation()->unlink(
        this->lowerStops[_index].interaction);

    // Create and store the stop Relation and Interaction
    SP::JointStopR rel = std11::make_shared<JointStopR>(
      this->Relation(_index), _limit, false /* stop direction */,
      _index /* dof number */);

    // Use the global joint stop NSL -- TODO: create per-stop
    // NewtonImpactNSL when restitution parameter available in SDF.
    SP::Interaction inter = std11::make_shared<::Interaction>(
      this->siconosWorld->JointStopNSL(), rel);

    this->lowerStops[_index] = {rel, inter};

    // If we are connected, link the interaction immediately,
    // otherwise it is defered until Attach() calls SiconosConnect().
    if (IsConnected())
    {
      // Get dynamical systems
      SiconosLinkPtr par = boost::static_pointer_cast<SiconosLink>(this->parentLink);
      SiconosLinkPtr chld  = boost::static_pointer_cast<SiconosLink>(this->childLink);
      SP::BodyDS ds1( par ? par->GetSiconosBodyDS() : nullptr );
      SP::BodyDS ds2( chld  ? chld->GetSiconosBodyDS()  : nullptr );

      if (ds2 && !ds1)
        std::swap(ds1, ds2);

      // Link the stop Interaction
      this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);
    }
  }
  else
  {
    gzlog << "Joint relation not yet created.\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetupJointLimits()
{
  GZ_ASSERT(Relation(0), "SiconosJoint::Relation was null during limit setup.");

  if (this->DOF() >= 1)
  {
    sdf::ElementPtr limitElem(this->sdf->GetElement("axis")->GetElement("limit"));
    this->SetUpperLimit(0, limitElem->Get<double>("upper"));
    this->SetLowerLimit(0, limitElem->Get<double>("lower"));
    if (limitElem->HasAttribute("velocity"))
      this->SetVelocityLimit(0, limitElem->Get<double>("velocity"));
  }

  if (this->DOF() >= 2)
  {
    sdf::ElementPtr limitElem(this->sdf->GetElement("axis2")->GetElement("limit"));
    this->SetUpperLimit(1, limitElem->Get<double>("upper"));
    this->SetLowerLimit(1, limitElem->Get<double>("lower"));
    if (limitElem->HasAttribute("velocity"))
      this->SetVelocityLimit(1, limitElem->Get<double>("velocity"));
  }
}

//////////////////////////////////////////////////
bool SiconosJoint::SetParam(const std::string &_key,
    unsigned int _index,
    const boost::any &_value)
{
  if (_key == "friction") {
    return this->SetFriction(_index, boost::any_cast<double>(_value));
  }
  else
    gzdbg << "Not implement in Siconos\n";
  return false;
}

//////////////////////////////////////////////////
double SiconosJoint::GetParam(const std::string &_key,
    unsigned int _index)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }
  GZ_ASSERT(_index < this->jointFriction.size(),
            "_index < this->jointFriction.size() in SiconosJoint::GetParam()");

  if (_key == "friction")
  {
    return this->jointFriction[_index].value;
  }
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
bool SiconosJoint::SetFriction(unsigned int _index, double _value)
{
  gzlog << "SetFriction[" << GetName() << "]: " << _index << ", " << _value;
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return false;
  }

  if (!this->Relation(_index))
  {
    gzlog << "Joint relation not yet created.\n";
    return false;
  }

  // Make room for this axis index
  if (_index >= this->jointFriction.size())
    this->jointFriction.resize(_index+1);

  // Check that we don't already have friction at the same value
  if (this->jointFriction[_index].relation
      && this->jointFriction[_index].value > _value-1e-15
      && this->jointFriction[_index].value < _value+1e-15)
    return true;

  // Unlink existing Interaction if necessary
  if (this->IsConnected() && this->jointFriction[_index].interaction)
    this->siconosWorld->GetSimulation()->unlink(
      this->jointFriction[_index].interaction);

  // If no limit requested, clear things out and just return
  if (_value < 0.0)
  {
    this->jointFriction[_index] = {};
    return true;
  }

  // Create and store the stop Relation and Interaction
  SP::JointFrictionR rel =
    std11::make_shared<JointFrictionR>(this->Relation(_index), _index);

  // Joint friction in Siconos is specified as a force limit using
  // RelayNSL, i.e.: F_friction <= mu * F_normal, where mu * F_normal
  // is a constant.

  // Note: This appears to be much stronger friction than what one
  // gets from ODE's dParamFMax, not clear why, so in the meantime we
  // scale down the limit to approximately match ODE behaviour.
  double limit = _value*0.003;

  // Look up an existing nslaw if possible, otherwise create one.
  SP::RelayNSL nslaw(this->frictionNSL[limit]);
  if (!nslaw)
  {
    this->frictionNSL[limit] = nslaw =
      std11::make_shared<RelayNSL>(1 /*size*/, -limit, limit);
  }

  SP::Interaction inter = std11::make_shared<::Interaction>(nslaw, rel);

  this->jointFriction[_index] = {rel, inter, _value};

  // If we are connected, link the interaction immediately,
  // otherwise it is defered until Attach() calls SiconosConnect().
  if (IsConnected())
  {
    // Get dynamical systems
    SiconosLinkPtr par = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    SiconosLinkPtr chld  = boost::static_pointer_cast<SiconosLink>(this->childLink);
    SP::BodyDS ds1( par ? par->GetSiconosBodyDS() : nullptr );
    SP::BodyDS ds2( chld  ? chld->GetSiconosBodyDS()  : nullptr );

    if (ds2 && !ds1)
      std::swap(ds1, ds2);

    // Link the friction Interaction
    this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);
  }

  return true;
}

//////////////////////////////////////////////////
bool SiconosJoint::SetPosition(unsigned int _index, double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}

//////////////////////////////////////////////////
bool SiconosJoint::SiconosConnect()
{
  // If no relation for this joint (strange, but valid), or first
  // interaction is already present, we just return true.
  if (relInterPairs.size() < 1 || relInterPairs[0].interaction)
    return true;

  // Cast to SiconosLink
  SiconosLinkPtr siconosChildLink =
    boost::static_pointer_cast<SiconosLink>(this->childLink);
  SiconosLinkPtr siconosParentLink =
    boost::static_pointer_cast<SiconosLink>(this->parentLink);

  SP::BodyDS ds1, ds2;

  // If both links exist, then create a joint between the two links.
  if (siconosChildLink && siconosParentLink)
  {
    ds1 = siconosParentLink->GetSiconosBodyDS();
    ds2 = siconosChildLink->GetSiconosBodyDS();
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (siconosChildLink)
  {
    ds1 = siconosChildLink->GetSiconosBodyDS();
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (siconosParentLink)
  {
    ds1 = siconosParentLink->GetSiconosBodyDS();
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "unable to create siconos hinge without links.\n";
    return false;
  }

  GZ_ASSERT(this->siconosWorld, "SiconosWorld pointer is NULL");

  // Create and link joint interactions
  this->SiconosConnectJoint(ds1, ds2);

  // Don't process the other relations if something went wrong.
  if (!IsConnected()) {
    gzerr << "Joint " << GetName() << " did not connect correctly.";
    return false;
  }

  // Add Stop interactions (must already be created in SetUpperLimit)
  for (auto& stop : this->upperStops)
  {
    GZ_ASSERT(stop.interaction, "an upper stop interaction was empty");
    this->siconosWorld->GetSimulation()->link(stop.interaction, ds1, ds2);
  }

  // Add Friction interactions (must already be created in SetLowerLimit)
  for (auto& stop : this->lowerStops)
  {
    GZ_ASSERT(stop.interaction, "a lower stop interaction was empty");
    this->siconosWorld->GetSimulation()->link(stop.interaction, ds1, ds2);
  }

  // Add Friction interactions (if already created in SetFriction)
  for (auto& fr : this->jointFriction)
  {
    if (fr.interaction)
      this->siconosWorld->GetSimulation()->link(fr.interaction, ds1, ds2);
  }

  return true;
}

//////////////////////////////////////////////////
void SiconosJoint::SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2)
{
  // Create and link Joint interactions; we assume that if the
  // Interaction pointer is non-empty, it is linked.
  for (auto & ri : this->relInterPairs)
  {
    ri.relation->setBasePositions(ds1->q(), ds2 ? ds2->q() : SP::SiconosVector());

    GZ_ASSERT(!ri.interaction, "joint already connected");

    // Create a Siconos Interacton with an EqualityConditionNSL
    int nc = this->Relation(0)->numberOfConstraints();
    ri.interaction = std11::make_shared<::Interaction>(
      std11::make_shared<EqualityConditionNSL>(nc), ri.relation);

    // Add the interaction to the NSDS
    this->siconosWorld->GetSimulation()->link(ri.interaction, ds1, ds2);
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SiconosDisconnect()
{
  if (!IsConnected())
    return;

  // Disconnect upper stop interactions
  for (auto& stop : this->upperStops)
  {
    if (stop.interaction)
      this->siconosWorld->GetSimulation()->unlink(stop.interaction);
  }

  // Disconnect lower stop interactions
  for (auto& stop : this->lowerStops)
  {
    if (stop.interaction)
      this->siconosWorld->GetSimulation()->unlink(stop.interaction);
  }

  // Disconnect friction interactions
  for (auto& fr : this->jointFriction)
  {
    if (fr.interaction)
      this->siconosWorld->GetSimulation()->unlink(fr.interaction);
  }

  // Disconnect joint interactions, destroy them to indicate
  // disconnected status.
  for (auto & ri : this->relInterPairs)
  {
    if (ri.interaction)
      this->siconosWorld->GetSimulation()->unlink(ri.interaction);
    ri.interaction.reset();
  }
}

//////////////////////////////////////////////////
SP::NewtonEulerJointR SiconosJoint::Relation(unsigned int _index) const
{
  GZ_ASSERT(relInterPairs.size() == 0 || relInterPairs.size() == 1,
            "should be overridden for joints with multiple relations");
  GZ_ASSERT(_index == 0, "this joint has a single relation");

  if (relInterPairs.size() > 0)
    return relInterPairs[0].relation;
  else
    return SP::NewtonEulerJointR();
}

//////////////////////////////////////////////////
SP::Interaction SiconosJoint::Interaction(unsigned int _index) const
{
  GZ_ASSERT(relInterPairs.size() == 0 || relInterPairs.size() == 1,
            "should be overridden for joints with multiple relations");
  GZ_ASSERT(_index == 0, "this joint has a single relation");

  if (relInterPairs.size() > 0)
    return relInterPairs[0].interaction;
  else
    return SP::Interaction();
}
