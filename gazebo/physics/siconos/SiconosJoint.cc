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

  if (this->interaction)
    this->siconosWorld->GetSimulation()->unlink(this->interaction);
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
void SiconosJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr SiconosJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;

  if (!this->interaction)
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
  return this->interaction && ((this->childLink.get() == _one.get() &&
                               this->parentLink.get() == _two.get()) ||
                              (this->childLink.get() == _two.get() &&
                               this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void SiconosJoint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();
  this->siconosWorld->GetSimulation()->unlink(this->interaction);
  this->Relation().reset();
  this->interaction.reset();

  for (auto& stop : this->upperStops)
  {
    if (stop.interaction)
      this->siconosWorld->GetSimulation()->unlink(stop.interaction);
    stop.relation.reset();
    stop.interaction.reset();
  }

  for (auto& stop : this->lowerStops)
  {
    if (stop.interaction)
      this->siconosWorld->GetSimulation()->unlink(stop.interaction);
    stop.relation.reset();
    stop.interaction.reset();
  }
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
  GZ_ASSERT(!this->interaction, "interaction should be valid");
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

  if (this->Relation() && _index < this->DOF())
  {
    // Make room for this axis index
    if (_index >= this->upperStops.size())
      this->upperStops.resize(_index+1);

    // Check that we don't already have a stop at the same position
    if (this->upperStops[_index].relation
        && this->upperStops[_index].relation->position(0) == _limit)
      return;

    // Unlink existing Interaction if necessary
    if (this->upperStops[_index].interaction)
      this->siconosWorld->GetSimulation()->unlink(
        this->upperStops[_index].interaction);

    // Create and store the stop Relation and Interaction
    SP::JointStopR rel = std11::make_shared<JointStopR>(
      this->Relation(), _limit, true /* stop direction */, _index /* dof number */);

    SP::Interaction inter = std11::make_shared<::Interaction>(
      std11::make_shared<NewtonImpactNSL>(), rel);

    this->upperStops[_index] = {rel, inter};

    // Get dynamical systems
    SiconosLinkPtr parent = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    SiconosLinkPtr child  = boost::static_pointer_cast<SiconosLink>(this->childLink);
    SP::BodyDS ds1( parent ? parent->GetSiconosBodyDS() : nullptr );
    SP::BodyDS ds2( child  ? child->GetSiconosBodyDS()  : nullptr );

    if (ds2 && !ds1)
      std::swap(ds1, ds2);

    // Link the stop Interaction
    this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);
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

  if (this->Relation() && _index < this->DOF())
  {
    // Make room for this axis index
    if (_index >= this->lowerStops.size())
      this->lowerStops.resize(_index+1);

    // Check that we don't already have a stop at the same position
    if (this->lowerStops[_index].relation
        && this->lowerStops[_index].relation->position(0) == _limit)
      return;

    // Unlink existing Interaction if necessary
    if (this->lowerStops[_index].interaction)
      this->siconosWorld->GetSimulation()->unlink(
        this->lowerStops[_index].interaction);

    // Create and store the stop Relation and Interaction
    SP::JointStopR rel = std11::make_shared<JointStopR>(
      this->Relation(), _limit, false /* stop direction */, _index /* dof number */);

    // Use the global joint stop NSL -- TODO: create per-stop
    // NewtonImpactNSL when restitution parameter available in SDF.
    SP::Interaction inter = std11::make_shared<::Interaction>(
      this->siconosWorld->JointStopNSL(), rel);

    this->lowerStops[_index] = {rel, inter};

    // Get dynamical systems
    SiconosLinkPtr parent = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    SiconosLinkPtr child  = boost::static_pointer_cast<SiconosLink>(this->childLink);
    SP::BodyDS ds1( parent ? parent->GetSiconosBodyDS() : nullptr );
    SP::BodyDS ds2( child  ? child->GetSiconosBodyDS()  : nullptr );

    if (ds2 && !ds1)
      std::swap(ds1, ds2);

    // Link the stop Interaction
    this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);
  }
  else
  {
    gzlog << "Joint relation not yet created.\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetupJointLimits()
{
  GZ_ASSERT(Relation(), "SiconosJoint::Relation was null during limit setup.");

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
  return Joint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
bool SiconosJoint::SetFriction(unsigned int _index, double _value)
{
  if (!this->Relation() || _index >= this->DOF())
  {
    if (this->Relation())
      gzerr << "Invalid index [" << _index << "]" << std::endl;
    else
      gzlog << "Joint relation not yet created.\n";
    return false;
  }

  // Make room for this axis index
  if (_index >= this->jointFriction.size())
    this->jointFriction.resize(_index+1);

  // Check that we don't already have friction at the same value
  if (this->jointFriction[_index].relation
      && this->jointFriction[_index].value == _value)
    return true;

  // Unlink existing Interaction if necessary
  if (this->jointFriction[_index].interaction)
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
    std11::make_shared<JointFrictionR>(this->Relation(), _index);

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

  // Get dynamical systems
  SiconosLinkPtr parent = boost::static_pointer_cast<SiconosLink>(this->parentLink);
  SiconosLinkPtr child  = boost::static_pointer_cast<SiconosLink>(this->childLink);
  SP::BodyDS ds1( parent ? parent->GetSiconosBodyDS() : nullptr );
  SP::BodyDS ds2( child  ? child->GetSiconosBodyDS()  : nullptr );

  if (ds2 && !ds1)
    std::swap(ds1, ds2);

  // Link the friction Interaction
  this->siconosWorld->GetSimulation()->link(inter, ds1, ds2);

  return true;
}

//////////////////////////////////////////////////
bool SiconosJoint::SetPosition(unsigned int _index, double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}
