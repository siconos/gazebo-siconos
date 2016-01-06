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

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosJoint::SiconosJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->constraint = NULL;
  this->siconosWorld = NULL;
  //this->feedback = NULL;
  this->stiffnessDampingInitialized = false;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
SiconosJoint::~SiconosJoint()
{
  delete this->constraint;
  this->constraint = NULL;
  //delete this->feedback;
  //this->feedback = NULL;
  this->siconosWorld = NULL;
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

  if (this->constraint == NULL)
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
  return this->constraint && ((this->childLink.get() == _one.get() &&
                               this->parentLink.get() == _two.get()) ||
                              (this->childLink.get() == _two.get() &&
                               this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void SiconosJoint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();
  // if (this->constraint && this->siconosWorld)
  //   this->siconosWorld->removeConstraint(this->constraint);
  delete this->constraint;
  this->constraint = NULL;
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
  GZ_ASSERT(this->constraint != NULL, "constraint should be valid");
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
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "SiconosJoint::SetDamping: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetStiffness(unsigned int _index, double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "SiconosJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
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
  if (_index < this->GetAngleCount())
  {
    if (this->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->GetSimTime();
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
  if (_index < this->GetAngleCount())
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
  for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->GetAngle(i).Radian());

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
void SiconosJoint::SetAnchor(unsigned int /*_index*/,
    const gazebo::math::Vector3 & /*_anchor*/)
{
  // nothing to do here for siconos.
}

//////////////////////////////////////////////////
math::Vector3 SiconosJoint::GetAnchor(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 SiconosJoint::GetLinkForce(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 SiconosJoint::GetLinkTorque(unsigned int /*_index*/) const
{
  gzerr << "Not implement in Siconos\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
bool SiconosJoint::SetParam(const std::string &/*_key*/,
    unsigned int /*_index*/,
    const boost::any &/*_value*/)
{
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
math::Angle SiconosJoint::GetHighStop(unsigned int _index)
{
  return this->GetUpperLimit(_index);
}

//////////////////////////////////////////////////
math::Angle SiconosJoint::GetLowStop(unsigned int _index)
{
  return this->GetLowerLimit(_index);
}

//////////////////////////////////////////////////
bool SiconosJoint::SetPosition(unsigned int _index, double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}
