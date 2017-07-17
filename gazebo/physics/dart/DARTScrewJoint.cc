/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include <string>
#include <boost/bind.hpp>
#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTScrewJoint::DARTScrewJoint(BasePtr _parent)
  : ScrewJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTScrewJoint::~DARTScrewJoint()
{
}

//////////////////////////////////////////////////
void DARTScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<DARTJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::ScrewJoint::Properties(
          *this->dataPtr->dtProperties.get()));
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetAnchor(const unsigned int /*index*/,
    const ignition::math::Vector3d &/*_anchor*/)
{
  gzerr << "DARTScrewJoint::SetAnchor not implemented.\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::Init()
{
  ScrewJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTScrewJoint::Anchor(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Anchor" + std::to_string(_index));
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
      this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3Ign(worldOrigin);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTScrewJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Axis" + std::to_string(_index));
  }

  Eigen::Vector3d globalAxis = Eigen::Vector3d::UnitX();

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index < this->DOF())
  {
    dart::dynamics::ScrewJoint *dtScrewJoint =
        dynamic_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);
    GZ_ASSERT(dtScrewJoint, "ScrewJoint is NULL");

    Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
    Eigen::Vector3d axis = dtScrewJoint->getAxis();
    globalAxis = T.linear() * axis;
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  // TODO: Issue #494
  // See: https://bitbucket.org/osrf/gazebo/issue/494
  // joint-axis-reference-frame-doesnt-match
  return DARTTypes::ConvVec3Ign(globalAxis);
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Axis" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetAxis, this, _index, _axis));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index < this->DOF())
  {
    dart::dynamics::ScrewJoint *dtScrewJoint =
        dynamic_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);
    GZ_ASSERT(dtScrewJoint, "ScrewJoint is NULL");

    // TODO: Issue #494
    // See: https://bitbucket.org/osrf/gazebo/issue/494
    // joint-axis-reference-frame-doesnt-match
    Eigen::Vector3d dartVec3 = DARTTypes::ConvVec3(_axis);
    Eigen::Isometry3d dartTransfJointLeftToParentLink
        = this->dataPtr->dtJoint->getTransformFromParentBodyNode().inverse();
    dartVec3 = dartTransfJointLeftToParentLink.linear() * dartVec3;

    dtScrewJoint->setAxis(dartVec3);
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetVelocity(unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<double>(
          "Velocity" + std::to_string(_index));
  }

  double result = 0.0;

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
    result = this->dataPtr->dtJoint->getVelocity(0);
  else if (_index == 1)
    gzerr << "DARTScrewJoint::GetVelocity: Not implemented for index[1].\n";
  else
    gzerr << "Invalid index[" << _index << "]\n";

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetVelocity(unsigned int _index, double _vel)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Velocity" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetVelocity, this, _index, _vel));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
    this->dataPtr->dtJoint->setVelocity(0, _vel);
  else if (_index == 1)
    gzerr << "DARTScrewJoint::SetVelocity: Not implemented for index[1].\n";
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetThreadPitch(unsigned int _index, double _threadPitch)
{
  if (_index >= this->DOF())
    gzerr << "Invalid index[" << _index << "]\n";

  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetThreadPitch(double _threadPitch)
{
  this->threadPitch = _threadPitch;
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "ThreadPitch",
        boost::bind(&DARTScrewJoint::SetThreadPitch, this, _threadPitch));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  dart::dynamics::ScrewJoint *dtScrewJoint =
      dynamic_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);
  GZ_ASSERT(dtScrewJoint, "ScrewJoint is NULL");

  this->threadPitch = _threadPitch;
  dtScrewJoint->setPitch(DARTTypes::InvertThreadPitch(_threadPitch));
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetThreadPitch(unsigned int _index)
{
  if (_index >= this->DOF())
    gzerr << "Invalid index[" << _index << "]\n";

  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetThreadPitch()
{
  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");
  GZ_ASSERT(
    !this->dataPtr->IsInitialized() ||
    (std::abs(dynamic_cast<dart::dynamics::ScrewJoint *>(
      this->dataPtr->dtJoint)->getPitch() -
      DARTTypes::InvertThreadPitch(this->threadPitch)) < 1e-6),
    "Gazebo and DART disagree in thread pitch.");

  dart::dynamics::ScrewJoint *dtScrewJoint =
      dynamic_cast<dart::dynamics::ScrewJoint *>(this->dataPtr->dtJoint);
  GZ_ASSERT(dtScrewJoint, "ScrewJoint is NULL");

  double result = this->threadPitch;
  if (dtScrewJoint)
    result = DARTTypes::InvertThreadPitch(dtScrewJoint->getPitch());
  else
    gzwarn << "dartScrewJoint not created yet, returning cached threadPitch.\n";

  return result;
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  }

  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<double>(_key + std::to_string(_index));

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_key  == "thread_pitch")
  {
    return this->GetThreadPitch();
  }
  else if (_key == "friction")
  {
    if (_index == 0)
    {
      return this->dataPtr->dtJoint->getCoulombFriction(_index);
    }
    else if (_index == 1)
    {
      gzerr << "DARTScrewJoint::GetParam(friction): "
            << "Not implemented for index[1].\n";
      return false;
    }
    else
    {
      gzerr << "Should never be here. Joint index invalid limit not set.\n";
      return false;
    }
  }

  return DARTJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
bool DARTScrewJoint::SetParam(const std::string &_key,
                              unsigned int _index,
                              const boost::any &_value)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return false;
  }

  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
          _key + std::to_string(_index),
          boost::bind(&DARTScrewJoint::SetParam, this, _key, _index, _value));
    return true;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  // try because boost::any_cast can throw
  try
  {
    if (_key  == "thread_pitch")
    {
      this->SetThreadPitch(boost::any_cast<double>(_value));
      return true;
    }
    else if (_key == "friction")
    {
      if (_index == 0)
      {
        this->dataPtr->dtJoint->setCoulombFriction(
              _index, boost::any_cast<double>(_value));
        return true;
      }
      else if (_index == 1)
      {
        gzerr << "DARTScrewJoint::SetParam(friction): "
              << "Not implemented for index[1].\n";
        return false;
      }
      else
      {
        gzerr << "Should never be here. Joint index invalid limit not set.\n";
        return false;
      }
    }
  }
  catch(const boost::bad_any_cast &_e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << _e.what()
          << std::endl;
    return false;
  }

  return DARTJoint::SetParam(_key, _index, _value);
}

//////////////////////////////////////////////////
double DARTScrewJoint::PositionImpl(const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
    return this->dataPtr->GetCached<double>("Angle");

  double result = ignition::math::NAN_D;

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
  {
    // angular position
    result = this->dataPtr->dtJoint->getPosition(0);
  }
  else if (_index == 1)
  {
    // linear position
    const double radianAngle = this->dataPtr->dtJoint->getPosition(0);
    result = -radianAngle / const_cast<DARTScrewJoint*>(this)->GetThreadPitch();
    // TODO: The ScrewJoint::GetThreadPitch() function is not const. As a
    // workaround, we use const_cast here until #1686 is resolved.
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
  }

  return result;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (!this->dataPtr->IsInitialized())
  {
    this->dataPtr->Cache(
        "Force" + std::to_string(_index),
        boost::bind(&DARTScrewJoint::SetForceImpl, this, _index, _effort));
    return;
  }

  GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");

  if (_index == 0)
    this->dataPtr->dtJoint->setForce(0, _effort);
  else if (_index == 1)
    gzerr << "DARTScrewJoint::SetForceImpl: Not implemented for index[1].\n";
  else
    gzerr << "Invalid index[" << _index << "]\n";
}

//////////////////////////////////////////////////
double DARTScrewJoint::UpperLimit(const unsigned int _index) const
{
  switch (_index)
  {
    case 0:
      if (!this->dataPtr->IsInitialized())
      {
        return this->dataPtr->GetCached<double>(
              "HighStop" + std::to_string(_index));
      }

      GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");
      return this->dataPtr->dtJoint->getPositionUpperLimit(0);
    case 1:
      gzerr << "DARTScrewJoint::UpperLimit: Not implemented for index[1].\n";
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };

  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double DARTScrewJoint::LowerLimit(const unsigned int _index) const
{
  switch (_index)
  {
    case 0:
      if (!this->dataPtr->IsInitialized())
      {
        return this->dataPtr->GetCached<double>(
              "LowStop" + std::to_string(_index));
      }
      GZ_ASSERT(this->dataPtr->dtJoint, "DART joint is nullptr.");
      return this->dataPtr->dtJoint->getPositionLowerLimit(0);
    case 1:
      gzerr << "DARTScrewJoint::LowerLimit: Not implemented for index[1].\n";
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };

  return ignition::math::NAN_D;
}
