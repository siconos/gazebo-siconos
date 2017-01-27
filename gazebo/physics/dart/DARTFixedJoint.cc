/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTJointPrivate.hh"
#include "gazebo/physics/dart/DARTFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTFixedJoint::DARTFixedJoint(BasePtr _parent)
  : FixedJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTFixedJoint::~DARTFixedJoint()
{
}

//////////////////////////////////////////////////
void DARTFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<DARTJoint>::Load(_sdf);

  this->dataPtr->dtProperties.reset(
        new dart::dynamics::WeldJoint::Properties(
          *this->dataPtr->dtProperties.get()));
}

//////////////////////////////////////////////////
void DARTFixedJoint::Init()
{
  FixedJoint<DARTJoint>::Init();
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTFixedJoint::Anchor(
    const unsigned int _index) const
{
  if (!this->dataPtr->IsInitialized())
  {
    return this->dataPtr->GetCached<ignition::math::Vector3d>(
          "Anchor" + std::to_string(_index));
  }

  Eigen::Isometry3d T = this->dataPtr->dtChildBodyNode->getTransform() *
                        this->dataPtr->dtJoint->getTransformFromChildBodyNode();
  Eigen::Vector3d worldOrigin = T.translation();

  return DARTTypes::ConvVec3Ign(worldOrigin);
}

//////////////////////////////////////////////////
ignition::math::Vector3d DARTFixedJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "GlobalAxis that is not valid for joints of type fixed.\n";

  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetAxis(const unsigned int /*_index*/,
                             const ignition::math::Vector3d& /*_axis*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
double DARTFixedJoint::PositionImpl(const unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "PositionImpl that is not valid for joints of type fixed.\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_vel*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
double DARTFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzwarn << "DARTFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void DARTFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "DARTFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
}
