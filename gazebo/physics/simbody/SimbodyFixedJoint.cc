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

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyFixedJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyFixedJoint::SimbodyFixedJoint(SimTK::MultibodySystem */*_world*/,
                                     BasePtr _parent)
    : FixedJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyFixedJoint::~SimbodyFixedJoint()
{
}

//////////////////////////////////////////////////
void SimbodyFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzwarn << "SimbodyFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double SimbodyFixedJoint::GetVelocity(unsigned int /*index*/) const
{
  gzwarn << "SimbodyFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void SimbodyFixedJoint::SetForceImpl(unsigned int /*_index*/,
                                     double /*_torque*/)
{
  gzwarn << "SimbodyFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyFixedJoint::GlobalAxis(
    const unsigned int /*index*/) const
{
  gzwarn << "SimbodyFixedJoint: called method "
         << "GlobalAxis that is not valid for joints of type fixed.\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
double SimbodyFixedJoint::PositionImpl(const unsigned int /*_index*/) const
{
  gzwarn << "SimbodyFixedJoint: called method "
         << "PositionImpl that is not valid for joints of type fixed.\n";
  return ignition::math::NAN_D;
}
