/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyBallJoint::SimbodyBallJoint(SimTK::MultibodySystem * /*_world*/,
                                   BasePtr _parent)
    : BallJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyBallJoint::~SimbodyBallJoint()
{
}

//////////////////////////////////////////////////
void SimbodyBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyBallJoint::Anchor(
    const unsigned int /*_index*/) const
{
  return this->anchorPos;
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double SimbodyBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
ignition::math::Vector3d SimbodyBallJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
double SimbodyBallJoint::PositionImpl(const unsigned int /*_index*/) const
{
  gzerr << "SimbodyBallJoint::PositionImpl not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetAxis(const unsigned int /*_index*/,
                               const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "SimbodyBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
double SimbodyBallJoint::UpperLimit(const unsigned int /*_index*/) const
{
  gzerr << "SimbodyBallJoint::UpperLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double SimbodyBallJoint::LowerLimit(const unsigned int /*_index*/) const
{
  gzerr << "SimbodyBallJoint::LowerLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetUpperLimit(const unsigned int /*_index*/,
                                     const double /*_limit*/)
{
  gzerr << "SimbodyBallJoint::SetUpperLimit not implemented" << std::endl;
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetLowerLimit(const unsigned int /*_index*/,
                                     const double /*_limit*/)
{
  gzerr << "SimbodyBallJoint::SetLowerLimit not implemented" << std::endl;
}

