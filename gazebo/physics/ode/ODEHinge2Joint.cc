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
/* Desc: A hinge joint with 2 degrees of freedom
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEHinge2Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEHinge2Joint::ODEHinge2Joint(dWorldID _worldId, BasePtr _parent)
    : Hinge2Joint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateHinge2(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEHinge2Joint::~ODEHinge2Joint()
{
  this->applyDamping.reset();
}

//////////////////////////////////////////////////
void ODEHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHinge2Joint::Anchor(
    const unsigned int _index) const
{
  dVector3 result;

  if (this->jointId)
  {
    if (_index == 0)
      dJointGetHinge2Anchor(this->jointId, result);
    else
      dJointGetHinge2Anchor2(this->jointId, result);
  }
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  if (this->jointId)
    dJointSetHinge2Anchor(this->jointId, _anchor.X(), _anchor.Y(), _anchor.Z());
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// ODE needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  ignition::math::Vector3d globalAxis = _axis;
  if (this->parentLink)
  {
    globalAxis =
      this->GetParent()->GetModel()->WorldPose().Rot().RotateVector(
          _axis);
  }

  if (this->jointId)
  {
    if (_index == 0)
    {
      dJointSetHinge2Axis1(this->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
    else
    {
      dJointSetHinge2Axis2(this->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
    }
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHinge2Joint::GlobalAxis(
    const unsigned int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetHinge2Axis1(this->jointId, result);
  else
    dJointGetHinge2Axis2(this->jointId, result);

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
double ODEHinge2Joint::PositionImpl(const unsigned int _index) const
{
  double result = ignition::math::NAN_D;

  if (this->jointId)
  {
    /// \todo Return position of axis 1
    if (_index == 0)
      result = dJointGetHinge2Angle1(this->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEHinge2Joint::GetVelocity(unsigned int _index) const
{
  double result = 0;

  if (this->jointId)
  {
    if (_index == 0)
      result = dJointGetHinge2Angle1Rate(this->jointId);
    else
      result = dJointGetHinge2Angle2Rate(this->jointId);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetVelocity(unsigned int _index, double _angle)
{
  if (_index == 0)
    this->SetParam(dParamVel, _angle);
  else
    this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->jointId)
  {
    if (_index == 0)
      dJointAddHinge2Torques(this->jointId, _effort, 0);
    else
      dJointAddHinge2Torques(this->jointId, 0, _effort);
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEHinge2Joint::GetParam(unsigned int _parameter) const
{
  double result = 0;

  if (this->jointId)
    result = dJointGetHinge2Param(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHinge2Joint::SetParam(unsigned int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  if (this->jointId)
    dJointSetHinge2Param(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
