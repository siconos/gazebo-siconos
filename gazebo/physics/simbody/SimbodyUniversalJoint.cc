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

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyUniversalJoint::SimbodyUniversalJoint(SimTK::MultibodySystem* /*_world*/,
  BasePtr _parent) : UniversalJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyUniversalJoint::~SimbodyUniversalJoint()
{
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::Load(sdf::ElementPtr _sdf)
{
  UniversalJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyUniversalJoint::Anchor(
    const unsigned int /*_index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetAxis(const unsigned int /*_index*/,
                                   const ignition::math::Vector3d &/*_axis*/)
{
  /// Universal Joint are built in SimbodyPhysics.cc, so this init block
  /// actually does nothing.
  gzdbg << "SetAxis: setting axis is not yet implemented. The axes are set "
        << "during joint construction in SimbodyPhyiscs.cc for now.\n";
}


//////////////////////////////////////////////////
double SimbodyUniversalJoint::GetVelocity(unsigned int _index) const
{
  if (_index < this->DOF())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
    {
      return this->mobod.getOneU(
        this->simbodyPhysics->integ->getState(),
        SimTK::MobilizerUIndex(_index));
    }
    else
    {
      gzdbg << "GetVelocity() simbody not yet initialized, "
            << "initial velocity should be zero until restart from "
            << "state has been implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "Invalid index for joint, returning NaN\n";
    return SimTK::NaN;
  }
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetVelocity(unsigned int _index,
    double _rate)
{
  if (_index < this->DOF())
  {
    this->mobod.setOneU(
      this->simbodyPhysics->integ->updAdvancedState(),
      SimTK::MobilizerUIndex(_index), _rate);
    this->simbodyPhysics->system.realize(
      this->simbodyPhysics->integ->getAdvancedState(), SimTK::Stage::Velocity);
  }
  else
  {
    gzerr << "SetVelocity _index too large.\n";
  }
}

//////////////////////////////////////////////////
void SimbodyUniversalJoint::SetForceImpl(unsigned int _index,
    double _torque)
{
  if (_index < this->DOF() && this->physicsInitialized)
  {
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyUniversalJoint::GlobalAxis(
    unsigned int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < this->DOF())
  {
    if (!this->mobod.isEmptyHandle())
    {
      if (_index == UniversalJoint::AXIS_PARENT)
      {
        // express X-axis of X_IF in world frame
        const SimTK::Transform &X_IF = this->mobod.getInboardFrame(
          this->simbodyPhysics->integ->getState());

        SimTK::Vec3 x_W(
          this->mobod.getParentMobilizedBody().expressVectorInGroundFrame(
          this->simbodyPhysics->integ->getState(), X_IF.x()));

        return SimbodyPhysics::Vec3ToVector3Ign(x_W);
      }
      else if (_index == UniversalJoint::AXIS_CHILD)
      {
        // express Y-axis of X_OM in world frame
        const SimTK::Transform &X_OM = this->mobod.getOutboardFrame(
          this->simbodyPhysics->integ->getState());

        SimTK::Vec3 y_W(
          this->mobod.expressVectorInGroundFrame(
          this->simbodyPhysics->integ->getState(), X_OM.y()));

        return SimbodyPhysics::Vec3ToVector3Ign(y_W);
      }
      else
      {
        gzerr << "GlobalAxis: internal error, DOF < 0.\n";
        return ignition::math::Vector3d(SimTK::NaN, SimTK::NaN, SimTK::NaN);
      }
    }
    else
    {
      gzerr << "Joint mobod not initialized correctly.  Returning"
            << " initial axis vector in world frame (not valid if"
            << " joint frame has moved). Please file"
            << " a report on issue tracker.\n";
      return this->AxisFrame(_index).RotateVector(
        this->LocalAxis(_index));
    }
  }
  else
  {
    if (_index >= this->DOF())
    {
      gzerr << "index out of bound\n";
      return ignition::math::Vector3d(SimTK::NaN, SimTK::NaN, SimTK::NaN);
    }
    else
    {
      gzdbg << "GlobalAxis() sibmody physics engine not yet initialized, "
            << "use local axis and initial pose to compute "
            << "global axis.\n";
      // if local axis specified in model frame (to be changed)
      // switch to code below if issue #494 is to be addressed
      return this->AxisFrame(_index).RotateVector(
        this->LocalAxis(_index));
    }
  }
}

//////////////////////////////////////////////////
double SimbodyUniversalJoint::PositionImpl(const unsigned int _index) const
{
  if (_index < this->DOF())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
    {
      if (!this->mobod.isEmptyHandle())
      {
        return this->mobod.getOneQ(
          this->simbodyPhysics->integ->getState(), _index);
      }
      else
      {
        gzerr << "Joint mobod not initialized correctly.  Please file"
              << " a report on issue tracker.\n";
        return ignition::math::NAN_D;
      }
    }
    else
    {
      gzdbg << "PositionImpl(): simbody not yet initialized, "
            << "initial angle should be zero until <initial_angle> "
            << "is implemented.\n";
      return 0.0;
    }
  }
  else
  {
    gzerr << "Invalid index[" << _index << "]\n";
    return ignition::math::NAN_D;
  }
}
