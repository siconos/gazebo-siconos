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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyHingeJoint::SimbodyHingeJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
    : HingeJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyHingeJoint::~SimbodyHingeJoint()
{
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetAxis(const unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  // Simbody seems to handle setAxis improperly. It readjust all the pivot
  // points
  gzdbg << "SetAxis: setting axis is not yet implemented.  The axis are set "
        << " during joint construction in SimbodyPhysics.cc for now.\n";
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::SetVelocity(unsigned int _index, double _rate)
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
    gzerr << "SetVelocity _index too large.\n";
}

//////////////////////////////////////////////////
double SimbodyHingeJoint::GetVelocity(unsigned int _index) const
{
  if (_index < this->DOF())
  {
    if (this->physicsInitialized &&
        this->simbodyPhysics->simbodyPhysicsInitialized)
      return this->mobod.getOneU(
        this->simbodyPhysics->integ->getState(),
        SimTK::MobilizerUIndex(_index));
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
void SimbodyHingeJoint::SetForceImpl(unsigned int _index, double _torque)
{
  if (_index < this->DOF() && this->physicsInitialized)
    this->simbodyPhysics->discreteForces.setOneMobilityForce(
      this->simbodyPhysics->integ->updAdvancedState(),
      this->mobod, SimTK::MobilizerUIndex(_index), _torque);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyHingeJoint::GlobalAxis(
    const unsigned int _index) const
{
  if (this->simbodyPhysics &&
      this->simbodyPhysics->simbodyPhysicsStepped &&
      _index < this->DOF())
  {
    if (!this->mobod.isEmptyHandle())
    {
      const SimTK::Transform &X_OM = this->mobod.getOutboardFrame(
        this->simbodyPhysics->integ->getState());

      // express Z-axis of X_OM in world frame
      SimTK::Vec3 z_W(this->mobod.expressVectorInGroundFrame(
        this->simbodyPhysics->integ->getState(), X_OM.z()));

      return SimbodyPhysics::Vec3ToVector3Ign(z_W);
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
      gzdbg << "GlobalAxis() sibmody physics engine not initialized yet, "
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
double SimbodyHingeJoint::PositionImpl(const unsigned int _index) const
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
      gzdbg << "PositionImpl() simbody not yet initialized, "
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

//////////////////////////////////////////////////
void SimbodyHingeJoint::SaveSimbodyState(const SimTK::State &_state)
{
  if (!this->mobod.isEmptyHandle())
  {
    if (this->simbodyQ.empty())
      this->simbodyQ.resize(this->mobod.getNumQ(_state));

    if (this->simbodyU.empty())
      this->simbodyU.resize(this->mobod.getNumU(_state));

    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->simbodyQ[i] = this->mobod.getOneQ(_state, i);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->simbodyU[i] = this->mobod.getOneU(_state, i);
  }
  else
  {
    // gzerr << "debug: joint name: " << this->GetScopedName() << "\n";
  }
}

//////////////////////////////////////////////////
void SimbodyHingeJoint::RestoreSimbodyState(SimTK::State &_state)
{
  if (!this->mobod.isEmptyHandle())
  {
    for (unsigned int i = 0; i < this->simbodyQ.size(); ++i)
      this->mobod.setOneQ(_state, i, this->simbodyQ[i]);

    for (unsigned int i = 0; i < this->simbodyU.size(); ++i)
      this->mobod.setOneU(_state, i, this->simbodyU[i]);
  }
  else
  {
    // gzerr << "restoring model [" << this->GetScopedName()
    //       << "] failed due to uninitialized mobod\n";
  }
}
