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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosFixedJoint.hh"

#include <boost/make_shared.hpp>

#include "SiconosWorld.hh"
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/Model.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/FixedJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosFixedJoint::SiconosFixedJoint(BasePtr _parent, SP::SiconosWorld _world)
  : FixedJoint<SiconosJoint>(_parent)
{
  siconosWorld = _world;
  GZ_ASSERT(siconosWorld, "SiconosWorld pointer is NULL");
}

//////////////////////////////////////////////////
SiconosFixedJoint::~SiconosFixedJoint()
{
}

//////////////////////////////////////////////////
void SiconosFixedJoint::Load(sdf::ElementPtr _sdf)
{
  FixedJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosFixedJoint::Init()
{
  FixedJoint<SiconosJoint>::Init();

  // Throw an error if no links are given.
  if (!this->childLink && !this->parentLink)
  {
    gzerr << "unable to create siconos hinge without links.\n";
    return;
  }

  this->siconosFixedJointR = std11::make_shared<FixedJointR>();

  if (!this->siconosFixedJointR)
  {
    gzerr << "unable to create siconos hinge constraint\n";
    return;
  }

  // Put the relation in our pair list, associated Interaction will be
  // initialized during SiconosConnect().
  this->relInterPairs.clear();
  this->relInterPairs.push_back({this->siconosFixedJointR, SP::Interaction()});

  // Allows access to impulse
  // this->siconosHinge->enableFeedback(true);

  // Connect the dynamical systems in the Siconos graph
  this->SiconosConnect();
}

//////////////////////////////////////////////////
void SiconosFixedJoint::SetAxis(unsigned int /*_index*/,
    const ignition::math::Vector3d &/*_axis*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double SiconosFixedJoint::PositionImpl(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void SiconosFixedJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetVelocity that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double SiconosFixedJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GetVelocity that is not valid for joints of type fixed.\n";
  return 0.0;
}

//////////////////////////////////////////////////
void SiconosFixedJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetForceImpl that is not valid for joints of type fixed.\n";
  return;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosFixedJoint::GlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GlobalAxis that is not valid for joints of type fixed.\n";
  return ignition::math::Vector3d();
}
