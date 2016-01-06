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

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosFixedJoint::SiconosFixedJoint(SP::SiconosWorld _world, BasePtr _parent)
    : FixedJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "SiconosWorld pointer is NULL");
  this->siconosWorld = _world;
  this->siconosFixed = NULL;
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

  // Cast to SiconosLink
  SiconosLinkPtr siconosChildLink =
      boost::static_pointer_cast<SiconosLink>(this->childLink);
  SiconosLinkPtr siconosParentLink =
      boost::static_pointer_cast<SiconosLink>(this->parentLink);

  // Get axis unit vector (expressed in world frame).
  ignition::math::Vector3d axis = ignition::math::Vector3d::UnitZ;

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  ignition::math::Vector3d pivotParent, pivotChild, axisParent, axisChild;
  ignition::math::Pose3d pose;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->WorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.Rot().RotateVectorReverse(pivotParent);
    axisParent = pose.Rot().RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->WorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.Pos();
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.Rot().RotateVectorReverse(pivotChild);
    axisChild = pose.Rot().RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
  }

  // If both links exist, then create a joint between the two links.
  if (siconosChildLink && siconosParentLink)
  {
    // this->siconosFixed = new btHingeConstraint(
    //     *(siconosChildLink->GetSiconosLink()),
    //     *(siconosParentLink->GetSiconosLink()),
    //     SiconosTypes::ConvertVector3(pivotChild),
    //     SiconosTypes::ConvertVector3(pivotParent),
    //     SiconosTypes::ConvertVector3(axisChild),
    //     SiconosTypes::ConvertVector3(axisParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (siconosChildLink)
  {
    // this->siconosFixed = new btHingeConstraint(
    //     *(siconosChildLink->GetSiconosLink()),
    //     SiconosTypes::ConvertVector3(pivotChild),
    //     SiconosTypes::ConvertVector3(axisChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (siconosParentLink)
  {
    // this->siconosFixed = new btHingeConstraint(
    //     *(siconosParentLink->GetSiconosLink()),
    //     SiconosTypes::ConvertVector3(pivotParent),
    //     SiconosTypes::ConvertVector3(axisParent));
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "unable to create siconos hinge without links.\n";
    return;
  }

  if (!this->siconosFixed)
  {
    gzerr << "unable to create siconos hinge constraint\n";
    return;
  }

  // Give parent class SiconosJoint a pointer to this constraint.
  this->constraint = this->siconosFixed;

  //this->siconosFixed->setLimit(0.0, 0.0);

  // Add the joint to the world
  GZ_ASSERT(this->siconosWorld, "SiconosWorld pointer is NULL");
  //this->siconosWorld->addConstraint(this->siconosFixed, true);

  // Allows access to impulse
  //this->siconosFixed->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
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
void SiconosFixedJoint::SetUpperLimit(const unsigned int /*_index*/,
                                      const double /*_limit*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetUpperLimit that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
void SiconosFixedJoint::SetLowerLimit(const unsigned int /*_index*/,
                                      const double /*_limit*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetLowerLimit that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosFixedJoint::GlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GlobalAxis that is not valid for joints of type fixed.\n";
  return ignition::math::Vector3d();
}
