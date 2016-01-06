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
SiconosFixedJoint::SiconosFixedJoint(SP::Model _world, BasePtr _parent)
    : FixedJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
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
  math::Vector3 axis = math::Vector3::UnitZ;

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3 pivotParent, pivotChild, axisParent, axisChild;
  math::Pose pose;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.rot.RotateVectorReverse(pivotParent);
    axisParent = pose.rot.RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.rot.RotateVectorReverse(pivotChild);
    axisChild = pose.rot.RotateVectorReverse(axis);
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
  GZ_ASSERT(this->siconosWorld, "siconos world pointer is NULL");
  //this->siconosWorld->addConstraint(this->siconosFixed, true);

  // Allows access to impulse
  //this->siconosFixed->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
void SiconosFixedJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetAxis that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
math::Angle SiconosFixedJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GetAngleImpl that is not valid for joints of type fixed.\n";
  return math::Angle();
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
void SiconosFixedJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetMaxForce that is not valid for joints of type fixed.\n";
}

//////////////////////////////////////////////////
double SiconosFixedJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GetMaxForce that is not valid for joints of type fixed.\n";
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
bool SiconosFixedJoint::SetHighStop(unsigned int /*_index*/,
                      const math::Angle &/*_angle*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetHighStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
bool SiconosFixedJoint::SetLowStop(unsigned int /*_index*/,
                     const math::Angle &/*_angle*/)
{
  gzwarn << "SiconosFixedJoint: called method "
         << "SetLowStop that is not valid for joints of type fixed.\n";
  return false;
}

//////////////////////////////////////////////////
math::Vector3 SiconosFixedJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzwarn << "SiconosFixedJoint: called method "
         << "GetGlobalAxis that is not valid for joints of type fixed.\n";
  return math::Vector3();
}
