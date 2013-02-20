/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: A bullet slider or primastic joint
 * Author: Nate Koenig
 * Date: 13 Oct 2009
 */

#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/bullet/bullet_inc.h"
#include "physics/bullet/BulletLink.hh"
#include "physics/bullet/BulletPhysics.hh"
#include "physics/bullet/BulletSliderJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletSliderJoint::BulletSliderJoint(btDynamicsWorld *_world, BasePtr _parent)
    : SliderJoint<BulletJoint>(_parent)
{
  this->bulletWorld = _world;
  this->bulletSlider = NULL;
}

//////////////////////////////////////////////////
BulletSliderJoint::~BulletSliderJoint()
{
}

//////////////////////////////////////////////////
void BulletSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<BulletJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void BulletSliderJoint::Attach(LinkPtr _one, LinkPtr _two)
{
  if (this->constraint)
    this->Detach();

  SliderJoint<BulletJoint>::Attach(_one, _two);

  BulletLinkPtr bulletChildLink =
    boost::shared_static_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::shared_static_cast<BulletLink>(this->parentLink);


  btVector3 anchor, axis1, axis2;
  btTransform frame1, frame2;
  frame1 = btTransform::getIdentity();
  frame2 = btTransform::getIdentity();

  // Get axis from sdf.
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  math::Vector3 axis = axisElem->GetValueVector3("xyz");

  math::Vector3 pivotA, pivotB;
  math::Pose pose;

  pivotA = this->anchorPos;
  pivotB = this->anchorPos;
  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotA -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotA = pose.rot.RotateVectorReverse(pivotA);
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotB -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotB = pose.rot.RotateVectorReverse(pivotB);
  }

  std::cout << "AnchorPos[" << this->anchorPos << "]\n";
  std::cout << "Slider PivotA[" << pivotA << "] PivotB[" << pivotB << "]\n";

  frame1.setOrigin(btVector3(pivotA.x, pivotA.y, pivotA.z));
  frame2.setOrigin(btVector3(pivotB.x, pivotB.y, pivotB.z));

  frame1.getBasis().setEulerZYX(0, M_PI*0.5, 0);
  frame2.getBasis().setEulerZYX(0, M_PI*0.5, 0);

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        frame1, frame2, true);
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletChildLink->GetBulletLink(), frame2, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletSlider = new btSliderConstraint(
        *bulletParentLink->GetBulletLink(), frame1, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzthrow("joint without links\n");
  }

   this->bulletSlider->setLowerAngLimit(0.0);
   this->bulletSlider->setUpperAngLimit(0.0);

  this->constraint = this->bulletSlider;

  // Add the joint to the world
  this->bulletWorld->addConstraint(this->bulletSlider, true);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetVelocity(int /*_index*/) const
{
  double result = 0;
  if (this->bulletSlider)
    result = this->bulletSlider->getTargetLinMotorVelocity();
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetVelocity(int /*_index*/, double _angle)
{
  if (this->bulletSlider)
    this->bulletSlider->setTargetLinMotorVelocity(_angle);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in bullet\n";
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetDamping(int /*index*/, const double _damping)
{
  if (this->bulletSlider)
    this->bulletSlider->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetForce(int /*_index*/, double _force)
{
  /*btVector3 hingeAxisLocal = this->bulletSlider->getAFrame().getBasis().getColumn(2); // z-axis of constraint frame
  btVector3 hingeAxisWorld = this->bulletSlider->getRigidBodyA().getWorldTransform().getBasis() * hingeAxisLocal;

  btVector3 hingeTorque = _torque * hingeAxisWorld;
  */

  btVector3 force(0, 0, _force);
  this->constraint->getRigidBodyA().applyCentralForce(force);
  this->constraint->getRigidBodyB().applyCentralForce(-force);
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetHighStop(int /*_index*/,
                                    const math::Angle &_angle)
{
  if (this->bulletSlider)
    this->bulletSlider->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetLowStop(int /*_index*/,
                                   const math::Angle &_angle)
{
  if (this->bulletSlider)
    this->bulletSlider->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetHighStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getUpperLinLimit();
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetLowStop(int /*_index*/)
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLowerLinLimit();
  return result;
}

//////////////////////////////////////////////////
void BulletSliderJoint::SetMaxForce(int /*_index*/, double _force)
{
  if (this->bulletSlider)
    this->bulletSlider->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletSliderJoint::GetMaxForce(int /*_index*/)
{
  double result = 0;
  if (this->bulletSlider)
    result = this->bulletSlider->getMaxLinMotorForce();
  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  math::Vector3 result;
  if (this->bulletSlider)
  {
    // I have not verified the following math, though I based it on internal
    // bullet code at line 250 of btHingeConstraint.cpp
    btVector3 vec =
      this->bulletSlider->getRigidBodyA().getCenterOfMassTransform().getBasis() *
      this->bulletSlider->getFrameOffsetA().getBasis().getColumn(2);
    result = BulletTypes::ConvertVector3(vec);
  }
  else
    gzwarn << "bulletHinge does not exist, returning fake axis\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletSliderJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->bulletSlider)
    result = this->bulletSlider->getLinearPos();
  return result;
}
