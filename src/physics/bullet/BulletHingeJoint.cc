/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A BulletHingeJoint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: BulletHingeJoint.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include "Model.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "World.hh"
#include "BulletLink.hh"
#include "BulletPhysics.hh"
#include "common/XMLConfig.hh"
#include "BulletHingeJoint.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletHingeJoint::BulletHingeJoint(btDynamicsWorld *world )
    : HingeJoint<BulletJoint>()
{
  this->world = world;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletHingeJoint::~BulletHingeJoint()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load a hinge joint
void BulletHingeJoint::Load(common::XMLConfigNode *node)
{
  HingeJoint<BulletJoint>::Load(node);
}


//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void BulletHingeJoint::Attach( Link *one, Link *two )
{
  HingeJoint<BulletJoint>::Attach(one,two);
  BulletLink *bulletLink1 = dynamic_cast<BulletLink*>(this->body1);
  BulletLink *bulletLink2 = dynamic_cast<BulletLink*>(this->body2);

  if (!bulletLink1 || !bulletLink2)
    gzthrow("Requires bullet bodies");

  btRigidLink *rigidLink1 = bulletLink1->GetBulletLink();
  btRigidLink *rigidLink2 = bulletLink2->GetBulletLink();

  math::Vector3 pivotA, pivotB;
  btmath::Vector3 axisA, axisB;

  // Compute the pivot point, based on the anchorPos
  pivotA = (this->anchorPos - this->body1->GetWorldPose().pos);
  pivotB = (this->anchorPos - this->body2->GetWorldPose().pos);

  axisA = btmath::Vector3((**this->axisP).x,(**this->axisP).y,(**this->axisP).z);
  axisB = btmath::Vector3((**this->axisP).x,(**this->axisP).y,(**this->axisP).z);

  this->constraint = new btHingeConstraint( *rigidLink1, *rigidLink2,
      btmath::Vector3(pivotA.x, pivotA.y, pivotA.z),
      btmath::Vector3(pivotB.x, pivotB.y, pivotB.z), axisA, axisB); 

  // Add the joint to the world
  this->world->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);
  ((btHingeConstraint*)this->constraint)->setAngularOnly(true);
}

//////////////////////////////////////////////////////////////////////////////
// Get the anchor point
math::Vector3 BulletHingeJoint::GetAnchor(int index ) const
{
  btTransform trans = ((btHingeConstraint*)this->constraint)->getAFrame();
  trans.getOrigin() += this->constraint->getRigidLinkA().getCenterOfMassTransform().getOrigin();
  return math::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
}

//////////////////////////////////////////////////////////////////////////////
// Set the anchor point
void BulletHingeJoint::SetAnchor( int index, const math::Vector3 &anchor )
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the axis of rotation
math::Vector3 BulletHingeJoint::GetAxis(int index) const
{
  return (**this->axisP);
}

//////////////////////////////////////////////////////////////////////////////
// Set the axis of rotation
void BulletHingeJoint::SetAxis( int index, const math::Vector3 &axis )
{
  gzerr << "Bullet handles setAxis improperly\n";
  // Bullet seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btmath::Vector3 vec(axis.x, axis.y, axis.z);
  ((btHingeConstraint*)this->constraint)->setAxis(vec);
  */
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint damping
void BulletHingeJoint::SetDamping( int /*index*/, const double damping )
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the angle of rotation
math::Angle BulletHingeJoint::GetAngle(int index ) const
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getHingemath::Angle();
  else
    gzthrow("Joint has not been created");
}

//////////////////////////////////////////////////////////////////////////////
/// Set the velocity of an axis(index).
void BulletHingeJoint::SetVelocity(int index, double angle)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
// Get the rotation rate
double BulletHingeJoint::GetVelocity(int index) const
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the max allowed force of an axis(index).
void BulletHingeJoint::SetMaxForce(int index, double t)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the max allowed force of an axis(index).
double BulletHingeJoint::GetMaxForce(int index)
{
  gzerr << "Not implemented\n";
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Set the torque of this joint
void BulletHingeJoint::SetForce(int index, double torque)
{
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////////////////////////////////
/// Get the torque of a joint.
double BulletHingeJoint::GetForce(int index)
{
  gzerr << "Not implemented...\n";
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index).
void BulletHingeJoint::SetHighStop(int index, math::Angle angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day 
    // implement. Be warned that this function will reset them to default
    // settings
    ((btHingeConstraint*)this->constraint)->setLimit( 
      this->GetLowStop(index).GetAsRadian(), angle.GetAsRadian() );
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index).
void BulletHingeJoint::SetLowStop(int index, math::Angle angle)
{
  if (this->constraint)
    // this function has additional parameters that we may one day 
    // implement. Be warned that this function will reset them to default
    // settings
    ((btHingeConstraint*)this->constraint)->setLimit( angle.GetAsRadian(), 
      this->GetHighStop(index).GetAsRadian() );
  else
    gzthrow("Joint must be created first");

}
 
//////////////////////////////////////////////////////////////////////////////
/// Get the high stop of an axis(index).
math::Angle BulletHingeJoint::GetHighStop(int index)
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getUpperLimit();
  else
    gzthrow("Joint must be created first");
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Get the low stop of an axis(index).
math::Angle BulletHingeJoint::GetLowStop(int index)
{
  if (this->constraint)
    return ((btHingeConstraint*)this->constraint)->getLowerLimit();
  else
    gzthrow("Joint must be created first");
}
