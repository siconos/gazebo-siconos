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
/* Desc: Bullet motion state class.
 * Author: Nate Koenig
 * Date: 25 May 2009
 */

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletMotionState.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletMotionState::BulletMotionState(LinkPtr _link)
  : btMotionState()
{
  this->link = _link;
}

//////////////////////////////////////////////////
BulletMotionState::~BulletMotionState()
{
  this->link.reset();
}

//////////////////////////////////////////////////
void BulletMotionState::getWorldTransform(btTransform &_cogWorldTrans) const
{
  _cogWorldTrans =
    BulletTypes::ConvertPose(this->link->WorldInertialPose());
}

//////////////////////////////////////////////////
void BulletMotionState::setWorldTransform(const btTransform &/*_cogWorldTrans*/)
{
  // _cogWorldTrans seems to be one time-step behind
  // for now get transform from btRigidBody directly
  physics::BulletLinkPtr bulletLink =
    boost::static_pointer_cast<BulletLink>(this->link);
  GZ_ASSERT(bulletLink, "parent link must be valid");
  ignition::math::Pose3d pose;
  if (bulletLink->GetBulletLink())
  {
    pose = BulletTypes::ConvertPoseIgn(
      bulletLink->GetBulletLink()->getCenterOfMassTransform());
  }
  else
  {
    pose = bulletLink->WorldInertialPose();
  }

  // transform pose from cg location to link location
  // cg: pose of cg in link frame, so -cg is transform from cg to
  //     link defined in cg frame.
  // pose: transform from world origin to cg in inertial frame.
  // -cg + pose:  transform from world origin to link frame in inertial frame.
  ignition::math::Pose3d cg = this->link->GetInertial()->Pose();
  pose = -cg + pose;

  // The second argument is set to false to prevent Entity.cc from propagating
  // the pose change all the way back to bullet.
  // \TODO: consider using the dirtyPose mechanism employed by ODE.
  this->link->SetWorldPose(pose, false);

  // below is inefficient as we end up double caching for some joints
  // should consider adding a "dirty" flag.
  // or trying doing this during BulletPhysics::InternalTickCallback(...)
  Joint_V parentJoints = this->link->GetParentJoints();
  for (unsigned int j = 0; j < parentJoints.size(); ++j)
  {
    JointPtr joint = parentJoints[j];
    joint->CacheForceTorque();
  }
  Joint_V childJoints = this->link->GetChildJoints();
  for (unsigned int j = 0; j < childJoints.size(); ++j)
  {
    JointPtr joint = childJoints[j];
    joint->CacheForceTorque();
  }
}
