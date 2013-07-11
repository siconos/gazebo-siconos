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
/* Desc: The base Simbody joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 15 May 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyJoint::SimbodyJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->isReversed = false;
  this->mustBreakLoopHere = false;
}

//////////////////////////////////////////////////
SimbodyJoint::~SimbodyJoint()
{
}

//////////////////////////////////////////////////
void SimbodyJoint::Load(sdf::ElementPtr _sdf)
{
  // store a pointer to the simbody physics engine for convenience
  this->simbodyPhysics = boost::shared_dynamic_cast<SimbodyPhysics>(
    this->model->GetWorld()->GetPhysicsEngine());

  Joint::Load(_sdf);

  // read must_be_loop_joint
  // \TODO: clean up
  if (_sdf->HasElement("physics") &&
    _sdf->GetElement("physics")->HasElement("simbody"))
  {
    this->mustBreakLoopHere = _sdf->GetElement("physics")->
      GetElement("simbody")->GetValueBool("must_be_loop_joint");
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      /// \TODO: switch to GetElement so default values apply
      /// \TODO: check all physics engines
      if (dynamicsElem->HasElement("damping"))
      {
        this->dampingCoefficient = dynamicsElem->GetValueDouble("damping");
      }
    }
  }

  // Read old style
  //    <pose>pose on child</pose>
  // or new style

  // to support alternative unassembled joint pose specification
  // check if the new style of pose specification exists
  //    <parent>
  //      <link>parentName</link>
  //      <pose>parentPose</pose>
  //    </parent>
  // as compared to old style
  //    <parent>parentName</parent>
  //
  // \TODO: consider storing the unassembled format parent pose when
  // calling Joint::Load(sdf::ElementPtr)

  math::Pose childPose = _sdf->GetValuePose("pose");
  if (_sdf->GetElement("child")->HasElement("pose"))
    childPose = _sdf->GetElement("child")->GetValuePose("pose");

  this->X_CB = physics::SimbodyPhysics::Pose2Transform(childPose);

  math::Pose parentPose;
  if (_sdf->GetElement("parent")->HasElement("pose"))
    this->X_PA = physics::SimbodyPhysics::GetPose(_sdf->GetElement("parent"));
  else
  {
    SimTK::Transform X_MC, X_MP;
    if (this->parentLink)
    {
      X_MP = physics::SimbodyPhysics::Pose2Transform(
        this->parentLink->GetRelativePose());
    }
    else
    {
      // TODO: verify
      // parent frame is at the world frame
      X_MP = ~physics::SimbodyPhysics::Pose2Transform(
        this->model->GetWorldPose());
    }

    if (this->childLink)
    {
      X_MC = physics::SimbodyPhysics::Pose2Transform(
        this->childLink->GetRelativePose());
    }
    else
    {
      // TODO: verify
      X_MC = ~physics::SimbodyPhysics::Pose2Transform(
        this->model->GetWorldPose());
    }

    const SimTK::Transform X_PC = ~X_MP*X_MC;
    this->X_PA = X_PC*this->X_CB; // i.e., A spatially coincident with B 
  }
}

//////////////////////////////////////////////////
void SimbodyJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
LinkPtr SimbodyJoint::GetJointLink(int _index) const
{
  LinkPtr result;

  if (_index == 0 || _index == 1)
  {
    SimbodyLinkPtr simbodyLink1 =
      boost::shared_static_cast<SimbodyLink>(this->childLink);

    SimbodyLinkPtr simbodyLink2 =
      boost::shared_static_cast<SimbodyLink>(this->parentLink);
  }

  return result;
}

//////////////////////////////////////////////////
bool SimbodyJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  return ((this->childLink.get() == _one.get() &&
           this->parentLink.get() == _two.get()) ||
          (this->childLink.get() == _two.get() &&
           this->parentLink.get() == _one.get()));
}

//////////////////////////////////////////////////
void SimbodyJoint::Detach()
{
  this->childLink.reset();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::GetForceTorque(int _index)
{
  return this->GetForceTorque(static_cast<unsigned int>(_index));
}

//////////////////////////////////////////////////
JointWrench SimbodyJoint::GetForceTorque(unsigned int /*_index*/)
{
  JointWrench wrench;
  const SimTK::State &state = this->simbodyPhysics->integ->getState();

  // force calculation of reaction forces
  this->simbodyPhysics->system.realize(state);

  SimTK::SpatialVec reactionForceOnParentBodyInGround =
    this->mobod.findMobilizerReactionOnParentAtFInGround(state);

  SimTK::SpatialVec reactionForceOnChildBodyInGround =
    this->mobod.findMobilizerReactionOnBodyAtMInGround(state);

  // re-express in link frame
  SimTK::Vec3 reactionTorqueOnChildBody =
    this->mobod.expressGroundVectorInBodyFrame(
    state, reactionForceOnParentBodyInGround[0]);   // [0] torque element
  SimTK::Vec3 reactionForceOnChildBody =
    this->mobod.expressGroundVectorInBodyFrame(
    state, reactionForceOnParentBodyInGround[1]);   // [1] force element

  SimTK::Vec3 reactionTorqueOnParentBody =
    this->mobod.getParentMobilizedBody().expressGroundVectorInBodyFrame(
    state, reactionForceOnParentBodyInGround[0]);   // [0] torque element
  SimTK::Vec3 reactionForceOnParentBody =
    this->mobod.getParentMobilizedBody().expressGroundVectorInBodyFrame(
    state, reactionForceOnParentBodyInGround[1]);   // [1] force element

  if (!this->isReversed)
  {
    wrench.body1Force =
      SimbodyPhysics::Vec3ToVector3(reactionForceOnChildBody);
    wrench.body1Torque =
      SimbodyPhysics::Vec3ToVector3(reactionTorqueOnChildBody);

    wrench.body2Force =
      SimbodyPhysics::Vec3ToVector3(reactionForceOnParentBody);
    wrench.body2Torque =
      SimbodyPhysics::Vec3ToVector3(reactionTorqueOnParentBody);
  }
  else
  {
    wrench.body1Force =
      SimbodyPhysics::Vec3ToVector3(reactionForceOnParentBody);
    wrench.body1Torque =
      SimbodyPhysics::Vec3ToVector3(reactionTorqueOnParentBody);

    wrench.body2Force =
      SimbodyPhysics::Vec3ToVector3(reactionForceOnChildBody);
    wrench.body2Torque =
      SimbodyPhysics::Vec3ToVector3(reactionTorqueOnChildBody);
  }
  return wrench;
}
