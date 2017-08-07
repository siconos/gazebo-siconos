/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/physics/siconos/SiconosHingeJoint.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"

#include <siconos/Model.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>
#include <siconos/BodyDS.hpp>
#include <siconos/PivotJointR.hpp>
#include <siconos/Interaction.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosHingeJoint::SiconosHingeJoint(BasePtr _parent, SP::SiconosWorld _world)
  : HingeJoint<SiconosJoint>(_parent)
{
  siconosWorld = _world;
  GZ_ASSERT(siconosWorld, "SiconosWorld pointer is NULL");
}

//////////////////////////////////////////////////
SiconosHingeJoint::~SiconosHingeJoint()
{
}

//////////////////////////////////////////////////
void SiconosHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosHingeJoint::Init()
{
  HingeJoint<SiconosJoint>::Init();

  // Cast to SiconosLink
  SiconosLinkPtr siconosChildLink =
    boost::static_pointer_cast<SiconosLink>(this->childLink);
  SiconosLinkPtr siconosParentLink =
    boost::static_pointer_cast<SiconosLink>(this->parentLink);

  // Get axis unit vector (expressed in world frame).
  ignition::math::Vector3d axis = this->initialWorldAxis;
  if (axis == ignition::math::Vector3d::Zero)
  {
    gzerr << "axis must have non-zero length, resetting to 0 0 1\n";
    axis.Set(0, 0, 1);
  }

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

  SP::BodyDS ds1, ds2;

  // If both links exist, then create a joint between the two links.
  if (siconosChildLink && siconosParentLink)
  {
    this->siconosPivotJointR = std11::make_shared<PivotJointR>(
      SiconosTypes::ConvertVector3(pivotParent),
      SiconosTypes::ConvertVector3(axisParent),
      false /* not absoluteRef */,
      ds1 = siconosParentLink->GetSiconosBodyDS(),
      ds2 = siconosChildLink->GetSiconosBodyDS());
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (siconosChildLink)
  {
    this->siconosPivotJointR = std11::make_shared<PivotJointR>(
      SiconosTypes::ConvertVector3(pivotChild),
      SiconosTypes::ConvertVector3(axisChild),
      false /* not absoluteRef */,
      ds1 = siconosChildLink->GetSiconosBodyDS());
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (siconosParentLink)
  {
    this->siconosPivotJointR = std11::make_shared<PivotJointR>(
      SiconosTypes::ConvertVector3(pivotParent),
      SiconosTypes::ConvertVector3(axisParent),
      false /* not absoluteRef */,
      ds1 = siconosParentLink->GetSiconosBodyDS());
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "unable to create siconos hinge without links.\n";
    return;
  }

  if (!this->siconosPivotJointR)
  {
    gzerr << "unable to create PivotJointR\n";
    return;
  }

  // Create a Siconos Interacton with an EqualityConditionNSL
  int nc = this->siconosPivotJointR->numberOfConstraints();
  this->interaction = std11::make_shared<::Interaction>(
    std11::make_shared<EqualityConditionNSL>(nc), this->siconosPivotJointR);

  // Apply joint angle limits here.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != NULL, "Joint axis sdf member is NULL");
  this->SetupJointLimits();

  // TODO: Velocity and effort limits

  // Set Joint friction here in Init, since the siconos data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  // Add the joint to the NSDS
  GZ_ASSERT(this->siconosWorld, "SiconosWorld pointer is NULL");
  this->siconosWorld->GetModel()->nonSmoothDynamicalSystem()
    ->link(this->interaction, ds1, ds2);

  // Initialize Interaction states for the Simulation
  this->siconosWorld->GetSimulation()->initializeInteraction(
    this->siconosWorld->GetSimulation()->nextTime(),
    this->interaction);

  // Allows access to impulse
  // this->siconosHinge->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosHingeJoint::Anchor(unsigned int /*_index*/) const
{
  // btTransform trans = this->siconosHinge->getAFrame();
  // trans.getOrigin() +=
  //   this->siconosHinge->getRigidBodyA().getCenterOfMassTransform().getOrigin();
  // return ignition::math::Vector3d(trans.getOrigin().getX(),
  //     trans.getOrigin().getY(), trans.getOrigin().getZ());
  return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
void SiconosHingeJoint::SetAxis(unsigned int /*_index*/,
    const ignition::math::Vector3d &_axis)
{
  // Note that _axis is given in a world frame,
  // but siconos uses a body-fixed frame
  if (!this->siconosPivotJointR)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }

  // Siconos seems to handle setAxis improperly. It readjust all the pivot
  // points
  /*btignition::math::Vector3d vec(_axis.x, _axis.y, _axis.z);
  ((btHingeConstraint*)this->siconosHinge)->setAxis(vec);
  */
}

//////////////////////////////////////////////////
double SiconosHingeJoint::PositionImpl(const unsigned int _index) const
{
  double result = 0.0;
  SiconosLinkPtr link0, link1;

  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
  {
    if (this->parentLink) {
      link0 = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    }
    if (this->parentLink && this->childLink) {
      link1 = boost::static_pointer_cast<SiconosLink>(this->childLink);
    }
    else if (this->childLink) {
      link0 = boost::static_pointer_cast<SiconosLink>(this->childLink);
    }
    else
      return 0.0;

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector y(1);
    this->siconosPivotJointR->computehDoF(0.0, bv, y, _index);
    result = y(0);
  }

  return result;
}

//////////////////////////////////////////////////
void SiconosHingeJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
double SiconosHingeJoint::GetVelocity(unsigned int _index) const
{
  double result = 0.0;

  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
  {
    ignition::math::Vector3d globalAxis = this->GlobalAxis(_index);
    if (this->parentLink)
      result = globalAxis.Dot(this->parentLink->WorldAngularVel());
    if (this->parentLink && this->childLink)
      result -= globalAxis.Dot(this->childLink->WorldAngularVel());
    else if (this->childLink)
      result = globalAxis.Dot(this->childLink->WorldAngularVel());
  }
  return result;
}

//////////////////////////////////////////////////
void SiconosHingeJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosPivotJointR
      && _index < this->siconosPivotJointR->numberOfDoF())
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    BlockVector bv((link0 ? 1 : 0) + (link1 ? 1 : 0), 7);
    unsigned int i = 0;
    if (link0) bv.setVectorPtr(i++, link0->GetSiconosBodyDS()->q());
    if (link1) bv.setVectorPtr(i++, link1->GetSiconosBodyDS()->q());

    SiconosVector v(3);
    this->siconosPivotJointR->normalDoF(v, bv, _index, true);
    ignition::math::Vector3d axis(SiconosTypes::ConvertVector3(v));

    if (this->parentLink && this->childLink) {
      this->parentLink->AddTorque(-_effort * axis);
      this->childLink->AddTorque(_effort * axis);
    }

    else if (this->parentLink) {
      this->parentLink->AddTorque(_effort * axis);
    }

    else if (this->childLink) {
      this->childLink->AddTorque(_effort * axis);
    }
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosHingeJoint::GlobalAxis(unsigned int _index) const
{
  ignition::math::Vector3d result = this->initialWorldAxis;

  if (this->siconosPivotJointR)
  {
    GZ_ASSERT(_index < this->siconosPivotJointR->numberOfDoF(),
              "SiconosHingeJoint::GlobalAxis(): axis index too large.");

    SiconosLinkPtr link;
    if (this->parentLink) {
      link = boost::static_pointer_cast<SiconosLink>(this->parentLink);
    }
    else if (this->childLink) {
      link = boost::static_pointer_cast<SiconosLink>(this->childLink);
    }
    if (link) {
      BlockVector bv(1, 7);
      bv.setVectorPtr(0, link->GetSiconosBodyDS()->q());

      SiconosVector v(3);
      this->siconosPivotJointR->normalDoF(v, bv, _index, false);
      result = SiconosTypes::ConvertVector3(v);
    }
  }

  if (this->siconosPivotJointR)
  {

    // I have not verified the following math, though I based it on internal
    // siconos code at line 250 of btHingeConstraint.cpp
    // btVector3 vec =
    //   siconosHinge->getRigidBodyA().getCenterOfMassTransform().getBasis() *
    //   siconosHinge->getFrameOffsetA().getBasis().getColumn(2);
    // result = SiconosTypes::ConvertVector3(vec);
  }

  return result;
}

//////////////////////////////////////////////////
bool SiconosHingeJoint::SetParam(const std::string &_key,
    unsigned int _index,
    const boost::any &_value)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return false;
  }

  try
  {
    if (_key == "friction")
    {
      if (this->siconosPivotJointR)
      {
        // enableAngularMotor takes max impulse as a parameter
        // instead of max force.
        // this means the friction will change when the step size changes.
        // double dt = this->world->Physics()->GetMaxStepSize();
        // this->siconosHinge->enableAngularMotor(true, 0.0,
        //   dt * boost::any_cast<double>(_value));
      }
      else
      {
        gzerr << "Joint must be created before setting " << _key << std::endl;
        return false;
      }
    }
    else
    {
      return SiconosJoint::SetParam(_key, _index, _value);
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ")"
          << " boost any_cast error:" << e.what()
          << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
double SiconosHingeJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->siconosPivotJointR)
    {
      // double dt = this->world->Physics()->GetMaxStepSize();
      //return this->siconosHinge->getMaxMotorImpulse() / dt;
      return 0.0;
    }
    else
    {
      gzerr << "Joint must be created before getting " << _key << std::endl;
      return 0.0;
    }
  }
  return SiconosJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
SP::NewtonEulerJointR SiconosHingeJoint::Relation() const
{
  return siconosPivotJointR;
}
