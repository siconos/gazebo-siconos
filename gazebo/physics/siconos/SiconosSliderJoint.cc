/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosSliderJoint.hh"

// Use Bullet math to calculate reference frames
#include "gazebo/physics/bullet/BulletTypes.hh"

#include "SiconosWorld.hh"
#include <siconos/PrismaticJointR.hpp>
#include <siconos/NonSmoothDynamicalSystem.hpp>

#include <siconos/Model.hpp>
#include <siconos/BodyDS.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosSliderJoint::SiconosSliderJoint(BasePtr _parent, SP::SiconosWorld _world)
    : SliderJoint<SiconosJoint>(_parent)
{
  GZ_ASSERT(_world, "siconos world pointer is NULL");
  this->siconosWorld = _world;
  this->siconosPrismaticJointR = NULL;
}

//////////////////////////////////////////////////
SiconosSliderJoint::~SiconosSliderJoint()
{
}

//////////////////////////////////////////////////
void SiconosSliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<SiconosJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosSliderJoint::Init()
{
  SliderJoint<SiconosJoint>::Init();

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
    this->siconosPrismaticJointR = std11::make_shared<PrismaticJointR>(
        ds1 = siconosParentLink->GetSiconosBodyDS(),
        ds2 = siconosChildLink->GetSiconosBodyDS(),
        SiconosTypes::ConvertVector3(axisParent));
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (siconosChildLink)
  {
    this->siconosPrismaticJointR = std11::make_shared<PrismaticJointR>(
        ds1 = siconosChildLink->GetSiconosBodyDS(),
        SiconosTypes::ConvertVector3(axisChild));
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (siconosParentLink)
  {
    this->siconosPrismaticJointR = std11::make_shared<PrismaticJointR>(
        ds1 = siconosParentLink->GetSiconosBodyDS(),
        SiconosTypes::ConvertVector3(axisParent));
  }
  // Throw an error if no links are given.
  else
  {
    gzerr << "joint without links\n";
    return;
  }

  if (!this->siconosPrismaticJointR)
  {
    gzerr << "unable to create siconos slider joint\n";
    return;
  }

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
  GZ_ASSERT(axisElem != NULL, "Joint axis sdf member is NULL");
  {
    sdf::ElementPtr limitElem;
    limitElem = this->sdf->GetElement("axis")->GetElement("limit");
    // TODO
    // this->siconosPrismaticJointR->setLowerLinLimit(limitElem->Get<double>("lower"));
    // this->siconosPrismaticJointR->setUpperLinLimit(limitElem->Get<double>("upper"));
  }

  // Set Joint friction here in Init, since the siconos data structure didn't
  // exist when the friction was set during Joint::Load
  this->SetParam("friction", 0,
    axisElem->GetElement("dynamics")->Get<double>("friction"));

  // Give parent class SiconosJoint a pointer to this constraint.
  this->relation = this->siconosPrismaticJointR;

  // Create a Siconos Interacton with an EqualityConditionNSL
  int nc = this->siconosPrismaticJointR->numberOfConstraints();
  this->interaction = std11::make_shared<Interaction>(
    nc, std11::make_shared<EqualityConditionNSL>(nc), this->siconosPrismaticJointR);

  // Add the joint to the NSDS
  GZ_ASSERT(this->siconosWorld, "SiconosWorld pointer is NULL");
  this->siconosWorld->GetModel()->nonSmoothDynamicalSystem()
    ->link(this->interaction, ds1, ds2);

  // Initialize Interaction states for the Simulation
  this->siconosWorld->GetSimulation()->initializeInteraction(
    this->siconosWorld->GetSimulation()->nextTime(),
    this->interaction);

  // Allows access to impulse TODO
  //this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
double SiconosSliderJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0;
  ignition::math::Vector3d globalAxis = this->GlobalAxis(0);
  if (this->childLink)
    result += globalAxis.Dot(this->childLink->WorldLinearVel());
  if (this->parentLink)
    result -= globalAxis.Dot(this->parentLink->WorldLinearVel());
  return result;
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetVelocity(unsigned int _index, double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosSliderJoint::Anchor(unsigned int /*_index*/) const
{
  // TODO
  // btTransform trans = this->siconosHinge->getAFrame();
  // trans.getOrigin() +=
  //   this->siconosHinge->getRigidBodyA().getCenterOfMassTransform().getOrigin();
  // return ignition::math::Vector3d(trans.getOrigin().getX(),
  //     trans.getOrigin().getY(), trans.getOrigin().getZ());
  return ignition::math::Vector3d(0,0,0);
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetAxis(unsigned int /*_index*/,
                                 const ignition::math::Vector3d &_axis)
{
  // Note that _axis is given in a world frame,
  // but siconos uses a body-fixed frame
  if (!this->siconosPrismaticJointR)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetDamping(unsigned int /*index*/,
                                    const double /*_damping*/)
{
  /// \TODO: special case siconos specific linear damping, this needs testing.
  // if (this->siconosPrismaticJointR)
  //   this->siconosPrismaticJointR->setDampingDirLin(_damping);
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetForceImpl(unsigned int _index, double _effort)
{
  if (this->siconosPrismaticJointR && _index==0)
  {
    SiconosLinkPtr link0(boost::static_pointer_cast<SiconosLink>(this->parentLink));
    SiconosLinkPtr link1(boost::static_pointer_cast<SiconosLink>(this->childLink));

    ignition::math::Vector3d axis(
      SiconosTypes::ConvertVector3(this->siconosPrismaticJointR->_axis0) );

    if (link0 && link1) {
      link0->AddRelativeForce(axis * -_effort);
      link1->AddRelativeForce(axis * _effort);
    }
    else if (link0) {
      link0->AddRelativeForce(axis * _effort);
    }
    else if (link1) {
      link1->AddRelativeForce(axis * _effort);
    }
  }
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetUpperLimit(unsigned int /*_index*/,
                                       const double _limit)
{
  Joint::SetUpperLimit(0, _limit);
  if (this->siconosPrismaticJointR)
  {
    //TODO
    //this->siconosPrismaticJointR->setUpperLinLimit(_angle.Radian());
  }
  else
  {
    gzlog << "siconosPrismaticJointR not yet created.\n";
  }
}

//////////////////////////////////////////////////
void SiconosSliderJoint::SetLowerLimit(unsigned int /*_index*/,
                                       const double _limit)
{
  Joint::SetLowerLimit(0, _limit);
  if (this->siconosPrismaticJointR)
  {
    //TODO
    //this->siconosPrismaticJointR->setLowerLinLimit(_angle.Radian());
  }
  else
  {
    gzlog << "siconosPrismaticJointR not yet created.\n";
  }
}

//////////////////////////////////////////////////
double SiconosSliderJoint::UpperLimit(unsigned int /*_index*/) const
{
  double result;
  if (this->siconosPrismaticJointR)
    result = 0.0; // TODO this->siconosPrismaticJointR->getUpperLinLimit();
  else
    gzlog << "Joint must be created before getting high stop\n";
  return result;
}

//////////////////////////////////////////////////
double SiconosSliderJoint::LowerLimit(unsigned int /*_index*/) const
{
  double result;
  if (this->siconosPrismaticJointR)
    result = 0.0; // TODO this->siconosPrismaticJointR->getLowerLinLimit();
  else
    gzlog << "Joint must be created before getting low stop\n";
  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SiconosSliderJoint::GlobalAxis(unsigned int /*_index*/) const
{
  ignition::math::Vector3d result = this->initialWorldAxis;

  if (this->siconosPrismaticJointR)
  {
    // bullet uses x-axis for slider
    // TODO
    // btVector3 vec =
    //   this->siconosPrismaticJointR->getRigidBodyA().getCenterOfMassTransform().getBasis()
    //   * this->siconosPrismaticJointR->getFrameOffsetA().getBasis().getColumn(0);
    // result = SiconosTypes::ConvertVector3(vec);
  }

  return result;
}

//////////////////////////////////////////////////
double SiconosSliderJoint::PositionImpl(unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid axis index [" << _index << "]" << std::endl;
    return 0.0;
  }

  // The getLinearPos function seems to be off by one time-step
  // https://github.com/siconosphysics/siconos3/issues/239
  // if (this->siconosPrismaticJointR)
  //   result = this->siconosPrismaticJointR->getLinearPos();
  // else
  //   gzlog << "siconosPrismaticJointR does not exist, returning default position\n";

  // Compute slider angle from gazebo's cached poses instead
  ignition::math::Vector3d offset = this->WorldPose().Pos()
                                    - this->ParentWorldPose().Pos();
  ignition::math::Vector3d axis = this->GlobalAxis(_index);
  ignition::math::Pose3d poseParent = this->WorldPose();
  return axis.Dot(offset);
}

//////////////////////////////////////////////////
bool SiconosSliderJoint::SetParam(const std::string &_key,
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
      if (this->siconosPrismaticJointR)
      {
        // TODO
        // this->siconosPrismaticJointR->setPoweredLinMotor(true);
        // this->siconosPrismaticJointR->setTargetLinMotorVelocity(0.0);
        // double value = boost::any_cast<double>(_value);
        // this->siconosPrismaticJointR->setMaxLinMotorForce(value);
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
double SiconosSliderJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_index >= this->DOF())
  {
    gzerr << "Invalid index [" << _index << "]" << std::endl;
    return 0;
  }

  if (_key == "friction")
  {
    if (this->siconosPrismaticJointR)
    {
      double value = 0; // TODO this->siconosPrismaticJointR->getMaxLinMotorForce();
      return value;
    }
    else
    {
      gzerr << "Joint must be created before getting " << _key << std::endl;
      return 0.0;
    }
  }
  return SiconosJoint::GetParam(_key, _index);
}
