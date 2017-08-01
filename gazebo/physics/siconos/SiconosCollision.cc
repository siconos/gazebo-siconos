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

#include "gazebo/physics/PlaneShape.hh"

#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosSurfaceParams.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "SiconosTypes.hh"

#include <boost/make_shared.hpp>
#include <siconos/SiconosContactor.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosCollision::SiconosCollision(LinkPtr _parent)
    : Collision(_parent)
{
  this->SetName("Siconos_Collision");
  this->surface.reset(new SiconosSurfaceParams());
  this->pose_offset = std11::make_shared<SiconosVector>(7);
  this->pose_offset->zero();
  (*this->pose_offset)(3) = 1;

  this->siconosContactor =
    std11::make_shared<SiconosContactor>(SP::SiconosShape(), this->pose_offset);
}

//////////////////////////////////////////////////
SiconosCollision::~SiconosCollision()
{
}

//////////////////////////////////////////////////
void SiconosCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }

  UpdateCollisionGroup();
}

//////////////////////////////////////////////////
void SiconosCollision::OnPoseChange()
{
  ignition::math::Pose3d pose = this->RelativePose();
  ignition::math::Vector3d cog = this->GetLink()->GetInertial()->CoG();
  pose.Pos() -= cog;
  if (this->base_offset)
  {
    ignition::math::Pose3d offset = SiconosTypes::ConvertPose(this->base_offset);
    pose = (offset + pose);
  }
  SiconosTypes::ConvertPoseToVector7(pose, this->pose_offset);
}

//////////////////////////////////////////////////
void SiconosCollision::SetCategoryBits(unsigned int _bits)
{
  this->categoryBits = _bits;
}

//////////////////////////////////////////////////
void SiconosCollision::SetCollideBits(unsigned int _bits)
{
  this->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int SiconosCollision::GetCategoryBits() const
{
  return this->categoryBits;
}

//////////////////////////////////////////////////
unsigned int SiconosCollision::GetCollideBits() const
{
  return this->collideBits;
}

//////////////////////////////////////////////////
/*Mass SiconosCollision::GetLinkMassMatrix()
{
  Mass result;
  return result;
}*/

//////////////////////////////////////////////////
ignition::math::Box SiconosCollision::BoundingBox() const
{
  ignition::math::Box result;
  if (this->siconosContactor)
  {
      /*
    btVector3 btMin, btMax;
    this->collisionShape->getAabb(btTransform::getIdentity(), btMin, btMax);

    result = ignition::math::Box(
        ignition::math::Vector3d(btMin.x(), btMin.y(), btMin.z()),
        ignition::math::Vector3d(btMax.x(), btMax.y(), btMax.z()));

    if (this->GetShapeType() & PLANE_SHAPE)
    {
      PlaneShapePtr plane =
        boost::dynamic_pointer_cast<PlaneShape>(this->shape);
      ignition::math::Vector3d normal = plane->GetNormal();
      if (normal == ignition::math::Vector3d::UnitZ)
      {
        // Should check altitude, but it's not implemented
        result.max.z =  0.0;
      }
    }
      */
  }
  return result;
}

//////////////////////////////////////////////////
void SiconosCollision::SetCollisionShape(SP::SiconosShape _shape,
                                         bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->siconosContactor->shape = _shape;

  // btmath::Vector3 vec;
  // this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);

  // this->mass.SetCoG(this->GetRelativePose().pos);

  // this->collisionShape->setFriction(1.0);
  // this->collisionShape->setAnisotropicFriction(btVector3(0, 0, 0));
}

//////////////////////////////////////////////////
SP::SiconosContactor SiconosCollision::GetSiconosContactor() const
{
  return this->siconosContactor;
}

//////////////////////////////////////////////////
void SiconosCollision::SetCompoundShapeIndex(int /*_index*/)
{
  // this->compoundShapeIndex = 0;
}

/////////////////////////////////////////////////
SiconosSurfaceParamsPtr SiconosCollision::GetSiconosSurface() const
{
  return boost::dynamic_pointer_cast<SiconosSurfaceParams>(this->surface);
}

/////////////////////////////////////////////////
void SiconosCollision::SetBaseTransform(SP::SiconosVector _offset)
{
  this->base_offset = _offset;
  this->OnPoseChange();
}

/////////////////////////////////////////////////
void SiconosCollision::SetBaseTransform(ignition::math::Pose3d _offset)
{
  this->base_offset = SiconosTypes::ConvertPose(_offset);
  this->OnPoseChange();
}

/////////////////////////////////////////////////
unsigned int SiconosCollision::UpdateCollisionGroup()
{
  SiconosPhysicsPtr physics = boost::static_pointer_cast<SiconosPhysics>(
    this->GetWorld()->Physics() );

  SiconosSurfaceParamsPtr surface = boost::static_pointer_cast<SiconosSurfaceParams>(
    this->surface );

  // This must be updated if SurfaceParams change, done in SiconosLink::UpdateSurface.
  this->siconosContactor->collision_group = physics->GetCollisionGroup(surface);
}

