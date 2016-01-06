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

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosCollision::SiconosCollision(LinkPtr _parent)
    : Collision(_parent)
{
  this->SetName("Siconos_Collision");
  this->collisionInteraction = NULL;
  this->surface.reset(new SiconosSurfaceParams());
}

//////////////////////////////////////////////////
SiconosCollision::~SiconosCollision()
{
  /*
  delete this->collisionShape;
  this->collisionShape = NULL;
  */
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
}

//////////////////////////////////////////////////
void SiconosCollision::OnPoseChange()
{
  math::Pose pose = this->GetRelativePose();
  SiconosLinkPtr bbody = boost::dynamic_pointer_cast<SiconosLink>(this->parent);

  // bbody->motionState.setWorldTransform(this, pose);
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
math::Box SiconosCollision::GetBoundingBox() const
{
  math::Box result;
  if (this->collisionInteraction)
  {
      /*
    btVector3 btMin, btMax;
    this->collisionShape->getAabb(btTransform::getIdentity(), btMin, btMax);

    result.min.Set(btMin.x(), btMin.y(), btMin.z());
    result.max.Set(btMax.x(), btMax.y(), btMax.z());

    if (this->GetShapeType() & PLANE_SHAPE)
    {
      PlaneShapePtr plane =
        boost::dynamic_pointer_cast<PlaneShape>(this->shape);
      math::Vector3 normal = plane->GetNormal();
      if (normal == math::Vector3::UnitZ)
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
void SiconosCollision::SetCollisionInteraction(Interaction *_interaction,
                                               bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->collisionInteraction = _interaction;

  // btmath::Vector3 vec;
  // this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);

  // this->mass.SetCoG(this->GetRelativePose().pos);

  // this->collisionShape->setFriction(1.0);
  // this->collisionShape->setAnisotropicFriction(btVector3(0, 0, 0));
}

//////////////////////////////////////////////////
Interaction *SiconosCollision::GetCollisionInteraction() const
{
  return this->collisionInteraction;
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
