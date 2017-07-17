/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <sstream>

#include "gazebo/common/Console.hh"

#include "gazebo/physics/SurfaceParams.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/physics/dart/DARTSurfaceParams.hh"

#include "gazebo/physics/dart/DARTCollisionPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTCollision::DARTCollision(LinkPtr _link)
  : Collision(_link),
    dataPtr(new DARTCollisionPrivate())
{
  this->SetName("DART_Collision");
  this->surface.reset(new DARTSurfaceParams());
}

//////////////////////////////////////////////////
DARTCollision::~DARTCollision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
}

//////////////////////////////////////////////////
void DARTCollision::Init()
{
  Collision::Init();

  // Create the collision shapes. Since DART6, this has to be done in
  // Init(), because the BodyNode will only have been created in Load()
  // and is not guaranteed to exist before.

  // Set the pose offset.
  if (this->dataPtr->dtCollisionShape)
  {
    // TODO: Remove type check once DART completely supports plane shape.
    // Please see: https://github.com/dartsim/dart/issues/114
    const bool isPlaneShape =
        (boost::dynamic_pointer_cast<DARTPlaneShape>(this->shape) != nullptr);

    if (!isPlaneShape)
    {
      Eigen::Isometry3d tf = DARTTypes::ConvPose(this->RelativePose());
      this->dataPtr->dtCollisionShape->setRelativeTransform(tf);
    }
  }
}

//////////////////////////////////////////////////
void DARTCollision::Fini()
{
  Collision::Fini();
}

//////////////////////////////////////////////////
void DARTCollision::OnPoseChange()
{
  // Nothing to do in dart.
}

//////////////////////////////////////////////////
void DARTCollision::SetCategoryBits(unsigned int _bits)
{
  this->dataPtr->categoryBits = _bits;
}

//////////////////////////////////////////////////
void DARTCollision::SetCollideBits(unsigned int _bits)
{
  this->dataPtr->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCategoryBits() const
{
  return this->dataPtr->categoryBits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCollideBits() const
{
  return this->dataPtr->collideBits;
}

//////////////////////////////////////////////////
ignition::math::Box DARTCollision::BoundingBox() const
{
  GZ_ASSERT(this->dataPtr->dtCollisionShape,
            "DARTCollision Must have a collision shape");
  GZ_ASSERT(this->dataPtr->dtCollisionShape->getShape(),
            "DARTCollision has no shape");

  const dart::math::BoundingBox& bb =
    this->dataPtr->dtCollisionShape->getShape()->getBoundingBox();

  ignition::math::Box result(bb.getMin().x(),
                             bb.getMin().y(),
                             bb.getMin().z(),
                             bb.getMax().x(),
                             bb.getMax().y(),
                             bb.getMax().z());
  return result;
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode *DARTCollision::DARTBodyNode() const
{
  return boost::static_pointer_cast<DARTLink>(this->link)->DARTBodyNode();
}

//////////////////////////////////////////////////
void DARTCollision::SetDARTCollisionShapeNode(
                                          dart::dynamics::ShapeNodePtr _shape,
                                          bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->dataPtr->dtCollisionShape = _shape;
}

//////////////////////////////////////////////////
dart::dynamics::ShapeNodePtr DARTCollision::DARTCollisionShapeNode() const
{
  return this->dataPtr->dtCollisionShape;
}

/////////////////////////////////////////////////
DARTSurfaceParamsPtr DARTCollision::DARTSurface() const
{
  return boost::dynamic_pointer_cast<DARTSurfaceParams>(this->surface);
}
