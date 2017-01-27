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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/physics/simbody/SimbodyCollision.hh"
#include "gazebo/physics/SurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyCollision::SimbodyCollision(LinkPtr _parent)
    : Collision(_parent)
{
  this->SetName("Simbody_Collision");
  this->collisionShape = nullptr;
  this->surface.reset(new SurfaceParams());
}

//////////////////////////////////////////////////
SimbodyCollision::~SimbodyCollision()
{
  this->collisionShape = nullptr;
}

//////////////////////////////////////////////////
void SimbodyCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);
}

//////////////////////////////////////////////////
void SimbodyCollision::OnPoseChange()
{
  // auto pose = this->RelativePose();
  // SimbodyLink *bbody = static_cast<SimbodyLink*>(this->body);

  // bbody->SetCollisionRelativePose(this, pose);
}

//////////////////////////////////////////////////
void SimbodyCollision::SetCategoryBits(unsigned int /*_bits*/)
{
}

//////////////////////////////////////////////////
void SimbodyCollision::SetCollideBits(unsigned int /*_bits*/)
{
}

//////////////////////////////////////////////////
ignition::math::Box SimbodyCollision::BoundingBox() const
{
  ignition::math::Box result(0, 0, 0, 0, 0, 0);

  gzerr << "Simbody does not provide bounding box info.\n";

  return result;
}

//////////////////////////////////////////////////
void SimbodyCollision::SetCollisionShape(SimTK::ContactGeometry *_shape)
{
  this->collisionShape = _shape;
}

//////////////////////////////////////////////////
SimTK::ContactGeometry *SimbodyCollision::GetCollisionShape() const
{
  return this->collisionShape;
}
