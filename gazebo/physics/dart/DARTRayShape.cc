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

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTRayShape.hh"

#include "gazebo/physics/dart/DARTRayShapePrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine),
    dataPtr(new DARTRayShapePrivate())
{
  this->SetName("DART_ray_shape");
}

//////////////////////////////////////////////////
DARTRayShape::DARTRayShape(CollisionPtr _parent)
  : RayShape(_parent),
    dataPtr(new DARTRayShapePrivate())
{
  this->SetName("DART_ray_shape");
}

//////////////////////////////////////////////////
DARTRayShape::~DARTRayShape()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTRayShape::Update()
{
  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void DARTRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  // Not implemented yet, please see issue #911
}

//////////////////////////////////////////////////
void DARTRayShape::SetPoints(const ignition::math::Vector3d &_posStart,
                             const ignition::math::Vector3d &_posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);

  // Not implemented yet, please see issue #911
}
