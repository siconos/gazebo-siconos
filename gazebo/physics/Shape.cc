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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Console.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Shape::Shape(CollisionPtr _p)
  : Base(_p)
{
  this->AddType(Base::SHAPE);
  this->SetName("shape");

  if (_p)
    this->collisionParent = _p;
}

//////////////////////////////////////////////////
Shape::~Shape()
{
  if (this->collisionParent)
    this->collisionParent->SetShape(ShapePtr());
}

//////////////////////////////////////////////////
math::Vector3 Shape::GetScale() const
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return this->Scale();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}

//////////////////////////////////////////////////
ignition::math::Vector3d Shape::Scale() const
{
  return this->scale;
}

//////////////////////////////////////////////////
double Shape::ComputeVolume() const
{
  if (!this->collisionParent)
  {
    gzerr << "Cannot discern shape type, returning 0 volume" << std::endl;
    return 0;
  }
  gzwarn << "ComputeVolume not fully implemented for this shape type, returning"
         << " bounding box approximation" << std::endl;

  ignition::math::Vector3d size =
    this->collisionParent->BoundingBox().Size();
  return size.X() * size.Y() * size.Z();
}

//////////////////////////////////////////////////
void Shape::SetScale(const math::Vector3 &_scale)
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return this->SetScale(_scale.Ign());
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}
