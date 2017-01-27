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

#include "gazebo/physics/simbody/SimbodyPlaneShape.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
SimbodyPlaneShape::SimbodyPlaneShape(CollisionPtr _parent)
  : PlaneShape(_parent)
{
}

/////////////////////////////////////////////////
SimbodyPlaneShape::~SimbodyPlaneShape()
{
}

/////////////////////////////////////////////////
void SimbodyPlaneShape::SetAltitude(const ignition::math::Vector3d &_pos)
{
  PlaneShape::SetAltitude(_pos);
}

/////////////////////////////////////////////////
void SimbodyPlaneShape::CreatePlane()
{
  PlaneShape::CreatePlane();
  SimbodyCollisionPtr bParent;
  bParent = boost::dynamic_pointer_cast<SimbodyCollision>(
      this->collisionParent);

  ignition::math::Vector3d n = this->Normal();

  // set collision shape
}
