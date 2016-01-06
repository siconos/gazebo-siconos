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

#ifndef _SICONOSPLANESHAPE_HH_
#define _SICONOSPLANESHAPE_HH_

#include <iostream>

#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/util/system.hh"

#include <SiconosShape.hpp>

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Siconos collision for an infinite plane.
    class GZ_PHYSICS_VISIBLE SiconosPlaneShape : public PlaneShape
    {
      /// \brief Constructor
      public: SiconosPlaneShape(CollisionPtr _parent) : PlaneShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SiconosPlaneShape() {}

      /// \brief Set the altitude of the plane
      public: void SetAltitude(const ignition::math::Vector3d &pos)
              {
                PlaneShape::SetAltitude(pos);
              }

      /// \brief Create the plane
      public: void CreatePlane()
              {
                PlaneShape::CreatePlane();
                SiconosCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SiconosCollision>(
                    this->collisionParent);

                ignition::math::Vector3d n = this->Normal(); // TODO Siconos
                SP::SiconosPlane siconosPlane(new SiconosPlane());

                bParent->SetCollisionShape(siconosPlane, false);
              }
    };
    /// \}
  }
}
#endif
