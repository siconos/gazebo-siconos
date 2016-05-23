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

#ifndef _SICONOSRAYGEOM_HH_
#define _SICONOSRAYGEOM_HH_

#include <string>
#include "gazebo/physics/RayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Ray shape for siconos
    class GZ_PHYSICS_VISIBLE SiconosRayShape : public RayShape
    {
      public: SiconosRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor
      /// \param body Link the ray is attached to
      public: SiconosRayShape(CollisionPtr _collision);

      /// \brief Destructor
      public: virtual ~SiconosRayShape();

      /// \brief Update the ray collision
      public: virtual void Update();

      /// \brief Get the nearest intersection
      public: virtual void GetIntersection(double &_dist, std::string &_entity);

      /// \brief Set the ray based on starting and ending points relative to
      ///        the body
      /// \param posStart Start position, relative the body
      /// \param posEnd End position, relative to the body
      public: void SetPoints(const math::Vector3 &_posStart,
                             const math::Vector3 &_posEnd);

      /// \brief Pointer to the Siconos physics engine
      private: SiconosPhysicsPtr physicsEngine;
    };
    /// \}
  }
}
#endif
