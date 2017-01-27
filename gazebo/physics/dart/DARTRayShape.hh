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

#ifndef _GAZEBO_DARTRAYSHAPE_HH_
#define _GAZEBO_DARTRAYSHAPE_HH_

#include <string>

#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_dart
    /// \{

    /// Forward declare private data class
    class DARTRayShapePrivate;

    /// \brief Ray collision
    class GZ_PHYSICS_VISIBLE DARTRayShape : public RayShape
    {
      /// \brief Constructor for a global ray.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit DARTRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor.
      /// \param[in] _collision Collision object this ray is attached to.
      public: explicit DARTRayShape(CollisionPtr _collision);

      /// \brief Destructor.
      public: virtual ~DARTRayShape();

      /// \brief Update the ray collision
      public: virtual void Update();

      /// \brief Get the nearest intersection
      /// \param[out] _dist Distance to the intersection.
      /// \param[out] _entity Name of the entity that was hit.
      public: virtual void GetIntersection(double &_dist, std::string &_entity);

      /// \brief Set the ray based on starting and ending points relative to
      ///        the body
      /// \param[in] _posStart Start position, relative the body
      /// \param[in] _posEnd End position, relative to the body
      public: virtual void SetPoints(const ignition::math::Vector3d &_posStart,
                                     const ignition::math::Vector3d &_posEnd);

      /// \internal
      /// \brief Pointer to private data
      private: DARTRayShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
