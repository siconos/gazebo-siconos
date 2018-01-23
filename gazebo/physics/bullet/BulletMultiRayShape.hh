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
#ifndef GAZEBO_PHYSICS_BULLET_BULLETMULTIRAYSHAPE_HH_
#define GAZEBO_PHYSICS_BULLET_BULLETMULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Bullet specific version of MultiRayShape
    class GZ_PHYSICS_VISIBLE BulletMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor
      public: explicit BulletMultiRayShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~BulletMultiRayShape();

      /// \brief Update the rays
      public: virtual void UpdateRays();

      /// \brief Add a ray to the collision
      protected: void AddRay(const ignition::math::Vector3d &start,
                             const ignition::math::Vector3d &end);

      private: BulletPhysicsPtr physicsEngine;
    };
    /// \}
  }
}
#endif
