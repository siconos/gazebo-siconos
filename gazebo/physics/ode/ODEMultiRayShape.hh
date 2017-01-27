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
#ifndef GAZEBO_PHYSICS_ODE_ODEMULTIRAYSHAPE_HH_
#define GAZEBO_PHYSICS_ODE_ODEMULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief ODE specific version of MultiRayShape
    class GZ_PHYSICS_VISIBLE ODEMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit ODEMultiRayShape(CollisionPtr _parent);

      /// \brief Constructor for a global multiray shape.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit ODEMultiRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Destructor.
      public: virtual ~ODEMultiRayShape();

      // Documentation inherited.
      public: virtual void UpdateRays();

      /// \brief Ray-intersection callback.
      /// \param[in] _data Pointer to user data.
      /// \param[in] _o1 First geom to check for collisions.
      /// \param[in] _o2 Second geom to check for collisions.
      private: static void UpdateCallback(void *_data, dGeomID _o1,
                                          dGeomID _o2);

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start of a ray.
      /// \param[in] _end End of a ray.
      protected: void AddRay(const ignition::math::Vector3d &_start,
                             const ignition::math::Vector3d &_end);

      /// \brief Space to contain the ray space, for efficiency.
      private: dSpaceID superSpaceId;

      /// \brief Ray space for collision detector.
      private: dSpaceID raySpaceId;

      /// \brief Helper to get the correct ray shape in the UpdateCallback
      /// function.
      private: bool defaultUpdate = true;
    };
    /// \}
  }
}
#endif
