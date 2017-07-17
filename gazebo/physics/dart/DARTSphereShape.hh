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
#ifndef _GAZEBO_DARTSPHERESHAPE_HH_
#define _GAZEBO_DARTSPHERESHAPE_HH_

#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/SphereShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTSphereShapePrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief A DART sphere shape
    class GZ_PHYSICS_VISIBLE DARTSphereShape : public SphereShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTSphereShape(DARTCollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTSphereShape();

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void SetRadius(double _radius);

      /// \internal
      /// \brief Pointer to private data
      private: DARTSphereShapePrivate *dataPtr;
    };
    /// \}
  }
}
#endif
