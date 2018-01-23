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
#ifndef GAZEBO_PHYSICS_ODE_ODEHEIGHTMAPSHAPE_HH_
#define GAZEBO_PHYSICS_ODE_ODEHEIGHTMAPSHAPE_HH_

#include <vector>

#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief ODE Height map collision.
    class GZ_PHYSICS_VISIBLE ODEHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Collision parent.
      public: explicit ODEHeightmapShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~ODEHeightmapShape();

      // Documentation inerited.
      public: virtual void Init();

      /// \brief Called by ODE to get the height at a vertex.
      /// \param[in] _data Pointer to the heightmap data.
      /// \param[in] _x X location.
      /// \param[in] _y Y location.
      private: static dReal GetHeightCallback(void *_data, int _x, int _y);

      /// \brief The heightmap data.
      private: dHeightfieldDataID odeData;
    };
    /// \}
  }
}
#endif
