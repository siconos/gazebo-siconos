/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SICONOSMESHSHAPE_HH_
#define _GAZEBO_SICONOSMESHSHAPE_HH_

#include "gazebo/physics/MeshShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Triangle mesh collision
    class GZ_PHYSICS_VISIBLE SiconosMeshShape : public MeshShape
    {
      /// \brief Constructor
      public: SiconosMeshShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~SiconosMeshShape();

      /// \brief Load the trimesh
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the mesh shape.
      protected: virtual void Init();

      /// \brief Siconos collision mesh helper class
      private: SP::SiconosMesh siconosMesh;
    };
    /// \}
  }
}
#endif
