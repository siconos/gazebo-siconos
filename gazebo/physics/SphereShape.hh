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
#ifndef GAZEBO_PHYSICS_SPHERESHAPE_HH_
#define GAZEBO_PHYSICS_SPHERESHAPE_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SphereShape SphereShape.hh physics/physics.hh
    /// \brief Sphere collision shape.
    class GZ_PHYSICS_VISIBLE SphereShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision object.
      public: explicit SphereShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~SphereShape();

      /// \brief Initialize the sphere.
      public: virtual void Init();

      /// \brief Set the size.
      /// \param[in] _radius Radius of the sphere.
      public: virtual void SetRadius(double _radius);

      /// \brief Get the sphere's radius.
      /// \return Radius of the sphere.
      public: double GetRadius() const;

      // Documentation inherited
      public: virtual void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Fill in the values for a geomertry message.
      /// \param[out] _msg The geometry message to fill.
      public: virtual void FillMsg(msgs::Geometry &_msg);

      /// \brief Process a geometry message.
      /// \param[in] _msg The message to set values from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;
    };
    /// \}
  }
}
#endif
