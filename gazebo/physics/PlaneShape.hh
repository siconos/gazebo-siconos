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
#ifndef GAZEBO_PHYSICS_PLANESHAPE_HH_
#define GAZEBO_PHYSICS_PLANESHAPE_HH_

#include <ignition/math/Vector2.hh>

#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class PlaneShape PlaneShape.hh physics/physics.hh
    /// \brief Collision for an infinite plane.
    ///
    /// This collision is used primarily for ground planes.  Note that while
    /// the plane in infinite, only the part near the camera is drawn.
    class GZ_PHYSICS_VISIBLE PlaneShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Link to which we are attached.
      public: explicit PlaneShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~PlaneShape();

      /// \brief Initialize the plane.
      public: virtual void Init();

      /// \brief Create the plane.
      public: virtual void CreatePlane();

      /// \brief Set the altitude of the plane.
      /// \param[in] _pos Position of the plane.
      public: virtual void SetAltitude(const ignition::math::Vector3d &_pos);

      /// \brief Set the normal.
      /// \param[in] _norm Plane normal.
      public: void SetNormal(const ignition::math::Vector3d &_norm);

      /// \brief Get the plane normal.
      /// \return The plane normal.
      public: ignition::math::Vector3d Normal() const;

      /// \brief Set the size.
      /// \param[in] _size 2D size of the plane.
      public: void SetSize(const ignition::math::Vector2d &_size);

      /// \brief Get the size.
      /// \return Size of the plane.
      public: ignition::math::Vector2d Size() const;

      /// \brief Set the scale of the plane.
      /// \return _scale Scale to set the plane to.
      public: virtual void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Fill a geometry message with data from this object.
      /// \param[out] _msg Message to fill.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Process a geometry message and use the data to update
      /// this object.
      /// \param[in] _msg Message to update from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;
    };
    /// \}
  }
}
#endif
