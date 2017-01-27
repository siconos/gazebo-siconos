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

#ifndef _SICONOSSPHERESHAPE_HH_
#define _SICONOSSPHERESHAPE_HH_

#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/SphereShape.hh"
#include "gazebo/util/system.hh"

#include <siconos/SiconosShape.hpp>
#include <siconos/SiconosContactor.hpp>

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Siconos sphere collision
    class GZ_PHYSICS_VISIBLE SiconosSphereShape : public SphereShape
    {
      /// \brief Constructor
      public: SiconosSphereShape(CollisionPtr _parent) : SphereShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SiconosSphereShape() {}

      /// \brief Set the radius
      /// \param[in] _radius Sphere radius
      public: void SetRadius(double _radius)
              {
                if (_radius < 0)
                {
                  gzerr << "Sphere shape does not support negative radius\n";
                  return;
                }
                if (ignition::math::equal(_radius, 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting sphere shape's radius to zero \n";
                  _radius = 1e-4;
                }

                SphereShape::SetRadius(_radius);
                SiconosCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SiconosCollision>(
                    this->collisionParent);

                SP::SiconosContactor c(bParent->GetSiconosContactor());
                if (!c)
                {
                  this->initialSize.X() = _radius;
                  this->initialSize.Y() = _radius;
                  this->initialSize.Z() = _radius;
                  SP::SiconosSphere sphere(new SiconosSphere(_radius));
                  sphere->setInsideMargin(_radius/2);
                  bParent->SetCollisionShape(sphere, true);
                }
                else
                {
                  SP::SiconosSphere sphere(
                    boost::static_pointer_cast<SiconosSphere>(c->shape));
                  sphere->setRadius(_radius);
                  sphere->setInsideMargin(_radius/2);
                  bParent->SetCollisionShape(sphere, true);
                }
              }

      /// \brief Initial size of sphere.
      private: ignition::math::Vector3d initialSize;
    };
    /// \}
  }
}
#endif
