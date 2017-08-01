/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SICONOSBOXSHAPE_HH_
#define _GAZEBO_SICONOSBOXSHAPE_HH_

#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/BoxShape.hh"
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

    /// \brief Siconos box collision
    class GZ_PHYSICS_VISIBLE SiconosBoxShape : public BoxShape
    {
      /// \brief Constructor
      public: SiconosBoxShape(CollisionPtr _parent) : BoxShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SiconosBoxShape() {}

      /// \brief Set the size of the box
      public: void SetSize(const ignition::math::Vector3d &_size)
              {
                if (_size.X() < 0 || _size.Y() < 0 || _size.Z() < 0)
                {
                  gzerr << "Box shape does not support negative size\n";
                  return;
                }
                ignition::math::Vector3d size = _size;
                if (ignition::math::equal(size.X(), 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting box shape's x to zero \n";
                  size.X() = 1e-4;
                }
                if (ignition::math::equal(size.Y(), 0.0))
                {
                  gzwarn << "Setting box shape's y to zero \n";
                  size.Y() = 1e-4;
                }
                if (ignition::math::equal(size.Z(), 0.0))
                {
                  gzwarn << "Setting box shape's z to zero \n";
                  size.Z() = 1e-4;
                }

                BoxShape::SetSize(size);
                SiconosCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SiconosCollision>(
                    this->collisionParent);

                /// Siconos requires the full extents of the box
                SP::SiconosContactor c(bParent->GetSiconosContactor());
                if (c->shape)
                {
                  SP::SiconosBox box(boost::static_pointer_cast<SiconosBox>(c->shape));
                  box->setDimensions(size.X(), size.Y(), size.Z());
                  box->setInsideMargin(std::min(std::min(size.X(), size.Y()),
                                                size.Z())*0.1);
                }
                else
                {
                  this->initialSize = size;
                  SP::SiconosBox box(new SiconosBox(size.X(), size.Y(), size.Z()));
                  box->setInsideMargin(std::min(std::min(size.X(), size.Y()),
                                                size.Z())*0.1);
                  bParent->SetCollisionShape(box);
                }
              }

      /// \brief Initial size of box.
      private: ignition::math::Vector3d initialSize;
    };
    /// \}
  }
}
#endif
