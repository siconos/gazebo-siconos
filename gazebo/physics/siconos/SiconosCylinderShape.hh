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
#ifndef _GAZEBO_SICONOSCYLINDERSHAPE_HH_
#define _GAZEBO_SICONOSCYLINDERSHAPE_HH_

#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/CylinderShape.hh"
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
    class GZ_PHYSICS_VISIBLE SiconosCylinderShape : public CylinderShape
    {
      /// \brief Constructor
      public: SiconosCylinderShape(CollisionPtr _parent) : CylinderShape(_parent) {}

      /// \brief Destructor
      public: virtual ~SiconosCylinderShape() {}

      /// \brief Set the size of the cylinder
      /// \param[in] _radius Cylinder radius
      /// \param[in] _length Cylinder length
      public: void SetSize(double _radius, double _length)
              {
                if (_radius < 0)
                {
                  gzerr << "Cylinder shape does not support negative radius\n";
                  return;
                }
                if (_length < 0)
                {
                  gzerr << "Cylinder shape does not support negative length\n";
                  return;
                }
                if (ignition::math::equal(_radius, 0.0))
                {
                  // Warn user, but still create shape with very small value
                  // otherwise later resize operations using setLocalScaling
                  // will not be possible
                  gzwarn << "Setting cylinder shape's radius to zero \n";
                  _radius = 1e-4;
                }
                if (ignition::math::equal(_length, 0.0))
                {
                  gzwarn << "Setting cylinder shape's length to zero \n";
                  _length = 1e-4;
                }

                CylinderShape::SetSize(_radius, _length);
                SiconosCollisionPtr bParent;
                bParent = boost::dynamic_pointer_cast<SiconosCollision>(
                    this->collisionParent);

                /// Siconos requires the full extents of the box
                SP::SiconosContactor c(bParent->GetSiconosContactor());
                if (!c)
                {
                  SP::SiconosCylinder cyl(new SiconosCylinder(_radius, _length));
                  cyl->setInsideMargin(std::min(_radius*2, _length)*0.1);
                  bParent->SetCollisionShape(cyl);

                  // The Siconos cylinder requires a 90-degree base rotation
                  // to match Gazebo's.
                  SP::SiconosVector offset(std11::make_shared<SiconosVector>(7));
                  offset->zero();
                  offset->setValue(3, 0.70710678);
                  offset->setValue(4, 0.70710678);
                  bParent->SetBaseTransform(offset);
                }
                else
                {
                  SP::SiconosCylinder cyl(
                    boost::static_pointer_cast<SiconosCylinder>(c->shape));
                  cyl->setRadius(_radius);
                  cyl->setLength(_length);
                  cyl->setInsideMargin(std::min(_radius*2, _length)*0.1);
                }
              }
    };
    /// \}
  }
}
#endif
