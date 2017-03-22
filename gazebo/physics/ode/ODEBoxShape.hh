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
#ifndef _ODEBOXSHAPE_HH_
#define _ODEBOXSHAPE_HH_

#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ODECollision.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/BoxShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief ODE Box shape
    class GZ_PHYSICS_VISIBLE ODEBoxShape : public BoxShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit ODEBoxShape(ODECollisionPtr _parent)
              : BoxShape(_parent) {}

      /// \brief Destructor.
      public: virtual ~ODEBoxShape() {}

      // Documentation inherited.
      public: virtual void SetSize(const ignition::math::Vector3d &_size)
      {
        BoxShape::SetSize(_size);

        ODECollisionPtr oParent;
        oParent = boost::dynamic_pointer_cast<ODECollision>(
            this->collisionParent);

        if (oParent->GetCollisionId() == nullptr)
        {
          oParent->SetCollision(
              dCreateBox(0, _size.X(), _size.Y(), _size.Z()), true);
        }
        else
        {
          dGeomBoxSetLengths(oParent->GetCollisionId(),
              _size.X(), _size.Y(), _size.Z());
        }
      }
    };
    /// \}
  }
}
#endif
