/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SICONOSFIXEDJOINT_HH_
#define _GAZEBO_SICONOSFIXEDJOINT_HH_

#include <string>
#include "gazebo/physics/FixedJoint.hh"
#include "gazebo/physics/siconos/SiconosJoint.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/util/system.hh"

#include <SiconosFwd.hpp>

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief A fixed joint.
    class GZ_PHYSICS_VISIBLE SiconosFixedJoint : public FixedJoint<SiconosJoint>
    {
      /// \brief Constructor
      /// \param[in] _parent pointer to the parent Model
      /// \param[in] world pointer to the siconos composite dynamical system
      public: SiconosFixedJoint(BasePtr _parent, SP::SiconosWorld _world);

      /// \brief Destructor
      public: virtual ~SiconosFixedJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _vel);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);

      /// \brief Pointer to siconos fixed constraint implementation
      // TODO: FixedJointR does not yet exist in Siconos!
      private: SP::FixedJointR siconosFixedJointR;
    };
    /// \}
  }
}
#endif
