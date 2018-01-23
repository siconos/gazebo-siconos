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
#ifndef _GAZEBO_SICONOSSCREWJOINT_HH_
#define _GAZEBO_SICONOSSCREWJOINT_HH_

#include <string>
#include "gazebo/physics/ScrewJoint.hh"
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

    /// \brief A single axis screw joint
    class GZ_PHYSICS_VISIBLE SiconosScrewJoint : public ScrewJoint<SiconosJoint>
    {
      ///  Constructor
      /// \param[in] _parent pointer to the parent Model
      /// \param[in] _world pointer to the siconos composite dynamical system
    public: SiconosScrewJoint(BasePtr _parent, SP::SiconosWorld _world);

      /// Destructor
      public: virtual ~SiconosScrewJoint();

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

      // Documentation inherited.
      protected: virtual void SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2);

      // Documentation inherited.
      protected: virtual void SiconosDisconnectJoint();

      // Documentation inherited.
      public: virtual void SetThreadPitch(double _threadPitch);

      // Documentation inherited.
      public: virtual double GetThreadPitch();

      /// \brief Pointer to siconos cylindrical constraint.
      private: SP::CylindricalJointR siconosCylindricalJointR;

      /// \brief Pointer to siconos coupler constraint.
      private: SP::CouplerJointR siconosCouplerJointR;

      /// \brief Pointer to siconos interaction for the coupler constraint.
      private: SP::Interaction siconosCouplerInteraction;
    };
    /// \}
  }
}
#endif
