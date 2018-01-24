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
#ifndef _GAZEBO_SICONOSGEARBOXJOINT_HH_
#define _GAZEBO_SICONOSGEARBOXJOINT_HH_

#include <string>
#include "gazebo/physics/GearboxJoint.hh"
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

    /// \brief A single axis gearbox joint
    class GZ_PHYSICS_VISIBLE SiconosGearboxJoint : public GearboxJoint<SiconosJoint>
    {
      ///  Constructor
      /// \param[in] _parent pointer to the parent Model
      /// \param[in] _world pointer to the siconos composite dynamical system
      public: SiconosGearboxJoint(BasePtr _parent, SP::SiconosWorld _world);

      /// Destructor
      public: virtual ~SiconosGearboxJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      bool IsInitialized() const;

      /// \brief Do nothing if anchor point was set
      public: virtual void SetAnchor(unsigned int,
                                     const ignition::math::Vector3d &) {}

      /// \brief Do nothing if axis was set
      public: virtual void SetAxis(const unsigned int,
                                   const ignition::math::Vector3d &) {}

      // Documentation inherited
      public: virtual void SetGearboxRatio(double _gearRatio);

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

      /// \brief Set gearbox joint gear reference body
      /// \param[in] _body a Siconos body as the reference link for the gears.
      private: void SetReferenceBody(LinkPtr _body);

      /// \brief Pointers to reference joints
      private: SiconosJointPtr joints[2];

      /// \brief Pointer to siconos coupler constraint.
      private: SP::CouplerJointR siconosCouplerJointR;

      /// \brief Pointer to siconos interaction for the coupler constraint.
      private: SP::Interaction siconosCouplerInteraction;

      /// \brief Pointer to link used as reference.
      private: SiconosLinkPtr referenceLink;
    };
    /// \}
  }
}
#endif
