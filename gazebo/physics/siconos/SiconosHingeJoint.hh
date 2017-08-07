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
#ifndef _GAZEBO_SICONOSHINGEJOINT_HH_
#define _GAZEBO_SICONOSHINGEJOINT_HH_

#include <string>
#include "gazebo/physics/HingeJoint.hh"
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

    /// \brief A single axis hinge joint
    class GZ_PHYSICS_VISIBLE SiconosHingeJoint : public HingeJoint<SiconosJoint>
    {
      ///  Constructor
      /// \param[in] _parent pointer to the parent Model
      /// \param[in] _world pointer to the siconos composite dynamical system
    public: SiconosHingeJoint(BasePtr _parent, SP::SiconosWorld _world);

      /// Destructor
      public: virtual ~SiconosHingeJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual ignition::math::Vector3d Anchor(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetAxis(unsigned int _index,
                  const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _vel);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      // Documentation inherited.
      public: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                    unsigned int _index,
                                    const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                      unsigned int _index);

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);

      // Return the Siconos Relation associated with this joint
      public: virtual SP::NewtonEulerJointR Relation() const;

      /// \brief Pointer to siconos hinge constraint.
      private: SP::PivotJointR siconosPivotJointR;

      /// \brief Initial value of joint axis, expressed as unit vector
      ///        in world frame.
      private: ignition::math::Vector3d initialWorldAxis;
    };
    /// \}
  }
}
#endif
