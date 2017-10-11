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
#ifndef _GAZEBO_SICONOSSLIDERJOINT_HH_
#define _GAZEBO_SICONOSSLIDERJOINT_HH_

#include <string>
#include "gazebo/physics/siconos/SiconosJoint.hh"
#include "gazebo/physics/SliderJoint.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/util/system.hh"

class btSliderConstraint;

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief A slider joint
    class GZ_PHYSICS_VISIBLE SiconosSliderJoint : public SliderJoint<SiconosJoint>
    {
      /// \brief Constructor
      public: SiconosSliderJoint(BasePtr _parent, SP::SiconosWorld world);

      /// \brief Destructor
      public: virtual ~SiconosSliderJoint();

      /// \brief Load the SiconosSliderJoint
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      protected: virtual void Init();

      // Documentation inherited.
      public: virtual void SetDamping(unsigned int _index,
                  const double _damping);

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited.
      public: virtual double PositionImpl(unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);

      /// \brief Pointer to siconos slider constraint.
      private: SP::PrismaticJointR siconosPrismaticJointR;
    };

  /// \}
  }
}
#endif
