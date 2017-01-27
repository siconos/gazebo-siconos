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
#ifndef _ODESCREWJOINT_HH_
#define _ODESCREWJOINT_HH_

#include <string>
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief A screw joint.
    class GZ_PHYSICS_VISIBLE ODEScrewJoint : public ScrewJoint<ODEJoint>
    {
      /// \brief Constructor.
      /// \param[in] _worldId ODE world id.
      /// \param[in] _parent Pointer to the Link that is the joint' parent
      public: ODEScrewJoint(dWorldID _worldId, BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~ODEScrewJoint();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual ignition::math::Vector3d Anchor(
          const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(const unsigned int _index,
                  const ignition::math::Vector3d &_anchor);

      // Documentation inherited
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAxis(const unsigned int _index,
                  const ignition::math::Vector3d &_axis);

      // Documentation inherited
      public: virtual void SetThreadPitch(unsigned int _index,
                  double _threadPitch);

      // Documentation inherited
      public: virtual void SetThreadPitch(double _threadPitch);

      // Documentation inherited
      public: virtual double GetThreadPitch(unsigned int _index);

      // Documentation inherited
      public: virtual double GetThreadPitch();

      // Documentation inherited
      public: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetVelocity(unsigned int _index, double _angle);

      // Documentation inherited
      public: virtual double GetParam(unsigned int _parameter) const;

      // Documentation inherited
      public: virtual void SetParam(unsigned int _parameter, double _value);

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                        unsigned int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                                unsigned int _index);

      // Documentation inherited
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);
    };
    /// \}
  }
}
#endif
