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

#ifndef _SIMBODY_SCREWJOINT_HH_
#define _SIMBODY_SCREWJOINT_HH_

#include <string>
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \{

    /// \brief A screw joint
    class GZ_PHYSICS_VISIBLE SimbodyScrewJoint : public ScrewJoint<SimbodyJoint>
    {
      /// \brief Constructor
      /// \param[in] _world Pointer to the Simbody world.
      /// \param[in] _parent Parent of the screw joint.
      public: SimbodyScrewJoint(SimTK::MultibodySystem *_world,
                  BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyScrewJoint();

      // Documentation inherited.
      protected: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void SetAxis(const unsigned int _index,
                  const ignition::math::Vector3d &_axis);

      // Documentation inherited.
      public: virtual void SetUpperLimit(const unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual void SetLowerLimit(const unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual double UpperLimit(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual double LowerLimit(const unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetThreadPitch(unsigned int _index,
                  double _threadPitch);

      // Documentation inherited.
      public: virtual void SetThreadPitch(double _threadPitch);

      // Documentation inherited.
      public: virtual double GetThreadPitch(unsigned int /*_index*/);

      // Documentation inherited.
      public: virtual double GetThreadPitch();

      // Documentation inherited.
      public: virtual double GetVelocity(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetVelocity(unsigned int _index, double _angle);

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
      protected: virtual void SetForceImpl(unsigned int _index, double _force);
    };
    /// \}
  }
}
#endif
