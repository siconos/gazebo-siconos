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
/* Desc: A screw or primastic joint
 * Author: Nate Koenig
 * Date: 24 May 2009
 */

#ifndef _BULLETSCREWJOINT_HH_
#define _BULLETSCREWJOINT_HH_

#include <string>

#include "gazebo/physics/bullet/BulletJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class btScrewConstraint;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief A screw joint
    class GZ_PHYSICS_VISIBLE BulletScrewJoint : public ScrewJoint<BulletJoint>
    {
      /// \brief Constructor
      /// \param[in] _world Pointer to the dynamics world.
      /// \param[in] _parent Pointer to the parent.
      public: BulletScrewJoint(btDynamicsWorld *_world, BasePtr _parent);

      /// \brief Destructor
      public: virtual ~BulletScrewJoint();

      /// \brief Load the BulletScrewJoint
      /// \param[in] _sdf SDF values.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual ignition::math::Vector3d Anchor(
          const unsigned int _index) const;

      // Documentation inherited
      public: virtual void SetAnchor(const unsigned int _index,
                  const ignition::math::Vector3d &_anchor);

      // Documentation inherited.
      public: virtual void Init();

      /// \brief Set the axis of motion
      /// \param[in] _index Axis index.
      /// \param[in] _axis Axis value.
      public: void SetAxis(const unsigned int _index,
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

      // Documentation inherited.
      public: virtual void SetUpperLimit(const unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual void SetLowerLimit(const unsigned int _index,
                                         const double _limit);

      /// \brief Get the rate of change
      /// \param[in] _index Axis index.
      public: virtual double GetVelocity(unsigned int _index) const;

      /// \brief Set the velocity of an axis(index).
      /// \param[in] _index Axis index.
      /// \param[in] _vel Velocity to apply to the joint axis.
      public: virtual void SetVelocity(unsigned int _index, double _vel);

      /// \brief Get the axis of rotation
      /// \param[in] _index Axis index.
      /// \return The axis in world coordinate frame.
      public: virtual ignition::math::Vector3d GlobalAxis(
          const unsigned int _index) const;

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                  unsigned int _index);

      // Documentation inherited.
      protected: virtual double PositionImpl(const unsigned int _index) const;

      // Documentation inherited.
      protected: virtual void SetForceImpl(unsigned int _index, double _force);

      /// \brief Pointer to bullet screw constraint
      private: btScrewConstraint *bulletScrew;

      /// \brief Initial value of joint axis, expressed as unit vector
      ///        in world frame.
      private: ignition::math::Vector3d initialWorldAxis;
    };
    /// \}
  }
}
#endif
