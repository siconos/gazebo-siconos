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
#ifndef _GAZEBO_SICONOSHINGE2JOINT_HH_
#define _GAZEBO_SICONOSHINGE2JOINT_HH_

#include <string>
#include "gazebo/physics/Hinge2Joint.hh"
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

    /// \brief A two axis hinge joint
    class GZ_PHYSICS_VISIBLE SiconosHinge2Joint : public Hinge2Joint<SiconosJoint>
    {
      ///  Constructor
      /// \param[in] _parent pointer to the parent Model
      /// \param[in] _world pointer to the siconos composite dynamical system
    public: SiconosHinge2Joint(BasePtr _parent, SP::SiconosWorld _world);

      /// Destructor
      public: virtual ~SiconosHinge2Joint();

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

      // Return the Siconos Relation associated with this joint
      public: virtual SP::NewtonEulerJointR Relation(unsigned int _index) const;

      /// \brief Return the point index associated with this joint's Relation.
      /// \param[in] _index Index of the joint anchor.
      /// \return The index of the Relation point for the joint anchor.
      public: virtual unsigned int RelationPointIndex(unsigned int _index) const;

      /// \brief Return the axis index associated with this joint's Relation.
      /// \param[in] _index Index of the joint axis.
      /// \return The index of the Relation axis for the joint axis.
      public: virtual unsigned int RelationAxisIndex(unsigned int _index) const;

      /// \brief Pointer to first siconos hinge constraint.
      private: SP::PivotJointR siconosPivotJointR1;

      /// \brief Pointer to second siconos hinge constraint.
      private: SP::PivotJointR siconosPivotJointR2;

      /// \brief Pointer to siconos DS used as a coupler.
      private: SP::NewtonEulerDS siconosCouplerDS;

      /// \brief Initial value of joint axis, expressed as unit vector
      ///        in world frame.
      private: ignition::math::Vector3d initialWorldAxis;

      /// \brief Called by SiconosConnect() to establish the
      /// joint-related Interactions; this joint connects an
      /// intermediate body between ds1 and ds2.
      protected: void SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2);

      /// \brief Called by SiconosDisconnect() after unlinking the
      /// joint-related Interactions; should be not be called
      /// externally, but may be overridden by individual joints.
      protected: void SiconosDisconnectJoint();
    };
    /// \}
  }
}
#endif
