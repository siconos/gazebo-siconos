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
/* Desc: Siconos Link class
 * Author: Nate Koenig
 * Date: 15 May 2009
 */

#ifndef _SICONOSLINK_HH_
#define _SICONOSLINK_HH_

#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \brief siconos physics engine wrapper
    /// \{

    /// \brief Siconos Link class
    class GZ_PHYSICS_VISIBLE SiconosLink : public Link
    {
      /// \brief Constructor
      public: SiconosLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~SiconosLink();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited.
      public: virtual bool GetEnabled() const;

      // Documentation inherited.
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void SetTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
                  const ignition::math::Vector3d &_offset) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
                  const ignition::math::Vector3d &_offset,
                  const ignition::math::Quaterniond &_q) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldCoGLinearVel() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldForce() const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldTorque() const;

      // Documentation inherited.
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited.
      public: virtual bool GetGravityMode() const;

      // Documentation inherited.
      public: virtual void SetSelfCollide(bool _collide);

      /// \brief Get the Siconos dynamical system.
      /// \return Pointer to Siconos body DynamicalSystem object.
      public: SP::BodyDS GetSiconosBodyDS() const;

      /// \brief Get the Siconos contactor.
      /// \return Pointer to SiconosContactor object.
      public: SP::SiconosContactorSet GetSiconosContactorSet() const;

      // Documentation inherited.
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited.
      public: virtual void SetAngularDamping(double _damping);

      /// \brief Set the relative pose of a child collision.
      /*public: void SetCollisionRelativePose(SiconosCollision *collision,
                                            const math::Pose &newPose);
                                            */

      // Documentation inherited.
      public: virtual void AddForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddRelativeForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddForceAtWorldPosition(const ignition::math::Vector3d &_force,
                                                   const ignition::math::Vector3d &_pos);

      // Documentation inherited.
      public: virtual void AddForceAtRelativePosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(const ignition::math::Vector3d &_force,
          const ignition::math::Vector3d &_offset = ignition::math::Vector3d::Zero);

      // Documentation inherited.
      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void AddRelativeTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      // \brief Update pose of link from the Siconos body state
      public: virtual void UpdatePoseFromBody();

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Pointer to siconos compound dynamical system, which
      ///        is a container for other child dynamical systems.
      private: SP::SiconosContactorSet contactorSet;

      /// \brief Pointer to the siconos rigid body object.
      private: SP::BodyDS body;

      /// \brief Pointer to the siconos physics engine.
      private: SiconosPhysicsPtr siconosPhysics;
    };
    /// \}
  }
}
#endif
