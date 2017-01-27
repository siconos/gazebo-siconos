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
#ifndef GAZEBO_PHYSICS_ODE_ODELINK_HH_
#define GAZEBO_PHYSICS_ODE_ODELINK_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief ODE Link class.
    class GZ_PHYSICS_VISIBLE ODELink : public Link
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent model.
      public: explicit ODELink(EntityPtr _parent);

      /// \brief Destructor.
      public: virtual ~ODELink();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited
      public: virtual bool GetEnabled() const;

      /// \brief Update location of collisions relative to center of mass.
      /// This used to be done only in the Init function, but was moved
      /// to a separate function to handle dynamic updates to the
      /// center of mass.
      public: void UpdateCollisionOffsets();

      // Documentation inherited
      public: virtual void UpdateMass();

      // Documentation inherited
      public: virtual void UpdateSurface();

      // Documentation inherited
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited
      public: virtual void SetForce(const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void SetTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual void AddForce(const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(
                  const ignition::math::Vector3d &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_offset =
                  ignition::math::Vector3d::Zero);

      // Documentation inherited
      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldLinearVel(
                  const ignition::math::Vector3d &_offset,
                  const ignition::math::Quaterniond &_q) const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldCoGLinearVel() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldAngularVel() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldForce() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d WorldTorque() const;

      // Documentation inherited
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited
      public: virtual bool GetGravityMode() const;

      // Documentation inherited
      public: void SetSelfCollide(bool _collide);

      // Documentation inherited
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited
      public: virtual void SetKinematic(const bool &_state);

      // Documentation inherited
      public: virtual bool GetKinematic() const;

      // Documentation inherited
      public: virtual void SetAutoDisable(bool _disable);

      /// \brief Return the ID of this link
      /// \return ODE link id
      public: dBodyID GetODEId() const;

      /// \brief Get the ID of the collision space this link is in.
      /// \return The collision space ID for the link.
      public: dSpaceID GetSpaceId() const;

      /// \brief Set the ID of the collision space this link is in.
      /// \param[in] _spaceId The collision space ID for the link.
      public: void SetSpaceId(dSpaceID _spaceid);

      /// \brief Callback when ODE determines a body is disabled.
      /// \param[in] _id Id of the body.
      public: static void DisabledCallback(dBodyID _id);

      /// \brief when ODE updates dynamics bodies, this callback
      ///        propagates the chagnes in pose back to Gazebo
      /// \param[in] _id Id of the body.
      public: static void MoveCallback(dBodyID _id);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      /// \brief ODE link handle
      private: dBodyID linkId;

      /// \brief Pointer to the ODE Physics engine
      private: ODEPhysicsPtr odePhysics;

      /// \brief Collision space id.
      private: dSpaceID spaceId;

      /// \brief Cache force applied on body
      private: ignition::math::Vector3d force;

      /// \brief Cache torque applied on body
      private: ignition::math::Vector3d torque;
    };
    /// \}
  }
}
#endif
