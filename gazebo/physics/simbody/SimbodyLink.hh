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
#ifndef GAZEBO_PHYSICS_SIMBODY_SIMBODYLINK_HH_
#define GAZEBO_PHYSICS_SIMBODY_SIMBODYLINK_HH_

#include <vector>

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/physics/simbody/simbody_inc.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_simbody Simbody Physics
    /// \brief simbody physics engine wrapper
    /// \{

    /// \brief Simbody Link class
    class GZ_PHYSICS_VISIBLE SimbodyLink : public Link
    {
      /// \brief Constructor
      public: explicit SimbodyLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~SimbodyLink();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _ptr);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetEnabled(bool enable) const;

      // Documentation inherited.
      public: virtual bool GetEnabled() const;

      // Documentation inherited.
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      // Documentation inherited.
      public: virtual void SetForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void SetTorque(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldLinearVel(
        const ignition::math::Vector3d &_vector3) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset,
          const ignition::math::Quaterniond &_q) const;

      // Documentation inherited.
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

      // Documentation inherited.
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited.
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited.
      public: virtual void AddForce(const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddRelativeForce(
                  const ignition::math::Vector3d &_force);

      // Documentation inherited.
      public: virtual void AddForceAtWorldPosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_pos);

      // Documentation inherited.
      public: virtual void AddForceAtRelativePosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(const ignition::math::Vector3d &_force,
          const ignition::math::Vector3d &_offset =
          ignition::math::Vector3d::Zero);

      // Documentation inherited.
      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void AddRelativeTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited.
      public: virtual void SaveSimbodyState(const SimTK::State &_state);

      // Documentation inherited.
      public: virtual void RestoreSimbodyState(SimTK::State &_state);

      /// \brief If the inboard body of this link is ground, simply
      /// lock the inboard joint to freeze it to ground.  Otherwise,
      /// add a weld constraint to simulate freeze to ground effect.
      /// \param[in] _static if true, freeze link to ground.  Otherwise
      /// unfreeze link.
      public: virtual void SetLinkStatic(bool _static);

      /// \brief Convert Gazebo Inertia to Simbody MassProperties
      /// Where Simbody MassProperties contains mass,
      /// center of mass location, and unit inertia about body origin.
      public: SimTK::MassProperties GetMassProperties() const;

      public: SimTK::MassProperties GetEffectiveMassProps(
        int _numFragments) const;

      /// \brief Set the dirty pose
      /// \param[in] New dirty pose
      public: void SetDirtyPose(const ignition::math::Pose3d &_pose);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Internal call to change effect of gravity on Link
      /// based on gravityMode if gravityModeDirty is true.
      private: void ProcessSetGravityMode();

      /// \brief Internal call to set link static
      /// based on staticLink if staticLinkDirty is true.
      private: void ProcessSetLinkStatic();

      /// \brief: Force this link to be a base body, where its inboard
      /// body is the world with 6DOF.
      public: bool mustBeBaseLink;

      // Below to be filled in after everything is loaded
      // Which MobilizedBody corresponds to the master instance of this link.
      public: SimTK::MobilizedBody masterMobod;

      // Keeps track if physics has been initialized
      public: bool physicsInitialized;

      // If this link got split into a master and slaves, these are the
      // MobilizedBodies used to mobilize the slaves.
      public: std::vector<SimTK::MobilizedBody> slaveMobods;

      // And these are the Weld constraints used to attach slaves to master.
      public: std::vector<SimTK::Constraint::Weld> slaveWelds;

      /// \brief store gravity mode given link might not be around
      private: bool gravityMode;

      /// \brief Trigger setting of link according to staticLink.
      private: bool staticLinkDirty;

      /// \brief Trigger setting of link gravity mode
      private: bool gravityModeDirty;

      /// \brief If true, freeze link to world (inertial) frame.
      private: bool staticLink;

      /// \brief Event connection for SetLinkStatic
      private: event::ConnectionPtr staticLinkConnection;

      /// \brief Event connection for SetGravityMode
      private: event::ConnectionPtr gravityModeConnection;

      /// \brief save simbody free state for reconstructing simbody model graph
      private: std::vector<double> simbodyQ;

      /// \brief save simbody free state for reconstructing simbody model graph
      private: std::vector<double> simbodyU;

      /// \brief keep a pointer to the simbody physics engine for convenience
      private: SimbodyPhysicsPtr simbodyPhysics;
    };
    /// \}
  }
}
#endif
