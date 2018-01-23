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
#ifndef GAZEBO_PHYSICS_BULLET_BULLETLINK_HH_
#define GAZEBO_PHYSICS_BULLET_BULLETLINK_HH_

#include "gazebo/physics/bullet/bullet_inc.h"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"

class btRigidBody;

namespace gazebo
{
  namespace physics
  {
    class BulletMotionState;

    /// \addtogroup gazebo_physics_bullet
    /// \{

    /// \brief Bullet Link class
    class GZ_PHYSICS_VISIBLE BulletLink : public Link
    {
      /// \brief Constructor
      public: explicit BulletLink(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~BulletLink();

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

      /// \brief Get the bullet rigid body.
      /// \return Pointer to bullet rigid body object.
      public: btRigidBody *GetBulletLink() const;

      /// \brief Remove and re-add this rigid body from the world.
      public: void RemoveAndAddBody() const;

      /// \internal
      /// \brief Clear bullet collision cache needed when the body is resized.
      public: void ClearCollisionCache();

      // Documentation inherited.
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited.
      public: virtual void SetAngularDamping(double _damping);

      /// \brief Set the relative pose of a child collision.
      // public: void SetCollisionRelativePose(BulletCollision *collision,
      // const ignition::math::Pose3d &newPose);

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
      public: virtual void AddLinkForce(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_offset =
                  ignition::math::Vector3d::Zero);

      // Documentation inherited.
      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void AddRelativeTorque(
                  const ignition::math::Vector3d &_torque);

      // Documentation inherited.
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Pointer to bullet compound shape, which is a container
      ///        for other child shapes.
      private: btCollisionShape *compoundShape;

      /// \brief Pointer to bullet motion state, which manages updates to the
      ///        world pose from bullet.
      public: BulletMotionStatePtr motionState;

      /// \brief Pointer to the bullet rigid body object.
      private: btRigidBody *rigidLink;

      /// \brief Pointer to the bullet physics engine.
      private: BulletPhysicsPtr bulletPhysics;
    };
    /// \}
  }
}
#endif
