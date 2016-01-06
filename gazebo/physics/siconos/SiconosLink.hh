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
      public: virtual void SetLinearVel(const math::Vector3 &_vel);

      // Documentation inherited.
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      // Documentation inherited.
      public: virtual void SetForce(const math::Vector3 &_force);

      // Documentation inherited.
      public: virtual void SetTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset,
                  const math::Quaternion &_q) const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldCoGLinearVel() const;

      // Documentation inherited.
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited.
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited.
      public: virtual math::Vector3 GetWorldTorque() const;

      // Documentation inherited.
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited.
      public: virtual bool GetGravityMode() const;

      // Documentation inherited.
      public: virtual void SetSelfCollide(bool _collide);

      /// \brief Get the siconos dynamical system.
      /// \return Pointer to siconos DynamicalSystem object.
      public: DynamicalSystem *GetSiconosLink() const;

      /// \internal
      /// \brief Clear siconos collision cache needed when the body is resized.
      public: void ClearCollisionCache();

      // Documentation inherited.
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited.
      public: virtual void SetAngularDamping(double _damping);

      /// \brief Set the relative pose of a child collision.
      /*public: void SetCollisionRelativePose(SiconosCollision *collision,
                                            const math::Pose &newPose);
                                            */

      // Documentation inherited.
      public: virtual void AddForce(const math::Vector3 &_force);

      // Documentation inherited.
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      // Documentation inherited.
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      // Documentation inherited.
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relpos);

      // Documentation inherited
      public: virtual void AddLinkForce(const math::Vector3 &_force,
          const math::Vector3 &_offset = math::Vector3::Zero);

      // Documentation inherited.
      public: virtual void AddTorque(const math::Vector3 &_torque);

      // Documentation inherited.
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      // Documentation inherited.
      public: virtual void SetAutoDisable(bool _disable);

      // Documentation inherited
      public: virtual void SetLinkStatic(bool _static);

      // Documentation inherited.
      public: virtual void UpdateMass();

      /// \brief Pointer to siconos compound dynamical system, which
      ///        is a container for other child dynamical systems.
      private: DynamicalSystem *compoundDS;

      /// \brief Pointer to the siconos rigid body object.
      private: DynamicalSystem *rigidLink;

      /// \brief Pointer to the siconos physics engine.
      private: SiconosPhysicsPtr siconosPhysics;
    };
    /// \}
  }
}
#endif
