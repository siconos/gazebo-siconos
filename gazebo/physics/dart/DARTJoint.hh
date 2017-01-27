/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTJOINT_HH_
#define _GAZEBO_DARTJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTJointPrivate;

    /// \addtogroup gazebo_physics_dart
    /// \{

    /// \brief DART joint interface
    class GZ_PHYSICS_VISIBLE DARTJoint : public Joint
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the Joint.
      public: DARTJoint(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTJoint();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize a joint.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual LinkPtr GetJointLink(unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      // Documentation inherited.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      // Documentation inherited.
      public: virtual void CacheForceTorque();

      // Documentation inherited.
      public: virtual void Detach();

      /// \brief Set the anchor point
      public: virtual void SetAnchor(const unsigned int /*_index*/,
          const ignition::math::Vector3d &/*_anchor*/);

      // Documentation inherited
      public: virtual void SetDamping(unsigned int _index, double _damping);

      // Documentation inherited.
      public: virtual void SetStiffness(unsigned int _index,
                  const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0);

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
      public: virtual ignition::math::Vector3d LinkForce(
          const unsigned int _index) const;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d LinkTorque(
          const unsigned int _index) const;

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                        unsigned int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                          unsigned int _index);

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void ApplyDamping();

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      protected: virtual void SetForceImpl(unsigned int _index,
                     double _force) = 0;

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(unsigned int _index, double _force);

      /// \brief Get DART model pointer.
      /// \return A pointer to the DART model.
      public: DARTModelPtr GetDARTModel() const;

      /// \brief Get DART Joint properties
      public: DARTJointPropPtr DARTProperties() const;

      /// \brief Set DART joint pointer.
      /// \param[in] A pointer to the DART joint.
      public: void SetDARTJoint(dart::dynamics::Joint *_dtJoint);

      /// \brief Get DART joint pointer.
      /// \return A pointer to the DART joint.
      public: dart::dynamics::Joint *GetDARTJoint();

      /// \internal
      /// \brief Pointer to private data
      protected: DARTJointPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
