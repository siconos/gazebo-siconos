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

#ifndef _SICONOSJOINT_HH_
#define _SICONOSJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

#include <SiconosFwd.hpp>

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_siconos Siconos Physics
    /// \{

    /// \brief Base class for all joints
    class GZ_PHYSICS_VISIBLE SiconosJoint : public Joint
    {
      /// \brief Constructor
      public: SiconosJoint(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~SiconosJoint();

      /// \brief Load a SiconosJoint
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Reset the joint
      public: virtual void Reset();

      /// \brief Get the body to which the joint is attached
      ///        according the _index
      public: LinkPtr GetJointLink(unsigned int _index) const;

      /// \brief Determines of the two bodies are connected by a joint
      public: bool AreConnected(LinkPtr _one, LinkPtr _two) const;

      /// \brief Determines if the joint is established between its two bodies
      public: bool IsConnected() const;

      /// \brief Attach the two bodies with this joint.
      /// \param[in] _parent Parent link.
      /// \param[in] _child Child link.
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);

      /// \brief Detach this joint from all bodies
      public: virtual void Detach();

      /// \brief Set the anchor point
      public: virtual void SetAnchor(unsigned int _index,
                                     const ignition::math::Vector3d &_anchor);

      // Documentation inherited
      public: virtual void SetDamping(unsigned int _index, double _damping);

      // Documentation inherited.
      public: virtual bool SetPosition(unsigned int _index, double _position);

      // Documentation inherited.
      public: virtual void SetStiffness(unsigned int _index,
                  const double _stiffness);

      // Documentation inherited.
      public: virtual void SetStiffnessDamping(unsigned int _index,
        double _stiffness, double _damping, double _reference = 0);

      /// \brief Get the anchor point
      public: virtual ignition::math::Vector3d Anchor(unsigned int _index) const;

      /// \brief Get the force the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual ignition::math::Vector3d LinkForce(unsigned int _index) const;

      /// \brief Get the torque the joint applies to the first body
      /// \param index The index of the body(0 or 1)
      public: virtual ignition::math::Vector3d LinkTorque(unsigned int _index) const;

      // Documentation inherited.
      public: virtual void SetUpperLimit(unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual void SetLowerLimit(unsigned int _index,
                                         const double _limit);

      // Documentation inherited.
      public: virtual bool SetParam(const std::string &_key,
                                        unsigned int _index,
                                        const boost::any &_value);

      // Documentation inherited.
      public: virtual double GetParam(const std::string &_key,
                                          unsigned int _index);

      /// \brief Set the joint friction for an axis index.
      /// \param _index The axis index to set friction for.
      /// \param _value The friction value to set.
      /// \return True if the friction was set successfully to the desired value.
      public: virtual bool SetFriction(unsigned int _index, double _value);

      // Documentation inherited.
      public: virtual void SetProvideFeedback(bool _enable);

      // Documentation inherited.
      public: virtual void CacheForceTorque();

      // Documentation inherited.
      public: virtual JointWrench GetForceTorque(unsigned int _index);

      // Documentation inherited.
      public: virtual void SetForce(unsigned int _index, double _force);

      // Documentation inherited.
      public: virtual double GetForce(unsigned int _index);

      // Documentation inherited.
      public: virtual void Init();

      /// \brief Check if Attach() can be safely called on this joint.
      /// \return True if Attach() can be safely called on this joint.
      public: virtual bool IsInitialized();

      // Documentation inherited.
      public: virtual void ApplyStiffnessDamping();

      /// \brief Set the force applied to this physics::Joint.
      /// Note that the unit of force should be consistent with the rest
      /// of the simulation scales.
      /// Force is additive (multiple calls
      /// to SetForceImpl to the same joint in the same time
      /// step will accumulate forces on that Joint).
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      /// internal force, e.g. damping forces.  This way, Joint::appliedForce
      /// keep track of external forces only.
      protected: virtual void SetForceImpl(unsigned int _index,
                     double _force) = 0;

      /// \brief: Setup joint feedback datatructure.
      /// This is called after Relation is setup in Init.
      protected: void SetupJointFeedback();

      /// \brief: Setup joint limits.
      /// This is called after Relation is setup in Init.
      protected: virtual void SetupJointLimits();

      /// \brief Save external forces applied to this Joint.
      /// \param[in] _index Index of the axis.
      /// \param[in] _force Force value.
      private: void SaveForce(unsigned int _index, double _force);

      /// \brief Return the Siconos Relation associated with this joint
      /// \param[in] _index Index of the axis.
      /// \return The NewtonEulerJointR relation assocated with the
      ///         axis.  Not necessarily unique to the desired axis!
      public: virtual SP::NewtonEulerJointR Relation(unsigned int _index) const;

      /// \brief Return the Siconos Interaction associated with this joint
      /// \param[in] _index Index of the axis.
      /// \return The Interaction assocated with the axis.  Not
      ///         necessarily unique to the desired axis!
      public: virtual SP::Interaction Interaction(unsigned int _index) const;

      struct JointInteractionPair {
        SP::NewtonEulerJointR relation;
        SP::Interaction interaction;
      };

      /// \brief Pointer to Relation/Interactions for this joint in Siconos.
      protected: std::vector<JointInteractionPair> relInterPairs;

      /// \brief Pointer to Siconos' composite dynamical system.
      protected: SP::SiconosWorld siconosWorld;

      struct StopInteractionPair {
        SP::JointStopR relation;
        SP::Interaction interaction;
      };

      /// \brief Stop relations for upper limit per axis, only exist if set.
      protected: std::vector< StopInteractionPair > upperStops;

      /// \brief Stop relations for lower limit per axis, only exist if set.
      protected: std::vector< StopInteractionPair > lowerStops;

      struct FrictionInteraction {
        SP::JointFrictionR relation;
        SP::Interaction interaction;
        double value;
      };

      /// \brief Friction relations for friction per axis, only exist if set.
      protected: std::vector< FrictionInteraction > jointFriction;

      /// \brief Global storage for friction-related NonSmoothLaws.
      protected: static std::map< double, SP::RelayNSL > frictionNSL;

      /// \brief Feedback data for this joint
      // private: btJointFeedback *feedback;

      /// \brief internal variable to keep track if ConnectJointUpdate
      /// has been called on a damping method
      private: bool stiffnessDampingInitialized;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by Siconos to store external force applied by the user.
      private: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      private: common::Time forceAppliedTime;

      /// \brief Establish the Interactions for this joint in the Siconos graph.
      /// \return True if connection was successful, false if there was a problem.
      protected: bool SiconosConnect();

      /// \brief Called by SiconosConnect() to establish the
      /// joint-related Interactions; should be not be called
      /// externally, but may be overridden by individual joints.
      protected: virtual void SiconosConnectJoint(SP::BodyDS ds1, SP::BodyDS ds2);

      /// \brief Remove the Interactions for this joint from the Siconos graph.
      protected: void SiconosDisconnect();
    };
    /// \}
  }
}
#endif
