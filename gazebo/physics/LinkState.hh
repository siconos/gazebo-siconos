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
#ifndef GAZEBO_PHYSICS_LINKSTATE_HH_
#define GAZEBO_PHYSICS_LINKSTATE_HH_

#include <iomanip>
#include <iostream>
#include <vector>
#include <string>

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include "gazebo/physics/State.hh"
#include "gazebo/physics/CollisionState.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class LinkState LinkState.hh physics/physics.hh
    /// \brief Store state information of a physics::Link object
    ///
    /// This class captures the entire state of a Link at one
    /// specific time during a simulation run.
    ///
    /// State of a Link includes the state of itself all its child Collision
    /// entities.
    class GZ_PHYSICS_VISIBLE LinkState : public State
    {
      /// \brief Default constructor
      public: LinkState();

      /// \brief Constructor
      ///
      /// Build a LinkState from an existing Link.
      /// \param[in] _model Pointer to the Link from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp
      /// \param[in] _iterations Simulation iterations.
      public: LinkState(const LinkPtr _link, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Constructor
      ///
      /// Build a LinkState from an existing Link.
      /// \param[in] _model Pointer to the Link from which to gather state
      /// info.
      public: explicit LinkState(const LinkPtr _link);

      /// \brief Constructor
      ///
      /// Build a LinkState from SDF data
      /// \param[in] _sdf SDF data to load a link state from.
      public: explicit LinkState(const sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: virtual ~LinkState();

      /// \brief Load a LinkState from a Link pointer.
      ///
      /// Build a LinkState from an existing Link.
      /// \param[in] _model Pointer to the Link from which to gather state
      /// info.
      /// \param[in] _realTime Real time stamp.
      /// \param[in] _simTime Sim time stamp.
      /// \param[in] _iterations Simulation iterations.
      public: void Load(const LinkPtr _link, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations);

      /// \brief Load state from SDF element.
      ///
      /// Load LinkState information from stored data in and SDF::Element.
      /// \param[in] _elem Pointer to the SDF::Element containing state info.
      public: virtual void Load(const sdf::ElementPtr _elem);

      /// \brief Get the link pose.
      /// \return The ignition::math::Pose3d of the Link.
      public: const ignition::math::Pose3d &Pose() const;

      /// \brief Get the link velocity.
      /// \return The velocity represented as a ignition::math::Pose3d.
      public: const ignition::math::Pose3d &Velocity() const;

      /// \brief Get the link acceleration.
      /// \return The acceleration represented as a ignition::math::Pose3d.
      public: const ignition::math::Pose3d &Acceleration() const;

      /// \brief Get the force and torque applied to the Link.
      /// \return The wrench represented as an ignition::math::Pose3d.
      public: const ignition::math::Pose3d &Wrench() const;

      /// \brief Get the number of link states.
      ///
      /// This returns the number of Collisions recorded.
      /// \return Number of CollisionState recorded.
      public: unsigned int GetCollisionStateCount() const;

      /// \brief Get a collision state.
      ///
      /// Get a Collision State based on an index, where index is in the
      /// range of  0...LinkState::GetCollisionStateCount.
      /// \param[in] _index Index of the CollisionState.
      /// \return State of the Collision.
      /// \throws common::Exception When _index is invalid.
      public: CollisionState GetCollisionState(unsigned int _index) const;

      /// \brief Get a link state by link name.
      ///
      /// Searches through all CollisionStates.
      /// Returns the CollisionState with the matching name, if any.
      /// \param[in] _collisionName Name of the CollisionState
      /// \return State of the Collision.
      /// \throws common::Exception When _collisionName is invalid
      public: CollisionState GetCollisionState(
                  const std::string &_collisionName) const;

      /// \brief Get the collision states.
      /// \return A vector of collision states.
      public: const std::vector<CollisionState> &GetCollisionStates() const;

      /// \brief Return true if the values in the state are zero.
      /// \return True if the values in the state are zero.
      public: bool IsZero() const;

      /// \brief Populate a state SDF element with data from the object.
      /// \param[out] _sdf SDF element to populate.
      public: void FillSDF(sdf::ElementPtr _sdf);

      /// \brief Set the wall time when this state was generated
      /// \param[in] _time The absolute clock time when the State
      /// data was recorded.
      public: virtual void SetWallTime(const common::Time &_time);

      /// \brief Set the real time when this state was generated
      /// \param[in] _time Clock time since simulation was stated.
      public: virtual void SetRealTime(const common::Time &_time);

      /// \brief Set the sim time when this state was generated
      /// \param[in] _time Simulation time when the data was recorded.
      public: virtual void SetSimTime(const common::Time &_time);

      /// \brief Set the simulation iterations when this state was generated
      /// \param[in] _iterations Simulation iterations when the data was
      /// recorded.
      public: virtual void SetIterations(const uint64_t _iterations);

      /// \brief Assignment operator
      /// \param[in] _state State value
      /// \return this
      public: LinkState &operator=(const LinkState &_state);

      /// \brief Subtraction operator.
      /// \param[in] _pt A state to substract.
      /// \return The resulting state.
      public: LinkState operator-(const LinkState &_state) const;

      /// \brief Addition operator.
      /// \param[in] _pt A state to add.
      /// \return The resulting state.
      public: LinkState operator+(const LinkState &_state) const;

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _state Link state to output
      /// \return the stream
      public: inline friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::LinkState &_state)
      {
        ignition::math::Vector3d euler(_state.pose.Rot().Euler());
        _out.unsetf(std::ios_base::floatfield);
        _out << std::setprecision(4)
          << "<link name='" << _state.name << "'>"
          << "<pose>"
          << ignition::math::precision(_state.pose.Pos().X(), 4) << " "
          << ignition::math::precision(_state.pose.Pos().Y(), 4) << " "
          << ignition::math::precision(_state.pose.Pos().Z(), 4) << " "
          << ignition::math::precision(euler.X(), 4) << " "
          << ignition::math::precision(euler.Y(), 4) << " "
          << ignition::math::precision(euler.Z(), 4) << " "
          << "</pose>";

        if (_state.RecordVelocity())
        {
          /// Disabling this for efficiency.
          euler = _state.velocity.Rot().Euler();
          _out.unsetf(std::ios_base::floatfield);
          _out << std::setprecision(4)
            << "<velocity>"
            << ignition::math::precision(_state.velocity.Pos().X(), 4) << " "
            << ignition::math::precision(_state.velocity.Pos().Y(), 4) << " "
            << ignition::math::precision(_state.velocity.Pos().Z(), 4) << " "
            << ignition::math::precision(euler.X(), 4) << " "
            << ignition::math::precision(euler.Y(), 4) << " "
            << ignition::math::precision(euler.Z(), 4) << " "
            << "</velocity>";
        }
        // << "<acceleration>" << _state.acceleration << "</acceleration>"
        // << "<wrench>" << _state.wrench << "</wrench>";

        /// Disabling this for efficiency.
        // for (std::vector<CollisionState>::const_iterator iter =
        //      _state.collisionStates.begin();
        //      iter != _state.collisionStates.end(); ++iter)
        // {
        //   _out << *iter;
        // }

        _out << "</link>";

        return _out;
      }

      /// \brief Set whether to record link velocity
      /// \param[in] _record True to record link velocity, false to leave it
      /// out of the log
      public: void SetRecordVelocity(const bool _record);

      /// \brief Get whether link velocity is recorded
      /// \return True if link velocity is recorded
      public: bool RecordVelocity() const;

      /// \brief 3D pose of the link relative to the model.
      private: ignition::math::Pose3d pose;

      /// \brief Velocity of the link (linear and angular)
      /// in the world frame.
      private: ignition::math::Pose3d velocity;

      /// \brief Acceleration of the link (linear and angular)
      /// in the world frame.
      private: ignition::math::Pose3d acceleration;

      /// \brief Force on the link(linear and angular)
      /// in the world frame.
      private: ignition::math::Pose3d wrench;

      /// \brief State of all the child Collision objects.
      private: std::vector<CollisionState> collisionStates;
    };
    /// \}
  }
}
#endif
