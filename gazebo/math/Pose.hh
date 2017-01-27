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
#ifndef _GAZEBO_POSE_HH_
#define _GAZEBO_POSE_HH_

#include <iostream>

#include <ignition/math/Pose3.hh>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/util/system.hh"

#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class Pose Pose.hh math/gzmath.hh
    /// \brief Encapsulates a position and rotation in three space
    class GZ_MATH_VISIBLE Pose
    {
      /// \brief math::Pose(0, 0, 0, 0, 0, 0)
      public: static const Pose Zero;

      /// \brief Default constructors
      public: Pose() GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _pos A position
      /// \param[in] _rot A rotation
      public: Pose(const Vector3 &_pos, const Quaternion &_rot)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Yaw (rotation about z-axis) in radians.
      public: Pose(double _x, double _y, double _z,
                   double _roll, double _pitch, double _yaw)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor
      /// \param[in] _pose Pose to copy
      public: Pose(const Pose &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor for ignition math
      /// \param[in] _pose Pose to copy
      public: Pose(const ignition::math::Pose3d &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Destructor
      public: virtual ~Pose() GAZEBO_DEPRECATED(8.0);

      /// \brief Set the pose from a Vector3 and a Quaternion
      /// \param[in] _pos The position.
      /// \param[in] _rot The rotation.
      public: void Set(const Vector3 &_pos, const Quaternion &_rot)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Set the pose from  pos and rpy vectors
      /// \param[in] _pos The position.
      /// \param[in] _rpy The rotation expressed as Euler angles.
      public: void Set(const Vector3 &_pos, const Vector3 &_rpy)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Set the pose from a six tuple.
      /// \param[in] _x x position in meters.
      /// \param[in] _y y position in meters.
      /// \param[in] _z z position in meters.
      /// \param[in] _roll Roll (rotation about X-axis) in radians.
      /// \param[in] _pitch Pitch (rotation about y-axis) in radians.
      /// \param[in] _yaw Pitch (rotation about z-axis) in radians.
      public: void Set(double _x, double _y, double _z,
                       double _roll, double _pitch, double _yaw)
          GAZEBO_DEPRECATED(8.0);

      /// \brief See if a pose is finite (e.g., not nan)
      public: bool IsFinite() const GAZEBO_DEPRECATED(8.0);

      /// \brief Fix any nan values
      public: inline void Correct() GAZEBO_DEPRECATED(8.0)
              {
                this->pos.Correct();
                this->rot.Correct();
              }

      /// \brief Get the inverse of this pose
      /// \return the inverse pose
      public: Pose GetInverse() const GAZEBO_DEPRECATED(8.0);

      /// \brief Addition operator
      /// A is the transform from O to P specified in frame O
      /// B is the transform from P to Q specified in frame P
      /// then, B + A is the transform from O to Q specified in frame O
      /// \param[in] _pose Pose to add to this pose
      /// \return The resulting pose
      public: Pose operator+(const Pose &_pose) const GAZEBO_DEPRECATED(8.0);

      /// \brief Add-Equals operator
      /// \param[in] _pose Pose to add to this pose
      /// \return The resulting pose
      public: const Pose &operator+=(const Pose &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Negation operator
      /// A is the transform from O to P in frame O
      /// then -A is transform from P to O specified in frame P
      /// \return The resulting pose
      public: inline Pose operator-() const GAZEBO_DEPRECATED(8.0)
              {
                return Pose() - *this;
              }

      /// \brief Subtraction operator
      /// A is the transform from O to P in frame O
      /// B is the transform from O to Q in frame O
      /// B - A is the transform from P to Q in frame P
      /// \param[in] _pose Pose to subtract from this one
      /// \return The resulting pose
      public: inline Pose operator-(const Pose &_pose) const
          GAZEBO_DEPRECATED(8.0)
              {
                return Pose(this->CoordPositionSub(_pose),
                            this->CoordRotationSub(_pose.rot));
              }

      /// \brief Subtraction operator
      /// \param[in] _pose Pose to subtract from this one
      /// \return The resulting pose
      public: const Pose &operator-=(const Pose &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Equality operator
      /// \param[in] _pose Pose for comparison
      /// \return True if equal
      /// Note: not explicitly deprecated on purpose, because gtest catches it
      public: bool operator ==(const Pose &_pose) const;

      /// \brief Inequality operator
      /// \param[in] _pose Pose for comparison
      /// \return True if not equal
      public: bool operator!=(const Pose &_pose) const GAZEBO_DEPRECATED(8.0);

      /// \brief Multiplication operator
      /// \param[in] _pose the other pose
      /// \return itself
      public: Pose operator*(const Pose &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Equal operator
      /// \param[in] _pose Pose to copy
      public: Pose &operator=(const Pose &_pose) GAZEBO_DEPRECATED(8.0);

      /// \brief Equal operator for ignition math
      /// \param[in] _pose Pose to copy
      public: Pose &operator=(const ignition::math::Pose3d &_pose)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Add one point to a vector: result = this + pos
      /// \param[in] _pos Position to add to this pose
      /// \return the resulting position
      public: Vector3 CoordPositionAdd(const Vector3 &_pos) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Add one point to another: result = this + pose
      /// \param[in] _pose The Pose to add
      /// \return The resulting position
      public: Vector3 CoordPositionAdd(const Pose &_pose) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Subtract one position from another: result = this - pose
      /// \param[in] _pose Pose to subtract
      /// \return The resulting position
      public: inline Vector3 CoordPositionSub(const Pose &_pose) const
          GAZEBO_DEPRECATED(8.0)
              {
                Quaternion tmp(0.0,
                    this->pos.x - _pose.pos.x,
                    this->pos.y - _pose.pos.y,
                    this->pos.z - _pose.pos.z);

                tmp = _pose.rot.GetInverse() * (tmp * _pose.rot);
                return Vector3(tmp.x, tmp.y, tmp.z);
              }

      /// \brief Add one rotation to another: result =  this->rot + rot
      /// \param[in] _rot Rotation to add
      /// \return The resulting rotation
      public: Quaternion CoordRotationAdd(const Quaternion &_rot) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Subtract one rotation from another: result = this->rot - rot
      /// \param[in] _rot The rotation to subtract
      /// \return The resulting rotation
      public: inline Quaternion CoordRotationSub(const Quaternion &_rot) const
          GAZEBO_DEPRECATED(8.0)
              {
                Quaternion result(_rot.GetInverse() * this->rot);
                result.Normalize();
                return result;
              }

      /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and
      ///        this, find a
      /// \param[in] _b the other pose
      public: Pose CoordPoseSolve(const Pose &_b) const GAZEBO_DEPRECATED(8.0);

      /// \brief Reset the pose
      public: void Reset() GAZEBO_DEPRECATED(8.0);

      /// \brief Rotate vector part of a pose about the origin
      /// \param[in] _rot rotation
      /// \return the rotated pose
      public: Pose RotatePositionAboutOrigin(const Quaternion &_rot) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision
      public: void Round(int _precision) GAZEBO_DEPRECATED(8.0);

      /// \brief Convert this pose to an ignition::math::Pose3d object.
      /// \return This pose represented as an ignition::math::Pose3d.
      public: ignition::math::Pose3d Ign() const GAZEBO_DEPRECATED(8.0);

      /// \brief Stream insertion operator
      /// \param[in] _out output stream
      /// \param[in] _pose pose to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const gazebo::math::Pose &_pose)
          GAZEBO_DEPRECATED(8.0)
              {
                _out << _pose.pos << " " << _pose.rot;
                return _out;
              }

      /// \brief Stream extraction operator
      /// \param[in] _in the input stream
      /// \param[in] _pose the pose
      /// \return the stream
      public: friend std::istream &operator>>(std::istream &_in,
                gazebo::math::Pose &_pose) GAZEBO_DEPRECATED(8.0)
            {
              // Skip white spaces
              _in.setf(std::ios_base::skipws);
              _in >> _pose.pos >> _pose.rot;
              return _in;
            }

      /// \brief The position
      public: Vector3 pos;

      /// \brief The rotation
      public: Quaternion rot;
    };
    /// \}
  }
}
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
#endif
