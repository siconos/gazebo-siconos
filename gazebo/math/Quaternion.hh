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

#ifndef _GAZEBO_MATH_QUATERNION_HH_
#define _GAZEBO_MATH_QUATERNION_HH_

#include <math.h>
#include <iostream>
#include <cmath>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Quaternion.hh>

#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/math/Matrix4.hh"
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

  /// \class Quaternion Quaternion.hh math/gzmath.hh
  /// \brief A quaternion class
  class GZ_MATH_VISIBLE Quaternion
  {
    /// \brief Default Constructor
    public: Quaternion();

    /// \brief Constructor
    /// \param[in] _w W param
    /// \param[in] _x X param
    /// \param[in] _y Y param
    /// \param[in] _z Z param
    public: Quaternion(const double &_w, const double &_x, const double &_y,
                        const double &_z) GAZEBO_DEPRECATED(8.0);

    /// \brief Constructor from Euler angles in radians
    /// \param[in] _roll  roll
    /// \param[in] _pitch pitch
    /// \param[in] _yaw   yaw
    public: Quaternion(const double &_roll, const double &_pitch,
                        const double &_yaw) GAZEBO_DEPRECATED(8.0);

    /// \brief Constructor from axis angle
    /// \param[in] _axis the rotation axis
    /// \param[in] _angle the rotation angle in radians
    public: Quaternion(const Vector3 &_axis, const double &_angle)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Constructor
    /// \param[in] _rpy euler angles
    public: Quaternion(const Vector3 &_rpy) GAZEBO_DEPRECATED(8.0);

    /// \brief Copy constructor
    /// \param[in] _qt Quaternion to copy
    public: Quaternion(const Quaternion &_qt) GAZEBO_DEPRECATED(8.0);

    /// \brief Copy constructor for ignition::math::Quaterniond
    /// \param[in] _qt Ignition math quaterniond to copy
    public: Quaternion(const ignition::math::Quaterniond &_qt)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Destructor
    public: ~Quaternion() GAZEBO_DEPRECATED(8.0);

    /// \brief Equal operator
    /// \param[in] _qt Quaternion to copy
    public: Quaternion &operator =(const Quaternion &_qt)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Convert this quaternion to an ignition::math::Quaterniond.
    /// \return This quaternion as an ignition::math::Quaterniond.
    public: ignition::math::Quaterniond Ign() const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Assignment operator for ignition math
    /// \param[in] _v a new value
    /// \return The new quaternion.
    public: Quaternion &operator =(const ignition::math::Quaterniond &_v)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Invert the quaternion
    public: void Invert() GAZEBO_DEPRECATED(8.0);

    /// \brief Get the inverse of this quaternion
    /// \return Inverse quaternion
    public: inline Quaternion GetInverse() const GAZEBO_DEPRECATED(8.0)
            {
              double s = 0;
              Quaternion q(this->w, this->x, this->y, this->z);

              // use s to test if quaternion is valid
              s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

              if (ignition::math::equal(s, 0.0))
              {
                q.w = 1.0;
                q.x = 0.0;
                q.y = 0.0;
                q.z = 0.0;
              }
              else
              {
                // deal with non-normalized quaternion
                // div by s so q * qinv = identity
                q.w =  q.w / s;
                q.x = -q.x / s;
                q.y = -q.y / s;
                q.z = -q.z / s;
              }
              return q;
            }

    /// \brief Set the quaternion to the identity
    public: void SetToIdentity() GAZEBO_DEPRECATED(8.0);

    /// \brief Return the logarithm
    /// \return the log
    public: Quaternion GetLog() const GAZEBO_DEPRECATED(8.0);

    /// \brief Return the exponent
    /// \return the exp
    public: Quaternion GetExp() const GAZEBO_DEPRECATED(8.0);

    /// \brief Normalize the quaternion
    public: void Normalize() GAZEBO_DEPRECATED(8.0);

    /// \brief Set the quaternion from an axis and angle
    /// \param[in] _x X axis
    /// \param[in] _y Y axis
    /// \param[in] _z Z axis
    /// \param[in] _a Angle in radians
    public: void SetFromAxis(double _x, double _y, double _z, double _a)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Set the quaternion from an axis and angle
    /// \param[in] _axis Axis
    /// \param[in] _a Angle in radians
    public: void SetFromAxis(const Vector3 &_axis, double _a)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Set this quaternion from 4 floating numbers
    /// \param[in] _u u
    /// \param[in] _x x
    /// \param[in] _y y
    /// \param[in] _z z
    public: void Set(double _u, double _x, double _y, double _z)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Set the quaternion from Euler angles. The order of operations
    /// is roll, pitch, yaw around a fixed body frame axis
    /// (the original frame of the object before rotation is applied).
    /// Roll is a rotation about x, pitch is about y, yaw is about z.
    /// \param[in] _vec Euler angle
    public: void SetFromEuler(const Vector3 &_vec) GAZEBO_DEPRECATED(8.0);

    /// \brief Set the quaternion from Euler angles.
    /// \param[in] _roll Roll angle (radians).
    /// \param[in] _pitch Pitch angle (radians).
    /// \param[in] _yaw Yaw angle (radians).
    public: void SetFromEuler(double _roll, double _pitch, double _yaw)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Return the rotation in Euler angles
    /// \return This quaternion as an Euler vector
    public: Vector3 GetAsEuler() const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Convert euler angles to a quaternion.
    /// \param[in] _vec The vector of angles to convert.
    /// \return The converted quaternion.
    public: static Quaternion EulerToQuaternion(const Vector3 &_vec)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Convert euler angles to quatern.
    /// \param[in] _x rotation along x
    /// \param[in] _y rotation along y
    /// \param[in] _z rotation along z
    /// \return The converted quaternion.
    public: static Quaternion EulerToQuaternion(double _x,
                                                double _y,
                                                double _z)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Get the Euler roll angle in radians
    /// \return the roll component
    public: double GetRoll() GAZEBO_DEPRECATED(8.0);

    /// \brief Get the Euler pitch angle in radians
    /// \return the pitch component
    public: double GetPitch() GAZEBO_DEPRECATED(8.0);

    /// \brief Get the Euler yaw angle in radians
    /// \return the yaw component
    public: double GetYaw() GAZEBO_DEPRECATED(8.0);

    /// \brief Return rotation as axis and angle
    /// \param[in] _axis rotation axis
    /// \param[in] _angle ccw angle in radians
    public: void GetAsAxis(Vector3 &_axis, double &_angle) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Scale a Quaternionion
    /// \param[in] _scale Amount to scale this rotation
    public: void Scale(double _scale) GAZEBO_DEPRECATED(8.0);

    /// \brief Addition operator
    /// \param[in] _qt quaternion for addition
    /// \return this quaternion + _qt
    public: Quaternion operator+(const Quaternion &_qt) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Addition operator
    /// \param[in] _qt quaternion for addition
    /// \return this quaternion + qt
    public: Quaternion operator+=(const Quaternion &_qt)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Subtraction operator
    /// \param[in] _qt quaternion to subtract
    /// \return this quaternion - _qt
    public: Quaternion operator-(const Quaternion &_qt) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Subtraction operator
    /// \param[in] _qt Quaternion for subtraction
    /// \return This quaternion - qt
    public: Quaternion operator-=(const Quaternion &_qt)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Multiplication operator
    /// \param[in] _q Quaternion for multiplication
    /// \return This quaternion multiplied by the parameter
    public: inline Quaternion operator*(const Quaternion &_q) const
        GAZEBO_DEPRECATED(8.0)
            {
              return Quaternion(
                  this->w*_q.w - this->x*_q.x - this->y*_q.y - this->z*_q.z,
                  this->w*_q.x + this->x*_q.w + this->y*_q.z - this->z*_q.y,
                  this->w*_q.y - this->x*_q.z + this->y*_q.w + this->z*_q.x,
                  this->w*_q.z + this->x*_q.y - this->y*_q.x + this->z*_q.w);
            }

    /// \brief Multiplication operator by a scalar.
    /// \param[in] _f factor
    /// \return quaternion multiplied by the scalar
    public: Quaternion operator*(const double &_f) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Multiplication operator
    /// \param[in] _qt Quaternion for multiplication
    /// \return This quaternion multiplied by the parameter
    public: Quaternion operator*=(const Quaternion &qt)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Vector3 multiplication operator
    /// \param[in] _v vector to multiply
    /// \return The result of the vector multiplication
    public: Vector3 operator*(const Vector3 &_v) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Equal to operator
    /// \param[in] _qt Quaternion for comparison
    /// \return True if equal
    /// Note: not explicitly deprecated on purpose, because gtest catches it
    public: bool operator ==(const Quaternion &_qt) const;

    /// \brief Not equal to operator
    /// \param[in] _qt Quaternion for comparison
    /// \return True if not equal
    /// Note: not explicitly deprecated on purpose, because gtest catches it
    public: bool operator!=(const Quaternion &_qt) const;

    /// \brief Unary minus operator
    /// \return negates each component of the quaternion
    public: Quaternion operator-() const GAZEBO_DEPRECATED(8.0);

    /// \brief Rotate a vector using the quaternion
    /// \param[in] _vec vector to rotate
    /// \return the rotated vector
    public: inline Vector3 RotateVector(const Vector3 &_vec) const
        GAZEBO_DEPRECATED(8.0)
            {
              Quaternion tmp(0.0, _vec.x, _vec.y, _vec.z);
              tmp = (*this) * (tmp * this->GetInverse());
              return Vector3(tmp.x, tmp.y, tmp.z);
            }

    /// \brief Do the reverse rotation of a vector by this quaternion
    /// \param[in] _vec the vector
    /// \return the reversed vector
    public: Vector3 RotateVectorReverse(Vector3 _vec) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief See if a quaternion is finite (e.g., not nan)
    /// \return True if quaternion is finite
    public: bool IsFinite() const GAZEBO_DEPRECATED(8.0);

    /// \brief Correct any nan values in this quaternion
    public: inline void Correct() GAZEBO_DEPRECATED(8.0)
            {
              if (!std::isfinite(this->x))
                this->x = 0;
              if (!std::isfinite(this->y))
                this->y = 0;
              if (!std::isfinite(this->z))
                this->z = 0;
              if (!std::isfinite(this->w))
                this->w = 1;

              if (ignition::math::equal(this->w, 0.0) &&
                  ignition::math::equal(this->x, 0.0) &&
                  ignition::math::equal(this->y, 0.0) &&
                  ignition::math::equal(this->z, 0.0))
              {
                this->w = 1;
              }
            }

    /// \brief Get the quaternion as a 3x3 matrix
    /// \return The 3x3 matrix form of the quaternion
    public: Matrix3 GetAsMatrix3() const GAZEBO_DEPRECATED(8.0);

    /// \brief Get the quaternion as a 4x4 matrix
    /// \return a 4x4 matrix
    public: Matrix4 GetAsMatrix4() const GAZEBO_DEPRECATED(8.0);

    /// \brief Return the X axis
    /// \return the X axis of the vector
    public: Vector3 GetXAxis() const GAZEBO_DEPRECATED(8.0);

    /// \brief Return the Y axis
    /// \return the Y axis of the vector
    public: Vector3 GetYAxis() const GAZEBO_DEPRECATED(8.0);

    /// \brief Return the Z axis
    /// \return the Z axis of the vector
    public: Vector3 GetZAxis() const GAZEBO_DEPRECATED(8.0);

    /// \brief Round all values to _precision decimal places
    /// \param[in] _precision the precision
    public: void Round(int _precision) GAZEBO_DEPRECATED(8.0);

    /// \brief Dot product
    /// \param[in] _q the other quaternion
    /// \return the product
    public: double Dot(const Quaternion &_q) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief Spherical quadratic interpolation
    /// given the ends and an interpolation parameter between 0 and 1
    /// \param[in] _ft the interpolation parameter
    /// \param[in] _rkP the beginning quaternion
    /// \param[in] _rkA first intermediate quaternion
    /// \param[in] _rkB second intermediate quaternion
    /// \param[in] _rkQ the end quaternion
    /// \param[in] _shortestPath when true, the rotation may be inverted to
    /// get to minimize rotation
    /// \return The result of the quadratic interpolation
    public: static Quaternion Squad(double _fT, const Quaternion &_rkP,
                const Quaternion &_rkA, const Quaternion &_rkB,
                const Quaternion &_rkQ, bool _shortestPath = false)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Spherical linear interpolation between 2 quaternions,
    /// given the ends and an interpolation parameter between 0 and 1
    /// \param[in] _ft the interpolation parameter
    /// \param[in] _rkP the beginning quaternion
    /// \param[in] _rkQ the end quaternion
    /// \param[in] _shortestPath when true, the rotation may be inverted to
    /// get to minimize rotation
    /// \return The result of the linear interpolation
    public: static Quaternion Slerp(double _fT, const Quaternion &_rkP,
                const Quaternion &_rkQ, bool _shortestPath = false)
        GAZEBO_DEPRECATED(8.0);

    /// \brief Integrate quaternion for constant angular velocity vector
    /// along specified interval `_deltaT`.
    /// \param[in] _angularVelocity Angular velocity vector, specified in
    /// same reference frame as base of this quaternion.
    /// \param[in] _deltaT Time interval in seconds to integrate over.
    /// \return Quaternion at integrated configuration.
    public: Quaternion Integrate(const Vector3 &_angularVelocity,
                                 const double _deltaT) const
        GAZEBO_DEPRECATED(8.0);

    /// \brief w value of the quaternion
    public: double w;

    /// \brief x value of the quaternion
    public: double x;

    /// \brief y value of the quaternion
    public: double y;

    /// \brief z value of the quaternion
    public: double z;

    /// \brief Stream insertion operator
    /// \param[in] _out output stream
    /// \param[in] _q quaternion to output
    /// \return the stream
    public: friend  std::ostream &operator<<(std::ostream &_out,
                const gazebo::math::Quaternion &_q)
        GAZEBO_DEPRECATED(8.0)
    {
      Vector3 v(_q.GetAsEuler());
      _out << precision(v.x, 6) << " " << precision(v.y, 6) << " "
           << precision(v.z, 6);
      return _out;
    }

    /// \brief Stream extraction operator
    /// \param[in] _in input stream
    /// \param[in] _q Quaternion to read values into
    /// \return The istream
    public: friend std::istream &operator>>(std::istream &_in,
                                             gazebo::math::Quaternion &_q)
        GAZEBO_DEPRECATED(8.0)
    {
      double roll, pitch, yaw;

      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> roll >> pitch >> yaw;

      _q.SetFromEuler(Vector3(roll, pitch, yaw));

      return _in;
    }
  };
  /// \}
  }
}
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
#endif
