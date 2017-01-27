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
#ifndef GAZEBO_MATH_VECTOR3_HH_
#define GAZEBO_MATH_VECTOR3_HH_

#include <math.h>
#include <iostream>
#include <fstream>
#include <ignition/math/Vector3.hh>

#include "gazebo/math/Helpers.hh"
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

    /// \class Vector3 Vector3.hh math/gzmath.hh
    /// \brief The Vector3 class represents the generic vector containing 3
    ///        elements.  Since it's commonly used to keep coordinate system
    ///        related information, its elements are labeled by x, y, z.
    class GZ_MATH_VISIBLE Vector3
    {
      /// \brief math::Vector3(0, 0, 0)
      public: static const Vector3 Zero;

      /// \brief math::Vector3(1, 1, 1)
      public: static const Vector3 One;

      /// \brief math::Vector3(1, 0, 0)
      public: static const Vector3 UnitX;

      /// \brief math::Vector3(0, 1, 0)
      public: static const Vector3 UnitY;

      /// \brief math::Vector3(0, 0, 1)
      public: static const Vector3 UnitZ;

      /// \brief Constructor
      public: Vector3() GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value along z
      public: Vector3(const double &_x, const double &_y, const double &_z)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Ignition math copy constructor
      /// \param[in] _v a vector
      public: Vector3(const ignition::math::Vector3d &_v)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor
      /// \param[in] _v a vector
      public: Vector3(const Vector3 &_v) GAZEBO_DEPRECATED(8.0);

      /// \brief Destructor
      public: virtual ~Vector3() GAZEBO_DEPRECATED(8.0);

      /// \brief Return the sum of the values
      /// \return the sum
      public: double GetSum() const GAZEBO_DEPRECATED(8.0);

      /// \brief Calc distance to the given point
      /// \param[in] _pt the point
      /// \return the distance
      public: double Distance(const Vector3 &_pt) const GAZEBO_DEPRECATED(8.0);

      /// \brief Calc distance to the given point
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value along z
      /// \return the distance
      public: double Distance(double _x, double _y, double _z) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Returns the length (magnitude) of the vector
      /// \ return the length
      public: double GetLength() const GAZEBO_DEPRECATED(8.0);

      /// \brief Return the square of the length (magnitude) of the vector
      /// \return the squared length
      public: double GetSquaredLength() const GAZEBO_DEPRECATED(8.0);

      /// \brief Normalize the vector length
      /// \return unit length vector
      public: Vector3 Normalize() GAZEBO_DEPRECATED(8.0);

      /// \brief Round to near whole number, return the result.
      /// \return the result
      public: Vector3 Round() GAZEBO_DEPRECATED(8.0);

      /// \brief Get a rounded version of this vector
      /// \return a rounded vector
      public: Vector3 GetRounded() const GAZEBO_DEPRECATED(8.0);

      /// \brief Set the contents of the vector
      /// \param[in] _x value along x
      /// \param[in] _y value along y
      /// \param[in] _z value aling z
      public: inline void Set(double _x = 0, double _y = 0 , double _z = 0)
          GAZEBO_DEPRECATED(8.0)
              {
                this->x = _x;
                this->y = _y;
                this->z = _z;
              }

      /// \brief Return the cross product of this vector and pt
      /// \return the product
      public: Vector3 Cross(const Vector3 &_pt) const GAZEBO_DEPRECATED(8.0);

      /// \brief Return the dot product of this vector and pt
      /// \return the product
      public: double Dot(const Vector3 &_pt) const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the absolute value of the vector
      /// \return a vector with positive elements
      public: Vector3 GetAbs() const GAZEBO_DEPRECATED(8.0);

      /// \brief Return a vector that is perpendicular to this one.
      /// \return an orthogonal vector
      public: Vector3 GetPerpendicular() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get a normal vector to a triangle
      /// \param[in] _v1 first vertex of the triangle
      /// \param[in] _v2 second vertex
      /// \param[in] _v3 third vertex
      /// \return the normal
      public: static Vector3 GetNormal(const Vector3 &_v1, const Vector3 &_v2,
                                       const Vector3 &_v3)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Get distance to a line
      /// \param[in] _pt1 first point on the line
      /// \param[in] _pt2 second point on the line
      /// \return the minimum distance from this point to the line
      public: double GetDistToLine(const Vector3 &_pt1, const Vector3 &_pt2)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Set this vector's components to the maximum of itself and the
      ///        passed in vector
      /// \param[in] _v the maximum clamping vector
      public: void SetToMax(const Vector3 &_v) GAZEBO_DEPRECATED(8.0);

      /// \brief Set this vector's components to the minimum of itself and the
      ///        passed in vector
      /// \param[in] _v the minimum clamping vector
      public: void SetToMin(const Vector3 &_v) GAZEBO_DEPRECATED(8.0);

      /// \brief Get the maximum value in the vector
      /// \return the maximum element
      public: double GetMax() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the minimum value in the vector
      /// \return the minimum element
      public: double GetMin() const GAZEBO_DEPRECATED(8.0);

      /// \brief Convert this vector to an ignition::math::Vector3d.
      /// \return This vector as an ignition::math::Vector3d.
      public: ignition::math::Vector3d Ign() const GAZEBO_DEPRECATED(8.0);

      /// \brief Assignment operator for ignition math
      /// \param[in] _v a new value
      /// \return this
      public: Vector3 &operator=(const ignition::math::Vector3d &_v)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Assignment operator
      /// \param[in] _v a new value
      /// \return The new vector
      public: Vector3 &operator =(const Vector3 &_v) GAZEBO_DEPRECATED(8.0);

      /// \brief Assignment operator
      /// \param[in] _value assigned to all elements
      /// \return this
      public: Vector3 &operator =(double _value) GAZEBO_DEPRECATED(8.0);

      /// \brief Addition operator
      /// \param[in] _v vector to add
      /// \return the sum vector
      public: Vector3 operator+(const Vector3 &_v) const GAZEBO_DEPRECATED(8.0);

      /// \brief Addition assignment operator
      /// \param[in] _v vector to add
      public: const Vector3 &operator+=(const Vector3 &_v)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Negation operator
      /// \return negative of this vector
      public: inline Vector3 operator-() const GAZEBO_DEPRECATED(8.0)
              {
                return Vector3(-this->x, -this->y, -this->z);
              }

      /// \brief Subtraction operators
      /// \param[in] _pt a vector to substract
      /// \return a vector
      public: inline Vector3 operator-(const Vector3 &_pt) const
          GAZEBO_DEPRECATED(8.0)
              {
                return Vector3(this->x - _pt.x,
                               this->y - _pt.y,
                               this->z - _pt.z);
              }

      /// \brief Subtraction operators
      /// \param[in] _pt subtrahend
      public: const Vector3 &operator-=(const Vector3 &_pt)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Division operator
      /// \brief[in] _pt the vector divisor
      /// \remarks this is an element wise division
      /// \return a vector
      public: const Vector3 operator/(const Vector3 &_pt) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Division assignment operator
      /// \brief[in] _pt the vector divisor
      /// \remarks this is an element wise division
      /// \return a vector
      public: const Vector3 &operator/=(const Vector3 &_pt)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \return a vector
      public: const Vector3 operator/(double _v) const GAZEBO_DEPRECATED(8.0);

      /// \brief Division operator
      /// \remarks this is an element wise division
      /// \return this
      public: const Vector3 &operator/=(double _v) GAZEBO_DEPRECATED(8.0);

      /// \brief Multiplication operator
      /// \remarks this is an element wise multiplication, not a cross product
      /// \param[in] _v
      public: Vector3 operator*(const Vector3 &_p) const GAZEBO_DEPRECATED(8.0);

      /// \brief Multiplication operators
      /// \remarks this is an element wise multiplication, not a cross product
      /// \param[in] _v a vector
      /// \return this
      public: const Vector3 &operator*=(const Vector3 &_v)
          GAZEBO_DEPRECATED(8.0);

      /// \brief Multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _v input vector
      /// \return a scaled vector
      public: friend inline Vector3 operator*(double _s,
                                              const Vector3 &_v)
          GAZEBO_DEPRECATED(8.0)
      { return Vector3(_v.x * _s, _v.y * _s, _v.z * _s); }

      /// \brief Multiplication operators
      /// \param[in] _v the scaling factor
      /// \return a scaled vector
      public: Vector3 operator*(double _v) const GAZEBO_DEPRECATED(8.0);

      /// \brief Multiplication operator
      /// \param[in] _v scaling factor
      /// \return this
      public: const Vector3 &operator*=(double _v) GAZEBO_DEPRECATED(8.0);

      /// \brief Equal to operator
      /// \param[in] _pt The vector to compare against
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
      /// Note: not explicitly deprecated on purpose, because gtest catches it
      public: bool operator ==(const Vector3 &_pt) const;

      /// \brief Not equal to operator
      /// \param[in] _v The vector to compare against
      /// \return true if each component is equal withing a
      /// default tolerence (1e-6), false otherwise
      /// Note: not explicitly deprecated on purpose, because gtest catches it
      public: bool operator!=(const Vector3 &_v) const;

      /// \brief See if a point is finite (e.g., not nan)
      public: bool IsFinite() const GAZEBO_DEPRECATED(8.0);

      /// \brief Corrects any nan values
      public: inline void Correct() GAZEBO_DEPRECATED(8.0)
              {
                if (!std::isfinite(this->x))
                  this->x = 0;
                if (!std::isfinite(this->y))
                  this->y = 0;
                if (!std::isfinite(this->z))
                  this->z = 0;
              }

      /// \brief [] operator
      public: double operator[](unsigned int index) const
          GAZEBO_DEPRECATED(8.0);

      /// \brief Round all values to _precision decimal places
      /// \param[in] _precision the decimal places
      public: void Round(int _precision) GAZEBO_DEPRECATED(8.0);

      /// \brief Equality test
      /// \remarks This is equivalent to the == operator
      /// \param[in] _v the other vector
      /// \return true if the 2 vectors have the same values, false otherwise
      public: bool Equal(const Vector3 &_v) const GAZEBO_DEPRECATED(8.0);

      /// \brief X location
      public: double x;

      /// \brief Y location
      public: double y;

      /// \brief Z location
      public: double z;

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _pt Vector3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const gazebo::math::Vector3 &_pt)
          GAZEBO_DEPRECATED(8.0)
      {
        _out << precision(_pt.x, 6) << " " << precision(_pt.y, 6) << " "
             << precision(_pt.z, 6);
        return _out;
      }

      /// \brief Stream extraction operator
      /// \param _in input stream
      /// \param _pt vector3 to read values into
      /// \return the stream
      public: friend std::istream &operator>>(std::istream &_in,
                                              gazebo::math::Vector3 &_pt)
          GAZEBO_DEPRECATED(8.0)
      {
        // Skip white spaces
        _in.setf(std::ios_base::skipws);
        _in >> _pt.x >> _pt.y >> _pt.z;
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
