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
#ifndef GAZEBO_MATH_MATRIX3_HH_
#define GAZEBO_MATH_MATRIX3_HH_

#include <assert.h>
#include <ignition/math/Matrix3.hh>

#include "gazebo/math/Vector3.hh"
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

    /// \class Matrix3 Matrix3hh math/gzmath.hh
    /// \brief A 3x3 matrix class
    class GZ_MATH_VISIBLE Matrix3
    {
      /// \brief Constructor
      public: Matrix3() GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      public: Matrix3(const Matrix3 &_m) GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      public: Matrix3(double _v00, double _v01, double _v02,
                      double _v10, double _v11, double _v12,
                      double _v20, double _v21, double _v22)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor for ignition math
      /// \param _m Matrix to copy
      public: Matrix3(const ignition::math::Matrix3d &_m)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Desctructor
      public: virtual ~Matrix3();

      /// \brief Convert this matrix to an ignition::math::Matrix3d.
      /// \return This matrix as an ignition::math::Matrix3d.
      public: ignition::math::Matrix3d Ign() const;

      /// \brief Set the matrix from three axis (1 per column)
      /// \param[in] _xAxis The x axis
      /// \param[in] _yAxis The y axis
      /// \param[in] _zAxis The z axis
      public: void SetFromAxes(const Vector3 &_xAxis,
                               const Vector3 &_yAxis,
                               const Vector3 &_zAxis);


      /// \brief Set the matrix from an axis and angle
      /// \param[in] _axis the axis
      /// \param[in] _angle ccw rotation around the axis in radians
      public: void SetFromAxis(const Vector3 &_axis, double _angle);

      /// \brief Set a column
      /// \param[in] _c The colum index (0, 1, 2)
      /// \param[in] _v The value to set in each row of the column
      public: void SetCol(unsigned int _c, const Vector3 &_v);

      /// \brief Return the inverse matrix
      /// \return Inverse of this matrix.
      public: Matrix3 Inverse() const;

      /// \brief returns the element wise difference of two matrices
      public: Matrix3 operator-(const Matrix3 &_m) const
      {
        return Matrix3(
        // first row
        this->m[0][0]-_m[0][0], this->m[0][1]-_m[0][1], this->m[0][2]-_m[0][2],
        this->m[1][0]-_m[1][0], this->m[1][1]-_m[1][1], this->m[1][2]-_m[1][2],
        this->m[2][0]-_m[2][0], this->m[2][1]-_m[2][1], this->m[2][2]-_m[2][2]);
      }

      /// \brief returns the element wise sum of two matrices
      public: Matrix3 operator+(const Matrix3 &_m) const
      {
        return Matrix3(
        // first row
        this->m[0][0]+_m[0][0], this->m[0][1]+_m[0][1], this->m[0][2]+_m[0][2],
        this->m[1][0]+_m[1][0], this->m[1][1]+_m[1][1], this->m[1][2]+_m[1][2],
        this->m[2][0]+_m[2][0], this->m[2][1]+_m[2][1], this->m[2][2]+_m[2][2]);
      }

      /// \brief returns the element wise scalar multiplication
      public: Matrix3 operator*(const double &_s) const
      {
        return Matrix3(
          // first row
          _s * this->m[0][0], _s * this->m[0][1], _s * this->m[0][2],
          _s * this->m[1][0], _s * this->m[1][1], _s * this->m[1][2],
          _s * this->m[2][0], _s * this->m[2][1], _s * this->m[2][2]);
      }

      /// \brief Multiplication operators
      /// \param[in] _s the scaling factor
      /// \param[in] _m input matrix
      /// \return a scaled matrix
      public: friend inline Matrix3 operator*(double _s,
                                              const Matrix3 &_m)
      { return _m * _s; }

      /// \brief Matrix multiplication operator
      /// \param[in] _m Matrix3 to multiply
      /// \return product of this * _m
      public: Matrix3 operator*(const Matrix3 &_m) const
      {
        return Matrix3(
          // first row
          this->m[0][0]*_m[0][0]+this->m[0][1]*_m[1][0]+this->m[0][2]*_m[2][0],
          this->m[0][0]*_m[0][1]+this->m[0][1]*_m[1][1]+this->m[0][2]*_m[2][1],
          this->m[0][0]*_m[0][2]+this->m[0][1]*_m[1][2]+this->m[0][2]*_m[2][2],
          // second row
          this->m[1][0]*_m[0][0]+this->m[1][1]*_m[1][0]+this->m[1][2]*_m[2][0],
          this->m[1][0]*_m[0][1]+this->m[1][1]*_m[1][1]+this->m[1][2]*_m[2][1],
          this->m[1][0]*_m[0][2]+this->m[1][1]*_m[1][2]+this->m[1][2]*_m[2][2],
          // third row
          this->m[2][0]*_m[0][0]+this->m[2][1]*_m[1][0]+this->m[2][2]*_m[2][0],
          this->m[2][0]*_m[0][1]+this->m[2][1]*_m[1][1]+this->m[2][2]*_m[2][1],
          this->m[2][0]*_m[0][2]+this->m[2][1]*_m[1][2]+this->m[2][2]*_m[2][2]);
      }

      /// \brief Equality test operator
      /// \param[in] _m Matrix3 to test
      /// \return True if equal (using the default tolerance of 1e-6)
      public: bool operator==(const Matrix3 &_m) const;

      /// \brief Matrix times Vector3 operator
      /// \param[in] _v a Vector3
      /// \return this * _v
      public: inline math::Vector3 operator*(const math::Vector3 &_v) const
              {
                return math::Vector3(
                  this->m[0][0]*_v.x + this->m[0][1]*_v.y + this->m[0][2]*_v.z,
                  this->m[1][0]*_v.x + this->m[1][1]*_v.y + this->m[1][2]*_v.z,
                  this->m[2][0]*_v.x + this->m[2][1]*_v.y + this->m[2][2]*_v.z);
              }

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline const double *operator[](size_t _row) const
              {
                assert(_row < 3);
                return this->m[_row];
              }

      /// \brief Array subscript operator
      /// \param[in] _row row index
      /// \return a pointer to the row
      public: inline double *operator[](size_t _row)
              {
                assert(_row < 3);
                return this->m[_row];
              }


      /// \brief Stream insertion operator
      /// \param[in] _out Output stream
      /// \param[in] _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const gazebo::math::Matrix3 &_m)
            {
              for (int i = 0; i < 3; i++)
              {
                for (int j = 0; j < 3; j++)
                {
                  _out << _m.m[i][j] << " ";
                }
                _out << "\n";
              }

              return _out;
            }

      /// \brief Identity matrix
      public: static const Matrix3 IDENTITY;

      /// \brief Zero matrix
      public: static const Matrix3 ZERO;

      /// \brief the 3x3 matrix
      protected: double m[3][3];

      friend class Matrix4;
    };
    /// \}
  }
}
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
#endif



