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
#ifndef _GAZEBO_MATRIX4_HH_
#define _GAZEBO_MATRIX4_HH_

#include <assert.h>
#include <iostream>
#include <ignition/math/Matrix4.hh>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    class Quaternion;
    class Pose;

    /// \addtogroup gazebo_math
    /// \{

    /// \class Matrix4 Matrix4.hh math/gzmath.hh
    /// \brief A 3x3 matrix class
    class GZ_MATH_VISIBLE Matrix4
    {
      /// \brief Constructor
      /// \deprecated See ignition::math::Matrix4
      public: Matrix4()
              GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor
      /// \param _m Matrix to copy
      /// \deprecated See ignition::math::Matrix4
      public: Matrix4(const Matrix4 &_m)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Copy constructor for ignition math
      /// \param _m Matrix to copy
      /// \deprecated See ignition::math::Matrix4
      public: Matrix4(const ignition::math::Matrix4d &_m)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Constructor
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v03 Row 0, Col 3 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v13 Row 1, Col 3 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      /// \param[in] _v23 Row 2, Col 3 value
      /// \param[in] _v30 Row 3, Col 0 value
      /// \param[in] _v31 Row 3, Col 1 value
      /// \param[in] _v32 Row 3, Col 2 value
      /// \param[in] _v33 Row 3, Col 3 value
      /// \deprecated See ignition::math::Matrix4
      public: Matrix4(double _v00, double _v01, double _v02, double _v03,
                      double _v10, double _v11, double _v12, double _v13,
                      double _v20, double _v21, double _v22, double _v23,
                      double _v30, double _v31, double _v32, double _v33)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Destructor
      public: virtual ~Matrix4();

      /// \brief Change the values
      /// \param[in] _v00 Row 0, Col 0 value
      /// \param[in] _v01 Row 0, Col 1 value
      /// \param[in] _v02 Row 0, Col 2 value
      /// \param[in] _v03 Row 0, Col 3 value
      /// \param[in] _v10 Row 1, Col 0 value
      /// \param[in] _v11 Row 1, Col 1 value
      /// \param[in] _v12 Row 1, Col 2 value
      /// \param[in] _v13 Row 1, Col 3 value
      /// \param[in] _v20 Row 2, Col 0 value
      /// \param[in] _v21 Row 2, Col 1 value
      /// \param[in] _v22 Row 2, Col 2 value
      /// \param[in] _v23 Row 2, Col 3 value
      /// \param[in] _v30 Row 3, Col 0 value
      /// \param[in] _v31 Row 3, Col 1 value
      /// \param[in] _v32 Row 3, Col 2 value
      /// \param[in] _v33 Row 3, Col 3 value
      public: void Set(double _v00, double _v01, double _v02, double _v03,
                       double _v10, double _v11, double _v12, double _v13,
                       double _v20, double _v21, double _v22, double _v23,
                       double _v30, double _v31, double _v32, double _v33);



      /// \brief Set the translational values [ (0, 3) (1, 3) (2, 3) ]
      /// \param[in] _t Values to set
      public: void SetTranslate(const Vector3 &_t);

      /// \brief Get the translational values as a Vector3
      /// \return x,y,z
      public: Vector3 GetTranslation() const;

      /// \brief Get the rotation as a quaternion
      /// \return the rotation
      public: Quaternion GetRotation() const;

      /// \brief Get the rotation as a Euler angles
      /// \return the rotation
      public: Vector3 GetEulerRotation(unsigned int solution_number = 1) const;

      /// \brief Get the transformation as math::Pose
      /// \return the pose
      public: math::Pose GetAsPose() const;

      /// \brief Set the scale
      /// \param[in] _s scale
      public: void SetScale(const Vector3 &_s);

      /// \brief Return true if the matrix is affine
      /// \return true if the matrix is affine, false otherwise
      public: bool IsAffine() const;

      /// \brief Perform an affine transformation
      /// \param _v Vector3 value for the transformation
      /// \return The result of the transformation
      public: Vector3 TransformAffine(const Vector3 &_v) const;

      /// \brief Return the inverse matrix
      /// \return Inverse of this matrix.
      public: Matrix4 Inverse() const;

      /// \brief Equal operator. this = _mat
      /// \param _mat Incoming matrix
      /// \return itself
      public: Matrix4 &operator =(const Matrix4 &_mat);

      /// \brief Equal operator for ignition math
      /// \param _mat Incoming matrix
      /// \return itself
      public: Matrix4 &operator=(const ignition::math::Matrix4d &_mat);

      /// \brief Equal operator for 3x3 matrix
      /// \param _mat Incoming matrix
      /// \return itself
      public: const Matrix4 & operator =(const Matrix3 &_mat);

      /// \brief Multiplication operator
      /// \param _mat Incoming matrix
      /// \return This matrix * _mat
      public: Matrix4 operator*(const Matrix4 &_mat) const;

      /// \brief Multiplication operator
      /// \param _mat Incoming matrix
      /// \return This matrix * _mat
      public: Matrix4 operator*(const Matrix3 &_mat) const;


      /// \brief Multiplication operator
      /// \param _vec Vector3
      /// \return Resulting vector from multiplication
      public: Vector3 operator*(const Vector3 &_vec) const;

      /// \brief Array subscript operator
      /// \param[in] _row the row index
      /// \return the row
      public: inline double *operator[](size_t _row)
              {
                assert(_row < 4);
                return this->m[_row];
              }
      /// \param[in] _row the row index
      /// \return the row
       public: inline const double *operator[](size_t _row) const
              {
                assert(_row < 4);
                return this->m[_row];
              }

      /// \brief Equality operator
      /// \param[in] _m Matrix3 to test
      /// \return true if the 2 matrices are equal (using the tolerance 1e-6),
      ///  false otherwise
      public: bool operator==(const Matrix4 &_m) const;

      /// \brief Convert this matrix to an ignition::math::Matrix4d.
      /// \return This matrix as an ignition::math::Matrix4d.
      public: ignition::math::Matrix4d Ign() const;

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _m Matrix to output
      /// \return the stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                               const gazebo::math::Matrix4 &_m)
            {
              for (int i = 0; i < 4; i++)
              {
                for (int j = 0; j < 4; j++)
                {
                  _out << (fabs(_m.m[i][j]) < 1e-6 ? 0 : _m.m[i][j]) << " ";
                }
                _out << "\n";
              }

              return _out;
            }

      /// \brief Identity matrix
      public: static const Matrix4 IDENTITY;

      /// \brief Zero matrix
      public: static const Matrix4 ZERO;

      /// \brief The 4x4 matrix
      protected: double m[4][4];
    };
    /// \}
  }
}
#endif



