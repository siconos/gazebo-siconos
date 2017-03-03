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

#ifndef _SICONOSTYPES_HH
#define _SICONOSTYPES_HH

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <SiconosVector.hpp>

/// \file
/// \ingroup gazebo_physics
/// \ingroup gazebo_physics_siconos
/// \brief Siconos wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    class SiconosCollision;
    class SiconosLink;
    class SiconosPhysics;
    class SiconosSurfaceParams;

    typedef boost::shared_ptr<SiconosCollision> SiconosCollisionPtr;
    typedef boost::shared_ptr<SiconosLink> SiconosLinkPtr;
    typedef boost::shared_ptr<SiconosPhysics> SiconosPhysicsPtr;
    typedef boost::shared_ptr<SiconosSurfaceParams> SiconosSurfaceParamsPtr;

    class GZ_PHYSICS_VISIBLE SiconosTypes {
      /// \brief Convert a SiconosVector to a gazebo Vector3.
      /// \param[in] _bt SiconosVector of size 3.
      /// \return Gazebo Vector3.
      public: static ignition::math::Vector3d ConvertVector3(SP::SiconosVector _v)
              {
                assert(_v->size() >= 3);
                return ignition::math::Vector3d((*_v)(0), (*_v)(1), (*_v)(2));
              }

      /// \brief Convert a gazebo Vector3 to a SiconosVector.
      /// \param[in] _vec Gazebo Vector3.
      /// \return SiconosVector of size 3.
      public: static SP::SiconosVector ConvertVector3(
        const ignition::math::Vector3d &_vec)
              {
                auto v = std11::make_shared<SiconosVector>(3);
                (*v)(0) = _vec.X();
                (*v)(1) = _vec.Y();
                (*v)(2) = _vec.Z();
                return v;
              }

      /// \brief Convert a gazebo Vector3 to an existing SiconosVector with optional offset.
      /// \param[in] _vec Gazebo Vector3.
      /// \return SiconosVector of size > 3.
      public: static void ConvertVector3(
        const ignition::math::Vector3d &_vec,
        SP::SiconosVector &v, unsigned int offset=0)
              {
                (*v)(offset+0) = _vec.X();
                (*v)(offset+1) = _vec.Y();
                (*v)(offset+2) = _vec.Z();
              }

      /// \brief Convert a gazebo Vector3 to an existing SiconosVector with optional offset.
      /// \param[in] _vec Gazebo Vector3.
      /// \return SiconosVector of size > 3.
      public: static void ConvertVector3(
        const ignition::math::Vector3d &_vec,
        SiconosVector &v, unsigned int offset=0)
              {
                v(offset+0) = _vec.X();
                v(offset+1) = _vec.Y();
                v(offset+2) = _vec.Z();
              }

      /// \brief Convert a SiconosVector(7) to a gazebo pose.
      /// \param[in] _v SiconosVector of size 7.
      /// \return Gazebo pose.
      public: static ignition::math::Pose3d ConvertPose(SP::SiconosVector _v)
              {
                ignition::math::Pose3d pose;
                pose.Pos() = ConvertVector3(_v);
                pose.Rot().W() = (*_v)(3);
                pose.Rot().X() = (*_v)(4);
                pose.Rot().Y() = (*_v)(5);
                pose.Rot().Z() = (*_v)(6);
                return pose;
              }

      /// \brief Convert a gazebo pose to a new SiconosVector(7)
      /// \param[in] _pose Gazebo pose.
      /// \return SP::SiconosVector pose (size 7).
      public: static SP::SiconosVector ConvertPose(const ignition::math::Pose3d &_pose)
              {
                auto v = std11::make_shared<SiconosVector>(7);
                (*v)(0) = _pose.Pos().X();
                (*v)(1) = _pose.Pos().Y();
                (*v)(2) = _pose.Pos().Z();
                (*v)(3) = _pose.Rot().W();
                (*v)(4) = _pose.Rot().X();
                (*v)(5) = _pose.Rot().Y();
                (*v)(6) = _pose.Rot().Z();
                return v;
              }

      /// \brief Convert a gazebo pose to an existing SiconosVector(7)
      /// \param[in] _pose Gazebo pose.
      /// \return SP::SiconosVector pose (size 7).
      public: static SP::SiconosVector ConvertPoseToVector7(
        const ignition::math::Pose3d &_pose,
        SP::SiconosVector _v)
              {
                (*_v)(0) = _pose.Pos().X();
                (*_v)(1) = _pose.Pos().Y();
                (*_v)(2) = _pose.Pos().Z();
                (*_v)(3) = _pose.Rot().W();
                (*_v)(4) = _pose.Rot().X();
                (*_v)(5) = _pose.Rot().Y();
                (*_v)(6) = _pose.Rot().Z();
                return _v;
              }
    };
  }
}
#endif  // #ifndef _SICONOSTYPES_HH
