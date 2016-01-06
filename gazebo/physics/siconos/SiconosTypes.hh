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
  }
}
#endif  // #ifndef _SICONOSTYPES_HH
