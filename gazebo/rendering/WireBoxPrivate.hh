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
#ifndef GAZEBO_RENDERING_WIREBOX_PRIVATE_HH_
#define GAZEBO_RENDERING_WIREBOX_PRIVATE_HH_

#include <ignition/math/Box.hh>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/DynamicLines.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the WireBox class
    class WireBoxPrivate
    {
      /// \brief Copy of bounding box.
      public: ignition::math::Box box;

      /// \brief The lines which outline the box.
      public: DynamicLines *lines;

      /// \brief The visual which this box is attached to.
      public: VisualPtr parent;
    };
  }
}
#endif
