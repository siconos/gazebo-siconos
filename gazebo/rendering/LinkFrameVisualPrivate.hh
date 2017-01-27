/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GAZEBO_LINK_FRAME_VISUAL_PRIVATE_HH_
#define GAZEBO_LINK_FRAME_VISUAL_PRIVATE_HH_

#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/AxisVisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the LinkFrame Visual class.
    class LinkFrameVisualPrivate : public AxisVisualPrivate
    {
      /// \brief Scale based on the size of the parent link.
      public: ignition::math::Vector3d scaleToLink;

      /// \brief Transparency when highlighted.
      public: float highlightedTransp;

      /// \brief Transparency when not highlighted.
      public: float nonHighlightedTransp;
    };
  }
}
#endif
