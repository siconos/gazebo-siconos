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

#ifndef _GAZEBO_GUI_BUILDING_SEGMENTITEM_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_SEGMENTITEM_PRIVATE_HH_
#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the SegmentItem class
    class SegmentItemPrivate
    {
      /// \brief Segment's start position in pixel coordinates.
      public: ignition::math::Vector2d start;

      /// \brief Segment's end position in pixel coordinates.
      public: ignition::math::Vector2d end;

      /// \brief Keep track of mouse press position for translation.
      public: ignition::math::Vector2d segmentMouseMove;

      /// \brief Thickness of the segment on the 2d view, in pixels.
      public: double thickness;

      /// \brief Width of grabbers in pixels.
      public: double grabberWidth;

      /// \brief Height of grabbers in pixels.
      public: double grabberHeight;
    };
  }
}
#endif
