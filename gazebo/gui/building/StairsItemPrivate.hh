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

#ifndef _GAZEBO_GUI_BUILDING_STAIRSITEM_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_STAIRSITEM_PRIVATE_HH_

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class StairsInspectorDialog;

    /// \brief Private data for the StairsItem class
    class StairsItemPrivate
    {
      /// \brief Depth of staircase item in pixels.
      public: double stairsDepth;

      /// \brief Height of staircase item in pixels.
      public: double stairsHeight;

      /// \brief Width of staircase item in pixels.
      public: double stairsWidth;

      /// \brief Scene position of staircase item in pixel coordinates.
      public: ignition::math::Vector2d stairsPos;

      /// \brief Elevation of staircase item in pixels.
      public: double stairsElevation;

      /// \brief Number of steps in the staircase item.
      public: int stairsSteps;

      /// \brief Inspector for configuring the staircase item.
      public: StairsInspectorDialog *inspector;
    };
  }
}
#endif
