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

#ifndef _GAZEBO_GUI_LINK_INSPECTOR_PRIVATE_HH_
#define _GAZEBO_GUI_LINK_INSPECTOR_PRIVATE_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class CollisionConfig;
    class LinkConfig;
    class VisualConfig;

    /// \brief Private data for the LinkInspector class
    class LinkInspectorPrivate
    {
      /// \brief Main tab widget within the link inspector.
      public: QTabWidget *tabWidget;

      /// \brief Label that displays the name of the link.
      public: QLabel *linkNameLabel;

      /// \brief Widget with configurable link properties.
      public: LinkConfig *linkConfig;

      /// \brief Widget with configurable visual properties.
      public: VisualConfig *visualConfig;

      /// \brief Widget with configurable collision properties.
      public: CollisionConfig *collisionConfig;

      /// \brief Unique id for this link.
      public: std::string linkId;
    };
  }
}
#endif
