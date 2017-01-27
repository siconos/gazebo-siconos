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
#ifndef _GAZEBO_GUI_BUILDING_LEVELINSPECTORDIALOGPRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_LEVELINSPECTORDIALOGPRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for LevelInspectorDialog
    class LevelInspectorDialogPrivate
    {
      /// \brief Widget containing the floor specs.
      public: QWidget *floorWidget;

      /// \brief Editable line that holds the the level name.
      public: QLineEdit *levelNameLineEdit;

      /// \brief Spin box for configuring the level height.
      public: QDoubleSpinBox *heightSpinBox;

      /// \brief Spin box for configuring the floor thickness.
      public: QDoubleSpinBox *floorThicknessSpinBox;
    };
  }
}
#endif
