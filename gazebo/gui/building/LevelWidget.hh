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
#ifndef GAZEBO_GUI_BUILDING_BUILDINGLEVELWIDGET_HH_
#define GAZEBO_GUI_BUILDING_BUILDINGLEVELWIDGET_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class LevelWidgetPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class LevelWidget LevelWidget.hh
    /// \brief A widget for adding and changing building levels.
    class GZ_GUI_VISIBLE LevelWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: explicit LevelWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~LevelWidget();

      /// \brief Qt callback when the selection of the level combo box has been
      /// changed.
      /// \param[in] _level Chosen level.
      public slots: void OnCurrentLevelChanged(int _level);

      /// \brief Qt callback when the add level button has been pressed.
      public slots: void OnAddLevel();

      /// \brief Qt callback when the delete level button has been pressed.
      public slots: void OnDeleteLevel();

      /// \brief Qt callback when the show floorplan button has been pressed.
      public slots: void OnShowFloorplan();

      /// \brief Trigger show floorplan.
      public slots: void OnTriggerShowFloorplan();

      /// \brief Qt callback when the show elements button has been pressed.
      public slots: void OnShowElements();

      /// \brief Trigger show elements.
      public slots: void OnTriggerShowElements();

      /// \brief Callback received when levels are changed externally.
      /// \param[in] _level Level number.
      /// \param[in] _newName New level name.
      private: void OnUpdateLevelWidget(int _level,
          const std::string &_newName);

      /// \brief Callback received when the widget must be reset.
      private: void OnDiscard();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LevelWidgetPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
