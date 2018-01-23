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
#ifndef GAZEBO_GUI_BUILDING_BUILDINGEDITOR_HH_
#define GAZEBO_GUI_BUILDING_BUILDINGEDITOR_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Editor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingEditorPalette;
    class BuildingEditorWidget;

    /// \class TerrainEditor TerrainEditor.hh gui/gui.hh
    /// \brief Interface to the terrain editor.
    class GZ_GUI_VISIBLE BuildingEditor : public Editor
    {
      Q_OBJECT

      /// \brief Constuctor.
      /// \param[in] _mainWindow Pointer to the mainwindow.
      public: explicit BuildingEditor(MainWindow *_mainWindow);

      /// \brief Destuctor.
      public: virtual ~BuildingEditor();

      /// \brief Qt callback when the building editor's save action is
      /// triggered.
      private slots: void Save();

      /// \brief Qt callback when the building editor's save as action is
      /// triggered.
      private slots: void SaveAs();

      /// \brief Qt callback when the building editor's new action is
      /// triggered.
      private slots: void New();

      /// \brief Qt callback when the building editor's exit action is
      /// triggered.
      private slots: void Exit();

      /// \brief Callback from the building editor when the building model
      /// has been completed.
      private: void OnFinish();

      /// \brief QT callback when entering building edit mode
      /// \param[in] _checked True if the menu item is checked
      private slots: void OnEdit(bool _checked);

      /// \brief Contains all the building editor tools.
      private: BuildingEditorPalette *buildingPalette;

      /// \brief Building editor widget for creating a building model
      private: BuildingEditorWidget *buildingEditorWidget;

      /// \brief Create menus
      private: void CreateMenus();

      /// \brief Qt event filter currently used to filter tips label events.
      /// \param[in] _obj Object that is watched by the event filter.
      /// \param[in] _event Qt event.
      /// \return True if the event is handled.
      private: bool eventFilter(QObject *_obj, QEvent *_event);

      /// \brief Our custom menubar
      private: QMenuBar *menuBar;

      /// \brief Action to save model.
      private: QAction *saveAct;

      /// \brief Action to save model as.
      private: QAction *saveAsAct;

      /// \brief Action to start a new model.
      private: QAction *newAct;

      /// \brief Action to exit the editor.
      private: QAction *exitAct;

      /// \brief Save the main window paused state to use when returning.
      private: bool mainWindowPaused;

      /// \brief Label which shows tips when clicked or hovered.
      private: QLabel *tipsLabel;
    };
  }
}
#endif
