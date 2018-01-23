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
#ifndef GAZEBO_GUI_BUILDING_BUILDINGEDITORPALETTE_HH_
#define GAZEBO_GUI_BUILDING_BUILDINGEDITORPALETTE_HH_

#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// Forward declare private data class.
    class BuildingEditorPalettePrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingEditorPalette BuildingEditorPalette.hh
    /// \brief A palette of building items which can be added to the editor.
    class GZ_GUI_VISIBLE BuildingEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: explicit BuildingEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~BuildingEditorPalette();

      /// \brief Get model name
      /// \return Model name
      public: std::string GetModelName() const;

      /// \brief Get a pointer to the custom color dialog.
      /// \return Pointer to the custom color dialog.
      public: QColorDialog *CustomColorDialog() const;

      /// \brief Qt callback when a brush is pressed.
      /// \param[in] _buttonId Id of the button clicked.
      private slots: void OnBrush(int _buttonId);

      /// \brief Qt callback when the Model Name field is changed.
      /// \param[in] _name New name.
      private slots: void OnNameChanged(const QString &_name);

      /// \brief Qt callback when custom color has been selected on the dialog.
      /// \param[in] _color Selected color.
      private slots: void OnCustomColor(const QColor _color);

      /// \brief Cancel whatever is being drawn and uncheck all brushes.
      private slots: void CancelDrawModes();

      /// \brief Callback when user has provided information on where to save
      /// the model to.
      /// \param[in] _saveName Name of model being saved.
      private: void OnSaveModel(const std::string &_saveName);

      /// \brief Event received when an editor item is selected.
      /// \param[in] _mode Type of item to add or empty for none.
      private: void OnCreateEditorItem(const std::string &_mode);

      /// \brief Event received when the user starts a new building model.
      private: void OnNewModel();

      /// \brief Qt callback when the palette is pressed.
      /// \param[in] _event Event.
      private: void mousePressEvent(QMouseEvent *_event);

      /// \brief When the draw wall button is selected.
      private: void OnDrawWall();

      /// \brief When the draw window button is selected.
      private: void OnAddWindow();

      /// \brief When the draw door button is selected.
      private: void OnAddDoor();

      /// \brief When the draw stairs button is selected.
      private: void OnAddStair();

      /// \brief When a default color button is selected.
      /// \param[in] _colorId Id of the color clicked.
      private: void OnDefaultColor(int _colorId);

      /// \brief Open a color dialog when the custom color button is clicked.
      private: void OnCustomColorDialog();

      /// \brief When any color is selected.
      /// \param[in] _color Color selected.
      private: void OnColor(QColor _color);

      /// \brief When a default texture button is selected.
      /// \param[in] _textureId Id of the texture clicked.
      private: void OnTexture(int _textureId);

      /// \brief When the import image button is selected.
      private: void OnImportImage();

      /// \internal
      /// \brief Private data pointer
      private: BuildingEditorPalettePrivate *dataPtr;
    };
    /// \}
  }
}

#endif
