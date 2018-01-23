/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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

#ifndef GAZEBO_GUI_MODEL_MODELEDITORPALETTE_HH_
#define GAZEBO_GUI_MODEL_MODELEDITORPALETTE_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/ModelCreator.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class KeyEvent;
  }

  namespace gui
  {
    // Forward declare private data.
    class ModelEditorPalettePrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelEditorPalette ModelEditorPalette.hh
    /// \brief A palette of model items which can be added to the editor.
    class GZ_GUI_VISIBLE ModelEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: explicit ModelEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ModelEditorPalette();

      /// \brief Add an item to the model editor palette.
      /// \param[in] _Item item to add.
      /// \param[in] _category Category to add the item too.
      public: void AddItem(QWidget *_item,
          const std::string &_category = "Other");

      /// \brief Add a widget inside the model editor palette widget
      /// \param[in] _index Index in the splitter to insert the widget at.
      /// \param[in] _widget Widget to be added.
      public: void InsertWidget(const unsigned int _index, QWidget *_widget);

      /// \brief Remove a widget from the model editor palette widget
      /// \param[in] _widget Widget to be added.
      public: void RemoveWidget(QWidget *_widget);

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      public: void CreateJoint(const std::string &_type);

      /// \brief Get the model creator.
      /// \return a pointer to the model creator.
      public: gui::ModelCreator *ModelCreator();

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief Qt callback when cylinder button is clicked.
      private slots: void OnCylinder();

      /// \brief Qt callback when sphere button is clicked.
      private slots: void OnSphere();

      /// \brief Qt callback when box button is clicked.
      private slots: void OnBox();

      /// \brief Qt callback when custom button is clicked.
      private slots: void OnCustom();

      /// \brief Qt callback when a link has been added.
      private slots: void OnLinkAdded();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelEditorPalettePrivate> dataPtr;
    };
  }
}
#endif
