/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_WINDOWDOORINSPECTORDIALOG_HH_
#define _GAZEBO_GUI_WINDOWDOORINSPECTORDIALOG_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"
#include "gazebo/gui/building/BaseInspectorDialog.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class WindowDoorInspectorDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class WindowDoorInspectorDialog WindowDoorInspectorDialog.hh
    /// \brief Dialog for configuring a window or door item.
    class GZ_GUI_VISIBLE WindowDoorInspectorDialog
      : public BaseInspectorDialog
    {
      Q_OBJECT

      /// \brief Dialog modes
      public: enum mode {WINDOW, DOOR};

      /// \brief Constructor
      /// \param[in] _mode Dialog mode
      /// \param[in] _parent Parent QWidget.
      public: WindowDoorInspectorDialog(int _mode = WINDOW,
          QWidget *_parent = 0);

      /// \brief Destructor
      public: ~WindowDoorInspectorDialog();

      /// \brief Get the item width.
      /// \return Width in pixels.
      public: double GetWidth() const;

      /// \brief Get the item height.
      /// \return Height in pixels.
      public: double GetHeight() const;

      /// \brief Get the item depth.
      /// \return Depth in pixels.
      public: double GetDepth() const;

      /// \brief Get the item position.
      /// \return Item position in pixel coordinates.
      public: QPointF GetPosition() const;

      /// \brief Get the item elevation.
      /// \return Item elevation in pixels.
      public: double GetElevation() const;

      /// \brief Get the item type.
      /// \return Item type.
      public: std::string GetType() const;

      /// \brief Set the item name.
      /// \param[in] _name Name to set to.
      public: void SetName(const std::string &_name);

      /// \brief Set the item width.
      /// \param[in] _width Width in pixels.
      public: void SetWidth(double _width);

      /// \brief Set the item height.
      /// \param[in] _height Height in pixels.
      public: void SetHeight(double _height);

      /// \brief Set the item depth.
      /// \param[in] _depth Depth in pixels.
      public: void SetDepth(double _depth);

      /// \brief Set the item scene position.
      /// \param[in] _pos Position in pixel coordinates.
      public: void SetPosition(const QPointF &_pos);

      /// \brief Set the item elevation.
      /// \param[in] _elevation Item elevation in pixels.
      public: void SetElevation(double _elevation);

      /// \brief Set the item type.
      /// \param[in] _type Item type.
      public: void SetType(const std::string &_type);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<WindowDoorInspectorDialogPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
