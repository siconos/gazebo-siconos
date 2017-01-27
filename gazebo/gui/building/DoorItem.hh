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

#ifndef _GAZEBO_GUI_BUILDING_DOORITEM_HH_
#define _GAZEBO_GUI_BUILDING_DOORITEM_HH_

#include <memory>
#include <ignition/math/Vector3.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/RectItem.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class DoorItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class DoorItem DoorItem.hh
    /// \brief 2D representation of a door
    class GZ_GUI_VISIBLE DoorItem : public RectItem
    {
      Q_OBJECT

      /// \brief Constructor
      public: DoorItem();

      /// \brief Destructor
      public: ~DoorItem();

      // Documentation inherited
      public: virtual ignition::math::Vector3d Size() const;

      // Documentation inherited
      public: virtual ignition::math::Vector3d ScenePosition() const;

      // Documentation inherited
      public: virtual double SceneRotation() const;

      // Documentation inherited
      private: virtual void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      // Documentation inherited
      private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

      // Documentation inherited
      private slots: void OnApply();

      // Documentation inherited
      private slots: void OnOpenInspector();

      // Documentation inherited
      private slots: void OnDeleteItem();

      /// \brief Emit door changed signals
      public: void DoorChanged();

      /// \brief Emit size changed signals
      private: void SizeChanged();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<DoorItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
