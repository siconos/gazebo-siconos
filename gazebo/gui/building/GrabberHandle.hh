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

#ifndef GAZEBO_GUI_BUILDING_GRABBERHANDLE_HH_
#define GAZEBO_GUI_BUILDING_GRABBERHANDLE_HH_

#include <vector>
#include <memory>

#include <ignition/math/Color.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class.
    class GrabberHandlePrivate;

    class GZ_GUI_VISIBLE GrabberHandle : public QGraphicsItem
    {
      /// \brief Constructor
      /// \param[in] _parent Parent graphics item
      /// \param[in] _index Index of the grabber handle
      public: GrabberHandle(QGraphicsItem *_parent = 0, int index = 0);

      /// \brief Destructor.
      public: virtual ~GrabberHandle();

      /// \brief Get the index of the grabber handle.
      /// \return Index of the grabber handle.
      public: int Index() const;

      /// \brief Get the current mouse state.
      /// \return The current mouse state.
      public: int MouseState() const;

      /// \brief Get the center point of the grabber handle.
      /// \return Center point in pixel coordinates.
      public: ignition::math::Vector2d CenterPoint() const;

      /// \brief Get the X position of the mouse press.
      /// \return Mouse press X position in pixel coordinates.
      public: double MouseDownX() const;

      /// \brief Get the Y position of the mouse press.
      /// \return Mouse press Y position in pixel coordinates.
      public: double MouseDownY() const;
      /// \brief Get the width of the grabber handle.
      /// \return The width of the grabber handle in pixel coordinates.
      public: double Width() const;

      /// \brief Get the height of the grabber handle.
      /// \return The height of the grabber handle in pixels.
      public: double Height() const;

      /// \brief Get the fill color of the grabber handle.
      /// \return _color Fill color.
      public: ignition::math::Color Color() const;

      /// \brief Set the current mouse state.
      /// \param[in] _state Current mouse state.
      public: void SetMouseState(int _state);

      /// \brief Set the X position of the mouse press.
      /// \param[in] _x Mouse press X position in pixel coordinates.
      public: void SetMouseDownX(double _x);

      /// \brief Set the Y position of the mouse press.
      /// \param[in] _y Mouse press Y position in pixel coordinates.
      public: void SetMouseDownY(double _y);

      /// \brief Set the width of the grabber handle.
      /// \param[in] _width Width in pixels.
      public: void SetWidth(double _width);

      /// \brief Set the height of the grabber handle.
      /// \param[in] _height Height in pixels.
      public: void SetHeight(double _height);

      /// \brief Set the fill color of the grabber handle.
      /// \param[in] _color Fill Color.
      public: void SetColor(const ignition::math::Color &_color);

      /// \brief Set the border color of the grabber handle.
      /// \param[in] _borderColor Border Color.
      public: void SetBorderColor(const ignition::math::Color &_borderColor);

      /// \brief Get the bounding box of the grabber handle.
      /// \return The grabber handle bounding box.
      public: virtual QRectF boundingRect() const;

      /// \brief Get the vector of grabbers linked to this.
      /// \return Vector of linked grabbers.
      public: std::vector<GrabberHandle *> LinkedGrabbers() const;

      /// \brief Push a grabber to the list of grabbers linked to this.
      /// \param[in] _grabber New grabber.
      public: void PushLinkedGrabber(GrabberHandle *_grabber);

      /// \brief Erase a grabber from the list of grabbers linked to this.
      /// \param[in] _grabber Grabber which will be removed.
      public: void EraseLinkedGrabber(GrabberHandle *_grabber);

      /// \brief Qt paint function for drawing the grabber handle.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: virtual void paint(QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Qt mouse hover enter event.
      /// \param[in] _event Qt mouse hover event.
      protected: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover leave event.
      /// \param[in] _event Qt mouse hover event.
      protected: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      protected: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      protected: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      protected: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      protected: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

      /// \brief Qt mouse move event during drag and drop.
      /// \param[in] _event Qt mouse drag and drop event.
      protected: void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GrabberHandlePrivate> dataPtr;
    };
    /// \}
  }
}

#endif
