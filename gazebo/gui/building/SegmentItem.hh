/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_BUILDING_SEGMENTITEM_HH_
#define GAZEBO_GUI_BUILDING_SEGMENTITEM_HH_

#include <memory>
#include <vector>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/EditorItem.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GrabberHandle;

    // Forward declare private data.
    class SegmentItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class SegmentItem SegmentItem.hh
    /// \brief 2D line segment.
    class GZ_GUI_VISIBLE SegmentItem
      : public EditorItem, public QGraphicsLineItem
    {
      /// \brief Constructor
      /// \param[in] _parent Parent graphics item.
      public: explicit SegmentItem(QGraphicsItem *_parent = 0);

      /// \brief Destructor
      public: ~SegmentItem();

      /// \brief Set the segment's line.
      /// \param[in] _start Start position of the line in pixel coordinates.
      /// \param[in] _end End position of the line in pixel coordinates.
      public: void SetLine(const ignition::math::Vector2d &_start,
          const ignition::math::Vector2d &_end);

      /// \brief Set the start point of the segment.
      /// \param[in] _start Start point of the segment in pixel coordinates.
      public: void SetStartPoint(const ignition::math::Vector2d &_start);

      /// \brief Get the start point of the segment.
      /// \return Start point of the segment in pixel coordinates.
      public: ignition::math::Vector2d StartPoint() const;

      /// \brief Set the end point of the segment.
      /// \param[in] _end End point of the segment in pixel coordinates.
      public: void SetEndPoint(const ignition::math::Vector2d &_end);

      /// \brief Get the end point of the segment.
      /// \return End point of the segment in pixel coordinates.
      public: ignition::math::Vector2d EndPoint() const;

      /// \brief Set the thickness of the segment item on the 2d view.
      /// \param[in] _thickness Thickness in pixels.
      public: void SetThickness(const double _thickness);

      /// \brief Get the thickness of the segment item.
      /// \return Thickness in pixels.
      public: double Thickness() const;

      /// \brief Get the scale of the segment item.
      /// \return Scale of the segment item in px/m.
      public: double Scale() const;

      /// \brief Set the scale of the segment item.
      /// param[in] _scale Scale of the segment item in px/m.
      public: void SetScale(const double _scale);

      /// \brief Set the color of the segment item.
      /// \param[in] _color Color.
      public: void SetColor(const ignition::math::Color &_color);

      /// \brief Show the grabber handles of the segment item.
      /// \param[in] _show True to draw the handles, and false to hide them.
      public: void ShowHandles(const bool _show);

      /// \brief Emit segment changed Qt signals.
      public: void SegmentChanged();

      // Documentation Inherited
      public: ignition::math::Vector3d Size() const;

      // Documentation Inherited
      public: ignition::math::Vector3d ScenePosition() const;

      // Documentation Inherited
      public: double SceneRotation() const;

      /// \brief Get the grabber handles.
      /// \return Vector of grabber pointers.
      public: std::vector<GrabberHandle *>Grabbers() const;

      /// \brief Update item.
      protected: virtual void SegmentUpdated();

      /// \brief Update the position of all grabbers linked to the given one.
      /// \param[in] _grabber Original grabber.
      /// \param[in] _pos New position.
      protected: void UpdateLinkedGrabbers(GrabberHandle *_grabber,
          const ignition::math::Vector2d &_pos);

      /// \brief Filter Qt events and redirect them to the another item.
      /// \param[in] _watched Item that watches and will handle the event.
      /// \param[in] _event Qt event.
      /// \return True to prevent further processing of a given event.
      private: bool sceneEventFilter(QGraphicsItem *watched,
          QEvent *_event);

      /// \brief Filter Qt events and redirect them to the another item.
      /// \param[in] _grabber Grabber that will handle the event.
      /// \param[in] _event Qt event.
      /// \return True to prevent further processing of a given event.
      private: bool GrabberEventFilter(GrabberHandle *_grabber,
          QEvent *_event);

      /// \brief Qt mouse hover enter event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover move event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse hover leave event.
      /// \param[in] _event Qt mouse hover event.
      private: void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: void mousePressEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

      /// \brief Qt paint function for drawing the line segment.
      /// \param[in] _painter Qt painter object.
      /// \param[in] _option Qt style options for the item.
      /// \param[in] _widget Qt widget being painted on.
      private: void paint(QPainter *_painter,
          const QStyleOptionGraphicsItem *_option, QWidget *_widget);

      /// \brief Angle to snap in degrees.
      public: static const double SnapAngle;

      /// \brief Length to snap in meters.
      public: static const double SnapLength;

      /// \brief A list of grabber handles for this item. One grabber for each
      /// endpoint.
      protected: std::vector<GrabberHandle *> grabbers;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<SegmentItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
