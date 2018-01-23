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
#ifndef GAZEBO_GUI_VIEWERS_IMAGEFRAME_HH_
#define GAZEBO_GUI_VIEWERS_IMAGEFRAME_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace gui
  {
    class ImageFramePrivate;

    /// \brief Frame that draws an image when a paintevent is received.
    class ImageFrame : public QFrame
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Qt widget
      public: explicit ImageFrame(QWidget *_parent);

      /// \brief Destructor
      public: virtual ~ImageFrame();

      /// \brief Receives incoming image messages.
      /// \param[in] _msg New image message.
      public: void OnImage(const msgs::Image &_msg);

      /// \brief Event used to paint the image.
      /// \param[in] _event Pointer to the event information.
      protected: void paintEvent(QPaintEvent *_event);

      /// \brief Pointer to private data
      private: std::unique_ptr<ImageFramePrivate> dataPtr;
    };
  }
}

#endif
