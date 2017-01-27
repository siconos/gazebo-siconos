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
#ifndef _GAZEBO_EXTRUDE_DIALOG_PRIVATE_HH_
#define _GAZEBO_EXTRUDE_DIALOG_PRIVATE_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class ExtrudeDialogPrivate ExtrudeDialogPrivate.hh
    /// \brief Private data for the ExtrudeDialog class
    class ExtrudeDialogPrivate
    {
      /// \brief Full file path of the SVG file.
      public: std::string filename;

      /// \brief Thickness spin box.
      public: QDoubleSpinBox *thicknessSpin;

      /// \brief Resolution spin box.
      public: QDoubleSpinBox *resolutionSpin;

      /// \brief Samples spin box.
      public: QSpinBox *samplesSpin;

      /// \brief Width of the graphics view in px.
      public: int viewWidth;

      /// \brief Display the SVG paths.
      public: QGraphicsView *view;
    };
  }
}
#endif
