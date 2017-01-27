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
#ifndef _ALIGN_WIDGET_PRIVATE_HH_
#define _ALIGN_WIDGET_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class AlignWidgetPrivate AlignWidgetPrivate.hh
    /// \brief Private data for the AlignWidget class
    class AlignWidgetPrivate
    {
      /// \brief Toolbar containing x alignment actions.
      public: QToolBar *xAlignBar;

      /// \brief Toolbar containing y alignment actions.
      public: QToolBar *yAlignBar;

      /// \brief Toolbar containing z alignment actions.
      public: QToolBar *zAlignBar;

      /// \brief Qt Signal mapper for mapping align actions
      public: QSignalMapper *alignSignalMapper;

      /// \brief Keep track whether to align with first or last selected entity.
      public: int alignRelativeTarget;

      /// \brief Keep track whether to align in the default direction (min to
      /// min, max to max) or in the inverted direction (min to max). It doesn't
      /// affect center align.
      public: bool inverted = false;
    };
  }
}
#endif
