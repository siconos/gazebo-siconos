/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _WALL_INSPECTOR_DIALOG_HH_
#define _WALL_INSPECTOR_DIALOG_HH_

#include "math/Vector2d.hh"
#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class WallInspectorDialog : public QDialog
    {
      Q_OBJECT

      public: WallInspectorDialog(QWidget *_parent = 0);

      public: ~WallInspectorDialog();

      public: double GetLength();

      public: math::Vector2d GetStartPosition();

      public: math::Vector2d GetEndPosition();

      public: double GetHeight();

      public: double GetThickness();

      public: std::string GetMaterial();

      public: void SetLength(double _length);

      public: void SetStartPosition(math::Vector2d _pos);

      public: void SetEndPosition(math::Vector2d _pos);

      public: void SetHeight(double _height);

      public: void SetThickness(double _thickness);

      public: void SetMaterial(std::string _material);

      private slots: void OnCancel();

      private slots: void OnOK();

      private: QDoubleSpinBox *startXSpinBox;

      private: QDoubleSpinBox *startYSpinBox;

      private: QDoubleSpinBox *endXSpinBox;

      private: QDoubleSpinBox *endYSpinBox;

      private: QDoubleSpinBox *heightSpinBox;

      private: QDoubleSpinBox *thicknessSpinBox;

      private: QLabel* wallNameLabel;

      private: QDoubleSpinBox *lengthSpinBox;

      private: QComboBox *materialComboBox;
   };
 }
}

#endif
