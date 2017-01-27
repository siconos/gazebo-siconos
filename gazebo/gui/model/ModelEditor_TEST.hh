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

#ifndef _GAZEBO_MODEL_EDITOR_TEST_HH_
#define _GAZEBO_MODEL_EDITOR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the ModelEditor class.
class ModelEditor_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Tests adding an item to the model editor palette.
  private slots: void AddItemToPalette();

  /// \brief Tests entering and exiting model editor mode.
  private slots: void OnEdit();

  /// \brief Tests the visibility of the insert tab.
  private slots: void InsertTab();
};

#endif
