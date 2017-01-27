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

#ifndef _GAZEBO_JOINT_INSPECTOR_TEST_HH_
#define _GAZEBO_JOINT_INSPECTOR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the joint inspector.
class JointInspector_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Constructor
  public: JointInspector_TEST() = default;

  /// \brief Test adding and removing links.
  private slots: void AddRemoveLink();

  /// \brief Test adding and removing nested links.
  private slots: void AddRemoveNestedLink();

  /// \brief Test swapping parent / child links
  private slots: void Swap();

  /// \brief Test pressing remove button.
  private slots: void RemoveButton();

  /// \brief Test that the Applied signal is emitted when widgets are edited.
  private slots: void AppliedSignal();

  /// \brief Slot that receives Applied signals.
  private slots: void OnApply();

  /// \brief Count how many Applied signals have been emitted.
  private: unsigned int g_appliedSignalCount = 0;
};

#endif
