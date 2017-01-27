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

#ifndef _GAZEBO_MODEL_TREE_WIDGET_TEST_HH_
#define _GAZEBO_MODEL_TREE_WIDGET_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the ModelTreeWidget class.
class ModelTreeWidget_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Tests loading nested models.
  /// FIXME: This test only passes if ran before the others, otherwise it
  /// crashes on QCoreApplication::processEvents
  private slots: void LoadNestedModel();

  /// \brief Tests the nested model list.
  private slots: void AddRemoveNestedModels();

  /// \brief Tests the link list.
  private slots: void AddRemoveLinks();

  /// \brief Tests the joint list.
  private slots: void AddRemoveJoints();

  /// \brief Tests the model plugin list.
  private slots: void AddRemoveModelPlugins();
};

#endif
