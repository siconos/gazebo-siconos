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

#ifndef _JOINT_MAKER_TEST_HH_
#define _JOINT_MAKER_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the JointMaker class.
class JointMaker_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test joint states
  private slots: void JointState();

  /// \brief Test creating and removing joints
  private slots: void CreateRemoveJoint();

  /// \brief Test creating and removing joints between nested model links
  private slots: void CreateRemoveNestedJoint();

  /// \brief Test values of joint default properties.
  private slots: void JointDefaultProperties();

  /// \brief Test toggling joint visualization.
  private slots: void ShowJoints();

  /// \brief Tests selecting joints in the model editor
  private slots: void Selection();

  /// \brief Test getting joint material.
  private slots: void JointMaterial();

  /// \brief Test managing the link list
  private slots: void LinkList();

  /// \brief Test updating joint message
  private slots: void UpdateMsg();
};

#endif
