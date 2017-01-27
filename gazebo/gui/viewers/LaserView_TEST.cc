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

#include <ignition/math/Rand.hh>
#include "gazebo/gui/viewers/LaserView_TEST.hh"

/////////////////////////////////////////////////
void LaserView_TEST::Construction()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/ray_test.world");

  // Create a new laser view widget
  gazebo::gui::LaserView *view = new gazebo::gui::LaserView(NULL);

  QVERIFY(view != NULL);

  view->show();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame*>("blackBorderFrame");

  QVERIFY(frame != NULL);

  view->SetTopic("~/hokuyo/link/laser/scan");

  // Spin the Qt update loop for a while to process events.
  for (int j = 0; j < 50; ++j)
  {
    gazebo::common::Time::MSleep(ignition::math::Rand::IntUniform(10, 50));
    QCoreApplication::processEvents();
  }

  view->hide();
  delete view;
}

/////////////////////////////////////////////////
void LaserView_TEST::Buttons()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/ray_test.world");

  // Create a new laser view widget
  gazebo::gui::LaserView *view = new gazebo::gui::LaserView(NULL);

  QVERIFY(view != NULL);

  view->show();

  view->SetTopic("~/hokuyo/link/laser/scan");

  // Spin the Qt update loop for a while to process events.
  this->ProcessEventsAndDraw(NULL, 50, 10);

  // Get Fit in View button and click it
  QPushButton *pushButton = view->findChild<QPushButton *>();
  QVERIFY(pushButton != NULL);
  pushButton->click();

  // Spin the Qt update loop for a while to process events.
  this->ProcessEventsAndDraw(NULL, 50, 10);

  // Get Degrees radio button and toggle it
  QRadioButton *radioButton = view->findChild<QRadioButton *>();
  QVERIFY(radioButton != NULL);
  radioButton->toggle();

  // Spin the Qt update loop for a while to process events.
  this->ProcessEventsAndDraw(NULL, 50, 10);

  view->hide();
  delete view;
}

// Generate a main function for the test
QTEST_MAIN(LaserView_TEST)
