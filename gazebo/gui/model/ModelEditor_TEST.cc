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

#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/model/ModelEditor.hh"
#include "gazebo/gui/model/ModelEditor_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelEditor_TEST::AddItemToPalette()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // verify we have a model editor widget
  gui::ModelEditor *modelEditor =
      dynamic_cast<gui::ModelEditor *>(mainWindow->Editor("model"));
  QVERIFY(modelEditor);

  // add a custom push button to the model editor palette
  QPushButton *testButton = new QPushButton("TEST_BUTTON");
  testButton->setObjectName("my_custom_test_button");
  modelEditor->AddItemToPalette(testButton, "test_categorty");

  QPushButton *retButton =
      mainWindow->findChild<QPushButton *>("my_custom_test_button");

  // verify that the push button is added.
  QVERIFY(retButton);
  QVERIFY(retButton->text().toStdString() ==  "TEST_BUTTON");

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelEditor_TEST::OnEdit()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // verify we have a model editor widget
  gui::ModelEditor *modelEditor =
      dynamic_cast<gui::ModelEditor *>(mainWindow->Editor("model"));
  QVERIFY(modelEditor);

  QVERIFY(gui::g_editModelAct != NULL);

  // verify simulation is not paused
  QVERIFY(!mainWindow->IsPaused());

  // switch to editor mode
  gui::g_editModelAct->toggle();

  // wait for the gui paused state to update
  int maxSleep = 50;
  int sleep = 0;
  while (!mainWindow->IsPaused() && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(50);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // verify simulation is paused
  QVERIFY(mainWindow->IsPaused());

  // swtich back to simulation mode
  gui::g_editModelAct->toggle();

  // check the gui paused state and it should not change
  maxSleep = 50;
  sleep = 0;
  while (mainWindow->IsPaused() && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(50);
    sleep++;
  }
  QVERIFY(sleep == maxSleep);

  // verify simulation is still paused
  QVERIFY(mainWindow->IsPaused());

  // run the simulation
  mainWindow->Play();

  // wait for the gui paused state to update
  maxSleep = 50;
  sleep = 0;
  while (mainWindow->IsPaused() && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(50);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // verify simulation is now running
  QVERIFY(!mainWindow->IsPaused());

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelEditor_TEST::InsertTab()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the main tab
  auto mainTab = mainWindow->findChild<QTabWidget *>("mainTab");
  QVERIFY(mainTab != NULL);

  // Get the insert tab
  QWidget *insertModel = NULL;
  for (int i = 0; i < mainTab->count(); ++i)
  {
    if (mainTab->tabText(i) == tr("Insert"))
    {
      insertModel = mainTab->widget(i);
      break;
    }
  }
  QVERIFY(insertModel != NULL);

  // Switch to editor mode
  QVERIFY(gui::g_editModelAct != NULL);
  gui::g_editModelAct->toggle();

  // Check that the insert tab is not in mainTab anymore
  insertModel = NULL;
  for (int i = 0; i < mainTab->count(); ++i)
  {
    if (mainTab->tabText(i) == tr("Insert"))
    {
      insertModel = mainTab->widget(i);
      break;
    }
  }
  QVERIFY(insertModel == NULL);

  // Switch back to simulation
  gui::g_editModelAct->toggle();

  // Check that the insert tab in mainTab again
  insertModel = NULL;
  for (int i = 0; i < mainTab->count(); ++i)
  {
    if (mainTab->tabText(i) == tr("Insert"))
    {
      insertModel = mainTab->widget(i);
      break;
    }
  }
  QVERIFY(insertModel != NULL);

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditor_TEST)
