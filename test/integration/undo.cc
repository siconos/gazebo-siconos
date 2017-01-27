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

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelAlign.hh"
#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/UserCmdHistory.hh"
#include "undo.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void UndoTest::OnUndoRedo(ConstUndoRedoPtr &_msg)
{
  if (_msg->undo())
    this->g_undoMsgReceived = true;
  else
    this->g_redoMsgReceived = true;
}

/////////////////////////////////////////////////
void UndoTest::OnUserCmdStats(ConstUserCmdStatsPtr &_msg)
{
  g_undoCmdCount = _msg->undo_cmd_count();
  g_redoCmdCount = _msg->redo_cmd_count();
}

/////////////////////////////////////////////////
void UndoTest::MsgPassing()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", true, false, false);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Check actions
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_redoAct != NULL);
  QVERIFY(gazebo::gui::g_undoHistoryAct != NULL);
  QVERIFY(gazebo::gui::g_redoHistoryAct != NULL);

  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Transport
  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr userCmdPub =
      node->Advertise<gazebo::msgs::UserCmd>("~/user_cmd");
  gazebo::transport::PublisherPtr undoRedoPub =
      node->Advertise<gazebo::msgs::UndoRedo>("~/undo_redo");
  gazebo::transport::SubscriberPtr undoRedoSub = node->Subscribe("~/undo_redo",
      &UndoTest::OnUndoRedo, this);
  gazebo::transport::SubscriberPtr userCmdStatsSub =
      node->Subscribe("~/user_cmd_stats",
      &UndoTest::OnUserCmdStats, this);

  // Check that no user cmd stats have been received
  QCOMPARE(g_undoCmdCount, -1);
  QCOMPARE(g_redoCmdCount, -1);

  // Publish a few command msgs as if the user was performing commands
  std::vector<gazebo::common::Time> cmdTimes;
  for (auto num : {1, 2, 3})
  {
    world->Step(100);

    gazebo::msgs::UserCmd msg;
    msg.set_description("description_" + std::to_string(num));
    msg.set_type(gazebo::msgs::UserCmd::MOVING);

    userCmdPub->Publish(msg);

    this->ProcessEventsAndDraw(mainWindow);

    // Save sim times
    cmdTimes.push_back(world->SimTime());
    if (num == 1)
      QVERIFY(cmdTimes[num-1] > gazebo::common::Time::Zero);
    else
      QVERIFY(cmdTimes[num-1] > cmdTimes[num-2]);

    // Check that the server received the message and published proper stats
    QCOMPARE(g_undoCmdCount, num);
    QCOMPARE(g_redoCmdCount, 0);

    // Check that only undo was enabled
    QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
    QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
    QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
    QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);
  }
  world->Step(100);

  // Get sim time
  gazebo::common::Time currentTime;
  gazebo::common::Time afterCmdsTime = world->SimTime();
  QVERIFY(afterCmdsTime > cmdTimes[2]);

  // Trigger undo
  QVERIFY(this->g_undoMsgReceived == false);
  gazebo::gui::g_undoAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Check undo msg was published
  QVERIFY(this->g_undoMsgReceived == true);

  // Check we went back to the last cmd time
  currentTime = world->SimTime();
  QVERIFY(currentTime == cmdTimes[2]);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 2);
  QCOMPARE(g_redoCmdCount, 1);

  // Check that redo is also enabled now
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == true);

  // Trigger redo
  QVERIFY(this->g_redoMsgReceived == false);
  gazebo::gui::g_redoAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Check undo msg was published
  QVERIFY(this->g_redoMsgReceived == true);

  // Check we moved forward to the moment undo was triggered
  currentTime = world->SimTime();
  QVERIFY(currentTime == afterCmdsTime);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 3);
  QCOMPARE(g_redoCmdCount, 0);

  // Check that redo is not enabled anymore
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Publish an undo request with skipped samples
  {
    gazebo::msgs::UndoRedo msg;
    msg.set_undo(true);
    msg.set_id(0);

    undoRedoPub->Publish(msg);
  }

  this->ProcessEventsAndDraw(mainWindow);

  // Check we went back to the first cmd time
  currentTime = world->SimTime();
  QVERIFY(currentTime == cmdTimes[0]);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 0);
  QCOMPARE(g_redoCmdCount, 3);

  // Check that redo is enabled but undo isn't
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == true);

  // Publish a redo request with skipped samples
  {
    gazebo::msgs::UndoRedo msg;
    msg.set_undo(false);
    msg.set_id(2);

    undoRedoPub->Publish(msg);
  }

  this->ProcessEventsAndDraw(mainWindow);

  // Check we moved forward to the moment undo was triggered
  currentTime = world->SimTime();
  QVERIFY(currentTime == afterCmdsTime);

  // Check that the server received the message and published proper stats
  QCOMPARE(g_undoCmdCount, 3);
  QCOMPARE(g_redoCmdCount, 0);

  // Check that undo is enabled but redo isn't
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoAct->isEnabled() == false);
  QVERIFY(gazebo::gui::g_undoHistoryAct->isEnabled() == true);
  QVERIFY(gazebo::gui::g_redoHistoryAct->isEnabled() == false);

  // Clean up
  node.reset();
  delete mainWindow;
  mainWindow = NULL;
  world->Fini();
}

/////////////////////////////////////////////////
void UndoTest::UndoTranslateModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get box model
  auto boxModel = world->ModelByName("box");
  QVERIFY(boxModel != NULL);
  auto boxInitialPose = boxModel->WorldPose();

  // Get box visual
  auto boxVis = scene->GetVisual("box");
  QVERIFY(boxVis != NULL);
  QVERIFY(boxVis->WorldPose() == boxInitialPose);

  // Move visual
  auto boxFinalPose = ignition::math::Pose3d(10, 20, 0.5, 0, 0, 0);
  boxVis->SetWorldPose(boxFinalPose);
  QVERIFY(boxVis->WorldPose() != boxInitialPose);
  QVERIFY(boxVis->WorldPose() == boxFinalPose);

  // Check that model has not moved yet
  QVERIFY(boxModel->WorldPose() == boxInitialPose);

  // Trigger user command
  gazebo::gui::ModelManipulator::Instance()->SetManipulationMode("translate");
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(boxVis);

  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetDragging(true);
  gazebo::gui::ModelManipulator::Instance()->OnMouseReleaseEvent(mouseEvent);

  // Check that box model moved
  int sleep = 0;
  int maxSleep = 10;
  while (boxModel->WorldPose() != boxFinalPose && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose() << "] final pose [" <<
      boxFinalPose << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() == boxFinalPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check box is back to initial pose
  sleep = 0;
  maxSleep = 10;
  while (boxModel->WorldPose() != boxInitialPose && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose() << "] initial pose [" <<
      boxInitialPose << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() == boxInitialPose);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoRotateLight()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get sun light
  auto sunLight = world->LightByName("sun");
  QVERIFY(sunLight != NULL);
  auto sunInitialRot = sunLight->WorldPose().Rot();

  // Get sun visual
  auto sunVis = scene->GetVisual("sun");
  QVERIFY(sunVis != NULL);
  QVERIFY(sunVis->Rotation() == sunInitialRot);

  // Move visual
  auto sunFinalRot = ignition::math::Quaterniond(1, 0, 0);
  sunVis->SetRotation(sunFinalRot);
  QVERIFY(sunVis->Rotation() != sunInitialRot);
  QVERIFY(sunVis->Rotation() == sunFinalRot);

  // Check that light has not moved yet
  QVERIFY(sunLight->WorldPose().Rot() == sunInitialRot);

  // Trigger user command
  gazebo::gui::ModelManipulator::Instance()->SetManipulationMode("rotate");
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(sunVis);

  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetDragging(true);
  gazebo::gui::ModelManipulator::Instance()->OnMouseReleaseEvent(mouseEvent);

  // Check that sun light moved
  int sleep = 0;
  int maxSleep = 10;
  while (sunLight->WorldPose().Rot() != sunFinalRot && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Sun rot [" << sunLight->WorldPose().Rot() << "] final pose [" <<
      sunFinalRot << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(sunLight->WorldPose().Rot() == sunFinalRot);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check sun is back to initial pose
  sleep = 0;
  maxSleep = 10;
  while (sunLight->WorldPose().Rot() != sunInitialRot && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Sun pose [" << sunLight->WorldPose().Rot() << "] initial pose [" <<
      sunInitialRot << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(sunLight->WorldPose().Rot() == sunInitialRot);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoScaleModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get box model
  auto boxModel = world->ModelByName("box");
  QVERIFY(boxModel != NULL);
  auto boxInitialScale = boxModel->Scale();

  // Get box visual
  auto boxVis = scene->GetVisual("box");
  QVERIFY(boxVis != NULL);
  QVERIFY(boxVis->Scale() == boxInitialScale);

  // Scale visual
  auto boxFinalScale = ignition::math::Vector3d(0.1, 2, 3);
  boxVis->SetScale(boxFinalScale);
  QVERIFY(boxVis->Scale() != boxInitialScale);
  QVERIFY(boxVis->Scale() == boxFinalScale);

  // Check that model has not been scaled yet
  QVERIFY(boxModel->Scale() == boxInitialScale);

  // Trigger user command
  gazebo::gui::ModelManipulator::Instance()->SetManipulationMode("scale");
  gazebo::gui::ModelManipulator::Instance()->SetAttachedVisual(boxVis);

  gazebo::common::MouseEvent mouseEvent;
  mouseEvent.SetDragging(true);
  gazebo::gui::ModelManipulator::Instance()->OnMouseReleaseEvent(mouseEvent);

  // Check that box model was scaled
  int sleep = 0;
  int maxSleep = 10;
  while (boxModel->Scale() != boxFinalScale && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box scale [" << boxModel->Scale() << "] final scale [" <<
      boxFinalScale << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->Scale() == boxFinalScale);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check box is back to initial scale
  sleep = 0;
  maxSleep = 10;
  while (boxModel->Scale() != boxInitialScale && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box scale [" << boxModel->Scale() << "] initial scale [" <<
      boxInitialScale << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->Scale() == boxInitialScale);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoSnap()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get box model
  auto boxModel = world->ModelByName("box");
  QVERIFY(boxModel != NULL);
  auto boxInitialPose = boxModel->WorldPose();

  // Get box visual
  auto boxVis = scene->GetVisual("box");
  QVERIFY(boxVis != NULL);
  QVERIFY(boxVis->WorldPose() == boxInitialPose);

  // Trigger user command
  ignition::math::Triangle3d triangleSrc(
      ignition::math::Vector3d(0.5, 0.5, 0),
      ignition::math::Vector3d(-0.5, 0.5, 0),
      ignition::math::Vector3d(0.5, -0.5, 0));

  ignition::math::Triangle3d triangleDest(
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d(0, 0, 10),
      ignition::math::Vector3d(10, 0, 0));

  gazebo::gui::ModelSnap::Instance()->Snap(triangleSrc, triangleDest, boxVis);

  // Check that visual moved but model didn't
  QVERIFY(boxVis->WorldPose() != boxInitialPose);
  QVERIFY(boxModel->WorldPose() == boxInitialPose);

  // Check that box model moved
  int sleep = 0;
  int maxSleep = 10;
  while (boxModel->WorldPose() == boxInitialPose && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose() << "] initial pose [" <<
      boxInitialPose << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() != boxInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check box is back to initial pose
  sleep = 0;
  maxSleep = 10;
  while (boxModel->WorldPose() != boxInitialPose && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose() << "] initial pose [" <<
      boxInitialPose << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() == boxInitialPose);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoAlign()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get models
  auto boxModel = world->ModelByName("box");
  QVERIFY(boxModel != NULL);
  auto boxInitialPose = boxModel->WorldPose();

  auto cylinderModel = world->ModelByName("cylinder");
  QVERIFY(cylinderModel != NULL);
  auto cylinderInitialPose = cylinderModel->WorldPose();

  auto sphereModel = world->ModelByName("sphere");
  QVERIFY(sphereModel != NULL);
  auto sphereInitialPose = sphereModel->WorldPose();

  // Get visuals
  auto boxVis = scene->GetVisual("box");
  QVERIFY(boxVis != NULL);
  QVERIFY(boxVis->WorldPose() == boxInitialPose);

  auto cylinderVis = scene->GetVisual("cylinder");
  QVERIFY(cylinderVis != NULL);
  QVERIFY(cylinderVis->WorldPose() == cylinderInitialPose);

  auto sphereVis = scene->GetVisual("sphere");
  QVERIFY(sphereVis != NULL);
  QVERIFY(sphereVis->WorldPose() == sphereInitialPose);

  // Trigger user command
  std::vector<gazebo::rendering::VisualPtr> visuals;
  visuals.push_back(sphereVis);
  visuals.push_back(cylinderVis);
  visuals.push_back(boxVis);

  gazebo::gui::ModelAlign::Instance()->AlignVisuals(visuals, "y", "min",
      "first", true);

  // Check that models haven't moved yet
  QVERIFY(sphereModel->WorldPose() == sphereInitialPose);
  QVERIFY(boxModel->WorldPose() == boxInitialPose);
  QVERIFY(cylinderModel->WorldPose() == cylinderInitialPose);

  // Check that box and cylinder models moved
  int sleep = 0;
  int maxSleep = 10;
  while (boxModel->WorldPose() == boxInitialPose &&
      cylinderModel->WorldPose() == cylinderInitialPose &&
      sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose()
        << "] box initial pose [" << boxInitialPose
        << "] cylinder pose [" << cylinderModel->WorldPose()
        << "] cylinder initial pose [" << cylinderInitialPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() != boxInitialPose);
  QVERIFY(cylinderModel->WorldPose() != cylinderInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check that box and cylinder are back to initial pose
  sleep = 0;
  maxSleep = 10;
  while (boxModel->WorldPose() != boxInitialPose &&
      cylinderModel->WorldPose() != cylinderInitialPose &&
      sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Box pose [" << boxModel->WorldPose()
        << "] box initial pose [" << boxInitialPose
        << "] cylinder pose [" << cylinderModel->WorldPose()
        << "] cylinder initial pose [" << cylinderInitialPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxModel->WorldPose() == boxInitialPose);
  QVERIFY(cylinderModel->WorldPose() == cylinderInitialPose);
  QVERIFY(sphereModel->WorldPose() == sphereInitialPose);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoResetTime()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get box and move it
  auto box = world->ModelByName("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->WorldPose();

  auto boxFinalPose = ignition::math::Pose3d(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->WorldPose() != boxInitialPose);
  QVERIFY(box->WorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->SimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset time
  mainWindow->RenderWidget()->GetTimePanel()->OnTimeReset();

  // Check time
  int sleep = 0;
  int maxSleep = 100;
  auto newTime = world->SimTime();
  while (newTime != gazebo::common::Time::Zero && sleep < maxSleep)
  {
    newTime = world->SimTime();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
      << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == gazebo::common::Time::Zero);

  // Check that box pose wasn't reset
  QVERIFY(box->WorldPose() == boxFinalPose);
  QVERIFY(box->WorldPose() != boxInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time is back to initial time
  sleep = 0;
  maxSleep = 10;
  newTime = world->SimTime();
  while (newTime != initialTime && sleep < maxSleep)
  {
    newTime = world->SimTime();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
      << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == initialTime);

  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoResetWorld()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get box and move it
  auto box = world->ModelByName("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->WorldPose();

  ignition::math::Pose3d boxFinalPose =
    ignition::math::Pose3d(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->WorldPose() != boxInitialPose);
  QVERIFY(box->WorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->SimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset world
  gazebo::gui::g_resetWorldAct->trigger();

  // Check time and box pose
  int sleep = 0;
  int maxSleep = 100;
  auto newTime = world->SimTime();
  auto boxNewPose = box->WorldPose();
  while (newTime != gazebo::common::Time::Zero && boxNewPose != boxInitialPose
      && sleep < maxSleep)
  {
    newTime = world->SimTime();
    boxNewPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
        << "] Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == gazebo::common::Time::Zero);
  QVERIFY(boxNewPose == boxInitialPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time and box were reset
  sleep = 0;
  newTime = world->SimTime();
  boxNewPose = box->WorldPose();
  while (newTime != initialTime && boxNewPose == boxInitialPose &&
      sleep < maxSleep)
  {
    newTime = world->SimTime();
    boxNewPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial time [" << initialTime << "] new time [" << newTime
        << "] Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newTime == initialTime);
  QVERIFY(boxNewPose == boxFinalPose);

  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoResetModelPoses()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get box and move it
  auto box = world->ModelByName("box");
  QVERIFY(box != NULL);
  auto boxInitialPose = box->WorldPose();

  ignition::math::Pose3d boxFinalPose =
    ignition::math::Pose3d(10, 20, 0.5, 0, 0, 0);
  box->SetWorldPose(boxFinalPose);
  QVERIFY(box->WorldPose() != boxInitialPose);
  QVERIFY(box->WorldPose() == boxFinalPose);

  // Get sim time
  world->SetPaused(true);
  auto initialTime = world->SimTime();
  QVERIFY(initialTime != gazebo::common::Time::Zero);

  // Reset model poses
  gazebo::gui::g_resetModelsAct->trigger();

  // Check time and box pose
  int sleep = 0;
  int maxSleep = 100;
  auto boxNewPose = box->WorldPose();
  while (boxNewPose != boxInitialPose && sleep < maxSleep)
  {
    boxNewPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxNewPose == boxInitialPose);

  // Check that time wasn't reset
  QVERIFY(world->SimTime() != gazebo::common::Time::Zero);
  QVERIFY(world->SimTime() == initialTime + gazebo::common::Time(0.001));

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check time and box were reset
  sleep = 0;
  boxNewPose = box->WorldPose();
  while (boxNewPose == boxInitialPose && sleep < maxSleep)
  {
    boxNewPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Initial pose [" << boxInitialPose << "] new pose [" << boxNewPose
        << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(boxNewPose == boxFinalPose);

  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void UndoTest::UndoWrench()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Get world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Get box
  auto box = world->ModelByName("box");
  QVERIFY(box != NULL);
  auto boxPose = box->WorldPose();

  // Create the main window.
  auto mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Transport
  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();

  auto userCmdPub = node->Advertise<gazebo::msgs::UserCmd>("~/user_cmd");

  // Apply wrench to box
  gazebo::msgs::Wrench wrenchMsg;
  gazebo::msgs::Set(wrenchMsg.mutable_force(),
      ignition::math::Vector3d(10000, 0, 0));
  gazebo::msgs::Set(wrenchMsg.mutable_torque(),
      ignition::math::Vector3d::Zero);
  gazebo::msgs::Set(wrenchMsg.mutable_force_offset(),
      ignition::math::Vector3d::Zero);

  gazebo::msgs::UserCmd msg;
  msg.set_description("Apply wrench");
  msg.set_type(gazebo::msgs::UserCmd::WRENCH);
  msg.mutable_wrench()->CopyFrom(wrenchMsg);
  msg.set_entity_name("box::link");

  userCmdPub->Publish(msg);

  // Check box has moved
  int sleep = 0;
  int maxSleep = 100;
  auto newBoxPose = box->WorldPose();
  while (newBoxPose == boxPose && sleep < maxSleep)
  {
    newBoxPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Command: Initial pose [" << boxPose << "] new pose [" << newBoxPose
      << "]    sleep [" << sleep << "]" << std::endl;
  QVERIFY(newBoxPose != boxPose);

  // Undo
  QVERIFY(gazebo::gui::g_undoAct != NULL);
  QVERIFY(gazebo::gui::g_undoAct->isEnabled() == true);

  gazebo::gui::g_undoAct->trigger();

  // Check box is back at original pose
  sleep = 0;
  maxSleep = 10;
  newBoxPose = box->WorldPose();
  while (newBoxPose != boxPose && sleep < maxSleep)
  {
    newBoxPose = box->WorldPose();
    gazebo::common::Time::MSleep(100);
    QCoreApplication::processEvents();
    mainWindow->repaint();
    sleep++;
  }
  gzmsg << "Undo: Initial pose [" << boxPose << "] new pose [" << newBoxPose <<
      "]" << std::endl;
  QVERIFY(newBoxPose == boxPose);

  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(UndoTest)
