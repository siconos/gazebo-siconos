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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelAlign.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/gui/ModelAlign_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMinX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X() - targetBbox.XLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minX = vis->WorldPose().Pos().X() + centerOffsets[i].X() -
        bbox.XLength()/2.0;
    QVERIFY(ignition::math::equal(minX, targetMinX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMinReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMinX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X() - targetBbox.XLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minX = vis->WorldPose().Pos().X() + centerOffsets[i].X() +
        bbox.XLength()/2.0;
    QVERIFY(ignition::math::equal(minX, targetMinX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "center", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetCenterX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X();

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double centerX = vis->WorldPose().Pos().X() + centerOffsets[i].X();
    QVERIFY(ignition::math::equal(centerX, targetCenterX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "max", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMaxX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X() + targetBbox.XLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxX = vis->WorldPose().Pos().X() + centerOffsets[i].X()
        + bbox.XLength()/2.0;
    QVERIFY(ignition::math::equal(maxX, targetMaxX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMaxReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "max", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMaxX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X() + targetBbox.XLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxX = vis->WorldPose().Pos().X() + centerOffsets[i].X() -
        bbox.XLength()/2.0;
    QVERIFY(ignition::math::equal(maxX, targetMaxX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "min", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMinY = modelVisuals[0]->WorldPose().Pos().Y() +
      centerOffsets[0].Y() - targetBbox.YLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minY = vis->WorldPose().Pos().Y() + centerOffsets[i].Y() -
        bbox.YLength()/2.0;
    QVERIFY(ignition::math::equal(minY, targetMinY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMinReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "min", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMinY = modelVisuals[0]->WorldPose().Pos().Y() +
      centerOffsets[0].Y() - targetBbox.YLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minY = vis->WorldPose().Pos().Y() + centerOffsets[i].Y() +
        bbox.YLength()/2.0;
    QVERIFY(ignition::math::equal(minY, targetMinY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "center", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetCenterY = modelVisuals[0]->WorldPose().Pos().Y() +
      centerOffsets[0].Y();


  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double centerY = vis->WorldPose().Pos().Y() + centerOffsets[i].Y();
    QVERIFY(ignition::math::equal(centerY, targetCenterY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "max", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMaxY = modelVisuals[0]->WorldPose().Pos().Y() +
      centerOffsets[0].Y() + targetBbox.YLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxY = vis->WorldPose().Pos().Y() + centerOffsets[i].Y()
        + bbox.YLength()/2.0;
    QVERIFY(ignition::math::equal(maxY, targetMaxY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMaxReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "max", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMaxY = modelVisuals[0]->WorldPose().Pos().Y() +
      centerOffsets[0].Y() + targetBbox.YLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxY = vis->WorldPose().Pos().Y() + centerOffsets[i].Y() -
        bbox.YLength()/2.0;
    QVERIFY(ignition::math::equal(maxY, targetMaxY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "min", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMinZ = modelVisuals[0]->WorldPose().Pos().Z() +
      centerOffsets[0].Z() - targetBbox.ZLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minZ = vis->WorldPose().Pos().Z() + centerOffsets[i].Z() -
        bbox.ZLength()/2.0;
    QVERIFY(ignition::math::equal(minZ, targetMinZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMinReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "min", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMinZ = modelVisuals[0]->WorldPose().Pos().Z() +
      centerOffsets[0].Z() - targetBbox.ZLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double minZ = vis->WorldPose().Pos().Z() + centerOffsets[i].Z() +
        bbox.ZLength()/2.0;
    QVERIFY(ignition::math::equal(minZ, targetMinZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "center", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetCenterZ = modelVisuals[0]->WorldPose().Pos().Z() +
      centerOffsets[0].Z();

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double centerZ = vis->WorldPose().Pos().Z() + centerOffsets[i].Z();
    QVERIFY(ignition::math::equal(centerZ, targetCenterZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "max", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();

  double targetMaxZ = modelVisuals[0]->WorldPose().Pos().Z() +
      centerOffsets[0].Z() + targetBbox.ZLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxZ = vis->WorldPose().Pos().Z() + centerOffsets[i].Z()
        + bbox.ZLength()/2.0;
    QVERIFY(ignition::math::equal(maxZ, targetMaxZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMaxReverse()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load the align world
  this->Load("worlds/align.world", false, false, true);

  // Get the scene
  auto scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Check all visuals are there
  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  // Get the offsets for all models
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // Align visuals
  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "max", "first", true, true);

  // Get the target pose
  auto targetBbox = modelVisuals[0]->BoundingBox();
  double targetMaxZ = modelVisuals[0]->WorldPose().Pos().Z() +
      centerOffsets[0].Z() + targetBbox.ZLength()/2.0;

  // Check models were properly aligned
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    auto vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();

    double maxZ = vis->WorldPose().Pos().Z() + centerOffsets[i].Z() -
        bbox.ZLength()/2.0;
    QVERIFY(ignition::math::equal(maxZ, targetMaxZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignScale()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("box");
  modelNames.push_back("cylinder");
  modelNames.push_back("sphere");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->Initialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<ignition::math::Vector3d> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    auto modelCenterOffset = modelVis->BoundingBox().Center();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // manually change scale of model visual and verify
  gazebo::rendering::VisualPtr targetVis = modelVisuals[0];
  targetVis->SetScale(ignition::math::Vector3d(1.5, 1, 1));
  QVERIFY(targetVis->Scale() == ignition::math::Vector3d(1.5, 1, 1));

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "first");

  auto targetBbox = modelVisuals[0]->BoundingBox();
  auto targetScale = modelVisuals[0]->Scale();

  // verify other models align at minx of the scaled target model
  double targetMinX = modelVisuals[0]->WorldPose().Pos().X() +
      centerOffsets[0].X() - targetScale.X() * targetBbox.XLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    auto bbox = vis->BoundingBox();
    auto visScale = vis->Scale();

    double minX = vis->WorldPose().Pos().X() + centerOffsets[i].X() -
        visScale.X() * bbox.XLength()/2.0;
    QVERIFY(ignition::math::equal(minX, targetMinX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::SetHighlighted()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/blank.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  // Create a deeply nested visual where each level has a different transparency
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();
  double vis1Transp = 0.0;
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis1->GetTransparency()), vis1Transp, 1e-5));

  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", vis1));
  vis2->Load();
  double vis2Transp = 0.6;
  vis2->SetTransparency(vis2Transp);
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis2->GetTransparency()), vis2Transp, 1e-5));

  gazebo::rendering::VisualPtr vis3_1;
  vis3_1.reset(new gazebo::rendering::Visual("vis3_1", vis2));
  vis3_1->Load();
  double vis3_1Transp = 0.25;
  vis3_1->SetTransparency(vis3_1Transp);
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis3_1->GetTransparency()), vis3_1Transp, 1e-5));

  gazebo::rendering::VisualPtr vis3_2;
  vis3_2.reset(new gazebo::rendering::Visual("vis3_2_LEAF", vis2));
  vis3_2->Load();
  double vis3_2Transp = 1.0;
  vis3_2->SetTransparency(vis3_2Transp);
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis3_2->GetTransparency()), vis3_2Transp, 1e-5));

  gazebo::rendering::VisualPtr vis4;
  vis4.reset(new gazebo::rendering::Visual("vis4_LEAF", vis3_1));
  vis4->Load();
  double vis4Transp = 0.9;
  vis4->SetTransparency(vis4Transp);
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis4->GetTransparency()), vis4Transp, 1e-5));

  // Create another model just to align them
  gazebo::rendering::VisualPtr otherVis;
  otherVis.reset(
      new gazebo::rendering::Visual("otherVis", scene->WorldVisual()));
  otherVis->Load();

  // Align preview
  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  modelVisuals.push_back(vis1);
  modelVisuals.push_back(otherVis);

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "last", false);

  this->ProcessEventsAndDraw();

  // Check that the transparency of the leaves have changed
  QVERIFY(!ignition::math::equal(
      static_cast<double>(vis3_2->GetTransparency()), vis3_2Transp, 1e-5));
  QVERIFY(!ignition::math::equal(
      static_cast<double>(vis4->GetTransparency()), vis4Transp, 1e-5));

  // Reset
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "reset", "last", false);

  this->ProcessEventsAndDraw();

  // Check that the transparency of the leaves have the original value
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis3_2->GetTransparency()), vis3_2Transp, 1e-5));
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis4->GetTransparency()), vis4Transp, 1e-5));

  // Align preview again
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "max", "last", false);

  this->ProcessEventsAndDraw();

  // Check that the transparency of the leaves have changed
  QVERIFY(!ignition::math::equal(
      static_cast<double>(vis3_2->GetTransparency()), vis3_2Transp, 1e-5));
  QVERIFY(!ignition::math::equal(
      static_cast<double>(vis4->GetTransparency()), vis4Transp, 1e-5));

  // Publish the position
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "max", "last", true);

  this->ProcessEventsAndDraw();

  // Check that the transparency of the leaves have the original value
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis3_2->GetTransparency()), vis3_2Transp, 1e-5));
  QVERIFY(ignition::math::equal(
      static_cast<double>(vis4->GetTransparency()), vis4Transp, 1e-5));
}

// Generate a main function for the test
QTEST_MAIN(ModelAlign_TEST)
