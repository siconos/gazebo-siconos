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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/ModelCreator_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
std::string NestedModelSDFString()
{
  // nested model - two top level links, one joint, and a nested model
  std::stringstream nestedModelSdfStream;
  nestedModelSdfStream << "<sdf version='" << SDF_VERSION << "'>"
      << "<model name ='model_00'>"
      << "  <pose>0 0 1 0 0 0</pose>"
      << "  <link name ='link_00'>"
      << "    <pose>1 0 0 0 0 0</pose>"
      << "    <collision name ='collision_01'>"
      << "      <geometry>"
      << "        <box><size>1 1 1</size></box>"
      << "      </geometry>"
      << "    </collision>"
      << "    <visual name ='visual_01'>"
      << "      <geometry>"
      << "        <box><size>1 1 1</size></box>"
      << "      </geometry>"
      << "    </visual>"
      << "  </link>"
      << "  <link name ='link_01'>"
      << "    <pose>-1 0 0 0 0 0</pose>"
      << "    <collision name ='collision_02'>"
      << "      <geometry>"
      << "        <cylinder>"
      << "          <radius>0.5</radius>"
      << "          <length>1.0</length>"
      << "        </cylinder>"
      << "      </geometry>"
      << "    </collision>"
      << "    <visual name ='visual_02'>"
      << "      <geometry>"
      << "        <cylinder>"
      << "          <radius>0.5</radius>"
      << "          <length>1.0</length>"
      << "        </cylinder>"
      << "      </geometry>"
      << "    </visual>"
      << "  </link>"
      << "  <joint name ='joint_01' type='prismatic'>"
      << "    <pose>0 1 0 0 0 0</pose>"
      << "    <parent>link_00</parent>"
      << "    <child>link_01</child>"
      << "    <axis>"
      << "      <xyz>0 0 1</xyz>"
      << "    </axis>"
      << "  </joint>"
      << "  <model name ='model_01'>"
      << "    <pose>0 1 0 0 0 0</pose>"
      << "    <link name ='link_01'>"
      << "      <pose>1 0 0 0 0 0</pose>"
      << "      <collision name ='collision_01'>"
      << "        <geometry>"
      << "          <box><size>1 1 1</size></box>"
      << "        </geometry>"
      << "      </collision>"
      << "      <visual name ='visual_01'>"
      << "        <geometry>"
      << "          <box><size>1 1 1</size></box>"
      << "        </geometry>"
      << "      </visual>"
      << "     </link>"
      << "  </model>"
      << "</model>"
      << "</sdf>";
  return nestedModelSdfStream.str();
}

/////////////////////////////////////////////////
void ModelCreator_TEST::NestedModel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Create a model creator
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator != NULL);

  // Create a box model and add it to the model creator
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);

  modelCreator->AddEntity(boxModelSDF);

  // Verify it has been added
  gazebo::rendering::VisualPtr boxModelVis =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis != NULL);

  // test loading nested model from sdf
  sdf::ElementPtr modelSDF(new sdf::Element);
  sdf::initFile("model.sdf", modelSDF);
  sdf::readString(NestedModelSDFString(), modelSDF);
  modelCreator->AddEntity(modelSDF);

  // verify the model with joint has been added
  gazebo::rendering::VisualPtr modelVis =
      scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis != NULL);
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis != NULL);
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr model01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::model_01");
  QVERIFY(model01Vis != NULL);

  // remove box model and verify
  modelCreator->RemoveEntity(boxModelVis->Name());
  boxModelVis = scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis == NULL);

  // remove nested model and verify
  modelCreator->RemoveEntity(modelVis->Name());
  modelVis = scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis == NULL);
  link00Vis = scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis == NULL);
  link01Vis = scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis == NULL);
  model01Vis = scene->GetVisual("ModelPreview_0_0::model_00::model_01");
  QVERIFY(model01Vis == NULL);

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::SaveState()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Start never saved
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QCOMPARE(modelCreator->CurrentSaveState(), gui::ModelCreator::NEVER_SAVED);

  // Inserting a link and it still is never saved
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::NEVER_SAVED);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Insert another link to have unsaved changes
  modelCreator->AddShape(gui::ModelCreator::ENTITY_BOX);
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Move a link to have unsaved changes
  gazebo::gui::Events::moveEntity(cylinder->Name(),
      ignition::math::Pose3d(1, 2, 3, 4, 5, 6), true);

  this->ProcessEventsAndDraw(mainWindow);

  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  // Remove a link to have unsaved changes
  modelCreator->RemoveEntity(cylinder->Name());
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::UNSAVED_CHANGES);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->CurrentSaveState(),
      gui::ModelCreator::ALL_SAVED);

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::Selection()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Start never saved
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator);

  // Inserting a few links
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_BOX);
  gazebo::rendering::VisualPtr box =
      scene->GetVisual("ModelPreview_0_0::link_1");
  QVERIFY(box != NULL);

  modelCreator->AddShape(gui::ModelCreator::ENTITY_SPHERE);
  gazebo::rendering::VisualPtr sphere =
      scene->GetVisual("ModelPreview_0_0::link_2");
  QVERIFY(sphere != NULL);

  // verify initial selected state
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!box->GetHighlighted());
  QVERIFY(!sphere->GetHighlighted());

  // select the shapes and verify that they are selected
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());

  modelCreator->SetSelected(box, true);
  QVERIFY(box->GetHighlighted());

  modelCreator->SetSelected(sphere, true);
  QVERIFY(sphere->GetHighlighted());

  // deselect and verify
  modelCreator->SetSelected(cylinder, false);
  QVERIFY(!cylinder->GetHighlighted());

  modelCreator->SetSelected(box, false);
  QVERIFY(!box->GetHighlighted());

  modelCreator->SetSelected(sphere, false);
  QVERIFY(!sphere->GetHighlighted());

  // select one and verify all
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(!box->GetHighlighted());
  QVERIFY(!sphere->GetHighlighted());

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::ModelPlugin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Start never saved
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator);

  // Inserting a few links
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  // add model plugin
  modelCreator->OnAddModelPlugin("test_name", "test_filename",
      "<data>test</data>");
  gazebo::gui::ModelPluginData *modelPluginData =
       modelCreator->ModelPlugin("test_name");
  QVERIFY(modelPluginData != NULL);
  sdf::ElementPtr modelPluginSDF = modelPluginData->modelPluginSDF;
  QCOMPARE(modelPluginSDF->Get<std::string>("name"), std::string("test_name"));
  QCOMPARE(modelPluginSDF->Get<std::string>("filename"),
      std::string("test_filename"));
  QVERIFY(modelPluginSDF->HasElement("data"));
  QCOMPARE(modelPluginSDF->Get<std::string>("data"), std::string("test"));

  // remove the model plugin
  modelCreator->RemoveModelPlugin("test_name");
  modelPluginData = modelCreator->ModelPlugin("test_name");
  QVERIFY(modelPluginData == NULL);

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::NestedModelSelection()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Create a model creator
  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator != NULL);

  // a link
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  // Add various models and links into the editor
  // a box nested model
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);

  modelCreator->AddModel(boxModelSDF);

  /// Verify it has been added
  gazebo::rendering::VisualPtr boxModelVis =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModelVis != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // a more complicated a nested model loaded from from sdf
  sdf::ElementPtr modelSDF(new sdf::Element);
  sdf::initFile("model.sdf", modelSDF);
  sdf::readString(NestedModelSDFString(), modelSDF);
  modelCreator->AddModel(modelSDF);

  this->ProcessEventsAndDraw(mainWindow);

  // verify the model has been added
  gazebo::rendering::VisualPtr modelVis =
      scene->GetVisual("ModelPreview_0_0::model_00");
  QVERIFY(modelVis != NULL);
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_00");
  QVERIFY(link00Vis != NULL);
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr model01Vis =
      scene->GetVisual("ModelPreview_0_0::model_00::model_01");
  QVERIFY(model01Vis != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // verify initial selected state
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  // test selecting links and nested models
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(boxModelVis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(modelVis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(link00Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(link01Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  modelCreator->SetSelected(model01Vis, true);
  QVERIFY(cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(cylinder, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(boxModelVis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(modelVis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(link00Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(link01Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(model01Vis->GetHighlighted());

  modelCreator->SetSelected(model01Vis, false);
  QVERIFY(!cylinder->GetHighlighted());
  QVERIFY(!boxModelVis->GetHighlighted());
  QVERIFY(!modelVis->GetHighlighted());
  QVERIFY(!link00Vis->GetHighlighted());
  QVERIFY(!link01Vis->GetHighlighted());
  QVERIFY(!model01Vis->GetHighlighted());

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelCreator_TEST::CopyPaste()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  gui::ModelCreator *modelCreator = new gui::ModelCreator();
  QVERIFY(modelCreator);

  QVERIFY(gazebo::gui::g_copyAct != NULL);
  QVERIFY(gazebo::gui::g_pasteAct != NULL);
  QVERIFY(gui::g_editModelAct != NULL);

  // switch to editor mode
  gui::g_editModelAct->toggle();

  // Inserting a link and it still is never saved
  modelCreator->AddShape(gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");

  QVERIFY(cylinder != NULL);

  // Add various models and links into the editor
  // a box nested model
  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;
  msgs::Model model;
  model.set_name("box_model");
  msgs::AddBoxLink(model, mass, size);
  sdf::ElementPtr boxModelSDF = msgs::ModelToSDF(model);
  modelCreator->AddModel(boxModelSDF);

  /// Verify it has been added
  gazebo::rendering::VisualPtr boxModel =
      scene->GetVisual("ModelPreview_0_0::box_model");
  QVERIFY(boxModel != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  // copy and paste cylinder link
  modelCreator->SetSelected(cylinder, true);
  QVERIFY(cylinder->GetHighlighted());

  gui::g_copyAct->trigger();

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
    mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Move to center of the screen
  QPoint moveTo(glWidget->width() * 0.5, glWidget->height() * 0.5);
  QTest::mouseMove(glWidget, moveTo, 100);

  gui::g_pasteAct->trigger();
  QCoreApplication::processEvents();

  // Verify there is a clone of the cylinder link
  rendering::VisualPtr cylinderClone =
      scene->GetVisual(cylinder->Name() + "_clone");
  QVERIFY(cylinderClone != NULL);

  // copy and paste box model
  modelCreator->SetSelected(boxModel, true);
  QVERIFY(boxModel->GetHighlighted());

  gui::g_copyAct->trigger();

  // Move to center of the screen
  QTest::mouseMove(glWidget, moveTo, 100);

  gui::g_pasteAct->trigger();
  QCoreApplication::processEvents();

  // Verify there is a clone of the box model
  rendering::VisualPtr boxModelClone =
      scene->GetVisual(boxModel->Name() + "_clone");
  QVERIFY(boxModelClone != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  delete modelCreator;
  modelCreator = NULL;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelCreator_TEST)
