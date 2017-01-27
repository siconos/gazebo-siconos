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

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/model/ModelPluginInspector.hh"
#include "gazebo/gui/model/ModelPluginInspector_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::Buttons()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check buttons
  QList<QPushButton *> buttons = inspector->findChildren<QPushButton *>();
  QCOMPARE(buttons.size(), 2);

  QCOMPARE(buttons[0]->text(), QString::fromStdString("Cancel"));
  QCOMPARE(buttons[1]->text(), QString::fromStdString("OK"));

  // Check pressing Cancel button closes the inspector
  inspector->show();
  QCOMPARE(inspector->isVisible(), true);

  buttons[0]->click();
  QCOMPARE(inspector->isVisible(), false);

  // Check pressing OK button closes the inspector
  inspector->show();
  QCOMPARE(inspector->isVisible(), true);

  buttons[1]->click();
  QCOMPARE(inspector->isVisible(), false);

  // Destructor
  delete inspector;
}

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::Update()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check fields
  QList<QLineEdit *> lineEdits = inspector->findChildren<QLineEdit *>();
  QList<QPlainTextEdit *> plainTextEdits =
      inspector->findChildren<QPlainTextEdit *>();
  QCOMPARE(lineEdits.size(), 2);
  QCOMPARE(plainTextEdits.size(), 1);

  QCOMPARE(lineEdits[0]->text(), QString());
  QCOMPARE(lineEdits[1]->text(), QString());
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString());

  // Create a message
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<xml>1000</xml>\n";

  gazebo::msgs::Plugin pluginMsg;
  pluginMsg.set_name(name);
  pluginMsg.set_filename(filename);
  pluginMsg.set_innerxml(innerxml);
  gazebo::msgs::PluginPtr pluginPtr(new gazebo::msgs::Plugin);
  pluginPtr->CopyFrom(pluginMsg);

  // Update inspector
  inspector->Update(pluginPtr);

  // Check fields
  QCOMPARE(lineEdits[0]->text(), QString::fromStdString(name));
  QCOMPARE(lineEdits[1]->text(), QString::fromStdString(filename));
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString::fromStdString(innerxml));

  // Destructor
  delete inspector;
}

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::RemoveButton()
{
  // Create a modelPlugin inspector
  gazebo::gui::ModelPluginInspector *modelPluginInspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(modelPluginInspector != NULL);

  // Open it
  modelPluginInspector->open();
  QVERIFY(modelPluginInspector->isVisible());

  // Get buttons
  QList<QToolButton *> toolButtons =
      modelPluginInspector->findChildren<QToolButton *>();
  QVERIFY(toolButtons.size() == 1);
  QVERIFY(toolButtons[0]->text() == "");

  // Trigger remove
  toolButtons[0]->click();

  // Check modelPlugin inspector disappeared
  QVERIFY(!modelPluginInspector->isVisible());

  delete modelPluginInspector;
}

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::ReadOnly()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check fields
  QList<QLineEdit *> lineEdits = inspector->findChildren<QLineEdit *>();
  QList<QPlainTextEdit *> plainTextEdits =
      inspector->findChildren<QPlainTextEdit *>();
  QCOMPARE(lineEdits.size(), 2);
  QCOMPARE(plainTextEdits.size(), 1);

  QVERIFY(!lineEdits[0]->isEnabled());
  QVERIFY(!lineEdits[1]->isEnabled());
  QVERIFY(!plainTextEdits[0]->isEnabled());

  inspector->SetReadOnly(false);
  QVERIFY(lineEdits[0]->isEnabled());
  QVERIFY(lineEdits[1]->isEnabled());
  QVERIFY(plainTextEdits[0]->isEnabled());

  // Destructor
  delete inspector;
}

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::Clear()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check fields
  QList<QLineEdit *> lineEdits = inspector->findChildren<QLineEdit *>();
  QList<QPlainTextEdit *> plainTextEdits =
      inspector->findChildren<QPlainTextEdit *>();
  QCOMPARE(lineEdits.size(), 2);
  QCOMPARE(plainTextEdits.size(), 1);

  QCOMPARE(lineEdits[0]->text(), QString());
  QCOMPARE(lineEdits[1]->text(), QString());
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString());

  // set test data
  lineEdits[0]->setText(QString("my_plugin_name"));
  lineEdits[1]->setText(QString("my_plugin_filename"));
  plainTextEdits[0]->setPlainText(QString("<data>test</data>"));

  // verify values
  QCOMPARE(lineEdits[0]->text(), QString("my_plugin_name"));
  QCOMPARE(lineEdits[1]->text(), QString("my_plugin_filename"));
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString("<data>test</data>"));

  // clear and verify values are empty
  inspector->SetReadOnly(false);
  inspector->Clear();

  QCOMPARE(lineEdits[0]->text(), QString());
  QCOMPARE(lineEdits[1]->text(), QString());
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString());

  gazebo::msgs::Plugin *pluginMsg = inspector->Data();
  QVERIFY(pluginMsg->name().empty());
  QVERIFY(pluginMsg->filename().empty());
  QVERIFY(pluginMsg->innerxml().empty());

  // Destructor
  delete inspector;
}


/////////////////////////////////////////////////
void ModelPluginInspector_TEST::GetData()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check fields
  QList<QLineEdit *> lineEdits = inspector->findChildren<QLineEdit *>();
  QList<QPlainTextEdit *> plainTextEdits =
      inspector->findChildren<QPlainTextEdit *>();
  QCOMPARE(lineEdits.size(), 2);
  QCOMPARE(plainTextEdits.size(), 1);

  QCOMPARE(lineEdits[0]->text(), QString());
  QCOMPARE(lineEdits[1]->text(), QString());
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString());

  gazebo::msgs::Plugin *pluginMsg = inspector->Data();
  QVERIFY(pluginMsg->name().empty());
  QVERIFY(pluginMsg->filename().empty());
  QVERIFY(pluginMsg->innerxml().empty());

  // set test data
  lineEdits[0]->setText(QString("my_plugin_name"));
  lineEdits[1]->setText(QString("my_plugin_filename"));
  plainTextEdits[0]->setPlainText(QString("<data>test</data>"));

  // verify values
  QCOMPARE(lineEdits[0]->text(), QString("my_plugin_name"));
  QCOMPARE(lineEdits[1]->text(), QString("my_plugin_filename"));
  QCOMPARE(plainTextEdits[0]->toPlainText(), QString("<data>test</data>"));

  // set read-only to false so plugin msg can be updated with new values
  inspector->SetReadOnly(false);
  pluginMsg = inspector->Data();
  QCOMPARE(pluginMsg->name(), std::string("my_plugin_name"));
  QCOMPARE(pluginMsg->filename(), std::string("my_plugin_filename"));
  QCOMPARE(pluginMsg->innerxml(), std::string("<data>test</data>"));

  // Destructor
  delete inspector;
}

// Generate a main function for the test
QTEST_MAIN(ModelPluginInspector_TEST)
