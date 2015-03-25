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

#include "gazebo/gui/SaveDialog.hh"
#include "gazebo/gui/SaveDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void SaveDialogTestHelper::CheckFileDialog()
{
  QVERIFY(this->dialog);

  QFileDialog *fileDialog = this->dialog->findChild<QFileDialog *>();
  QVERIFY(fileDialog);

  // set default path to home dir.
  fileDialog->selectFile(QDir::homePath());

  // hit enter to close dialog
  QTest::keyClick(fileDialog, Qt::Key_Enter);

  QVERIFY(!this->dialog->isVisible());
}

/////////////////////////////////////////////////
void SaveDialog_TEST::SaveLocation()
{
  gazebo::gui::SaveDialog *saveDialog = new gazebo::gui::SaveDialog;
  QCoreApplication::processEvents();

  // find the browse button
  QList<QPushButton *> pushButtons = saveDialog->findChildren<QPushButton *>();
  QVERIFY(!pushButtons.empty());
  QPushButton *browseButton = NULL;
  for (int i = 0; i < pushButtons.size(); ++i)
  {
    QPushButton *button = pushButtons[i];
    QVERIFY(button);
    if (button->text().toLower().toStdString() == "browse")
      browseButton = button;
  }
  QVERIFY(browseButton);

  // set a path in the browse file dialog and verify value
  SaveDialogTestHelper helper;
  helper.dialog = saveDialog;
  QTimer::singleShot(0, &helper, SLOT(CheckFileDialog()));
  browseButton->click();
  QVERIFY(saveDialog->GetSaveLocation() == QDir::homePath().toStdString());

  delete saveDialog;
}

// Generate a main function for the test
QTEST_MAIN(SaveDialog_TEST)
