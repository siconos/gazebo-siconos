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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Console.hh"

#include "gazebo/rendering/Material.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/JointCreationDialog.hh"
#include "gazebo/gui/model/JointCreationDialogPrivate.hh"
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointCreationDialog::JointCreationDialog(JointMaker *_jointMaker,
    QWidget *_parent) : QDialog(_parent),
    dataPtr(new JointCreationDialogPrivate)
{
  this->setObjectName("JointCreationDialog");
  this->setWindowTitle(tr("Create Joint"));
  this->setWindowFlags(this->windowFlags() | Qt::WindowStaysOnTopHint);
  this->setModal(false);

  // Dialog size
  this->setMinimumWidth(500);
  this->setMinimumHeight(300);
  int desiredHeight = 940;
  MainWindow *mainWindow = qobject_cast<MainWindow *>(_parent);
  if (mainWindow)
  {
    desiredHeight = std::min(desiredHeight, (mainWindow->height()-100));
  }
  this->resize(500, desiredHeight);

  this->dataPtr->jointMaker = _jointMaker;

  // ConfigWidget
  this->dataPtr->configWidget = new ConfigWidget();

  // Joint types
  QRadioButton *fixedJointRadio = new QRadioButton(tr("Fixed"));
  QRadioButton *prismaticJointRadio = new QRadioButton(tr("Prismatic"));
  QRadioButton *revoluteJointRadio = new QRadioButton(tr("Revolute"));
  QRadioButton *revolute2JointRadio = new QRadioButton(tr("Revolute2"));
  QRadioButton *screwJointRadio = new QRadioButton(tr("Screw"));
  QRadioButton *universalJointRadio = new QRadioButton(tr("Universal"));
  QRadioButton *ballJointRadio = new QRadioButton(tr("Ball"));
  QRadioButton *gearboxJointRadio = new QRadioButton(tr("Gearbox"));

  this->dataPtr->typeButtons = new QButtonGroup();
  this->dataPtr->typeButtons->addButton(fixedJointRadio, 1);
  this->dataPtr->typeButtons->addButton(prismaticJointRadio, 2);
  this->dataPtr->typeButtons->addButton(revoluteJointRadio, 3);
  this->dataPtr->typeButtons->addButton(revolute2JointRadio, 4);
  this->dataPtr->typeButtons->addButton(screwJointRadio, 5);
  this->dataPtr->typeButtons->addButton(universalJointRadio, 6);
  this->dataPtr->typeButtons->addButton(ballJointRadio, 7);
  this->dataPtr->typeButtons->addButton(gearboxJointRadio, 8);
  connect(this->dataPtr->typeButtons, SIGNAL(buttonClicked(int)),
      this->dataPtr->jointMaker, SLOT(OnType(int)));
  connect(this->dataPtr->typeButtons, SIGNAL(buttonClicked(int)),
      this, SLOT(OnType(int)));

  for (auto button : this->dataPtr->typeButtons->buttons())
  {
    button->setStyleSheet("QRadioButton{color : #d0d0d0}");
  }

  // Types layout
  QGridLayout *typesLayout = new QGridLayout();
  typesLayout->addWidget(fixedJointRadio, 0, 0);
  typesLayout->addWidget(revoluteJointRadio, 1, 0);
  typesLayout->addWidget(revolute2JointRadio, 2, 0);
  typesLayout->addWidget(prismaticJointRadio, 3, 0);
  typesLayout->addWidget(screwJointRadio, 0, 1);
  typesLayout->addWidget(universalJointRadio, 1, 1);
  typesLayout->addWidget(ballJointRadio, 2, 1);
  typesLayout->addWidget(gearboxJointRadio, 3, 1);

  // Types group widget
  ConfigChildWidget *typesWidget = new ConfigChildWidget();
  typesWidget->setLayout(typesLayout);

  QWidget *typesGroupWidget =
      this->dataPtr->configWidget->CreateGroupWidget("Joint types",
      typesWidget, 0);

  // Link selections
  this->dataPtr->selectionsText = new QLabel(tr(
      "Click a link in the scene to select parent.\n"
      "Click again to select child."));

  // Parent config
  std::vector<std::string> links;
  this->dataPtr->parentLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("parent", links, 0);
  this->dataPtr->parentLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("active", 1));
  this->dataPtr->configWidget->AddConfigChildWidget("parentCombo",
      this->dataPtr->parentLinkWidget);

  // Resize parent label
  auto parentLabel = this->dataPtr->parentLinkWidget->findChild<QLabel *>();
  parentLabel->setMaximumWidth(50);

  // Add parent icon
  this->dataPtr->parentIcon = new QLabel();
  this->dataPtr->parentIcon->setMinimumWidth(15);
  this->dataPtr->parentIcon->setMaximumHeight(15);
  auto parentLayout = qobject_cast<QHBoxLayout *>(
      this->dataPtr->parentLinkWidget->layout());
  if (parentLayout)
  {
    parentLayout->insertWidget(1, this->dataPtr->parentIcon);
    parentLayout->setAlignment(this->dataPtr->parentIcon, Qt::AlignLeft);
  }

  // Child config
  this->dataPtr->childLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("child", links, 0);
  this->dataPtr->childLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal", 1));
  this->dataPtr->configWidget->AddConfigChildWidget("childCombo",
      this->dataPtr->childLinkWidget);
  this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", true);

  // Resize child label
  auto childLabel = this->dataPtr->childLinkWidget->findChild<QLabel *>();
  childLabel->setMaximumWidth(50);

  // Add child icon
  QPixmap childPix(":/images/child-link.png");
  childPix = childPix.scaled(15, 15);
  auto childIcon = new QLabel();
  childIcon->setPixmap(childPix);
  childIcon->setMaximumWidth(15);
  auto childLayout = qobject_cast<QHBoxLayout *>(
      this->dataPtr->childLinkWidget->layout());
  if (childLayout)
  {
    childLayout->insertWidget(1, childIcon);
    childLayout->setAlignment(childIcon, Qt::AlignLeft);
  }

  // Connect all enum value changes
  QObject::connect(this->dataPtr->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnEnumChanged(const QString &, const QString &)));

  // Swap button
  this->dataPtr->swapButton = new QToolButton();
  this->dataPtr->swapButton->setObjectName("JointCreationSwapButton");
  this->dataPtr->swapButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  this->dataPtr->swapButton->setIcon(
      QPixmap(":/images/swap-parent-child.png"));
  this->dataPtr->swapButton->setFixedSize(QSize(50, 50));
  this->dataPtr->swapButton->setIconSize(QSize(35, 35));
  this->dataPtr->swapButton->setToolTip("Swap parent and child");
  this->dataPtr->swapButton->setStyleSheet(
      "QToolButton\
      {\
        background-color: " + ConfigWidget::bgColors[0] +
      "}");
  connect(this->dataPtr->swapButton, SIGNAL(clicked()), this, SLOT(OnSwap()));

  // Links layout
  QGridLayout *linksLayout = new QGridLayout();
  linksLayout->setContentsMargins(0, 0, 0, 0);
  linksLayout->setSpacing(0);
  linksLayout->addWidget(this->dataPtr->selectionsText, 0, 0, 1, 2);
  linksLayout->addWidget(this->dataPtr->parentLinkWidget, 1, 0);
  linksLayout->addWidget(this->dataPtr->childLinkWidget, 2, 0);
  linksLayout->addWidget(this->dataPtr->swapButton, 1, 1, 2, 1);

  // Links group widget
  ConfigChildWidget *linksWidget = new ConfigChildWidget();
  linksWidget->setLayout(linksLayout);

  QWidget *linksGroupWidget = this->dataPtr->configWidget->CreateGroupWidget(
      "Link Selections", linksWidget, 0);

  // Label when joint has no axes
  this->dataPtr->axis0Widget = new QLabel(
      tr("The joint type chosen has no axes"));

  // Axis1 widget
  this->dataPtr->axis1Widget =
      this->dataPtr->configWidget->CreateVector3dWidget("axis1", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("axis1",
      this->dataPtr->axis1Widget);
  this->dataPtr->configWidget->SetVector3dWidgetValue("axis1",
      ignition::math::Vector3d::UnitX);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis1", true);

  // Axis2 widget
  this->dataPtr->axis2Widget =
      this->dataPtr->configWidget->CreateVector3dWidget("axis2", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("axis2",
      this->dataPtr->axis2Widget);
  this->dataPtr->configWidget->SetVector3dWidgetValue("axis2",
      ignition::math::Vector3d::UnitY);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis2", true);

  // Axis general layout
  auto axisGeneralLayout = new QVBoxLayout();
  axisGeneralLayout->setContentsMargins(0, 0, 0, 0);
  axisGeneralLayout->setSpacing(0);
  axisGeneralLayout->addWidget(this->dataPtr->axis0Widget);
  axisGeneralLayout->addWidget(this->dataPtr->axis1Widget);
  axisGeneralLayout->addWidget(this->dataPtr->axis2Widget);

  ConfigChildWidget *axisGeneralWidget = new ConfigChildWidget();
  axisGeneralWidget->setLayout(axisGeneralLayout);

  QWidget *axisGroupWidget = this->dataPtr->configWidget->CreateGroupWidget(
      "Joint axis", axisGeneralWidget, 0);

  // Axis signals
  connect(this->dataPtr->configWidget,
      SIGNAL(Vector3dValueChanged(const QString,
      const ignition::math::Vector3d)),
      this, SLOT(OnVector3dFromDialog(const QString,
      const ignition::math::Vector3d)));

  // Align widgets
  QLabel *xAlignLabel = new QLabel(tr("X: "));
  xAlignLabel->setMaximumWidth(20);
  QLabel *yAlignLabel = new QLabel(tr("Y: "));
  yAlignLabel->setMaximumWidth(20);
  QLabel *zAlignLabel = new QLabel(tr("Z: "));
  zAlignLabel->setMaximumWidth(20);

  auto xAlignMin = new QToolButton();
  xAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignMin->setIcon(QIcon(":/images/x_min.png"));
  xAlignMin->setToolTip("X Align Min");
  xAlignMin->setCheckable(true);

  auto xAlignCenter = new QToolButton();
  xAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignCenter->setIcon(QIcon(":/images/x_center.png"));
  xAlignCenter->setToolTip("X Align Center");
  xAlignCenter->setCheckable(true);

  auto xAlignMax = new QToolButton();
  xAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignMax->setIcon(QIcon(":/images/x_max.png"));
  xAlignMax->setToolTip("X Align Max");
  xAlignMax->setCheckable(true);

  auto yAlignMin = new QToolButton();
  yAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignMin->setIcon(QIcon(":/images/y_min.png"));
  yAlignMin->setToolTip("Y Align Min");
  yAlignMin->setCheckable(true);

  auto yAlignCenter = new QToolButton();
  yAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignCenter->setIcon(QIcon(":/images/y_center.png"));
  yAlignCenter->setToolTip("Y Align Center");
  yAlignCenter->setCheckable(true);

  auto yAlignMax = new QToolButton();
  yAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignMax->setIcon(QIcon(":/images/y_max.png"));
  yAlignMax->setToolTip("Y Align Max");
  yAlignMax->setCheckable(true);

  auto zAlignMin = new QToolButton();
  zAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignMin->setIcon(QIcon(":/images/z_min.png"));
  zAlignMin->setToolTip("Z Align Min");
  zAlignMin->setCheckable(true);

  auto zAlignCenter = new QToolButton();
  zAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignCenter->setIcon(QIcon(":/images/z_center.png"));
  zAlignCenter->setToolTip("Z Align Center");
  zAlignCenter->setCheckable(true);

  auto zAlignMax = new QToolButton();
  zAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignMax->setIcon(QIcon(":/images/z_max.png"));
  zAlignMax->setToolTip("Z Align Max");
  zAlignMax->setCheckable(true);

  auto alignXButtonGroup = new QButtonGroup();
  alignXButtonGroup->setExclusive(false);
  alignXButtonGroup->addButton(xAlignMin, 0);
  alignXButtonGroup->addButton(xAlignCenter, 1);
  alignXButtonGroup->addButton(xAlignMax, 2);
  connect(alignXButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignXButtonGroup);

  auto alignYButtonGroup = new QButtonGroup();
  alignYButtonGroup->setExclusive(false);
  alignYButtonGroup->addButton(yAlignMin, 0);
  alignYButtonGroup->addButton(yAlignCenter, 1);
  alignYButtonGroup->addButton(yAlignMax, 2);
  connect(alignYButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignYButtonGroup);

  auto alignZButtonGroup = new QButtonGroup();
  alignZButtonGroup->setExclusive(false);
  alignZButtonGroup->addButton(zAlignMin, 0);
  alignZButtonGroup->addButton(zAlignCenter, 1);
  alignZButtonGroup->addButton(zAlignMax, 2);
  connect(alignZButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignZButtonGroup);

  // Reverse checkboxes
  this->dataPtr->reverseXBox = new QCheckBox("Reverse");
  connect(this->dataPtr->reverseXBox, SIGNAL(stateChanged(const int)), this,
      SLOT(OnAlign(const int)));

  this->dataPtr->reverseYBox = new QCheckBox("Reverse");
  connect(this->dataPtr->reverseYBox, SIGNAL(stateChanged(const int)), this,
      SLOT(OnAlign(const int)));

  this->dataPtr->reverseZBox = new QCheckBox("Reverse");
  connect(this->dataPtr->reverseZBox, SIGNAL(stateChanged(const int)), this,
      SLOT(OnAlign(const int)));

  // Target dropdown
  this->dataPtr->alignCombo = new QComboBox();
  this->dataPtr->alignCombo->addItem("Child to Parent", 0);
  this->dataPtr->alignCombo->addItem("Parent to Child", 1);
  this->dataPtr->alignCombo->setToolTip("Choose alignment reference");
  connect(this->dataPtr->alignCombo, SIGNAL(currentIndexChanged(const int)),
      this, SLOT(OnAlign(const int)));

  // Align layout
  QGridLayout *alignLayout = new QGridLayout();
  alignLayout->addWidget(xAlignLabel, 0, 0);
  alignLayout->addWidget(xAlignMin, 0, 1);
  alignLayout->addWidget(xAlignCenter, 0, 2);
  alignLayout->addWidget(xAlignMax, 0, 3);
  alignLayout->addWidget(yAlignLabel, 1, 0);
  alignLayout->addWidget(yAlignMin, 1, 1);
  alignLayout->addWidget(yAlignCenter, 1, 2);
  alignLayout->addWidget(yAlignMax, 1, 3);
  alignLayout->addWidget(zAlignLabel, 2, 0);
  alignLayout->addWidget(zAlignMin, 2, 1);
  alignLayout->addWidget(zAlignCenter, 2, 2);
  alignLayout->addWidget(zAlignMax, 2, 3);
  alignLayout->addWidget(this->dataPtr->reverseXBox, 0, 4, 1, 3);
  alignLayout->addWidget(this->dataPtr->reverseYBox, 1, 4, 1, 3);
  alignLayout->addWidget(this->dataPtr->reverseZBox, 2, 4, 1, 3);
  alignLayout->addWidget(this->dataPtr->alignCombo, 3, 0, 1, 5);
  alignLayout->setAlignment(xAlignLabel, Qt::AlignRight);
  alignLayout->setAlignment(yAlignLabel, Qt::AlignRight);
  alignLayout->setAlignment(zAlignLabel, Qt::AlignRight);

  // Align group widget
  ConfigChildWidget *alignWidget = new ConfigChildWidget();
  alignWidget->setLayout(alignLayout);
  this->dataPtr->configWidget->AddConfigChildWidget("align",
      alignWidget);
  this->dataPtr->configWidget->SetWidgetReadOnly("align", true);

  QWidget *alignGroupWidget =
      this->dataPtr->configWidget->CreateGroupWidget("Align links",
      alignWidget, 0);

  // Joint pose widget
  ConfigChildWidget *jointPoseWidget =
      this->dataPtr->configWidget->CreatePoseWidget("joint_pose", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("joint_pose",
      jointPoseWidget);
  this->dataPtr->configWidget->SetWidgetReadOnly("joint_pose", true);

  QWidget *jointPoseGroupWidget =
      this->dataPtr->configWidget->CreateGroupWidget(
      "Joint Pose", jointPoseWidget, 0);
  jointPoseGroupWidget->setToolTip(
      "Joint pose expressed in the child link frame");

  // Relative pose widget
  ConfigChildWidget *relativePoseWidget =
      this->dataPtr->configWidget->CreatePoseWidget("relative_pose", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("relative_pose",
      relativePoseWidget);
  this->dataPtr->configWidget->SetWidgetReadOnly("relative_pose", true);

  QWidget *relativePoseGroupWidget =
      this->dataPtr->configWidget->CreateGroupWidget(
      "Relative Pose", relativePoseWidget, 0);
  relativePoseGroupWidget->setToolTip(
      "Child link pose expressed in parent link frame");

  // Connect pose widgets signal
  connect(this->dataPtr->configWidget,
      SIGNAL(PoseValueChanged(const QString,
      const ignition::math::Pose3d)),
      this, SLOT(OnPoseFromDialog(const QString,
      const ignition::math::Pose3d)));

  // Spacer
  QWidget *spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Config Widget layout
  QVBoxLayout *configLayout = new QVBoxLayout();
  configLayout->setSpacing(0);
  configLayout->addWidget(typesGroupWidget);
  configLayout->addWidget(linksGroupWidget);
  configLayout->addWidget(axisGroupWidget);
  configLayout->addWidget(alignGroupWidget);
  configLayout->addWidget(jointPoseGroupWidget);
  configLayout->addWidget(relativePoseGroupWidget);
  configLayout->addWidget(spacer);

  this->dataPtr->configWidget->setLayout(configLayout);

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->dataPtr->configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Reset all button
  QPushButton *resetAllButton = new QPushButton(tr("Reset"));
  resetAllButton->setObjectName("JointCreationResetButton");
  resetAllButton->setToolTip("Reset");
  connect(resetAllButton, SIGNAL(clicked()), this, SLOT(OnResetAll()));

  // Cancel button
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  cancelButton->setObjectName("JointCreationCancelButton");
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  // Create button
  this->dataPtr->createButton = new QPushButton(tr("Create"));
  this->dataPtr->createButton->setObjectName("JointCreationCreateButton");
  this->dataPtr->createButton->setEnabled(false);
  connect(this->dataPtr->createButton, SIGNAL(clicked()), this,
      SLOT(OnCreate()));

  // Buttons layout
  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(resetAllButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(this->dataPtr->createButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  // Qt signal-slot connections
  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

  // Initialize variables
  this->dataPtr->validLinks = false;
  this->dataPtr->validAxis1 = true;
  this->dataPtr->validAxis2 = true;
}

/////////////////////////////////////////////////
JointCreationDialog::~JointCreationDialog()
{
}

/////////////////////////////////////////////////
void JointCreationDialog::Open(const JointMaker::JointType _type)
{
  if (!this->dataPtr->jointMaker)
  {
    gzerr << "Joint maker not found, can't open joint creation dialog."
        << std::endl;
    this->OnCancel();
  }

  // Check joint type
  this->dataPtr->typeButtons->button(static_cast<int>(_type))
      ->setChecked(true);
  this->OnType(_type);

  // Clear links
  this->dataPtr->configWidget->ClearEnumWidget("parentCombo");
  this->dataPtr->configWidget->ClearEnumWidget("childCombo");

  // Add an empty option to each
  this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", "");
  this->dataPtr->configWidget->AddItemEnumWidget("childCombo", "");

  // Fill with all existing links
  for (auto link : this->dataPtr->jointMaker->LinkList())
  {
    this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", link.second);
    this->dataPtr->configWidget->AddItemEnumWidget("childCombo", link.second);
  }

  // Reset fields
  this->OnResetAll();

  // Reset enabled states
  this->dataPtr->createButton->setEnabled(false);
  this->dataPtr->swapButton->setEnabled(false);
  this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis1", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis2", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("align", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("joint_pose", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("relative_pose",
      true);
  this->dataPtr->parentLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("active", 1));
  this->dataPtr->childLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal", 1));
  this->dataPtr->axis1Widget->setStyleSheet(
      ConfigWidget::StyleSheet("normal", 1));
  this->dataPtr->axis2Widget->setStyleSheet(
      ConfigWidget::StyleSheet("normal", 1));

  if (this->parentWidget())
    this->move(this->parentWidget()->pos());
  this->show();
  QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
}

/////////////////////////////////////////////////
void JointCreationDialog::OnLinkFromDialog()
{
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  // Notify so 3D is updated
  if (currentParent != currentChild)
  {
    if (currentParent != "")
      this->dataPtr->jointMaker->SetParentLink(currentParent);
    if (currentChild != "")
      this->dataPtr->jointMaker->SetChildLink(currentChild);
  }

  if (currentParent != "")
    this->OnParentImpl(currentParent);

  if (currentChild != "")
    this->OnChildImpl(currentChild);

  this->UncheckAllAlign();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnPoseFromDialog(const QString &_name,
    const ignition::math::Pose3d &_pose)
{
  if (_name == "relative_pose")
    this->dataPtr->jointMaker->SetLinksRelativePose(_pose, false);
  if (_name == "joint_pose")
    this->dataPtr->jointMaker->SetJointPose(_pose);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnVector3dFromDialog(const QString &_name,
    const ignition::math::Vector3d &_value)
{
  if (_value == ignition::math::Vector3d::Zero)
  {
    if (_name == "axis1")
    {
      this->dataPtr->axis1Widget->setStyleSheet(
          ConfigWidget::StyleSheet("warning", 1));
      this->dataPtr->validAxis1 = false;
    }
    else if (_name == "axis2")
    {
      this->dataPtr->axis2Widget->setStyleSheet(
          ConfigWidget::StyleSheet("warning", 1));
      this->dataPtr->validAxis2 = false;
    }
  }
  else
  {
    if (_name == "axis1")
    {
      this->dataPtr->axis1Widget->setStyleSheet(
          ConfigWidget::StyleSheet("normal", 1));
      this->dataPtr->validAxis1 = true;
    }
    else if (_name == "axis2")
    {
      this->dataPtr->axis2Widget->setStyleSheet(
          ConfigWidget::StyleSheet("normal", 1));
      this->dataPtr->validAxis2 = true;
    }
  }

  if (this->CheckValid())
    this->dataPtr->jointMaker->SetAxis(_name.toStdString(), _value);
}

/////////////////////////////////////////////////
void JointCreationDialog::SetParent(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for parent" << std::endl;
    return;
  }

  std::string unscopedName = _linkName;
  size_t idx = _linkName.find("::");
  if (idx != std::string::npos)
    unscopedName = _linkName.substr(idx+2);

  this->dataPtr->configWidget->blockSignals(true);
  if (!this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo",
      unscopedName))
  {
    gzerr << "Requested link [" << unscopedName << "] not found" << std::endl;
    return;
  }
  this->dataPtr->configWidget->blockSignals(false);

  this->OnParentImpl(_linkName);
}

/////////////////////////////////////////////////
void JointCreationDialog::SetChild(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for child" << std::endl;
    return;
  }

  if (this->dataPtr->configWidget->WidgetReadOnly("childCombo"))
  {
    gzerr << "It's not possible to set child yet." << std::endl;
    return;
  }

  // Update child combo box
  std::string unscopedName = _linkName;
  size_t idx = _linkName.find("::");
  if (idx != std::string::npos)
    unscopedName = _linkName.substr(idx+2);

  this->dataPtr->configWidget->blockSignals(true);
  if (!this->dataPtr->configWidget->SetEnumWidgetValue("childCombo",
      unscopedName))
  {
    gzerr << "Requested link [" << unscopedName << "] not found" << std::endl;
    return;
  }
  this->dataPtr->configWidget->blockSignals(false);

  this->OnChildImpl(_linkName);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnParentImpl(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for parent" << std::endl;
    return;
  }

  // Remove empty option
  this->dataPtr->configWidget->RemoveItemEnumWidget("parentCombo", "");

  // Check if links are valid
  this->CheckLinksValid();

  // Enable child selection
  if (this->dataPtr->configWidget->WidgetReadOnly("childCombo"))
  {
    this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", false);
    this->dataPtr->childLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("active", 1));
  }
}

/////////////////////////////////////////////////
void JointCreationDialog::OnChildImpl(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for child" << std::endl;
    return;
  }

  // Enable features
  this->dataPtr->createButton->setEnabled(true);
  this->dataPtr->swapButton->setEnabled(true);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis1", false);
  this->dataPtr->configWidget->SetWidgetReadOnly("axis2", false);
  this->dataPtr->configWidget->SetWidgetReadOnly("joint_pose", false);
  this->dataPtr->configWidget->SetWidgetReadOnly("align", false);
  this->dataPtr->configWidget->SetWidgetReadOnly("relative_pose", false);

  // Remove empty option
  this->dataPtr->configWidget->RemoveItemEnumWidget("childCombo", "");

  // Hide selection text
  this->dataPtr->selectionsText->setVisible(false);
  QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  // Check if links are valid
  this->CheckLinksValid();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCancel()
{
  this->close();
  this->dataPtr->jointMaker->Stop();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCreate()
{
  this->accept();
  this->dataPtr->jointMaker->FinalizeCreation();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnSwap()
{
  // Get current values
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  // Choose new values
  this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo", currentChild);
  this->dataPtr->configWidget->SetEnumWidgetValue("childCombo", currentParent);
}

/////////////////////////////////////////////////
void JointCreationDialog::UpdateRelativePose(
    const ignition::math::Pose3d &_pose)
{
  this->dataPtr->configWidget->SetPoseWidgetValue("relative_pose", _pose);

  if (this->dataPtr->alignPending)
  {
    this->dataPtr->alignPending = false;
    return;
  }

  this->UncheckAllAlign(true);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnResetAll()
{
  this->dataPtr->jointMaker->SetLinksRelativePose(ignition::math::Pose3d::Zero,
      true);
  this->dataPtr->configWidget->SetVector3dWidgetValue("axis1",
      ignition::math::Vector3d::UnitX);
  this->dataPtr->configWidget->SetVector3dWidgetValue("axis2",
      ignition::math::Vector3d::UnitY);

  this->dataPtr->configWidget->SetPoseWidgetValue("joint_pose",
      ignition::math::Pose3d::Zero);
  this->OnPoseFromDialog("joint_pose", ignition::math::Pose3d::Zero);

  this->UncheckAllAlign();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnEnumChanged(const QString &_name,
    const QString &/*_value*/)
{
  if (_name == "parentCombo" || _name == "childCombo")
    this->OnLinkFromDialog();
}

/////////////////////////////////////////////////
void JointCreationDialog::CheckLinksValid()
{
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  // Warning if parent is the same as the child and don't allow creation
  if (currentParent == currentChild)
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("warning", 1));
    this->dataPtr->childLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("warning", 1));
    this->dataPtr->validLinks = false;
  }
  else
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("normal", 1));
    if (!this->dataPtr->configWidget->WidgetReadOnly("childCombo"))
    {
      this->dataPtr->childLinkWidget->setStyleSheet(
          ConfigWidget::StyleSheet("normal", 1));
      this->dataPtr->validLinks = true;
    }
  }

  // Disable align and relative_pose widgets if parent or child link
  // is a nested link
  // TODO: re-enable once pose of nested model links can be updated
  if (currentParent.find("::") != std::string::npos ||
      currentChild.find("::") != std::string::npos)
  {
    this->dataPtr->configWidget->SetWidgetReadOnly("align", true);
    this->dataPtr->configWidget->SetWidgetReadOnly("relative_pose", true);
    gzwarn << "Pose updates for nested model links are not yet supported. "
        << "The Align and Relative Pose widgets will be disabled for now."
        << std::endl;
  }

  this->CheckValid();
}

/////////////////////////////////////////////////
bool JointCreationDialog::CheckValid()
{
  bool valid = this->dataPtr->validLinks && this->dataPtr->validAxis1 &&
      this->dataPtr->validAxis2;

  if (this->dataPtr->createButton)
    this->dataPtr->createButton->setEnabled(valid);

  return valid;
}

/////////////////////////////////////////////////
void JointCreationDialog::OnAlign(const int _int)
{
  // Button groups
  std::vector<std::string> axes = {"x", "y", "z"};
  std::vector<std::string> configs = {"min", "center", "max"};

  // Find out which axis was changed
  QButtonGroup *group = NULL;
  unsigned int g;
  QObject *senderObject = this->sender();
  if (senderObject == this->dataPtr->reverseXBox)
  {
    g = 0;
    group = this->dataPtr->alignGroups[g];
  }
  else if (senderObject == this->dataPtr->reverseYBox)
  {
    g = 1;
    group = this->dataPtr->alignGroups[g];
  }
  else if (senderObject == this->dataPtr->reverseZBox)
  {
    g = 2;
    group = this->dataPtr->alignGroups[g];
  }
  else
  {
    for (g = 0; g < this->dataPtr->alignGroups.size(); ++g)
    {
      group = this->dataPtr->alignGroups[g];

      if (senderObject == group)
      {
        break;
      }
    }
  }

  // When changing target, reset everything
  if (senderObject == this->dataPtr->alignCombo)
  {
    this->dataPtr->jointMaker->SetLinksRelativePose(
        ignition::math::Pose3d::Zero, true);
  }
  // Reset only the axis which was changed
  else
  {
    this->dataPtr->jointMaker->SetLinksRelativePose(
        ignition::math::Pose3d::Zero, false, g);
  }

  // Uncheck other buttons in the same group
  if (group && qobject_cast<QButtonGroup *>(senderObject))
  {
    for (int i = 0; i < group->buttons().size(); ++i)
    {
      if (i != _int)
      {
        group->buttons()[i]->setChecked(false);
      }
    }
  }

  // Reference link
  bool childToParent = this->dataPtr->alignCombo->currentIndex() == 0;

  // Direction
  std::vector<bool> reverse;
  reverse.push_back(this->dataPtr->reverseXBox->isChecked());
  reverse.push_back(this->dataPtr->reverseYBox->isChecked());
  reverse.push_back(this->dataPtr->reverseZBox->isChecked());

  // Go through all groups and align
  for (g = 0; g < this->dataPtr->alignGroups.size(); ++g)
  {
    group = this->dataPtr->alignGroups[g];

    // Align if there is a checked button
    int checked = group->checkedId();
    if (checked >= 0 && checked <=2)
    {
      // Align
      this->dataPtr->jointMaker->AlignLinks(childToParent, axes[g],
          configs[checked], reverse[g]);
    }
  }

  this->dataPtr->alignPending = true;
}

/////////////////////////////////////////////////
void JointCreationDialog::UncheckAllAlign(const bool _blockSignals)
{
  for (auto group : this->dataPtr->alignGroups)
  {
    for (auto button : group->buttons())
    {
      bool blocked = button->signalsBlocked();
      button->blockSignals(_blockSignals);
      button->setChecked(false);
      button->blockSignals(blocked);
    }
  }

  bool blocked = this->dataPtr->reverseXBox->signalsBlocked();
  this->dataPtr->reverseXBox->blockSignals(_blockSignals);
  this->dataPtr->reverseXBox->setChecked(false);
  this->dataPtr->reverseXBox->blockSignals(blocked);

  blocked = this->dataPtr->reverseYBox->signalsBlocked();
  this->dataPtr->reverseYBox->blockSignals(_blockSignals);
  this->dataPtr->reverseYBox->setChecked(false);
  this->dataPtr->reverseYBox->blockSignals(blocked);

  blocked = this->dataPtr->reverseZBox->signalsBlocked();
  this->dataPtr->reverseZBox->blockSignals(_blockSignals);
  this->dataPtr->reverseZBox->setChecked(false);
  this->dataPtr->reverseZBox->blockSignals(blocked);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnType(const int _typeInt)
{
  auto type = static_cast<JointMaker::JointType>(_typeInt);
  unsigned int axisCount = JointMaker::JointAxisCount(type);

  // Display correct number of axes for this type
  this->dataPtr->axis0Widget->setVisible(axisCount == 0);
  this->dataPtr->axis1Widget->setVisible(axisCount > 0);
  this->dataPtr->axis2Widget->setVisible(axisCount > 1);

  // Change parent icon color according to type
  common::Color matAmbient, matDiffuse, matSpecular, matEmissive;
  rendering::Material::GetMaterialAsColor(
      this->dataPtr->jointMaker->jointMaterials[type],
      matAmbient, matDiffuse, matSpecular, matEmissive);

  std::ostringstream sheet;
  sheet << "QLabel{background-color: rgb(" <<
          (matAmbient[0] * 255) << ", " <<
          (matAmbient[1] * 255) << ", " <<
          (matAmbient[2] * 255) << "); }";

  this->dataPtr->parentIcon->setStyleSheet(QString::fromStdString(sheet.str()));
}

/////////////////////////////////////////////////
void JointCreationDialog::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Enter)
    _event->accept();
  else
    QDialog::keyPressEvent(_event);
}
