/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/terrain/TerrainEditorPalette.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditorPalette::TerrainEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Create the button to raise terrain
  QPushButton *raiseButton = new QPushButton("Raise / Lower", this);
  raiseButton->setStatusTip(tr("Left-mouse press to raise terrain."));
  raiseButton->setCheckable(true);
  raiseButton->setChecked(false);
  connect(raiseButton, SIGNAL(toggled(bool)), this, SLOT(OnRaise(bool)));

  // Create the button to lower terrain
  QPushButton *lowerButton = new QPushButton("Flatten", this);
  lowerButton->setStatusTip(tr("Left-mouse press to flatten terrain."));
  lowerButton->setCheckable(true);
  lowerButton->setChecked(false);
  connect(lowerButton, SIGNAL(toggled(bool)), this, SLOT(OnLower(bool)));

  // Create the button to flatten terrain
  QPushButton *flattenButton = new QPushButton("Smooth", this);
  flattenButton->setStatusTip(tr("Left-mouse press to smooth terrain."));
  flattenButton->setCheckable(true);
  flattenButton->setChecked(false);
  connect(flattenButton, SIGNAL(toggled(bool)), this, SLOT(OnFlatten(bool)));

  // Create the button to roughen terrain
  QPushButton *heightButton = new QPushButton("Pick Height", this);
  heightButton->setStatusTip(
      tr("Left-mouse press to select a terrain height."));
  heightButton->setCheckable(true);
  heightButton->setChecked(false);
  connect(heightButton, SIGNAL(toggled(bool)), this, SLOT(OnPickHeight(bool)));

  QButtonGroup *buttonGroup = new QButtonGroup;
  buttonGroup->addButton(raiseButton);
  buttonGroup->addButton(lowerButton);
  buttonGroup->addButton(flattenButton);
  buttonGroup->addButton(heightButton);

  // Create the layout to hold all the buttons
  QGridLayout *buttonLayout = new QGridLayout;
  buttonLayout->addWidget(raiseButton, 0, 0);
  buttonLayout->addWidget(lowerButton, 0, 1);
  buttonLayout->addWidget(flattenButton, 1, 0);
  buttonLayout->addWidget(heightButton, 1, 1);

  // Add a save button
  QPushButton *saveButton = new QPushButton("Save Image", this);
  saveButton->setStatusTip(tr("Save terrain as a PNG."));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  // Create a slider to control the outer size of the brush
  this->outsideRadiusSlider = new QSlider(this);
  this->outsideRadiusSlider->setRange(1, 1000);
  this->outsideRadiusSlider->setTickInterval(1);
  this->outsideRadiusSlider->setOrientation(Qt::Horizontal);
  this->outsideRadiusSlider->setValue(10);

  this->outsideRadiusSpin = new QDoubleSpinBox(this);
  connect(this->outsideRadiusSlider, SIGNAL(sliderMoved(int)),
      this, SLOT(OnOutsideRadiusSlider(int)));


  // Create a layout to hold the outer brush size slider and its label
  QHBoxLayout *outsideRadiusSpinLayout = new QHBoxLayout;
  outsideRadiusSpinLayout->addWidget(new QLabel(tr("Outside radius: ")));
  outsideRadiusSpinLayout->addStretch(2);
  outsideRadiusSpinLayout->addWidget(this->outsideRadiusSpin);

  QVBoxLayout *outsideRadiusLayout = new QVBoxLayout;
  outsideRadiusLayout->addLayout(outsideRadiusSpinLayout);
  outsideRadiusLayout->addWidget(this->outsideRadiusSlider);


  // Create a slider to control the inner size of the brush
  this->insideRadiusSlider = new QSlider(this);
  this->insideRadiusSlider->setRange(0, 1000);
  this->insideRadiusSlider->setTickInterval(1);
  this->insideRadiusSlider->setOrientation(Qt::Horizontal);
  this->insideRadiusSlider->setValue(20);
  connect(this->insideRadiusSlider, SIGNAL(sliderMoved(int)),
      this, SLOT(OnInsideRadiusSlider(int)));


  this->insideRadiusSpin = new QDoubleSpinBox(this);
  this->insideRadiusSpin->setRange(0, 1.0);

  // Create a layout to hold the inner brush size slider and its label
  QHBoxLayout *insideRadiusSpinLayout = new QHBoxLayout;
  insideRadiusSpinLayout->addWidget(new QLabel(tr("Inside radius: ")));
  insideRadiusSpinLayout->addStretch(2);
  insideRadiusSpinLayout->addWidget(this->insideRadiusSpin);

  QVBoxLayout *insideRadiusLayout = new QVBoxLayout;
  insideRadiusLayout->addLayout(insideRadiusSpinLayout);
  insideRadiusLayout->addWidget(this->insideRadiusSlider);

  // Create a slider to control the weight of the brush
  this->weightSlider = new QSlider(this);
  this->weightSlider->setRange(1, 1000);
  this->weightSlider->setTickInterval(1);
  this->weightSlider->setOrientation(Qt::Horizontal);
  this->weightSlider->setValue(10);
  connect(this->weightSlider, SIGNAL(sliderMoved(int)),
      this, SLOT(OnWeightSlider(int)));


  this->weightSpin = new QDoubleSpinBox(this);
  this->weightSpin->setRange(.01, 1.0);

  // Create a layout to hold the brush weight slider and its label
  QHBoxLayout *weightSpinLayout = new QHBoxLayout;
  weightSpinLayout->addWidget(new QLabel(tr("Weight: ")));
  weightSpinLayout->addStretch(2);
  weightSpinLayout->addWidget(this->weightSpin);

  QVBoxLayout *weightLayout = new QVBoxLayout;
  weightLayout->addLayout(weightSpinLayout);
  weightLayout->addWidget(this->weightSlider);


  // Create a slider to control the weight of the brush
  this->heightSlider = new QSlider(this);
  this->heightSlider->setRange(1, 100);
  this->heightSlider->setTickInterval(1);
  this->heightSlider->setOrientation(Qt::Horizontal);
  this->heightSlider->setValue(10);
  connect(this->heightSlider, SIGNAL(sliderMoved(int)),
      this, SLOT(OnHeightSlider(int)));

  this->heightSpin = new QDoubleSpinBox(this);

  // Create a layout to hold the brush height slider and its label
  QHBoxLayout *heightSpinLayout = new QHBoxLayout;
  heightSpinLayout->addWidget(new QLabel(tr("Height: ")));
  heightSpinLayout->addStretch(2);
  heightSpinLayout->addWidget(this->heightSpin);

  QVBoxLayout *heightLayout = new QVBoxLayout;
  heightLayout->setContentsMargins(0, 0, 0, 0);
  heightLayout->addLayout(heightSpinLayout);
  heightLayout->addWidget(this->heightSlider);


  // Add all the layouts and widgets to the main layout
  mainLayout->addLayout(buttonLayout);
  mainLayout->addLayout(outsideRadiusLayout);
  mainLayout->addLayout(insideRadiusLayout);
  mainLayout->addLayout(weightLayout);
  mainLayout->addLayout(heightLayout);
  mainLayout->addStretch(1);
  mainLayout->addWidget(saveButton);

  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  this->setObjectName("terrainEditorPalette");
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
TerrainEditorPalette::~TerrainEditorPalette()
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnRaise(bool _toggle)
{
  this->SetState(_toggle ? "raise" : std::string());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower(bool _toggle)
{
  this->SetState(_toggle ? "lower" : std::string());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnFlatten(bool _toggle)
{
  this->SetState(_toggle ? "flatten" : std::string());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnPickHeight(bool /*_toggle*/)
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::SetState(const std::string &_state)
{
  if (!_state.empty())
  {
    this->state = _state;

    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
  else
  {
    this->state.clear();

    // Remove the event filters.
    MouseEventHandler::Instance()->RemovePressFilter("terrain");
    MouseEventHandler::Instance()->RemoveMoveFilter("terrain");
  }
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnSave()
{
  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;
  common::Image img = heightmap->GetImage();

  // Get a filename to save to.
  std::string filename = QFileDialog::getSaveFileName(this,
      tr("Save Heightmap"), QString(),
      tr("PNG Files (*.png)")).toStdString();

  // Return if the user has canceled.
  if (filename.empty())
    return;

  img.SavePNG(filename);
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::OnMousePress(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  // Only try to modify if the heightmap exists, and the LEFT mouse button
  // was used.
  if (heightmap && !_event.shift)
  {
    handled = this->Apply(_event, camera, heightmap);
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
  }
  else
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::OnMouseMove(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  if (heightmap && !_event.shift)
  {
    if (_event.dragging)
      handled = this->Apply(_event, camera, heightmap);
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
  }
  else
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::Apply(const common::MouseEvent &_event,
    rendering::CameraPtr _camera, rendering::Heightmap *_heightmap)
{
  bool handled = false;

  // Get the brush weight and size from the sliders.
  double weight = this->weightSpin->value();
  double outsideRadius = this->outsideRadiusSpin->value();

  if (this->state == "lower")
    handled = _heightmap->Lower(_camera, _event.pos, outsideRadius, weight);
  else if (this->state == "raise")
    handled = _heightmap->Raise(_camera, _event.pos, outsideRadius, weight);
  else if (this->state == "flatten")
    handled = _heightmap->Flatten(_camera, _event.pos, outsideRadius, weight);
  else if (this->state == "smooth")
    handled = _heightmap->Smooth(_camera, _event.pos, outsideRadius, weight);

  return handled;
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnOutsideRadiusSlider(int _value)
{
  this->outsideRadiusSpin->setValue(_value / 10.0);
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnInsideRadiusSlider(int _value)
{
  this->insideRadiusSpin->setValue(_value /
      (double)this->insideRadiusSlider->maximum());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnWeightSlider(int _value)
{
  this->weightSpin->setValue(_value / (double)this->weightSlider->maximum());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnHeightSlider(int _value)
{
  this->heightSpin->setValue(_value / 10.0);
}
