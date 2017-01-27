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

#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/InertiaVisualPrivate.hh"
#include "gazebo/rendering/InertiaVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
InertiaVisual::InertiaVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new InertiaVisualPrivate, _name, _vis, false)
{
  InertiaVisualPrivate *dPtr =
      reinterpret_cast<InertiaVisualPrivate *>(this->dataPtr);
  dPtr->type = VT_PHYSICS;
}

/////////////////////////////////////////////////
InertiaVisual::~InertiaVisual()
{
}

/////////////////////////////////////////////////
void InertiaVisual::Load(sdf::ElementPtr _elem)
{
  Visual::Load();
  ignition::math::Pose3d pose = _elem->Get<ignition::math::Pose3d>("origin");
  this->Load(pose);
}

/////////////////////////////////////////////////
void InertiaVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  auto inertial = msgs::Convert(_msg->inertial());
  auto xyz = inertial.Pose().Pos();
  auto q = inertial.Pose().Rot();

  // Use ignition::math::MassMatrix3 to compute
  // equivalent box size and rotation
  auto m = inertial.MassMatrix();
  ignition::math::Vector3d boxScale;
  ignition::math::Quaterniond boxRot;
  if (!m.EquivalentBox(boxScale, boxRot))
  {
    // Invalid inertia, load with default scale
    gzlog << "The link " << _msg->name() << " has unrealistic inertia, "
          << "unable to visualize box of equivalent inertia." << std::endl;
    this->Load(ignition::math::Pose3d(xyz, q));
  }
  else
  {
    // Apply additional rotation by boxRot
    this->Load(ignition::math::Pose3d(xyz, q * boxRot), boxScale);
  }
}

/////////////////////////////////////////////////
void InertiaVisual::Load(const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_scale)
{
  InertiaVisualPrivate *dPtr =
      reinterpret_cast<InertiaVisualPrivate *>(this->dataPtr);

  // Inertia position indicator
  ignition::math::Vector3d p1(0, 0, -2*_scale.Z());
  ignition::math::Vector3d p2(0, 0, 2*_scale.Z());
  ignition::math::Vector3d p3(0, -2*_scale.Y(), 0);
  ignition::math::Vector3d p4(0, 2*_scale.Y(), 0);
  ignition::math::Vector3d p5(-2*_scale.X(), 0, 0);
  ignition::math::Vector3d p6(2*_scale.X(), 0, 0);
  p1 = _pose.Rot().RotateVector(p1);
  p2 = _pose.Rot().RotateVector(p2);
  p3 = _pose.Rot().RotateVector(p3);
  p4 = _pose.Rot().RotateVector(p4);
  p5 = _pose.Rot().RotateVector(p5);
  p6 = _pose.Rot().RotateVector(p6);
  p1 += _pose.Pos();
  p2 += _pose.Pos();
  p3 += _pose.Pos();
  p4 += _pose.Pos();
  p5 += _pose.Pos();
  p6 += _pose.Pos();

  dPtr->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/Green");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  VisualPtr boxVis(
      new Visual(this->Name()+"_BOX_", shared_from_this(), false));
  boxVis->Load();

  // Inertia indicator: equivalent box of uniform density
  boxVis->AttachMesh("unit_box");

  boxVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  boxVis->SetMaterial("__GAZEBO_TRANS_PURPLE_MATERIAL__");
  boxVis->SetCastShadows(false);

  boxVis->SetScale(_scale);
  boxVis->SetPosition(_pose.Pos());
  boxVis->SetRotation(_pose.Rot());

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
