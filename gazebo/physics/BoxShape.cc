/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/BoxShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BoxShape::BoxShape(CollisionPtr _parent) : Shape(_parent)
{
  this->AddType(Base::BOX_SHAPE);
}

//////////////////////////////////////////////////
BoxShape::~BoxShape()
{
}

//////////////////////////////////////////////////
void BoxShape::Init()
{
  this->SetSize(this->sdf->Get<ignition::math::Vector3d>("size"));
}

//////////////////////////////////////////////////
void BoxShape::SetSize(const ignition::math::Vector3d &_size)
{
  this->sdf->GetElement("size")->Set(_size);
}

//////////////////////////////////////////////////
ignition::math::Vector3d BoxShape::Size() const
{
  return this->sdf->Get<ignition::math::Vector3d>("size");
}

//////////////////////////////////////////////////
void BoxShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (_scale.X() < 0 || _scale.Y() < 0 || _scale.Z() < 0)
    return;

  if (_scale == this->scale)
    return;

  this->SetSize((_scale/this->scale)*this->Size());

  this->scale = _scale;
}

//////////////////////////////////////////////////
void BoxShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::BOX);
  msgs::Set(_msg.mutable_box()->mutable_size(), this->Size());
}

//////////////////////////////////////////////////
void BoxShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetSize(msgs::ConvertIgn(_msg.box().size()));
}

//////////////////////////////////////////////////
double BoxShape::ComputeVolume() const
{
  return IGN_BOX_VOLUME_V(this->Size());
}
