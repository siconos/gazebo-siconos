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
#include "gazebo/physics/SphereShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SphereShape::SphereShape(CollisionPtr _parent)
  : Shape(_parent)
{
  this->AddType(Base::SPHERE_SHAPE);
}

//////////////////////////////////////////////////
SphereShape::~SphereShape()
{
}

//////////////////////////////////////////////////
void SphereShape::Init()
{
  this->SetRadius(this->sdf->Get<double>("radius"));
}

//////////////////////////////////////////////////
void SphereShape::SetRadius(double _radius)
{
  this->sdf->GetElement("radius")->Set(_radius);
}

//////////////////////////////////////////////////
double SphereShape::GetRadius() const
{
  return this->sdf->Get<double>("radius");
}

//////////////////////////////////////////////////
void SphereShape::SetScale(const ignition::math::Vector3d &_scale)
{
  if (_scale.X() < 0 || _scale.Y() < 0 || _scale.Z() < 0)
    return;

  if (_scale == this->scale)
    return;

  double newRadius = _scale.Max();
  double oldRadius = this->scale.Max();

  this->SetRadius((newRadius/oldRadius)*this->GetRadius());

  this->scale = _scale;
}

//////////////////////////////////////////////////
void SphereShape::FillMsg(msgs::Geometry &_msg)
{
  _msg.set_type(msgs::Geometry::SPHERE);
  _msg.mutable_sphere()->set_radius(this->GetRadius());
}

//////////////////////////////////////////////////
void SphereShape::ProcessMsg(const msgs::Geometry &_msg)
{
  this->SetRadius(_msg.sphere().radius());
}

//////////////////////////////////////////////////
double SphereShape::ComputeVolume() const
{
  return IGN_SPHERE_VOLUME(this->GetRadius());
}
