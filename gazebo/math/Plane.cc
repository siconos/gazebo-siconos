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
#include "gazebo/math/Plane.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;
using namespace math;

/////////////////////////////////////////////////
Plane::Plane()
{
  this->d = 0.0;
}

/////////////////////////////////////////////////
Plane::Plane(const Vector3 &_normal, double _offset)
{
  this->normal = _normal;
  this->d = _offset;
}

/////////////////////////////////////////////////
Plane::Plane(const Vector3 &_normal, const Vector2d &_size, double _offset)
{
  this->Set(_normal, _size, _offset);
}

/////////////////////////////////////////////////
Plane::Plane(const ignition::math::Planed &_plane)
{
  this->Set(_plane.Normal(), _plane.Size(), _plane.Offset());
}

/////////////////////////////////////////////////
Plane::~Plane()
{
}

/////////////////////////////////////////////////
void Plane::Set(const Vector3 &_n, const Vector2d &_s, double _offset)
{
  this->normal = _n;
  this->size = _s;
  this->d = _offset;
}

//////////////////////////////////////////////////
Plane &Plane::operator =(const Plane & _p)
{
  this->normal = _p.normal;
  this->size = _p.size;
  this->d = _p.d;

  return *this;
}

//////////////////////////////////////////////////
Plane &Plane::operator=(const ignition::math::Planed &_p)
{
  this->normal = _p.Normal();
  this->size = _p.Size();
  this->d = _p.Offset();

  return *this;
}

//////////////////////////////////////////////////
double Plane::Distance(const Vector3 &_origin, const Vector3 &_dir) const
{
  double denom = this->normal.Dot(_dir);

  if (fabs(denom) < 1e-3)
  {
    // parallel
    return 0;
  }
  else
  {
    double nom = _origin.Dot(this->normal) - this->d;
    double t = -(nom/denom);
    return t;
  }
}

//////////////////////////////////////////////////
ignition::math::Planed Plane::Ign() const
{
  return ignition::math::Planed(this->normal.Ign(), this->size.Ign(), this->d);
}
