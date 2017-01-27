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
#include "gazebo/math/RotationSpline.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;
using namespace math;

RotationSpline::RotationSpline()
: autoCalc(true)
{
}

RotationSpline::~RotationSpline()
{
}

void RotationSpline::AddPoint(const Quaternion &_p)
{
  this->points.push_back(_p);
  if (this->autoCalc)
    this->RecalcTangents();
}

Quaternion RotationSpline::Interpolate(double _t, bool _useShortestPath)
{
  // Work out which segment this is in
  double fSeg = _t * (this->points.size() - 1);
  unsigned int segIdx = (unsigned int)fSeg;

  // Apportion t
  _t = fSeg - segIdx;

  return this->Interpolate(segIdx, _t, _useShortestPath);
}

Quaternion RotationSpline::Interpolate(unsigned int _fromIndex, double _t,
                                       bool _useShortestPath)
{
  // Bounds check
  assert (_fromIndex < this->points.size() && "fromIndex out of bounds");

  if ((_fromIndex + 1) == this->points.size())
  {
    // Duff request, cannot blend to nothing
    // Just return source
    return this->points[_fromIndex];
  }

  // Fast special cases
  if (math::equal(_t, 0.0))
    return this->points[_fromIndex];
  else if (math::equal(_t, 1.0))
    return this->points[_fromIndex + 1];

  // double interpolation
  // Use squad using tangents we've already set up
  Quaternion &p = this->points[_fromIndex];
  Quaternion &q = this->points[_fromIndex+1];
  Quaternion &a = this->tangents[_fromIndex];
  Quaternion &b = this->tangents[_fromIndex+1];

  // NB interpolate to nearest rotation
  return Quaternion::Squad(_t, p, a, b, q, _useShortestPath);
}

void RotationSpline::RecalcTangents()
{
  // ShoeMake (1987) approach
  // Just like Catmull-Rom really, just more gnarly
  // And no, I don't understand how to derive this!
  //
  // let p = point[i], pInv = p.Inverse
  // tangent[i] = p * exp(-0.25 *
  // (log(pInv * point[i+1]) + log(pInv * point[i-1])))
  //
  // Assume endpoint tangents are parallel with line with neighbour

  unsigned int i, numPoints;
  bool isClosed;

  numPoints = this->points.size();

  if (numPoints < 2)
  {
    // Can't do anything yet
    return;
  }

  this->tangents.resize(numPoints);

  if (this->points[0] == this->points[numPoints-1])
    isClosed = true;
  else
    isClosed = false;

  Quaternion invp, part1, part2, preExp;
  for (i = 0; i < numPoints; ++i)
  {
    Quaternion &p = this->points[i];
    invp = p.GetInverse();

    if (i == 0)
    {
      // special case start
      part1 = (invp * this->points[i+1]).GetLog();
      if (isClosed)
      {
        // Use numPoints-2 since numPoints-1 == end == start == this one
        part2 = (invp * this->points[numPoints-2]).GetLog();
      }
      else
      {
        part2 = (invp * p).GetLog();
      }
    }
    else if (i == numPoints-1)
    {
      // special case end
      if (isClosed)
      {
        // Wrap to [1] (not [0], this is the same as end == this one)
        part1 = (invp * this->points[1]).GetLog();
      }
      else
      {
        part1 = (invp * p).GetLog();
      }
      part2 = (invp * this->points[i-1]).GetLog();
    }
    else
    {
      part1 = (invp * this->points[i+1]).GetLog();
      part2 = (invp * this->points[i-1]).GetLog();
    }

    preExp = (part1 + part2) * -0.25;
    this->tangents[i] = p * preExp.GetExp();
  }
}

const Quaternion& RotationSpline::GetPoint(unsigned int _index) const
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");

  return this->points[_index];
}

unsigned int RotationSpline::GetNumPoints() const
{
  return this->points.size();
}

void RotationSpline::Clear()
{
  this->points.clear();
  this->tangents.clear();
}

void RotationSpline::UpdatePoint(unsigned int _index,
                                 const Quaternion &_value)
{
  assert (_index < this->points.size() && "Point index is out of bounds!!");

  this->points[_index] = _value;
  if (this->autoCalc)
    this->RecalcTangents();
}

void RotationSpline::SetAutoCalculate(bool _autoCalc)
{
  this->autoCalc = _autoCalc;
}


