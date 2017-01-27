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

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/ViewController.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
ViewController::ViewController(UserCameraPtr _cam)
  : camera(_cam), enabled(true)
{
}

//////////////////////////////////////////////////
ViewController::~ViewController()
{
}

//////////////////////////////////////////////////
void ViewController::Init(const ignition::math::Vector3d &/*_focalPoint*/,
    const double /*_yaw*/, const double /*_pitch*/)
{
}

//////////////////////////////////////////////////
std::string ViewController::GetTypeString() const
{
  return this->typeString;
}

//////////////////////////////////////////////////
void ViewController::SetEnabled(bool _value)
{
  this->enabled = _value;
}

//////////////////////////////////////////////////
void ViewController::Resize(const unsigned int /*_width*/,
                            const unsigned int /*_height*/)
{
}
