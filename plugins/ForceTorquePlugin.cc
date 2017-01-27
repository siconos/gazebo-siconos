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

#include <functional>

#include "plugins/ForceTorquePlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(ForceTorquePlugin)

/////////////////////////////////////////////////
ForceTorquePlugin::ForceTorquePlugin()
{
}

/////////////////////////////////////////////////
ForceTorquePlugin::~ForceTorquePlugin()
{
  this->connection.reset();
  this->parentSensor.reset();
}

/////////////////////////////////////////////////
void ForceTorquePlugin::Load(sensors::SensorPtr _parent,
    sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(_parent);

  if (!this->parentSensor)
  {
    gzerr << "ForceTorquePlugin requires a force_torque "
          << "sensor as its parent.\n";
    return;
  }

  this->connection = this->parentSensor->ConnectUpdate(
        std::bind(&ForceTorquePlugin::OnUpdate, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void ForceTorquePlugin::OnUpdate(msgs::WrenchStamped /*_msg*/)
{
  // overload with useful callback here
}
