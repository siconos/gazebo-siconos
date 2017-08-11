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
#ifndef _GAZEBO_SENSORS_WIDEANGLECAMERASENSOR_HH_
#define _GAZEBO_SENSORS_WIDEANGLECAMERASENSOR_HH_

#include <memory>
#include <string>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class
    class WideAngleCameraSensorPrivate;

    /// \addtogroup gazebo_sensors Sensors
    /// \{

    /// \class WideAngleCameraSensor WideAngleCameraSensor.hh sensors/sensors.hh
    /// \brief Camera sensor with variable mapping function
    ///
    /// This sensor is used for simulating cameras with wide-angle lens
    class GZ_SENSORS_VISIBLE WideAngleCameraSensor : public CameraSensor
    {
      /// \brief Constructor
      public: WideAngleCameraSensor();

      /// \brief Destructor
      public: virtual ~WideAngleCameraSensor();

      // Documentation inherited
      public: void Load(const std::string &_worldName) override;

      // Documentation inherited
      public: void Init() override;

      // Documentation inherited
      protected: void Fini() override;

      // Documentation inherited
      protected: bool UpdateImpl(const bool _force) override;

      /// \brief Handle incoming control message
      /// \param[in] _msg Message received from topic
      protected: void OnCtrlMessage(ConstCameraLensPtr &_msg);

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<WideAngleCameraSensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
