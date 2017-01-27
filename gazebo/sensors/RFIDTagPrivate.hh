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
#ifndef _GAZEBO_SENSORS_RFIDTAG_PRIVATE_HH_
#define _GAZEBO_SENSORS_RFIDTAG_PRIVATE_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief RFID tag private data.
    class RFIDTagPrivate
    {
      /// \brief Pointer the entity that has the RFID tag.
      public: physics::EntityPtr entity;

      /// \brief Publisher for tag pose messages.
      public: transport::PublisherPtr scanPub;
    };
  }
}
#endif
