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

#ifndef _RFIDVISUAL_PRIVATE_HH_
#define _RFIDVISUAL_PRIVATE_HH_

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the RFID Visual class.
    class RFIDVisualPrivate : public VisualPrivate
    {
      /// \brief Pointer to the transport::Node for communication
      public: transport::NodePtr node;

      /// \brief Pointer to the transport::Subscriber for recieving data
      public: transport::SubscriberPtr rfidSub;
    };
  }
}
#endif
