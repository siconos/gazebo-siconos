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

#ifndef CAMERAVISUAL_PRIVATE_HH
#define CAMERAVISUAL_PRIVATE_HH

#include <vector>

#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    class Camera;
    class VisualPrivate;

    class CameraVisualPrivate : public VisualPrivate
    {
      /// \brief Event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Pointer to the camera.
      public: CameraPtr camera;
    };
  }
}
#endif
