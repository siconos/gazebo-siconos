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
#ifndef GAZEBO_RENDERING_FPSVIEWCONTROLLER_HH_
#define GAZEBO_RENDERING_FPSVIEWCONTROLLER_HH_

#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/ViewController.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class FPSViewController FPSViewController.hh rendering/rendering.hh
    /// \brief First Person Shooter style view controller
    class GZ_RENDERING_VISIBLE FPSViewController : public ViewController
    {
      /// \brief Constructor
      /// \param[in] Camera to controll
      public: explicit FPSViewController(UserCameraPtr _camera);

      /// \brief Destructor
      public: virtual ~FPSViewController();

      /// \brief Initialize the controller
      public: virtual void Init();
      using ViewController::Init;

      /// \brief Update the camera position
      public: virtual void Update();

      /// \brief Get the type name of this view controller
      /// \return The name of the controller type: "fps"
      public: static std::string GetTypeString();

      // Documentation inherited from parent
      public: virtual void HandleMouseEvent(const common::MouseEvent &_event);

      // Documentation inherited from parent
      public: void HandleKeyReleaseEvent(const std::string &_key);

      // Documentation inherited from parent
      public: void HandleKeyPressEvent(const std::string &_key);

      /// \brief Translation velocity factor along the x-axis
      private: float xVelocityFactor;

      /// \brief Translation velocity factor along the y-axis
      private: float yVelocityFactor;

      /// \brief Translation velocity along the x-axis
      private: ignition::math::Vector3d xVelocity;

      /// \brief Translation velocity along the y-axis
      private: ignition::math::Vector3d yVelocity;
    };
    /// \}
  }
}
#endif
