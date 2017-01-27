/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_USERCAMERA_PRIVATE_HH_
#define GAZEBO_RENDERING_USERCAMERA_PRIVATE_HH_

#include <string>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the UserCamera class
    class UserCameraPrivate
    {
      /// \brief Gazebo communication node pointer.
      public: transport::NodePtr node;

      /// \brief Subscribes to relative joystick messages.
      public: transport::SubscriberPtr joySubTwist;

      /// \brief Subscribes to absolute joystick messages.
      public: transport::SubscriberPtr joySubPose;

      /// \brief The currently active view controller.
      public: ViewController *viewController;

      /// \brief The previously used view controller.
      public: std::string prevViewControllerName;

      /// \brief An orbit view controller.
      public: OrbitViewController *orbitViewController;

      /// \brief An orthographic view controller.
      public: OrthoViewController *orthoViewController;

      /// \brief A FPS view controller.
      public: FPSViewController *fpsViewController;

      /// \brief Draws a 3D axis in the viewport.
      // public: Ogre::SceneNode *axisNode;

      /// \brief Used to select objects from mouse clicks.
      public: SelectionBuffer *selectionBuffer;

      /// \brief Flag to detect if the user changed the camera pose in the
      /// world file.
      public: bool isCameraSetInWorldFile;

      /// \brief Toggle joystick camera move through ~/user_camera/joy_twist
      public: bool joyTwistControl;

      /// \brief Toggle joystick camera move through ~/user_camera/joy_pose
      public: bool joyPoseControl;

      /// \brief Used to detect joystick button release
      public: bool joystickButtonToggleLast;

      /// \brief An optional Ogre camera for stereo rendering.
      public: Ogre::Camera *rightCamera;

      /// \brief An optional viewport for stereo rendering.
      public: Ogre::Viewport *rightViewport;

      /// \brief Publishes user camera world pose
      public: transport::PublisherPtr posePub;

      /// \brief True if stereo rendering should be enabled.
      public: bool stereoEnabled;

      /// \brief Ratio of screen point to pixel.
      public: double devicePixelRatio = 1.0;

      /// \brief Initial camera pose.
      public: ignition::math::Pose3d initialPose;
    };
  }
}
#endif
