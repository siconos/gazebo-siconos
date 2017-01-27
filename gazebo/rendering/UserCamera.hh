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
#ifndef GAZEBO_RENDERING_USERCAMERA_HH_
#define GAZEBO_RENDERING_USERCAMERA_HH_

#include <string>
#include <vector>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  // Forward declare private data
  class UserCameraPrivate;

  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class UserCamera UserCamera.hh rendering/rendering.hh
    /// \brief A camera used for user visualization of a scene
    class GZ_RENDERING_VISIBLE UserCamera : public Camera
    {
      /// \brief Constructor
      /// \param[in] _name Name of the camera.
      /// \param[in] _scene Scene to put the camera in.
      /// \param[in] _stereoEnabled True to enable stereo rendering. This is
      /// here for compatibility with 3D monitors/TVs.
      public: UserCamera(const std::string &_name, ScenePtr _scene,
                  bool _stereoEnabled = false);

      /// \brief Destructor
      public: virtual ~UserCamera();

      /// \brief Load the user camera.
      /// \param[in] _sdf Parameters for the camera.
      public: void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void SetClipDist(const float _near, const float _far);
      using Camera::SetClipDist;

      /// \brief Generic load function
      public: void Load();

      /// \brief Initialize
      public: void Init();

      /// \brief Render the camera
      public: virtual void Update();

      /// \brief Render the camera. The _force parameter is ignored.
      /// Called after the pre-render signal. This function will generate
      /// camera images.
      ///
      /// UserCamera does not throttle the Render function. It is assumed
      /// Render is called at the desired and correct time interval.
      /// We recommend setting this time interval to the value returned by
      /// the RenderRate() function.
      /// \param[in] _force This parameter is not used.
      public: virtual void Render(const bool _force = false);
      using Camera::Render;

      /// \brief Post render
      public: virtual void PostRender();

      /// \brief Finialize
      public: void Fini();

      // Documentation inherited.
      public: virtual void SetWorldPose(const ignition::math::Pose3d &_pose);

      /// \brief Set the initial pose in the world coordinate frame and set
      /// that as the current camera world pose.
      /// \param[in] _pose New default pose of the camera.
      public: void SetInitialPose(const ignition::math::Pose3d &_pose);

      /// \brief Get the initial pose in the world coordinate frame.
      /// \return Initial pose of the camera.
      public: ignition::math::Pose3d InitialPose() const;

      /// \brief Handle a mouse event.
      /// \param[in] _evt The mouse event.
      public: void HandleMouseEvent(const common::MouseEvent &_evt);

      /// \brief Handle a key press.
      /// \param[in] _key The key pressed.
      public: void HandleKeyPressEvent(const std::string &_key);

      /// \brief Handle a key release.
      /// \param[in] _key The key released.
      public: void HandleKeyReleaseEvent(const std::string &_key);

      /// \brief Set view controller.
      /// \param[in] _type The type of view controller: "orbit", "fps"
      public: void SetViewController(const std::string &_type);

      /// \brief Set view controller
      /// \param[in] _type The type of view controller: "orbit", "fps"
      /// \param[in] _pos The initial pose of the camera.
      public: void SetViewController(const std::string &_type,
                                     const ignition::math::Vector3d &_pos);

      /// \brief Get current view controller type.
      /// \return Type of the current view controller: "orbit", "fps"
      public: std::string GetViewControllerTypeString();

      /// \brief Resize the camera.
      /// \param[in] _w Width of the camera image.
      /// \param[in] _h Height of the camera image.
      public: void Resize(unsigned int _w, unsigned int _h);

      /// \brief Set the dimensions of the viewport.
      /// \param[in] _x X position of the viewport.
      /// \param[in] _y Y position of the viewport.
      /// \param[in] _w Width of the viewport.
      /// \param[in] _h Height of the viewport.
      public: void SetViewportDimensions(float _x, float _y,
                                         float _w, float _h);

      /// \brief Move the camera to focus on a visual.
      /// \param[in] _visual Visual to move the camera to.
      public: void MoveToVisual(VisualPtr _visual);

      /// \brief Move the camera to focus on a visual.
      /// \param[in] _visualName Name of the visual to move the camera to.
      public: void MoveToVisual(const std::string &_visualName);

      /// \brief Set the screen point to device pixel ratio
      /// \param[in] _ratio Point to pixel ratio.
      public: void SetDevicePixelRatio(const double _ratio);

      /// \brief Get the screen point to device pixel ratio
      /// \return Point to pixel ratio
      public: double DevicePixelRatio() const;

      // Documentation Inherited
      public: virtual void CameraToViewportRay(const int _screenx,
                  const int _screeny,
                  ignition::math::Vector3d &_origin,
                  ignition::math::Vector3d &_dir) const;

      /// \brief Set to true to enable rendering
      ///
      /// Use this only if you really know what you're doing.
      /// \param[in] _target The new rendering target.
      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target);

      /// \brief Set whether the view controller is enabled.
      ///
      /// The view controller is used to handle user camera movements.
      /// \param[in] _value True to enable viewcontroller, False to
      /// disable.
      public: void EnableViewController(bool _value) const;

      /// \brief Get an entity at a pixel location using a camera. Used for
      /// mouse picking.
      /// \param[in] _mousePos The position of the mouse in screen coordinates
      /// \param[out] _mod Used for object manipulation
      /// \return The selected entity, or NULL
      public: VisualPtr Visual(const ignition::math::Vector2i &_mousePos,
                  std::string &_mod) const;

      /// \brief Get a visual at a mouse position
      /// \param[in] _mousePos 2D position of the mouse in pixels.
      /// \return The selected entity, or NULL
      public: VisualPtr Visual(
                  const ignition::math::Vector2i &_mousePos) const;

      /// \brief Set the point the camera should orbit around.
      /// \param[in] _pt The focal point
      public: void SetFocalPoint(const ignition::math::Vector3d &_pt);

      // Documentation inherited
      public: virtual unsigned int GetImageWidth() const;

      // Documentation inherited
      public: virtual unsigned int GetImageHeight() const;

      /// brief Show if the user camera pose has changed in the world file.
      /// return true if the camera pose changed in the world file.
      public: bool IsCameraSetInWorldFile();

      /// brief Set if the user camera pose has changed in the world file.
      /// \param[in] _value True if the camera pose changed in the world file.
      public: void SetUseSDFPose(bool _value);

      /// brief Enable or disable camera control through ~/user_camera/joy_twist
      /// gz topic. Defaults to true.
      /// \param[in] _value True to enable camera pose control by
      /// gz topic ~/user_camera/joy_twist.
      public: void SetJoyTwistControl(bool _value);

      /// brief Enable or disable camera control through ~/user_camera/joy_pose
      /// gz topic. Defaults to true.
      /// \param[in] _value True to enable camera pose control by
      /// gz topic ~/user_camera/joy_pose.
      public: void SetJoyPoseControl(bool _value);

      /// \brief Get whether stereo is enabled.
      /// \return True if stereo is enabled.
      public: bool StereoEnabled() const;

      /// \brief Turn on/off stereo rendering. Stereo must be initially
      /// enable in the ~/.gazebo/gui.ini file using:
      ///
      ///     [rendering]
      ///     stereo=1
      ///
      /// \param[in] _enable True to turn on stereo, false to turn off.
      public: void EnableStereo(bool _enable);

      // Documentation inherited.
      public: virtual bool SetProjectionType(const std::string &_type);

      // Documentation inherited.
      public: virtual ignition::math::Vector2i Project(
          const ignition::math::Vector3d &_pt) const;

      /// \brief Set the camera to be attached to a visual.
      ///
      /// This causes the camera to move in relation to the specified visual.
      /// \param[in] _visual The visual to attach to.
      /// \param[in] _inheritOrientation True if the camera should also
      /// rotate when the visual rotates.
      /// \param[in] _minDist Minimum distance the camera can get to the
      /// visual.
      /// \param[in] _maxDist Maximum distance the camera can get from the
      /// visual.
      /// \return True if successfully attach to the visual.
      protected: virtual bool AttachToVisualImpl(VisualPtr _visual,
                     const bool _inheritOrientation, const double _minDist = 0,
                     const double _maxDist = 0);
      using Camera::AttachToVisualImpl;

      // Documentation inherited.
      protected: virtual void AnimationComplete();

      /// \brief Set the camera to track a scene node.
      ///
      /// Tracking just causes the camera to rotate to follow the visual.
      /// \param[in] _visual Visual to track.
      /// \return True if the camera is now tracking the visual.
      protected: virtual bool TrackVisualImpl(VisualPtr _visual);

      // Documentation inherited.
      protected: virtual void UpdateFOV();

      /// \brief Toggle whether to show the visual.
      private: void ToggleShowVisual();

      /// \brief Set whether to show the visual.
      /// \param[in] _show True to show the visual representation for this
      /// camera. Currently disabled.
      private: void ShowVisual(bool _show);

      /// \brief Callback used when the camera has finished moving to
      /// a visual.
      private: void OnMoveToVisualComplete();

      /// \brief Handles incoming relative joystick messages.
      /// Incoming joystick messages are used to control
      /// translation and rotation rates of the camera position.
      /// \param[in] _msg New joystick message.
      private: void OnJoyTwist(ConstJoystickPtr &_msg);

      /// \brief Handles incoming absolute joystick messages.
      /// Incoming joystick messages are used to control
      /// camera's world pose.
      /// \param[in] _msg New pose message.
      private: void OnJoyPose(ConstPosePtr &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: UserCameraPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
