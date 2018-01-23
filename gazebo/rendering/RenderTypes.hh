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
#ifndef GAZEBO_RENDERING_RENDERTYPES_HH_
#define GAZEBO_RENDERING_RENDERTYPES_HH_

#include <boost/shared_ptr.hpp>
#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

/// \def GZ_VISIBILITY_ALL
/// \brief Render everything visibility mask.
#define GZ_VISIBILITY_ALL             0x0FFFFFFF

/// \def GZ_VISIBILITY_SELECTION
/// \brief Renders only objects that can be selected.
#define GZ_VISIBILITY_SELECTION       0x10000000

/// \def GZ_VISIBILITY_GUI
/// \brief Render GUI visuals mask.
#define GZ_VISIBILITY_GUI             0x00000001

/// \def GZ_VISIBILITY_SELECTABLE
/// \brief Render visuals that are selectable mask.
#define GZ_VISIBILITY_SELECTABLE      0x00000002

namespace gazebo
{
  namespace rendering
  {
    class Scene;
    class Light;
    class Camera;
    class UserCamera;
    class DepthCamera;
    class WideAngleCamera;
    class GpuLaser;
    class DynamicLines;
    class Visual;
    class LaserVisual;
    class LogicalCameraVisual;
    class SonarVisual;
    class WrenchVisual;
    class CameraVisual;
    class JointVisual;
    class AxisVisual;
    class ArrowVisual;
    class ContactVisual;
    class COMVisual;
    class InertiaVisual;
    class RFIDVisual;
    class RFIDTagVisual;
    class ApplyWrenchVisual;
    class OriginVisual;
    class LinkFrameVisual;
    class WindowManager;
    class SelectionObj;
    class RayQuery;
    class Distortion;
    class LensFlare;
    class Road2d;

#ifdef HAVE_OCULUS
    class OculusCamera;
#endif

    /// \def ScenePtr
    /// \brief Shared pointer to Scene
    typedef boost::shared_ptr<Scene> ScenePtr;

    /// \def LightPtr
    /// \brief Shared pointer to Light
    typedef boost::shared_ptr<Light> LightPtr;

    /// \def CameraPtr
    /// \brief Shared pointer to Camera
    typedef boost::shared_ptr<Camera> CameraPtr;

    /// \def UserCameraPtr
    /// \brief Shared pointer to UserCamera
    typedef boost::shared_ptr<UserCamera> UserCameraPtr;

    /// \def DepthCameraPtr
    /// \brief Shared pointer to DepthCamera
    typedef boost::shared_ptr<DepthCamera> DepthCameraPtr;

    /// \def WideAngleCameraPtr
    /// \brief Shared pointer to WideAngleCamera
    typedef boost::shared_ptr<WideAngleCamera> WideAngleCameraPtr;

    /// \def GpuLaserPtr
    /// \brief Shared pointer to GpuLaser
    typedef boost::shared_ptr<GpuLaser> GpuLaserPtr;

    /// \def DynamicLinesPtr
    /// \brief Shared pointer to DynamicLines
    typedef boost::shared_ptr<DynamicLines> DynamicLinesPtr;

    /// \def VisualPtr
    /// \brief Shared pointer to Visual
    typedef std::shared_ptr<Visual> VisualPtr;

    /// \def VisualWeakPtr
    /// \brief Weak pointer to Visual
    typedef std::weak_ptr<Visual> VisualWeakPtr;

    /// \def LaserVisualPtr
    /// \brief Shared pointer to LaserVisual
    typedef std::shared_ptr<LaserVisual> LaserVisualPtr;

    /// \def SonarVisualPtr
    /// \brief Shared pointer to SonarVisual
    typedef std::shared_ptr<SonarVisual> SonarVisualPtr;

    /// \def WrenchVisualPtr
    /// \brief Shared pointer to WrenchVisual
    typedef std::shared_ptr<WrenchVisual> WrenchVisualPtr;

    /// \def CameraVisualPtr
    /// \brief Shared pointer to CameraVisual
    typedef std::shared_ptr<CameraVisual> CameraVisualPtr;

    /// \def LogicalCameraVisualPtr
    /// \brief Shared pointer to LogicalCameraVisual
    typedef std::shared_ptr<LogicalCameraVisual> LogicalCameraVisualPtr;

    /// \def JointVisualPtr
    /// \brief Shared pointer to JointVisual
    typedef std::shared_ptr<JointVisual> JointVisualPtr;

    /// \def ContactVisualPtr
    /// \brief Shared pointer to ContactVisual
    typedef std::shared_ptr<ContactVisual> ContactVisualPtr;

    /// \def ArrowVisualPtr
    /// \brief Shared pointer to ArrowVisual
    typedef std::shared_ptr<ArrowVisual> ArrowVisualPtr;

    /// \def AxisVisualPtr
    /// \brief Shared pointer to AxisVisual
    typedef std::shared_ptr<AxisVisual> AxisVisualPtr;

    /// \def COMVisualPtr
    /// \brief Shared pointer to COMVisual
    typedef std::shared_ptr<COMVisual> COMVisualPtr;

    /// \def InertiaVisualPtr
    /// \brief Shared pointer to InertiaVisual
    typedef std::shared_ptr<InertiaVisual> InertiaVisualPtr;

    /// \def RFIDVisual
    /// \brief Shared pointer to RFIDVisual
    typedef std::shared_ptr<RFIDVisual> RFIDVisualPtr;

    /// \def RFIDTagVisual
    /// \brief Shared pointer to RFIDTagVisual
    typedef std::shared_ptr<RFIDTagVisual> RFIDTagVisualPtr;

    /// \def ApplyWrenchVisualPtr
    /// \brief Shared pointer to ApplyWrenchVisual
    typedef std::shared_ptr<ApplyWrenchVisual> ApplyWrenchVisualPtr;

    /// \def OriginVisualPtr
    /// \brief Shared pointer to OriginVisual
    typedef std::shared_ptr<OriginVisual> OriginVisualPtr;

    /// \def LinkFrameVisualPtr
    /// \brief Shared pointer to LinkFrameVisual
    typedef std::shared_ptr<LinkFrameVisual> LinkFrameVisualPtr;

    /// \def WindowManager
    /// \brief Shared pointer to WindowManager
    typedef boost::shared_ptr<WindowManager> WindowManagerPtr;

    /// \def SelectionObj
    /// \brief Shared pointer to SelectionObj
    typedef std::shared_ptr<SelectionObj> SelectionObjPtr;

    /// \def RayQueryPtr
    /// \brief Shared pointer to RayQuery
    typedef boost::shared_ptr<RayQuery> RayQueryPtr;

    /// \def DistortionPtr
    /// \brief Shared pointer to Distortion
    typedef boost::shared_ptr<Distortion> DistortionPtr;

    /// \def LensFlarePtr
    /// \brief Shared pointer to LensFlare
    typedef std::shared_ptr<LensFlare> LensFlarePtr;

    /// \def Road2dPtr
    /// \brief Shared pointer to Road2d
    typedef std::shared_ptr<Road2d> Road2dPtr;

#ifdef HAVE_OCULUS
    /// \def OculusCameraPtr
    /// \brief Shared pointer to OculusCamera
    typedef boost::shared_ptr<OculusCamera> OculusCameraPtr;
#endif

    /// \enum RenderOpType
    /// \brief Type of render operation for a drawable
    enum RenderOpType
    {
      /// \brief A list of points, 1 vertex per point
      RENDERING_POINT_LIST = 0,

      /// \brief A list of lines, 2 vertices per line
      RENDERING_LINE_LIST = 1,

      /// \brief A strip of connected lines, 1 vertex per line plus 1
      /// start vertex
      RENDERING_LINE_STRIP = 2,

      /// \brief A list of triangles, 3 vertices per triangle
      RENDERING_TRIANGLE_LIST = 3,

      /// \brief A strip of triangles, 3 vertices for the first triangle,
      /// and 1 per triangle after that
      RENDERING_TRIANGLE_STRIP = 4,

      /// \brief A fan of triangles, 3 vertices for the first triangle,
      /// and 1 per triangle after that
      RENDERING_TRIANGLE_FAN = 5,

      /// \brief N/A
      RENDERING_MESH_RESOURCE = 6
    };

    /// \brief Frame of reference
    enum ReferenceFrame
    {
      /// \brief Local frame
      RF_LOCAL,

      /// \brief Parent frame
      RF_PARENT,

      /// \brief World frame
      RF_WORLD
    };
  }
}
#endif
