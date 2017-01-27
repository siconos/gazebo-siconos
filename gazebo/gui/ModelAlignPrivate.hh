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
#ifndef GAZEBO_GUI_MODELALIGNPRIVATE_HH_
#define GAZEBO_GUI_MODELALIGNPRIVATE_HH_

#include <map>
#include <vector>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class ModelAlignPrivate ModelAlignPrivate.hh
    /// \brief Private data for the ModelAlign class
    class ModelAlignPrivate
    {
      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Publish user command messages for the server to place in the
      /// undo queue.
      public: transport::PublisherPtr userCmdPub;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief The last selected visual which will be used for alignment.
      public: rendering::VisualPtr targetVis;

      /// \brief True if the model align tool is initialized.
      public: bool initialized;

      /// \brief selected visuals.
      public: std::vector<rendering::VisualPtr> selectedVisuals;

      /// \brief A list of connections. Currently used only
      /// to get the align configuration event.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Original model pose used when user resets alignment.
      public: std::map<rendering::VisualPtr, ignition::math::Pose3d>
          originalVisualPose;
    };
  }
}
#endif
