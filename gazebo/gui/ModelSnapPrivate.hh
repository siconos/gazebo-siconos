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
#ifndef GAZEBO_GUI_MODELSNAPPRIVATE_HH_
#define GAZEBO_GUI_MODELSNAPPRIVATE_HH_

#include <mutex>
#include <ignition/math/Triangle3.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class ModelSnap ModelSnap.hh
    /// \brief Private data for the ModelSnap class
    class ModelSnapPrivate
    {
      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Publish user command messages for the server to place in the
      /// undo queue.
      public: transport::PublisherPtr userCmdPub;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr userCamera;

      /// \brief Pointer to the scene where models are in.
      public: rendering::ScenePtr scene;

      /// \brief Ray query for selecting a surface of an entity.
      public: rendering::RayQueryPtr rayQuery;

      /// \brief Current mouse event.
      public: common::MouseEvent mouseEvent;

      /// \brief True if the model align tool is initialized.
      public: bool initialized;

      /// \brief A mesh triangle used as the basis for alignment.
      public: ignition::math::Triangle3d selectedTriangle;

      /// \brief A mesh triangle being hovered.
      public: ignition::math::Triangle3d hoverTriangle;

      /// \brief Currently selected visual.
      public: rendering::VisualPtr selectedVis;

      /// \brief Currently hovered visual.
      public: rendering::VisualPtr hoverVis;

      /// \brief Connection for the render event.
      public: event::ConnectionPtr renderConnection;

      /// \brief Mutex to protect the selected triangle vertices.
      public: std::recursive_mutex updateMutex;

      /// \brief A visual to represent the snap spot.
      public: rendering::VisualPtr snapVisual;

      /// \brief Lines to highlight the selected triangle.
      public: rendering::DynamicLines *snapLines;

      /// \brief A visual to represent the hovered area.
      public: rendering::VisualPtr highlightVisual;

      /// \brief A highlight of the hovered triangle.
      public: rendering::DynamicLines *snapHighlight;

      /// \brief A variable to indicate the selected triangle has changed.
      public: bool selectedTriangleDirty;

      /// \brief A variable to indicate the hovered triangle has changed.
      public: bool hoverTriangleDirty;
    };
  }
}
#endif
