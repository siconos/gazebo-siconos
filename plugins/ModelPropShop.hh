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

#ifndef GAZEBO_PLUGINS_MODELPROPSHOP_HH_
#define GAZEBO_PLUGINS_MODELPROPSHOP_HH_

#include <string>
#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief This plugin will generate 5 pictures of a model: perspective,
  /// top, front, side, back.
  class GAZEBO_VISIBLE ModelPropShop : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~ModelPropShop();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief Callback triggered when the world has been created.
    private: void OnWorldCreated();

    /// \brief Update the plugin.
    private: void Update();

    /// \brief The update connection.
    private: event::ConnectionPtr updateConn;

    /// \brief The world created connection.
    private: event::ConnectionPtr worldCreatedConn;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    /// \brief Publisher used to stop the server.
    private: transport::PublisherPtr pub;

    /// \brief Publisher used to spawn the model.
    private: transport::PublisherPtr factoryPub;

    /// \brief Pointer to the scene.
    private: rendering::ScenePtr scene;

    /// \brief Pointer to the camera.
    private: rendering::CameraPtr camera;

    /// \brief Pointer to the light.
    private: rendering::LightPtr light;

    /// \brief Pointer to the sdf document.
    private: sdf::SDFPtr sdf;

    /// \brief Name of the model.
    private: std::string modelName;

    /// \brief Path in which to save the output images.
    private: boost::filesystem::path savePath;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Ignition node for communication.
    private: ignition::transport::Node nodeIgn;

    /// \brief Ignition publisher used to stop the server.
    private: ignition::transport::Node::Publisher pubIgn;

    /// \brief Ignition publisher used to spawn the model.
    private: ignition::transport::Node::Publisher factoryPubIgn;
  };
}

#endif
