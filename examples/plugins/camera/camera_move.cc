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
#include <functional>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  class CameraMove : public ModelPlugin
  {
    public: CameraMove() : ModelPlugin() {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Get a pointer to the model
      this->model = _model;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CameraMove::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      ignition::math::Vector3d v(0.03, 0, 0);
      ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();
      v = pose.Rot() * v;

      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(v);
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.01));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}
