/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_ACTUATORPLUGIN_
#define GAZEBO_PLUGINS_ACTUATORPLUGIN_

#include <functional>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

/// Example SDF:
///       <plugin name="actuator_plugin" filename="libActuatorPlugin.so">
///        <actuator>
///          <name>actuator_0</name> <!-- optional -->
///          <joint>JOINT_0</joint> <!-- name of joint to actuate -->
///          <index>0</index> <!-- needed for multi-DOF joints -->
///          <type>electric_motor</type> <!-- motor model type -->
///          <power>20</power> <!-- parameters for motor model -->
///          <max_velocity>6</max_velocity>
///          <max_torque>10.0</max_torque>
///        </actuator>
///      </plugin>
///    </model>
///
/// Required fields:
/// - name
/// - joint
/// - index (can be 0 in most cases)
/// - type: current options are electric_motor, velocity_limiter or null
/// Required for motor model electric_motor:
/// - power
/// - max_velocity
/// - max_torque
/// Required for motor model velocity_limiter:
/// - max_velocity
/// - max_torque

namespace gazebo
{
  /// \brief Properties for a model of a rotational actuator
  class ActuatorProperties
  {
    /// \brief An identifier for the actuator.
    public: std::string name;

    /// \brief Which joint index is actuated by this actuator.
    public: int jointIndex;

    /// \brief Mechanical power output of the actuator (Watts)
    public: float power;

    /// \brief Maximum velocity of the actuator (radians per second)
    public: float maximumVelocity;

    /// \brief Maximum torque of the actuator (Newton-meters)
    public: float maximumTorque;

    /// \brief Function used to calculate motor output.
    /// \param[in] float1 Input velocity.
    /// \param[in] float2 Input torque.
    /// \param[in] ActuatorProperties Static properties of this actuator
    /// \return Torque according to the model.
    public: std::function<float (float, float, const ActuatorProperties&)>
              modelFunction;
  };

  /// \brief Plugin for simulating a torque-speed curve for actuators.
  class GAZEBO_VISIBLE ActuatorPlugin : public ModelPlugin
  {
    /// Documentation inherited
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback on world update event.
    private: void WorldUpdateCallback();

    /// \brief The joints we want to actuate
    private: std::vector<physics::JointPtr> joints;

    /// \brief Corresponding actuator properties (power, max torque, etc.)
    private: std::vector<ActuatorProperties> actuators;

    /// \brief Connections to events associated with this class.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}

#endif
