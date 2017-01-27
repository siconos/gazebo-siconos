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

#include <boost/program_options.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  boost::program_options::options_description desc("Data collection options");

  // Maximum number of collection iterations.
  int maxIterations = 2000;
  // Number of timesteps between samples.
  int sampleTimesteps = 1;
  // How much to scale the actuator's maximum torque by
  float maxTorqueAdj = 2;

  desc.add_options()("max_iter,i", boost::program_options::value<int>(),
                      "number of collection iterations")
                    ("sample_ts,s", boost::program_options::value<int>(),
                      "number of timesteps between samples")
                    ("torque_scale,t", boost::program_options::value<float>(),
                      "scale factor for applied torque");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(
                                  _argc, _argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("max_iter"))
  {
    maxIterations = vm["max_iter"].as<int>();
  }

  if (vm.count("sample_ts"))
  {
    sampleTimesteps = vm["sample_ts"].as<int>();
  }

  if (vm.count("torque_scale"))
  {
    maxTorqueAdj = vm["sample_ts"].as<float>();
  }

  // The joint index which we are collecting data on.
  unsigned int index;
  // The torque we are commanding to the actuator
  float maxTorque;

  // Initialize gazebo.
  gazebo::setupServer(_argc, _argv);

  // Load a world with two models: one actuated, one not
  gazebo::physics::WorldPtr world =
    gazebo::loadWorld("../actuator_example.world");
  if (!world)
  {
    std::cout << "Could not load world actuator_example" << std::endl;
    return -1;
  }

  // Get the models and pointers to their joints
  std::vector<std::string> modelNames;
  modelNames.push_back("actuator_example");
  modelNames.push_back("unactuated_example");

  std::string jointName;

  std::vector<gazebo::physics::JointPtr> joints;

  for (unsigned int i = 0; i < modelNames.size(); i++)
  {
    gazebo::physics::ModelPtr model = world->ModelByName(modelNames[i]);
    if (!model)
    {
      std::cout << "Couldn't find model: " << modelNames[i] << std::endl;
      return -1;
    }

    if (modelNames[i].compare("actuator_example") == 0)
    {
      const sdf::ElementPtr modelSDF = model->GetSDF();
      // Find the ActuatorPlugin SDF block
      if (!modelSDF->HasElement("plugin"))
      {
        std::cout << "ERROR: couldn't find index element." << std::endl;
        return -1;
      }
      sdf::ElementPtr elem = modelSDF->GetElement("plugin");
      while (elem->GetAttribute("filename")->GetAsString().
              compare("libActuatorPlugin.so"))
      {
        elem = elem->GetNextElement("plugin");
      }

      if (!elem->HasElement("actuator"))
      {
        std::cout << "ERROR: couldn't find actuator element" << std::endl;
        return -1;
      }
      elem = elem->GetElement("actuator");

      if (!elem->HasElement("index"))
      {
        std::cout << "ERROR: couldn't find index element." << std::endl;
        return -1;
      }
      index = elem->GetElement("index")->Get<unsigned int>();

      if (!elem->HasElement("max_torque"))
      {
        std::cout << "ERROR: couldn't find max_torque element." << std::endl;
        return -1;
      }
      maxTorque = maxTorqueAdj*elem->GetElement("max_torque")->Get<float>();

      if (!elem->HasElement("joint"))
      {
        std::cout << "ERROR: couldn't find joint element." << std::endl;
        return -1;
      }
      jointName = elem->GetElement("joint")->Get<std::string>();
    }

    gazebo::physics::JointPtr joint = model->GetJoint(jointName);
    if (!joint)
    {
      std::cout << "Couldn't find joint " << jointName << " for model "
                << modelNames[i] << std::endl;
    }
    else
    {
      joints.push_back(joint);
    }
  }

  // Create a data directory
  boost::filesystem::path path("../data");
  if (!boost::filesystem::exists(path))
  {
    boost::filesystem::create_directories(path);
  }
  // Open a file for recording data
  std::ofstream fileStream;
  fileStream.open((path.string() + "/data.csv").c_str());
  // Push initial file headings
  fileStream << "actuated_joint_pos\tactuated_joint_vel\t"
             << "actuated_joint_torque\tunactuated_joint_pos\t"
             << "unactuated_joint_vel\tunactuated_joint_torque" << std::endl;

  // Run the simulation for a fixed number of iterations.
  std::cout << "Collecting data for " << maxIterations << " iterations."
            << std::endl;

  for (unsigned int i = 0; i < maxIterations; ++i)
  {
    for (unsigned int j = 0; j < joints.size(); ++j)
    {
      if (!joints[j])
      {
        std::cout << "got NULL joint in actuator example main.cc" << std::endl;
        continue;
      }
      // Command a constant force on each joint.
      // This causes the joints to accelerate linearly
      // The linear velocity data gives a range of values, allowing us to
      // construct a torque vs. velocity graph
      // By commanding a torque above the actuator's maximum torque value
      // we are forcing the maximum force state of the actuator
      joints[j]->SetForce(index, maxTorque);
    }

    gazebo::runWorld(world, sampleTimesteps);

    // Print joint position, velocity and torques for each model to file
    for (unsigned int j = 0; j < joints.size(); ++j)
    {
      fileStream << joints[j]->Position(index) << "\t"
                 << joints[j]->GetVelocity(index) << "\t"
                 << joints[j]->GetForce(index);
      if (j == joints.size() - 1)
      {
        fileStream << std::endl;
      }
      else
      {
        fileStream << "\t";
      }
    }
  }

  // Close everything.
  fileStream.close();
  gazebo::shutdown();

  std::cout << "Finished data collection. Closing Gazebo." << std::endl;
  return 0;
}
