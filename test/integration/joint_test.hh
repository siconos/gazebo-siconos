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

#ifndef GAZEBO_TEST_INTEGRATION_JOINT_TEST_HH_
#define GAZEBO_TEST_INTEGRATION_JOINT_TEST_HH_

#include <string>
#include <sstream>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/test/ServerFixture.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

typedef std::tr1::tuple<const char *, const char *> std_string2;

class JointTest : public ServerFixture,
                  public ::testing::WithParamInterface<std_string2>
{
  protected: JointTest() : ServerFixture()
             {
             }

  /// \brief Test Joint::GetInertiaRatio.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetInertiaRatio(const std::string &_physicsEngine);

  /// \brief Test spring dampers
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SpringDamperTest(const std::string &_physicsEngine);

  /// \brief Create and destroy joints repeatedly, monitors memory usage.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void JointCreationDestructionTest(const std::string &_physicsEngine);

  /// \brief Create joints dynamically and verify that they will be visualized.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void DynamicJointVisualization(const std::string &_physicsEngine);

  // Documentation inherited.
  public: virtual void SetUp()
          {
            const ::testing::TestInfo *const test_info =
              ::testing::UnitTest::GetInstance()->current_test_info();
            if (test_info->value_param())
            {
              gzdbg << "Params: " << test_info->value_param() << std::endl;
              this->physicsEngine = std::tr1::get<0>(GetParam());
              this->jointType = std::tr1::get<1>(GetParam());
            }
          }

  /// \brief Class to hold parameters for spawning joints.
  public: class SpawnJointOptions
  {
    /// \brief Constructor.
    public: SpawnJointOptions() : worldChild(false), worldParent(false),
              wait(common::Time(99, 0)),
              noLinkPose(false), axis(ignition::math::Vector3d(1, 0, 0)),
              useParentModelFrame(false)
            {
            }

    /// \brief Destructor.
    public: ~SpawnJointOptions()
            {
            }

    /// \brief Type of joint to create.
    public: std::string type;

    /// \brief Flag to set child link to the world.
    public: bool worldChild;

    /// \brief Flag to set parent link to the world.
    public: bool worldParent;

    /// \brief Length of time to wait for model to spawn in order to return
    ///        Joint pointer.
    public: common::Time wait;

    /// \brief Model pose for spawned model.
    public: ignition::math::Pose3d modelPose;

    /// \brief Child link pose for spawned model.
    public: ignition::math::Pose3d childLinkPose;

    /// \brief Parent link pose for spawned model.
    public: ignition::math::Pose3d parentLinkPose;

    /// \brief Flag to disable including link pose per issue #978.
    public: bool noLinkPose;

    /// \brief Joint pose for spawned joint.
    public: ignition::math::Pose3d jointPose;

    /// \brief Axis value for spawned joint.
    public: ignition::math::Vector3d axis;

    /// \brief Use parent model frame (#494)
    public: bool useParentModelFrame;
  };

  /// \brief Spawn a model with a joint connecting to the world. The function
  ///        will wait for duration _wait for the model to spawn and attempt
  ///        to return a pointer to the spawned joint. This function is not
  ///        guaranteed to return a valid JointPtr, so the output should be
  ///        checked.
  /// \param[in] _type Type of joint to create.
  /// \param[in] _worldChild Flag to set child link to the world.
  /// \param[in] _worldParent Flag to set parent link to the world.
  /// \param[in] _wait Length of time to wait for model to spawn in order
  ///                  to return Joint pointer.
  public: physics::JointPtr SpawnJoint(const std::string &_type,
                                       bool _worldChild = false,
                                       bool _worldParent = false,
                                   common::Time _wait = common::Time(99, 0))
          {
            SpawnJointOptions opt;
            opt.type = _type;
            opt.worldChild = _worldChild;
            opt.worldParent = _worldParent;
            opt.wait = _wait;
            return SpawnJoint(opt);
          }

  /// \brief Spawn a model with a joint connecting to the world. The function
  ///        will wait for duration _wait for the model to spawn and attempt
  ///        to return a pointer to the spawned joint. This function is not
  ///        guaranteed to return a valid JointPtr, so the output should be
  ///        checked.
  /// \param[in] _opt Options for spawned model and joint.
  public: physics::JointPtr SpawnJoint(const SpawnJointOptions &_opt)
          {
            msgs::Model msg;
            std::string modelName = this->GetUniqueString("joint_model");
            msg.set_name(modelName);
            msgs::Set(msg.mutable_pose(), _opt.modelPose);

            if (!_opt.worldParent)
            {
              msg.add_link();
              int linkCount = msg.link_size();
              auto link = msg.mutable_link(linkCount-1);

              link->set_name("parent");
              if (!_opt.noLinkPose)
              {
                msgs::Set(link->mutable_pose(), _opt.parentLinkPose);
              }
            }
            if (!_opt.worldChild)
            {
              msg.add_link();
              int linkCount = msg.link_size();
              auto link = msg.mutable_link(linkCount-1);

              link->set_name("child");
              if (!_opt.noLinkPose)
              {
                msgs::Set(link->mutable_pose(), _opt.childLinkPose);
              }
            }
            msg.add_joint();
            auto jointMsg = msg.mutable_joint(0);
            jointMsg->set_name("joint");
            jointMsg->set_type(msgs::ConvertJointType(_opt.type));
            msgs::Set(jointMsg->mutable_pose(), _opt.jointPose);
            if (_opt.worldParent)
            {
              jointMsg->set_parent("world");
            }
            else
            {
              jointMsg->set_parent("parent");
            }
            if (_opt.worldChild)
            {
              jointMsg->set_child("world");
            }
            else
            {
              jointMsg->set_child("child");
            }

            {
              auto axis = jointMsg->mutable_axis1();
              msgs::Set(axis->mutable_xyz(), _opt.axis);
              axis->set_use_parent_model_frame(_opt.useParentModelFrame);
            }
            // Hack: hardcode a second axis for universal joints
            if (_opt.type == "universal")
            {
              auto axis2 = jointMsg->mutable_axis2();
              msgs::Set(axis2->mutable_xyz(),
                  ignition::math::Vector3d(0, 1, 0));
              axis2->set_use_parent_model_frame(_opt.useParentModelFrame);
            }

            auto model = this->SpawnModel(msg);
            physics::JointPtr joint;
            if (model != NULL)
              joint = model->GetJoint("joint");

            return joint;
          }

  /// \brief Physics engine for test.
  protected: std::string physicsEngine;

  /// \brief Joint type for test.
  protected: std::string jointType;
};
#endif
