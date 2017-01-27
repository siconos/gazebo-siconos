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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/ConfigWidget_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ConfigWidget_TEST::EmptyMsgWidget()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;
  visualConfigWidget->Load(&visualMsg);
  gazebo::msgs::Visual *retVisualMsg =
      dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->Msg());
  QVERIFY(retVisualMsg != NULL);
  delete visualConfigWidget;

  gazebo::gui::ConfigWidget *collisionConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Collision collisionMsg;
  collisionConfigWidget->Load(&collisionMsg);
  gazebo::msgs::Collision *retCollisionMsg =
      dynamic_cast<gazebo::msgs::Collision *>(collisionConfigWidget->Msg());
  QVERIFY(retCollisionMsg != NULL);
  delete collisionConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::JointMsgWidget()
{
  gazebo::gui::ConfigWidget *jointConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Joint jointMsg;

  {
    // joint
    jointMsg.set_name("test_joint");
    jointMsg.set_id(1122u);
    jointMsg.set_parent("test_joint_parent");
    jointMsg.set_parent_id(212121u);
    jointMsg.set_child("test_joint_child");
    jointMsg.set_child_id(454545u);

    // type
    jointMsg.set_type(gazebo::msgs::ConvertJointType("revolute"));

    // pose
    ignition::math::Vector3d pos(4.0, -1.0, 3.5);
    ignition::math::Quaterniond quat(0.0, 1.57, 0.0);
    gazebo::msgs::Set(jointMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // axis1
    gazebo::msgs::Axis *axisMsg = jointMsg.mutable_axis1();
    gazebo::msgs::Set(axisMsg->mutable_xyz(), ignition::math::Vector3d::UnitX);
    axisMsg->set_use_parent_model_frame(false);
    axisMsg->set_limit_lower(-999.0);
    axisMsg->set_limit_upper(999.0);
    axisMsg->set_limit_effort(-1.0);
    axisMsg->set_limit_velocity(-1.0);
    axisMsg->set_damping(0.0);

    // other joint physics properties
    jointMsg.set_cfm(0.2);
    jointMsg.set_bounce(0.3);
    jointMsg.set_velocity(0.4);
    jointMsg.set_fudge_factor(0.5);
    jointMsg.set_limit_cfm(0.6);
    jointMsg.set_limit_erp(0.7);
    jointMsg.set_suspension_cfm(0.8);
    jointMsg.set_suspension_erp(0.9);
  }
  jointConfigWidget->Load(&jointMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->Msg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint");
    QCOMPARE(retJointMsg->id(), 1122u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent");
    QCOMPARE(retJointMsg->parent_id(), 212121u);
    QVERIFY(retJointMsg->child() == "test_joint_child");
    QCOMPARE(retJointMsg->child_id(), 454545u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("revolute"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 4.0);
    QCOMPARE(posMsg.y(), -1.0);
    QCOMPARE(posMsg.z(), 3.5);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    ignition::math::Quaterniond quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.Euler().X(), 0.0);
    QCOMPARE(quat.Euler().Y(), 1.57);
    QCOMPARE(quat.Euler().Z(), 0.0);

    // axis1
    gazebo::msgs::Axis *axisMsg = jointMsg.mutable_axis1();
    QCOMPARE(axisMsg->xyz().x(), 1.0);
    QCOMPARE(axisMsg->xyz().y(), 0.0);
    QCOMPARE(axisMsg->xyz().z(), 0.0);
    QCOMPARE(axisMsg->use_parent_model_frame(), false);
    QCOMPARE(axisMsg->limit_lower(), -999.0);
    QCOMPARE(axisMsg->limit_upper(), 999.0);
    QCOMPARE(axisMsg->limit_effort(), -1.0);
    QCOMPARE(axisMsg->limit_velocity(), -1.0);
    QCOMPARE(axisMsg->damping(), 0.0);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.2);
    QCOMPARE(retJointMsg->bounce(), 0.3);
    QCOMPARE(retJointMsg->velocity(), 0.4);
    QCOMPARE(retJointMsg->fudge_factor(), 0.5);
    QCOMPARE(retJointMsg->limit_cfm(), 0.6);
    QCOMPARE(retJointMsg->limit_erp(), 0.7);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.8);
    QCOMPARE(retJointMsg->suspension_erp(), 0.9);
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  // Joint type revolute -> universal
  {
    // joint
    jointConfigWidget->SetStringWidgetValue("name", "test_joint_updated");
    jointConfigWidget->SetUIntWidgetValue("id", 9999999u);
    jointConfigWidget->SetStringWidgetValue("parent",
        "test_joint_parent_updated");
    jointConfigWidget->SetUIntWidgetValue("parent_id", 1u);
    jointConfigWidget->SetStringWidgetValue("child",
        "test_joint_child_updated");
    jointConfigWidget->SetUIntWidgetValue("child_id", 2u);

    // type
    jointConfigWidget->SetEnumWidgetValue("type",
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_UNIVERSAL));

    // pose
    ignition::math::Vector3d pos(2.0, 9.0, -4.0);
    ignition::math::Quaterniond quat(0.0, 0.0, 1.57);
    jointConfigWidget->SetPoseWidgetValue("pose",
        ignition::math::Pose3d(pos, quat));

    // axis1
    jointConfigWidget->SetVector3dWidgetValue("axis1::xyz",
        ignition::math::Vector3d::UnitY);
    jointConfigWidget->SetBoolWidgetValue("axis1::use_parent_model_frame",
        true);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_lower", -1.2);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_upper", -1.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_effort", 1.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_velocity", 100.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::damping", 0.9);

    // axis2
    jointConfigWidget->SetVector3dWidgetValue("axis2::xyz",
        ignition::math::Vector3d::UnitZ);
    jointConfigWidget->SetBoolWidgetValue("axis2::use_parent_model_frame",
        true);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_lower", -3.2);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_upper", -3.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_effort", 3.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_velocity", 300.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::damping", 3.9);

    // other joint physics properties
    jointConfigWidget->SetDoubleWidgetValue("cfm", 0.9);
    jointConfigWidget->SetDoubleWidgetValue("bounce", 0.8);
    jointConfigWidget->SetDoubleWidgetValue("velocity", 0.7);
    jointConfigWidget->SetDoubleWidgetValue("fudge_factor", 0.6);
    jointConfigWidget->SetDoubleWidgetValue("limit_cfm", 0.5);
    jointConfigWidget->SetDoubleWidgetValue("limit_erp", 0.4);
    jointConfigWidget->SetDoubleWidgetValue("suspension_cfm", 0.3);
    jointConfigWidget->SetDoubleWidgetValue("suspension_erp", 0.2);
  }

  // verify widget values
  {
    // joint
    QVERIFY(jointConfigWidget->StringWidgetValue("name") ==
        "test_joint_updated");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("id"), 9999999u);
    QVERIFY(jointConfigWidget->StringWidgetValue("parent") ==
        "test_joint_parent_updated");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("parent_id"), 1u);
    QVERIFY(jointConfigWidget->StringWidgetValue("child") ==
        "test_joint_child_updated");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("child_id"), 2u);

    // type
    QCOMPARE(jointConfigWidget->EnumWidgetValue("type"),
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_UNIVERSAL));

    // pose
    ignition::math::Vector3d pos(2.0, 9.0, -4.0);
    ignition::math::Quaterniond quat(0.0, 0.0, 1.57);
    QCOMPARE(jointConfigWidget->PoseWidgetValue("pose"),
        ignition::math::Pose3d(pos, quat));

    // axis1
    QCOMPARE(jointConfigWidget->Vector3dWidgetValue("axis1::xyz"),
        ignition::math::Vector3d::UnitY);
    QCOMPARE(jointConfigWidget->BoolWidgetValue(
        "axis1::use_parent_model_frame"), true);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis1::limit_lower"), -1.2);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis1::limit_upper"), -1.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis1::limit_effort"), 1.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis1::limit_velocity"),
        100.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis1::damping"), 0.9);

    // axis2
    QCOMPARE(jointConfigWidget->Vector3dWidgetValue("axis2::xyz"),
        ignition::math::Vector3d::UnitZ);
    QCOMPARE(jointConfigWidget->BoolWidgetValue(
        "axis1::use_parent_model_frame"), true);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis2::limit_lower"), -3.2);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis2::limit_upper"), -3.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis2::limit_effort"), 3.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis2::limit_velocity"),
        300.0);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("axis2::damping"), 3.9);

    // other joint physics properties
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("cfm"), 0.9);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("bounce"), 0.8);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("velocity"), 0.7);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("fudge_factor"), 0.6);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("limit_cfm"), 0.5);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("limit_erp"), 0.4);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("suspension_cfm"), 0.3);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("suspension_erp"), 0.2);
  }

  // verify updates in new msg
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->Msg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint_updated");
    QCOMPARE(retJointMsg->id(), 9999999u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent_updated");
    QCOMPARE(retJointMsg->parent_id(), 1u);
    QVERIFY(retJointMsg->child() == "test_joint_child_updated");
    QCOMPARE(retJointMsg->child_id(), 2u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("universal"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 2.0);
    QCOMPARE(posMsg.y(), 9.0);
    QCOMPARE(posMsg.z(), -4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    ignition::math::Quaterniond quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.Euler().X(), 0.0);
    QCOMPARE(quat.Euler().Y(), 0.0);
    QCOMPARE(quat.Euler().Z(), 1.57);

    // axis1
    gazebo::msgs::Axis *axisMsg = retJointMsg->mutable_axis1();
    QCOMPARE(axisMsg->xyz().x(), 0.0);
    QCOMPARE(axisMsg->xyz().y(), 1.0);
    QCOMPARE(axisMsg->xyz().z(), 0.0);
    QCOMPARE(axisMsg->use_parent_model_frame(), true);
    QCOMPARE(axisMsg->limit_lower(), -1.2);
    QCOMPARE(axisMsg->limit_upper(), -1.0);
    QCOMPARE(axisMsg->limit_effort(), 1.0);
    QCOMPARE(axisMsg->limit_velocity(), 100.0);
    QCOMPARE(axisMsg->damping(), 0.9);

    // axis2
    gazebo::msgs::Axis *axis2Msg = retJointMsg->mutable_axis2();
    QCOMPARE(axis2Msg->xyz().x(), 0.0);
    QCOMPARE(axis2Msg->xyz().y(), 0.0);
    QCOMPARE(axis2Msg->xyz().z(), 1.0);
    QCOMPARE(axis2Msg->use_parent_model_frame(), true);
    QCOMPARE(axis2Msg->limit_lower(), -3.2);
    QCOMPARE(axis2Msg->limit_upper(), -3.0);
    QCOMPARE(axis2Msg->limit_effort(), 3.0);
    QCOMPARE(axis2Msg->limit_velocity(), 300.0);
    QCOMPARE(axis2Msg->damping(), 3.9);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.9);
    QCOMPARE(retJointMsg->bounce(), 0.8);
    QCOMPARE(retJointMsg->velocity(), 0.7);
    QCOMPARE(retJointMsg->fudge_factor(), 0.6);
    QCOMPARE(retJointMsg->limit_cfm(), 0.5);
    QCOMPARE(retJointMsg->limit_erp(), 0.4);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.3);
    QCOMPARE(retJointMsg->suspension_erp(), 0.2);
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  // Joint type universal -> ball
  {
    // joint
    jointConfigWidget->SetStringWidgetValue("name", "test_joint_updated2");
    jointConfigWidget->SetUIntWidgetValue("id", 2222222u);
    jointConfigWidget->SetStringWidgetValue("parent",
        "test_joint_parent_updated2");
    jointConfigWidget->SetUIntWidgetValue("parent_id", 10u);
    jointConfigWidget->SetStringWidgetValue("child",
        "test_joint_child_updated2");
    jointConfigWidget->SetUIntWidgetValue("child_id", 20u);

    // type
    jointConfigWidget->SetEnumWidgetValue("type",
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_BALL));

    // pose
    ignition::math::Vector3d pos(-2.0, 1.0, 2.0);
    ignition::math::Quaterniond quat(0.0, 0.0, 0.0);
    jointConfigWidget->SetPoseWidgetValue("pose",
        ignition::math::Pose3d(pos, quat));

    // other joint physics properties
    jointConfigWidget->SetDoubleWidgetValue("cfm", 0.19);
    jointConfigWidget->SetDoubleWidgetValue("bounce", 0.18);
    jointConfigWidget->SetDoubleWidgetValue("velocity", 2.7);
    jointConfigWidget->SetDoubleWidgetValue("fudge_factor", 0.26);
    jointConfigWidget->SetDoubleWidgetValue("limit_cfm", 0.15);
    jointConfigWidget->SetDoubleWidgetValue("limit_erp", 0.24);
    jointConfigWidget->SetDoubleWidgetValue("suspension_cfm", 0.13);
    jointConfigWidget->SetDoubleWidgetValue("suspension_erp", 0.12);
  }

  // verify widget values
  {
    // joint
    QVERIFY(jointConfigWidget->StringWidgetValue("name") ==
        "test_joint_updated2");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("id"), 2222222u);
    QVERIFY(jointConfigWidget->StringWidgetValue("parent") ==
        "test_joint_parent_updated2");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("parent_id"), 10u);
    QVERIFY(jointConfigWidget->StringWidgetValue("child") ==
        "test_joint_child_updated2");
    QCOMPARE(jointConfigWidget->UIntWidgetValue("child_id"), 20u);

    // type
    QCOMPARE(jointConfigWidget->EnumWidgetValue("type"),
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_BALL));

    // pose
    ignition::math::Vector3d pos(-2.0, 1.0, 2.0);
    ignition::math::Quaterniond quat(0.0, 0.0, 0.0);
    QCOMPARE(jointConfigWidget->PoseWidgetValue("pose"),
        ignition::math::Pose3d(pos, quat));

    // other joint physics properties
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("cfm"), 0.19);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("bounce"), 0.18);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("velocity"), 2.7);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("fudge_factor"), 0.26);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("limit_cfm"), 0.15);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("limit_erp"), 0.24);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("suspension_cfm"), 0.13);
    QCOMPARE(jointConfigWidget->DoubleWidgetValue("suspension_erp"), 0.12);
  }

  // verify updates in new msg
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->Msg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint_updated2");
    QCOMPARE(retJointMsg->id(), 2222222u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent_updated2");
    QCOMPARE(retJointMsg->parent_id(), 10u);
    QVERIFY(retJointMsg->child() == "test_joint_child_updated2");
    QCOMPARE(retJointMsg->child_id(), 20u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("ball"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), -2.0);
    QCOMPARE(posMsg.y(), 1.0);
    QCOMPARE(posMsg.z(), 2.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    ignition::math::Quaterniond quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.Euler().X(), 0.0);
    QCOMPARE(quat.Euler().Y(), 0.0);
    QCOMPARE(quat.Euler().Z(), 0.0);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.19);
    QCOMPARE(retJointMsg->bounce(), 0.18);
    QCOMPARE(retJointMsg->velocity(), 2.7);
    QCOMPARE(retJointMsg->fudge_factor(), 0.26);
    QCOMPARE(retJointMsg->limit_cfm(), 0.15);
    QCOMPARE(retJointMsg->limit_erp(), 0.24);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.13);
    QCOMPARE(retJointMsg->suspension_erp(), 0.12);
  }
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::VisualMsgWidget()
{
  // create a visual message with test values
  // leave out plugin field for now

  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_name("test_visual");
    visualMsg.set_id(12345u);
    visualMsg.set_parent_name("test_visual_parent");
    visualMsg.set_parent_id(54321u);
    visualMsg.set_cast_shadows(true);
    visualMsg.set_transparency(0.0);
    visualMsg.set_visible(true);
    visualMsg.set_delete_me(false);
    visualMsg.set_is_static(false);
    gazebo::msgs::Set(visualMsg.mutable_scale(),
        ignition::math::Vector3d(1.0, 1.0, 1.0));

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    materialMsg->set_shader_type(
        gazebo::msgs::Material::Material::VERTEX);
    materialMsg->set_normal_map("test_normal_map");
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));
    gazebo::msgs::Set(materialMsg->mutable_specular(),
        gazebo::common::Color(1.0, 1.0, 1.0, 0.6));
    gazebo::msgs::Set(materialMsg->mutable_emissive(),
        gazebo::common::Color(0.0, 0.5, 0.2, 1.0));
    materialMsg->set_lighting(true);
    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->add_uri("test_script_uri_0");
    scriptMsg->add_uri("test_script_uri_1");
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Visual *retVisualMsg =
        dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->Msg());
    QVERIFY(retVisualMsg != NULL);

    // visual
    QVERIFY(retVisualMsg->name() == "test_visual");
    QCOMPARE(retVisualMsg->id(), 12345u);
    QVERIFY(retVisualMsg->parent_name() == "test_visual_parent");
    QCOMPARE(retVisualMsg->parent_id(), 54321u);
    QCOMPARE(retVisualMsg->cast_shadows(), true);
    QCOMPARE(retVisualMsg->transparency(), 0.0);
    QCOMPARE(retVisualMsg->visible(), true);
    QCOMPARE(retVisualMsg->delete_me(), false);
    QCOMPARE(retVisualMsg->is_static(), false);
    const gazebo::msgs::Vector3d scaleMsg = retVisualMsg->scale();
    QCOMPARE(scaleMsg.x(), 1.0);
    QCOMPARE(scaleMsg.y(), 1.0);
    QCOMPARE(scaleMsg.z(), 1.0);

    // pose
    const gazebo::msgs::Pose poseMsg = retVisualMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 2.0);
    QCOMPARE(posMsg.y(), 3.0);
    QCOMPARE(posMsg.z(), 4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    ignition::math::Quaterniond quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.Euler().X(), 1.57);
    QCOMPARE(quat.Euler().Y(), 0.0);
    QCOMPARE(quat.Euler().Z(), 0.0);

    // geometry
    const gazebo::msgs::Geometry geometryMsg = retVisualMsg->geometry();
    QCOMPARE(geometryMsg.type(), gazebo::msgs::Geometry::CYLINDER);
    const gazebo::msgs::CylinderGeom cylinderGeomMsg = geometryMsg.cylinder();
    QCOMPARE(cylinderGeomMsg.radius(), 3.0);
    QCOMPARE(cylinderGeomMsg.length(), 0.2);

    // material
    const gazebo::msgs::Material materialMsg = retVisualMsg->material();
    QCOMPARE(materialMsg.shader_type(),
        gazebo::msgs::Material::Material::VERTEX);
    QVERIFY(materialMsg.normal_map() == "test_normal_map");
    const gazebo::msgs::Color ambientMsg = materialMsg.ambient();
    QCOMPARE(ambientMsg.r(), 0.0f);
    QCOMPARE(ambientMsg.g(), 1.0f);
    QCOMPARE(ambientMsg.b(), 0.0f);
    QCOMPARE(ambientMsg.a(), 1.0f);
    const gazebo::msgs::Color diffuseMsg = materialMsg.diffuse();
    QCOMPARE(diffuseMsg.r(), 0.0f);
    QCOMPARE(diffuseMsg.g(), 1.0f);
    QCOMPARE(diffuseMsg.b(), 1.0f);
    QCOMPARE(diffuseMsg.a(), 0.4f);
    const gazebo::msgs::Color specularMsg = materialMsg.specular();
    QCOMPARE(specularMsg.r(), 1.0f);
    QCOMPARE(specularMsg.g(), 1.0f);
    QCOMPARE(specularMsg.b(), 1.0f);
    QCOMPARE(specularMsg.a(), 0.6f);
    const gazebo::msgs::Color emissiveMsg = materialMsg.emissive();
    QCOMPARE(emissiveMsg.r(), 0.0f);
    QCOMPARE(emissiveMsg.g(), 0.5f);
    QCOMPARE(emissiveMsg.b(), 0.2f);
    QCOMPARE(emissiveMsg.a(), 1.0f);
    QCOMPARE(materialMsg.lighting(), true);
    // material::script
    const gazebo::msgs::Material::Script scriptMsg = materialMsg.script();
    QVERIFY(scriptMsg.uri(0) == "test_script_uri_0");
    QVERIFY(scriptMsg.uri(1) == "test_script_uri_1");
    QVERIFY(scriptMsg.name() == "test_script_name");
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  {
    // visual
    visualConfigWidget->SetStringWidgetValue("name", "test_visual_updated");
    visualConfigWidget->SetUIntWidgetValue("id", 11111u);
    visualConfigWidget->SetStringWidgetValue("parent_name",
        "test_visual_parent_updated");
    visualConfigWidget->SetUIntWidgetValue("parent_id", 55555u);
    visualConfigWidget->SetBoolWidgetValue("cast_shadows", false);
    visualConfigWidget->SetDoubleWidgetValue("transparency", 1.0);
    visualConfigWidget->SetBoolWidgetValue("visible", false);
    visualConfigWidget->SetBoolWidgetValue("delete_me", true);
    visualConfigWidget->SetBoolWidgetValue("is_static", true);
    visualConfigWidget->SetVector3dWidgetValue("scale",
        ignition::math::Vector3d(2.0, 1.5, 0.5));

    // pose
    ignition::math::Vector3d pos(-2.0, -3.0, -4.0);
    ignition::math::Quaterniond quat(0.0, 1.57, 0.0);
    visualConfigWidget->SetPoseWidgetValue("pose",
        ignition::math::Pose3d(pos, quat));

    // geometry
    visualConfigWidget->SetGeometryWidgetValue("geometry", "box",
        ignition::math::Vector3d(5.0, 3.0, 4.0));

    // material
    visualConfigWidget->SetStringWidgetValue("material::normal_map",
        "test_normal_map_updated");
    visualConfigWidget->SetColorWidgetValue("material::ambient",
        gazebo::common::Color(0.2, 0.3, 0.4, 0.5));
    visualConfigWidget->SetColorWidgetValue("material::diffuse",
        gazebo::common::Color(0.1, 0.8, 0.6, 0.4));
    visualConfigWidget->SetColorWidgetValue("material::specular",
        gazebo::common::Color(0.5, 0.4, 0.3, 0.2));
    visualConfigWidget->SetColorWidgetValue("material::emissive",
        gazebo::common::Color(0.4, 0.6, 0.8, 0.1));
    visualConfigWidget->SetBoolWidgetValue("material::lighting", false);
    // material::script
    visualConfigWidget->SetStringWidgetValue("material::script::name",
        "test_script_name_updated");
  }

  // verify widget values
  {
    QVERIFY(visualConfigWidget->StringWidgetValue("name") ==
        "test_visual_updated");
    QCOMPARE(visualConfigWidget->UIntWidgetValue("id"), 11111u);
    QVERIFY(visualConfigWidget->StringWidgetValue("parent_name") ==
        "test_visual_parent_updated");
    QCOMPARE(visualConfigWidget->UIntWidgetValue("parent_id"), 55555u);
    QCOMPARE(visualConfigWidget->BoolWidgetValue("cast_shadows"), false);
    QCOMPARE(visualConfigWidget->DoubleWidgetValue("transparency"), 1.0);
    QCOMPARE(visualConfigWidget->BoolWidgetValue("visible"), false);
    QCOMPARE(visualConfigWidget->BoolWidgetValue("delete_me"), true);
    QCOMPARE(visualConfigWidget->BoolWidgetValue("is_static"), true);
    QCOMPARE(visualConfigWidget->Vector3dWidgetValue("scale"),
        ignition::math::Vector3d(2.0, 1.5, 0.5));

    // pose
    ignition::math::Vector3d pos(-2.0, -3.0, -4.0);
    ignition::math::Quaterniond quat(0.0, 1.57, 0.0);
    QCOMPARE(visualConfigWidget->PoseWidgetValue("pose"),
        ignition::math::Pose3d(pos, quat));

    // geometry
    ignition::math::Vector3d dimensions;
    std::string uri;
    QVERIFY(visualConfigWidget->GeometryWidgetValue("geometry", dimensions,
        uri) == "box");
    QCOMPARE(dimensions, ignition::math::Vector3d(5.0, 3.0, 4.0));

    // material
    QVERIFY(visualConfigWidget->StringWidgetValue("material::normal_map") ==
        "test_normal_map_updated");
    QCOMPARE(visualConfigWidget->ColorWidgetValue("material::ambient"),
        gazebo::common::Color(0.2, 0.3, 0.4, 0.5));
    QCOMPARE(visualConfigWidget->ColorWidgetValue("material::diffuse"),
        gazebo::common::Color(0.1, 0.8, 0.6, 0.4));
    QCOMPARE(visualConfigWidget->ColorWidgetValue("material::specular"),
        gazebo::common::Color(0.5, 0.4, 0.3, 0.2));
    QCOMPARE(visualConfigWidget->ColorWidgetValue("material::emissive"),
        gazebo::common::Color(0.4, 0.6, 0.8, 0.1));
    QCOMPARE(visualConfigWidget->BoolWidgetValue("material::lighting"),
        false);
    // material::script
    QVERIFY(visualConfigWidget->StringWidgetValue("material::script::name")
        == "test_script_name_updated");
  }

  // verify updates in new msg
  {
    gazebo::msgs::Visual *retVisualMsg =
        dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->Msg());
    QVERIFY(retVisualMsg != NULL);

    // visual
    QVERIFY(retVisualMsg->name() == "test_visual_updated");
    QCOMPARE(retVisualMsg->id(), 11111u);
    QVERIFY(retVisualMsg->parent_name() == "test_visual_parent_updated");
    QCOMPARE(retVisualMsg->parent_id(), 55555u);
    QCOMPARE(retVisualMsg->cast_shadows(), false);
    QCOMPARE(retVisualMsg->transparency(), 1.0);
    QCOMPARE(retVisualMsg->visible(), false);
    QCOMPARE(retVisualMsg->delete_me(), true);
    QCOMPARE(retVisualMsg->is_static(), true);
    const gazebo::msgs::Vector3d scaleMsg = retVisualMsg->scale();
    QCOMPARE(scaleMsg.x(), 2.0);
    QCOMPARE(scaleMsg.y(), 1.5);
    QCOMPARE(scaleMsg.z(), 0.5);

    // pose
    const gazebo::msgs::Pose poseMsg = retVisualMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), -2.0);
    QCOMPARE(posMsg.y(), -3.0);
    QCOMPARE(posMsg.z(), -4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    ignition::math::Quaterniond quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.Euler().X(), 0.0);
    QCOMPARE(quat.Euler().Y(), 1.57);
    QCOMPARE(quat.Euler().Z(), 0.0);

    // geometry
    const gazebo::msgs::Geometry geometryMsg = retVisualMsg->geometry();
    QCOMPARE(geometryMsg.type(), gazebo::msgs::Geometry::BOX);
    const gazebo::msgs::BoxGeom boxGeomMsg = geometryMsg.box();
    const gazebo::msgs::Vector3d boxGeomSizeMsg = boxGeomMsg.size();
    QCOMPARE(boxGeomSizeMsg.x(), 5.0);
    QCOMPARE(boxGeomSizeMsg.y(), 3.0);
    QCOMPARE(boxGeomSizeMsg.z(), 4.0);

    // material
    const gazebo::msgs::Material materialMsg = retVisualMsg->material();
    QCOMPARE(materialMsg.shader_type(),
        gazebo::msgs::Material::Material::VERTEX);
    QVERIFY(materialMsg.normal_map() == "test_normal_map_updated");
    const gazebo::msgs::Color ambientMsg = materialMsg.ambient();
    QCOMPARE(ambientMsg.r(), 0.2f);
    QCOMPARE(ambientMsg.g(), 0.3f);
    QCOMPARE(ambientMsg.b(), 0.4f);
    QCOMPARE(ambientMsg.a(), 0.5f);
    const gazebo::msgs::Color diffuseMsg = materialMsg.diffuse();
    QCOMPARE(diffuseMsg.r(), 0.1f);
    QCOMPARE(diffuseMsg.g(), 0.8f);
    QCOMPARE(diffuseMsg.b(), 0.6f);
    QCOMPARE(diffuseMsg.a(), 0.4f);
    const gazebo::msgs::Color specularMsg = materialMsg.specular();
    QCOMPARE(specularMsg.r(), 0.5f);
    QCOMPARE(specularMsg.g(), 0.4f);
    QCOMPARE(specularMsg.b(), 0.3f);
    QCOMPARE(specularMsg.a(), 0.2f);
    const gazebo::msgs::Color emissiveMsg = materialMsg.emissive();
    QCOMPARE(emissiveMsg.r(), 0.4f);
    QCOMPARE(emissiveMsg.g(), 0.6f);
    QCOMPARE(emissiveMsg.b(), 0.8f);
    QCOMPARE(emissiveMsg.a(), 0.1f);
    QCOMPARE(materialMsg.lighting(), false);
    // material::script
    const gazebo::msgs::Material::Script scriptMsg = materialMsg.script();
    QVERIFY(scriptMsg.uri(0) == "test_script_uri_0");
    QVERIFY(scriptMsg.uri(1) == "test_script_uri_1");
    QVERIFY(scriptMsg.name() == "test_script_name_updated");
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::PluginMsgWidget()
{
  // create a plugin message with test values

  gazebo::gui::ConfigWidget *pluginConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Plugin pluginMsg;

  {
    // plugin
    pluginMsg.set_name("test_plugin");
    pluginMsg.set_filename("test_plugin_filename");
    pluginMsg.set_innerxml("<param>1</param>\n");
  }
  pluginConfigWidget->Load(&pluginMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Plugin *retPluginMsg =
        dynamic_cast<gazebo::msgs::Plugin *>(pluginConfigWidget->Msg());
    QVERIFY(retPluginMsg != NULL);

    // plugin
    QVERIFY(retPluginMsg->name() == "test_plugin");
    QVERIFY(retPluginMsg->filename() == "test_plugin_filename");
    QVERIFY(retPluginMsg->innerxml() == "<param>1</param>\n");
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  {
    // plugin
    pluginConfigWidget->SetStringWidgetValue("name", "test_plugin_updated");
    pluginConfigWidget->SetStringWidgetValue("filename",
        "test_plugin_filename_updated");
    pluginConfigWidget->SetStringWidgetValue("innerxml",
        "<param2>new_param</param2>\n");
  }

  // verify widget values
  {
    QVERIFY(pluginConfigWidget->StringWidgetValue("name") ==
        "test_plugin_updated");
    QVERIFY(pluginConfigWidget->StringWidgetValue("filename") ==
        "test_plugin_filename_updated");
    QVERIFY(pluginConfigWidget->StringWidgetValue("innerxml") ==
        "<param2>new_param</param2>\n");
  }

  // verify updates in new msg
  {
    gazebo::msgs::Plugin *retPluginMsg =
        dynamic_cast<gazebo::msgs::Plugin *>(pluginConfigWidget->Msg());
    QVERIFY(retPluginMsg != NULL);

    // plugin
    QVERIFY(retPluginMsg->name() == "test_plugin_updated");
    QVERIFY(retPluginMsg->filename() == "test_plugin_filename_updated");
    QVERIFY(retPluginMsg->innerxml() == "<param2>new_param</param2>\n");
  }

  delete pluginConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ConfigWidgetVisible()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_id(12345u);

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));

    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);
  visualConfigWidget->show();

  // set different types of widgets to be not visibile
  {
    // primitive widget
    visualConfigWidget->SetWidgetVisible("id", false);
    // custom pose message widget
    visualConfigWidget->SetWidgetVisible("pose", false);
    // custom geometry message widget
    visualConfigWidget->SetWidgetVisible("geometry", false);
    // widget inside a group widget
    visualConfigWidget->SetWidgetVisible("material::diffuse", false);
    // widget two levels deep
    visualConfigWidget->SetWidgetVisible("material::script::name", false);
    // group widget
    visualConfigWidget->SetWidgetVisible("material", false);

    QCOMPARE(visualConfigWidget->WidgetVisible("id"), false);
    QCOMPARE(visualConfigWidget->WidgetVisible("pose"), false);
    QCOMPARE(visualConfigWidget->WidgetVisible("geometry"), false);
    QCOMPARE(visualConfigWidget->WidgetVisible("material::diffuse"), false);
    QCOMPARE(visualConfigWidget->WidgetVisible("material::script::name"),
        false);
    QCOMPARE(visualConfigWidget->WidgetVisible("material"), false);
  }

  // set visible back to true
  {
    visualConfigWidget->SetWidgetVisible("id", true);
    visualConfigWidget->SetWidgetVisible("pose", true);
    visualConfigWidget->SetWidgetVisible("geometry", true);
    visualConfigWidget->SetWidgetVisible("material::diffuse", true);
    visualConfigWidget->SetWidgetVisible("material::script::name", true);
    visualConfigWidget->SetWidgetVisible("material", true);

    QCOMPARE(visualConfigWidget->WidgetVisible("id"), true);
    QCOMPARE(visualConfigWidget->WidgetVisible("pose"), true);
    QCOMPARE(visualConfigWidget->WidgetVisible("geometry"), true);
    QCOMPARE(visualConfigWidget->WidgetVisible("material::diffuse"), true);
    QCOMPARE(visualConfigWidget->WidgetVisible("material::script::name"),
        true);
    QCOMPARE(visualConfigWidget->WidgetVisible("material"), true);
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::CustomConfigWidgetReadOnly()
{
  auto configWidget = new gazebo::gui::ConfigWidget;
  auto configLayout = new QVBoxLayout();

  // Create a child widget
  {
    auto vecWidget = configWidget->CreateVector3dWidget("vector3d", 0);
    auto stringWidget = configWidget->CreateStringWidget("string", 0);

    QVERIFY(configWidget->AddConfigChildWidget("vector3d", vecWidget));
    QVERIFY(configWidget->AddConfigChildWidget("string", stringWidget));

    auto *childLayout = new QVBoxLayout();
    childLayout->addWidget(vecWidget);
    childLayout->addWidget(stringWidget);

    auto childWidget = new gazebo::gui::ConfigChildWidget();
    childWidget->setLayout(childLayout);

    auto groupWidget = configWidget->CreateGroupWidget("group",
        childWidget, 0);

    configLayout->addWidget(groupWidget);
  }

  // Create a custom child widget
  {
    auto customLabel = new QLabel("custom label");
    customLabel->setObjectName("label");
    auto custom1 = new QToolButton();
    auto custom2 = new QDoubleSpinBox();
    auto custom3 = new QLineEdit();

    auto customLayout = new QGridLayout();
    customLayout->addWidget(customLabel, 0, 0);
    customLayout->addWidget(custom1, 0, 1);
    customLayout->addWidget(custom2, 0, 2);
    customLayout->addWidget(custom3, 0, 3);

    auto customLayout2 = new QVBoxLayout();
    customLayout2->addLayout(customLayout);

    auto customWidget = new gazebo::gui::ConfigChildWidget();
    customWidget->setLayout(customLayout2);

    QVERIFY(configWidget->AddConfigChildWidget("custom", customWidget));

    auto customGroupWidget = configWidget->CreateGroupWidget("custom group",
        customWidget, 0);

    configLayout->addWidget(customGroupWidget);
  }
  configWidget->setLayout(configLayout);

  // set to be read-only
  {
    configWidget->SetWidgetReadOnly("vector3d", true);
    configWidget->SetWidgetReadOnly("string", true);
    configWidget->SetWidgetReadOnly("custom", true);

    QCOMPARE(configWidget->WidgetReadOnly("vector3d"), true);
    QCOMPARE(configWidget->WidgetReadOnly("string"), true);
    QCOMPARE(configWidget->WidgetReadOnly("custom"), true);
    {
      auto childWidgets =
          configWidget->ConfigChildWidgetByName("custom")->
          findChildren<QWidget *>();
      QCOMPARE(childWidgets.size(), 5);
      for (auto it : childWidgets)
        QCOMPARE(it->isEnabled(), false);
    }
  }

  // set read-only back to false
  {
    configWidget->SetWidgetReadOnly("vector3d", false);
    configWidget->SetWidgetReadOnly("string", false);
    configWidget->SetWidgetReadOnly("custom", false);

    QCOMPARE(configWidget->WidgetReadOnly("vector3d"), false);
    QCOMPARE(configWidget->WidgetReadOnly("string"), false);
    QCOMPARE(configWidget->WidgetReadOnly("custom"), false);
    {
      auto childWidgets =
          configWidget->ConfigChildWidgetByName("custom")->
          findChildren<QWidget *>();
      QCOMPARE(childWidgets.size(), 5);
      for (auto it : childWidgets)
        QCOMPARE(it->isEnabled(), true);
    }
  }

  // set read-only back to true
  {
    configWidget->SetWidgetReadOnly("vector3d", true);
    configWidget->SetWidgetReadOnly("string", true);
    configWidget->SetWidgetReadOnly("custom", true);

    QCOMPARE(configWidget->WidgetReadOnly("vector3d"), true);
    QCOMPARE(configWidget->WidgetReadOnly("string"), true);
    QCOMPARE(configWidget->WidgetReadOnly("custom"), true);
    {
      auto childWidgets =
          configWidget->ConfigChildWidgetByName("custom")->
          findChildren<QWidget *>();
      QCOMPARE(childWidgets.size(), 5);
      for (auto it : childWidgets)
        QCOMPARE(it->isEnabled(), false);
    }
  }

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ConfigWidgetReadOnly()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_id(12345u);

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));

    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);

  // set different types of widgets to be read-only
  {
    // primitive widget
    visualConfigWidget->SetWidgetReadOnly("id", true);
    // custom pose message widget
    visualConfigWidget->SetWidgetReadOnly("pose", true);
    // custom geometry message widget
    visualConfigWidget->SetWidgetReadOnly("geometry", true);
    // widget inside a group widget
    visualConfigWidget->SetWidgetReadOnly("material::diffuse", true);
    // widget two levels deep
    visualConfigWidget->SetWidgetReadOnly("material::script::name", true);
    // group widget
    visualConfigWidget->SetWidgetReadOnly("material", true);

    QCOMPARE(visualConfigWidget->WidgetReadOnly("id"), true);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("pose"), true);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("geometry"), true);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material::diffuse"), true);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material::script::name"),
        true);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material"), true);
  }

  // set read-only back to false
  {
    visualConfigWidget->SetWidgetReadOnly("id", false);
    visualConfigWidget->SetWidgetReadOnly("pose", false);
    visualConfigWidget->SetWidgetReadOnly("geometry", false);
    visualConfigWidget->SetWidgetReadOnly("material::diffuse", false);
    visualConfigWidget->SetWidgetReadOnly("material::script::name", false);
    visualConfigWidget->SetWidgetReadOnly("material", false);

    QCOMPARE(visualConfigWidget->WidgetReadOnly("id"), false);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("pose"), false);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("geometry"), false);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material::diffuse"), false);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material::script::name"),
        false);
    QCOMPARE(visualConfigWidget->WidgetReadOnly("material"), false);
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::CreatedExternally()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create predefined child widgets
  gazebo::gui::ConfigChildWidget *uintWidget =
      configWidget->CreateUIntWidget("uint", 0);
  gazebo::gui::ConfigChildWidget *intWidget =
      configWidget->CreateIntWidget("int", 0);
  gazebo::gui::ConfigChildWidget *doubleWidget =
      configWidget->CreateDoubleWidget("double", 1);
  gazebo::gui::ConfigChildWidget *stringWidget =
      configWidget->CreateStringWidget("string", 1);
  gazebo::gui::ConfigChildWidget *boolWidget =
      configWidget->CreateBoolWidget("bool", 2);
  gazebo::gui::ConfigChildWidget *vector3dWidget =
      configWidget->CreateVector3dWidget("vector3d", 2);
  gazebo::gui::ConfigChildWidget *colorWidget =
      configWidget->CreateColorWidget("color", 3);
  gazebo::gui::ConfigChildWidget *poseWidget =
      configWidget->CreatePoseWidget("pose", 3);

  std::vector<std::string> enumValues;
  enumValues.push_back("value1");
  enumValues.push_back("value2");
  enumValues.push_back("value3");
  gazebo::gui::ConfigChildWidget *enumWidget =
      configWidget->CreateEnumWidget("enum", enumValues, 4);

  QVERIFY(uintWidget != NULL);
  QVERIFY(intWidget != NULL);
  QVERIFY(doubleWidget != NULL);
  QVERIFY(stringWidget != NULL);
  QVERIFY(boolWidget != NULL);
  QVERIFY(vector3dWidget != NULL);
  QVERIFY(colorWidget != NULL);
  QVERIFY(poseWidget != NULL);
  QVERIFY(enumWidget != NULL);

  // Create a custom child widget
  QLabel *customLabel = new QLabel("custom label");
  QLineEdit *customLineEdit = new QLineEdit();
  QHBoxLayout *customLayout = new QHBoxLayout();
  customLayout->addWidget(customLabel);
  customLayout->addWidget(customLineEdit);

  gazebo::gui::ConfigChildWidget *customWidget =
      new gazebo::gui::ConfigChildWidget();
  customWidget->setLayout(customLayout);
  customWidget->widgets.push_back(customLineEdit);

  // Add child widgets to config widget
  QCOMPARE(configWidget->ConfigChildWidgetCount(), 0u);

  QVERIFY(configWidget->AddConfigChildWidget("uint", uintWidget));
  QVERIFY(configWidget->AddConfigChildWidget("int", intWidget));
  QVERIFY(configWidget->AddConfigChildWidget("double", doubleWidget));
  QVERIFY(configWidget->AddConfigChildWidget("string", stringWidget));
  QVERIFY(configWidget->AddConfigChildWidget("bool", boolWidget));
  QVERIFY(configWidget->AddConfigChildWidget("vector3d", vector3dWidget));
  QVERIFY(configWidget->AddConfigChildWidget("color", colorWidget));
  QVERIFY(configWidget->AddConfigChildWidget("pose", poseWidget));
  QVERIFY(configWidget->AddConfigChildWidget("enum", enumWidget));
  QVERIFY(configWidget->AddConfigChildWidget("custom", customWidget));

  QCOMPARE(configWidget->ConfigChildWidgetCount(), 10u);

  // Fail to add invalid children
  QCOMPARE(configWidget->AddConfigChildWidget("", uintWidget), false);
  QCOMPARE(configWidget->AddConfigChildWidget("validName", NULL), false);
  QCOMPARE(configWidget->AddConfigChildWidget("uint", intWidget), false);

  QCOMPARE(configWidget->ConfigChildWidgetCount(), 10u);

  // Check that checking visibility works
  QCOMPARE(configWidget->WidgetVisible("uint"), uintWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("int"), intWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("double"), doubleWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("string"), stringWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("bool"), boolWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("vector3d"),
      vector3dWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("color"), colorWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("pose"), poseWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("enum"), enumWidget->isVisible());
  QCOMPARE(configWidget->WidgetVisible("custom"), customWidget->isVisible());

  // Set widgets values
  unsigned int uintValue = 123;
  int intValue = -456;
  double doubleValue = 123.456;
  std::string stringValue("123");
  bool boolValue = true;
  ignition::math::Vector3d vector3dValue(1, 2, 3);
  gazebo::common::Color colorValue(0.1, 0.2, 0.3, 0.4);
  ignition::math::Pose3d poseValue(1, 2, 3, 0.1, 0.2, 0.3);
  std::string enumValue("value2");
  std::string customValue("123456789");

  QVERIFY(configWidget->SetUIntWidgetValue("uint", uintValue));
  QVERIFY(configWidget->SetIntWidgetValue("int", intValue));
  QVERIFY(configWidget->SetDoubleWidgetValue("double", doubleValue));
  QVERIFY(configWidget->SetStringWidgetValue("string", stringValue));
  QVERIFY(configWidget->SetBoolWidgetValue("bool", boolValue));
  QVERIFY(configWidget->SetVector3dWidgetValue("vector3d",
      ignition::math::Vector3d(vector3dValue)));
  QVERIFY(configWidget->SetColorWidgetValue("color", colorValue));
  QVERIFY(configWidget->SetPoseWidgetValue("pose", poseValue));
  QVERIFY(configWidget->SetEnumWidgetValue("enum", enumValue));
  QVERIFY(configWidget->SetStringWidgetValue("custom", customValue));

  // Get widgets values
  QCOMPARE(configWidget->UIntWidgetValue("uint"), uintValue);
  QCOMPARE(configWidget->IntWidgetValue("int"), intValue);
  QCOMPARE(configWidget->DoubleWidgetValue("double"), doubleValue);
  QCOMPARE(configWidget->StringWidgetValue("string"), stringValue);
  QCOMPARE(configWidget->BoolWidgetValue("bool"), boolValue);
  QCOMPARE(configWidget->Vector3dWidgetValue("vector3d"),
      ignition::math::Vector3d(vector3dValue));
  QCOMPARE(configWidget->ColorWidgetValue("color"), colorValue);
  QCOMPARE(configWidget->PoseWidgetValue("pose"),
      ignition::math::Pose3d(poseValue));
  QCOMPARE(configWidget->EnumWidgetValue("enum"), enumValue);
  QCOMPARE(configWidget->StringWidgetValue("custom"), customValue);

  // Group some widgets
  QVBoxLayout *groupLayout = new QVBoxLayout();
  groupLayout->addWidget(uintWidget);
  groupLayout->addWidget(intWidget);
  groupLayout->addWidget(doubleWidget);

  QGroupBox *groupBox = new QGroupBox();
  groupBox->setLayout(groupLayout);

  QVBoxLayout *groupChildWidgetLayout = new QVBoxLayout();
  groupChildWidgetLayout->addWidget(groupBox);

  gazebo::gui::ConfigChildWidget *groupChildWidget =
      new gazebo::gui::ConfigChildWidget();
  groupChildWidget->setLayout(groupChildWidgetLayout);
  groupChildWidget->widgets.push_back(groupBox);

  gazebo::gui::GroupWidget *groupWidget =
      configWidget->CreateGroupWidget("groupWidget", groupChildWidget, 0);
  QVERIFY(groupWidget != NULL);
  QVERIFY(groupWidget->childWidget != NULL);
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::EnumConfigWidget()
{
  // Create a parent widget
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget();
  QVERIFY(configWidget != NULL);

  // Create an enum child widget
  std::vector<std::string> enumValues;
  enumValues.push_back("value1");
  enumValues.push_back("value2");
  enumValues.push_back("value3");
  gazebo::gui::ConfigChildWidget *enumWidget =
      configWidget->CreateEnumWidget("Enum Label", enumValues);

  QVERIFY(enumWidget != NULL);

  // Add it to parent
  QVERIFY(configWidget->AddConfigChildWidget("enumWidgetName", enumWidget));

  // Check that all items can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value1"));
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2"));
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value3"));

  // Check that an inexistent item cannot be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4")
      == false);

  // Check the number of items
  QComboBox *comboBox = enumWidget->findChild<QComboBox *>();
  QVERIFY(comboBox != NULL);
  QCOMPARE(comboBox->count(), 3);

  // Add an item and check count
  QVERIFY(configWidget->AddItemEnumWidget("enumWidgetName", "value4"));
  QCOMPARE(comboBox->count(), 4);

  // Check that the new item can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4"));

  // Remove an item and check count
  QVERIFY(configWidget->RemoveItemEnumWidget("enumWidgetName", "value2"));
  QCOMPARE(comboBox->count(), 3);

  // Check that the removed item cannot be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2")
      == false);

  // Clear all items and check count
  QVERIFY(configWidget->ClearEnumWidget("enumWidgetName"));
  QCOMPARE(comboBox->count(), 0);

  // Check that none of the previous items can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value1")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value3")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4")
      == false);
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildUIntSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child uint widget
  gazebo::gui::ConfigChildWidget *uintWidget =
      configWidget->CreateUIntWidget("uint");
  QVERIFY(uintWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("uint", uintWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(UIntValueChanged(const QString, uint)),
      this,
      SLOT(OnUIntValueChanged(const QString, uint)));

  // Check default uint
  QVERIFY(configWidget->UIntWidgetValue("uint") == 0u);

  // Get signal emitting widgets
  QList<QSpinBox *> spins = uintWidget->findChildren<QSpinBox *>();
  QCOMPARE(spins.size(), 1);

  // Change the value and check new uint at OnUIntValueChanged
  spins[0]->setValue(3);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_uIntSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnUIntValueChanged(const QString &_name,
    const unsigned int _uint)
{
  QVERIFY(_name == "uint");
  QVERIFY(_uint == 3);
  g_uIntSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildIntSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child int widget
  gazebo::gui::ConfigChildWidget *intWidget =
      configWidget->CreateIntWidget("int");
  QVERIFY(intWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("int", intWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(IntValueChanged(const QString, int)),
      this,
      SLOT(OnIntValueChanged(const QString, int)));

  // Check default int
  QCOMPARE(configWidget->IntWidgetValue("int"), 0);

  // Get signal emitting widgets
  QList<QSpinBox *> spins = intWidget->findChildren<QSpinBox *>();
  QCOMPARE(spins.size(), 1);

  // Change the value and check new int at OnIntValueChanged
  spins[0]->setValue(-2);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_intSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnIntValueChanged(const QString &_name,
    const int _int)
{
  QVERIFY(_name == "int");
  QVERIFY(_int == -2);
  g_intSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildDoubleSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child double widget
  gazebo::gui::ConfigChildWidget *doubleWidget =
      configWidget->CreateDoubleWidget("double");
  QVERIFY(doubleWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("double", doubleWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(DoubleValueChanged(const QString, double)),
      this,
      SLOT(OnDoubleValueChanged(const QString, double)));

  // Check default double
  QCOMPARE(configWidget->DoubleWidgetValue("double"), 0.0);

  // Get signal emitting widgets
  QList<QDoubleSpinBox *> spins =
      doubleWidget->findChildren<QDoubleSpinBox *>();
  QCOMPARE(spins.size(), 1);

  // Change the value and check new double at OnDoubleValueChanged
  spins[0]->setValue(1.5);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_doubleSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnDoubleValueChanged(const QString &_name,
    const double _double)
{
  QVERIFY(_name == "double");
  QVERIFY(fabs(_double - 1.5) < 0.00001);
  g_doubleSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildBoolSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child bool widget
  gazebo::gui::ConfigChildWidget *boolWidget =
      configWidget->CreateBoolWidget("bool");
  QVERIFY(boolWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("bool", boolWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(BoolValueChanged(const QString, bool)),
      this,
      SLOT(OnBoolValueChanged(const QString, bool)));

  // Check default bool
  QCOMPARE(configWidget->BoolWidgetValue("bool"), false);

  // Get signal emitting widgets
  QList<QRadioButton *> radios =
      boolWidget->findChildren<QRadioButton *>();
  QCOMPARE(radios.size(), 2);

  // Change the value and check new bool at OnBoolValueChanged
  radios[0]->setChecked(true);
  radios[1]->setChecked(false);
  QTest::keyClick(radios[0], Qt::Key_Enter);
  QVERIFY(g_boolSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnBoolValueChanged(const QString &_name,
    const bool _bool)
{
  QVERIFY(_name == "bool");
  QVERIFY(_bool == true);
  g_boolSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildStringSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child string widget
  gazebo::gui::ConfigChildWidget *stringWidget =
      configWidget->CreateStringWidget("string");
  QVERIFY(stringWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("string", stringWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(StringValueChanged(const QString, std::string)),
      this,
      SLOT(OnStringValueChanged(const QString, std::string)));

  // Check default string
  QVERIFY(configWidget->StringWidgetValue("string") == "");

  // Get signal emitting widgets
  QList<QLineEdit *> lineEdits =
      stringWidget->findChildren<QLineEdit *>();
  QCOMPARE(lineEdits.size(), 1);

  // Change the value and check new string at OnStringValueChanged
  lineEdits[0]->setText("new text");
  QTest::keyClick(lineEdits[0], Qt::Key_Enter);
  QVERIFY(g_stringSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnStringValueChanged(const QString &_name,
    const std::string &_string)
{
  QVERIFY(_name == "string");
  QVERIFY(_string == "new text");
  g_stringSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildVector3dSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child vector3 widget
  gazebo::gui::ConfigChildWidget *vector3Widget =
      configWidget->CreateVector3dWidget("vector3");
  QVERIFY(vector3Widget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("vector3", vector3Widget));

  // Connect signals
  connect(configWidget,
      SIGNAL(Vector3dValueChanged(const QString, ignition::math::Vector3d)),
      this,
      SLOT(OnVector3dValueChanged(const QString, ignition::math::Vector3d)));

  // Check default vector3
  QVERIFY(configWidget->Vector3dWidgetValue("vector3") ==
      ignition::math::Vector3d());

  // Get axes spins
  QList<QDoubleSpinBox *> spins =
      vector3Widget->findChildren<QDoubleSpinBox *>();
  QCOMPARE(spins.size(), 3);

  // Get preset combo
  auto combos = vector3Widget->findChildren<QComboBox *>();
  QCOMPARE(combos.size(), 1);

  // Change the X value and check new vector3 at OnVector3dValueChanged
  QVERIFY(g_vector3SignalCount == 0);
  spins[0]->setValue(2.5);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_vector3SignalCount == 1);

  // Change the preset value and check new vector3 at OnVector3dValueChanged
  combos[0]->setCurrentIndex(4);
  QTest::keyClick(combos[0], Qt::Key_Enter);
  QVERIFY(g_vector3SignalCount == 2);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnVector3dValueChanged(const QString &_name,
    const ignition::math::Vector3d &_vector3)
{
  QVERIFY(_name == "vector3");

  // From spins
  if (g_vector3SignalCount == 0)
  {
    QVERIFY(_vector3 == ignition::math::Vector3d(2.5, 0, 0));
    g_vector3SignalCount++;
  }
  // From preset combo
  else if (g_vector3SignalCount == 1)
  {
    QVERIFY(_vector3 == ignition::math::Vector3d(0, -1, 0));
    g_vector3SignalCount++;
  }
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildColorSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child color widget
  gazebo::gui::ConfigChildWidget *colorWidget =
      configWidget->CreateColorWidget("color");
  QVERIFY(colorWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("color", colorWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(ColorValueChanged(const QString, const gazebo::common::Color)),
      this,
      SLOT(OnColorValueChanged(const QString, const gazebo::common::Color)));

  // Check default color
  QVERIFY(configWidget->ColorWidgetValue("color") ==
      gazebo::common::Color());

  // Get signal emitting widgets
  QList<QDoubleSpinBox *> spins =
      colorWidget->findChildren<QDoubleSpinBox *>();
  QCOMPARE(spins.size(), 4);

  // Change the X value and check new color at OnColorValueChanged
  spins[0]->setValue(0.5);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_colorSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnColorValueChanged(const QString &_name,
    const gazebo::common::Color &_color)
{
  QVERIFY(_name == "color");
  QVERIFY(_color == gazebo::common::Color(0.5, 0.0, 0.0, 0.0));
  g_colorSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildPoseSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child pose widget
  gazebo::gui::ConfigChildWidget *poseWidget =
      configWidget->CreatePoseWidget("pose");
  QVERIFY(poseWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("pose", poseWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(PoseValueChanged(const QString, const ignition::math::Pose3d)),
      this,
      SLOT(OnPoseValueChanged(const QString, const ignition::math::Pose3d)));

  // Check default pose
  QVERIFY(configWidget->PoseWidgetValue("pose") == ignition::math::Pose3d());

  // Get signal emitting widgets
  QList<QDoubleSpinBox *> spins = poseWidget->findChildren<QDoubleSpinBox *>();
  QCOMPARE(spins.size(), 6);

  // Change the X value and check new pose at OnPoseValueChanged
  spins[0]->setValue(1.0);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QVERIFY(g_poseSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnPoseValueChanged(const QString &_name,
    const ignition::math::Pose3d &_value)
{
  QVERIFY(_name == "pose");
  QVERIFY(_value == ignition::math::Pose3d(1, 0, 0, 0, 0, 0));
  g_poseSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildGeometrySignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child widget
  gazebo::gui::ConfigChildWidget *geometryWidget =
      configWidget->CreateGeometryWidget("geometry");
  QVERIFY(geometryWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("geometry", geometryWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(GeometryValueChanged(const std::string &, const std::string &,
      const ignition::math::Vector3d &, const std::string &)),
      this,
      SLOT(OnGeometryValueChanged(const std::string &, const std::string &,
      const ignition::math::Vector3d &, const std::string &)));

  // Check default
  ignition::math::Vector3d dimensions;
  std::string uri;
  std::string value = configWidget->GeometryWidgetValue("geometry",
      dimensions, uri);
  QVERIFY(value == "box");
  QVERIFY(dimensions == ignition::math::Vector3d(1, 1, 1));
  QVERIFY(uri == "");

  // Get signal emitting widgets
  QList<QDoubleSpinBox *> spins =
      geometryWidget->findChildren<QDoubleSpinBox *>();
  QCOMPARE(spins.size(), 5);

  // Change the value and check new pose at OnGeometryValueChanged
  spins[2]->setValue(2.0);
  QTest::keyClick(spins[2], Qt::Key_Enter);
  QVERIFY(g_geometrySignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnGeometryValueChanged(const std::string &_name,
    const std::string &_value, const ignition::math::Vector3d &_dimensions,
    const std::string &_uri)
{
  QVERIFY(_name == "geometry");
  QVERIFY(_value == "box");
  QVERIFY(_dimensions == ignition::math::Vector3d(2, 1, 1));
  QVERIFY(_uri == "");
  g_geometrySignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ChildEnumSignal()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create child pose widget
  std::vector<std::string> enumValues;
  enumValues.push_back("value1");
  enumValues.push_back("value2");
  enumValues.push_back("value3");
  gazebo::gui::ConfigChildWidget *enumWidget =
      configWidget->CreateEnumWidget("enum", enumValues);
  QVERIFY(enumWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("enum", enumWidget));

  // Connect signals
  connect(configWidget,
      SIGNAL(EnumValueChanged(const QString, const QString)),
      this,
      SLOT(OnEnumValueChanged(const QString, const QString)));

  // Check default pose
  QVERIFY(configWidget->EnumWidgetValue("enum") == "value1");

  // Get signal emitting widgets
  QList<QComboBox *> comboBoxes = enumWidget->findChildren<QComboBox *>();
  QCOMPARE(comboBoxes.size(), 1);

  // Change the value and check new pose at OnPoseValueChanged
  comboBoxes[0]->setCurrentIndex(2);
  QTest::keyClick(comboBoxes[0], Qt::Key_Enter);
  QVERIFY(g_enumSignalReceived == true);

  delete configWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::OnEnumValueChanged(const QString &_name,
    const QString &_value)
{
  QVERIFY(_name == "enum");
  QVERIFY(_value == "value3");
  g_enumSignalReceived = true;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::GetChildWidgetByName()
{
  // Create config widget and check it has no children
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;
  QVERIFY(configWidget != NULL);
  QCOMPARE(configWidget->ConfigChildWidgetCount(), 0u);

  // Try to get a child widget by name
  gazebo::gui::ConfigChildWidget *widget =
      configWidget->ConfigChildWidgetByName("child_widget");
  QVERIFY(widget == NULL);

  widget = configWidget->ConfigChildWidgetByName("");
  QVERIFY(widget == NULL);

  // Create child widget
  gazebo::gui::ConfigChildWidget *childWidget =
      configWidget->CreateBoolWidget("child_widget");
  QVERIFY(childWidget != NULL);

  // Add to config widget
  QVERIFY(configWidget->AddConfigChildWidget("child_widget", childWidget));
  QCOMPARE(configWidget->ConfigChildWidgetCount(), 1u);

  // Get the widget by name
  widget = configWidget->ConfigChildWidgetByName("child_widget");
  QVERIFY(widget != NULL);

  // Check that a bad name returns NULL
  widget = configWidget->ConfigChildWidgetByName("bad_name");
  QVERIFY(widget == NULL);

  delete configWidget;
}

// Generate a main function for the test
QTEST_MAIN(ConfigWidget_TEST)
