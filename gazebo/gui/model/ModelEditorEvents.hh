/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MODEL_MODELEDITOREVENTS_HH_
#define GAZEBO_GUI_MODEL_MODELEDITOREVENTS_HH_

#include <map>
#include <string>
#include <sdf/sdf.hh>

#include "gazebo/common/Event.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    namespace model
    {
      class GZ_GUI_VISIBLE Events
      {
        /// \brief Connect a boost::slot to the finish model signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectFinishModel(T _subscriber)
          { return finishModel.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the save signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveModelEditor(T _subscriber)
          { return saveModelEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the save as signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveAsModelEditor(T _subscriber)
          { return saveAsModelEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the new signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectNewModelEditor(T _subscriber)
          { return newModelEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the exit signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectExitModelEditor(T _subscriber)
          { return exitModelEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the model changed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelChanged(T _subscriber)
          { return modelChanged.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the name changed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelNameChanged(T _subscriber)
          { return modelNameChanged.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the model properties changed
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectModelPropertiesChanged(T _subscriber)
          { return modelPropertiesChanged.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the save model signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveModel(T _subscriber)
          { return saveModel.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the new model signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectNewModel(T _subscriber)
          { return newModel.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request nested model removal
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestNestedModelRemoval(
            T _subscriber)
          { return requestNestedModelRemoval.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request nested model insertion
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestNestedModelInsertion(
            T _subscriber)
          { return requestNestedModelInsertion.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request link scale signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestLinkScale(
            T _subscriber)
          { return requestLinkScale.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request link move signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestLinkMove(
            T _subscriber)
          { return requestLinkMove.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request nested model move
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestNestedModelMove(
            T _subscriber)
          { return requestNestedModelMove.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the nested model inserted signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectNestedModelInserted(T _subscriber)
          { return nestedModelInserted.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the link inserted signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectLinkInserted(T _subscriber)
          { return linkInserted.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the joint inserted signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectJointInserted(T _subscriber)
          { return jointInserted.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the joint changed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectJointChanged(T _subscriber)
          { return jointChanged.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the nested model removed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectNestedModelRemoved(T _subscriber)
          { return nestedModelRemoved.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request link insertion signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestLinkInsertion(
            T _subscriber)
          { return requestLinkInsertion.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request link removal signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestLinkRemoval(
            T _subscriber)
          { return requestLinkRemoval.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the link removed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectLinkRemoved(T _subscriber)
          { return linkRemoved.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request joint removal signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestJointRemoval(
            T _subscriber)
          { return requestJointRemoval.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request joint insertion signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestJointInsertion(
            T _subscriber)
          { return requestJointInsertion.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the joint removed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectJointRemoved(T _subscriber)
          { return jointRemoved.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the open link inspector signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectOpenLinkInspector(T _subscriber)
          { return openLinkInspector.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the open model plugin inspector
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectOpenModelPluginInspector(
            T _subscriber)
          { return openModelPluginInspector.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the open joint inspector signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectOpenJointInspector(T _subscriber)
          { return openJointInspector.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the joint name changed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectJointNameChanged(T _subscriber)
          { return jointNameChanged.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the show link context menu signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectShowLinkContextMenu(T _subscriber)
          { return showLinkContextMenu.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the show joint context menu signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectShowJointContextMenu(T _subscriber)
          { return showJointContextMenu.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the show model plugin context menu
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectShowModelPluginContextMenu(T _subscriber)
          { return showModelPluginContextMenu.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the set selected entity signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectSetSelectedEntity(T _subscriber)
          { return setSelectedEntity.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the set selected joint signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectSetSelectedJoint(T _subscriber)
          { return setSelectedJoint.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the set selected model plugin
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T> static event::ConnectionPtr
            ConnectSetSelectedModelPlugin(T _subscriber)
          { return setSelectedModelPlugin.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the model plugin inserted signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelPluginInserted(
            T _subscriber)
          { return modelPluginInserted.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the model plugin removed signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelPluginRemoved(
            T _subscriber)
          { return modelPluginRemoved.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request model plugin removal
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestModelPluginRemoval(
            T _subscriber)
          { return requestModelPluginRemoval.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the request model plugin insertion
        /// signal.
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectRequestModelPluginInsertion(
            T _subscriber)
          { return requestModelPluginInsertion.Connect(_subscriber); }

        /// \brief A model has been completed and uploaded onto the server.
        public: static event::EventT<void ()> finishModel;

        /// \brief Request to save the model.
        public: static event::EventT<bool ()> saveModelEditor;

        /// \brief Request to save the model as.
        public: static event::EventT<bool ()> saveAsModelEditor;

        /// \brief Request to start a new model.
        public: static event::EventT<void ()> newModelEditor;

        /// \brief Request to exit the editor.
        public: static event::EventT<void ()> exitModelEditor;

        /// \brief Model has been changed.
        public: static event::EventT<void ()> modelChanged;

        /// \brief Name was changed in the editor palette.
        public: static event::EventT<void (std::string)> modelNameChanged;

        /// \brief Notify that model properties have been changed.
        // The properties are: is_static, auto_disable.
        public: static event::EventT<void (bool, bool)> modelPropertiesChanged;

        /// \brief Notify that model has been saved.
        public: static event::EventT<void (std::string)> saveModel;

        /// \brief Notify that model has been newed.
        public: static event::EventT<void ()> newModel;

        /// \brief Notify that a nested model has been inserted.
        public: static event::EventT<void (std::string)> nestedModelInserted;

        /// \brief Notify that a link has been inserted.
        public: static event::EventT<void (std::string)> linkInserted;

        /// \brief Notify that a nested model has been removed.
        public: static event::EventT<void (std::string)> nestedModelRemoved;

        /// \brief Notify that a link has been removed.
        public: static event::EventT<void (std::string)> linkRemoved;

        /// \brief Request to insert a link.
        public: static event::EventT<void (sdf::ElementPtr)>
            requestLinkInsertion;

        /// \brief Request to remove a nested model.
        public: static event::EventT<void (std::string)>
            requestNestedModelRemoval;

        /// \brief Request to insert a nested model.
        public: static event::EventT<void (sdf::ElementPtr)>
            requestNestedModelInsertion;

        /// \brief Request to remove a link.
        public: static event::EventT<void (std::string)> requestLinkRemoval;

        /// \brief Request to remove a joint.
        public: static event::EventT<void (std::string)> requestJointRemoval;

        /// \brief Request to insert a joint.
        public: static event::EventT<void (sdf::ElementPtr, std::string)>
            requestJointInsertion;

        /// \brief Notify that a joint has been inserted. The parameters are:
        /// joint's unique id, joint name, joint type, parent link's name, and
        /// child link's name. All names are scoped.
        public: static event::EventT<void (std::string, std::string,
            std::string, std::string, std::string)> jointInserted;

        /// \brief Notify that a joint has been removed.
        public: static event::EventT<void (std::string)> jointRemoved;

        /// \brief Notify that a joint has been changed. The parameters are:
        /// joint's unique id, joint name, joint type, parent link's name, and
        /// child link's name. All names are scoped.
        public: static event::EventT<void (std::string, std::string,
            std::string, std::string, std::string)> jointChanged;

        /// \brief Request to open the link inspector.
        public: static event::EventT<void (std::string)> openLinkInspector;

        /// \brief Request to open the joint inspector.
        public: static event::EventT<void (std::string)> openJointInspector;

        /// \brief Request to open the model plugin inspector.
        public: static event::EventT<void (std::string)>
            openModelPluginInspector;

        /// \brief Notify that the joint name has been changed. The first
        /// string is the joint's unique id and the second string is the
        /// new joint name.
        public: static event::EventT<void (std::string, std::string)>
            jointNameChanged;

        /// \brief Request to show the link context menu.
        public: static event::EventT<void (std::string)> showLinkContextMenu;

        /// \brief Request to show the joint context menu.
        public: static event::EventT<void (std::string)> showJointContextMenu;

        /// \brief Request to show the model plugin context menu.
        public: static event::EventT<void (std::string)>
            showModelPluginContextMenu;

        /// \brief Request to select or deselect an entity.
        public: static event::EventT<void (std::string, bool)>
            setSelectedEntity;

        /// \brief Request to select or deselect a joint.
        public: static event::EventT<void (std::string, bool)> setSelectedJoint;

        /// \brief Request to select or deselect a model plugin.
        public: static event::EventT<void (std::string, bool)>
            setSelectedModelPlugin;

        /// \brief Notify that a model plugin has been inserted.
        public: static event::EventT<void (std::string)> modelPluginInserted;

        /// \brief Notify that a model plugin has been removed.
        public: static event::EventT<void (std::string)> modelPluginRemoved;

        /// \brief Request to remove a model plugin.
        /// The parameters are: name, flag to indicate whether a new user
        /// command should be created.
        public: static event::EventT<void (std::string, bool)>
            requestModelPluginRemoval;

        /// \brief Request to insert a model plugin.
        /// The parameters are: name, filename, inner XML, flag to indicate
        /// whether a new user command should be created.
        public: static event::EventT<void (std::string, std::string,
            std::string, bool)> requestModelPluginInsertion;

        /// \brief Request to scale a link.
        public: static event::EventT<void (std::string,
            std::map<std::string, ignition::math::Vector3d>)> requestLinkScale;

        /// \brief Request to move a link.
        public: static event::EventT<void (std::string, ignition::math::Pose3d)>
            requestLinkMove;

        /// \brief Request to move a nestedModel.
        public: static event::EventT<void (std::string, ignition::math::Pose3d)>
            requestNestedModelMove;
      };
    }
  }
}
#endif
