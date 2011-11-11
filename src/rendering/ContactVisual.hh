/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Camera Visualization Class
 * Author: Nate Koenig
 */

#ifndef CONTACTVISUAL_HH
#define CONTACTVISUAL_HH

#include "rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    class ContactVisual : public Visual
    {
      public: ContactVisual(const std::string &_name, Scene *_scene,
                            const std::string &_topicName);

      public: virtual ~ContactVisual();

      private: void Update();
      private: void OnContact(
                   const boost::shared_ptr<msgs::Contacts const> &_msg);

      private: void SetupInstancedMaterialToEntity(Ogre::Entity *ent);
      private: Ogre::String BuildInstancedMaterial(
                   const Ogre::String &originalMaterialName);

      private: Scene *scene;
      private: transport::NodePtr node;
      private: transport::SubscriberPtr contactsSub;
      private: Ogre::InstancedGeometry *instancedGeom;
      private: boost::shared_ptr<msgs::Contacts const> contactsMsg;
      private: std::vector<event::ConnectionPtr> connections;
      private: Ogre::Entity *obj;
    };
  }
}
#endif
