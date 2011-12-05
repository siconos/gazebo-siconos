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
#ifndef SCENE_HH
#define SCENE_HH

#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

#include "sdf/sdf.h"
#include "msgs/msgs.h"

//#include "rendering/ogre.h"
#include "rendering/RenderTypes.hh"

#include "transport/TransportTypes.hh"
#include "common/Events.hh"
#include "common/Color.hh"
#include "math/Vector2i.hh"


namespace Ogre
{
  class SceneManager;
  class RaySceneQuery;
  class Node;
  class Entity;
  class Mesh;
  class Vector3;
  class Quaternion;
}

namespace boost
{
  class mutex;
}

namespace gazebo
{

	namespace rendering
  {
    class Light;
    class Visual;
    class Grid;
    class Camera;
    class UserCamera;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief Representation of an entire scene graph  
    class Scene
    {
      public: enum SceneType {BSP, GENERIC};

      private: Scene() {}
  
      /// \brief Constructor
      public: Scene(const std::string &_name, 
                    bool _enableVisualizations=false);
  
      /// \brief Destructor
      public: virtual ~Scene();
  
      /// \brief Load the scene from a set of parameters
      public: void Load(sdf::ElementPtr &_scene);

      /// \brief Load the scene with default parameters
      public: void Load();
  
      /// \brief Init
      public: void Init();

      /// \brief Process all received messages 
      public: void PreRender();

      /// \brief Get the OGRE scene manager
      public: Ogre::SceneManager *GetManager() const;
  
      /// \brief Get the name of the scene
      public: std::string GetName() const;
  
      /// \brief Set the ambient color
      public: void SetAmbientColor(const common::Color &color);
  
      /// \brief Get the ambient color
      public: common::Color GetAmbientColor() const;
  
      /// \brief Set the background color
      public: void SetBackgroundColor(const common::Color &color);

      /// \brief Get the background color
      public: common::Color GetBackgroundColor() const;
  
      /// \brief Create a grid
      public: void CreateGrid(uint32_t cell_count, float cell_length, 
                              float line_width, const common::Color &color );
  
      /// \brief Get the grid
      public: Grid *GetGrid(unsigned int index) const;
  
      /// \brief Create a camera
      public: CameraPtr CreateCamera(const std::string &name, 
                                     bool _autoRender=true);
 
      /// \brief Create depth camera 
      public: DepthCameraPtr CreateDepthCamera( const std::string &_name, bool _autoRender=true);

      /// \brief Get the number of cameras in this scene
      public: unsigned int GetCameraCount() const;
  
      /// \brief Get a camera
      public: CameraPtr GetCamera(unsigned int index) const;

      /// \brief Get a camera by name
      public: CameraPtr GetCamera( const std::string &_name ) const;
  
      /// \brief Create a user camera
      public: UserCameraPtr CreateUserCamera(const std::string &name);
  
      /// \brief Get the number of user cameras in this scene
      public: unsigned int GetUserCameraCount() const;
  
      /// \brief Get a user camera
      public: UserCameraPtr GetUserCamera(unsigned int index) const;

      /// \brief Get a visual by name 
      public: VisualPtr GetVisual( const std::string &_name ) const;

      public: VisualPtr SelectVisualAt(CameraPtr camera, math::Vector2i mousePos);
      /// \brief Select a visual by name
      public: void SelectVisual( const std::string &_name ) const;

      /// \brief Get an entity at a pixel location using a camera. Used for
      ///        mouse picking. 
      /// \param camera The ogre camera, used to do mouse picking
      /// \param mousePos The position of the mouse in screen coordinates
      /// \param _mod Used for object manipulation
      /// \return The selected entity, or NULL
      public: VisualPtr GetVisualAt(CameraPtr camera, 
                                    math::Vector2i mousePos, 
                                    std::string &mod);

      /// \brief Get a visual at a mouse position
      public: VisualPtr GetVisualAt(CameraPtr camera, math::Vector2i mousePos);
 
      /// \brief Helper function for GetVisualAt functions 
      private: Ogre::Entity *GetOgreEntityAt(CameraPtr _camera, 
                                             math::Vector2i _mousePos,
                                             bool _ignorSelectionObj);

      /// \brief Get the world pos of a the first contact at a pixel location
      public: math::Vector3 GetFirstContact(CameraPtr camera, 
                                            math::Vector2i mousePos);
  
      public: void PrintSceneGraph();
  
      /// \brief Hide a visual
      public: void SetVisible(const std::string &name, bool visible);
  
      /// \brief Draw a named line
      public: void DrawLine(const math::Vector3 &start, 
                            const math::Vector3 &end, 
                            const std::string &name);
  
      public: void SetFog( const std::string &_type, 
                           const common::Color &_color, 
                           double _density, double _start, double _end );

      // Get the scene ID
      public: unsigned int GetId() const;
  
      // Get the scene Id as a string
      public: std::string GetIdString() const;
 
      /// \brief Get a unique scene node node 
      public: std::string GetUniqueName(const std::string &_prefix);

      /// \brief Get the selection object
      public: SelectionObj *GetSelectionObj() const;

      /// \brief Print scene graph
      private: void PrintSceneGraphHelper(const std::string &prefix, 
                                          Ogre::Node *node);
 
      /// \brief Deprecated: use RTShader::ApplyShadows
      public: void InitShadows();

      public: void SetSky(const std::string &_material);

      /// \brief Set whether shadows are on or off
      /// \param _value True to enable shadows, False to disable
      public: void SetShadowsEnabled(bool _value);

      /// \brief Get whether shadows are on or off
      public: bool GetShadowsEnabled() const;

      /// \brief Add a visual to the scene
      public: void AddVisual( VisualPtr &_vis );

      /// \brief Remove a visual from the scene
      public: void RemoveVisual( VisualPtr _vis );

      /// \brief Set the grid on or off
      /// \param _enabled Set to true to turn on the grid
      public: void SetGrid( bool _enabled );

      public: VisualPtr GetWorldVisual() const;


      // \brief Get the mesh information for the given mesh.
      // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
      private: void GetMeshInformation(const Ogre::Mesh *mesh,
                                       size_t &vertex_count,
                                       Ogre::Vector3* &vertices,
                                       size_t &index_count,
                                       unsigned long* &indices,
                                       const Ogre::Vector3 &position,
                                       const Ogre::Quaternion &orient,
                                       const Ogre::Vector3 &scale);
  
      private: void OnRequest(
                   ConstRequestPtr &_msg);
      private: void OnResponse(
                   ConstResponsePtr &_msg);
      private: void OnJointMsg(ConstJointPtr &_msg);

      private: void ProcessSensorMsg(
                   ConstSensorPtr &_msg);
      private: void ProcessJointMsg(
                   ConstJointPtr &_msg);

      private: void ProcessSceneMsg(
                   ConstScenePtr &_msg);

      private: void OnSceneMsg(
                   ConstScenePtr &_msg);
      private: void OnVisualMsg(
                   ConstVisualPtr &msg);
      private: void ProcessVisualMsg(
                   ConstVisualPtr &msg);

      private: void OnLightMsg(ConstLightPtr &msg);
      private: void ProcessLightMsg(
                   ConstLightPtr &msg);

      private: void OnSelectionMsg(
                   ConstSelectionPtr &msg);
               
      private: void OnPoseMsg(ConstPosePtr &msg);

      public: void Clear();

      //private: void ClearImpl();
      private: std::string name;

      private: sdf::ElementPtr sdf;
  
      private: std::vector<CameraPtr> cameras;
      private: std::vector<UserCameraPtr> userCameras;
  
      private: Ogre::SceneManager *manager;
      private: Ogre::RaySceneQuery *raySceneQuery;
  
      private: std::vector<Grid *> grids;
  
      private: static unsigned int idCounter;
      private: unsigned int id;
      private: std::string idString;
  
      typedef std::list<boost::shared_ptr<msgs::Visual const> > VisualMsgs_L;
      private: VisualMsgs_L visualMsgs;

      typedef std::list<boost::shared_ptr<msgs::Light const> > LightMsgs_L;
      private: LightMsgs_L lightMsgs;

      typedef std::list<boost::shared_ptr<msgs::Pose const> > PoseMsgs_L;
      private: PoseMsgs_L poseMsgs;

      typedef std::list<boost::shared_ptr<msgs::Scene const> > SceneMsgs_L;
      private: SceneMsgs_L sceneMsgs;

      typedef std::list<boost::shared_ptr<msgs::Joint const> > JointMsgs_L;
      private: JointMsgs_L jointMsgs;

      typedef std::list<boost::shared_ptr<msgs::Sensor const> > SensorMsgs_L;
      private: SensorMsgs_L sensorMsgs;

 
      typedef std::map<std::string, VisualPtr> Visual_M;
      private: Visual_M visuals;

      typedef std::map<std::string, Light*> Light_M;
      private: Light_M lights;

      private: boost::shared_ptr<msgs::Selection const> selectionMsg;

      private: boost::mutex *receiveMutex;

      private: transport::NodePtr node;  
      private: transport::SubscriberPtr sceneSub;
      private: transport::SubscriberPtr visSub;
      private: transport::SubscriberPtr lightSub;
      private: transport::SubscriberPtr poseSub;
      private: transport::SubscriberPtr selectionSub;
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr requestSub;
      private: transport::PublisherPtr requestPub;

      private: std::vector<event::ConnectionPtr> connections;

      private: SelectionObj *selectionObj;

      private: VisualPtr worldVisual;
      private: msgs::Request *requestMsg;

      private: bool enableVisualizations;
      private: bool clearAll;
    };
    /// \}
  }
}
#endif 
