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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <sys/stat.h>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RTShaderSystemPrivate.hh"
#include "gazebo/rendering/RTShaderSystem.hh"

#define MINOR_VERSION 7
using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
RTShaderSystem::RTShaderSystem()
  : dataPtr(new RTShaderSystemPrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->shadowsApplied = false;
  this->dataPtr->pssmSetup.setNull();
  this->dataPtr->updateShaders = false;
}

//////////////////////////////////////////////////
RTShaderSystem::~RTShaderSystem()
{
  this->Fini();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void RTShaderSystem::Init()
{
#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 &&\
    OGRE_VERSION_MINOR >= MINOR_VERSION

  // Only initialize if using FORWARD rendering
  if (RenderEngine::Instance()->GetRenderPathType() != RenderEngine::FORWARD)
    return;

  if (Ogre::RTShader::ShaderGenerator::initialize())
  {
    this->dataPtr->initialized = true;

    std::string coreLibsPath, cachePath;
    this->GetPaths(coreLibsPath, cachePath);

    // Get the shader generator pointer
    this->dataPtr->shaderGenerator =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();

    // Add the shader libs resource location
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        coreLibsPath, "FileSystem");

    // Set shader cache path.
    this->dataPtr->shaderGenerator->setShaderCachePath(cachePath);

    this->dataPtr->shaderGenerator->setTargetLanguage("glsl");
  }
  else
    gzerr << "RT Shader system failed to initialize\n";
#endif
}

//////////////////////////////////////////////////
void RTShaderSystem::Fini()
{
  if (!this->dataPtr->initialized)
    return;

  // Restore default scheme.
  Ogre::MaterialManager::getSingleton().setActiveScheme(
      Ogre::MaterialManager::DEFAULT_SCHEME_NAME);

  // Finalize RTShader system.
  if (this->dataPtr->shaderGenerator != NULL)
  {
#if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
    Ogre::RTShader::ShaderGenerator::finalize();
#else
    Ogre::RTShader::ShaderGenerator::destroy();
#endif
    this->dataPtr->shaderGenerator = NULL;
  }

  this->dataPtr->pssmSetup.setNull();
  this->dataPtr->scenes.clear();
  this->dataPtr->initialized = false;
}

#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 &&\
    OGRE_VERSION_MINOR >= MINOR_VERSION
//////////////////////////////////////////////////
void RTShaderSystem::AddScene(ScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  // Set the scene manager
  this->dataPtr->shaderGenerator->addSceneManager(_scene->OgreSceneManager());
  this->dataPtr->shaderGenerator->createScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->dataPtr->scenes.push_back(_scene);
}
#else
void RTShaderSystem::AddScene(ScenePtr /*_scene*/)
{
}
#endif

//////////////////////////////////////////////////
void RTShaderSystem::RemoveScene(ScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  auto iter = this->dataPtr->scenes.begin();
  for (; iter != this->dataPtr->scenes.end(); ++iter)
    if ((*iter) == _scene)
      break;

  if (iter != this->dataPtr->scenes.end())
  {
    this->dataPtr->scenes.erase(iter);
    this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    this->dataPtr->shaderGenerator->removeSceneManager(
        _scene->OgreSceneManager());
    this->dataPtr->shaderGenerator->removeAllShaderBasedTechniques();
    this->dataPtr->shaderGenerator->flushShaderCache();
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::RemoveScene(const std::string &_scene)
{
  if (!this->dataPtr->initialized)
    return;

  for (auto iter : this->dataPtr->scenes)
  {
    if (iter->Name() == _scene)
    {
      this->RemoveScene(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::AttachViewport(Ogre::Viewport *_viewport, ScenePtr _scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  _viewport->setMaterialScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#endif
}

//////////////////////////////////////////////////
void RTShaderSystem::DetachViewport(Ogre::Viewport *_viewport, ScenePtr _scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  if (_viewport && _scene && _scene->Initialized())
    _viewport->setMaterialScheme(_scene->Name());
#endif
}

//////////////////////////////////////////////////
void RTShaderSystem::UpdateShaders()
{
  // shaders will be updated in the Update call on pre-render event.
  this->dataPtr->updateShaders = true;
}

//////////////////////////////////////////////////
void RTShaderSystem::UpdateShaders(VisualPtr _vis)
{
  if (_vis)
  {
    if (_vis->UseRTShader())
      this->GenerateShaders(_vis);
    for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
      this->UpdateShaders(_vis->GetChild(i));
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::GenerateShaders(const VisualPtr &_vis)
{
  if (!this->dataPtr->initialized || !_vis)
    return;

  for (unsigned int k = 0; _vis->GetSceneNode() &&
      k < _vis->GetSceneNode()->numAttachedObjects(); ++k)
  {
    Ogre::MovableObject *obj = _vis->GetSceneNode()->getAttachedObject(k);
    Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>(obj);
    if (!entity)
      continue;

    for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i)
    {
      Ogre::SubEntity* curSubEntity = entity->getSubEntity(i);
      const Ogre::String& curMaterialName = curSubEntity->getMaterialName();
      bool success = false;

      for (unsigned int s = 0; s < this->dataPtr->scenes.size(); s++)
      {
        try
        {
          success = this->dataPtr->shaderGenerator->createShaderBasedTechnique(
              curMaterialName,
              Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
              this->dataPtr->scenes[s]->Name() +
              Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
        }
        catch(Ogre::Exception &e)
        {
          gzerr << "Unable to create shader technique for material["
            << curMaterialName << "]\n";
          success = false;
        }

        // Setup custom shader sub render states according to current setup.
        if (success)
        {
          Ogre::MaterialPtr curMaterial =
            Ogre::MaterialManager::getSingleton().getByName(curMaterialName);

          // Grab the first pass render state.
          // NOTE:For more complicated samples iterate over the passes and build
          // each one of them as desired.
          Ogre::RTShader::RenderState* renderState =
            this->dataPtr->shaderGenerator->getRenderState(
                this->dataPtr->scenes[s]->Name() +
                Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
                curMaterialName, 0);

          // Remove all sub render states.
          renderState->reset();

          /// This doesn't seem to work properly.
          if (_vis->GetShaderType() == "normal_map_object_space")
          {
            Ogre::RTShader::SubRenderState* subRenderState =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::NormalMapLighting::Type);

            Ogre::RTShader::NormalMapLighting* normalMapSubRS =
              static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(
                Ogre::RTShader::NormalMapLighting::NMS_OBJECT);

            normalMapSubRS->setNormalMapTextureName(_vis->GetNormalMap());
            renderState->addTemplateSubRenderState(normalMapSubRS);
          }
          else if (_vis->GetShaderType() == "normal_map_tangent_space")
          {
            Ogre::RTShader::SubRenderState* subRenderState =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::NormalMapLighting::Type);

            Ogre::RTShader::NormalMapLighting* normalMapSubRS =
              static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(
                Ogre::RTShader::NormalMapLighting::NMS_TANGENT);

            normalMapSubRS->setNormalMapTextureName(_vis->GetNormalMap());

            renderState->addTemplateSubRenderState(normalMapSubRS);
          }
          else if (_vis->GetShaderType() == "vertex")
          {
            Ogre::RTShader::SubRenderState *perPerVertexLightModel =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::FFPLighting::Type);

            renderState->addTemplateSubRenderState(perPerVertexLightModel);
          }
          else
          {
            Ogre::RTShader::SubRenderState *perPixelLightModel =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::PerPixelLighting::Type);

            renderState->addTemplateSubRenderState(perPixelLightModel);
          }


          // Invalidate this material in order to re-generate its shaders.
          this->dataPtr->shaderGenerator->invalidateMaterial(
              this->dataPtr->scenes[s]->Name() +
              Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
              curMaterialName);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
bool RTShaderSystem::GetPaths(std::string &coreLibsPath, std::string &cachePath)
{
  if (!this->dataPtr->initialized)
    return false;

  Ogre::StringVector groupVector;

  // Setup the core libraries and shader cache path
  groupVector = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
  Ogre::StringVector::iterator itGroup = groupVector.begin();
  Ogre::StringVector::iterator itGroupEnd = groupVector.end();
  Ogre::String shaderCoreLibsPath;

  for (; itGroup != itGroupEnd; ++itGroup)
  {
    Ogre::ResourceGroupManager::LocationList resLocationsList;
    Ogre::ResourceGroupManager::LocationList::iterator it;
    Ogre::ResourceGroupManager::LocationList::iterator itEnd;
    bool coreLibsFound = false;

    resLocationsList =
      Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(
          *itGroup);
    it = resLocationsList.begin();
    itEnd = resLocationsList.end();
    // Try to find the location of the core shader lib functions and use it
    // as shader cache path as well - this will reduce the number of
    // generated files when running from different directories.

    for (; it != itEnd; ++it)
    {
      struct stat st;
      if (stat((*it)->archive->getName().c_str(), &st) == 0)
      {
        if ((*it)->archive->getName().find("rtshaderlib") != Ogre::String::npos)
        {
          coreLibsPath = (*it)->archive->getName() + "/";

          // setup patch name for rt shader cache in tmp
          char *tmpdir;
          char *user;
          std::ostringstream stream;
          std::ostringstream errStream;
          // Get the tmp dir
          tmpdir = getenv("TMP");
          if (!tmpdir)
          {
            common::SystemPaths *paths = common::SystemPaths::Instance();
            tmpdir = const_cast<char*>(paths->TmpPath().c_str());
          }
          // Get the user
          user = getenv("USER");
          if (!user)
            user = const_cast<char*>("nobody");
          stream << tmpdir << "/gazebo-" << user << "-rtshaderlibcache" << "/";
          cachePath = stream.str();
          // Create the directory
#ifdef _WIN32
          if (_mkdir(cachePath.c_str()) != 0)
#else
          if (mkdir(cachePath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR) != 0)
#endif
          {
            if (errno != EEXIST)
            {
              errStream << "failed to create [" << cachePath << "] : ["
                <<  strerror(errno) << "]";
              throw(errStream.str());
            }
          }

          coreLibsFound = true;
          break;
        }
      }
    }

    // Core libs path found in the current group.
    if (coreLibsFound)
      break;
  }

  // Core shader lib not found -> shader generating will fail.
  if (coreLibsPath.empty())
  {
    gzerr << "Unable to find shader lib. Shader generating will fail.";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void RTShaderSystem::RemoveShadows(ScenePtr _scene)
{
  if (!this->dataPtr->initialized || !this->dataPtr->shadowsApplied)
    return;

  _scene->OgreSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  _scene->OgreSceneManager()->setShadowCameraSetup(
      Ogre::ShadowCameraSetupPtr());

  Ogre::RTShader::RenderState* schemeRenderState =
    this->dataPtr->shaderGenerator->getRenderState(
        _scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  schemeRenderState->removeTemplateSubRenderState(
      this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->UpdateShaders();

  this->dataPtr->shadowsApplied = false;
}

/////////////////////////////////////////////////
void RTShaderSystem::ApplyShadows(ScenePtr _scene)
{
  if (!this->dataPtr->initialized || this->dataPtr->shadowsApplied)
    return;

  Ogre::SceneManager *sceneMgr = _scene->OgreSceneManager();

  // Grab the scheme render state.
  Ogre::RTShader::RenderState* schemRenderState =
    this->dataPtr->shaderGenerator->getRenderState(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);

  // 3 textures per directional light
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 3);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 0);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, 0);
  sceneMgr->setShadowTextureCount(3);
  sceneMgr->setShadowTextureConfig(0, 1024, 1024, Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(1, 512, 512, Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(2, 512, 512, Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureSelfShadow(false);
  sceneMgr->setShadowCasterRenderBackFaces(true);

  // TODO: We have two different shadow caster materials, both taken from
  // OGRE samples. They should be compared and tested.
  // Set up caster material - this is just a standard depth/shadow map caster
  // sceneMgr->setShadowTextureCasterMaterial("PSSM/shadow_caster");
  sceneMgr->setShadowTextureCasterMaterial("Gazebo/shadow_caster");

  // Disable fog on the caster pass.
  //  Ogre::MaterialPtr passCaterMaterial =
  //   Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
  // Ogre::Pass* pssmCasterPass =
  // passCaterMaterial->getTechnique(0)->getPass(0);
  // pssmCasterPass->setFog(true);

  // shadow camera setup
  if (this->dataPtr->pssmSetup.isNull())
  {
    this->dataPtr->pssmSetup =
        Ogre::ShadowCameraSetupPtr(new Ogre::PSSMShadowCameraSetup());
  }

  double shadowFarDistance = 500;
  double cameraNearClip = 0.01;
  sceneMgr->setShadowFarDistance(shadowFarDistance);

  Ogre::PSSMShadowCameraSetup *cameraSetup =
      dynamic_cast<Ogre::PSSMShadowCameraSetup*>(
      this->dataPtr->pssmSetup.get());

  cameraSetup->calculateSplitPoints(3, cameraNearClip, shadowFarDistance);
  cameraSetup->setSplitPadding(4);
  cameraSetup->setOptimalAdjustFactor(0, 2);
  cameraSetup->setOptimalAdjustFactor(1, 1);
  cameraSetup->setOptimalAdjustFactor(2, .5);

  sceneMgr->setShadowCameraSetup(this->dataPtr->pssmSetup);

  // These values do not seem to help at all. Leaving here until I have time
  // to properly fix shadow z-fighting.
  // cameraSetup->setOptimalAdjustFactor(0, 4);
  // cameraSetup->setOptimalAdjustFactor(1, 1);
  // cameraSetup->setOptimalAdjustFactor(2, 0.5);

  this->dataPtr->shadowRenderState =
      this->dataPtr->shaderGenerator->createSubRenderState(
      Ogre::RTShader::IntegratedPSSM3::Type);
  Ogre::RTShader::IntegratedPSSM3 *pssm3SubRenderState =
      static_cast<Ogre::RTShader::IntegratedPSSM3*>(
      this->dataPtr->shadowRenderState);

  const Ogre::PSSMShadowCameraSetup::SplitPointList &srcSplitPoints =
    cameraSetup->getSplitPoints();

  Ogre::RTShader::IntegratedPSSM3::SplitPointList dstSplitPoints;

  for (unsigned int i = 0; i < srcSplitPoints.size(); ++i)
  {
    dstSplitPoints.push_back(srcSplitPoints[i]);
  }

  pssm3SubRenderState->setSplitPoints(dstSplitPoints);
  schemRenderState->addTemplateSubRenderState(this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  this->UpdateShaders();

  this->dataPtr->shadowsApplied = true;
}

/////////////////////////////////////////////////
Ogre::PSSMShadowCameraSetup *RTShaderSystem::GetPSSMShadowCameraSetup() const
{
  return dynamic_cast<Ogre::PSSMShadowCameraSetup *>(
      this->dataPtr->pssmSetup.get());
}

/////////////////////////////////////////////////
void RTShaderSystem::Update()
{
  if (!this->dataPtr->initialized || !this->dataPtr->updateShaders)
    return;

  for (const auto &scene : this->dataPtr->scenes)
  {
    VisualPtr vis = scene->WorldVisual();
    if (vis)
    {
      this->UpdateShaders(vis);
    }
  }
  this->dataPtr->updateShaders = false;
}
