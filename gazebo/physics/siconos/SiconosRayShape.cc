/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosRayShape::SiconosRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("Siconos Ray Shape");

  this->physicsEngine =
    boost::static_pointer_cast<SiconosPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
SiconosRayShape::SiconosRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  this->SetName("Siconos Ray Shape");
  this->physicsEngine = boost::static_pointer_cast<SiconosPhysics>(
      this->collisionParent->GetWorld()->Physics());
}

//////////////////////////////////////////////////
SiconosRayShape::~SiconosRayShape()
{
}

//////////////////////////////////////////////////
void SiconosRayShape::Update()
{
  if (this->collisionParent)
  {
    SiconosCollisionPtr collision =
        boost::static_pointer_cast<SiconosCollision>(this->collisionParent);

    LinkPtr link = this->collisionParent->GetLink();
    GZ_ASSERT(link != nullptr, "Siconos link is null");

    this->globalStartPos = link->WorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos = link->WorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  // btVector3 start(this->globalStartPos.X(), this->globalStartPos.Y(),
  //     this->globalStartPos.Z());
  // btVector3 end(this->globalEndPos.X(), this->globalEndPos.Y(),
  //     this->globalEndPos.Z());

  // btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
  // rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
  // rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;

  boost::recursive_mutex::scoped_lock lock(
      *this->physicsEngine->GetPhysicsUpdateMutex());

  // this->physicsEngine->GetDynamicsWorld()->rayTest(
  //     start, end, rayCallback);

  // if (rayCallback.hasHit())
  // {
  //   math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
  //                        rayCallback.m_hitPointWorld.getY(),
  //                        rayCallback.m_hitPointWorld.getZ());
  //   this->SetLength(this->globalStartPos.Distance(result));
  // }
}

//////////////////////////////////////////////////
void SiconosRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  if (this->collisionParent)
  {
    SiconosCollisionPtr collision =
        boost::static_pointer_cast<SiconosCollision>(this->collisionParent);

    LinkPtr link = this->collisionParent->GetLink();
    GZ_ASSERT(link != NULL, "Siconos link is NULL");

    this->globalStartPos = link->WorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos = link->WorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  if (this->physicsEngine)
  {
    // btVector3 start(this->globalStartPos.x, this->globalStartPos.y,
    //     this->globalStartPos.z);
    // btVector3 end(this->globalEndPos.x, this->globalEndPos.y,
    //     this->globalEndPos.z);

    // btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    // rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
    // rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;
    // this->physicsEngine->GetDynamicsWorld()->rayTest(
    //     start, end, rayCallback);
    // if (rayCallback.hasHit())
    // {
    //   math::Vector3 result(rayCallback.m_hitPointWorld.getX(),
    //                        rayCallback.m_hitPointWorld.getY(),
    //                        rayCallback.m_hitPointWorld.getZ());
    //   _dist = this->globalStartPos.Distance(result);

    //   SiconosLink *link = static_cast<SiconosLink *>(
    //       rayCallback.m_collisionObject->getUserPointer());
    //   GZ_ASSERT(link != NULL, "Siconos link is NULL");
    //   _entity = link->GetScopedName();
    // }
  }
}

//////////////////////////////////////////////////
void SiconosRayShape::SetPoints(const ignition::math::Vector3d &_posStart,
                                const ignition::math::Vector3d &_posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);
}
