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
/* Desc: A ray
 * Author: Nate Koenig
 * Date: 24 May 2009
 */

#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletCollision.hh"
#include "gazebo/physics/bullet/BulletRayShape.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletRayShape::BulletRayShape(PhysicsEnginePtr _physicsEngine)
  : RayShape(_physicsEngine)
{
  this->SetName("Bullet Ray Shape");

  this->physicsEngine =
    boost::static_pointer_cast<BulletPhysics>(_physicsEngine);
}

//////////////////////////////////////////////////
BulletRayShape::BulletRayShape(CollisionPtr _parent)
    : RayShape(_parent)
{
  this->SetName("Bullet Ray Shape");
  this->physicsEngine = boost::static_pointer_cast<BulletPhysics>(
      this->collisionParent->GetWorld()->Physics());
}

//////////////////////////////////////////////////
BulletRayShape::~BulletRayShape()
{
}

//////////////////////////////////////////////////
void BulletRayShape::Update()
{
  if (this->collisionParent)
  {
    BulletCollisionPtr collision =
        boost::static_pointer_cast<BulletCollision>(this->collisionParent);

    LinkPtr link = this->collisionParent->GetLink();
    GZ_ASSERT(link != nullptr, "Bullet link is null");

    this->globalStartPos = link->WorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos = link->WorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  btVector3 start(this->globalStartPos.X(), this->globalStartPos.Y(),
      this->globalStartPos.Z());
  btVector3 end(this->globalEndPos.X(), this->globalEndPos.Y(),
      this->globalEndPos.Z());

  btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
  rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
  rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;

  boost::recursive_mutex::scoped_lock lock(
      *this->physicsEngine->GetPhysicsUpdateMutex());

  this->physicsEngine->GetDynamicsWorld()->rayTest(
      start, end, rayCallback);

  if (rayCallback.hasHit())
  {
    ignition::math::Vector3d result(rayCallback.m_hitPointWorld.getX(),
        rayCallback.m_hitPointWorld.getY(),
        rayCallback.m_hitPointWorld.getZ());
    this->SetLength(this->globalStartPos.Distance(result));
  }
}

//////////////////////////////////////////////////
void BulletRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  _dist = 0;
  _entity = "";

  if (this->collisionParent)
  {
    BulletCollisionPtr collision =
        boost::static_pointer_cast<BulletCollision>(this->collisionParent);

    LinkPtr link = this->collisionParent->GetLink();
    GZ_ASSERT(link != nullptr, "Bullet link is null");

    this->globalStartPos = link->WorldPose().CoordPositionAdd(
          this->relativeStartPos);

    this->globalEndPos = link->WorldPose().CoordPositionAdd(
          this->relativeEndPos);
  }

  if (this->physicsEngine)
  {
    btVector3 start(this->globalStartPos.X(), this->globalStartPos.Y(),
        this->globalStartPos.Z());
    btVector3 end(this->globalEndPos.X(), this->globalEndPos.Y(),
        this->globalEndPos.Z());

    btCollisionWorld::ClosestRayResultCallback rayCallback(start, end);
    rayCallback.m_collisionFilterGroup = GZ_SENSOR_COLLIDE;
    rayCallback.m_collisionFilterMask = ~GZ_SENSOR_COLLIDE;
    this->physicsEngine->GetDynamicsWorld()->rayTest(
        start, end, rayCallback);
    if (rayCallback.hasHit())
    {
      ignition::math::Vector3d result(rayCallback.m_hitPointWorld.getX(),
          rayCallback.m_hitPointWorld.getY(),
          rayCallback.m_hitPointWorld.getZ());
      _dist = this->globalStartPos.Distance(result);

      BulletLink *link = static_cast<BulletLink *>(
          rayCallback.m_collisionObject->getUserPointer());
      GZ_ASSERT(link != nullptr, "Bullet link is null");
      _entity = link->GetScopedName();
    }
  }
}

//////////////////////////////////////////////////
void BulletRayShape::SetPoints(const ignition::math::Vector3d &_posStart,
                               const ignition::math::Vector3d &_posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);
}
