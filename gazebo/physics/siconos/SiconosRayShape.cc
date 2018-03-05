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
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosWorld.hh"
#include "gazebo/physics/siconos/SiconosLink.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosRayShape.hh"

#include "siconos/SiconosCollisionQueryResult.hpp"
#include "siconos/SiconosCollisionManager.hpp"

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
  if (!this->physicsEngine)
    return;

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

  SiconosVector startPoint({this->globalStartPos.X(),
                            this->globalStartPos.Y(),
                            this->globalStartPos.Z()});
  SiconosVector endPoint({this->globalEndPos.X(),
                          this->globalEndPos.Y(),
                          this->globalEndPos.Z()});

  boost::recursive_mutex::scoped_lock lock(
      *this->physicsEngine->GetPhysicsUpdateMutex());

  auto result =
    this->physicsEngine->GetSiconosWorld()
    ->GetCollisionManager()->lineIntersectionQuery(
      startPoint, endPoint, true, false);

  if (result.size() > 0)
    this->SetLength(result[0]->distance);
}

//////////////////////////////////////////////////
void SiconosRayShape::GetIntersection(double &_dist, std::string &_entity)
{
  if (!this->physicsEngine)
    return;

  _dist = 0;
  _entity = "";

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

  SiconosVector startPoint({this->globalStartPos.X(),
                            this->globalStartPos.Y(),
                            this->globalStartPos.Z()});
  SiconosVector endPoint({this->globalEndPos.X(),
                          this->globalEndPos.Y(),
                          this->globalEndPos.Z()});

  boost::recursive_mutex::scoped_lock lock(
      *this->physicsEngine->GetPhysicsUpdateMutex());

  auto sworld = this->physicsEngine->GetSiconosWorld();
  auto result = sworld->GetCollisionManager()->lineIntersectionQuery(
    startPoint, endPoint, true, false);

  if (result.size() > 0)
  {
    _dist = result[0]->distance;

    if (result[0]->body)
    {
      SiconosLinkPtr link = SiconosLink::GetLinkForBody(result[0]->body).lock();
      GZ_ASSERT(link != nullptr, "Siconos link is null");
      _entity = link->GetScopedName();
    }
  }
}

//////////////////////////////////////////////////
void SiconosRayShape::SetPoints(const ignition::math::Vector3d &_posStart,
                                const ignition::math::Vector3d &_posEnd)
{
  RayShape::SetPoints(_posStart, _posEnd);
}
