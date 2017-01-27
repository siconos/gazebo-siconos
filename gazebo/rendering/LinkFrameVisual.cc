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

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/LinkFrameVisualPrivate.hh"
#include "gazebo/rendering/LinkFrameVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
LinkFrameVisual::LinkFrameVisual(const std::string &_name, VisualPtr _parent)
  : AxisVisual(*new LinkFrameVisualPrivate, _name, _parent)
{
  LinkFrameVisualPrivate *dPtr =
      reinterpret_cast<LinkFrameVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_PHYSICS;
  dPtr->highlightedTransp = 0.01;
  dPtr->nonHighlightedTransp = 0.8;
}

/////////////////////////////////////////////////
void LinkFrameVisual::Load()
{
  LinkFrameVisualPrivate *dPtr =
      reinterpret_cast<LinkFrameVisualPrivate *>(this->dataPtr);

  AxisVisual::Load();

  this->RecalculateScale();

  // Don't scale when link is scaled
  this->GetSceneNode()->setInheritScale(false);

  this->ShowAxisHead(0, false);
  this->ShowAxisHead(1, false);
  this->ShowAxisHead(2, false);
  this->SetInheritTransparency(false);
  this->SetTransparency(dPtr->nonHighlightedTransp);
  this->SetCastShadows(false);
}

/////////////////////////////////////////////////
void LinkFrameVisual::RecalculateScale()
{
  LinkFrameVisualPrivate *dPtr =
      reinterpret_cast<LinkFrameVisualPrivate *>(this->dataPtr);

  double linkSize = std::max(0.1,
      dPtr->parent->BoundingBox().Size().Length());
  linkSize = std::min(linkSize, 1.0);
  dPtr->scaleToLink = ignition::math::Vector3d(linkSize * 0.7,
                                               linkSize * 0.7,
                                               linkSize * 0.7);

  // Scale according to the link it is attached to
  if (dPtr->scaleToLink != ignition::math::Vector3d::Zero)
    this->SetScale(dPtr->scaleToLink);
}

//////////////////////////////////////////////////
void LinkFrameVisual::SetHighlighted(bool _highlighted)
{
  LinkFrameVisualPrivate *dPtr =
      reinterpret_cast<LinkFrameVisualPrivate *>(this->dataPtr);

  if (_highlighted)
  {
    this->SetTransparency(dPtr->highlightedTransp);
  }
  else
  {
    this->SetTransparency(dPtr->nonHighlightedTransp);
  }
}

//////////////////////////////////////////////////
bool LinkFrameVisual::GetHighlighted()
{
  LinkFrameVisualPrivate *dPtr =
      reinterpret_cast<LinkFrameVisualPrivate *>(this->dataPtr);

  return ignition::math::equal(this->GetTransparency(),
                               dPtr->highlightedTransp);
}
