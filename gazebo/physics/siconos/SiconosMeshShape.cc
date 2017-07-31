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

#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/siconos/siconos_inc.h"
#include "gazebo/physics/siconos/SiconosTypes.hh"
#include "gazebo/physics/siconos/SiconosCollision.hh"
#include "gazebo/physics/siconos/SiconosPhysics.hh"
#include "gazebo/physics/siconos/SiconosMeshShape.hh"

#include <siconos/SiconosShape.hpp>
#include <siconos/SiconosContactor.hpp>

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SiconosMeshShape::SiconosMeshShape(CollisionPtr _parent)
  : MeshShape(_parent)
{
}

//////////////////////////////////////////////////
SiconosMeshShape::~SiconosMeshShape()
{
}

//////////////////////////////////////////////////
void SiconosMeshShape::Load(sdf::ElementPtr _sdf)
{
  MeshShape::Load(_sdf);
}

//////////////////////////////////////////////////
void SiconosMeshShape::Init()
{
  MeshShape::Init();

  SiconosCollisionPtr bParent =
    boost::static_pointer_cast<SiconosCollision>(this->collisionParent);

  float *vertices = nullptr;
  int *indices = nullptr;

  unsigned int numVertices = 0;
  unsigned int numIndices = 0;

  ignition::math::Vector3d scale =
    this->sdf->Get<ignition::math::Vector3d>("scale");

  // Get all the vertex and index data
  if (this->submesh) {
    numVertices = this->submesh->GetVertexCount();
    numIndices = this->submesh->GetIndexCount();
    this->submesh->FillArrays(&vertices, &indices);
  }
  else {
    numVertices = this->mesh->GetVertexCount();
    numIndices = this->mesh->GetIndexCount();
    this->mesh->FillArrays(&vertices, &indices);
  }

  // Copy the indices to vector
  std11::shared_ptr< std::vector<unsigned int> > siconos_indexes(
    std11::make_shared< std::vector<unsigned int> >());
  siconos_indexes->resize(numIndices);
  std::copy(indices, indices + numIndices, siconos_indexes->begin());

  // Copy the vertex data (float -> double, apply scaling)
  SP::SiconosMatrix siconos_vertices(
    std11::make_shared< SimpleMatrix >(numVertices, 3));
  for (unsigned int i=0; i<numVertices; i++)
    for (unsigned int j=0; j<3; j++)
      (*siconos_vertices)(i,j) = vertices[i*3+j] * scale[j];

  delete [] vertices;
  delete [] indices;

  this->siconosMesh =
    std11::make_shared<SiconosMesh>(siconos_indexes, siconos_vertices);

  // TODO: scale
  // Put the shape in the SiconosCollision
  bParent->SetCollisionShape(this->siconosMesh);
}
