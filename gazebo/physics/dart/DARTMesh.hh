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

#ifndef GAZEBO_PHYSICS_DART_DARTMESH_HH_
#define GAZEBO_PHYSICS_DART_DARTMESH_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_dart Dart Physics
    /// \{

    /// Forward declare private data class
    class DARTMeshPrivate;

    /// \brief Triangle mesh collision helper class
    class GZ_PHYSICS_VISIBLE DARTMesh
    {
      /// \brief Constructor
      public: DARTMesh();

      /// \brief Destructor
      public: virtual ~DARTMesh();

      /// \brief Create a mesh collision shape using a submesh.
      /// \param[in] _subMesh Pointer to the submesh.
      /// \param[in] _collision Pointer to the collision object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::SubMesh *_subMesh,
                      DARTCollisionPtr _collision,
                      const ignition::math::Vector3d &_scale);

      /// \brief Create a mesh collision shape using a mesh.
      /// \param[in] _mesh Pointer to the mesh.
      /// \param[in] _collision Pointer to the collision object.
      /// \param[in] _scale Scaling factor.
      public: void Init(const common::Mesh *_mesh,
                      DARTCollisionPtr _collision,
                      const ignition::math::Vector3d &_scale);

      /// \brief Returns the DART mesh shape node
      public: dart::dynamics::ShapeNodePtr ShapeNode() const;

      /// \brief Helper function to create the collision shape.
      /// \param[in] _vertices Array of vertices.
      /// \param[in] _indices Array of indices.
      /// \param[in] _numVertices Number of vertices.
      /// \param[in] _numIndices Number of indices.
      /// \param[in] _collision Pointer to the collision object.
      private: void CreateMesh(float *_vertices, int *_indices,
                   unsigned int _numVertices, unsigned int _numIndices,
                   DARTCollisionPtr _collision,
                   const ignition::math::Vector3d &_scale);

      /// \internal
      /// \brief Pointer to private data
      private: DARTMeshPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
