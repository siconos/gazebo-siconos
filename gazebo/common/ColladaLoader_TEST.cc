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
#include <gtest/gtest.h>

#include "test_config.h"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "test/util.hh"

using namespace gazebo;

class ColladaLoader : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(ColladaLoader, LoadBox)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.dae");

  EXPECT_STREQ("unknown", mesh->GetName().c_str());
  EXPECT_EQ(ignition::math::Vector3d(1, 1, 1), mesh->Max());
  EXPECT_EQ(ignition::math::Vector3d(-1, -1, -1), mesh->Min());
  // 36 vertices, 24 unique, 12 shared.
  EXPECT_EQ(24u, mesh->GetVertexCount());
  EXPECT_EQ(24u, mesh->GetNormalCount());
  EXPECT_EQ(36u, mesh->GetIndexCount());
  EXPECT_EQ(0u, mesh->GetTexCoordCount());
  EXPECT_EQ(1u, mesh->GetSubMeshCount());
  EXPECT_EQ(1u, mesh->GetMaterialCount());

  // Make sure we can read a submesh name
  EXPECT_STREQ("Cube", mesh->GetSubMesh(0)->GetName().c_str());
}

/////////////////////////////////////////////////
TEST_F(ColladaLoader, ShareVertices)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.dae");

  // check number of shared vertices
  std::set<unsigned int> uniqueIndices;
  int shared = 0;
  for (unsigned int i = 0; i < mesh->GetSubMeshCount(); ++i)
  {
    const common::SubMesh *subMesh = mesh->GetSubMesh(i);
    for (unsigned int j = 0; j < subMesh->GetIndexCount(); ++j)
    {
      if (uniqueIndices.find(subMesh->GetIndex(j)) == uniqueIndices.end())
        uniqueIndices.insert(subMesh->GetIndex(j));
      else
        shared++;
    }
  }
  EXPECT_EQ(shared, 12);
  EXPECT_EQ(uniqueIndices.size(), 24u);

  // check all vertices are unique
  for (unsigned int i = 0; i < mesh->GetSubMeshCount(); ++i)
  {
    const common::SubMesh *subMesh = mesh->GetSubMesh(i);
    for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
    {
      ignition::math::Vector3d v = subMesh->Vertex(j);
      ignition::math::Vector3d n = subMesh->Normal(j);

      // Verify there is no other vertex with the same position AND normal
      for (unsigned int k = j+1; k < subMesh->GetVertexCount(); ++k)
      {
        if (v == subMesh->Vertex(k))
        {
          EXPECT_TRUE(n != subMesh->Normal(k));
        }
      }
    }
  }
}

/////////////////////////////////////////////////
TEST_F(ColladaLoader, LoadZeroCount)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/zero_count.dae");
  ASSERT_TRUE(mesh);

  std::string log = GetLogContent();

  // Expect no errors about missing values
  EXPECT_EQ(log.find("Loading what we can..."), std::string::npos);
  EXPECT_EQ(log.find("Vertex source missing float_array"), std::string::npos);
  EXPECT_EQ(log.find("Normal source missing float_array"), std::string::npos);

  // Expect the logs to contain information
  EXPECT_NE(log.find("Triangle input has a count of zero"), std::string::npos);
  EXPECT_NE(log.find("Vertex source has a float_array with a count of zero"),
      std::string::npos);
  EXPECT_NE(log.find("Normal source has a float_array with a count of zero"),
      std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(ColladaLoader, Material)
{
  common::ColladaLoader loader;
  common::Mesh *mesh = loader.Load(
      std::string(PROJECT_SOURCE_PATH) + "/test/data/box.dae");
  ASSERT_TRUE(mesh);

  EXPECT_EQ(mesh->GetMaterialCount(), 1u);

  const common::Material *mat = mesh->GetMaterial(0u);
  ASSERT_TRUE(mat);

  // Make sure we read the specular value
  EXPECT_EQ(mat->GetAmbient(), common::Color(0.0, 0.0, 0.0, 1.0));
  EXPECT_EQ(mat->GetDiffuse(), common::Color(0.64, 0.64, 0.64, 1.0));
  EXPECT_EQ(mat->GetSpecular(), common::Color(0.5, 0.5, 0.5, 1.0));
  EXPECT_EQ(mat->GetEmissive(), common::Color(0.0, 0.0, 0.0, 1.0));
  EXPECT_DOUBLE_EQ(mat->GetShininess(), 50.0);
  // transparent: opaque="A_ONE", color=[1 1 1 1]
  // transparency: 1.0
  // resulting transparency value = (1 - color.a * transparency)
  EXPECT_DOUBLE_EQ(mat->GetTransparency(), 0.0);
  double srcFactor = -1;
  double dstFactor = -1;
  mat->GetBlendFactors(srcFactor, dstFactor);
  EXPECT_DOUBLE_EQ(srcFactor, 1.0);
  EXPECT_DOUBLE_EQ(dstFactor, 0.0);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
