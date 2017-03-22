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
#include <boost/assign/list_of.hpp>
#include "gazebo/test/ServerFixture.hh"

#include "test_config.h"


using namespace gazebo;
class Heightmap_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
/// \brief Test Split a terrain in a number of subterrains
TEST_F(Heightmap_TEST, splitTerrain)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != nullptr);

  scene = gazebo::rendering::create_scene("default", false);

  // Make sure that the scene is created
  ASSERT_TRUE(scene != nullptr);

  gazebo::rendering::Heightmap *heightmap =
      new gazebo::rendering::Heightmap(scene);

  // Check that the heightmap is created
  EXPECT_TRUE(heightmap != nullptr);

  std::vector<float> heights;
  std::vector<std::vector<float> > heightsSplit;
  std::vector<std::vector<float> > slices;

  // Initialize a 9 x 9 terrain with known values
  int N = 9;
  heights.resize(N * N);
  for (int i = 0; i < N * N; ++i)
  {
    heights[i] = i + 1;
  }

  heightmap->SplitHeights(heights, heightmap->TerrainSubdivisionCount(),
      heightsSplit);

  ASSERT_TRUE(heightsSplit.size() == heightmap->TerrainSubdivisionCount());

  // Precomputed subterrains for a known 9 x 9 terrain starting from 1 and with
  // consecutive values
  slices.resize(16);
  slices[0] =  {1, 2, 2, 10, 11, 11, 10, 11, 11};
  slices[1] =  {3, 4, 4, 12, 13, 13, 12, 13, 13};
  slices[2] =  {5, 6, 6, 14, 15, 15, 14, 15, 15};
  slices[3] =  {7, 8, 8, 16, 17, 17, 16, 17, 17};
  slices[4] =  {19, 20, 20, 28, 29, 29, 28, 29, 29};
  slices[5] =  {21, 22, 22, 30, 31, 31, 30, 31, 31};
  slices[6] =  {23, 24, 24, 32, 33, 33, 32, 33, 33};
  slices[7] =  {25, 26, 26, 34, 35, 35, 34, 35, 35};
  slices[8] =  {37, 38, 38, 46, 47, 47, 46, 47, 47};
  slices[9] =  {39, 40, 40, 48, 49, 49, 48, 49, 49};
  slices[10] = {41, 42, 42, 50, 51, 51, 50, 51, 51};
  slices[11] = {43, 44, 44, 52, 53, 53, 52, 53, 53};
  slices[12] = {55, 56, 56, 64, 65, 65, 64, 65, 65};
  slices[13] = {57, 58, 58, 66, 67, 67, 66, 67, 67};
  slices[14] = {59, 60, 60, 68, 69, 69, 68, 69, 69};
  slices[15] = {61, 62, 62, 70, 71, 71, 70, 71, 71};

  // Make sure that the subterrain heights matches the precomputed slices
  for (unsigned int i = 0; i < heightmap->TerrainSubdivisionCount(); ++i)
  {
    EXPECT_TRUE(std::equal(heightsSplit[i].begin(), heightsSplit[i].end(),
          slices[i].begin()));
  }
}

/////////////////////////////////////////////////
/// \brief Test terrain's level of detail API
TEST_F(Heightmap_TEST, LOD)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != nullptr);

  gazebo::rendering::Heightmap *heightmap =
      new gazebo::rendering::Heightmap(scene);

  // test basic API
  EXPECT_EQ(heightmap->LOD(), 0u);
  heightmap->SetLOD(3u);
  EXPECT_EQ(heightmap->LOD(), 3u);

  // try 0 LOD
  heightmap->SetLOD(0u);
  EXPECT_EQ(heightmap->LOD(), 0u);
}

#ifdef HAVE_GDAL
/////////////////////////////////////////////////
/// \brief Test Loading a terrain from a DEM file
TEST_F(Heightmap_TEST, LoadDEM)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_TRUE(scene != nullptr);

  gazebo::rendering::Heightmap *heightmap =
      new gazebo::rendering::Heightmap(scene);

  // Check that the heightmap is created
  EXPECT_TRUE(heightmap != nullptr);

  // create a heightmapgeom msg for the heightmap
  msgs::Visual msg;
  msg.set_name("heightmap_visual");
  msg.set_parent_name("heightmap_visual_parent");
  auto geomMsg = msg.mutable_geometry();
  // set size to zero to let heightmap read actual size from dem file
  msgs::Set(geomMsg->mutable_heightmap()->mutable_size(),
      ignition::math::Vector3d::Zero);

  boost::filesystem::path path = TEST_PATH;
  path /= "data/dem_squared.tif";
  geomMsg->mutable_heightmap()->set_filename(path.string());

  // load the heightmap
  auto visMsg = new ConstVisualPtr(&msg);
  heightmap->LoadFromMsg(*visMsg);

  // verify heightmap image size
  unsigned int subsampling = 2;
  unsigned int tifSize = 129;
  unsigned int vertSize = (tifSize * subsampling) - 1;
  common::Image img = heightmap->Image();
  EXPECT_EQ(img.GetWidth(), vertSize);
  EXPECT_EQ(img.GetHeight(), vertSize);

  // verify heights between value obtained using ray casting (heightmap->Height)
  // and the actual elevation data (dem.GetElevation)
  common::Dem dem;
  EXPECT_EQ(dem.Load(path.string()), 0);
  EXPECT_FLOAT_EQ(heightmap->Height(0, 0),
      dem.GetElevation(dem.GetWidth()/2, dem.GetHeight()/2));

  delete heightmap;
}
#endif

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
