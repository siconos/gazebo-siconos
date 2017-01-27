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

#include <gtest/gtest.h>
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/OriginVisual.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class OriginVisual_TEST : public RenderingFixture
{
};

TEST_F(OriginVisual_TEST, Load)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // get scene visual child count before we create any visuals
  EXPECT_TRUE(scene->WorldVisual() != nullptr);
  unsigned int count = scene->WorldVisual()->GetChildCount();

  // Create and load visual
  rendering::OriginVisualPtr origin;
  origin.reset(new rendering::OriginVisual("origin", scene->WorldVisual()));
  origin->Load();
  EXPECT_TRUE(origin != nullptr);

  // Check that it was added to the scene (by Load)
  EXPECT_EQ(scene->GetVisual("origin"), origin);

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(origin);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("origin") == nullptr);

  // Reset pointer
  origin.reset();
  EXPECT_TRUE(origin == nullptr);

  // verify scene's child count is the same as before the visual was created
  EXPECT_EQ(scene->WorldVisual()->GetChildCount(), count);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
