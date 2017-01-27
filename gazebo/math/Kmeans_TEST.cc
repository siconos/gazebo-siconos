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

#include <vector>
#include <gtest/gtest.h>
#include "gazebo/math/Kmeans.hh"

#ifndef _WIN32
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

using namespace gazebo;

class KmeansTest : public ::testing::Test { };

//////////////////////////////////////////////////
TEST_F(KmeansTest, Kmeans)
{
  // Create some observations.
  std::vector<math::Vector3> obs;
  obs.push_back(math::Vector3(1.0, 1.0, 0.0));
  obs.push_back(math::Vector3(1.1, 1.0, 0.0));
  obs.push_back(math::Vector3(1.2, 1.0, 0.0));
  obs.push_back(math::Vector3(1.3, 1.0, 0.0));
  obs.push_back(math::Vector3(1.4, 1.0, 0.0));
  obs.push_back(math::Vector3(5.0, 1.0, 0.0));
  obs.push_back(math::Vector3(5.1, 1.0, 0.0));
  obs.push_back(math::Vector3(5.2, 1.0, 0.0));
  obs.push_back(math::Vector3(5.3, 1.0, 0.0));
  obs.push_back(math::Vector3(5.4, 1.0, 0.0));

  // Initialize Kmeans with two partitions.
  math::Kmeans kmeans(obs);

  // ::GetObservations()
  std::vector<math::Vector3> obsCopy;
  obsCopy = kmeans.Observations();
  for (size_t i = 0; i < obsCopy.size(); ++i)
    EXPECT_EQ(obsCopy[i], obs[i]);

  // ::SetObservations()
  for (size_t i = 0; i < obsCopy.size(); ++i)
    obsCopy[i] += math::Vector3(0.1, 0.2, 0.0);

  EXPECT_TRUE(kmeans.Observations(obsCopy));

  obsCopy = kmeans.Observations();
  for (size_t i = 0; i < obsCopy.size(); ++i)
    EXPECT_EQ(obsCopy[i], obs[i] + math::Vector3(0.1, 0.2, 0.0));
  EXPECT_TRUE(kmeans.Observations(obs));

  // ::Cluster()
  std::vector<math::Vector3> centroids;
  std::vector<unsigned int> labels;
  EXPECT_TRUE(kmeans.Cluster(2, centroids, labels));

  // Check that there are two centroids.
  EXPECT_EQ(centroids.size(), 2u);

  // Check that the observations are clustered properly.
  EXPECT_EQ(labels[0], labels[1]);
  EXPECT_EQ(labels[1], labels[2]);
  EXPECT_EQ(labels[2], labels[3]);
  EXPECT_EQ(labels[3], labels[4]);

  EXPECT_NE(labels[4], labels[5]);

  EXPECT_EQ(labels[5], labels[6]);
  EXPECT_EQ(labels[6], labels[7]);
  EXPECT_EQ(labels[7], labels[8]);
  EXPECT_EQ(labels[8], labels[9]);

  // Check the centroids.
  math::Vector3 expectedCentroid1(1.2, 1.0, 0.0);
  math::Vector3 expectedCentroid2(5.2, 1.0, 0.0);
  if (centroids[0] == expectedCentroid1)
    EXPECT_EQ(centroids[1], expectedCentroid2);
  else if (centroids[0] == expectedCentroid2)
    EXPECT_EQ(centroids[1], expectedCentroid1);
  else
    FAIL();

  // Try to use an empty observation vector.
  obsCopy.clear();
  EXPECT_FALSE(kmeans.Observations(obsCopy));

  // Try to use a k > num_observations.
  EXPECT_FALSE(kmeans.Cluster(obs.size() + 1, centroids, labels));
}

//////////////////////////////////////////////////
TEST_F(KmeansTest, Append)
{
  // Create some observations.
  std::vector<math::Vector3> obs, obs2, obsTotal;
  obs.push_back(math::Vector3(1.0, 1.0, 0.0));
  obs.push_back(math::Vector3(1.1, 1.0, 0.0));
  obs.push_back(math::Vector3(1.2, 1.0, 0.0));
  obs.push_back(math::Vector3(1.3, 1.0, 0.0));
  obs.push_back(math::Vector3(1.4, 1.0, 0.0));

  obs2.push_back(math::Vector3(5.0, 1.0, 0.0));
  obs2.push_back(math::Vector3(5.1, 1.0, 0.0));
  obs2.push_back(math::Vector3(5.2, 1.0, 0.0));
  obs2.push_back(math::Vector3(5.3, 1.0, 0.0));
  obs2.push_back(math::Vector3(5.4, 1.0, 0.0));

  obsTotal.insert(obsTotal.end(), obs.begin(), obs.end());
  obsTotal.insert(obsTotal.end(), obs2.begin(), obs2.end());

  // Initialize Kmeans with two partitions.
  math::Kmeans kmeans(obs);

  kmeans.AppendObservations(obs2);

  std::vector<math::Vector3> obsCopy;
  obsCopy = kmeans.Observations();
  for (unsigned int i = 0; i < obsTotal.size(); ++i)
    EXPECT_EQ(obsTotal[i], obsCopy[i]);
}
