/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GAZEBO_SVGLOADER_PRIVATE_HH_
#define GAZEBO_SVGLOADER_PRIVATE_HH_

namespace gazebo
{
  namespace common
  {
    class SVGLoaderPrivate
    {
      /// \brief The step distance between 2 sampled points in a bezier curve
      /// It is the inverse of the number of samples in the spline, and should
      /// be between 0 and 1
      public: double resolution;
    };
  }
}

#endif
