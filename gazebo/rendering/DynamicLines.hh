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
#ifndef GAZEBO_RENDERING_DYNAMICLINES_HH
#define GAZEBO_RENDERING_DYNAMICLINES_HH

#include <vector>
#include <string>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/DynamicRenderable.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class DynamicLines DynamicLines.hh rendering/rendering.hh
    /// \brief Class for drawing lines that can change
    class GZ_RENDERING_VISIBLE DynamicLines : public DynamicRenderable
    {
      /// \brief Constructor
      /// \param[in] _opType The type of Line
      public: explicit DynamicLines(
                  RenderOpType _opType = RENDERING_LINE_STRIP);

      /// \brief Destructor
      public: virtual ~DynamicLines();

      /// \brief Get type of movable
      /// \return This returns "gazebo::dynamiclines"
      public: static std::string GetMovableType();

      /// \brief Overridden function from Ogre's base class.
      /// \return Returns "gazebo::ogredynamicslines"
      public: virtual const Ogre::String &getMovableType() const;

      /// \brief Add a point to the point list
      /// \param[in] _pt ignition::math::Vector3d point
      /// \param[in] _color common::Color Point color
      public: void AddPoint(const ignition::math::Vector3d &_pt,
            const common::Color &_color = common::Color::White);

      /// \brief Add a point to the point list.
      /// \param[in] _x X position
      /// \param[in] _y Y position
      /// \param[in] _z Z position
      /// \param[in] _color common::Color Point color
      public: void AddPoint(double _x, double _y, double _z,
            const common::Color &_color = common::Color::White);

      /// \brief Change the location of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _value ignition::math::Vector3d value to set the point to
      public: void SetPoint(const unsigned int _index,
                  const ignition::math::Vector3d &_value);

      /// \brief Change the color of an existing point in the point list
      /// \param[in] _index Index of the point to set
      /// \param[in] _color common::Color Pixelcolor color to set the point to
      public: void SetColor(unsigned int _index, const common::Color &_color);

      /// \brief Return the location of an existing point in the point list
      /// \param[in] _index Number of the point to return
      /// \return ignition::math::Vector3d value of the point. A vector of
      /// [IGN_DBL_INF, IGN_DBL_INF, IGN_DBL_INF] is returned when then the
      /// _index is out of bounds.
      /// IGN_DBL_INF==std::numeric_limits<double>::infinity()
      public: ignition::math::Vector3d Point(const unsigned int _index) const;

      /// \brief Return the total number of points in the point list
      /// \return Number of points
      public: unsigned int GetPointCount() const;

      /// \brief Remove all points from the point list
      public: void Clear();

      /// \brief Call this to update the hardware buffer after making changes.
      public: void Update();

      /// \brief Implementation DynamicRenderable,
      /// creates a simple vertex-only decl
      private: virtual void  CreateVertexDeclaration();

      /// \brief Implementation DynamicRenderable, pushes point
      /// list out to hardware memory
      private: virtual void FillHardwareBuffers();

      /// \brief List of points for the line
      private: std::vector<ignition::math::Vector3d> points;

      /// \brief Used to indicate if the lines require an update
      private: bool dirty;

      /// \brief List of colors
      private: std::vector<common::Color> colors;
    };
    /// \}
  }
}
#endif
