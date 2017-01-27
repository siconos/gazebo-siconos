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
#ifndef _GAZEBO_GUI_BUILDING_BUILDINGMAKER_HH_
#define _GAZEBO_GUI_BUILDING_BUILDINGMAKER_HH_

#include <string>
#include <vector>
#include <memory>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/common/MouseEvent.hh"

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingModelManip;
    class EditorItem;

    // Forward declare provate data.
    class BuildingMakerPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \brief Create and manage 3D visuals of a building.
    class GZ_GUI_VISIBLE BuildingMaker
    {
      /// \brief Constructor
      public: BuildingMaker();

      /// \brief Destructor
      public: ~BuildingMaker();

      /// \brief QT callback when entering or leaving building edit mode
      /// \param[in] _checked True if the menu item is checked
      public: void OnEdit(bool _checked);

      /// \brief Set the name of this building model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

      /// \brief Add a building part to the model.
      /// \param[in] _type Type of the building part.
      /// \param[in] _size Size of the building part.
      /// \param[in] _pos Position of the building part in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the building part in degrees.
      /// \return Name of the 3D building part that has been added.
      public: std::string AddPart(const std::string &_type,
          const QVector3D &_size, const QVector3D &_pos, double _angle);

      /// \brief Add a wall to the model.
      /// \param[in] _size Size of the wall.
      /// \param[in] _pos Position of the wall in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the wall in degrees.
      /// \return Name of the 3D wall that has been added.
      public: std::string AddWall(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a window to the model.
      /// \param[in] _size Size of the window.
      /// \param[in] _pos Position of the window in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the window in degrees.
      /// \return Name of the 3D window that has been added.
      public: std::string AddWindow(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Add a door to the model.
      /// \param[in] _size Size of the door.
      /// \param[in] _pos Position of the door in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the door in degrees.
      /// \return Name of the 3D door that has been added.
      public: std::string AddDoor(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a staircase to the model.
      /// \param[in] _size Size of the staircase.
      /// \param[in] _pos Position of the staircase in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the staircase in degrees.
      /// \param[in] _steps Number of steps in the staircase.
      /// \return Name of the 3D staircase that has been added.
      public: std::string AddStairs(const QVector3D &_size,
          const QVector3D &_pos, double _angle, int _steps);

      /// \brief Add a floor to the model.
      /// \param[in] _size Size of the floor.
      /// \param[in] _pos Position of the floor in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the floor in radians.
      /// \return Name of the 3D floor that has been added.
      public: std::string AddFloor(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Remove a building part from the model.
      /// \param[in] _partName Name of the building part to remove
      public: void RemovePart(const std::string &_partName);

      /// \brief Remove a wall from the model.
      /// \param[in] _partName Name of the wall to remove
      public: void RemoveWall(const std::string &_wallName);

      /// \brief Connect the 2D editor item Qt signals to the 3D building part.
      /// \param[in] _partName Name of the 3D building part
      /// \param[in] _item 2D editor item.
      public: void ConnectItem(const std::string &_partName,
          const EditorItem *_item);

      /// \brief Attach a building part to another, this is currently used for
      /// making holes in walls and floors.
      /// This function doesn't check if the parts exist.
      /// \param[in] _child Name of the child building part
      /// \param[in] _parent Name of the parent building part.
      public: void AttachManip(const std::string &_child,
          const std::string &_parent);

      /// \brief Detach a child building part from its parent.
      /// \param[in] _child Name of the child building part.
      public: void DetachFromParent(const std::string &_child);

      /// \brief Detach all child building parts from the given manip.
      /// \param[in] _parent Name of the building part.
      public: void DetachAllChildren(const std::string &_parent);

      /// \brief Whether the given manip is attached to another manip or not.
      /// \param[in] _child Name of manip.
      /// \return True if manip has a parent.
      public: bool IsAttached(const std::string &_child) const;

      /// \brief Detach all child building parts from the given manip.
      /// \param[in] _manip Name of the building part.
      public: BuildingModelManip *ManipByName(const std::string &_name);

      /// \brief Helper method to convert size from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _width Width in pixels.
      /// \param[in] _depth Depth in pixels.
      /// \param[in] _height Height in pixels.
      /// \return Size in metric units.
      public: static ignition::math::Vector3d ConvertSize(
          const double _width, const double _depth, const double _height);

      /// \brief Helper method to convert pose from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _x X position in pixels.
      /// \param[in] _y Y position in pixels.
      /// \param[in] _y Z position in pixels.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      /// \return Pose with position in metric units and rotation in radians.
      public: static ignition::math::Pose3d ConvertPose(const double _x,
          const double _y, const double _z, const double _roll,
          const double _pitch, const double _yaw);

      /// \param[in] _value Convert a value from pixels to metric units
      /// \param[in] _value Value in pixels.
      /// \return Value in metric units.
      public: static double Convert(double _value);

      /// \brief Convert an angle from editor unit to Gazebo unit
      /// \param[in] _angle Angle in degrees.
      /// \return Angle in radians.
      public: static double ConvertAngle(double _angle);

      /// \brief Reset the building maker and the SDF.
      public: void Reset();

      /// \brief Generate the SDF from building part visuals.
      public: void GenerateSDF();

      /// \brief Set save state upon a change to the building.
      public: void BuildingChanged();

      /// \brief Get the last generated SDF as string.
      /// \return String representation of the building's SDF.
      public: std::string ModelSDF() const;

      /// \brief Publish a factory message to spawn the new building.
      private: void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief Generate SDF with CSG support (to be supported).
      private: void GenerateSDFWithCSG();

      /// \brief Get a template SDF string of a simple model.
      /// \return A string containing a simple model.
      private: std::string TemplateSDFString() const;

      /// \brief Internal helper function for QPointF comparison used by the
      /// surface subsivision algorithm.
      private: static bool PointCompareY(const QPointF &_a, const QPointF &_b);

      /// \brief Internal helper function for QRectF comparison used by the
      /// surface subsivision algorithm.
      private: static bool RectCompareX(const QRectF &_a, const QRectF &_b);

      /// \brief Internal helper function for QRectF comparison used by the
      /// surface subsivision algorithm.
      private: static bool RectCompareY(const QRectF &_a, const QRectF &_b);

      /// \brief Subdivide a rectangular surface with holes into multiple
      /// smaller rectangles.
      /// \param[in] _surface Parent rectangular surface.
      /// \param[in] _holes A list of rectangular holes on the surface.
      /// \param[in] _subdivisions The resulting smaller rectangles representing
      /// the surface with holes.
      private: void SubdivideRectSurface(const QRectF &_surface,
        const std::vector<QRectF> &_holes, std::vector<QRectF> &_subdivisions);

      /// \brief Helper function to manage writing files to disk.
      private: void SaveModelFiles();

      /// \brief Callback for saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSave();

      /// \brief Callback for selecting a folder and saving the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSaveAs();

      /// \brief Callback for when the name is changed through the Palette.
      /// \param[in] _modelName The newly entered building name.
      private: void OnNameChanged(const std::string &_modelName);

      /// \brief Callback for newing the model.
      private: void OnNew();

      /// \brief Callback received when exiting the editor mode.
      private: void OnExit();

      /// \brief Callback received when a level on a building model is to
      /// be changed.
      /// \param[in] _level The level that is currently being edited.
      private: void OnChangeLevel(int _level);

      /// \brief Cancel material modes.
      private: void StopMaterialModes();

      /// \brief Reset currently hovered visual to the properties it had before
      /// being hovered.
      private: void ResetHoverVis();

      /// \brief Callback received when a color has been selected on the
      /// palette.
      /// \param[in] _color Selected color.
      private: void OnColorSelected(QColor _color);

      /// \brief Callback received when a texture has been selected on the
      /// palette.
      /// \param[in] _texture Selected texture.
      private: void OnTextureSelected(QString _texture);

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool On3dMouseMove(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is pressed.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool On3dMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is released.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool On3dMouseRelease(const common::MouseEvent &_event);

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool On3dKeyPress(const common::KeyEvent &_event);

      /// \brief Conversion scale used by the Convert helper functions.
      public: static const double conversionScale;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<BuildingMakerPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
