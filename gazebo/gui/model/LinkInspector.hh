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
#ifndef GAZEBO_GUI_MODEL_LINKINSPECTOR_HH_
#define GAZEBO_GUI_MODEL_LINKINSPECTOR_HH_

#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class CollisionConfig;
    class LinkConfig;
    class VisualConfig;

    // Forward declare private data.
    class LinkInspectorPrivate;

    class GZ_GUI_VISIBLE LinkInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: explicit LinkInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~LinkInspector();

      /// \brief Set the name of the link.
      /// \param[in] Name to set the link to.
      public: void SetName(const std::string &_name);

      /// \brief Get the name of the link.
      /// \return Name of the link.
      public: std::string Name() const;

      /// \brief Get configurations of the link.
      /// \return Tab widget with link configurations.
      public: LinkConfig *GetLinkConfig() const;

      /// \brief Get visual configurations of the link.
      /// \return Tab widget with visual configurations.
      public: VisualConfig *GetVisualConfig() const;

      /// \brief Get collision configurations of the link.
      /// \return Tab widget with visual configurations.
      public: CollisionConfig *GetCollisionConfig() const;

      /// \brief Set the id for this link
      /// \param[in] New link id.
      public: void SetLinkId(const std::string &_id);

      /// \brief Open the inspector.
      public: void Open();

      /// \brief Set the state of show collisions button.
      /// \param[in] _show If true, button is checked.
      public: void SetShowCollisions(const bool _show);

      /// \brief Set the state of show visuals button.
      /// \param[in] _show If true, button is checked.
      public: void SetShowVisuals(const bool _show);

      /// \brief Set the state of show link frame button.
      /// \param[in] _show If true, button is checked.
      public: void SetShowLinkFrame(const bool _show);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Computes volume of associated link.
      /// \return The volume.
      private: double ComputeVolume() const;

      /// \brief Computes mass moment of inertia of associated link.
      /// \param[in] _mass Mass of the link.
      /// \return Vector containing principal moments of inertia.
      private: ignition::math::Vector3d ComputeInertia(
          const double _mass) const;

      /// \brief Set the item name.
      /// \param[in] _name Name to set to.
      // public: void SetName(const std::string &_name);

      /// \brief Qt signal emitted to indicate that the inspector was opened.
      Q_SIGNALS: void Opened();

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt signal emitted to indicate that changes should be applied
      /// and the inspector closed.
      Q_SIGNALS: void Accepted();

      /// \brief Qt signal emitted to indicate that all collisions should be
      /// shown/hidden.
      /// \param[in] _show True to show.
      Q_SIGNALS: void ShowCollisions(const bool _show);

      /// \brief Qt signal emitted to indicate that all visuals should be
      /// shown/hidden.
      /// \param[in] _show True to show.
      Q_SIGNALS: void ShowVisuals(const bool _show);

      /// \brief Qt signal emitted to indicate that link frame should be
      /// shown/hidden.
      /// \param[in] _show True to show.
      Q_SIGNALS: void ShowLinkFrame(const bool _show);

      /// \brief Qt callback when the Remove button is pressed.
      private slots: void OnRemove();

      /// \brief Qt callback when the show collisions button is pressed.
      /// \param[in] _show Show if checked, hide otherwise.
      private slots: void OnShowCollisions(const bool _show);

      /// \brief Qt callback when the show visuals button is pressed.
      /// \param[in] _show Show if checked, hide otherwise.
      private slots: void OnShowVisuals(const bool _show);

      /// \brief Qt callback when the show link frame button is pressed.
      /// \param[in] _show Show if checked, hide otherwise.
      private slots: void OnShowLinkFrame(const bool _show);

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Qt callback when one of the child configs has been applied.
      private slots: void OnConfigApplied();

      /// \brief Callback for density changes in link config.
      /// \param[in] _value The new density value.
      private slots: void OnDensityValueChanged(const double _value);

      /// \brief Callback for mass changes in link config.
      /// \param[in] _value The new mass value.
      private slots: void OnMassValueChanged(const double _value);

      /// \brief Callback for changes to collisions.
      /// \param[in] _name Name of the collision.
      /// \param[in] _type Type of change (eg, "geometry", etc).
      private slots: void OnCollisionChanged(const std::string &_name,
          const std::string &_type);

      /// \brief Restore the widget's data to how it was when first opened.
      private slots: void RestoreOriginalData();

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event.
      private: void keyPressEvent(QKeyEvent *_event);

      /// \brief Qt close event
      /// \param[in] _event Qt close event pointer
      private: void closeEvent(QCloseEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LinkInspectorPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
