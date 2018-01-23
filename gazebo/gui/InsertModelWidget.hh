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
#ifndef GAZEBO_GUI_INSERTMODELWIDGET_HH_
#define GAZEBO_GUI_INSERTMODELWIDGET_HH_


#include <string>
#include <map>
#include <vector>
#include <boost/filesystem.hpp>

#include "gazebo/common/Event.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;

namespace gazebo
{
  namespace common
  {
    // Forward declaration.
    class FuelModelDatabase;
  }
  namespace gui
  {
    // Forward declaration.
    class InsertModelWidgetPrivate;

    class GZ_GUI_VISIBLE InsertModelWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: explicit InsertModelWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~InsertModelWidget();

      /// \brief Check if input path is in the fileTreeWidget.
      public: bool LocalPathInFileWidget(const std::string &_path);

      /// \brief Callback triggered when the ModelDatabase has returned
      /// the list of models.
      /// \param[in] _models The map of all models in the database.
      private: void OnModels(
                   const std::map<std::string, std::string> &_models);

      /// \brief Callback triggered when a request to update the model database
      /// is received.
      /// \param[i] _localPath The model path that was updated.
      private: void OnModelUpdateRequest(const std::string &_localPath);

      /// \brief A signal to trigger the model population of an Ignition Fuel
      /// server.
      /// \param[in] server The name of the server containing the models.
      signals: void UpdateFuel(const std::string &_server);

      /// \brief Received model selection user input
      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);

      /// \brief An update function that lets this widget add in the results
      /// from ModelDatabase::GetModels.
      private slots: void Update();

      /// \brief An update function that lets this widget add in the results
      /// from ModelDatabase::GetModels.
      /// \param[in] _server The name of the server containing the models.
      private slots: void OnUpdateFuel(const std::string &_server);

      /// \brief QT callback when a path is changed.
      /// \param[in] _path The path that was changed.
      private slots: void OnDirectoryChanged(const QString &_path);

      /// \brief QT callback when addPathButton is clicked.
      private slots: void HandleButton();

      /// \brief check if path exists with special care to filesystem
      /// permissions
      /// \param[in] _path The path to check.
      private: static bool IsPathAccessible
        (const boost::filesystem::path &_path);

      /// \brief Update the list of models on the local system.
      private: void UpdateAllLocalPaths();

      /// \brief Update a specific path.
      /// \param[in] _path The path to update.
      private: void UpdateLocalPath(const std::string &_path);

      /// \brief Populate the model tree widget with the list of all available
      /// Ignition Fuel servers providing models.
      private: void InitializeFuelServers();

      /// \brief Populate each Ignition Fuel server in the tree widget with its
      /// list of models.
      private: void PopulateFuelServers();

      /// \brief Vector to store event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Private data pointer.
      private: InsertModelWidgetPrivate *dataPtr;
    };
  }
}
#endif
