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
#ifndef _GZ_GUI_PLUGIN_HH_
#define _GZ_GUI_PLUGIN_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  /// \brief A plugin loaded within the gzclient on startup.
  class GZ_GUI_VISIBLE GUIPlugin : public QWidget, public PluginT<GUIPlugin>
  {
    public: GUIPlugin() : QWidget(NULL)
            {this->type = GUI_PLUGIN;}

    /// \brief Load function
    ///
    /// Called when a plugin is first created.
    /// This function should not be blocking. This function will be
    /// called with an empty sdf element when a GUI plugin is loaded
    /// via a gui.ini file or via a command line argument.
    /// \param[in] _sdf Pointer the the SDF element of the plugin. This is
    /// the plugin SDF, <plugin ...>, and its children. It will be an empty
    /// element when loaded from INI file or command line argument.
    public: virtual void Load(sdf::ElementPtr /*_sdf*/) {}

    // \brief must be defined to support style sheets
    public: virtual void paintEvent(QPaintEvent *)
    {
      QStyleOption opt;
      opt.init(this);
      QPainter p(this);
      style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
    }
  };

/// \brief Plugin registration function for gui plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_GUI_PLUGIN(classname) \
  extern "C" GZ_PLUGIN_VISIBLE gazebo::GUIPlugin *RegisterPlugin(); \
  gazebo::GUIPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }
}

#endif
