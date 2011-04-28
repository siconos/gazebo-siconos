/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Trimesh geometry
 * Author: Nate Koenig
 * Date: 16 Oct 2009
 * SVN: $Id$
 */

#ifndef ODETRIMESHSHAPE_HH
#define ODETRIMESHSHAPE_HH

#include "physics/TrimeshShape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief Triangle mesh geom
    class ODETrimeshShape : public TrimeshShape
    {
      /// \brief Constructor
      public: ODETrimeshShape(GeomPtr parent);
  
      /// \brief Destructor
      public: virtual ~ODETrimeshShape();
  
      /// \brief Update function 
      public: void Update();
  
      /// \brief Load the trimesh
      protected: virtual void Load(common::XMLConfigNode *node);
  
      protected: virtual void Init();

      private: dReal matrix_dblbuff[16*2];
      private: int last_matrix_index;
    };
  }
}
#endif
