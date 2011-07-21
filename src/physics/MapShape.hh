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
/* Desc: Occupancy grid geom
 * Author: Nate Koenig
*/

#ifndef MAPSHAPE_HH
#define MAPSHAPE_HH

#include <deque>

#include "common/CommonTypes.hh"

#include "physics/Geom.hh"
#include "physics/Shape.hh"

namespace gazebo
{
	namespace physics
  {
    class SpaceTree;
    class QuadNode;

    /// \addtogroup gazebo_physics
    /// \{ 
    
    /// \brief Map geom
    class MapShape : public Shape
    {
      /// \brief Constructor
      public: MapShape(GeomPtr parent);
  
      /// \brief Destructor
      public: virtual ~MapShape();
  
      /// \brief Update function 
      public: void Update();
  
      /// \brief Load the map
      public: virtual void Load( sdf::ElementPtr _sdf );

      /// \brief Init the map
      public: virtual void Init();
  
      /// \brief Build the quadtree
      private: void BuildTree(QuadNode *node);
  
      /// \brief Get the number of free and occupied pixels in a given area
      private: void GetPixelCount(unsigned int xStart, unsigned int yStart, 
                                  unsigned int width, unsigned int height, 
                                  unsigned int &freePixels, 
                                  unsigned int &occPixels  );
  
      /// \brief Reduce the number of nodes in the tree. 
      private: void ReduceTree(QuadNode *node);
  
      /// \brief Try to merge to nodes
      private: void Merge(QuadNode *nodeA, QuadNode *nodeB);
  
      private: void CreateBox();
  
      /// \brief Create the boxes for the map
      private: void CreateBoxes(QuadNode *node);
  
      private: common::Image *mapImage;
  
      private: QuadNode *root;
  
      private: bool merged;
      private: static unsigned int geomCounter;
    };
  
 
    /// \cond 
    class QuadNode
    {
      public: QuadNode( QuadNode *_parent ) 
              {
                parent = _parent;
                occupied = false;
                leaf = true;
                valid = true;
              }
  
      public: ~QuadNode() 
              { 
                std::deque<QuadNode*>::iterator iter;
                for (iter = children.begin(); iter != children.end(); iter++) 
                    delete (*iter); 
              }
  
      public: void Print(std::string space)
              {
                std::deque<QuadNode*>::iterator iter;
  
                printf("%sXY[%d %d] WH[%d %d] O[%d] L[%d] V[%d]\n",space.c_str(),x,y,width, height, occupied, leaf, valid);
                space += "  ";
                for (iter = children.begin(); iter != children.end(); iter++) 
                  if ((*iter)->occupied)
                    (*iter)->Print(space);
              }
  
      public: unsigned int x, y;
      public: unsigned int width, height;
  
      public: QuadNode *parent;
      public: std::deque<QuadNode*> children;
      public: bool occupied;
      public: bool leaf;
  
      public: bool valid;
    };
    /// \endcond 

    /// \}
  }
}
#endif
